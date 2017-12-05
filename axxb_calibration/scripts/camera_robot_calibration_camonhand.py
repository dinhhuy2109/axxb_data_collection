#!/usr/bin/env python
import os
import copy
import yaml
import rospy
import rospkg
import numpy as np
import openravepy as orpy
import pickle
# ROS
import criros
import tf.transformations as tr
import dynamic_reconfigure.client
from criros.utils import read_parameter, read_parameter_err
# Progress bar
import progressbar
# Convex hull
from scipy.spatial import ConvexHull, Delaunay
# Image geometry
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
# Planning and control
from denso_openrave.motion import RoboMotion
from criros.raveutils import iktype5D, iktype6D
from denso_control.controllers import JointTrajectoryController
# Services and msgs
from ensenso.srv import CalibrateHandEye, CollectPattern,EstimatePatternPose
from geometry_msgs.msg import PoseArray


class RobotCameraCalibration(object):
  """
  The C{RobotCameraCalibration} performs the camera <-> robot calibration.
  Everything is done in the constructor.
  ROS Parameters:
    - C{debug}: If set, will show additional debugging messages.
    - C{robot_name}: Used to locate the robot topics. Typically C{left}, C{right} or C{denso}
    - C{grid_spacing}: The grid_spacing of the calibration pattern.
    - C{maxdist}: The maximum distance from the camera where we will look for IK solutions (meters)
    - C{maxtilt}: The maximum tilting angle to apply to the pattern (degrees)
    - C{samples}: Number of pattern observations to be collected
    - C{pausetime}: Time to wait for the camera to stabilize. Typically, between 1-2 seconds.
    - C{use_weights}: If set, will prefer bigger displacements in the joints near the end-effector
  """
  def __init__(self, logger=rospy):
    """
    C{RobotCameraCalibration} constructor. It subscribes to the C{camera/left/camera_info} topic
    to get the parameters to be used during the estimation of the camera FOV
    @type  logger: Object
    @param logger: Logger instance. When used in ROS, the recommended C{logger=rospy}.
    """
    # Generic
    np.set_printoptions(precision=5, suppress=True)
    axes = []
    # Get optional parameters
    debug = read_parameter('~debug', False)
    robot_name = read_parameter('~robot_name', False)
    grid_spacing = read_parameter('~grid_spacing', 15.94)
    maxdist = read_parameter('~maxdist', 1.2)
    mindist = read_parameter('~mindist', 0.5)
    maxtilt = np.deg2rad( read_parameter('~maxtilt', 30) )
    samples = int( read_parameter('~samples', 50) )
    pausetime = read_parameter('~pausetime', 1.0)
    use_weights = read_parameter('~use_weights', True)
    decode = grid_spacing <= 0
    # Load OpenRAVE environment
    env = orpy.Environment()
    orpy.RaveSetDebugLevel(orpy.DebugLevel.Fatal)
    worldxml = 'worlds/welding.env.xml' 
    if not env.Load(worldxml):
      raise Exception('World file does not exist: %s' % worldxml)
    robot = env.GetRobot(robot_name)
    if robot is None:
      raise Exception('Could not find robot: {0}'.format(robot_name))
    try:
      manip = robot.SetActiveManipulator('camera') 
    except:
      raise Exception('Could not find manipulator: camera')
    # Remove unnecessary objects from the world
    # criros.raveutils.remove_bodies(env, remove=['short_ws', 'back', 'long_ws',
    #                                                             'right_frame'])
    # Configure motion planning
    logger.loginfo('Loading motion planning and control pipeline')
    motion = RoboMotion(manip, checker='ode')
    motion.change_active_manipulator('camera', iktype=iktype6D)
    motion.scale_velocity_limits(0.1)
    motion.scale_acceleration_limits(0.1)
    motion.set_smoother_iterations(100)
    motion.set_planning_iterations(80)
    motion.enable_collision_checking(True)
    motion.update_link_stats()
    # Add the calibration plate to the world
    Tplate = tr.euler_matrix(0, -np.pi/2.5,0)
    Tplate[:3,3] = [1.3, 0, 0.35]
    # init guess of the plate pose as Transformation. Relative to the
    # robot origin.
    plate = env.ReadKinBodyXMLFile('objects/calibration_plate.kinbody.xml')
    if plate is None:
      raise Exception('Could not find: objects/calibration_plate.kinbody.xml')
    with env:
      env.AddKinBody(plate)
      plate.SetName('plate')
      plate.SetTransform(Tplate)
    # Move the origin to the robot base link
    criros.raveutils.move_origin_to_body(motion.robot)
    # Camera and pattern seeds
    Tcamera = np.eye(4)
    # Initial guess of the camera pose as Transformation. The pose must be given
    # relative to the robot hand (for a moving camera setup), or relative to the
    # robot origin (for the fixed camera setup).
    camera_seed = criros.conversions.to_pose(Tcamera)
    # Initial guess of the pattern pose as Transformation. This pose must be
    # given relative to the robot hand.
    pattern_seed = criros.conversions.to_pose(Tplate)
    if debug:
      motion.draw_axes(Tplate)
    # Trajectory controller
    controller = JointTrajectoryController(namespace=robot_name)
    # Camera configuration client and services
    logger.loginfo('Connecting to the ensenso_driver')
    dynclient = dynamic_reconfigure.client.Client('/camera/ensenso_driver',
                                                                    timeout=30)
    dynclient.update_configuration({'Cloud':False, 'Images':True,
                  'Projector':False, 'FrontLight':True, 'TargetBrightness':60})
    logger.loginfo('Waiting for ensenso camera services')
    calibrate_srv = rospy.ServiceProxy('camera/calibrate_handeye',
                                                              CalibrateHandEye)
    calibrate_srv.wait_for_service()
    collect_pattern_srv = rospy.ServiceProxy('camera/collect_pattern', CollectPattern)
    collect_pattern_srv.wait_for_service()
    pattern_srv = rospy.ServiceProxy('camera/estimate_pattern_pose', EstimatePatternPose)
    pattern_srv.wait_for_service()
    # Get left camera info
    self.left_cam_info = None
    sub = rospy.Subscriber('camera/left/camera_info', CameraInfo,
                                                self.cb_left_info, queue_size=1)
    while not rospy.is_shutdown() and self.left_cam_info is None:
      rospy.sleep(0.1)
    camera_info = copy.deepcopy(self.left_cam_info)
    sub.unregister()
    # OpenRAVE viewer
    if debug:
      env.SetViewer('QtCoin')
    # Camera convex hull and triangulation
    points = criros.exploration.camera_fov_corners(camera_info, zdist=maxdist,
                                                                Tcamera=Tplate)
    hull = ConvexHull(points)
    triangulation = Delaunay(points[hull.vertices])
    fov_mesh = criros.raveutils.trimesh_from_point_cloud(points[hull.vertices])
    fov_body = orpy.RaveCreateKinBody(env, '')
    fov_body.InitFromTrimesh(fov_mesh)
    fov_body.SetName('camera_fov')
    env.Add(fov_body)
    criros.raveutils.set_body_transparency(fov_body, 0.75)
    criros.raveutils.enable_body(fov_body, False)
    Tideal = np.eye(4)
    Tideal[:3,0] = Tplate[:3,1]
    Tideal[:3,1] = Tplate[:3,0]
    Tideal[:3,2] = -Tplate[:3,2]
    # Generate random poses, plan, move and collect pattern
    starttime = rospy.Time.now()
    logger.loginfo('Starting to collect pattern observations')
    robot_poses = PoseArray()
    pattern_poses = PoseArray()
    pbar = progressbar.ProgressBar(widgets=['Pattern observations: ',
                          progressbar.SimpleProgress()], maxval=samples).start()
    dynclient.update_configuration({'Images':False})
    while len(robot_poses.poses) < samples:
      if rospy.is_shutdown():
        return
      # Random pose for the calibration pattern
      rpy = maxtilt * (2*np.random.random_sample(3) - 1.)
      Tlook = np.dot(Tideal, tr.euler_matrix(*rpy))
      Tlook[:3,3] = criros.exploration.random_point_inside_fov(camera_info,
                                                               maxdist=maxdist, Tcamera=Tplate)
      # Check that the camera  will be inside the fov hull.
      corners = criros.raveutils.compute_bounding_box_corners(plate, Tbody=Tlook)
      if not np.alltrue( triangulation.find_simplex(corners)>=0 ):
        continue
      # Check we are at least mindist away from the camera
      distance = np.linalg.norm(Tplate[:3,3] - Tlook[:3,3])
      if distance < mindist:
        continue
      # Corresponding transform for the gripper
      Tcam = Tlook
      qstart = controller.get_joint_positions()
      qgoal = motion.find_closest_iksolution(Tcam, qseed=qstart)
      if qgoal is None:
        continue
      # Double check for collisions
      with robot:
        robot.SetActiveDOFValues(qgoal)
        if env.CheckCollision(robot) or env.CheckCollision(plate):
          continue
      # Plan trajectory
      traj = motion.generate_trajectory(qstart, qgoal)
      if traj is None:
        continue
      if debug:
        motion.draw_axes(Tlook)
      # Move the robot
      ros_traj = criros.conversions.ros_trajectory_from_openrave(robot_name, traj)
      controller.set_trajectory(ros_traj)
      controller.start()
      motion.robot.GetController().SetPath(traj)
      controller.wait()
      # Start images streaming and collect pattern
      dynclient.update_configuration({'Images':True})
      rospy.sleep(pausetime)   # Wait for the camera to stabilize
      # Collect the pattern and robot poses
      try:
        res1 = collect_pattern_srv.call(add_to_buffer=True, clear_buffer=True,
                                      decode=decode, grid_spacing=grid_spacing)
        res2 = pattern_srv.call(average=False)
      except rospy.ServiceException as e:
        logger.logwarn('collect_pattern service failed: ' + str(e))
        continue
      finally:
        dynclient.update_configuration({'Images':False})
      if res1.success and res2.success:
        pattern_poses.poses.append(res2.pose)
        qrobot = controller.get_joint_positions()
        with env:
          motion.robot.SetActiveDOFValues(qrobot)
        Tcamera = motion.get_transform(qrobot)
        robot_poses.poses.append( criros.conversions.to_pose(Tcamera) )
        pbar.update(len(robot_poses.poses))
    pbar.finish()
    duration = (rospy.Time.now() - starttime).to_sec()
    logger.loginfo('Finished collecting observations in %.2f seconds.' % duration)
    if not (len(pattern_poses.poses) == len(robot_poses.poses)):
      num_buffer = len(pattern_poses.poses)
      num_robot = len(robot_poses.poses)
      logger.logwarn( 'Pattern count mismatch. Buffer: %d, Robot: %d' %
                                                        (num_buffer, num_robot))
      return
    # Save files
    pattern_tfs = [criros.conversions.from_pose(pattern_pose) for pattern_pose in pattern_poses.poses]
    robot_tfs   = [criros.conversions.from_pose(robot_pose) for robot_pose in robot_poses.poses] 
    rospack = rospkg.RosPack()
    filepath =  rospack.get_path('axxb_calibration') 
    filename1 = filepath +"/config/pattern_tfs.p"
    pickle.dump(pattern_tfs, open(filename1, "wb" ) )
    filename2 = filepath+"/config/robot_tfs.p"
    pickle.dump(robot_tfs, open(filename2, "wb" ) )
    rospy.loginfo('Result written to file: %s \n %s' %(filename1,filename2))
    
    # # The ensenso SKD will return the transformation of the robot's origin wrt.
    # # the camera frame
    # try:
    #   res = calibrate_srv.call( robot_poses=robot_poses, camera_seed=camera_seed,
    #                             pattern_seed=pattern_seed, setup='Fixed' )
    # except rospy.ServiceException as e:
    #   logger.logwarn('calibrate_handeye service failed: ' + str(e))
    # if not res.success:
    #   return
    # # Store calibration results
    # Tcamera_seed = criros.conversions.from_pose(camera_seed)
    # seed_rot = tr.quaternion_from_matrix(Tcamera_seed)
    # seed_pos = Tcamera_seed[:3,3]
    # # Estimated pattern calibration pose. pattern wrt end-effector
    # Tpattern_wrt_gripper = criros.conversions.from_pose(
    #                                                 res.estimated_pattern_pose)
    # pattern_rot = tr.quaternion_from_matrix(Tpattern_wrt_gripper)
    # pattern_pos = Tpattern_wrt_gripper[:3,3]
    # # Estimated transformation. World wrt camera
    # Tworld_wrt_camera = criros.conversions.from_pose(res.estimated_camera_pose)
    # rotation = tr.quaternion_from_matrix(Tworld_wrt_camera)
    # translation = Tworld_wrt_camera[:3,3]
    # angle, axis, point = tr.rotation_from_matrix( Tworld_wrt_camera )
    # yamldata = {'camera_%s_robot' % robot_name: {
    #               'parent': 'camera_optical_frame',
    #               'child': '{0}/world'.format(robot_name),
    #               'angle_axis': [angle] + axis.flatten().tolist(),
    #               'duration': duration,
    #               'grid_spacing': grid_spacing,
    #               'iterations': res.iterations,
    #               'samples': samples,
    #               'rotation': rotation.flatten().tolist(),
    #               'translation': translation.flatten().tolist(),
    #               'pattern_pose': {
    #                 'rotation': pattern_rot.flatten().tolist(),
    #                 'translation': pattern_pos.flatten().tolist()},
    #               'seed': {
    #                 'rotation': seed_rot.flatten().tolist(),
    #                 'translation': seed_pos.flatten().tolist()}},
    #             'reprojection_error': {
    #               'pixels': res.reprojection_error}}
    # # Write the file
    # rospack = rospkg.RosPack()
    # yaml_path  = rospack.get_path('ikea_calibration')
    # yaml_path += '/config/camera_%s_robot_calibration.yaml' % robot_name
    # # Write YAML file
    # scriptname = os.path.basename(__file__)
    # header =  '# This document was autogenerated by the script %s\n' % scriptname
    # header += '# EDITING THIS FILE BY HAND IS NOT RECOMMENDED\n'
    # header += '# NOTE 1: This is the transformation between the camera '
    # header += 'coordinates and the robot reference system.\n'
    # header += '# NOTE 2: The pattern pose is the transformation of the '
    # header += 'calibration pattern wrt to the end-effector.\n'
    # header += '# More information: http://www.ensenso.com/manual\n'
    # header += '# seed: Camera transformation in OpenRAVE \n\n'
    # with open(yaml_path, 'w') as f:
    #   f.write(header)
    #   yaml.safe_dump(yamldata, f)
    # rospy.loginfo('Result written to file: %s' % yaml_path)
    raw_input('DONE')
    
  def cb_left_info(self, msg):
    self.left_cam_info = msg

if __name__ == "__main__":
  # Initialize the node
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  calibration = RobotCameraCalibration()
  rospy.loginfo('Shuting down [%s] node' % node_name)
