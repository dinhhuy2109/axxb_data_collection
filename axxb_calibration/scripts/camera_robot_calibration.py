#!/usr/bin/env python
import os
import copy
import yaml
import rospy
import rospkg
import numpy as np
import openravepy as orpy
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
from ensenso.srv import CalibrateHandEye, EstimatePatternPose
from geometry_msgs.msg import PoseArray


class RobotCameraCalibration(object):
  def __init__(self, logger=rospy):
    # Generic
    np.set_printoptions(precision=5, suppress=True)
    axes = []
    # Get required parameters
    has_serial, serial = read_parameter_err('/camera/serial')
    if not has_serial:
      raise Exception('Could not find all the required parameters')
    # Get optional parameters
    debug = read_parameter('~debug', False)
    robot_name = read_parameter('~robot_name', False)
    grid_spacing = read_parameter('~grid_spacing', 15.94)
    maxdist = read_parameter('~maxdist', 1.2)
    maxtilt = np.deg2rad( read_parameter('~maxtilt', 30) )
    samples = int( read_parameter('~samples', 50) )
    pausetime = read_parameter('~pausetime', 1.0)
    use_weights = read_parameter('~use_weights', True)
    decode = grid_spacing <= 0
    # Load OpenRAVE environment
    env = orpy.Environment()
    orpy.RaveSetDebugLevel(orpy.DebugLevel.Fatal)
    worldxml = 'worlds/bimanual_ikea_assembly.env.xml'
    if not env.Load(worldxml):
      raise Exception('World file does not exist: %s' % worldxml)
    robot = env.GetRobot('denso_{0}'.format(robot_name))
    if robot is None:
      raise Exception('Could not find robot: denso_{0}'.format(robot_name))
    try:
      manip = robot.SetActiveManipulator('denso_ft_sensor_gripper')
    except:
      raise Exception('Could not find manipulator: denso_ft_sensor_gripper')
    # Get camera transform
    table = env.GetKinBody('table')
    if table is None:
      raise Exception('Could not find table in the world')
    camera = env.GetKinBody('camera')
    if table is None:
      raise Exception('Could not find camera in the world')
    # Remove unnecessary objects from the world
    criros.raveutils.remove_bodies(env, remove=['ikea_stick', 'pin', 'reference_frame'])
    # Configure motion planning
    logger.loginfo('Loading motion planning and control pipeline')
    motion = RoboMotion(manip)
    motion.change_active_manipulator('denso_ft_sensor_gripper', iktype=iktype6D)
    motion.scale_velocity_limits(0.3)
    motion.scale_acceleration_limits(0.2)
    motion.set_smoother_iterations(100)
    motion.set_planning_iterations(50)
    motion.enable_collision_checking(True)
    motion.update_link_stats()
    # Add the calibration plate to the world
    Tplate_wrt_gripper = tr.euler_matrix(0, -np.pi/2, np.pi)
    Tplate_wrt_gripper[0,3] = 0.0115 + 0.03
    Tplate_wrt_gripper[2,3] = 0.005
    Tgripper = motion.get_transform()
    Tplate = np.dot(Tgripper, Tplate_wrt_gripper)
    plate = env.ReadKinBodyXMLFile('objects/calibration_plate.kinbody.xml')
    if plate is None:
      raise Exception('Could not find calibration plate: objects/calibration_plate.kinbody.xml')
    with env:
      env.AddKinBody(plate)
      plate.SetName('plate')
      plate.SetTransform(Tplate)
    # Grasp the calibration plate in OpenRAVE
    motion.taskmanip.CloseFingers()
    motion.robot.Grab(plate)
    motion.taskmanip.ReleaseFingers()
    motion.taskmanip.CloseFingers()
    # Add extra protection to avoid hitting the table
    table_aabb = table.GetLink('base').ComputeAABB()
    boxdef = np.zeros(6)
    boxdef[:3] = table_aabb.pos() + np.array([0, 0, table_aabb.extents()[2]+0.05])
    boxdef[3:5] = table_aabb.extents()[:2] + 0.05
    boxdef[5] = 0.01
    boxdef.shape = (1,6)
    with env:
      paranoia = orpy.RaveCreateKinBody(env, '')
      paranoia.SetName('paranoia_cover')
      paranoia.InitFromBoxes(boxdef, True)
      env.Add(paranoia)
    # Move the origin to the robot base link
    criros.raveutils.move_origin_to_body(motion.robot)
    # Camera and pattern seeds
    Tcamera = camera.GetTransform()
    # Initial guess of the camera pose as Transformation. The pose must be given relative to 
    # the robot hand (for a moving camera setup), or relative to the robot origin (for the fixed camera setup).
    camera_seed = criros.conversions.to_pose(Tcamera)
    # Initial guess of the pattern pose as Transformation. This pose must be given relative to
    # the robot hand (for a fixed camera setup), or relative to the robot origin (for the moving camera setup).
    pattern_seed = criros.conversions.to_pose(Tplate_wrt_gripper)
    if debug:
      motion.draw_axes(Tcamera)
    # Trajectory controller
    controller = JointTrajectoryController(namespace=robot_name, log_level=rospy.ERROR)
    # Camera configuration client and services
    logger.loginfo('Connecting to the ensenso_driver')
    dynclient = dynamic_reconfigure.client.Client('/camera/ensenso_driver', timeout=30)
    dynclient.update_configuration({'Cloud':False, 'Images':True, 'Projector':False, 'FrontLight':False, 'TargetBrightness':70})
    logger.loginfo('Waiting for ensenso camera services')
    calibrate_srv = rospy.ServiceProxy('camera/calibrate_handeye', CalibrateHandEye)
    calibrate_srv.wait_for_service()
    pattern_srv = rospy.ServiceProxy('camera/estimate_pattern_pose', EstimatePatternPose)
    pattern_srv.wait_for_service()
    # Get left camera info
    self.left_cam_info = None
    sub = rospy.Subscriber('camera/left/camera_info', CameraInfo, self.cb_left_info)
    while not rospy.is_shutdown() and self.left_cam_info is None:
      rospy.sleep(0.1)
    camera_info = copy.deepcopy(self.left_cam_info)
    sub.unregister()
    # OpenRAVE viewer
    if debug:
      env.SetViewer('QtCoin')
      T = tr.euler_matrix(-2, 0, np.pi/2.)
      T[:3,3] = [2.4, 0, 1.57]
      T[1,3] = Tcamera[1,3]
      env.GetViewer().SetCamera(T)
    # Camera convex hull and triangulation
    points = criros.exploration.camera_fov_corners(camera_info, zdist=maxdist, Tcamera=Tcamera)
    hull = ConvexHull(points)
    triangulation = Delaunay(points[hull.vertices])
    Tideal = np.eye(4)
    Tideal[:3,0] = Tcamera[:3,1]
    Tideal[:3,1] = Tcamera[:3,0]
    Tideal[:3,2] = -Tcamera[:3,2]
    # TODO: Add a mesh showing the camera FOV
    # Generate random poses, plan, move and collect pattern
    starttime = rospy.Time.now()
    logger.loginfo('Starting to collect pattern observations')
    robot_poses = PoseArray()
    pattern_poses = PoseArray()
    pbar = progressbar.ProgressBar(widgets=['Pattern observations: ', progressbar.SimpleProgress()], maxval=samples).start()
    while not rospy.is_shutdown() and (len(robot_poses.poses) < samples):
      dynclient.update_configuration({'Images':False})
      # Random pose for the calibration pattern
      rpy = maxtilt * (2*np.random.random_sample(3) - 1.)
      Tlook = np.dot(Tideal, tr.euler_matrix(*rpy))
      Tlook[:3,3] = criros.exploration.random_point_inside_fov(camera_info, maxdist=maxdist, Tcamera=Tcamera)
      # Check that the plate will be inside the fov hull
      corners = criros.raveutils.compute_bounding_box_corners(plate, Tbody=Tlook)
      if not np.alltrue( triangulation.find_simplex(corners)>=0 ):
        continue
      # Corresponding transform for the gripper
      Tgripper = np.dot( Tlook, criros.spalg.transform_inv(Tplate_wrt_gripper) )
      qstart = controller.get_joint_positions()
      qgoal = motion.find_closest_iksolution(Tgripper, qseed=qstart)
      if qgoal is None:
        continue
      # Plan trajectory
      traj = motion.generate_trajectory(qstart, qgoal)
      if traj is None:
        continue
      if debug:
        motion.draw_axes(Tlook)
      # Move the robot
      ros_traj = motion.openrave_to_ros_traj(traj, rate=125.)
      controller.set_trajectory(ros_traj)
      controller.start()
      motion.robot.GetController().SetPath(traj)
      controller.wait()
      # Start images streaming and collect pattern
      dynclient.update_configuration({'Images':True})
      rospy.sleep(pausetime)   # Wait for the camera to stabilize
      # Collect the pattern and robot poses
      try:
        # We discard previous patterns only for the 1st observation. This way we populate the buffer with the observations collected
        # by this script
        discard = len(robot_poses.poses) == 0
        res = pattern_srv.call(add_to_buffer=True, discard_patterns=discard, average=False, decode=decode, grid_spacing=grid_spacing)
      except rospy.ServiceException as e:
        logger.logwarn('estimate_pattern_pose service failed: ' + str(e))
        continue
      if res.success:
        pattern_poses.poses.append(res.pose)
        qrobot = controller.get_joint_positions()
        motion.robot.SetActiveDOFValues(qrobot)
        Tgripper = motion.get_transform()
        robot_poses.poses.append( criros.conversions.to_pose(Tgripper) )
        pbar.update(len(robot_poses.poses))
    pbar.finish()
    duration = (rospy.Time.now() - starttime).to_sec()
    logger.loginfo('Finished collecting observations in %.2f seconds.' % duration)
    if not ( res.pattern_count == len(robot_poses.poses) == len(pattern_poses.poses) ):
      logger.logwarn('Something went wrong. The collected poses dont match the buffer patterns' % duration)
      return
    # Save files
    rospack = rospkg.RosPack()
    filepath =  rospack.get_path('axxb_data_collection') 
    filename1 = filepath +"/config/pattern_poses"
    pickle.dump(pattern_poses, open(filename1, "wb" ) )
    filename2 = filepath+"/config/robot_poses" + str(read_parameter('~axis',0))
    pickle.dump(robot_poses, open(filename2, "wb" ) )
    rospy.loginfo('Result written to file: %s \n %s' %(filename1,filename2))
    # The ensenso SKD will return the transformation of the robot's origin wrt. the camera frame
    try:
      res = calibrate_srv.call( robot_poses=robot_poses, pattern_poses=pattern_poses, 
                                camera_seed=camera_seed, pattern_seed=pattern_seed, setup='Fixed' )
    except rospy.ServiceException as e:
      logger.logwarn('calibrate_handeye service failed: ' + str(e))
    if not res.success:
      return
    # Store calibration results
    Tcamera_seed = criros.conversions.from_pose(camera_seed)
    seed_rot = tr.quaternion_from_matrix(Tcamera_seed)
    seed_pos = Tcamera_seed[:3,3]
    # Estimated pattern calibration pose. pattern wrt end-effector
    Tpattern_wrt_gripper = criros.conversions.from_pose(res.estimated_pattern_pose)
    pattern_rot = tr.quaternion_from_matrix(Tpattern_wrt_gripper)
    pattern_pos = Tpattern_wrt_gripper[:3,3]
    # Estimated transformation. World wrt camera
    Tworld_wrt_camera = criros.conversions.from_pose(res.estimated_camera_pose)
    rotation = tr.quaternion_from_matrix(Tworld_wrt_camera)
    translation = Tworld_wrt_camera[:3,3]
    angle, axis, point = tr.rotation_from_matrix( Tworld_wrt_camera )
    yamldata = {'camera_%s_robot' % robot_name: {
                  'parent': 'camera_optical_frame',
                  'child': '{0}/base_link'.format(robot_name),
                  'angle_axis': [angle] + axis.flatten().tolist(),
                  'duration': duration,
                  'grid_spacing': grid_spacing,
                  'iterations': res.iterations, 
                  'samples': samples,
                  'rotation': rotation.flatten().tolist(), 
                  'translation': translation.flatten().tolist(),
                  'pattern_pose': {
                    'rotation': pattern_rot.flatten().tolist(), 
                    'translation': pattern_pos.flatten().tolist()},
                  'seed': {
                    'rotation': seed_rot.flatten().tolist(), 
                    'translation': seed_pos.flatten().tolist()}},
                'reprojection_error': {
                  'pixels': res.reprojection_error}}
    # Write the file
    rospack = rospkg.RosPack()
    yaml_path  = rospack.get_path('axxb_data_collection') 
    yaml_path += '/config/camera_%s_robot_calibration.yaml' % robot_name
    # Write YAML file
    scriptname = os.path.basename(__file__)
    header =  '# This document was autogenerated by the script %s\n' % scriptname
    header += '# EDITING THIS FILE BY HAND IS NOT RECOMMENDED\n'
    header += '# NOTE 1: This is the transformation between the camera coordinates and the robot reference system.\n'
    header += '# NOTE 2: The pattern pose is the transformation of the calibration pattern wrt to the end-effector.\n'
    header += '# More information: http://www.ensenso.com/manual/index.html?_cameras_byserialno_$stereoserial_link.htm\n\n'
    with open(yaml_path, 'w') as f:
      f.write(header)
      yaml.safe_dump(yamldata, f)
    rospy.loginfo('Result written to file: %s' % yaml_path)
  
  def cb_left_info(self, msg):
    self.left_cam_info = msg

if __name__ == "__main__":
  # Initialize the node
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  calibration = RobotCameraCalibration()
  rospy.loginfo('Shuting down [%s] node' % node_name)
