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
import IPython

class ErrorCollection(object):
  def __init__(self, logger=rospy):
    # Generic
    np.set_printoptions(precision=7, suppress=True)
    axes = []
    # Get required parameters
    has_serial, serial = read_parameter_err('/camera/serial')
    if not has_serial:
      raise Exception('Could not find all the required parameters')
    # Get optional parameters
    debug = read_parameter('~debug', False)
    robot_name = read_parameter('~robot_name', False)
    grid_spacing = read_parameter('~grid_spacing', 15.94)
    stepsize = read_parameter('~stepsize', 0.05)
    samples = int( read_parameter('~samples', 10) )
    pausetime = read_parameter('~pausetime', 1.0)
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
    # NOTE THAT THIS TRANSFORM IS ONLY FOR THE MODEL IN OPENRAVE (FOR COLLISION CHECKNING) # PLS USE THE ONE DECLARED LATER!
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
    # # Initial guess of the camera pose as Transformation. The pose must be given relative to 
    # # the robot hand (for a moving camera setup), or relative to the robot origin (for the fixed camera setup).
    # camera_seed = criros.conversions.to_pose(Tcamera)
    # # Initial guess of the pattern pose as Transformation. This pose must be given relative to
    # # the robot hand (for a fixed camera setup), or relative to the robot origin (for the moving camera setup).
    # pattern_seed = criros.conversions.to_pose(Tplate_wrt_gripper)
    # if debug:
    #   motion.draw_axes(Tcamera)
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
    Tplate_wrt_gripper = tr.quaternion_matrix([0.7161864681544047, 0.001856677420557697, -0.6978514634970954, 0.008765299563519747])
    Tplate_wrt_gripper[:3,3] = [0.024993163172439884, -0.0027451571511602985, 0.0060995670012445255]
    Tplate_wrt_gripper = np.dot(Tplate_wrt_gripper,tr.euler_matrix(np.pi,0,0))
    qstart = controller.get_joint_positions()
    motion.robot.SetActiveDOFValues(qstart)
    Tgripper_wrt_robot_start_pose = motion.get_transform()
    Tplate_wrt_robot_start_pose = np.dot( Tgripper_wrt_robot_start_pose, Tplate_wrt_gripper)
    Tplate_wrt_robot_start_pose = np.array([[-0.5609049,  0.5947824,  0.5758642,  0.4782526],
 [-0.4198947, -0.8038679,  0.4212894,  0.3683723],
 [ 0.7134943, -0.0054991,  0.7006395,  0.3831841],
 [ 0.       ,  0.       ,  0.       ,  1.       ]])
    Tgripper_wrt_robot_start_pose = np.array([[ 0.552164,   0.6087795,  0.5696511,  0.4626489],
 [ 0.4221552, -0.7933376,  0.4386347,  0.3529679],
 [ 0.7189575, -0.0017171, -0.6950519,  0.3694499],
 [ 0.       ,  0.       ,  0.       ,  1.       ]])
    # Start images streaming and collect pattern
    dynclient.update_configuration({'Images':True})
    rospy.sleep(pausetime)   # Wait for the camera to stabilize
    # Collect the pattern and robot poses
    try:
      res = pattern_srv.call(add_to_buffer=True, discard_patterns=True, average=False, decode=decode, grid_spacing=grid_spacing)
    except rospy.ServiceException as e:
      logger.logwarn('estimate_pattern_pose service failed: ' + str(e))
      return False
    if res.success:
        pattern_wrt_cam_start_pose = res.pose
    else:
        rospy.logerr('Init position with no pattern pose')
        return False

    # Move robot follow one axis of the pattern
    Xaxis = Tplate_wrt_robot_start_pose[:3,0]
    Yaxis = Tplate_wrt_robot_start_pose[:3,1]
    Zaxis = Tplate_wrt_robot_start_pose[:3,2]
    Tplate_goal = Tplate_wrt_robot_start_pose # init
    # X axis
    Xaxis_robot_poses = []
    Xaxis_pattern_poses = []
    i = 1
    rospy.loginfo('Xaxis collection starting..')
    while not rospy.is_shutdown() and (len(Xaxis_robot_poses) < samples and i <10):
        rospy.loginfo('i=%d'%(i))
        Tplate_goal[:3,3] += Xaxis*stepsize*(len(Xaxis_robot_poses)+i)
        Tgripper_goal = np.dot(Tplate_goal,criros.spalg.transform_inv(Tplate_wrt_gripper))
        qstart = controller.get_joint_positions()
        qgoal = motion.find_closest_iksolution(Tgripper_goal, qseed=qstart)
        if qgoal is None:
            i = i+1
            rospy.logwarn('No ik!')
            continue
        # Plan trajectory
        traj = motion.generate_trajectory(qstart, qgoal)
        if traj is None:
            i = i+1
            rospy.logwarn('Traj planning failed!')
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
            res = pattern_srv.call(add_to_buffer=True, discard_patterns=True, average=False, decode=decode, grid_spacing=grid_spacing)
        except rospy.ServiceException as e:
            logger.logwarn('estimate_pattern_pose service failed: ' + str(e))
            i = i+1
            continue
        if res.success:
            i = 1
            Xaxis_pattern_poses.append(criros.conversions.from_pose(res.pose))
            qrobot = controller.get_joint_positions()
            motion.robot.SetActiveDOFValues(qrobot)
            Tgripper = motion.get_transform()
            Tplate =  np.dot(Tgripper,Tplate_wrt_gripper)
            Xaxis_robot_poses.append(Tgripper)
        else:
            rospy.logwarn('No pattern detected!')
            i = i+1

    # IPython.embed()
    raw_input()

  def cb_left_info(self, msg):
    self.left_cam_info = msg

if __name__ == "__main__":
  # Initialize the node
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  calibration = ErrorCollection()
  rospy.loginfo('Shuting down [%s] node' % node_name)
