#!/usr/bin/env python
import os
import rospy
import numpy as np
import openravepy as orpy
# ROS
import criros
import tf.transformations as tr
# Planning and control
from denso_openrave.motion import RoboMotion
from criros.raveutils import iktype5D, iktype6D
from denso_control.controllers import JointTrajectoryController
# Grippers
import actionlib
from robotiq_msgs.msg import CModelCommandAction, CModelCommandGoal


class ValidateCalibration(object):
  def __init__(self, logger=rospy):
    # Generic
    np.set_printoptions(precision=5, suppress=True)
    # Read values from the ROS parameter server
    id_planning = criros.read_parameter('/openrave_ids/planning', 1)
    id_state = criros.read_parameter('/openrave_ids/state', 2)
    # Load OpenRAVE environment
    env = orpy.Environment()
    orpy.RaveSetDebugLevel(orpy.DebugLevel.Fatal)
    worldxml = 'worlds/bimanual_ikea_assembly.env.xml'
    if not env.Load(worldxml):
      raise Exception('World file does not exist: %s' % worldxml)
    left_robot = env.GetRobot('denso_left')
    if left_robot is None:
      raise Exception('Could not find robot: denso_left')
    try:
      left_manip = left_robot.SetActiveManipulator('denso_ft_sensor_gripper')
    except:
      raise Exception('In left_robot could not find manipulator: denso_ft_sensor_gripper')
    right_robot = env.GetRobot('denso_right')
    if right_robot is None:
      raise Exception('Could not find robot: denso_right')
    try:
      right_manip = right_robot.SetActiveManipulator('denso_ft_sensor_gripper')
    except:
      raise Exception('In denso_right could not find manipulator: denso_ft_sensor_gripper')
    # Remove unnecessary objects from the world
    criros.raveutils.remove_bodies(env, remove=['ikea_stick', 'pin', 'reference_frame'])
    logger.loginfo('Loading motion planning and control pipeline')
    # Configure motion planning for the left robot
    left_motion = RoboMotion(left_manip)
    left_motion.change_active_manipulator('denso_ft_sensor_gripper', iktype=iktype5D)
    left_motion.scale_velocity_limits(0.2)
    left_motion.scale_acceleration_limits(0.2)
    left_motion.set_smoother_iterations(100)
    left_motion.set_planning_iterations(50)
    left_motion.enable_collision_checking(True)
    left_motion.update_link_stats()
    # Configure motion planning for the right robot
    right_motion = RoboMotion(right_manip)
    right_motion.change_active_manipulator('denso_ft_sensor_gripper', iktype=iktype5D)
    right_motion.scale_velocity_limits(0.2)
    right_motion.scale_acceleration_limits(0.2)
    right_motion.set_smoother_iterations(100)
    right_motion.set_planning_iterations(50)
    right_motion.enable_collision_checking(True)
    right_motion.update_link_stats()
    # Add extra protection to avoid hitting the table
    table_aabb = env.GetKinBody('table').GetLink('base').ComputeAABB()
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
    # OpenRAVE viewer
    env.SetViewer('QtCoin')
    # Trajectory controllers
    left_controller = JointTrajectoryController(namespace='left', log_level=rospy.ERROR)
    right_controller = JointTrajectoryController(namespace='right', log_level=rospy.ERROR)
    # Gripper controllers
    action_server = '/left/gripper/gripper_action_controller'
    left_gripper = actionlib.SimpleActionClient(action_server, CModelCommandAction)
    logger.loginfo( 'Waiting for [{0}] action server'.format(action_server) )
    if not left_gripper.wait_for_server(timeout=rospy.Duration(3.0)):
      logger.logerr('Timed out waiting for Gripper Controller'
                    'Action Server to connect. Start the action server'
                    'before running this node.')
      return
    else:
      logger.loginfo( 'Successfully connected to {0}'.format(action_server) )
    action_server = '/right/gripper/gripper_action_controller'
    right_gripper = actionlib.SimpleActionClient(action_server, CModelCommandAction)
    logger.loginfo( 'Waiting for [{0}] action server'.format(action_server) )
    if not right_gripper.wait_for_server(timeout=rospy.Duration(3.0)):
      logger.logerr('Timed out waiting for Gripper Controller'
                    'Action Server to connect. Start the action server'
                    'before running this node.')
      return
    else:
      logger.loginfo( 'Successfully connected to {0}'.format(action_server) )
    # Synch the OpenRAVE environment with TF. Give 2 secs for proper synch
    envid = orpy.RaveGetEnvironmentId(env)
    state_updater = criros.raveutils.RaveStateUpdater(envid=envid, fixed_frame='left/base_link', logger=rospy)
    rospy.sleep(2.0)
    state_updater.stop()
    print left_motion.robot.GetTransform()
    print right_motion.robot.GetTransform()
    # Find a pose in the middle of the two robots
    left_bb_center = left_motion.robot.ComputeAABB().pos()
    right_bb_center = right_motion.robot.ComputeAABB().pos()
    midpoint = (left_bb_center + right_bb_center) / 2.
    ldir = tr.unit_vector(right_bb_center - left_bb_center)
    rdir = tr.unit_vector(left_bb_center - right_bb_center)
    standoff = 0.0325
    left_ray = orpy.Ray(midpoint-standoff*ldir, ldir)
    right_ray = orpy.Ray(midpoint-standoff*rdir, rdir)
    # Look at the middle of the robots
    T = tr.euler_matrix(-2, 0, np.pi/2.)
    T[:3,3] = [2.4, midpoint[1], 1.57]
    env.GetViewer().SetCamera(T)
    # Show the goal poses
    left_motion.draw_axes(criros.conversions.from_ray(left_ray))
    left_motion.draw_axes(criros.conversions.from_ray(right_ray))
    # Close the grippers 
    left_motion.taskmanip.CloseFingers()
    right_motion.taskmanip.CloseFingers()
    open_cmd = CModelCommandGoal(position=0.085, velocity=0.1, force=100)
    close_cmd = CModelCommandGoal(position=0.0, velocity=0.1, force=100)
    left_gripper.send_goal(close_cmd)
    right_gripper.send_goal_and_wait(close_cmd)
    # Move left arm
    qinit = left_controller.get_joint_positions()
    qleft = left_motion.find_closest_iksolution(left_ray)
    traj = left_motion.generate_trajectory(qinit, qleft)
    ros_traj = left_motion.openrave_to_ros_traj(traj, rate=125.)
    left_controller.set_trajectory(ros_traj)
    left_controller.start()
    left_motion.robot.GetController().SetPath(traj)
    left_controller.wait()
    # Move right arm
    qinit = right_controller.get_joint_positions()
    qright = right_motion.find_closest_iksolution(right_ray)
    traj = right_motion.generate_trajectory(qinit, qright)
    ros_traj = right_motion.openrave_to_ros_traj(traj, rate=125.)
    right_controller.set_trajectory(ros_traj)
    right_controller.start()
    right_motion.robot.GetController().SetPath(traj)
    right_controller.wait()
    import IPython
    IPython.embed()

if __name__ == "__main__":
  # Initialize the node
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  validate = ValidateCalibration()
  rospy.loginfo('Shuting down [%s] node' % node_name)
