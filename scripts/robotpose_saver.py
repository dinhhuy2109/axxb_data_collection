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


if __name__ == "__main__":
  # Initialize the node
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  calibration = AxxbRobotposeSaver()
  rospy.loginfo('Shuting down [%s] node' % node_name)
