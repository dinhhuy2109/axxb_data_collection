#!/usr/bin/env python
import os
import copy
import rospy
import numpy as np
import tf.transformations as tr
import matplotlib.pyplot as plt

import IPython
import pickle
# define constant
XAXIS = 0
YAXIS = 1
ZAXIS = 2

AXIS = XAXIS
rospack = rospkg.RosPack()
filepath  = rospack.get_path('axxb_data_collection') 
filename = filepath + "/config/pattern_poses_wrt_cam_axis_" + str(AXIS)
pattern_poses_wrt_cam =  pickle.load(open( filename, "rb" ) )
filename = filepath+"/config/robot_poses_axis_" + str(AXIS)
pattern_poses_wrt_robot =  pickle.load(open( filename, "rb" ) )

trans_dist_wrt_robot = [np.linalg.norm(pose[:3,3] - pattern_poses_wrt_robot[0][:3,3]) for pose in pattern_poses_wrt_robot]

trans_dist_wrt_cam = [np.linalg.norm(pose[:3,3] - pattern_poses_wrt_cam[0][:3,3]) for pose in pattern_poses_wrt_cam]

trans_errs = trans_dist_wrt_robot-trans_dist_wrt_cam

plt.scatter(trans_dist_wrt_robot, trans_errs)
line, = plt.plot(trans_dist_wrt_robot, trans_errs, '-')
plt.show()
