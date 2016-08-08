#!/usr/bin/env python
import sys
import copy
import rospy
import rospkg
import numpy as np
import tf.transformations as tr
import matplotlib.pyplot as plt

import IPython
import pickle
# define constant
XAXIS = 0
YAXIS = 1
ZAXIS = 2

if len(sys.argv) == 1:
    AXIS = raw_input('Enter which axis to be processed (XAXIS = 0, YAXIS = 1, ZAXIS = 2).\nAXIS: ')
    
else:
    AXIS = sys.argv[1]
    print "AXIS:", AXIS
rospack = rospkg.RosPack()
filepath  = rospack.get_path('axxb_data_collection') 
filename = filepath + "/config/pattern_poses_wrt_cam_axis_" + str(AXIS)
pattern_poses_wrt_cam =  pickle.load(open( filename, "rb" ) )
filename = filepath+"/config/pattern_poses_wrt_robot_axis_" + str(AXIS)
pattern_poses_wrt_robot =  pickle.load(open( filename, "rb" ) )

trans_dist_wrt_robot = [np.linalg.norm(pose[:3,3] - pattern_poses_wrt_robot[0][:3,3]) for pose in pattern_poses_wrt_robot]

trans_dist_wrt_cam = [np.linalg.norm(pose[:3,3] - pattern_poses_wrt_cam[0][:3,3]) for pose in pattern_poses_wrt_cam]
trans_errs = [trans_dist_wrt_cam[i]-trans_dist_wrt_robot[i] for i in range(len(trans_dist_wrt_robot))]
print trans_errs
print trans_dist_wrt_robot

plt.scatter(trans_dist_wrt_robot, trans_errs)
line, = plt.plot(trans_dist_wrt_robot, trans_errs, '-')

ax = plt.gca()
plt.xlabel('dist_wrt_robot (meters) AXIS: %s'%AXIS)
plt.ylabel('error = dist_wrt_cam - dist_wrt_robot')
# ax.set(aspect='equal')
start, end = ax.get_xlim()
# ax.xaxis.set_ticks(np.arange(start, end, 0.01))
# start, end = ax.get_ylim()
# ax.yaxis.set_ticks(np.arange(start, end, 0.002))

plt.show()
