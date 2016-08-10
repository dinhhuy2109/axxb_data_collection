#!/usr/bin/env python
import sys
import copy
import rospy
import rospkg
import numpy as np
import tf.transformations as tr
import matplotlib.pyplot as plt
import COPE

import IPython
import pickle

def help_func():
    print "\033[93mThis program requires some input parameters. Please enter as follow:\nprocess_error_data.py -rot/-trans [element]\neg. process_error_data.py -rot 0\033[0m"

# define constant
XAXIS = 0
YAXIS = 1
ZAXIS = 2
PROCESS_ROT = False

if len(sys.argv) != 3 :
    print help_func()
    quit()

if sys.argv[1] == "-rot":
    print "\033[92mProcess ROTATION data!\033[0m"
    PROCESS_ROT = True
elif sys.argv[1] == "-trans":
    print "\033[92mProcess TRANSLATION data!\033[0m"
    PROCESS_ROT = False
else:
    print help_func()
    quit()

ELEMENT = sys.argv[2]
AXIS = sys.argv[2]

if PROCESS_ROT:
    rospack = rospkg.RosPack()
    filepath  = rospack.get_path('axxb_data_collection') 
    filename = filepath + "/config/pattern_poses_wrt_cam_rot_" + str(ELEMENT)
    pattern_poses_wrt_cam =  pickle.load(open( filename, "rb" ) )
    filename = filepath+"/config/pattern_poses_wrt_robot_rot_" + str(ELEMENT)
    pattern_poses_wrt_robot =  pickle.load(open( filename, "rb" ) )
    
    rot_dist_wrt_robot = [np.linalg.norm(COPE.RotToVec(np.dot(pose[:3,:3],np.linalg.inv(pattern_poses_wrt_robot[0][:3,:3])))) for pose in pattern_poses_wrt_robot]
    rot_dist_wrt_cam = [np.linalg.norm(COPE.RotToVec(np.dot(pose[:3,:3],np.linalg.inv(pattern_poses_wrt_cam[0][:3,:3])))) for pose in pattern_poses_wrt_cam]
    rot_errs = [ rot_dist_wrt_cam[i] - rot_dist_wrt_robot[i] for i in range(len(rot_dist_wrt_robot))]

    plt.scatter(rot_dist_wrt_robot, rot_errs)
    line, = plt.plot(rot_dist_wrt_robot, rot_errs, '-')
    ax = plt.gca()
    plt.xlabel('dist_wrt_robot ELEMENT: %s'%ELEMENT)
    plt.ylabel('rot_error = dist_wrt_cam - dist_wrt_robot')
    start, end = ax.get_xlim()
    plt.show()
else:
    rospack = rospkg.RosPack()
    filepath  = rospack.get_path('axxb_data_collection') 
    filename = filepath + "/config/pattern_poses_wrt_cam_axis_" + str(AXIS)
    pattern_poses_wrt_cam =  pickle.load(open( filename, "rb" ) )
    filename = filepath+"/config/pattern_poses_wrt_robot_axis_" + str(AXIS)
    pattern_poses_wrt_robot =  pickle.load(open( filename, "rb" ) )

    trans_dist_wrt_robot = [np.linalg.norm(pose[:3,3] - pattern_poses_wrt_robot[0][:3,3]) for pose in pattern_poses_wrt_robot]
    trans_dist_wrt_cam = [np.linalg.norm(pose[:3,3] - pattern_poses_wrt_cam[0][:3,3]) for pose in pattern_poses_wrt_cam]

    # length = len(trans_dist_wrt_robot)
    # trans_dist_wrt_robot = [np.linalg.norm(pattern_poses_wrt_robot[length-i-1][:3,3] - pattern_poses_wrt_robot[-1][:3,3]) for i in range(length)]
    # trans_dist_wrt_cam = [np.linalg.norm(pattern_poses_wrt_cam[length-i-1][:3,3] - pattern_poses_wrt_cam[-1][:3,3])  for i in range(length)]

    trans_errs = [trans_dist_wrt_cam[i]-trans_dist_wrt_robot[i] for i in range(len(trans_dist_wrt_robot))]
    # print trans_errs
    # print trans_dist_wrt_robot

    plt.scatter(trans_dist_wrt_robot, trans_errs)
    line, = plt.plot(trans_dist_wrt_robot, trans_errs, '-')

    ax = plt.gca()
    plt.xlabel('dist_wrt_robot (meters) AXIS: %s'%AXIS)
    plt.ylabel('trans_error = dist_wrt_cam - dist_wrt_robot')
    start, end = ax.get_xlim()
    plt.show()
