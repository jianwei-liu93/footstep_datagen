#!/usr/bin/python3

import rospy
import numpy as np
import rospkg

rospack = rospkg.RosPack()
pkgpath = rospack.get_path("footstep_datagen")
datapath = pkgpath + "/data/"
f_name = "2D_footsteps_ex.npy"
f_path = datapath + f_name

test_path = np.load(f_path)
print(test_path)