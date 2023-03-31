#!/usr/bin/python3

import rospy
import rospkg
import numpy as np
import os, math
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D

from humanoid_nav_msgs.srv import PlanFootsteps,PlanFootstepsRequest, PlanFootstepsResponse 
from humanoid_nav_msgs.msg import StepTarget

class dataGenerator():
    def __init__(self) -> None:
        rospy.init_node('footstep_datagen_node')
        # get package path
        rospack = rospkg.RosPack()
        pkgpath = rospack.get_path("footstep_datagen")
        self.datapath = pkgpath + "/data/"


        self.ros_map_loaded = False
        self.map_sub_ = rospy.Subscriber('map', OccupancyGrid, self.map_callback)

        rospy.wait_for_service('plan_footsteps')
        self.footstep_plan_srv = rospy.ServiceProxy("plan_footsteps", PlanFootsteps)
        file_num_start = 0
        file_num_end = 10000
        for file_num in range(file_num_start, file_num_end):
            path = None
            while (path == None):
                path = self.gen_rand_path()

            path_np = self.convert_path_np(path)
            f_name = "2D_footsteps_" + str(file_num) + ".npy"
            f_path = self.datapath + f_name
            np.save(f_path, path_np)

        rospy.spin()


    def map_callback(self, map_grid):
        self.ros_map_loaded = True
        self.ros_map = map_grid.data
        self.map_info = map_grid.info
        self.map_res = map_grid.info.resolution
        self.map_width = map_grid.info.width * self.map_res
        self.map_height = map_grid.info.height * self.map_res
        self.origin = [map_grid.info.origin.position.x, 
                       map_grid.info.origin.position.y]
        self.map_range_x = [self.origin[0], 
                            self.origin[0]+self.map_width]
        self.map_range_y = [self.origin[1], 
                    self.origin[1]+self.map_height]
        self.map_free_thres = 0.1
    
    def convert_path_np(self, path):
        path_np = np.array([])
        for step in path:
            step_np = np.array([step.pose.x,  step.pose.y, step.pose.theta, step.leg]).T
            if (path_np.size == 0):
                path_np = step_np
            else:
                path_np = np.c_[path_np, step_np]
        
        return path_np

    def gen_rand_path(self):
        start_pose = self.gen_valid_pose()
        goal_pose = self.gen_valid_pose()
        req = PlanFootstepsRequest()
        req.start = start_pose
        req.goal = goal_pose
        resp = self.footstep_plan_srv(start_pose, goal_pose)
        if (resp.result):
            path = resp.footsteps
            return path
        else:
            return None


    def gen_valid_pose(self):
        rand_pose = self.gen_random_pose()
        # generate until pose is valid:
        while not (self.check_position_valid(rand_pose)):
            rand_pose = self.gen_random_pose()
        return rand_pose


    def gen_random_pose(self):
        if (self.ros_map_loaded): 
            rand_pose = Pose2D()
            rand_pose.x = np.random.uniform(
                                    self.map_range_x[0], 
                                    self.map_range_x[1])
            rand_pose.y = np.random.uniform(
                                    self.map_range_y[0], 
                                    self.map_range_y[1])
            rand_pose.theta = np.random.uniform(0, 
                                                        2*np.math.pi)
            return rand_pose
        else:
            return None

    
    
    def check_position_valid(self, pos):
        if (pos == None):
            return False

        if not self.check_in_range(pos.x, self.map_range_x):
            return False
        
        if not self.check_in_range(pos.y, self.map_range_y):
            return False

        if not self.check_occupied(pos):
            return False
        
        return True


    def check_in_range(self, num, range):
        if (num >= range[0]) and (num <= range[1]):
            return True
        else:
            return False
    
    def check_occupied(self, position):
        x_pos_map =  math.floor((position.x - self.origin[0])/self.map_res)
        y_pos_map =  math.floor((position.y - self.origin[1])/self.map_res)
        width_map = self.map_info.width
        idx = y_pos_map*width_map + x_pos_map
        if (self.ros_map[idx] <= self.map_free_thres):
            return True
        else:
            return False


if __name__ == "__main__":
    data_generator = dataGenerator()
