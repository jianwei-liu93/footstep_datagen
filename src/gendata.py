#!/usr/bin/python3

import rospy
import rospkg
import numpy as np
import os, math
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped 

from humanoid_nav_msgs.srv import PlanFootsteps, PlanFootstepsResponse 


class dataGenerator():
    def __init__(self) -> None:
        rospy.init_node('footstep_datagen_node')
        # get 
        rospack = rospkg.RosPack()
        pkgpath = rospack.get_path("footstep_datagen")
        self.datapath = pkgpath + "/data/"

        self.map_sub_ = rospy.Subscriber('map', OccupancyGrid, self.map_callback)

        rospy.wait_for_service('plan_footsteps')
        rospy.spin()


    def map_callback(self, map_grid):
        self.ros_map_loaded = True
        self.ros_map = map_grid.data
        self.map_res = map_grid.info.resolution
        self.map_width = map_grid.info.width * self.map_res
        self.map_height = map_grid.info.height * self.map_res
        self.origin = [map_grid.info.origin.position.x, 
                       map_grid.info.origin.position.y]
        self.map_range_x = [self.origin[0], 
                            self.origin[0]+self.map_width]
        self.map_range_y = [self.origin[1], 
                    self.origin[1]+self.map_height]

    def check_position_valid(self, pos):
        x_pos_map =  round(pos.position.x/self.map_res)
        y_pos_map =  round(pos.position.x/self.map_res)

        if not self.check_in_range(x_pos_map, self.map_range_x):
            return False
        
        if not self.check_in_range(x_pos_map, self.map_range_x):
            return False

        


    def check_in_range(self, num, range):
        
    



if __name__ == "__main__":
    data_generator = dataGenerator()
