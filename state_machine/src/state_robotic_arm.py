#!/usr/bin/env python3  

import rospy
from inverse_problem_srv.srv import point_cmd,point_cmdResponse

class Robotic_Arm_Machine():
    def __init__(self):
        self.grasp_object_srv = rospy.ServiceProxy('/angle_robot/grasp_object',point_cmd)
        self.object_to_board_srv = rospy.ServiceProxy('/angle_robot/object_to_board',point_cmd)
        self.object_from_board_srv = rospy.ServiceProxy('/angle_robot/object_from_board',point_cmd)
        self.object_to_table_srv = rospy.ServiceProxy('/angle_robot/object_to_table',point_cmd)

    def grasp_object(self,index_object):
        res = self.grasp_object_srv(str(index_object))
        return res.result
    
    def object_to_board(self,index_board):
        res = self.object_to_board_srv(str(index_board))
        return res.result
    
    def object_from_board(self,index_board):
        res = self.object_from_board_srv(str(index_board))
        return res.result

    def object_to_table(self,index_table):
        res = self.object_to_table_srv(str(index_table))
        return res.result