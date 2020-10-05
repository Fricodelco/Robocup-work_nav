#!/usr/bin/env python3
import rospy
from inverse_problem_srv.srv import point_cmd,point_cmdResponse

rospy.init_node('transmit_object_to_and_from_board')

take = rospy.ServiceProxy('/angle_robot/grasp_object',point_cmd)
to = rospy.ServiceProxy('/angle_robot/object_to_board',point_cmd)
from_ = rospy.ServiceProxy('/angle_robot/object_from_board',point_cmd)

take('0')
to('1')
from_('1')