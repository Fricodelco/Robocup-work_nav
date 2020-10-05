#!/usr/bin/env python3
import rospy
from inverse_problem_srv.srv import point_cmd,point_cmdResponse
from std_srvs.srv import SetBool

home_pose = '190 0 170 0'

pose11 = '0 150 180 1.57'
pose21 = '0 150 180 1.57'
pose31 = '0 -150 180 -1.57'

pose12 = '-150 50 180 3.14'
pose22 = '-200 0 180 3.14'
pose32 = '-150 -50 180 -3.14'

pose1 = '-160 50 145 3.14'
pose2 = '-250 0 145 3.14'
pose3 = '-160 -50 145 -3.14'

table11 = '220 50 120 -0.2'
table21 = '220 00 120 0'
table31 = '220 -50 120 0.2'

table1 = '260 50 100 -0.2'
table2 = '260 0 90 100'
table3 = '260 -50 100 0.2'

ang_point = rospy.ServiceProxy('/angle_robot/cmd_point',point_cmd)
ang_grip = rospy.ServiceProxy('/angle_robot/gripper_cmd',SetBool)

def to_board(msg):
    if(msg.point == '0'):
        ang_point(home_pose)
        ang_point(pose11)
        ang_point(pose12)
        ang_point(pose1)
        ang_grip(True)
        ang_point(pose12)
        ang_point(pose11)
        ang_point(home_pose)
    if(msg.point == '1'):
        ang_point(home_pose)
        ang_point(pose21)
        ang_point(pose22)
        ang_point(pose2)
        ang_grip(True)
        ang_point(pose22)
        ang_point(pose21)
        ang_point(home_pose)
    if(msg.point == '2'):
        ang_point(home_pose)
        ang_point(pose31)
        ang_point(pose32)
        ang_point(pose3)
        ang_grip(True)
        ang_point(pose32)
        ang_point(pose31)
        ang_point(home_pose)
    return point_cmdResponse(True)

def from_board(msg):
    if(msg.point == '0'):
        ang_point(home_pose)
        ang_grip(True)
        ang_point(pose11)
        ang_point(pose12)
        ang_point(pose1)
        ang_grip(False)
        ang_point(pose12)
        ang_point(pose11)
        ang_point(home_pose)
    if(msg.point == '1'):
        ang_point(home_pose)
        ang_grip(True)
        ang_point(pose21)
        ang_point(pose22)
        ang_point(pose2)
        ang_grip(False)
        ang_point(pose22)
        ang_point(pose21)
        ang_point(home_pose)
    if(msg.point == '2'):
        ang_point(home_pose)
        ang_grip(True)
        ang_point(pose31)
        ang_point(pose32)
        ang_point(pose3)
        ang_grip(False)
        ang_point(pose32)
        ang_point(pose31)
        ang_point(home_pose)
    return point_cmdResponse(True)

def to_table(msg):
    if(msg.point == '0'):
        ang_point(table11)
        ang_point(table1)
        ang_grip(True)
        ang_point(table11)
        ang_point(home_pose)
    if(msg.point == '1'):
        ang_point(table21)
        ang_point(table2)
        ang_grip(True)
        ang_point(table21)
        ang_point(home_pose)
    if(msg.point == '2'):
        ang_point(table31)
        ang_point(table3)
        ang_grip(True)
        ang_point(table31)
        ang_point(home_pose)
    return point_cmdResponse(True)


if __name__=='__main__':
    rospy.init_node('transmit_object_to_and_from_board')
    rospy.Service('/angle_robot/object_to_board',point_cmd,to_board)
    rospy.Service('/angle_robot/object_from_board',point_cmd,from_board)
    rospy.Service('/angle_robot/object_to_table',point_cmd,to_table)
    rospy.spin()