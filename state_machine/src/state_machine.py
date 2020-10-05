#!/usr/bin/env python3  

import rospy
from state_mobile_robot import Mobile_Robot_Machine
from state_robotic_arm import Robotic_Arm_Machine
import sys

table_from = ['t_3','t_2','t_1']
table_to = ['t_4','t_5']
object_from = [[0,3],[],[4]]
object_to = [[0,4],[3]]

object_on_board = [[],[]]
ind_on_board = 0
right_step = 0

if __name__ == '__main__':
    rospy.init_node('state_machine')
    mobile_robot = Mobile_Robot_Machine()
    robotic_arm = Robotic_Arm_Machine()

    mobile_robot.to_home()

    for i,table in enumerate(table_from):
        print('table index {}, table name {}'.format(i,table))
        if(len(object_from[i])==0):
            print('next table')
            continue
        elif(not mobile_robot.to_table(table)):
            sys.exit()
        cur_object = object_from[i]
        for ob in cur_object:
            print('try grasp object {}'.format(ob))
            for i in range(3):
                if(not robotic_arm.grasp_object(ob)):
                    mobile_robot.move_robot('right',0.1)
                    right_step +=1
                else:
                    print('object {} is grasped'.format(ob))
                    robotic_arm.object_to_board(ind_on_board)
                    object_on_board[0].append(ob)
                    object_on_board[1].append(ind_on_board)
                    ind_on_board +=1
                    break
            if(not right_step == 0):
                mobile_robot.move_robot('left',0.1*right_step)
                right_step = 0
    
    for i,table in enumerate(table_to):
        print('table index {}, table name {}'.format(i,table))
        if(len(object_to[i])==0):
            print('next table')
            continue
        elif(not mobile_robot.to_table(table)):
            sys.exit()
        cur_object = object_to[i]
        ind_on_table = 0
        for ob in cur_object:
            print('object {} to table in pose {}'.format(ob,ind_on_table))
            try:
                ind_object = object_on_board[0].index(ob)
            except ValueError:
                ind_object = -1
            if (ind_object == -1):
                print('cant find object {} on board'.format(ob))
                continue
            robotic_arm.object_from_board(object_on_board[1][ind_object])
            robotic_arm.object_to_table(ind_on_table)
            ind_on_table +=1
    
    mobile_robot.to_exit()