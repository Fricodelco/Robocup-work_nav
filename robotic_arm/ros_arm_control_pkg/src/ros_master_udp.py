#!/usr/bin/env python3
import rospy
import socket
from inverse_problem_srv.srv import point_cmd,point_cmdResponse
from std_srvs.srv import SetBool
import _thread
import os
import json
from RoboticArmClass import RoboticArm
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pub_joint = rospy.Publisher('/angle_robot/joint_states',JointState,queue_size = 10)
server_address_ang = server_address = 0
gripper = '0'
ang_status = -1
ang_point = 'g:190:0:170:0'
current_joint_state = 0
joint_name = ['ang_joint1','ang_joint2','ang_joint3','ang_joint4','ang_joint5','ang_gripper']
search_pose = 0

def read_udp_feedback(thread_name, delay):
    global ang_status,server_address
    sock.bind(server_address)
    while True:
        data, address = sock.recvfrom(4)
        if not data == None:
            data = data.decode()
            if(data[0] == 'g'):
                ang_status = int(data[2:])
        rospy.sleep(delay)
    
def pub_joint_states(thread_name,delay):
    global current_joint_state
    data = ang_point.split(':')
    data = list(map(float,data[1:]))
    roboticArm = RoboticArm()
    availJointState,goalJointState = roboticArm.InversProblem(data[0],data[1],data[2],-1.57,data[3])
    msg_joint_state = JointState()
    msg_joint_state.name = joint_name
    current_joint_state = goalJointState
    while True:
        msg_joint_state.position = current_joint_state[:-1]+[current_joint_state[0]+current_joint_state[-1]]+[0]
        msg_joint_state.header.stamp = rospy.Time.now()
        pub_joint.publish(msg_joint_state)
        rospy.sleep(delay)

def angle_point_remap(msg):
    global ang_status, ang_point,current_joint_state,search_pose
    search_pose = 0
    cmd = list(map(float,msg.point.split()))
    roboticArm = RoboticArm()
    availJointState,goalJointState = roboticArm.InversProblem(cmd[0],cmd[1],cmd[2],-1.57,cmd[3])
    if(not availJointState):
        print('unreacheble')
    #    return False
    current_joint_state = goalJointState
    str_cmd = 'g:' + msg.point.replace(' ',':')
    if(str_cmd == ang_point):
        return True
    ang_point = str_cmd
    str_cmd = str_cmd +':'+gripper+'#'
    sent = sock.sendto(str.encode(str_cmd), server_address_ang)
    ang_status = -1
    rospy.sleep(1)
    result = False
    start_time = rospy.get_time()
    while True:
        if(ang_status == 0):
            result = True
            ang_status = -1
            break
        if(rospy.get_time()-start_time>10):
            sent = sock.sendto(str.encode(str_cmd), server_address_ang)
            start_time = rospy.get_time()
        else:
            rospy.sleep(0.2)
    return result

def angle_gripper_remap(msg):
    global gripper,ang_status
    gripper_new = str(int(not msg.data))
    if(gripper_new == gripper):
        return True, 'Success'
    else:
        gripper = gripper_new
    str_cmd = ang_point +':'+gripper+'#'
    sent = sock.sendto(str.encode(str_cmd), server_address_ang)
    ang_status = -1
    rospy.sleep(1)
    result = False
    while True:
        if(ang_status == 0):
            result = True
            ang_status = -1
            break
        else:
            rospy.sleep(0.5)
    return True, 'Success'

def init_udp_param():
    global server_address_ang, server_address
    THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(THIS_FOLDER, 'config.json')
    if config_file==None:
        return False
    with open(config_file) as json_file:
        data = json.load(json_file)
        server_address = ('',data['input_server_port'])
        server_address_ang = (data['ang_address'],data['ang_port'])
    return True

def set_searching_pose(msg):
    global current_joint_state,search_pose
    if(search_pose == 1):
        return []
    search_pose = 1
    current_joint_state = [0,0,0.698,-0.698,0]
    str_cmd = '1:1'
    sent = sock.sendto(str.encode(str_cmd), server_address_ang)
    rospy.sleep(6)
    return []

if __name__ == '__main__':
    rospy.init_node('ros_udp')
    init_udp_param()
    _thread.start_new_thread( read_udp_feedback, ("read_udp_thread", 0.1))
    _thread.start_new_thread( pub_joint_states, ("pub_joint_states_thread", 1))
    rospy.Service('/angle_robot/cmd_point',point_cmd,angle_point_remap)
    rospy.Service('/angle_robot/gripper_cmd',SetBool,angle_gripper_remap)
    rospy.Service('/angle_robot/searching_pose',Empty,set_searching_pose)
    rospy.spin()
