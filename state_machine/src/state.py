import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from srv_msg.srv import DistCmd, DistCmdResponse
from nav_msgs.srv import GetMap, GetPlan
from std_msgs.msg import Bool
import tf
from math import pi, cos, sin, sqrt, atan2
import numpy as np

class state_machine:
    def __init__(self):
        self.status_pub = rospy.Publisher("navigation_result", Bool, queue_size = 5)
        self.client_move_base = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client_move_base.wait_for_server()
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		
        self.target_pose = (0,0,0)

    def reach_goal(self, x, y, theta):
        target_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        t0=rospy.Time.now()
        goal=MoveBaseGoal()
        goal.target_pose.header.stamp=t0
        goal.target_pose.header.frame_id="map"
        goal.target_pose.pose.position = Point(x, y, 0)
        goal.target_pose.pose.orientation.x = target_quat[0]
        goal.target_pose.pose.orientation.y = target_quat[1]
        goal.target_pose.pose.orientation.z = target_quat[2]
        goal.target_pose.pose.orientation.w = target_quat[3]  
        self.client_move_base.send_goal(goal)
        reached_the_goal = self.client_move_base.wait_for_result(rospy.Duration.from_sec(60))
        if not reached_the_goal:
            rospy.logwarn("I was not able to reach the goal within the\
                          allocated time")
            self.client.cancel_goal()    
        return self.client.get_state()


if __name__ == '__main__':
    rospy.init_node('state_machine')
    state = state_machine()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

        
