#! /usr/bin/env python

import roslib
import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def MoveBaseMessage(x, y, yaw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0 

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    print(quaternion)

    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    return goal




if __name__ == '__main__':
    rospy.init_node('move_base_client')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    
    
    # Fill in the goal here
    
    x = 0
    y = 0
    goal = MoveBaseMessage(x, y, 2)
    client.send_goal(goal)
    
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        x += 2
        y += 1
        goal = MoveBaseMessage(x, y, 2)
        state = client.get_state()
        print("state",state)
        if(state == 3 ):
            client.send_goal(goal)
        elif(state == 4):
            print("Aborted")
            client.cancel_all_goals()
        client.wait_for_result(rospy.Duration.from_sec(5.0))

        rate.sleep()




