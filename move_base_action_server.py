#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def move_base_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
 
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
 
    # Creates a goal to send to the action server.
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 2.85
    goal.target_pose.pose.position.y = -1.99
    goal.target_pose.pose.position.z = 0.00
    goal.target_pose.pose.orientation.x = 0.00
    goal.target_pose.pose.orientation.y = 0.00
    goal.target_pose.pose.orientation.z = 0.00
    goal.target_pose.pose.orientation.w = 1.00
 
    # Sends the goal to the action server.
    client.send_goal(goal)
 
    # Waits for the server to finish performing the action.
    client.wait_for_result()
 
    # Prints out the result of executing the action
    return client.get_result()  
 
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('client_py')
        result = move_base_client()
        if result :
            rospy.loginfo('%d', result)
            rospy.loginfo("Goal execution done") 
        else :
            rospy.loginfo("Failed!!")
    except rospy.ROSInterruptException:
        printf("program interrupted before completion", file=sys.stderr)