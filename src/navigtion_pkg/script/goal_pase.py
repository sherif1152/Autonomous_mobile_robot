#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

def move_robot(x, y, angle):
    rospy.init_node('move_robot_node', anonymous=True)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose = Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, angle, 1.0))

    client.send_goal(goal)
    client.wait_for_result()

if __name__ == '__main__':
    try:
        # Move the robot forward to position (1, 0) with an angle of 0 radians
        move_robot(-0.008315, -5.282071, 0.0)
        rospy.sleep(3)


        # Move the robot to the right to position (1, 1) with an angle of 90 degrees (1.57 radians)
        move_robot(2.248869, 6.635135, 1.962612)

        # Move the robot backward to position (0, 1) with an angle of 180 degrees (3.14 radians)
        move_robot(-4.572061, 3.908484, -3.128271)
        rospy.sleep(2)

        # Move the robot to the left to position (0, 0) with an angle of -90 degrees (-1.57 radians)
        move_robot(0.0, 0.0, -1.57)
        rospy.sleep(5)


    except rospy.ROSInterruptException:
        pass

