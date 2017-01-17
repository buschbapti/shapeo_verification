#!/usr/bin/env python
import rospy
import actionlib
from shapeo_verification.msg import VerifyInsertionAction, VerifyInsertionGoal
import sys


if __name__ == '__main__':
    rospy.init_node('verify_insertion_client')
    client = actionlib.SimpleActionClient('verify_insertion', VerifyInsertionAction)
    client.wait_for_server()

    goal = VerifyInsertionGoal()
    goal.shape = sys.argv[1]
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))