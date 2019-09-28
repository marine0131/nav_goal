#! /usr/bin/env python

import rospy
import roslib
import actionlib
from nav_goal.msg import CustomMoveAction, CustomMoveGoal

def active_callback():
    rospy.loginfo("goal active")

def done_callback(state, res):
    rospy.loginfo('result: %s',res.result)

def feedback_callback(fb):
    rospy.loginfo('moving  %f%%',fb.percent_complete*100)

if __name__ == '__main__':
    rospy.init_node('custom_move_action_client')
    # create a client object
    client = actionlib.SimpleActionClient('nav_goal/custom_move_action', CustomMoveAction)
    rospy.loginfo('waite for server')
    # waite for server
    client.wait_for_server()
    rospy.loginfo('server ready')
    # waite for server

    # set goal
    goal = CustomMoveGoal()
    goal.type = 'angular'
    goal.vel = 0.4
    goal.dist = 1.57
    # gh = DoDishesActionGoal()
    # gh.goal.dishwasher_id = 100

    # send goal, set callback function
    client.send_goal(goal, done_callback, active_callback, feedback_callback)

    # waite for result
    client.wait_for_result(rospy.Duration.from_sec(20))
