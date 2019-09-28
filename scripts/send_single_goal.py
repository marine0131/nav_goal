#! /usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseGoal


if __name__ == '__main__':
    rospy.init_node("send_single_goal")
    GoalID=rospy.get_param("~goal","goal")
    goal_pub = rospy.Publisher('nav_goal/goal', MoveBaseGoal, queue_size=1)
    goal_dict = {}

    goal = MoveBaseGoal()
    goal1 = MoveBaseGoal()

    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = -7.44262039743
    goal.target_pose.pose.position.y = 1.72414786896
    goal.target_pose.pose.position.z = 1.62304826538e-05
    goal.target_pose.pose.orientation.w = 0.54337521607
    goal.target_pose.pose.orientation.x = -0.00271313213747
    goal.target_pose.pose.orientation.y = -0.00165216859443
    goal.target_pose.pose.orientation.z = -0.839483938985
    goal.target_pose.header.stamp = rospy.Time.now()

    goal1.target_pose.header.frame_id = 'map'
    goal1.target_pose.pose.position.x = -4.4529918866
    goal1.target_pose.pose.position.y = 8.30406282363
    goal1.target_pose.pose.position.z = -7.84321296057e-06
    goal1.target_pose.pose.orientation.w = 0.606883731009
    goal1.target_pose.pose.orientation.x = -0.00117223038202
    goal1.target_pose.pose.orientation.y = -0.000323306787229
    goal1.target_pose.pose.orientation.z = -0.794789694438
    goal1.target_pose.header.stamp = rospy.Time.now()

    goal_dict['goal'] = goal
    goal_dict['goal1'] = goal1

    rospy.sleep(rospy.Duration(2.0))

    if GoalID in goal_dict:
        goal_pub.publish(goal_dict[GoalID])
        rospy.loginfo('published a goal')
