#! /usr/bin/env python

import rospy
import actionlib
import math
from geometry_msgs.msg import Twist, PoseStamped
from tf import transformations
from nav_goal.msg import CustomMoveAction, CustomMoveFeedback, CustomMoveResult


class CustomMoveActionServer:
    def __init__(self):
        self.pose_msg = PoseStamped()

        rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.cmd_vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=1)

        self.server = actionlib.SimpleActionServer('nav_goal/custom_move_action',
                                                   CustomMoveAction,
                                                   self.execute_move_cb, False)
        self.server.start()

    def pose_callback(self, msg):
        self.pose_msg.header = msg.header
        self.pose_msg.pose = msg.pose

    def execute_move_cb(self, goal):
        percent = CustomMoveFeedback()
        res = CustomMoveResult()

        vel = Twist()
        origin = []
        vel = goal.vel
        if goal.type == 'linear':
            origin = self.get_position_list(self.pose_msg)
        elif goal.type == 'angular':
            origin = self.get_orientation_list(self.pose_msg)
            # rotate more than pi
            if goal.dist > math.pi:
                goal.dist = goal.dist - math.pi * 2
            elif goal.dist < -math.pi:
                goal.dist = goal.dist + math.pi * 2

        else:
            rospy.logwarn('undefined custom move cmd')
            res.result = 'undefined custom move cmd'
            self.server.set_succeeded(res)
            return

        # rospy.loginfo('custom move: "%s",vel=%f,dist=%f',goal.type, goal.vel, goal.dist)
        complete_flag = False
        rate = rospy.Rate(20)
        while not complete_flag and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(vel)
            curr = []
            if goal.type == 'linear':
                curr = self.get_position_list(self.pose_msg)
                diff = math.sqrt((curr[1] - origin[1])**2 +
                                 (curr[0] - origin[0])**2)
            elif goal.type == 'angular':
                curr = self.get_orientation_list(self.pose_msg)
                diff = abs(transformations.euler_from_quaternion(curr)[2] -
                           transformations.euler_from_quaternion(origin)[2])
                # cross over pi & -pi
                if diff > math.pi:
                    diff = math.pi * 2 - diff
            else:
                pass
            # curr = self.get_orientation_list(self.pose_msg)
            # rospy.logwarn('yaw: %f', transformations.euler_from_quaternion(curr)[2]*57.3)
            # rospy.logwarn ('diff:%f', diff)
            percent.percent_complete = diff / goal.dist
            self.server.publish_feedback(percent)
            if diff > 0.98 * goal.dist:
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                self.cmd_vel_pub.publish(vel)
                complete_flag = True
            rate.sleep()

        res.result = 'success'
        self.server.set_succeeded(res)

    def get_position_list(self, pose_msg):
        return [pose_msg.pose.position.x, pose_msg.pose.position.y]

    def get_orientation_list(self, pose_msg):
        return [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]


if __name__ == '__main__':
    rospy.init_node('custom_move_action')
    vel_topic = rospy.get_param('~vel_topic', 'cmd_vel')
    server = CustomMoveActionServer()
    while not rospy.is_shutdown():
        rospy.spin()
