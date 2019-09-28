#! /usr/bin/env python

import rospy
import actionlib
import math
from geometry_msgs.msg import PoseStamped, Twist
from nav_goal.msg import CustomMoveAction, CustomMoveGoal
from tf import transformations
from nav_goal.srv import QRcodeAdjust, QRcodeAdjustResponse


class ArucoAdjust():
    def __init__(self, ):
        self.pose = PoseStamped()
        self.new_msg = False

        rospy.Subscriber('/ar_single_board/pose', PoseStamped, self.ar_pose_callback)
        self.move_client = actionlib.SimpleActionClient('nav_goal/custom_move_action', CustomMoveAction)
        rospy.Service('/nav_goal/qrcode_adjust', QRcodeAdjust, self.handle_qrcode_adjust_rt)
        self.vel_pub = rospy.Publisher('/smoother_cmd_vel', Twist, queue_size=1)

        rospy.loginfo('waiting for server')
        self.move_client.wait_for_server()
        rospy.loginfo('server connectted')

    def handle_qrcode_adjust_rt(self, req):
        res = QRcodeAdjustResponse()
        false_count = 0
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.new_msg:
                self.new_msg = False
                false_count = 0
                euler = transformations.euler_from_quaternion([self.pose.pose.orientation.x,
                                                               self.pose.pose.orientation.y,
                                                               self.pose.pose.orientation.z,
                                                               self.pose.pose.orientation.w])

                rospy.loginfo('pose:[%.3f,%.3f,%.3f],yaw:%.3f', self.pose.pose.position.x,
                              self.pose.pose.position.y, self.pose.pose.position.z, euler[2])

                vel_msg = Twist()
                if abs(self.pose.pose.position.x) <= 0.02 and abs(self.pose.pose.position.y) <= 0.04 and abs(euler[2]) < 0.05:
                    self.vel_pub.publish(vel_msg)
                    res.message = "success"
                    res.success = True
                    break

                elif abs(self.pose.pose.position.y) >= 0.5 or abs(self.pose.pose.position.x) >= 0.5 or euler[2] > 0.7:
                    continue

                else:
                    vx = 0.5 * abs(self.pose.pose.position.y) + 0.03
                    vy = 0.5 * abs(self.pose.pose.position.x) + 0.03
                    vel_msg.linear.x = vx if self.pose.pose.position.y < 0 else -vx
                    vel_msg.linear.y = vy if self.pose.pose.position.x < 0 else -vy
                    vel_msg.angular.z = - euler[2]
                    self.vel_pub.publish(vel_msg)
            else:
                false_count = false_count + 1
                rospy.logwarn("no ar_code")
                if false_count > 10:
                    res.message = 'no aruco code detected'
                    res.success = False
                    break
            rate.sleep()
        return res

    def handle_qrcode_adjust(self, req):
        res = QRcodeAdjustResponse()
        repeat_time = 0
        wrong_time = 0
        wrong_code = False
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            x_ready = False
            y_ready = False
            w_ready = False
            if self.new_msg:
                self.new_msg = False
                quat = [self.pose.pose.orientation.x, self.pose.pose.orientation.y,
                        self.pose.pose.orientation.z, self.pose.pose.orientation.w]
                euler = transformations.euler_from_quaternion(quat)

                rospy.loginfo('pose:[%.3f,%.3f,%.3f],yaw:%.3f', self.pose.pose.position.x,
                              self.pose.pose.position.y, self.pose.pose.position.z, euler[2])

                # x y move
                if abs(self.pose.pose.position.x) <= 0.02 and abs(self.pose.pose.position.y) <= 0.04:
                    x_ready = True
                    y_ready = True

                elif abs(self.pose.pose.position.y) >= 0.5 or abs(self.pose.pose.position.x) >= 0.5:
                    x_ready = False
                    continue

                else:
                    goal = CustomMoveGoal()
                    goal.type = 'linear'
                    goal.dist = 0.9 * math.sqrt(self.pose.pose.position.y**2 + self.pose.pose.position.x**2)
                    vx = 0.5 * abs(self.pose.pose.position.y) + 0.025
                    vy = 0.5 * abs(self.pose.pose.position.x) + 0.02
                    goal.vel.linear.x = vx if self.pose.pose.position.y < 0 else -vx
                    goal.vel.linear.y = vy if self.pose.pose.position.x < 0 else -vy
                    goal.vel.angular.z = -0.8 * euler[2]/(goal.dist/math.sqrt(goal.vel.linear.x**2 + goal.vel.linear.y**2)+0.5)
                    self.move_client.send_goal(goal, self.done_callback)
                    self.move_client.wait_for_result(rospy.Duration.from_sec(10))
                    repeat_time = 0
                    x_ready = False
                    rospy.sleep(rospy.Duration(0.5))

                # # x dir move
                # if 0.03 <= abs(self.pose.pose.position.y) < 0.5:
                #     goal = CustomMoveGoal()
                #     goal.type = 'linear'
                #     goal.dist = 0.9 * abs(self.pose.pose.position.y)
                #     if self.pose.pose.position.y > 0:
                #         goal.vel.linear.x = -(0.5 * self.pose.pose.position.y + 0.03)
                #         goal.vel.linear.x = -0.05 if goal.vel.linear.x < -0.05 else goal.vel.linear.x
                #     if self.pose.pose.position.y < 0:
                #         goal.vel.linear.x = 0.5 * -self.pose.pose.position.y + 0.03
                #         goal.vel.linear.x = 0.05 if goal.vel.linear.x > 0.05 else goal.vel.linear.x
                #     self.move_client.send_goal(goal, self.done_callback)
                #     self.move_client.wait_for_result(rospy.Duration.from_sec(10))
                #     repeat_time = 0
                #     x_ready = False
                # elif abs(self.pose.pose.position.y) >= 0.5:
                #     x_ready = False
                #     continue
                # else:
                #     x_ready = True
                # # rospy.sleep(rospy.Duration(0.5))

                # # y dir move
                # if 0.015 <= abs(self.pose.pose.position.x) < 0.5:
                #     goal = CustomMoveGoal()
                #     goal.type = 'linear'
                #     goal.dist = 0.6 * abs(self.pose.pose.position.x)
                #     if self.pose.pose.position.x > 0:
                #         goal.vel.linear.y = -(0.3 * self.pose.pose.position.x + 0.025)
                #         goal.vel.linear.y = -0.05 if goal.vel.linear.y < -0.05 else goal.vel.linear.y
                #     if self.pose.pose.position.x < 0:
                #         goal.vel.linear.y = 0.3 * -self.pose.pose.position.x + 0.025
                #         goal.vel.linear.y = 0.05 if goal.vel.linear.y > 0.05 else goal.vel.linear.y
                #     self.move_client.send_goal(goal, self.done_callback)
                #     self.move_client.wait_for_result(rospy.Duration.from_sec(10))
                #     repeat_time = 0
                #     y_ready = False
                # elif abs(self.pose.pose.position.x) >= 0.5:
                #     y_ready = False
                #     continue
                # else:
                #     y_ready = True
                # # rospy.sleep(rospy.Duration(0.5))

                # turn
                if abs(euler[2]) >= 0.05 and abs(euler[2]) < 0.7:
                    goal = CustomMoveGoal()
                    goal.type = 'angular'
                    goal.dist = 0.9 * abs(euler[2])
                    goal.vel.angular.z = 0.09 if euler[2] < 0 else -0.09
                    self.move_client.send_goal(goal, self.done_callback)
                    self.move_client.wait_for_result(rospy.Duration.from_sec(10))
                    repeat_time = 0
                    w_ready = False
                elif abs(euler[2]) >= 0.7:
                    w_ready = False
                    continue
                else:
                    w_ready = True
                # rospy.sleep(rospy.Duration(0.5))

                if x_ready and y_ready and w_ready:
                    rospy.loginfo('adjust done')
                    repeat_time = repeat_time+1
                    if repeat_time > 1:
                        repeat_time = 0
                        break
                wrong_time = 0
                rospy.sleep(rospy.Duration(0.5))

            else:
                rospy.logwarn("no pattern detected or updated")
                wrong_time = wrong_time+1
                if wrong_time > 20:
                    wrong_time = 0
                    wrong_code = True
                    break

            rate.sleep()

        if wrong_code:
            res.success = False
            res.message = 'incorrect_code'
            return res
        res.success = True
        res.message = 'success'
        return res

    def ar_pose_callback(self, msg):
        self.new_msg = True
        self.pose = msg

    def done_callback(self, state, res):
        rospy.loginfo('action complete')


if __name__ == '__main__':
    rospy.init_node('pose_adjust')
    ArucoAdjust()
    while not rospy.is_shutdown():
        rospy.spin()
