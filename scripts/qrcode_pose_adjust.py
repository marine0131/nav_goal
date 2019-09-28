#! /usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
# from visp_tracker.msg import MovingEdgeSites, KltPoints
from nav_goal.msg import CustomMoveAction, CustomMoveGoal
from tf import transformations
from std_msgs.msg import String
from nav_goal.srv import QRcodeAdjust, QRcodeAdjustResponse


class QRcode():
    def __init__(self, ):
        self.code = String()
        self.pose = PoseStamped()

        rospy.Subscriber('/visp_auto_tracker/code_message', String, self.code_callback)
        rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, self.pose_callback)
        self.move_client = actionlib.SimpleActionClient('nav_goal/custom_move_action', CustomMoveAction)
        rospy.Service('/nav_goal/qrcode_adjust', QRcodeAdjust, self.handle_qrcode_adjust)

        rospy.loginfo('waiting for server')
        self.move_client.wait_for_server()
        rospy.loginfo('server connectted')

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
            if self.code == req.code_message:
                quat = [self.pose.pose.orientation.x, self.pose.pose.orientation.y,
                        self.pose.pose.orientation.z, self.pose.pose.orientation.w]
                euler = transformations.euler_from_quaternion(quat)

                rospy.loginfo('pose:[%.3f,%.3f,%.3f],yaw:%.3f', self.pose.pose.position.x,
                              self.pose.pose.position.y, self.pose.pose.position.z, euler[2])
               if abs(euler[2]) >= 0.05 and abs(euler[2]) < 0.7:
                    goal = CustomMoveGoal()
                    goal.type = 'angular'
                    goal.dist = 0.9 * abs(euler[2])
                    goal.vel.angular.z = 0.1 if euler[2] < 0 else -0.1
                    self.move_client.send_goal(goal, self.done_callback)
                    self.move_client.wait_for_result(rospy.Duration.from_sec(10))
                    repeat_time = 0
                elif abs(euler[2]) >= 0.7:
                    continue
                else:
                    w_ready = True
                # rospy.sleep(rospy.Duration(0.5))

                # x dir move
                if 0.02 <= abs(self.pose.pose.position.y) < 0.5:
                    goal = CustomMoveGoal()
                    goal.type = 'linear'
                    goal.dist = 0.9 * abs(self.pose.pose.position.y)
                    goal.vel.linear.x = -0.05 if self.pose.pose.position.y > 0 else 0.05
                    self.move_client.send_goal(goal, self.done_callback)
                    self.move_client.wait_for_result(rospy.Duration.from_sec(10))
                    repeat_time = 0
                elif abs(self.pose.pose.position.y) >= 0.5:
                    continue
                else:
                    x_ready = True
                # rospy.sleep(rospy.Duration(0.5))

                # y dir move
                if 0.02 <= abs(self.pose.pose.position.x) < 0.5:
                    goal = CustomMoveGoal()
                    goal.type = 'linear'
                    goal.dist = 0.6 * abs(self.pose.pose.position.x)
                    goal.vel.linear.y = -0.05 if self.pose.pose.position.x > 0 else 0.05
                    self.move_client.send_goal(goal, self.done_callback)
                    self.move_client.wait_for_result(rospy.Duration.from_sec(10))
                    repeat_time = 0
                elif abs(self.pose.pose.position.x) >= 0.5:
                    continue
                else:
                    y_ready = True
                # rospy.sleep(rospy.Duration(0.5))

                if x_ready and y_ready and w_ready:
                    rospy.loginfo('adjust done')
                    repeat_time = repeat_time+1
                    if repeat_time > 1:
                        repeat_time = 0
                        break
                wrong_time = 0

            else:
                rospy.logwarn("wrong code: %s", self.code)
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

    def reach_goal_callback(self, msg):
        if msg.data == 'done':
            self.start_move = True

    def code_callback(self, msg):
        self.code = msg.data

    def pose_callback(self, msg):
        self.pose = msg

    def done_callback(self, state, res):
        rospy.loginfo('action complete')


if __name__ == '__main__':
    rospy.init_node('pose_adjust')
    QRcode()
    while not rospy.is_shutdown():
        rospy.spin()
