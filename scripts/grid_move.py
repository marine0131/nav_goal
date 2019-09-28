#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
import math
from nav_goal.srv import GridMove, GridMoveResponse, GridMoveRequest, QRcodeAdjust, QRcodeAdjustRequest, StorageAvoidance, StorageAvoidanceRequest
from tf import transformations


class StorageMovement():
    def __init__(self):
        self.ar_pose = PoseStamped()
        self.robot_pose = PoseStamped()
        self.new_aruco = False
        self.max_vel = rospy.get_param("~max_vel", 0.6)

        rospy.Subscriber('/robot_pose', PoseStamped, self.robot_pose_callback)
        rospy.Subscriber('/ar_single_board/pose', PoseStamped, self.ar_pose_callback)
        rospy.Service('nav_goal/grid_move', GridMove, self.handle_grid_move)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.adjust_client = rospy.ServiceProxy('nav_goal/qrcode_adjust', QRcodeAdjust)
        self.avoid_client = rospy.ServiceProxy('nav_goal/storage_avoidance', StorageAvoidance)
        # for test
        rospy.sleep(rospy.Duration(2.0))
        req = GridMoveRequest()
        req.dir = 1
        grid_move_client = rospy.ServiceProxy('nav_goal/grid_move', GridMove)
        while not rospy.is_shutdown():
            res = grid_move_client.call(req)
            req.dir = -req.dir
            rospy.loginfo(res.message)
            if not res.success:
                break

    def ar_pose_callback(self, msg):
        self.ar_pose = msg
        self.new_aruco = True

    def robot_pose_callback(self, msg):
        self.robot_pose = msg

    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1.pose.position.x - point2.pose.position.x)**2
                         + (point1.pose.position.y - point2.pose.position.y)**2)

    def SIGN(self, num):
        return 1 if num > 0 else -1

    def set_target_pose(self, t_name):
        target = PoseStamped()
        if t_name == "forward":
            target.pose.position.x = -6.90434507675
            target.pose.position.y = 2.90670866642
            target.pose.orientation.x = 0.0
            target.pose.orientation.y = 0.0
            target.pose.orientation.z = -0.8517705237
            target.pose.orientation.w = 0.523905657
        elif t_name == "backward":
            target.pose.position.x = -3.82552711509
            target.pose.position.y = 9.15885317831
            target.pose.orientation.x = 0.0
            target.pose.orientation.y = 0.0
            target.pose.orientation.z = -0.83166547
            target.pose.orientation.w = 0.5551722

        return target

    def handle_grid_move(self, req):
        res = GridMoveResponse()
        t_name = "forward" if req.dir > 0 else "backward"
        target_pose = self.set_target_pose(t_name)
        freq = 18.0  # calculate frequency
        # L = 0.28  # window size in x direction
        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            vel_msg = Twist()
            curr_pose = self.robot_pose
            target_dist = self.euclidean_distance(curr_pose, target_pose)
            if target_dist < 0.1 or (target_dist < 0.4 and self.new_aruco):
                self.vel_pub.publish(vel_msg)
                adjust_req = QRcodeAdjustRequest()
                adjust_req.code_message = 'whatever'
                adjust_res = self.adjust_client.call(adjust_req)
                break

            else:
                vx = target_dist * 0.6 + 0.02
                vx = req.dir * self.max_vel if vx > self.max_vel else req.dir * vx

                if self.new_aruco:  # new aruco code
                    self.new_aruco = False
                    euler = transformations.euler_from_quaternion([self.ar_pose.pose.orientation.x,
                                                                   self.ar_pose.pose.orientation.y,
                                                                   self.ar_pose.pose.orientation.z,
                                                                   self.ar_pose.pose.orientation.w])  # euler camera tf
                    # l_left = L/2.0 - req.dir*self.ar_pose.pose.position.y
                    # vy = -self.ar_pose.pose.position.x / (l_left / vx) * 0.3
                    # w = -euler[2] / (l_left / vx) * 0.3
                    # rospy.loginfo("%f,%f,%f\n",l_left,vy,w);
                    vy = -self.ar_pose.pose.position.x * 2.0
                    w = -euler[2] * 1.2
                    vel_msg.linear.x = vx
                    vel_msg.linear.y = self.SIGN(vy) * self.max_vel if abs(vy) > self.max_vel else vy
                    vel_msg.angular.z = self.SIGN(w) * 1.0 if abs(w) > 1.0 else w
                else:
                    avoid_req = StorageAvoidanceRequest()
                    avoid_req.dir = req.dir
                    avoid_req.vx = vx
                    avoid_res = self.avoid_client.call(avoid_req)
                    vel_msg = avoid_res.vel

                self.vel_pub.publish(vel_msg)

            rate.sleep()
        res.success = adjust_res.success
        res.message = adjust_res.message
        return res


if __name__ == '__main__':
    rospy.init_node("grid_move")
    StorageMovement()
