#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_goal.srv import StorageAvoidance, StorageAvoidanceResponse
import math
import copy


class ObstacleAvoidance():
    def __init__(self):
        scan_front_topic = rospy.get_param("~scan_front", "scan_front")
        scan_back_topic = rospy.get_param("~scan_back", "scan_back")
        # vel_topic = rospy.get_param("~vel_topic", "cmd_vel")

        self.params_config()

        self.scan_msg = LaserScan()
        rospy.Subscriber(scan_front_topic, LaserScan, self.scan_front_callback)
        rospy.Subscriber(scan_back_topic, LaserScan, self.scan_back_callback)
        rospy.Service("nav_goal/storage_avoidance", StorageAvoidance, self.handle_avoidance)
        # self.vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=1)
        # rospy.sleep(rospy.Duration(1.0))
        # rate = rospy.Rate(3)
        while not rospy.is_shutdown():
            rospy.spin()
            # self.avoidance()
            # rate.sleep()

    def scan_front_callback(self, msg):
        self.scan_front = copy.deepcopy(msg)

    def scan_back_callback(self, msg):
        self.scan_back = copy.deepcopy(msg)

    def params_config(self):
        self.scan_frame = rospy.get_param("~frame", "laser")
        self.draw_section = rospy.get_param("~draw_section", "True")

        self.danger_width = rospy.get_param("~danger_width", 0.5)
        self.adjust_width = rospy.get_param("~adjust_width", 1.0)
        self.danger_count = rospy.get_param("~danger_count", 5)

        # self.R = rospy.get_param("~forward_looking", 2.0)
        # self.D = rospy.get_param("~robot_radius", 0.5)
        # self.Padding = rospy.get_param("~robot_padding", 1.2)
        # self.L_T = rospy.get_param("~arc_thresh", 0.8)
        # self.K_W = rospy.get_param("~angular_velocity_coef", 1.0)
        # self.R_MAX = rospy.get_param("~forward_looking_max", 4.0)
        # self.W1 = rospy.get_param("~w1", 0.65)
        # self.W2 = rospy.get_param("~w2", 0.35)
        # self.R0 = rospy.get_param("~forward_looking_min", 0.5)
        # self.V_MAX = rospy.get_param("~vx_max", 0.6)
        # self.V_MIN = rospy.get_param("~vx_min", 0.05)
        # self.angle_start = rospy.get_param("~angle_start", -90.0)
        # self.angle_stop = rospy.get_param("~angle_stop", 90.0)
        # self.angle_increment = rospy.get_param("~angle_increment", 2.5)

    def handle_avoidance(self, req):
        res = StorageAvoidanceResponse()
        respect_angle = 160
        angle_start = math.radians(-respect_angle/2.0)
        angle_stop = math.radians(respect_angle/2.0)

        if req.vx > 0:
            ranges = self.laser_filter(list(self.scan_front.ranges))
            pp = self.cartesian_transformation(ranges,
                                               self.scan_front.angle_min,
                                               self.scan_front.angle_increment,
                                               [angle_start, angle_stop])
        else:
            ranges = self.laser_filter(list(self.scan_back.ranges))
            pp = self.cartesian_transformation(ranges,
                                               self.scan_back.angle_min,
                                               self.scan_back.angle_increment,
                                               [angle_start, angle_stop])
        danger_thresh = abs(req.vx) * 1.5  # set danger thresh dynamic with vx
        danger_count = 0
        right_bound = [-self.danger_width/2.0, -self.adjust_width/2.0]
        left_bound = [self.danger_width/2.0, self.adjust_width/2.0]
        danger_bound = [-self.danger_width/2.0, self.danger_width/2.0]
        left_list = []
        right_list = []
        danger_flag = False

        for p in pp:
            if p[0] < danger_thresh:
                if danger_bound[0] < p[1] < danger_bound[1]:
                    danger_count = danger_count + 1
                    if danger_count > self.danger_count:
                        danger_flag = True
                        break
                elif left_bound[0] < p[1] < left_bound[1]:
                    left_list.append(p)
                elif right_bound[1] < p[1] < right_bound[0]:
                    right_list.append(p)
                else:
                    continue
        if danger_flag:
            res.message = 'danger'
            res.success = True
            return res
        left_list = np.array(left_list)
        right_list = np.array(right_list)
        mv_dir = 1 if req.vx > 0 else -1
        if len(left_list) > 5 and len(right_list) > 5:
            res.vel.linear.x = req.vx
            res.message = "clear"
        elif len(left_list) > 5:
            res.vel.linear.x = mv_dir * np.min(left_list[:, 0]) / 1.5
            res.vel.linear.y = mv_dir * (np.min(left_list[:, 1]) - left_bound[1]) / 1.5
            res.message = "left"
        elif len(right_list) > 5:
            res.vel.linear.x = mv_dir * np.min(np.abs(right_list[:, 0])) / 1.5
            res.vel.linear.y = mv_dir * (np.max(right_list[:, 1]) - right_bound[1]) / 1.5
            res.message = "right"
        else:
            res.message = "clear"
            res.vel.linear.x = req.vx

        res.success = True
        rospy.loginfo(res.vel)
        return res

    def avoidance(self):
        theta = self.scan_msg.angle_increment
        angle_min = self.scan_msg.angle_min
        self.scan_msg.ranges = self.laser_filter(list(self.scan_msg.ranges))
        sec_angle_min = math.radians(self.angle_start)
        sec_angle_increment = math.radians(self.angle_increment)
        sec_ranges_num = int(sec_angle_increment / theta)
        sec_num = int(math.pi/sec_angle_increment)

        # calculate sections and mark if cadidate
        section = []
        candidate = []
        pp = self.cartesian_transformation(self.scan_msg.ranges,
                                           self.scan_msg.angle_min,
                                           self.scan_msg.angle_increment,
                                           [math.radians(self.angle_start),
                                            math.radians(self.angle_stop)])
        for i in range(sec_num):
            start_angle = sec_angle_min + sec_angle_increment * i
            start_index = int((start_angle - angle_min) / theta)
            d = min(self.scan_msg.ranges[start_index: start_index + sec_ranges_num])
            danger_flag = False
            ang = start_angle + sec_angle_increment/2.0
            for p in pp:
                dd = abs(p[1] - p[0] * math.tan(ang)) * math.cos(ang)
                # point is in danger zone, remove section and break
                if 0 < dd < self.D * self.Padding / 2 and math.sqrt(p[0]**2 + p[1]**2) < self.R:
                    danger_flag = True
                    break
            # get a section with [ start_angle, min_dist]
            section.append([start_angle, d])
            if d > self.R and not danger_flag:
                candidate.append(True)
            else:
                candidate.append(False)

        # merge sections
        Z = []
        j = 0
        last_val = False
        for i, c in enumerate(candidate):
            if c:
                if not last_val:
                    Z.append([self.R, section[i][0], i, i])

                if i == len(candidate)-1:
                    Z[j][1] = (Z[j][1] + section[i][0] + sec_angle_increment)/2.0
                    Z[j][3] = i
            else:
                if last_val:
                    # get a section, calculate section center angle
                    Z[j][1] = (Z[j][1] + section[i][0] + sec_angle_increment)/2.0
                    Z[j][3] = i
                    j = j+1
            last_val = c

        # find accessable section
        j = 0
        for i in range(len(Z)):
            arc = self.R * (Z[j][3]-Z[j][2]+1) * sec_angle_increment
            if arc < self.L_T:
                Z.remove(Z[j])
            else:
                j = j + 1

        # draw accessable section
        if self.draw_section:
            self.mark_section(Z, section)

        # find best section
        Z = np.array(Z)
        vel_msg = Twist()
        if len(Z) > 0:
            [r_s, optimal_ang, start, stop] = Z[np.fabs(Z[:, 1]).argmin()] if len(Z) > 1 else Z[0]
            theta_e = math.atan((self.target_pose.pose.position.y - self.robot_pose.pose.position.y) /
                                (self.target_pose.pose.position.x - self.robot_pose.pose.position.x))
            # if target pose locate in the optimal move section
            if section[int(start)][0] < theta_e < section[int(stop)][0] and abs(theta_e) < abs(optimal_ang):
                optimal_ang = theta_e

            vel_msg.angular.z = optimal_ang * self.K_W

            x_m = self.W1 * math.pi / 2 + self.W2 * (1 - self.R0 / self.R_MAX)
            k_v = -(self.V_MAX - self.V_MIN) / x_m
            x_v = self.W1 * optimal_ang + self.W2 * (self.R0 / self.R - self.R0 / self.R_MAX)
            vel_msg.linear.x = k_v * x_v + self.V_MAX

        rospy.loginfo("vx=%.4f, w=%.4f", vel_msg.linear.x, vel_msg.angular.z)
        # self.vel_pub.publish(vel_msg)

    def laser_filter(self, ranges):
        for i, r in enumerate(ranges):
            if r < 0.01:
                ranges[i] = self.scan_msg.range_max
            elif i > 0 and i < len(ranges) - 1:
                if (abs(r - ranges[i-1]) > 0.3 and abs(r - ranges[i+1]) > 0.3
                    and abs(r - self.scan_msg.range_max) > 0.3):
                    ranges[i] = self.scan_msg.range_max
        return ranges

    # transform ranges to cartesian x y and change to numpy array
    def cartesian_transformation(self, ranges, angle_min, angle_increment, limit):
        pnts = []
        start_ind = int((limit[0] - angle_min)/angle_increment)
        stop_ind = int((limit[1] - angle_min)/angle_increment)
        for i, r in enumerate(ranges[start_ind: stop_ind]):
            if not r == 0:
                pnts.append([r*math.cos(angle_min + (start_ind + i) * angle_increment),
                             r*math.sin(angle_min + (start_ind + i) * angle_increment)])
        return np.array(pnts)

    def euclidean_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)


if __name__ == '__main__':
    rospy.init_node("obstacle_avoidance")
    ObstacleAvoidance()
