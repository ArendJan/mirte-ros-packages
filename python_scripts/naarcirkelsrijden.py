#!/usr/bin/env python

import rospy
import time
import os
import numpy as np
import tf_conversions
import math
import copy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


class BEP_test:
    def __init__(self, car_number):
        self.car_number = car_number
        rospy.init_node('BEP_test_' + str(car_number), anonymous=True)

        # Setup topics publishing and nodes
        self.pub_throttle = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size=8)
        self.pub_steering = rospy.Publisher('steering_' + str(car_number), Float32, queue_size=8)
        # also publishing safety value
        self.pub_safety_value = rospy.Publisher('safety_value', Float32, queue_size=8)

        # setup subscriber for the odometry
        rospy.Subscriber('odom_' + str(car_number), Odometry, self.odom_callback)

        # PD-controller
        self.Kp = 1.5
        self.Kd = 0.5
        self.target_yaw = 1.0
        self.max_yaw_rate = 0.5
        self.last_time = 0.0

        # initialize odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.w = 0.0

        # initialize vicon
        rospy.Subscriber('/vrpn_client_node/jetracer2/pose', PoseStamped, self.vicon_callback)

        # initialize vicon variables
        self.vicon_x = 0.0
        self.vicon_y = 0.0
        self.vicon_theta = 0.0

        # keeping track of first position
        self.first_vicon_x = 0.0
        self.first_vicon_y = 0.0
        self.first_vicon_theta = 0.0
        self.last_vicon_msg_recieved = float('inf')
        self.vicon_received = False

        self.i = 0
        self.c_relative = [[2, -1], [5, 1], [1, 0], [5, -2], [3, 1], [3, -2], [5, -2], [0,0]]
        self.c = copy.deepcopy(self.c_relative) # gets changed later
        self.reached_pos_odo = []
        self.reached_pos_vicon = []

        self.c_x = self.c[self.i][0]
        self.c_y = self.c[self.i][1]
        self.r = 0.1
        self.d_target = float('inf')
        self.d_theta = 0.0


    def run_test(self):
        self.pub_safety_value.publish(1)
        print("Safety value sent")


        stop = False
        while not rospy.is_shutdown() and not stop:
            # startup sequence for calibrating steering
            rospy.sleep(0.5)
            self.pub_steering.publish(-1.0)
            rospy.sleep(0.5)
            self.pub_steering.publish(0)

            if self.vicon_received:
                for self.i in range(len(self.c)):
                    target = self.c[self.i]
                    print("Target", target)
                    self.drive_to_target(target)
                stop = True

        rospy.sleep(2)

        print("\nLast odometry position: \nx:" + str(self.x) + "\ny:" + str(self.y) + "\ntheta:" + str(self.theta))

        print("\nFirst Vicon position: \nx:" + str(self.first_vicon_x) + "\ny:" + str(self.first_vicon_y) + "\ntheta:" + str(self.first_vicon_theta))
        print("\nLast Vicon position: \nx:" + str(self.vicon_x) + "\ny:" + str(self.vicon_y) + "\ntheta:" + str(self.vicon_theta))

        # compute vicon difference
        diff_x = self.vicon_x - self.first_vicon_x
        diff_y = self.vicon_y - self.first_vicon_y
        diff_theta = self.vicon_theta - self.first_vicon_theta

        print("\nVicon relative difference: \nx:" + str(diff_x) + "\ny:" + str(diff_y) + "\ntheta:" + str(diff_theta))

        # compute difference between points
        print("Vicon target positions\n", self.c_relative)
        print("Measured odometry target reached pos")
        for i in range(len(self.reached_pos_odo)):
            print(str(i) + ": " + str(self.reached_pos_odo[i]))
        print("Measured vicon target reached pos")
        for i in range(len(self.reached_pos_vicon)):
            print(str(i) + ": " + str(self.reached_pos_vicon[i]))

        
        





    def drive_to_target(self, target):
        self.c_x = target[0]
        self.c_y = target[1]

        self.d_target = math.sqrt((self.c_y - self.vicon_y)**2+(self.c_x - self.vicon_x)**2)

        # init time variables
        rate = rospy.Rate(50)
        start_time = rospy.Time.now().to_sec()

        # loop variables
        self.last_time = rospy.Time.now().to_sec()

        while self.d_target > self.r and not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            dt = current_time - self.last_time
            self.last_time = current_time

            # update distances
            self.d_target = math.sqrt((self.c_y - self.vicon_y)**2+(self.c_x - self.vicon_x)**2)
            self.theta_c = math.atan2((self.c_y - self.vicon_y),(self.c_x - self.vicon_x))
            self.d_theta = math.atan2(
                math.sin(self.theta_c - self.vicon_theta),
                math.cos(self.theta_c - self.vicon_theta)
                    )

            # default to 50hz in case dt doesn't update correctly
            if dt <= 0:
                dt = 0.02

            print("Distance", self.d_target)

            if abs(self.d_target) > 0.5: # only apply PD-controller if vehicle actually drives
                throttle = 0.22
                p_term = self.Kp * self.d_theta
                # d_term = -self.Kd * self.w
                steering_corr = p_term #+ d_term
            elif abs(self.d_target) <= 0.5: 
                throttle = 0.18
                p_term = self.Kp * self.d_theta
                # d_term = -self.Kd * self.w
                steering_corr = p_term #+ d_term

                        
            steering_corr = max(-1.0, min(steering_corr, 1.0))

            self.pub_steering.publish(steering_corr)
            self.pub_safety_value.publish(1)

            # print("Publishing throttle: " + str(throttle))
            self.pub_throttle.publish(throttle)

            rate.sleep()

        print("We zijn er!")

        current_odometry_location = [self.x, self.y]
        self.reached_pos_odo.append(current_odometry_location)

        current_vicon_location = [self.vicon_x - self.first_vicon_x, self.vicon_y - self.first_vicon_y]
        self.reached_pos_vicon.append(current_vicon_location)

        self.pub_throttle.publish(0.0)
        self.pub_steering.publish(0.0)
            



    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.zyx_euler_angles = tf_conversions.transformations.euler_from_quaternion(q, 'rzyx')
        theta = self.zyx_euler_angles[0]
        
        w = msg.twist.twist.angular.z

        self.x, self.y, self.theta, self.w = x, y, theta, w


    def vicon_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.zyx_euler_angles = tf_conversions.transformations.euler_from_quaternion(q, 'rzyx')
        theta = self.zyx_euler_angles[0]

        time_now = msg.header.stamp.to_sec()

        if time_now < self.last_vicon_msg_recieved:
            self.first_vicon_x = x
            self.first_vicon_y = y
            self.first_vicon_theta = theta
            self.vicon_received = True

            print("Vicon received!")

            # make target coordinates circles relative to start of robot
            for i in range(len(self.c_relative)):
                self.c[i][0] += self.first_vicon_x
                self.c[i][1] += self.first_vicon_y
            
            self.last_vicon_msg_recieved = time_now
        
        self.vicon_x = x 
        self.vicon_y = y
        self.vicon_theta = theta
        self.vicon_state = [x, y, theta]
    

if __name__ == '__main__':
    try:
        try:
            car_number = os.environ['car_number']
        except KeyError:
            car_number = 1  # default to 1 if env var is not set

        bep_test_obj = BEP_test(car_number)
        bep_test_obj.run_test()



    except rospy.ROSInterruptException:
        pass