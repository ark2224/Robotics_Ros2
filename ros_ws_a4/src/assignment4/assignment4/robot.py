#!/usr/bin/env python3

# Columbia Engineering
# MECS 4603 - Fall 2023

import numpy
import time
import rclpy
from rclpy.node import Node

import random

from state_estimator_msgs.msg import RobotPose
from state_estimator_msgs.msg import SensorData
from state_estimator_msgs.msg import Landmark
from state_estimator_msgs.msg import LandmarkReading
from state_estimator_msgs.msg import LandmarkSet

def create_landmark(x, y):
   l = Landmark()
   l.x = float(x)
   l.y = float(y)
   return l

class Robot(Node):

    def __init__(self):
        super().__init__('robot')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.vt = 0.0
        self.vrot = 0.0

        self.step_size = 0.05

        self.model_noise_trans = 0.0025
        self.model_noise_rot = 0.005

        self.sensor_noise_range = 0.1
        self.sensor_noise_bearing = 0.05

        self.start_flag = False

        self.landmarks = []
        self.landmarks.append(create_landmark(5,5))
        self.landmarks.append(create_landmark(5,6))
        self.landmarks.append(create_landmark(6,5))        
        self.landmarks.append(create_landmark(-5,5))
        self.landmarks.append(create_landmark(-5,6))
        self.landmarks.append(create_landmark(-6,5))        
        self.landmarks.append(create_landmark(5,-5))
        self.landmarks.append(create_landmark(5,-6))
        self.landmarks.append(create_landmark(6,-5))        
        self.landmarks.append(create_landmark(-5,-5))
        self.landmarks.append(create_landmark(-5,-6))
        self.landmarks.append(create_landmark(-6,-5))        
        self.landmarks.append(create_landmark(5,0))
        self.landmarks.append(create_landmark(-5,0))
        self.landmarks.append(create_landmark(0,5))
        self.landmarks.append(create_landmark(0,-5))
        self.landmarks.append(create_landmark(1,0))

        self.sensing_range = 2.5

        self.pub_pose = self.create_publisher(RobotPose, "/robot_pose", 1)
        self.pub_sens = self.create_publisher(SensorData, "/sensor_data", 1)
        self.pub_landmarks = self.create_publisher(LandmarkSet, "/landmarks", 1)

        self.publish_timer = self.create_timer(1.0, self.publish_landmarks)
        self.step_timer = self.create_timer(self.step_size, self.step)

        time.sleep(1)

        self.vel_timer = self.create_timer(3.0, self.rand_vel)
        self.start_flag = True
        self.get_logger().info('Robot running!')

    def get_sensor_data(self):
        sens = SensorData()
        sens.vel_trans = self.vt
        sens.vel_ang = self.vrot
        
        for i in range(0,len(self.landmarks)):
            r = numpy.sqrt( (self.landmarks[i].x-self.x)*(self.landmarks[i].x-self.x) + 
                           (self.landmarks[i].y-self.y)*(self.landmarks[i].y-self.y) )
            if r < self.sensing_range:
                reading = LandmarkReading()
                reading.landmark = self.landmarks[i]
                reading.range = r
                reading.bearing = numpy.arctan2( (self.landmarks[i].y - self.y),
                                              (self.landmarks[i].x - self.x)) - self.theta

                if self.start_flag:
                    reading.range += numpy.random.normal(0.0, self.sensor_noise_range)
                    reading.bearing += numpy.random.normal(0.0, self.sensor_noise_bearing)

                sens.readings.append(reading)

        return sens

    def step(self):
        self.x = self.x + self.step_size * self.vt * numpy.cos(self.theta)
        self.y = self.y + self.step_size * self.vt * numpy.sin(self.theta)
        self.theta = self.theta + self.step_size * self.vrot

        if self.start_flag:
            self.x += numpy.random.normal(0.0, self.model_noise_trans)
            self.y += numpy.random.normal(0.0, self.model_noise_trans)
            self.theta += numpy.random.normal(0.0, self.model_noise_rot)

        time = self.get_clock().now().to_msg()
        pose_msg = RobotPose()
        pose_msg.header.stamp = time
        pose_msg.pose.x = float(self.x)
        pose_msg.pose.y = float(self.y)
        pose_msg.pose.theta = float(self.theta)
        self.pub_pose.publish(pose_msg)

        sensor_msg = self.get_sensor_data()
        sensor_msg.header.stamp = time
        self.pub_sens.publish(sensor_msg)

    def publish_landmarks(self):
        msg = LandmarkSet()
        msg.landmarks = self.landmarks
        self.pub_landmarks.publish(msg)

    def rand_vel(self):
        r = numpy.sqrt(self.x*self.x + self.y*self.y)
        if numpy.fabs(self.x) < 6 and numpy.fabs(self.y) < 6:
            self.vt = 0.5 + random.random() * 1.0
            self.vrot = (-numpy.pi + random.random() * 2 * numpy.pi) / 5
        else:
            if ((self.x * numpy.cos(self.theta) + self.y * numpy.sin(self.theta)) / r < -0.2):
                self.vt = 0.5 + random.random() * 1.0
                self.vrot = 0.0
            else:
                self.vt = 0.0
                self.vrot = (-numpy.pi + random.random() * 2 * numpy.pi) / 2

def main(args=None):
    rclpy.init(args=args)   
    robot = Robot()
    rclpy.spin(robot)
                
if __name__ == '__main__':
   main()
