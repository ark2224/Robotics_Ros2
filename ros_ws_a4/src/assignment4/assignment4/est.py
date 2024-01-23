#!/usr/bin/env python3

# Columbia Engineering
# MECS 4603 - Fall 2023

import math
import numpy
import time

import rclpy
from rclpy.node import Node

from state_estimator_msgs.msg import RobotPose
from state_estimator_msgs.msg import SensorData

class Estimator(Node):
    def __init__(self):
        super().__init__('estimator')

        # Publisher to publish state estimate
        self.pub_est = self.create_publisher(RobotPose, "/robot_pose_estimate", 1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3,1))
        self.P = numpy.zeros((3,3))

        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005

        self.step_size = 0.05

        # Subscribe to command input and sensory output of robot
        self.sensor_sub = self.create_subscription(SensorData, "/sensor_data", self.sensor_callback, 1)
        
    
    def estimate(self, sens: SensorData):
        '''This function gets called every time the robot publishes its control 
        input and sensory output. You must make use of what you know about 
        extended Kalman filters to come up with an estimate of the current
        state of the robot and covariance matrix. The SensorData message 
        contains fields 'vel_trans' and 'vel_ang' for the commanded 
        translational and rotational velocity respectively. Furthermore, 
        it contains a list 'readings' of the landmarks the robot can currently
        observe

        Args:
            sens: incoming sensor message
        '''
        
        t = self.step_size
        vt = sens.vel_trans
        va = sens.vel_ang
        
        #kinematic equations
        X_k1 = self.x[0][0] + t*vt*numpy.cos(self.x[2][0])
        Y_k1 = self.x[1][0] + t*vt*numpy.sin(self.x[2][0])
        Theta_k1 = self.x[2][0] + t*va
        x_k1pred = numpy.array([X_k1, Y_k1, Theta_k1]).reshape((3,1))
        
        F_k = numpy.array([[1., 0., -t*vt*numpy.sin(self.x[2])],
                          [0., 1., t*vt*numpy.cos(self.x[2])],
                          [0., 0., 1.]], dtype=numpy.float64)
        P_k1pred = numpy.matmul(F_k, numpy.matmul(self.P, F_k.T)) + self.V
        
        Rnu = numpy.zeros(shape=(3,1))
        RHP = numpy.zeros(shape=(3,3))
        
        # updating with corrections from landmarks
        if sens.readings:
            l_num = len(sens.readings)
            W = numpy.zeros(shape=(2*l_num,2*l_num), dtype=numpy.float64)
            H = numpy.zeros((2*l_num, 3))
            y_k1 = numpy.zeros((2*l_num, 1))
            y_k1pred = numpy.zeros((2*l_num, 1))

            for i in range(l_num):
                l = sens.readings[i]
                W[2*i,2*i] = 0.1
                W[2*i+1,2*i+1] = 0.05
                tmp = (l.landmark.y - Y_k1) / (l.landmark.x - X_k1)
                range_ = math.sqrt( (X_k1-l.landmark.x)*(X_k1-l.landmark.x) + (Y_k1-l.landmark.y)*(Y_k1-l.landmark.y) )
                bearing = math.atan2(l.landmark.y - Y_k1, l.landmark.x - X_k1) - Theta_k1
                if range_ <= 0.1 or l.range <= 0.1:
                    continue

                H[2*i:2+2*i, :] = numpy.array([[(X_k1-l.landmark.x) / range_, (Y_k1-l.landmark.y) / range_, 0.],
                                [(l.landmark.y-Y_k1) / ((l.landmark.x-X_k1)**2 * (1.+tmp**2)), -1. / ((l.landmark.x-X_k1) * (1.+tmp**2)), -1.]], dtype=numpy.float64)

                y_k1[2*i:2+2*i,:] = numpy.array([l.range, l.bearing], dtype=numpy.float64).reshape((2,1))
                y_k1pred[2*i:2+2*i,:] = numpy.array([range_, bearing], dtype=numpy.float64).reshape((2,1))
                
            S = numpy.matmul(H, numpy.matmul(P_k1pred, H.T, dtype=numpy.float64), dtype=numpy.float64) + W
            R = numpy.matmul(P_k1pred, numpy.matmul(H.T, numpy.linalg.inv(S)))
            bearing = math.atan2(l.landmark.y - Y_k1, l.landmark.x - X_k1) - Theta_k1

            nu = y_k1 - y_k1pred

                
            while abs(nu[1][0]) > numpy.pi:
                if nu[1][0] < -numpy.pi:
                    nu[1][0] += 2*numpy.pi
                else:
                    nu[1][0] -= 2*numpy.pi
            Rnu = numpy.matmul(R,nu, dtype=numpy.float64)
            RHP = -numpy.matmul(R, numpy.matmul(H, P_k1pred, dtype=numpy.float64), dtype=numpy.float64)

        x_k1 = x_k1pred + Rnu
        self.x = x_k1
        self.P = P_k1pred + RHP
        RP = RobotPose()
        RP.pose.x = x_k1[0][0]
        RP.pose.y = x_k1[1][0]
        RP.pose.theta = x_k1[2][0]
        self.pub_est.publish(RP)
        
        
    
    def sensor_callback(self,sens):

        # Publish state estimate 
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = float(self.x[0])
        est_msg.pose.y = float(self.x[1])
        est_msg.pose.theta = float(self.x[2])
        self.pub_est.publish(est_msg)

def main(args=None):
    rclpy.init(args=args)   
    est = Estimator()
    rclpy.spin(est)
                
if __name__ == '__main__':
   main()

 
