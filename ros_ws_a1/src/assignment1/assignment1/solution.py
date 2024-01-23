"""
Publishes following transforms at a rate of >=10Hz:
base_T_object: transform from base_frame to object_frame
base_T_robot: transform from base_frame to robot_frame
robot_T_camera: transform from robot_frame to camera_frame
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import visualization_msgs

import transforms3d as t3d
import numpy as np
import tf2_ros as tf

class SolutionPublisher(Node):

    def __init__(self):
        super().__init__('solution_publisher')
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        base_T_object = tf.TransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'object_frame'
        t.child_frame_id = 'base_frame'
        T_rot1 = t3d.axangles.axangle2aff((1, 0, 0), 0.64)
        T_rot2 = t3d.axangles.axangle2aff((0, 1, 0), 0.64)
        T_trans = t3d.affines.compose((1.5, 0.8, 0), np.eye(3,3), (1, 1, 1))
        
        T, R, _, _ = t3d.affines.decompose(np.dot(T_trans, np.dot(T_rot1, T_rot2)))
        q = t3d.quaternions.mat2quat(R)
        
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]
        t.transform.translation.x = T[0]
        t.transform.translation.y = T[1]
        t.transform.translation.z = T[2]
        
        base_T_object.sendTransform(t)

######################################################

        object_T_base = tf.TransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_frame'
        t.child_frame_id = 'object_frame'
        
        T_rot1 = t3d.axangles.axangle2aff((1, 0, 0), 0.64)
        T_rot2 = t3d.axangles.axangle2aff((0, 1, 0), 0.64)
        T_trans = t3d.affines.compose((1.5, 0.8, 0), np.eye(3,3), (1, 1, 1))
        T, R, _, _ = t3d.affines.decompose(np.dot(np.dot(T_rot1, T_rot2), T_trans))
        q = t3d.quaternions.mat2quat(R)
                
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]
        t.transform.translation.x = T[0]
        t.transform.translation.y = T[1]
        t.transform.translation.z = T[2]
        object_T_base.sendTransform(t)

######################################################
        
        base_T_robot = tf.TransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'robot_frame'
        t.child_frame_id = 'base_frame'
        T_rot = t3d.axangles.axangle2aff((0, 1, 0), -1.5)
        T_trans = t3d.affines.compose((0, 0, 2.0), np.eye(3,3), (1, 1, 1))
        
        T, R, _, _ = t3d.affines.decompose(np.dot(T_trans, T_rot))
        q = t3d.quaternions.mat2quat(R)
        
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]
        t.transform.translation.x = T[0]
        t.transform.translation.y = T[1]
        t.transform.translation.z = T[2]
        base_T_robot.sendTransform(t)

######################################################        
        
        robot_T_camera = tf.TransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_frame'
        t.child_frame_id = 'robot_frame'
        
        T_trans = t3d.affines.compose((-0.3, 0, -0.3), np.eye(3,3), (1, 1, 1))
        baseTrob = np.dot(t3d.affines.compose((0, 0, 2.0), np.eye(3,3), (1, 1, 1)), t3d.axangles.axangle2aff((0, 1, 0), -1.5))

        robTbase = np.copy(np.linalg.inv(baseTrob))
        R_ = np.copy(robTbase[:-1, :-1])

        base_to_camera_vector = np.matmul(robTbase, np.array([0.3, 0.0, 0.3, 1]).T)
        print('base_to_camera_vector: ', base_to_camera_vector)
        
        base_to_camera_vector = np.array([[base_to_camera_vector[0]], [base_to_camera_vector[1]], [base_to_camera_vector[2]], [base_to_camera_vector[3]]])  #correct
        T_rot1 = t3d.axangles.axangle2aff((1, 0, 0), 0.64)
        T_rot2 = t3d.axangles.axangle2aff((0, 1, 0), 0.64)

        global_vector_camera_to_object = (np.matmul(np.dot(T_rot1, T_rot2), np.array([[1.5], [0.8], [0], [1]])) - base_to_camera_vector)
        print('global_vector_camera_to_object: ', global_vector_camera_to_object)
        camera_vector_camera_to_object = (np.matmul(np.dot(T_trans, baseTrob), global_vector_camera_to_object))

        v1 = camera_vector_camera_to_object / np.linalg.norm(camera_vector_camera_to_object)
        v1 = v1[:-1].T

        v2 = np.array([1.0, 0.0, 0.0])
        print('v1: ', v1)
        print('v2: ', v2)

        s = np.cross(v1, v2)
        s = s / np.linalg.norm(s)
        c = np.dot(v1, v2.T)

        Z1, Z2, Z3 = s[0]
        h = 1 / (1 + c)
        V = np.array([[0, -Z3, Z2, 0],
                     [Z3, 0, -Z1, 0],
                     [-Z2, Z1, 0, 0],
                     [0, 0, 0, 0]])
        rotation_between_vector_robot_orientation = np.eye(4, dtype=np.float64) + V + (V.dot(V) * h)

        s = s[0]
        
        T, R, _, _ = t3d.affines.decompose(np.dot(rotation_between_vector_robot_orientation, T_trans))

        q = t3d.quaternions.mat2quat(R)

        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]
        t.transform.translation.x = T[0]
        t.transform.translation.y = T[1]
        t.transform.translation.z = T[2]
        robot_T_camera.sendTransform(t)



def main(args=None):
    rclpy.init(args=args)
    sol = SolutionPublisher()
    print('Running Solution Publisher')
    rclpy.spin(sol)
    sol.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()

