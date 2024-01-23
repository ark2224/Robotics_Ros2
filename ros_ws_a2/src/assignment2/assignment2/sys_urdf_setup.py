#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy
import transforms3d._gohlketransforms as tf
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
from sensor_msgs.msg import JointState
from custom_msg.msg import CartesianCommand
import geometry_msgs.msg
from geometry_msgs.msg import Transform
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
# from interactive_markers.interactive_marker_server import *
from urdf_parser_py.urdf import URDF


def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.translation_from_matrix(T)
    orientation = tf.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[1]
    t.orientation.y = orientation[2]
    t.orientation.z = orientation[3]
    t.orientation.w = orientation[0]
    return t


def is_same_trans(trans0, trans1):
    return (abs(numpy.subtract(trans0, trans1)) < 10e-3).all()


def is_same(matrix0, matrix1):
    matrix0 = numpy.array(matrix0, dtype=numpy.float64, copy=True)
    matrix0 /= matrix0[3, 3]
    matrix1 = numpy.array(matrix1, dtype=numpy.float64, copy=True)
    matrix1 /= matrix1[3, 3]
    return numpy.allclose(matrix0, matrix1, 0, 1e-2)

def convert_to_trans_message(T):
    t = Transform()
    position = tf.translation_from_matrix(T)
    orientation = tf.quaternion_from_matrix(T)
    t.translation.x = position[0]
    t.translation.y = position[1]
    t.translation.z = position[2]
    t.rotation.x = orientation[1]
    t.rotation.y = orientation[2]
    t.rotation.z = orientation[3]
    t.rotation.w = orientation[0]
    return t


class CartesianGrader(Node):
    #Initialization
    def __init__(self):
        super().__init__('cartesian_grader')

        # Publisher to set robot position
        self.pub_reset = self.create_publisher(JointState, "/joint_command", 1)

        #Publisher to issue commands
        self.pub_cmd = self.create_publisher(CartesianCommand, "/cartesian_command", 1)
        self.pub_ik_cmd = self.create_publisher(Transform, "/ik_command", 1)

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.get_joint_state,
            10
        )
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=True)

        self.declare_parameter(
            'rd_file', rclpy.Parameter.Type.STRING)
        robot_desription = self.get_parameter('rd_file').value
        with open(robot_desription, 'r') as file:
            robot_desription_text = file.read()
        self.robot = URDF.from_xml_string(robot_desription_text)

        self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link
        self.end_effector = link

    def get_joint_state(self, msg):
        # This really should not be hard-coded in like this...
        if self.robot.name == 'ur': self.q0_current = msg.position[0]
        else: self.q0_current = msg.position[2] #this is for the Franka

    def reset_robot(self):
        cmd = JointState()

        if self.robot.name == 'ur':
            cmd.position.append(1.9)
            cmd.position.append(0.65)
            cmd.position.append(1.5)
            cmd.position.append(1.15)
            cmd.position.append(0.0)
            cmd.position.append(0.3)
        else:
            cmd.position.append(0.0)
            cmd.position.append(0.0)
            cmd.position.append(1.0)
            cmd.position.append(0.0)
            cmd.position.append(1.0)
            cmd.position.append(0.0)
            cmd.position.append(1.0)
            cmd.position.append(1.0)
            cmd.position.append(0.0)
        self.pub_reset.publish(cmd)
        time.sleep(1.0)

    def go_to_ik_pose(self, name, T, timeout):
        msg = convert_to_trans_message(T)
        start_time = time.time()
        done = False
        self.pub_ik_cmd.publish(msg)
        while not done and rclpy.ok():
            try:
                t = self.buffer.lookup_transform(self.robot.get_root(), self.end_effector, rclpy.time.Time())
                trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                rot = [t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z]
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.get_logger().info("TF Exception!")
                continue

            TR = numpy.dot(tf.translation_matrix(trans),
                           tf.quaternion_matrix(rot))

            if (is_same_trans(tf.translation_from_matrix(T), tf.translation_from_matrix(TR))):
               self.get_logger().info(name + ": PASSED")
               done = True
               return 1

            if (time.time() - start_time > timeout) :
                self.get_logger().info(name + ": Robot took too long to reach desired pose")
                self.get_logger().info("Robot took too long to reach desired pose. Grader timed out")
                done = True
            else:
                time.sleep(0.1)
        return 0

    def go_to_pose(self, name, T, secondary_objective, q0_target, timeout, points):
        msg = convert_to_trans_message(T)
        cartesian_command_msg = CartesianCommand()
        cartesian_command_msg.x_target = msg
        cartesian_command_msg.secondary_objective = secondary_objective
        cartesian_command_msg.q0_target = q0_target
        start_time = time.time()
        done = False
        index = 0
        while not done and rclpy.ok():
            index += 1
            self.pub_cmd.publish(cartesian_command_msg)
            try:
                t = self.buffer.lookup_transform(self.robot.get_root(), self.end_effector, rclpy.time.Time())
                trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                rot = [t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z]
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.get_logger().info("TF Exception!")
                continue
            
            TR = numpy.dot(tf.translation_matrix(trans),
                           tf.quaternion_matrix(rot))

            if (is_same(T, TR)):
                if secondary_objective:
                    if self.robot.name == 'ur' and index>30:
                        # ur5 has no redundant joints, secondary objective isn't possible
                        return points
                    else:
                        if abs(self.q0_current-q0_target)< 10e-3:
                            #print(name + ": PASSED")
                            return points

                else:
                    #print(name + ": PASSED")
                    return points

            if (time.time() - start_time > timeout) :
                #print(name + ": Robot took too long to reach desired pose")
                #print("Robot took too long to reach desired pose. Grader timed out")
                return 0

            else:
                time.sleep(0.1)
        return 0
