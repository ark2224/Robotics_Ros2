#!/usr/bin/env python

import math
import numpy
from threading import Thread, Lock
import time

import geometry_msgs.msg
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import transforms3d._gohlketransforms as tf
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from copy import deepcopy

from custom_msg.msg import CartesianCommand

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

def convert_to_trans_message(T):
    t = geometry_msgs.msg.Transform()
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

# Returns the angle-axis representation of the rotation contained in the input matrix
# Use like this:
# angle, axis = rotation_from_matrix(R)
def rotation_from_matrix(matrix):
    R = numpy.array(matrix, dtype=numpy.float64, copy=False)
    R33 = R[:3, :3]
    # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, W = numpy.linalg.eig(R33.T)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    axis = numpy.real(W[:, i[-1]]).squeeze()
    # point: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, Q = numpy.linalg.eig(R)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    # rotation angle depending on axis
    cosa = (numpy.trace(R33) - 1.0) / 2.0
    if abs(axis[2]) > 1e-8:
        sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
    elif abs(axis[1]) > 1e-8:
        sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
    else:
        sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
    angle = math.atan2(sina, cosa)
    return angle, axis

class MarkerControl(Node):

    #Initialization
    def __init__(self):
        super().__init__('cartesian_control')
        self.mutex = Lock()        
        self.start = False
        self.rate = self.create_rate(1)
        foo = CartesianCommand()
        #Loads the robot model, which contains the robot's kinematics information
        self.declare_parameter(
            'rd_file', rclpy.Parameter.Type.STRING)
        robot_desription = self.get_parameter('rd_file').value
        with open(robot_desription, 'r') as file:
            robot_desription_text = file.read()
        # print(robot_desription_text)
        self.robot = URDF.from_xml_string(robot_desription_text)
        print('start')
        #Subscribes to information about what the current joint values are.
        # rospy.Subscriber("joint_states", JointState, self.callback)
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.callback,
            10
        )

        # Publishes Cartesian goals
        # self.pub_command = rospy.Publisher("/cartesian_command", CartesianCommand, queue_size=1)
        self.pub_command = self.create_publisher(CartesianCommand, "/cartesian_command", 1)

        # Publishes IK command
        # self.ik_command = rospy.Publisher("/ik_command", geometry_msgs.msg.Transform,
        #                                   queue_size=1)
        self.ik_command = self.create_publisher(geometry_msgs.msg.Transform, "/ik_command", 1)

        # Publisher to set robot position
        # self.pub_reset = rospy.Publisher("/joint_command", JointState, queue_size=1)
        self.pub_reset = self.create_publisher(JointState, "/joint_command", 1)

        #This is where we hold the most recent joint transforms
        self.joint_transforms = []
        self.x_current = tf.identity_matrix()
        self.R_base = tf.identity_matrix()

        #Create "Interactive Marker" that we can manipulate in RViz
        self.cc_mode = True
        # print('self.robot', self.robot)
        self.init_marker()
        self.ee_tracking = 0
        self.red_tracking = 0
        self.q_current = []

        self.x_target = tf.identity_matrix()
        self.q0_desired = 0.0
        self.q0_base = 0.0
        self.angle_base = 0.0
        self.delta = 0.0

        # self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start = True

    #Resets the robot to a known pose
    def reset_robot(self):
        cmd = JointState()
        n = 0.5
        movable_joints = 0
        for i in range(0,len(self.robot.joints)):
            if self.robot.joints[i].type != 'fixed':
                movable_joints += 1
        for joint_i in range(0, movable_joints):
            if joint_i < movable_joints - 6:
                cmd.position.append(0)
            else:
                cmd.position.append(n)
                n += 0.5
                if n > 2.1: n = 0.0
        self.pub_reset.publish(cmd)
        # rospy.sleep(1.0)
        time.sleep(1)

    def init_marker(self):

        self.server = InteractiveMarkerServer(self, "control_markers")

        control_marker = InteractiveMarker()
        control_marker.header.frame_id = self.robot.get_root()
        control_marker.name = "cc_marker"

        move_control = InteractiveMarkerControl()
        move_control.name = "move_x"
        move_control.orientation.w = 1.0
        move_control.orientation.x = 1.0
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "move_y"
        move_control.orientation.w = 1.0
        move_control.orientation.y = 1.0
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "move_z"
        move_control.orientation.w = 1.0
        move_control.orientation.z = 1.0
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)

        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_x"
        move_control.orientation.w = 1.0
        move_control.orientation.x = 1.0
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_y"
        move_control.orientation.w = 1.0
        move_control.orientation.z = 1.0
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_z"
        move_control.orientation.w = 1.0
        move_control.orientation.y = 1.0
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)

        control_marker.scale = 0.25
        self.server.insert(marker=control_marker, feedback_callback=self.control_marker_feedback)

        global cc_entry, ik_entry, ik_go_entry
        self.menu_handler = MenuHandler()
        cc_entry = self.menu_handler.insert("Cartesian Control Mode", callback=self.set_cc_callback)
        ik_entry = self.menu_handler.insert("Inverse Kinematics Mode", callback=self.set_ik_callback)
        ik_go_entry = self.menu_handler.insert("IK to Here", callback=self.set_ik_go_callback)
        self.menu_handler.setCheckState( cc_entry, MenuHandler.CHECKED )
        self.menu_handler.setCheckState( ik_entry, MenuHandler.UNCHECKED )
        self.menu_handler.setVisible( ik_go_entry, False )
        self.menu_handler.apply(self.server, "cc_marker",)

        self.red_server = InteractiveMarkerServer(self, "redundancy_control_markers")               
        (joint_name, first_link) = self.robot.child_map[self.robot.get_root()][0]
        redundancy_marker = InteractiveMarker()
        redundancy_marker.header.frame_id = first_link
        redundancy_marker.name = "red_marker"
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "rotate_z"
        rotate_control.orientation.w = 1.0
        rotate_control.orientation.y = 1.0
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        redundancy_marker.controls.append(rotate_control)
        redundancy_marker.scale = 0.25
        self.red_server.insert(marker=redundancy_marker, feedback_callback=self.redundancy_marker_feedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()
        self.red_server.applyChanges()

    def set_cc_callback(self, feedback):
        self.cc_mode = True
        self.menu_handler.setCheckState( cc_entry, MenuHandler.CHECKED )
        self.menu_handler.setCheckState( ik_entry, MenuHandler.UNCHECKED )
        self.menu_handler.setVisible( ik_go_entry, False )
        self.menu_handler.reApply( self.server )
        self.server.applyChanges()

    def set_ik_callback(self, feedback):
        self.cc_mode = False
        self.menu_handler.setCheckState( cc_entry, MenuHandler.UNCHECKED )
        self.menu_handler.setCheckState( ik_entry, MenuHandler.CHECKED )
        self.menu_handler.setVisible( ik_go_entry, True )
        self.menu_handler.reApply( self.server )
        self.server.applyChanges()
        self.update_marker(self.x_current)

    def set_ik_go_callback(self, feedback):
        if self.cc_mode: return
        msg = JointState()
        msg = convert_to_trans_message(self.x_target)
        self.ik_command.publish(msg)

    def update_marker(self, T):
        # Note that we keep marker orientation fixed so that robot can be taken out of
        # singularity even when only doing translation only control
        # Otherwise, arrows always point along arm, and arm stays stretched out
        # forever if it starts out that way.
        # Ttrans = tf.translation_matrix(
        #     tf.translation_from_matrix(T)
        # )
        self.server.setPose("cc_marker", convert_to_message(T))
        self.server.applyChanges()

    def redundancy_marker_feedback(self, feedback):
        if feedback.event_type == feedback.MOUSE_DOWN:
            self.x_target = self.x_current
            self.q0_base = self.q_current[0]
            q = feedback.pose.orientation
            qvec = ((q.w, q.x, q.y, q.z))
            R = tf.quaternion_matrix(qvec)
            angle, direction, point = tf.rotation_from_matrix(R)
            self.angle_base = angle
            self.delta = 0
            self.red_tracking = 1
        elif feedback.event_type == feedback.MOUSE_UP:
            #self.get_logger().info("Secondary UP")
            self.q0_desired = self.q_current[0]
            self.red_tracking = 0
        if feedback.event_type == feedback.POSE_UPDATE:
            q = feedback.pose.orientation
            qvec = ((q.w, q.x, q.y, q.z))
            R = tf.quaternion_matrix(qvec)
            angle, direction, point = tf.rotation_from_matrix(R)
            norm_angle = angle
            if (abs(angle - self.angle_base + 2*math.pi) < abs(angle - self.angle_base)): norm_angle = angle + 2*math.pi
            if (abs(angle - self.angle_base - 2*math.pi) < abs(angle - self.angle_base)): norm_angle = angle - 2*math.pi
            self.delta = self.delta + norm_angle - self.angle_base
            #print "Angle: " + str(angle) + "; base: " + str(self.angle_base) + "; delta: " + str(self.delta)
            self.angle_base = angle
            self.mutex.acquire()
            self.q0_desired = self.q0_base + self.delta
            self.mutex.release()

    def control_marker_feedback(self, feedback):
        if feedback.event_type == feedback.MOUSE_DOWN:
            # since default marker orientation stays fixed, change in orientation
            # must be applied relative to reference orientation when we started dragging
            self.x_target = self.x_current
            self.R_base = self.x_current
            self.R_base[0:3,3]=0
            self.ee_tracking = 1
        elif feedback.event_type == feedback.MOUSE_UP:
            #self.get_logger().info("Primary UP")
            self.ee_tracking = 0
        elif feedback.event_type == feedback.POSE_UPDATE:
            self.mutex.acquire()
            R = tf.quaternion_matrix((feedback.pose.orientation.w,
                                    feedback.pose.orientation.x,
                                    feedback.pose.orientation.y,
                                    feedback.pose.orientation.z))
            T = tf.translation_matrix((feedback.pose.position.x,
                                    feedback.pose.position.y,
                                    feedback.pose.position.z))
            #self.x_target = numpy.dot( T,numpy.dot(R,self.R_base) )
            self.x_target = numpy.dot( T,R )
            self.mutex.release()

    def timer_callback(self):
        if not self.cc_mode: return
        self.mutex.acquire()
        if self.ee_tracking:
            msg = CartesianCommand()
            msg.x_target = convert_to_trans_message(self.x_target)
            msg.secondary_objective = False
            self.pub_command.publish(msg)
        if self.red_tracking:
            msg = CartesianCommand()
            msg.x_target = convert_to_trans_message(self.x_current)
            msg.secondary_objective = True
            msg.q0_target = float(self.q0_desired)
            self.pub_command.publish(msg)
        self.mutex.release()

    def callback(self, joint_values):
        if self.start:
            root = self.robot.get_root()
            T = tf.identity_matrix()
            self.mutex.acquire()
            self.joint_transforms = []
            self.q_current = joint_values.position
            self.process_link_recursive(root, T, joint_values)
            if self.cc_mode and (not self.ee_tracking):
                self.update_marker(self.x_current)
            self.mutex.release()

    def align_with_z(self, axis):
        T = tf.identity_matrix()
        z = numpy.array([0,0,1])
        x = numpy.array([1,0,0])
        dot = numpy.dot(z,axis)
        if dot == 1: return T
        if dot == -1: return tf.rotation_matrix(math.pi, x)
        rot_axis = numpy.cross(z, axis)
        angle = math.acos(dot)
        return tf.rotation_matrix(angle, rot_axis)

    def process_link_recursive(self, link, T, joint_values):
        if link not in self.robot.child_map: 
            self.x_current = T
            return
        for i in range(0,len(self.robot.child_map[link])):
            (joint_name, next_link) = self.robot.child_map[link][i]
            if joint_name not in self.robot.joint_map:
                self.get_logger().info("Joint not found in map")
                continue
            current_joint = self.robot.joint_map[joint_name]        

            trans_matrix = tf.translation_matrix((current_joint.origin.xyz[0], 
                                                current_joint.origin.xyz[1],
                                                current_joint.origin.xyz[2]))
            rot_matrix = tf.euler_matrix(current_joint.origin.rpy[0], 
                                        current_joint.origin.rpy[1],
                                        current_joint.origin.rpy[2], 'rxyz')
            origin_T = numpy.dot(trans_matrix, rot_matrix)
            current_joint_T = numpy.dot(T, origin_T)
            if current_joint.type != 'fixed':
                if current_joint.name not in joint_values.name:
                    self.get_logger().info("Joint not found in list")
                    continue
                # compute transform that aligns rotation axis with z
                aligned_joint_T = numpy.dot(current_joint_T, self.align_with_z(current_joint.axis))
                self.joint_transforms.append(aligned_joint_T)
                index = joint_values.name.index(current_joint.name)
                angle = joint_values.position[index]
                joint_rot_T = tf.rotation_matrix(angle, 
                                                numpy.asarray(current_joint.axis))
                next_link_T = numpy.dot(current_joint_T, joint_rot_T)
            else:
                next_link_T = current_joint_T
            if i<1: # ignored other branches
                self.process_link_recursive(next_link, next_link_T, joint_values)       


def main(args=None):
    rclpy.init(args=args)
    mc = MarkerControl()
    mc.get_logger().info("Marker control initialization done")
    # rospy.sleep(1.0)
    time.sleep(1)
    # mc.get_logger().info("Resetting robot")
    # mc.reset_robot()
    # rospy.sleep(1.0)
    # time.sleep(1)
    mc.get_logger().info("Marker Control Ready")
    # rospy.spin()
    rclpy.spin(mc)

    mc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
