#!/usr/bin/env python3

import math
import numpy
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from custom_msg.msg import CartesianCommand
from urdf_parser_py.urdf import URDF
import random
import transforms3d
import transforms3d._gohlketransforms as tf
from threading import Thread, Lock

'''This is a class which will perform both cartesian control and inverse
   kinematics'''
class CCIK(Node):
    def __init__(self):
        super().__init__('ccik')
    #Load robot from parameter server
        # self.robot = URDF.from_parameter_server()
        self.declare_parameter(
            'rd_file', rclpy.Parameter.Type.STRING)
        robot_desription = self.get_parameter('rd_file').value
        with open(robot_desription, 'r') as file:
            robot_desription_text = file.read()
        self.robot = URDF.from_xml_string(robot_desription_text)

    #Subscribe to current joint state of the robot
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.get_joint_state, 10)

    #This will load information about the joints of the robot
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()

    #This is a mutex
        self.mutex = Lock()

    #Subscribers and publishers for for cartesian control
        self.cartesian_command_sub = self.create_subscription(
            CartesianCommand, '/cartesian_command', self.get_cartesian_command, 10)
        self.velocity_pub = self.create_publisher(JointState, '/joint_velocities', 10)
        self.joint_velocity_msg = JointState()

        #Subscribers and publishers for numerical IK
        self.ik_command_sub = self.create_subscription(
            Transform, '/ik_command', self.get_ik_command, 10)
        self.joint_command_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.joint_command_msg = JointState()

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
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

    '''This is the callback which will be executed when the cartesian control
       recieves a new command. The command will contain information about the
       secondary objective and the target q0. At the end of this callback, 
       you should publish to the /joint_velocities topic.'''
    def get_cartesian_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        # STEPS:
        #	1. calculate delta_x (ee'_T_ee)
        #		a. get translation x_target (b_T_ee' but in form of translation and )
        #		b. get orientation and change with rotation_from_matrix()
        #	2. convert delta_x into velocity v_ee using gain p (get experimentally; 0.1m/s and 1rad/s)
        #	3. get_jacobian()
        #	4. get pseudo-Jacobian (fuck does that look like for Vj?)
        #	5. q_des = J^+*v_ee
        #	6. secondary objective
        #	7. scale q_des
        #	8. publish that shit!
        
        joint_transforms, b_T_ee = self.forward_kinematics(self.q_current)

        # ===== STEP 1: =====
        quat = command.x_target.rotation
        b_R_ee_desired = transforms3d.quaternions.quat2mat(numpy.array([quat.w, quat.x, quat.y, quat.z], dtype=numpy.float64))
        vec = command.x_target.translation
        b_t_ee_desired = numpy.array([vec.x, vec.y, vec.z], dtype=numpy.float64)
        t = -numpy.dot(b_R_ee_desired.T, b_t_ee_desired).reshape(3, 1)
        ee_desired_T_b = numpy.concatenate((b_R_ee_desired.T, t), axis=1)
        ee_desired_T_b = numpy.concatenate((ee_desired_T_b, numpy.zeros((1,4))), axis=0)
        ee_desired_T_b[-1, -1] = 1
        
        ee_T_ee = numpy.matmul(ee_desired_T_b, b_T_ee)
        ee_trans = ee_T_ee[:-1, -1]
        ee_R_ee = ee_T_ee[:-1, :-1]

        angle, axis = self.rotation_from_matrix(ee_R_ee)
        ee_rot = angle*axis
        
        # ===== STEP 2: =====
        # "proportional gain p for the cartesian control should be greater than or equal to 1"
        #scale translation to not exceed 0.1 m/s:
        p_t = 1.0
        ee_trans *= p_t
        max_t = numpy.max(ee_trans)
        if max_t > 0.1:
            ee_trans /= 10*max_t
        #scale rotation to not exceed 1 rad/s:
        p_r = 1.0
        ee_rot *= p_r
        max_r = ee_rot.max()
        if max_r > 1.0:
            ee_rot /= max_r
        v_ee = numpy.concatenate((ee_trans, ee_rot), axis=0)
        
        # ===== STEP 3: =====
        J = self.get_jacobian(b_T_ee, joint_transforms)
        
        # ===== STEP 4: =====
        J_pinv = numpy.linalg.pinv(J, rcond=0.01)
        
        # ===== STEP 5: =====
        q_des = numpy.dot(J_pinv, v_ee)

        # ===== STEP 6: =====
        if command.secondary_objective:
            size = numpy.size(self.q_current)
            p_sec = float(3.0)
            q_sec = numpy.zeros(size, dtype=numpy.float64)
            q_sec[0] = p_sec*(command.q0_target - q_des[0])
            J_pinv_real = numpy.linalg.pinv(J)
            q_des = q_des + numpy.matmul(numpy.identity(size, dtype=numpy.float64) - numpy.matmul(J_pinv_real, J, dtype=numpy.float64), q_sec, dtype=numpy.float64)
        
        # ===== STEP 7: =====
        max_q = numpy.max(abs(q_des))
        if max_q > 1.0:
            q_des /= max_q
        # ===== Step 8: =====
        self.joint_velocity_msg.name = self.joint_names
        self.joint_velocity_msg.velocity = numpy.ndarray.tolist(q_des)
        self.velocity_pub.publish(self.joint_velocity_msg)
        
        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def get_jacobian(self, b_T_ee, joint_transforms):
        J = numpy.zeros((6,self.num_joints))
        #--------------------------------------------------------------------------
        
        ###### check the condition number: once it gets too big then stop
        # steps:
        # 1) compute ee_T_b by inverting b_T_ee
        b_R_ee = b_T_ee[:-1, :-1]
        b_trans_ee = b_T_ee[:-1, -1]
        t = -numpy.matmul(b_R_ee.T, b_trans_ee).reshape(3, 1)
        ee_T_b = numpy.concatenate((b_R_ee.T, t), axis=1)
        zeros = numpy.zeros(shape=(1, numpy.shape(ee_T_b)[-1]))
        ee_T_b = numpy.concatenate((ee_T_b, zeros), axis=0)
        ee_T_b[-1, -1] = 1
        
        # 2) ee_T_j = numpy.matmul(ee_T_b, b_T_j) for b_T_j in joint_transforms
        i = 0
        for b_T_j in joint_transforms:
            ee_T_j = numpy.matmul(ee_T_b, b_T_j)
            ee_R_j = ee_T_j[:-1, :-1]
            
            j_t_ee = -numpy.matmul(ee_T_j[:-1, :-1].T, ee_T_j[:-1, -1].reshape(3,1))
            axis = numpy.nonzero(self.joint_axes[i])[0]
            sign = 1
            if self.joint_axes[i][axis[0]] == -1:
                sign = -1
            
            x, y, z = j_t_ee
            S = numpy.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=numpy.float64)
            zeros = numpy.zeros(shape=numpy.shape(ee_R_j))
            Vj_1 = numpy.concatenate((ee_R_j, -numpy.matmul(ee_R_j, S)), axis=1)
            Vj_2 = numpy.concatenate((zeros, ee_R_j), axis=1)
            Vj = numpy.concatenate((Vj_1, Vj_2), axis=0)
            J[:,i] = sign*Vj[:, axis+3].reshape(6,)
            i += 1
        
        #--------------------------------------------------------------------------
        return -J

    '''This is the callback which will be executed when the inverse kinematics
       recieve a new command. The command will contain information about desired
       end effector pose relative to the root of your robot. At the end of this
       callback, you should publish to the /joint_command topic. This should not
       search for a solution indefinitely - there should be a time limit. When
       searching for two matrices which are the same, we expect numerical
       precision of 10e-3.'''
    def get_ik_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        # Process Transform passed in 'command' input (gives euler transformation and quaternion rotation) to get x_desired
        
        quat = command.rotation
        b_R_ee_desired = transforms3d.quaternions.quat2mat(numpy.array([quat.w, quat.x, quat.y, quat.z], dtype=numpy.float64))
        vec = command.translation
        b_t_ee_desired = numpy.array([vec.x, vec.y, vec.z], dtype=numpy.float64)
        t = -numpy.dot(b_R_ee_desired.T, b_t_ee_desired).reshape(3, 1)
        ee_desired_T_b = numpy.concatenate((b_R_ee_desired.T, t), axis=1)
        ee_desired_T_b = numpy.concatenate((ee_desired_T_b, numpy.zeros((1,4))), axis=0)
        ee_desired_T_b[-1, -1] = 1

        # set a timer for 10sec, and if so, try finding another random q_c 3xtimes
        i = 0
        size = numpy.size(self.q_current)
        delta_q = numpy.ones(size, numpy.float64)
        while (i < 3) and (numpy.linalg.norm(delta_q) > 0.01):
            i += 1
            start = time.time()
            q_c = numpy.random.uniform(-numpy.pi/2, numpy.pi/2, [size])
            while (numpy.linalg.norm(delta_q) > 0.01) and (time.time() - start < 10):
                joint_trans, b_T_ee = self.forward_kinematics(q_c)
                ee_T_ee = numpy.matmul(ee_desired_T_b, b_T_ee)
                ee_trans = ee_T_ee[:-1, -1]
                ee_R_ee = ee_T_ee[:-1, :-1]

                angle, axis = self.rotation_from_matrix(ee_R_ee)
                ee_rot = angle*axis
                delta_x = numpy.concatenate((ee_trans, ee_rot), axis=0)
                
                
                J = self.get_jacobian(b_T_ee, joint_trans)
                J_pinv = numpy.linalg.pinv(J, rcond=1e-2)
                
                delta_q = numpy.dot(J_pinv, delta_x)
                q_c = q_c + delta_q
               
                self.joint_command_msg.name = self.joint_names
                self.joint_command_msg.position = numpy.ndarray.tolist(q_c)
                self.joint_command_pub.publish(self.joint_command_msg)
                
                if (numpy.linalg.norm(delta_q) < 0.01):
                    break
            
        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This function will return the angle-axis representation of the rotation
       contained in the input matrix. Use like this: 
       angle, axis = rotation_from_matrix(R)'''
    def rotation_from_matrix(self, matrix):
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

    '''This is the function which will perform forward kinematics for your 
       cartesian control and inverse kinematics functions. It takes as input
       joint values for the robot and will return an array of 4x4 transforms
       from the base to each link of the robot, as well as the transform from
       the base to the end effector.
       Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = numpy.dot(tf.translation_matrix(joint.origin.xyz), tf.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2], 'rxyz'))
            T = numpy.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))
                T = numpy.dot(T, T_j)

            link = next_link
        return joint_transforms, T #where T = b_T_ee

    '''This is the callback which will recieve and store the current robot
       joint states.'''
    def get_joint_state(self, msg):
        self.mutex.acquire()
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
        self.mutex.release()


def main(args = None):
    rclpy.init()
    ccik = CCIK()
    rclpy.spin(ccik)
    ccik.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
