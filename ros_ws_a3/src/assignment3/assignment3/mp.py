#!/usr/bin/env python3
import numpy
import random
import sys

import moveit_msgs.msg
import moveit_msgs.srv
import rclpy
from rclpy.node import Node
import rclpy.duration
import transforms3d._gohlketransforms as tf
import transforms3d
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform, PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import copy
import math

def convert_to_message(T):
    t = Pose()
    position, Rot, _, _ = transforms3d.affines.decompose(T)
    orientation = transforms3d.quaternions.mat2quat(Rot)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[1]
    t.orientation.y = orientation[2]
    t.orientation.z = orientation[3]
    t.orientation.w = orientation[0]        
    return t

class MoveArm(Node):
    def __init__(self):
        super().__init__('move_arm')

        #Loads the robot model, which contains the robot's kinematics information
        self.ee_goal = None
        self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        #Loads the robot model, which contains the robot's kinematics information
        self.declare_parameter(
            'rd_file', rclpy.Parameter.Type.STRING)
        robot_desription = self.get_parameter('rd_file').value
        with open(robot_desription, 'r') as file:
            robot_desription_text = file.read()
        self.robot = URDF.from_xml_string(robot_desription_text)
        self.base = self.robot.get_root()
        self.get_joint_info()


        self.service_cb_group1 = MutuallyExclusiveCallbackGroup()
        self.service_cb_group2 = MutuallyExclusiveCallbackGroup()
        self.q_current = []

        # Wait for moveit IK service
        self.ik_service = self.create_client(moveit_msgs.srv.GetPositionIK, '/compute_ik', callback_group=self.service_cb_group1)
        while not self.ik_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
        self.get_logger().info('IK service ready')

        # Wait for validity check service
        self.state_valid_service = self.create_client(moveit_msgs.srv.GetStateValidity, '/check_state_validity',
                                                      callback_group=self.service_cb_group2)
        while not self.state_valid_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for state validity service...')
        self.get_logger().info('State validity service ready')

        # MoveIt parameter
        self.group_name = 'arm'
        self.get_logger().info(f'child map: \n{self.robot.child_map}')

        #Subscribe to topics
        self.sub_joint_states = self.create_subscription(JointState, '/joint_states', self.get_joint_state, 10)
        self.goal_cb_group = MutuallyExclusiveCallbackGroup()
        self.sub_goal = self.create_subscription(Transform, '/motion_planning_goal', self.motion_planning_cb, 2,
                                                 callback_group=self.goal_cb_group)
        self.current_obstacle = "NONE"
        self.sub_obs = self.create_subscription(String, '/obstacle', self.get_obstacle, 10)

        #Set up publisher
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.1, self.motion_planning_timer, callback_group=self.timer_cb_group)

    
    def get_joint_state(self, msg):
        '''This callback provides you with the current joint positions of the robot 
        in member variable q_current.
        '''
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    
    def get_obstacle(self, msg):
        '''This callback provides you with the name of the current obstacle which
        exists in the RVIZ environment. Options are "None", "Simple", "Hard",
        or "Super". '''
        self.current_obstacle = msg.data

    def motion_planning_cb(self, ee_goal):
        self.get_logger().info("Motion planner goal received.")
        if self.ee_goal is not None:
            self.get_logger().info("Motion planner busy. Please try again later.")
            return
        self.ee_goal = ee_goal

    def motion_planning_timer(self):
        if self.ee_goal is not None:
            self.get_logger().info("Calling motion planner")            
            self.motion_planning(self.ee_goal)
            self.ee_goal = None
            self.get_logger().info("Motion planner done")            
                
   
    def motion_planning(self, ee_goal: Transform): 
        '''Callback function for /motion_planning_goal. This is where you will
        implement your RRT motion planning which is to generate a joint
        trajectory for your manipulator. You are welcome to add other functions
        to this class (i.e. an is_segment_valid" function will likely come in 
        handy multiple times in the motion planning process and it will be 
        easiest to make this a seperate function and then call it from motion
        planning). You may also create trajectory shortcut and trajectory 
        sample functions if you wish, which will also be called from the 
        motion planning function.

        Args: 
            ee_goal: Transform() object describing the desired base to 
            end-effector transformation 
        '''

        quat = ee_goal.rotation
        R = transforms3d.quaternions.quat2mat(numpy.array([quat.w, quat.x, quat.y, quat.z], dtype=numpy.float64))
        vec = ee_goal.translation
        t = numpy.array([vec.x, vec.y, vec.z]).reshape(3, 1)
        T = numpy.concatenate((R, t), axis=1)
        T = numpy.concatenate((T, numpy.zeros((1, 4))), axis=0)
        T[-1, -1] = 1
        
        goal_q = self.IK(T)
        if not goal_q:
            return
        waypoints = []
        start_branch = RRTBranch(parent=None, q=numpy.array(self.q_current))
        waypoints.append(start_branch)
        
        continue_bool = numpy.linalg.norm(start_branch.q - goal_q) >= 0.1
        while continue_bool:
            rand_q = numpy.random.uniform(low=-numpy.pi, high=numpy.pi, size=len(self.q_current))
            closest_pt = self.find_closest_point_in_tree(waypoints, rand_q)
            delta_q = rand_q - closest_pt.q
            length = numpy.linalg.norm(delta_q) 
            if length > 0.1:
                delta_q = delta_q / (10.0*length)
            new_q = closest_pt.q + delta_q
            
            
            if self.is_segment_valid(new_q, goal_q) and self.is_segment_valid(closest_pt.q, new_q):
                new_branch = closest_pt
                total_vec = goal_q - new_q
                length = numpy.linalg.norm(total_vec)
                delta_q = (goal_q - new_q) / (length/0.4)
                iters = int(numpy.floor(length / 0.4))
                for j in range(iters):
                    interm_q = j*delta_q + new_q
                    new_branch = RRTBranch(new_branch, interm_q)
                    waypoints.append(new_branch)
                waypoints.append(RRTBranch(new_branch, goal_q))
                continue_bool = False
                break
            
            if self.current_obstacle is "NONE":
                new_branch = RRTBranch(closest_pt, new_q)
                waypoints.append(new_branch)
            else:
                while not self.is_segment_valid(closest_pt.q, new_q):
                    new_q -= (new_q - closest_pt.q)*0.1
                if self.is_segment_valid(closest_pt.q, new_q):
                    new_branch = RRTBranch(closest_pt, new_q)
                    waypoints.append(new_branch)
            
            closest_goal = self.find_closest_point_in_tree(waypoints, goal_q)
            if numpy.linalg.norm(goal_q - closest_goal.q) <= 0.5 and self.is_segment_valid(closest_goal.q, goal_q):
                continue_bool = False
                last_branch = RRTBranch(closest_goal, goal_q)
                waypoints.append(last_branch)

        q_path = []
        current_branch = waypoints[-1]
        q_path.append(current_branch)
        while current_branch.parent is not None:
            q_path.insert(0, current_branch)
            current_branch = current_branch.parent
            
        shortcut_bool = True
        m = 0
        while shortcut_bool:
            todelete = []
            for i in range(len(q_path)-3):
                shortcut_dist = numpy.linalg.norm(numpy.array(q_path[i].q) - numpy.array(q_path[i+2].q))
                orig_dist = numpy.linalg.norm(numpy.array(q_path[i].q) - numpy.array(q_path[i+1].q)) + numpy.linalg.norm(numpy.array(q_path[i+1].q) - numpy.array(q_path[i+2].q))
                if shortcut_dist < orig_dist and self.is_segment_valid(q_path[i].q, q_path[i+2].q):
                    todelete.append(i+1)
                    m += 1
            for k in range(len(todelete)-1, -1, -1):
                del q_path[todelete[k]]
            print("shortcut processing")
            if m == 0:
                shortcut_bool = False
            else:
                m = 0
        
        edge_num = len(q_path) - 1
        for i in range(edge_num):
            current_edge_dist = numpy.linalg.norm(numpy.array(q_path[i+1].q) - numpy.array(q_path[i].q))
            if current_edge_dist >= 0.5:
                num_new_pts = int(current_edge_dist / 0.5)
                delta = (numpy.array(q_path[i+1].q) - numpy.array(q_path[i].q)) / (num_new_pts + 1) # adding 1 into the denominator bc we don't need to add a new endpoint
                current = q_path[i]
                for j in range(num_new_pts):
                    new_q = RRTBranch(current, delta + current.q)
                    q_path.insert(i+j+1, new_q)
                    current = new_q
        
        JointTrajectoryPoints_list = []
        for joint_branch in q_path:
            msg = JointTrajectoryPoint()
            msg.positions = (joint_branch.q).tolist()
            JointTrajectoryPoints_list.append(msg)
        
        Joint_Tragjectory_Msg = JointTrajectory()
        Joint_Tragjectory_Msg.joint_names = self.joint_names
        Joint_Tragjectory_Msg.points = JointTrajectoryPoints_list
        self.pub.publish(Joint_Tragjectory_Msg)
        
        
    def is_segment_valid(self, q_old, q_new):
        step_num=numpy.linalg.norm(q_new-q_old)/0.1 + 1
        delta_q = (q_new - q_old) / step_num
        q_tmp = q_old
        for i in range(int(step_num)):
            q_tmp = q_tmp + delta_q
            if not self.is_state_valid(q_tmp):
                return False
        return True


    def find_closest_point_in_tree(self, tree, r):
        shortest_distance = numpy.linalg.norm(r-tree[0].q)
        closest_point = tree[0]
        for i in range(1, len(tree)-1):
            if shortest_distance > numpy.linalg.norm(r-tree[i].q):
                shortest_distance = numpy.linalg.norm(r-tree[i].q)
                closest_point = tree[i]
        return closest_point

    
    def IK(self, T_goal):
        """ This function will perform IK for a given transform T of the 
        end-effector. It .

        Returns:
            q: returns a list q[] of values, which are the result 
            positions for the joints of the robot arm, ordered from proximal 
            to distal. If no IK solution is found, it returns an empy list
        """

        req = moveit_msgs.srv.GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = list(numpy.zeros(self.num_joints))
        req.ik_request.robot_state.joint_state.velocity = list(numpy.zeros(self.num_joints))
        req.ik_request.robot_state.joint_state.effort = list(numpy.zeros(self.num_joints))
        req.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = 'base'
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
        
        self.get_logger().info('Sending IK request...')
        res = self.ik_service.call(req)
        self.get_logger().info('IK request returned')
        
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        for i in range(0,len(q)):
            while (q[i] < -math.pi): q[i] = q[i] + 2 * math.pi
            while (q[i] > math.pi): q[i] = q[i] - 2 * math.pi
        return q

    
    def get_joint_info(self):
        '''This is a function which will collect information about the robot which
        has been loaded from the parameter server. It will populate the variables
        self.num_joints (the number of joints), self.joint_names and
        self.joint_axes (the axes around which the joints rotate)
        '''
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
        self.get_logger().info('Num joints: %d' % (self.num_joints))


    
    def is_state_valid(self, q):
        """ This function checks if a set of joint angles q[] creates a valid state,
        or one that is free of collisions. The values in q[] are assumed to be values
        for the joints of the UR5 arm, ordered from proximal to distal.

        Returns:
            bool: true if state is valid, false otherwise
        """
        req = moveit_msgs.srv.GetStateValidity.Request()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = list(q)
        req.robot_state.joint_state.velocity = list(numpy.zeros(self.num_joints))
        req.robot_state.joint_state.effort = list(numpy.zeros(self.num_joints))
        req.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()

        res = self.state_valid_service.call(req)

        return res.valid



class RRTBranch(object):
    '''This is a class which you can use to keep track of your tree branches.
    It is easiest to do this by appending instances of this class to a list 
    (your 'tree'). The class has a parent field and a joint position field (q). 
    
    You can initialize a new branch like this:
        RRTBranch(parent, q)
    Feel free to keep track of your branches in whatever way you want - this
    is just one of many options available to you.
    '''
    def __init__(self, parent, q):
        self.parent = parent
        self.q = q


def main(args=None):
    rclpy.init(args=args)
    ma = MoveArm()
    ma.get_logger().info("Move arm initialization done")
    executor = MultiThreadedExecutor()
    executor.add_node(ma)
    executor.spin()
    ma.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

