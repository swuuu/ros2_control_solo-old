from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from tf_transformations import quaternion_matrix
from std_srvs.srv import Trigger

import numpy as np
from enum import Enum

# PD Controller imports
from ros2_control_test_nodes.effort_controller.controllers import PDController
from ros2_control_test_nodes.effort_controller.potential_field_planner import PotentialFieldPlanner
from ros2_control_test_nodes.effort_controller.pinocchio_helper_functions import PinocchioHelperFunctions

# Impedance Controller imports
from ros2_control_test_nodes.impedance.mim_control.demo_robot_impedance import Demo as ImpedanceDemo
from ros2_control_test_nodes.impedance.mim_control.demo_robot_com_ctrl import Demo as CentroidalDemo

class RobotImpedanceController(Node):

    def __init__(self):
        super().__init__('robot_impedance_controller')

        self.joint_config = np.zeros(12)
        self.joint_velocity = np.zeros(12)
        self.joint_effort = np.zeros(12)
        self.robot_pose = np.zeros(7)
        self.robot_twist = np.zeros(6)
        self.k = 2  # k in potential field planner
        self.delta_t = 0.002  # also for the potential field planner

        # dictionary to re-order the numbers read from /joint_states
        self.joint_states_order = {'FL_HAA': 0, 'FL_HFE': 1, 'FL_KFE': 2, 'FR_HAA': 3, 'FR_HFE': 4, 'FR_KFE': 5,
                                   'HL_HAA': 6, 'HL_HFE': 7, 'HL_KFE': 8, 'HR_HAA': 9, 'HR_HFE': 10, 'HR_KFE': 11}

        # Declare all paramters
        self.declare_parameter("path_to_urdf_file")
        self.declare_parameter("wait_sec_between_publish", 5)
        self.declare_parameter("path_to_meshes")
        self.declare_parameter("config_desired", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        # Read all parameters
        self.robot_description = self.get_parameter("path_to_urdf_file").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        self.path_to_meshes = self.get_parameter("path_to_meshes").value
        self.config_des = np.array(self.get_parameter("config_desired").value)

        # Create joint state publisher, timer, and joint state subscriber
        self.publisher_ = self.create_publisher(Float64MultiArray, "/effort_controllers/commands", 10)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.update_joint_states, 10)

        # Subscribe to the model_states parameter
        self.link_states_subscriber = self.create_subscription(LinkStates, "/link_states", self.update_link_states, 10)

        # Robot Impedance Demo
        self.impedance_demo = ImpedanceDemo(self.robot_description, self.path_to_meshes)

        # Robot Centroidal Demo
        self.centroidal_demo = CentroidalDemo(self.robot_description, self.path_to_meshes)

        # Helper functions for the inverse dynamics controller
        self.pin_helper_funcs = PinocchioHelperFunctions(self.robot_description)
        self.potential_field_planner = PotentialFieldPlanner(self.k, self.delta_t, self.config_des)
        self.PD_controller = PDController(1000000, 100)  # k = 20, d = 2

        # States
        self.States = Enum("States", "STAND IMPEDANCE CENTROIDAL RESET_EFFORT")
        self.curr_state = self.States.RESET_EFFORT

        # Service to start the PD controller
        self.srv_PD = self.create_service(Trigger, "trigger_PD", self.trigger_PD_callback)
        # Service to start the impedance controller
        self.srv_impedance = self.create_service(Trigger, "trigger_impedance", self.trigger_impedance_callback)
        # Service to start the impedance controller
        self.srv_centroidal = self.create_service(Trigger, "trigger_centroidal", self.trigger_centroidal_callback)
        # Service to reset effort to 0
        self.srv_reset_effort = self.create_service(Trigger, "reset_effort", self.trigger_reset_effort_callback)

    def timer_callback(self):

        def compute_tau_from_PD():
            m = self.pin_helper_funcs.get_mass_matrix(self.joint_config, self.joint_velocity)  # mass matrix
            h = self.pin_helper_funcs.get_h(self.joint_config, self.joint_velocity)
            # compute reference joint velocity and reference joint position
            q_dot_ref = self.potential_field_planner.compute_ref_vel(self.joint_config)
            q_ref = self.potential_field_planner.compute_ref_pos(self.joint_config)
            # compute torques and send
            tau = self.PD_controller.compute_torques(m, h, q_dot_ref, self.joint_velocity, q_ref, self.joint_config)
            return tau

        if self.curr_state == self.States.STAND:
            tau = compute_tau_from_PD()
            msg = Float64MultiArray()
            msg.data = tau.tolist()
            self.publisher_.publish(msg)

        if self.curr_state == self.States.IMPEDANCE:
            foot_positions = self.impedance_demo.read_foot_position_rel_shoulder()
            self.get_logger().info(f'foot_positions = {foot_positions}')

            msg = Float64MultiArray()
            joint_config_with_base = np.concatenate((self.robot_pose, self.joint_config))
            joint_vel_with_base = np.concatenate((self.robot_twist, self.joint_velocity))
            tau = self.impedance_demo.compute_torques(joint_config_with_base, joint_vel_with_base)
            tau_from_PD = compute_tau_from_PD()
            tau = tau + tau_from_PD
            self.get_logger().info(f'Tau = {tau}')
            msg.data = tau.tolist()
            self.publisher_.publish(msg)

        if self.curr_state == self.States.CENTROIDAL:
            msg = Float64MultiArray()

            # self.joint_config = np.array([0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6])
            # self.joint_velocity = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            joint_config_with_base = np.concatenate((self.robot_pose, self.joint_config))
            joint_vel_with_base = np.concatenate((self.robot_twist, self.joint_velocity))
            tau = self.centroidal_demo.compute_torques(joint_config_with_base, joint_vel_with_base, self)
            self.get_logger().info(f'q={joint_config_with_base}')
            self.get_logger().info(f'dq={joint_vel_with_base}')
            self.get_logger().info(f'current_effort={self.joint_effort}')
            self.get_logger().info(f'tau = {tau}')
            msg.data = tau.tolist()
            self.publisher_.publish(msg)

            # pin_helper_func = self.pin_helper_funcs
            # test = pin_helper_func.forward_kinematics(self.joint_config)
            # self.get_logger().info(f'forward_kinematics={test}')

        if self.curr_state == self.States.RESET_EFFORT:
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publisher_.publish(msg)

    def update_joint_states(self, msg):
        # get the correct order of joint configs/vel
        joint_config = np.zeros(12)
        joint_vel = np.zeros(12)
        joint_effort = np.zeros(12)
        for i, name in enumerate(msg.name):
            joint_config[self.joint_states_order[name]] = msg.position[i]
            joint_vel[self.joint_states_order[name]] = msg.velocity[i]
            joint_effort[self.joint_states_order[name]] = msg.effort[i]
        self.joint_config = joint_config
        self.joint_velocity = joint_vel
        self.joint_effort = joint_effort

    def update_link_states(self, msg):
        # Saving the pose
        self.robot_pose[0] = msg.pose[1].position.x
        self.robot_pose[1] = msg.pose[1].position.y
        self.robot_pose[2] = msg.pose[1].position.z
        self.robot_pose[3] = msg.pose[1].orientation.x
        self.robot_pose[4] = msg.pose[1].orientation.y
        self.robot_pose[5] = msg.pose[1].orientation.z
        self.robot_pose[6] = msg.pose[1].orientation.w

        # Saving the twist
        # Rotate twist to the body frame
        # transform = quaternion_matrix([self.robot_pose[6], self.robot_pose[3], self.robot_pose[4], self.robot_pose[5]])
        # rotation = transform[:3, :3]
        # linear_vel = np.array([msg.twist[1].linear.x, msg.twist[1].linear.y, msg.twist[1].linear.z])
        # angular_vel = np.array([msg.twist[1].angular.x, msg.twist[1].angular.y, msg.twist[1].angular.z])
        # self.robot_twist[0:3] = np.matmul(rotation, linear_vel)
        # self.robot_twist[3:6] = np.matmul(rotation, angular_vel)

        # self.robot_pose = np.zeros(7)
        # self.robot_twist = np.zeros(6)
        # self.robot_pose[2] = msg.pose[1].position.z
        # # self.robot_pose[2] = 0.2
        # self.robot_pose[3] = msg.pose[1].orientation.x
        # self.robot_pose[4] = msg.pose[1].orientation.y
        # self.robot_pose[5] = msg.pose[1].orientation.z
        # self.robot_pose[6] = msg.pose[1].orientation.w

        self.robot_twist[0] = msg.twist[1].linear.x
        self.robot_twist[1] = msg.twist[1].linear.y
        self.robot_twist[2] = msg.twist[1].linear.z
        self.robot_twist[3] = msg.twist[1].angular.x
        self.robot_twist[4] = msg.twist[1].angular.y
        self.robot_twist[5] = msg.twist[1].angular.z

        # self.robot_pose[0] = 0.0001
        # self.robot_pose[1] = 0.0001
        # self.robot_pose[2] = 0.0001
        # self.robot_pose[3] = 0.0001
        # self.robot_pose[4] = 0.0001
        # self.robot_pose[5] = 0.0001
        # self.robot_pose[6] = 1.0

    def trigger_PD_callback(self, request, response):
        self.curr_state = self.States.STAND
        return response

    def trigger_impedance_callback(self, request, response):
        self.curr_state = self.States.IMPEDANCE
        return response

    def trigger_centroidal_callback(self, request, response):
        self.curr_state = self.States.CENTROIDAL
        return response

    def trigger_reset_effort_callback(self, request, response):
        self.curr_state = self.States.RESET_EFFORT
        return response
