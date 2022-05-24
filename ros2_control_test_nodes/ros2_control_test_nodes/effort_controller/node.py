import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from gazebo_msgs.msg import ModelStates

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException

import numpy as np
from enum import Enum

# PD controller imports
from pinocchio_helper_functions import PinocchioHelperFunctions
from potential_field_planner import PotentialFieldPlanner, is_at_goal, is_HAA_symmetric


class EffortNode(Node):

    def __init__(self):
        super().__init__('test_effort_controller')

        # common variables
        self.trigger = False
        self.joint_config = np.zeros(12)
        self.joint_velocity = np.zeros(12)
        self.k = 2  # k in potential field planner
        self.delta_t = 0.002  # also for the potential field planner

        # added a dictionary to re-order the numbers read from /joint_states
        self.joint_states_order = {'FL_HAA': 0, 'FL_HFE': 1, 'FL_KFE': 2, 'FR_HAA': 3, 'FR_HFE': 4, 'FR_KFE': 5,
                                   'HL_HAA': 6, 'HL_HFE': 7, 'HL_KFE': 8, 'HR_HAA': 9, 'HR_HFE': 10, 'HR_KFE': 11}

        # Declare all parameters
        self.declare_parameter("wait_sec_between_publish", 5)
        self.declare_parameter("config_desired", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("path_to_urdf_file")

        # Read all parameters
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        self.config_des = np.array(self.get_parameter("config_desired").value)
        self.robot_description = self.get_parameter("path_to_urdf_file").value

        # Create joint state publisher and timer
        self.publisher_ = self.create_publisher(Float64MultiArray, "/effort_controllers/commands", 10)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)

        # Create a subscriber to read the current joint states
        self.joint_state_subscriber = self.create_subscription(JointState, 'joint_states', self.update_joint_states, 10)

        # Create a service to start sending efforts
        self.srv = self.create_service(Trigger, "trigger_solo", self.trigger_callback)

        # Helper functions for the inverse dynamics controller
        self.pin_helper_funcs = PinocchioHelperFunctions(self.robot_description)
        self.potential_field_planner = PotentialFieldPlanner(self.k, self.delta_t, self.config_des)
        self.PD_controller = PDController(12000, 150)  # k = 20, d = 2

        # States
        self.States = Enum("States", "MOVE_TO_DESIRED STOP")
        self.state = self.States.MOVE_TO_DESIRED

    def timer_callback(self):

        if is_at_goal(self.config_des, self.joint_config):
            self.get_logger().info(f'Arrived at targets!')
            self.trigger = False

        if self.trigger:
            # 1. check if joint config reached the goal position
            if not is_at_goal(self.config_des, self.joint_config) and self.state == self.States.MOVE_TO_DESIRED:
                # 2. get mass matrix and h from pinocchio
                m = self.pin_helper_funcs.get_mass_matrix(self.joint_config, self.joint_velocity)  # mass matrix
                h = self.pin_helper_funcs.get_h(self.joint_config, self.joint_velocity)
                # 3. compute reference joint velocity and reference joint position
                q_dot_ref = self.potential_field_planner.compute_ref_vel(self.joint_config)
                q_ref = self.potential_field_planner.compute_ref_pos(self.joint_config)
                # 4. compute torques and send
                tau = self.PD_controller.compute_torques(m, h, q_dot_ref, self.joint_velocity, q_ref, self.joint_config)
                msg = Float64MultiArray()
                msg.data = tau.tolist()
                self.publisher_.publish(msg)

    def update_joint_states(self, msg):
        # get the correct order of joint configs/vel
        joint_config = np.zeros(12)
        joint_vel = np.zeros(12)
        for i, name in enumerate(msg.name):
            joint_config[self.joint_states_order[name]] = msg.position[i]
            joint_vel[self.joint_states_order[name]] = msg.velocity[i]
        self.joint_config = joint_config
        self.joint_velocity = joint_vel

    def trigger_callback(self, request, response):
        self.trigger = True
        return response