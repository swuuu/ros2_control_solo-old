import numpy as np

from ros2_control_test_nodes.impedance.robot_properties_solo.Solo12Robot import Solo12Robot
from ros2_control_test_nodes.impedance.robot_properties_solo.Solo12Config import Solo12Config
from .robot_impedance_controller import RobotImpedanceController


class Demo:

    def __init__(self, urdf_path, path_to_meshes):
        # Create a robot instance in the simulator
        robot_config = Solo12Config(urdf_path, path_to_meshes)
        robot = Solo12Robot()

        # Impedance controller gains
        # kp = robot.nb_ee * [200, 200, 200]
        kp = robot.nb_ee * [200.0, 200.0, 200.0]
        # kp = robot.nb_ee * [20.0, 20.0, 20.0]
        kd = robot.nb_ee * [10.0, 10.0, 10.0]

        # Desired leg length.
        x_des = robot.nb_ee * [0.0, 0.0, -0.25]
        xd_des = robot.nb_ee * [0.0, 0.0, 0.0]

        # distributing forces to the active end-effectors
        f = robot.nb_ee * [0.0, 0.0, (robot_config.mass * 9.8) / 4]

        config_file = robot_config.ctrl_path
        robot_leg_ctrl = RobotImpedanceController(robot, config_file)

        self.robot = robot
        self.robot_leg_ctrl = robot_leg_ctrl
        self.kp = kp
        self.kd = kd
        self.x_des = x_des
        self.xd_des = xd_des
        self.f = f

    def compute_torques(self, q, dq):
        tau = self.robot_leg_ctrl.return_joint_torques(
            q, dq, self.kp, self.kd, self.x_des, self.xd_des, self.f
        )
        return tau

    def read_foot_position_rel_shoulder(self):
        return self.robot_leg_ctrl.get_foot_pos_rel_shoulder_pos()