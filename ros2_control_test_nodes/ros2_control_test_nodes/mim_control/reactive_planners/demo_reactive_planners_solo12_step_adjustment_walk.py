import numpy as np
from ros2_control_test_nodes.mim_control.robot_properties_solo.Solo12Config import Solo12Config
from ros2_control_test_nodes.mim_control.robot_properties_solo.Solo12Robot import Solo12Robot
from ros2_control_test_nodes.mim_control.mim_control.robot_centroidal_controller import RobotCentroidalController
from ros2_control_test_nodes.mim_control.mim_control.robot_impedance_controller import RobotImpedanceController
from .reactive_planners_cpp import QuadrupedDcmReactiveStepper
import pinocchio as pin
from scipy.spatial.transform import Rotation


class Demo():

    def __init__(self, robot_description, path_to_meshes):
        np.set_printoptions(suppress=True, precision=2)
        pin.switchToNumpyArray()
        self.data_collector = None

        # Create a robot instance. This initializes the simulator as well.
        self.robot = Solo12Robot()
        tau = np.zeros(12)

        # obtained from https://github.com/machines-in-motion/reactive_planners/blob/master/demos/demo_reactive_planners_solo12_step_adjustment_walk.ipynb
        q = np.array(
            [0.0, 0.0, 0.25, 0.0, 0.0, 0.38, 0.92, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6])
        total_mass = sum([i.mass for i in self.robot.pin_robot.model.inertias[1:]])
        self.kp = np.array(12 * [100.0])
        self.kd = 12 * [10.0]
        robot_config = Solo12Config(robot_description, path_to_meshes)
        config_file = robot_config.ctrl_path
        self.solo_leg_ctrl = RobotImpedanceController(self.robot, config_file)
        self.centr_controller = RobotCentroidalController(
            robot_config,
            mu=0.6,
            kc=[0, 0, 200],
            dc=[10, 10, 10],
            kb=[25, 25, 25.],
            db=[22.5, 22.5, 22.5],
            qp_penalty_lin=[1e0, 1e0, 1e6],
            qp_penalty_ang=[1e6, 1e6, 1e6],
        )
        is_left_leg_in_contact = True
        l_min = -0.1
        l_max = 0.1
        w_min = -0.08
        w_max = 0.2
        t_min = 0.1
        t_max = 1.0
        # t_max = 10.0
        l_p = 0.00  # Pelvis width
        # com_height = 0.25
        com_height = 0.193
        weight = [1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000]
        mid_air_foot_height = 0.05
        control_period = 0.001
        planner_loop = 0.010
        # init poses
        self.robot.pin_robot.framesForwardKinematics(q)
        base_pose = q[:7]
        front_left_foot_position = self.robot.pin_robot.data.oMf[
            self.solo_leg_ctrl.imp_ctrl_array[0].frame_end_idx].translation
        front_right_foot_position = self.robot.pin_robot.data.oMf[
            self.solo_leg_ctrl.imp_ctrl_array[1].frame_end_idx].translation
        hind_left_foot_position = self.robot.pin_robot.data.oMf[
            self.solo_leg_ctrl.imp_ctrl_array[2].frame_end_idx].translation
        hind_right_foot_position = self.robot.pin_robot.data.oMf[
            self.solo_leg_ctrl.imp_ctrl_array[3].frame_end_idx].translation

        self.v_des = np.array([0.0, 0.0, 0.0])
        self.y_des = 0.2  # Speed of the yaw angle

        self.quadruped_dcm_reactive_stepper = QuadrupedDcmReactiveStepper()
        self.quadruped_dcm_reactive_stepper.initialize(
            is_left_leg_in_contact,
            l_min,
            l_max,
            w_min,
            w_max,
            t_min,
            t_max,
            l_p,
            com_height,
            weight,
            mid_air_foot_height,
            control_period,
            planner_loop,
            base_pose,
            front_left_foot_position,
            front_right_foot_position,
            hind_left_foot_position,
            hind_right_foot_position,
        )

        self.quadruped_dcm_reactive_stepper.set_desired_com_velocity(self.v_des)
        self.quadruped_dcm_reactive_stepper.set_polynomial_end_effector_trajectory()

        self.x_com = [[0.0], [0.0], [com_height]]
        self.com_des = np.array([0.0, 0.0])
        self.yaw_des = self.yaw(q)
        self.cnt_array = [1, 1]
        self.control_time = 0
        self.open_loop = True
        self.dcm_force = np.array([0.0, 0.0, 0.0])
        self.offset = 0.015  # foot radius

        self.com_height = com_height

        # for tracking feet positions
        self.x_curr_local = None
        self.x_des_local = None

    def zero_cnt_gain(self, kp, cnt_array):
        gain = np.array(kp).copy()
        for i, v in enumerate(cnt_array):
            if v == 1:
                gain[3 * i: 3 * (i + 1)] = 0.0
        return gain

    def yaw(self, q):
        return np.array(
            Rotation.from_quat([np.array(q)[3:7]]).as_euler("xyz", degrees=False)
        )[0, 2]

    def quadruped_dcm_reactive_stepper_start(self):
        self.quadruped_dcm_reactive_stepper.start()

    def compute_torques(self, q, qdot, control_time, action="", node=None):
        # q = np.array(
        #     [0.0, 0.0, 0.25, 0.0, 0.0, 0.38, 0.92, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6])
        # qdot = np.array(
        #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.03, -0.34, 1.61, 0.02, -0.03, 0.07, -0.01, 0.03, -0.07, 0.06, 0.33, -1.6])
        self.robot.pin_robot.com(q, qdot)
        self.robot.update_pinocchio(q, qdot)
        x_com = self.robot.pin_robot.com(q, qdot)[0]
        xd_com = self.robot.pin_robot.com(q, qdot)[1]

        if action == "forward":
            self.v_des[0] = 0.2
            self.com_des += self.v_des[:2] * 0.001
            # self.com_des[0] = q[0] + self.v_des[0] * 0.001
            self.yaw_des = 0.0
        elif action == "left":
            self.v_des[1] = 0.5
            self.com_des += self.v_des[:2] * 0.001
            self.yaw_des = 0.0
        elif action == "right":
            self.v_des[1] = -0.5
            self.com_des += self.v_des[:2] * 0.001
            self.yaw_des = 0.0
        elif action == "turn":
            self.y_des = 0.1
            self.yaw_des = self.yaw(q)
            if node:
                node.get_logger().info(f'current yaw = {self.yaw_des}')
            # print(f'yaw_des = {self.yaw_des}')
            self.yaw_des += self.y_des * 0.001
            # self.yaw_des = self.yaw_des + (self.y_des * 0.001)

        FL = self.solo_leg_ctrl.imp_ctrl_array[0]
        FR = self.solo_leg_ctrl.imp_ctrl_array[1]
        HL = self.solo_leg_ctrl.imp_ctrl_array[2]
        HR = self.solo_leg_ctrl.imp_ctrl_array[3]

        # Define left as front left and back right leg
        front_left_foot_position = self.robot.pin_robot.data.oMf[FL.frame_end_idx].translation
        front_right_foot_position = self.robot.pin_robot.data.oMf[FR.frame_end_idx].translation
        hind_left_foot_position = self.robot.pin_robot.data.oMf[HL.frame_end_idx].translation
        hind_right_foot_position = self.robot.pin_robot.data.oMf[HR.frame_end_idx].translation
        front_left_foot_velocity = pin.getFrameVelocity(
            self.robot.pin_robot.model, self.robot.pin_robot.data, FL.frame_end_idx, pin.LOCAL_WORLD_ALIGNED).linear
        front_right_foot_velocity = pin.getFrameVelocity(
            self.robot.pin_robot.model, self.robot.pin_robot.data, FR.frame_end_idx, pin.LOCAL_WORLD_ALIGNED).linear
        hind_left_foot_velocity = pin.getFrameVelocity(
            self.robot.pin_robot.model, self.robot.pin_robot.data, HL.frame_end_idx, pin.LOCAL_WORLD_ALIGNED).linear
        hind_right_foot_velocity = pin.getFrameVelocity(
            self.robot.pin_robot.model, self.robot.pin_robot.data, HR.frame_end_idx, pin.LOCAL_WORLD_ALIGNED).linear

        self.quadruped_dcm_reactive_stepper.run(
            control_time,
            front_left_foot_position,
            front_right_foot_position,
            hind_left_foot_position,
            hind_right_foot_position,
            front_left_foot_velocity,
            front_right_foot_velocity,
            hind_left_foot_velocity,
            hind_right_foot_velocity,
            x_com,
            xd_com,
            self.yaw(q),
            not self.open_loop,
        )

        x_des_local = []
        x_des_local.extend(self.quadruped_dcm_reactive_stepper.get_front_left_foot_position())
        x_des_local.extend(self.quadruped_dcm_reactive_stepper.get_front_right_foot_position())
        x_des_local.extend(self.quadruped_dcm_reactive_stepper.get_hind_left_foot_position())
        x_des_local.extend(self.quadruped_dcm_reactive_stepper.get_hind_right_foot_position())

        # for tracking the feet position
        if self.x_curr_local is None:
            self.x_curr_local = np.array([front_left_foot_position])
            self.x_des_local = np.array([self.quadruped_dcm_reactive_stepper.get_front_left_foot_position()])
        else:
            self.x_curr_local = np.concatenate((self.x_curr_local, np.array([front_left_foot_position])))
            self.x_des_local = np.concatenate((self.x_des_local, np.array([self.quadruped_dcm_reactive_stepper.get_front_left_foot_position()])))

        cnt_array = self.quadruped_dcm_reactive_stepper.get_contact_array()
        next_footstep_pos = self.quadruped_dcm_reactive_stepper.get_next_support_foot_position()

        for j in range(4):
            imp = self.solo_leg_ctrl.imp_ctrl_array[j]
            x_des_local[3 * j: 3 * (j + 1)] -= imp.pin_robot.data.oMf[
                imp.frame_root_idx
            ].translation

        w_com = self.centr_controller.compute_com_wrench(
            q.copy(),
            qdot.copy(),
            [self.com_des[0], self.com_des[1], self.com_height],
            self.v_des,
            pin.Quaternion(pin.rpy.rpyToMatrix(0., 0., self.yaw_des)).coeffs(),
            [0.0, 0.0, self.y_des], # angular velocity desired
        )
        if node:
            node.get_logger().info(f'w_com = {w_com}')

        F = self.centr_controller.compute_force_qp(q, qdot, cnt_array, w_com)

        des_vel = np.concatenate(
            (
                self.quadruped_dcm_reactive_stepper.get_front_left_foot_velocity(),
                self.quadruped_dcm_reactive_stepper.get_front_right_foot_velocity(),
                self.quadruped_dcm_reactive_stepper.get_hind_left_foot_velocity(),
                self.quadruped_dcm_reactive_stepper.get_hind_right_foot_velocity(),
            )
        )

        if cnt_array[0] == 1:
            F[3:6] = -self.dcm_force[:3]
            F[6:9] = -self.dcm_force[:3]
        else:
            F[0:3] = -self.dcm_force[:3]
            F[9:12] = -self.dcm_force[:3]

        tau = self.solo_leg_ctrl.return_joint_torques(
            q.copy(),
            qdot.copy(),
            self.zero_cnt_gain(self.kp, cnt_array),
            self.zero_cnt_gain(self.kd, cnt_array),
            x_des_local,
            des_vel,
            F,
        )

        return tau

    # for tracking the feet position
    def get_pos_feet(self):
        return self.x_curr_local, self.x_des_local

