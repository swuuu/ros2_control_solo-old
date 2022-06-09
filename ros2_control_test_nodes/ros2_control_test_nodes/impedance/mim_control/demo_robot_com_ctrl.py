from ros2_control_test_nodes.impedance.robot_properties_solo.Solo12Robot import Solo12Robot
from ros2_control_test_nodes.impedance.robot_properties_solo.Solo12Config import Solo12Config
from .robot_impedance_controller import RobotImpedanceController
from .robot_centroidal_controller import RobotCentroidalController


class Demo():

    def __init__(self, urdf_path, path_to_meshes):
        # Create a robot instance in the simulator.
        robot_config = Solo12Config(urdf_path, path_to_meshes)
        robot = Solo12Robot()
        # mu = 0.2
        # kc = [200, 200, 200]
        # dc = [5, 5, 5]
        # kb = [200, 200, 200]
        # db = [1.0, 1.0, 1.0]
        mu = 0.7
        kc = [40, 40, 40]
        dc = [20, 20, 20]
        kb = [4, 4, 4]
        db = [4, 4, 4]
        # mu = 0.7
        # kc = [200, 200, 200]
        # dc = [50, 50, 50]
        # kb = [200, 200, 200]
        # db = [50, 50, 50]
        qp_penalty_lin = 3 * [1e6]
        qp_penalty_ang = 3 * [1e6]

        # Desired center of mass position and velocity.
        x_com = [0.0, 0.0, 0.18]
        # x_com = [0.0, 0.0, 0.45]
        xd_com = [0.0, 0.0, 0.0]
        # The base should be flat.
        x_ori = [0.0, 0.0, 0.0, 1.0]
        x_angvel = [0.0, 0.0, 0.0]
        # All end-effectors are in contact.
        cnt_array = robot.nb_ee * [1]

        # Impedance controller gains
        kp = robot.nb_ee * [0.0, 0.0, 0.0]  # Disable for now
        kd = robot.nb_ee * [0.0, 0.0, 0.0]
        # kp = robot.nb_ee * [200.0, 200.0, 200.0]
        # kd = robot.nb_ee * [10.0, 10.0, 10.0]
        x_des = robot.nb_ee * [0.0, 0.0, -0.25]  # Desired leg length
        xd_des = robot.nb_ee * [0.0, 0.0, 0.0]

        config_file = robot_config.ctrl_path
        robot_cent_ctrl = RobotCentroidalController(
            robot_config,
            mu=mu,
            kc=kc,
            dc=dc,
            kb=kb,
            db=db,
            qp_penalty_lin=qp_penalty_lin,
            qp_penalty_ang=qp_penalty_ang,
        )
        robot_leg_ctrl = RobotImpedanceController(robot, config_file)

        self.kp = kp
        self.kd = kd
        self.x_com = x_com
        self.xd_com = xd_com
        self.x_ori = x_ori
        self.x_angvel = x_angvel
        self.x_des = x_des
        self.xd_des = xd_des
        self.cnt_array = cnt_array
        self.robot_cent_ctrl = robot_cent_ctrl
        self.robot_leg_ctrl = robot_leg_ctrl

    # # Run the simulator for 100 steps
    # for _ in range(4000):
    #     # Step the simulator.
    #     env.step(
    #         sleep=True
    #     )  # You can sleep here if you want to slow down the replay
    #     # Read the final state and forces after the stepping.
    #     q, dq = robot.get_state()
    #     # computing forces to be applied in the centroidal space
    #     w_com = robot_cent_ctrl.compute_com_wrench(
    #         q, dq, x_com, xd_com, x_ori, x_angvel
    #     )
    #     # distributing forces to the active end effectors
    #     F = robot_cent_ctrl.compute_force_qp(q, dq, cnt_array, w_com)
    #     # passing forces to the impedance controller
    #     tau = robot_leg_ctrl.return_joint_torques(
    #         q, dq, kp, kd, x_des, xd_des, F
    #     )
    #     # passing torques to the robot
    #     robot.send_joint_command(tau)

    def compute_torques(self, q, dq, node=None):
        # computing forces to be applied in the centroidal space
        w_com = self.robot_cent_ctrl.compute_com_wrench(
            q, dq, self.x_com, self.xd_com, self.x_ori, self.x_angvel
        )
        # distributing forces to the active end effectors
        F = self.robot_cent_ctrl.compute_force_qp(q, dq, self.cnt_array, w_com, node)
        # passing forces to the impedance controller
        tau = self.robot_leg_ctrl.return_joint_torques(
            q, dq, self.kp, self.kd, self.x_des, self.xd_des, F
        )
        if node:
            node.get_logger().info(f'w_com={w_com}')
            node.get_logger().info(f'F={F}')
        return tau
