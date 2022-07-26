#include "ros2_control_test_nodes/mim_control/demo_reactive_planners_solo12_step_adjustment.hpp"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/math/rpy.hpp>
#include <cmath>

DemoReactivePlanner::DemoReactivePlanner() {}

DemoReactivePlanner::DemoReactivePlanner(std::string path_to_urdf, Eigen::VectorXd initial_q) {
    // building the pinocchio model
    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::urdf::buildModel(path_to_urdf, root_joint, model);
    data = pinocchio::Data(model);

    // temp
//     Eigen::VectorXd initial_q(19);
//     initial_q << 0.0, 0.0, 0.25, 0.0, 0.0, 0.38, 0.92, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6;
//     initial_q << 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6;
//     float norm = sqrt(pow(initial_q(3), 2) + pow(initial_q(4), 2) + pow(initial_q(5), 2) + pow(initial_q(6), 2));
//     Eigen::Vector4d norm_quat = {initial_q(3) / norm, initial_q(4) / norm, initial_q(5) / norm, initial_q(6) / norm};
//     initial_q.segment(3, 4) = norm_quat;

    // creating and initializing the controllers
    // initialize centroidal controller
    mu = 0.6;
    kc = {0.0, 0.0, 200.0};
    dc = {10.0, 10.0, 10.0};
    kb = {25.0, 25.0, 25.0};
    db = {22.5, 22.5, 22.5};
    qp_penalty_lin = {1e0, 1e0, 1e6};
    qp_penalty_ang = {1e6, 1e6, 1e6};
    Eigen::VectorXd qp_penalty_weights(qp_penalty_lin.size() + qp_penalty_ang.size());
    qp_penalty_weights << qp_penalty_lin, qp_penalty_ang;

    centrl_pd_ctrl = mim_control::CentroidalPDController();
    // values below obtained from printing the 2nd argument of centrl_pd_ctrl.initialize() in demo_robot_com_ctrl_cpp.py in mim_control repo
    Eigen::Vector3d inertia = {0.04196225, 0.0699186, 0.08607027};
    centrl_pd_ctrl.initialize(2.5, inertia);
    std::cout << "Here 1!" << std::endl;
    force_qp = mim_control::CentroidalForceQPController();
    force_qp.initialize(4, mu, qp_penalty_weights);

    // initialize impedance controllers
    kp = Eigen::VectorXd::Zero(12);
    kd = Eigen::VectorXd::Zero(12);
    kp << 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0;
    kd << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0;
    std::string root_name = "universe";
    endeff_names = {"FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"};
    imp_ctrls = {mim_control::ImpedanceController(),
                 mim_control::ImpedanceController(),
                 mim_control::ImpedanceController(),
                 mim_control::ImpedanceController()};
    for (int i = 0; i < 4; i++) {
        imp_ctrls[i].initialize(model, root_name, endeff_names[i]);
    }
    std::cout << "Here 2!" << std::endl;
    // initialize fields for the quadruped Dcm reactive stepper
    is_left_leg_in_contact = true;
    l_min = -0.1;
    l_max = 0.1;
    w_min = -0.08;
    w_max = 0.2;
    t_min = 0.1;
    t_max = 1.0;
    l_p = 0.00;
    com_height = 0.193;
    weight = Eigen::VectorXd::Zero(9);
    weight << 1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000;
    mid_air_foot_height = 0.05;
    control_period = 0.001;
    planner_loop = 0.010;

    pinocchio::framesForwardKinematics(model, data, initial_q);

    Eigen::VectorXd base_pose = initial_q.head(7);
    Eigen::Vector3d front_left_foot_position = data.oMf[imp_ctrls[0].get_endframe_index()].translation();
    Eigen::Vector3d front_right_foot_position = data.oMf[imp_ctrls[1].get_endframe_index()].translation();
    Eigen::Vector3d hind_left_foot_position = data.oMf[imp_ctrls[2].get_endframe_index()].translation();
    Eigen::Vector3d hind_right_foot_position = data.oMf[imp_ctrls[3].get_endframe_index()].translation();
    std::cout << "Here 3!" << std::endl;
    v_des = {0.0, 0.0, 0.0};
    y_des = 0.2;

    // initialize quadruped_dcm_reactive_stepper
    quadruped_dcm_reactive_stepper = reactive_planners::QuadrupedDcmReactiveStepper();
    quadruped_dcm_reactive_stepper.initialize(
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
            hind_right_foot_position
    );
    std::cout << "Here 3.1!" << std::endl;
    quadruped_dcm_reactive_stepper.set_desired_com_velocity(v_des);
    std::cout << "Here 3.2!" << std::endl;
    quadruped_dcm_reactive_stepper.set_polynomial_end_effector_trajectory();
    std::cout << "Here 4!" << std::endl;
    // initalize more fields
    com_des = {0.0, 0.0};
    yaw_des = yaw(initial_q);
    cnt_array = {0.0, 0.0};
    open_loop = true;
    dcm_force = {0.0, 0.0, 0.0};
    std::cout << "Here 5!" << std::endl;
};

Eigen::VectorXd DemoReactivePlanner::compute_torques(Eigen::VectorXd &q, Eigen::VectorXd &dq, float control_time) {

    // update pinocchio
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::framesForwardKinematics(model, data, q);
    pinocchio::computeCentroidalMomentum(model, data, q, dq);
    pinocchio::centerOfMass(model, data, q);

    // get x_com and xd_com
    Eigen::Matrix<double, 3, 1, 0> x_com = data.com[0];
    Eigen::MatrixXd xd_com = data.vcom[0];

    // make solo turn
    yaw_des = yaw(q);
    yaw_des = yaw_des + (y_des * 0.001);

    // get feet position
    Eigen::Vector3d front_left_foot_position = data.oMf[imp_ctrls[0].get_endframe_index()].translation();
    Eigen::Vector3d front_right_foot_position = data.oMf[imp_ctrls[1].get_endframe_index()].translation();
    Eigen::Vector3d hind_left_foot_position = data.oMf[imp_ctrls[2].get_endframe_index()].translation();
    Eigen::Vector3d hind_right_foot_position = data.oMf[imp_ctrls[3].get_endframe_index()].translation();

    // get feet velocity
    Eigen::Vector3d front_left_foot_velocity = pinocchio::getFrameVelocity(model, data,
                                                                           imp_ctrls[0].get_endframe_index(),
                                                                           pinocchio::LOCAL_WORLD_ALIGNED).linear();
    Eigen::Vector3d front_right_foot_velocity = pinocchio::getFrameVelocity(model, data,
                                                                            imp_ctrls[1].get_endframe_index(),
                                                                            pinocchio::LOCAL_WORLD_ALIGNED).linear();
    Eigen::Vector3d hind_left_foot_velocity = pinocchio::getFrameVelocity(model, data,
                                                                          imp_ctrls[2].get_endframe_index(),
                                                                          pinocchio::LOCAL_WORLD_ALIGNED).linear();
    Eigen::Vector3d hind_right_foot_velocity = pinocchio::getFrameVelocity(model, data,
                                                                           imp_ctrls[3].get_endframe_index(),
                                                                           pinocchio::LOCAL_WORLD_ALIGNED).linear();
    // compute x_des local
    quadruped_dcm_reactive_stepper.run(
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
            yaw(q),
            false
    );

    Eigen::VectorXd x_des_local = Eigen::VectorXd::Zero(12);
    x_des_local << quadruped_dcm_reactive_stepper.get_front_left_foot_position(),
            quadruped_dcm_reactive_stepper.get_front_right_foot_position(),
            quadruped_dcm_reactive_stepper.get_hind_left_foot_position(),
            quadruped_dcm_reactive_stepper.get_hind_right_foot_position();

    std::cout << "x_des_local: " << x_des_local << std::endl;

    Eigen::Vector4d contact_array = quadruped_dcm_reactive_stepper.get_contact_array(); // cnt_array
    std::cout << "contact-array: " << contact_array << std::endl;

    for (int j = 0; j < 4; j++) {
        mim_control::ImpedanceController imp = imp_ctrls[j];
        Eigen::Vector3d foot_des_local = data.oMf[imp.get_rootframe_index()].translation();
        x_des_local.segment(3 * j, 3) = x_des_local.segment(3 * j, 3) - foot_des_local;
    }

    // compute w_com and F
    // convert rpy to quaternion
    Eigen::Matrix<float, 3, 3> rpy_des = pinocchio::rpy::rpyToMatrix<float>(0.0, 0.0, yaw_des);
    Eigen::Quaternionf x_ori_quat;
    x_ori_quat = rpy_des;
    Eigen::Vector4d x_ori = {x_ori_quat.x(), x_ori_quat.y(), x_ori_quat.z(), x_ori_quat.w()};
    Eigen::Vector3d x_angvel = {0.0, 0.0, y_des};

    // compute w_com
    centrl_pd_ctrl.run(
            kc,
            dc,
            kb,
            db,
            q.head(3),
            x_com,
            dq.head(3),
            xd_com,
            q.segment(3, 4),
            x_ori,
            dq.segment(3, 3),
            x_angvel
    );
    Eigen::VectorXd w_com = Eigen::VectorXd::Zero(6);
    // w_com(2) = w_com(2) + 9.8 * 2.5;
    w_com = w_com + centrl_pd_ctrl.get_wrench();
    // std::cout << "w_com: " << w_com << std::endl;

    // compute ee_forces
    Eigen::Vector3d com = pinocchio::centerOfMass(model, data, q);
    Eigen::VectorXd rel_eff = Eigen::VectorXd::Zero(12);
    int i = 0;
    for (const std::string &eff_name: endeff_names) {
        int id = model.getFrameId(eff_name);
        Eigen::Vector3d diff = data.oMf[id].translation() - com;
        rel_eff(i * 3) = diff(0);
        rel_eff(i * 3 + 1) = diff(1);
        rel_eff(i * 3 + 2) = diff(2);
        i++;
    }

    force_qp.run(w_com, rel_eff, contact_array);
    Eigen::VectorXd ee_forces = force_qp.get_forces();

    // get des_vel
    Eigen::VectorXd des_vel = Eigen::VectorXd::Zero(12);
    des_vel
            << quadruped_dcm_reactive_stepper.get_front_left_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_front_right_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_hind_left_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_hind_right_foot_velocity();

    // passing forces to the impedance controller
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(12);
    for (int i = 0; i < 4; i++) {
        Eigen::Vector3d desired_pos = x_des_local.segment(3 * i, 3);
        pinocchio::Motion xd_des = pinocchio::Motion(des_vel.segment(3 * i, 3), Eigen::Vector3d({0, 0, 0}));
        Eigen::VectorXd kp_array(6);
        Eigen::VectorXd kd_array(6);
        if (contact_array(i) == 1) {
            kp_array = Eigen::VectorXd::Zero(6);
            kd_array = Eigen::VectorXd::Zero(6);
        } else {
            kp_array << kp.segment(3*i, 3), 0, 0, 0;
            kd_array << kd.segment(3*i, 3), 0, 0, 0;
        }
        // kp_array << kp.segment(3*i, 3), 0, 0, 0;
        // kp_array << 50, 50, 50, 0, 0, 0;
        // kd_array << kd.segment(3*i, 3), 0, 0, 0;
        // kd_array << 5, 5, 5, 0, 0, 0;
        imp_ctrls[i].run(
                q,
                dq,
                kp_array.array(),
                kd_array.array(),
                1.0,
                pinocchio::SE3(Eigen::Matrix3d::Identity(), desired_pos),
                pinocchio::Motion(xd_des),
                pinocchio::Force(ee_forces.segment(3 * i, 3), Eigen::Vector3d::Zero(3))
        );
        tau = tau + imp_ctrls[i].get_joint_torques();
    }
    return tau;
}

double DemoReactivePlanner::yaw(Eigen::VectorXd &q) {
    Eigen::Vector4d quat = q.segment(3, 4);
    double x = quat(0);
    double y = quat(1);
    double z = quat(2);
    double w = quat(3);
    return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
}