//
// Created by stanley on 6/6/22.
//
#include <chrono>
#include "ros2_control_test_nodes/Controllers.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

Controllers::States state = Controllers::NO_EFFORT;

// service callbacks
void PD_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    state = Controllers::STAND;
    response->success = true;
}

void centroidal_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    state = Controllers::CENTROIDAL;
    response->success = true;
};

Controllers::Controllers() : Node("test_controller_cpp") {
    // read parameters
    // robot URDF description
    this->declare_parameter<std::string>("path_to_urdf_file", "");
    std::string robot_description;
    this->get_parameter("path_to_urdf_file", robot_description);
    // loop rate (= wait_sec_between_publish)
    this->declare_parameter<double>("wait_sec_between_publish", 5.0);
    double wait_sec_between_publish;
    this->get_parameter("wait_sec_between_publish", wait_sec_between_publish);
    // desired configuration
    // std::vector<double>{}
    this->declare_parameter("config_desired");
    rclcpp::Parameter desired_config_rcl_param("config_desired", std::vector<double>({}));
    this->get_parameter("config_desired", desired_config_rcl_param);
    std::vector<double> desired_config_param = desired_config_rcl_param.as_double_array();

    // convert vector to eigen
    desired_config << desired_config_param[0], desired_config_param[1], desired_config_param[2],
            desired_config_param[3], desired_config_param[4], desired_config_param[5],
            desired_config_param[6], desired_config_param[7], desired_config_param[8],
            desired_config_param[9], desired_config_param[10], desired_config_param[11];

    // publishes effort to the joints
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controllers/commands", 10);

    // read the joint positions and velocities
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(
            &Controllers::update_joint_states, this, _1));
    // read the body position and velocity
    link_state_subscriber_ = this->create_subscription<gazebo_msgs::msg::LinkStates>("/link_states", 10, std::bind(
            &Controllers::update_body_state, this, _1));

    // service to start the PD control
    srv_PD = this->create_service<std_srvs::srv::Trigger>("trigger_PD", &PD_callback);
    // service to start the centroidal control
    srv_centroidal = this->create_service<std_srvs::srv::Trigger>("trigger_centroidal",
                                                                  &centroidal_callback);

    // controls
    // PD controller
    pdControl = PD_control(robot_description);

    // pdControl.build_model(robot_description);
    pdControl.set_desired_config(desired_config);
    pdControl.set_params(10, 0.002, 3000, 300);

    // COM controller
    demoComCtrl = DemoComCtrl(robot_description);

    // timer
    timer_ = this->create_wall_timer(std::chrono::duration<double>(wait_sec_between_publish),
                                     std::bind(&Controllers::timer_callback, this));
}

void Controllers::timer_callback() {
    if (state == 1) { // state = STAND
        Eigen::MatrixXd m = pdControl.get_mass_matrix(joint_config, joint_velocity);
        Eigen::VectorXd h = pdControl.get_h(joint_config, joint_velocity);
        Eigen::VectorXd q_dot_ref = pdControl.compute_ref_vel(joint_config);
        Eigen::VectorXd q_ref = pdControl.compute_ref_pos(joint_config);
        Eigen::VectorXd tau = pdControl.compute_torques(m, h, q_dot_ref, joint_velocity, q_ref, joint_config);
        auto msg = std_msgs::msg::Float64MultiArray();
        std::vector<double> tau_vector(tau.data(), tau.data() + tau.rows() * tau.cols());
        for (double i: tau_vector)
            std::cout << i << ',';
        std::cout << std::endl;
        msg.data = tau_vector;
        publisher_->publish(msg);
    } else if (state == 2) { // state = CENTROIDAL
        Eigen::VectorXd joint_config_with_base(robot_pose.size() + joint_config.size());
        joint_config_with_base << robot_pose, joint_config;
        Eigen::VectorXd joint_vel_with_base(robot_twist.size() + joint_velocity.size());
        joint_vel_with_base << robot_twist, joint_velocity;
        Eigen::VectorXd tau = demoComCtrl.compute_torques(joint_config_with_base, joint_vel_with_base);
        auto msg = std_msgs::msg::Float64MultiArray();
        std::vector<double> tau_vector(tau.data(), tau.data() + tau.rows() * tau.cols());
        for (double i: tau_vector)
            std::cout << i << ',';
        std::cout << std::endl;
        msg.data = tau_vector;
        publisher_->publish(msg);
    }

}

void Controllers::update_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::unordered_map<std::string, int> order_of_names = {{"FL_HAA", 0},
                                                           {"FL_HFE", 1},
                                                           {"FL_KFE", 2},
                                                           {"FR_HAA", 3},
                                                           {"FR_HFE", 4},
                                                           {"FR_KFE", 5},
                                                           {"HL_HAA", 6},
                                                           {"HL_HFE", 7},
                                                           {"HL_KFE", 8},
                                                           {"HR_HAA", 9},
                                                           {"HR_HFE", 10},
                                                           {"HR_KFE", 11}};
    for (int i = 0; i < 12; i++) {
        std::string name = msg->name[i];
        joint_config(order_of_names[name]) = msg->position[i];
        joint_velocity(order_of_names[name]) = msg->velocity[i];
    }
}

void Controllers::update_body_state(const gazebo_msgs::msg::LinkStates::SharedPtr msg) {
    // body pose
    robot_pose(0) = msg->pose[1].position.x;
    robot_pose(1) = msg->pose[1].position.y;
    robot_pose(2) = msg->pose[1].position.z;
    robot_pose(3) = msg->pose[1].orientation.x;
    robot_pose(4) = msg->pose[1].orientation.y;
    robot_pose(5) = msg->pose[1].orientation.z;
    robot_pose(6) = msg->pose[1].orientation.w;
    // body twist
    robot_twist(0) = msg->twist[1].linear.x;
    robot_twist(1) = msg->twist[1].linear.y;
    robot_twist(2) = msg->twist[1].linear.z;
    robot_twist(3) = msg->twist[1].angular.x;
    robot_twist(4) = msg->twist[1].angular.y;
    robot_twist(5) = msg->twist[1].angular.z;
}