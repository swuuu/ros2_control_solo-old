cmake_minimum_required(VERSION 3.8)
project(ros2_control_test_nodes)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


find_package(PkgConfig)        # find pkg_config

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(eigen-quadprog REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
        ${PINOCCHIO_INCLUDE_DIRS}
)

add_executable(test_controllers_cpp
        src/TestControllers.cpp
        src/ControllersNode.cpp
        src/Algorithms/PD_control/PD_control.cpp
        src/Algorithms/mim_control/centroidal_force_qp_controller.cpp
        src/Algorithms/mim_control/centroidal_pd_controller.cpp
        src/Algorithms/mim_control/impedance_controller.cpp
        src/Algorithms/mim_control/demo_com_ctrl_cpp.cpp
        src/Algorithms/mim_control/demo_reactive_planners_solo12_step_adjustment.cpp
        src/Algorithms/reactive_planners/quadruped_dcm_reactive_stepper.cpp
        src/Algorithms/reactive_planners/dcm_reactive_stepper.cpp
        src/Algorithms/reactive_planners/dcm_vrp_planner.cpp
        src/Algorithms/reactive_planners/dynamically_consistent_end_effector_trajectory.cpp
        src/Algorithms/reactive_planners/polynomial_end_effector_trajectory.cpp
        src/Algorithms/reactive_planners/stepper_head.cpp
        )
ament_target_dependencies(test_controllers_cpp rclcpp std_msgs sensor_msgs gazebo_msgs std_srvs eiquadprog)
target_link_libraries(test_controllers_cpp pinocchio::pinocchio /opt/openrobots/lib/libeiquadprog.so)
target_link_libraries(test_controllers_cpp eigen-quadprog::eigen-quadprog)

install(TARGETS
        test_controllers_cpp
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
