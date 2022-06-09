cmake_minimum_required(VERSION 3.8)
project(ros2_control_test_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(PkgConfig)        # find pkg_config
pkg_search_module(Eigen3 REQUIRED eigen3)   # use pkg_config to find eigen3
find_package(pinocchio REQUIRED)     # find pinocchio


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(eigen3_cmake_module REQUIRED)
# find_package(Eigen3)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
        ${PINOCCHIO_INCLUDE_DIRS}
)


add_executable(test_controller_cpp src/Node.cpp src/Controllers.cpp)
ament_target_dependencies(test_controller_cpp rclcpp std_msgs)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
