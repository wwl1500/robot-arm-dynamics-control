cmake_minimum_required(VERSION 3.16)

project(robot_arm_dynamics_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_library(arm_kinematics
    src/arm_kinematics.cpp
)

target_include_directories(arm_kinematics
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(arm_kinematics
    PUBLIC
        Eigen3::Eigen
)

add_executable(kinematics_demo
    src/main.cpp
)

target_link_libraries(kinematics_demo
    PRIVATE
        arm_kinematics
)

add_library(arm_dynamics
    src/arm_dynamics.cpp
)

target_include_directories(arm_dynamics
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(arm_dynamics
    PUBLIC
        Eigen3::Eigen
)

if(TARGET pinocchio::pinocchio)
    target_link_libraries(arm_dynamics
        PUBLIC
            pinocchio::pinocchio
    )
else()
    target_link_libraries(arm_dynamics
        PUBLIC
            pinocchio
    )
endif()

add_executable(dynamics_demo
    src/dynamics_demo.cpp
)

target_link_libraries(dynamics_demo
    PRIVATE
        arm_dynamics
)

add_library(arm_controller
    src/arm_controller.cpp
)

target_include_directories(arm_controller
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(arm_controller
    PUBLIC
        Eigen3::Eigen
        arm_kinematics
        arm_dynamics
)

add_executable(controller_demo
    src/controller_demo.cpp
)

target_link_libraries(controller_demo
    PRIVATE
        arm_controller
)

add_library(arm_control_runner
    src/arm_control_runner.cpp
)

target_include_directories(arm_control_runner
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(arm_control_runner
    PUBLIC
        Eigen3::Eigen
        arm_controller
)

add_executable(control_runner_demo
    src/control_runner_demo.cpp
)

target_link_libraries(control_runner_demo
    PRIVATE
        arm_control_runner
)
