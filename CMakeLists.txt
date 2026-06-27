cmake_minimum_required(VERSION 3.16)

project(robot_arm_dynamics_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

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
