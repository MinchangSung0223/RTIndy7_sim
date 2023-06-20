cmake_minimum_required(VERSION 3.0)
project(RTIndy7_sim)

find_package(jsoncpp REQUIRED)
find_package(Eigen3 REQUIRED)

set(MR_HEADER_DIR ${PROJECT_SOURCE_DIR}/include/MR)
set(LR_HEADER_DIR ${PROJECT_SOURCE_DIR}/include/LR)
set(SIM_ROBOT_HEADER_DIR ${PROJECT_SOURCE_DIR}/include/SimRobot)
set(BULLET3_HEADER_DIR ${PROJECT_SOURCE_DIR}/include/bullet3)

include_directories(
        ${PROJ_INSTALL_INCLUDE_DIR}
        ${JSONCPP_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${MR_HEADERS_DIR}
        ${LR_HEADERS_DIR}
        ${BULLET3_HEADER_DIR}
        ${SIM_HEADERS_DIR}
)

add_subdirectory(include/MR)
add_subdirectory(include/LR)
add_subdirectory(include/SimRobot)
add_subdirectory(include/bullet3)

file(GLOB_RECURSE PYBULLET_INDY7 src/Indy7.cpp src/Indy7.h)

set(EXECUTABLES
    JointHinfControl
    TaskMotionControl
    HybridMotionForceControl
    ImpedanceControl
    ModernRoboticsTest
    DualArmTest
    LieRoboticsTest
)

foreach(EXECUTABLE ${EXECUTABLES})
    file(GLOB EXECUTABLE_SOURCE src/${EXECUTABLE}/*.cpp)
    add_executable(${EXECUTABLE} ${EXECUTABLE_SOURCE})
    target_link_libraries(${EXECUTABLE} 
        sub::MR
        sub::LR
        sub::SimRobot
        pybullet_cpp
    )
endforeach(EXECUTABLE)


file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/json_info/MR_info.json DESTINATION ${CMAKE_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/json_info/LR_info.json DESTINATION ${CMAKE_BINARY_DIR})