cmake_minimum_required(VERSION 3.0)
project(pybullet_cpp_indy7)

find_package(jsoncpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${JSONCPP_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src/*")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../include")

add_subdirectory(bullet3)

file(GLOB_RECURSE PYBULLET_INDY7 src/Indy7.cpp src/Indy7.h)
file(GLOB_RECURSE MR ../include/MR/modern_robotics.h ../include/MR/modern_robotics.cpp)
file(GLOB_RECURSE relMR ../include/MR/modern_robotics_relative.h ../include/MR/modern_robotics_relative.cpp)
file(GLOB_RECURSE LR ../include/LieRobotics/lie_robotics.h ../include/LieRobotics/lie_robotics.cpp)
file(GLOB_RECURSE MR_Indy7 ../include/MR/MR_Indy7.h ../include/MR/MR_Indy7.cpp)
file(GLOB_RECURSE MR_Indy7_DualArm ../include/MR/MR_Indy7_DualArm.h ../include/MR/MR_Indy7_DualArm.cpp)
file(GLOB_RECURSE LR_Indy7 ../include/LieRobotics/LR_Indy7.h ../include/LieRobotics/LR_Indy7.cpp)

set(EXECUTABLES
    JointHinfControl
    TaskMotionControl
    HybridMotionForceControl
    ImpedanceControl
    ModernRoboticsTest
    DualArmTest
)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/MR_info.json DESTINATION ${CMAKE_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/LR_info.json DESTINATION ${CMAKE_BINARY_DIR})

foreach(EXECUTABLE ${EXECUTABLES})
    file(GLOB EXECUTABLE_SOURCE src/${EXECUTABLE}/*.cpp)
    add_executable(${EXECUTABLE} ${EXECUTABLE_SOURCE} ${MR_Indy7}  ${relMR} ${MR} ${MR_Indy7_DualArm} ${PYBULLET_INDY7} )
    target_link_libraries(${EXECUTABLE} PUBLIC pybullet_cpp)
    target_link_libraries(${EXECUTABLE} PUBLIC jsoncpp_lib)
endforeach(EXECUTABLE)


add_executable(LieRoboticsTest src/LieRoboticsTest/main.cpp ${LR_Indy7} ${LR} ${PYBULLET_INDY7} )
target_link_libraries(LieRoboticsTest PUBLIC pybullet_cpp)
target_link_libraries(LieRoboticsTest PUBLIC jsoncpp_lib)