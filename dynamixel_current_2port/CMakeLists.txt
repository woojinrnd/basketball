cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_current_2port)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################

# set(dynamixel_current_2port src/dynamixel.cpp ${src/dynamixel.hpp})
# set(dynamixel_current_2port src/dynamixel_controller.cpp ${src/dynamixel_controller.hpp})
# set(dynamixel_current_2port src/callback.cpp ${src/callback.hpp})
set(dynamixel_current_2port src/sensor.cpp ${src/sensor.hpp})
set(dynamixel_current_2port src/Move_decision.cpp ${src/Move_decision.hpp})



find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamixel_sdk
  dynamixel_workbench_msgs
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(yaml-cpp REQUIRED)
# find_package(RBDL REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_service_files(
  FILES
  Select_Motion.srv
  Turn_Angle.srv
  UD_NeckAngle.srv
  RL_NeckAngle.srv
  Emergency.srv
  SendMotion.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs 
)


catkin_package(
  INCLUDE_DIRS include
    LIBRARIES dynamixel_current_2port
  CATKIN_DEPENDS
    cmake_modules 
    dynamixel_sdk 
    dynamixel_workbench_msgs 
    roscpp 
    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime
  DEPENDS 
    system_lib
    Boost
    OpenCV
)



################################################################################
# Build
################################################################################

## Specify additional locations of header files
## Your package locations should be listed before other locations

##Library
# add_library(dynamixel_current_2port src/dynamixel_controller.cpp src/callback.cpp src/dynamixel.cpp)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIR}
  ${dynamixel_workbench_msgs_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${realsense_INCLUDE_DIR}
  ./include
)

#dynamixel_current_2port node
add_executable(
  dynamixel_current_2port
  src/main.cpp
  # src/main_2.cpp
  src/dynamixel.cpp ${dynamixel.hpp}
  src/callback.cpp ${callback.hpp}
  src/dynamixel_controller.cpp ${dynamixel_controller.hpp}
  src/Walkingpattern_generator.cc ${Walkingpattern_generator.hpp}
  # src/sensor.cpp ${sensor.hpp}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_current_2port ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})


#Move_decision node
add_executable(
  Move_decision_node
  src/Move_decision_node.cpp
  # src/dynamixel.cpp ${dynamixel.hpp}
  src/Move_decision.cpp ${Move_decision.hpp} 
  # src/callback.cpp ${callback.hpp}
  # src/Walkingpattern_generator.cc ${Walkingpattern_generator.hpp}
  src/img_proc.cpp ${img_proc.hpp}
  # src/sensor.cpp ${sensor.hpp}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Move_decision_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${realsense2_LIBRARY} yaml-cpp)

#Sensor node
add_executable(
  Sensor_node
  src/Sensor_node.cpp
  src/sensor.cpp ${sensor.hpp}
  # src/img_proc_basketball.cpp
  # src/dynamixel.cpp ${dynamixel.hpp}
  # src/Move_decision.cpp ${Move_decision.hpp} 
  # src/callback.cpp ${callback.hpp}
  # src/Walkingpattern_generator.cc ${Walkingpattern_generator.hpp}
  # src/img_proc.cpp ${img_proc.hpp}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Sensor_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${realsense2_LIBRARY} yaml-cpp)


install(TARGETS dynamixel_current_2port Move_decision_node Sensor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


