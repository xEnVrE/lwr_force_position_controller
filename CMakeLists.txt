cmake_minimum_required(VERSION 2.8.3)
project(lwr_force_position_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  kdl_conversions
  controller_interface
  lwr_controllers
  roscpp
  kdljacdot
  ft_calib
)

add_definitions (-fpermissive -std=c++11)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_message_files(
   FILES
   CartesianPositionCommandTrajMsg.msg
   CartesianPositionCommandGainsMsg.msg
   HybridImpedanceCommandTrajPosMsg.msg
   HybridImpedanceCommandTrajForceMsg.msg
   HybridImpedanceCommandGainsMsg.msg
   HybridImpedanceSwitchForcePosMsg.msg
   CartesianPositionJointsMsg.msg
)

add_service_files(
  FILES
  CartesianPositionCommandTraj.srv
  CartesianPositionCommandGains.srv
  HybridImpedanceCommandTrajPos.srv
  HybridImpedanceCommandTrajForce.srv
  HybridImpedanceCommandGains.srv
  HybridImpedanceSwitchForcePos.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  lwr_controllers
)

catkin_package(
  CATKIN_DEPENDS
    kdljacdot
    # ft_calib
    controller_interface
    lwr_controllers
    kdl_conversions
    eigen_conversions
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/ft_sensor_calib_controller.cpp
  src/cartesian_position_controller.cpp	
  src/cartesian_inverse_dynamics_controller.cpp	
  src/hybrid_impedance_controller.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES lwr_force_position_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
