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
)

add_definitions (-fpermissive -std=c++11)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_message_files(
   FILES
   CartesianPositionCommandMsg.msg
   CartesianInverseCommandMsg.msg
   HybridImpedanceCommandMsg.msg
   FtSensorToolEstimationMsg.msg
   CartesianPositionJointsMsg.msg
)

add_service_files(
  FILES
  CartesianPositionCommand.srv
  CartesianInverseCommand.srv
  HybridImpedanceCommand.srv
  FtSensorToolEstimation.srv
  FtSensorSetBias.srv
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
    controller_interface
    lwr_controllers
    kdl_conversions
    eigen_conversions
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/ft_sensor_controller.cpp
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
