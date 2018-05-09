cmake_minimum_required(VERSION 2.8.3)
project(arm_controller)

SET( CMAKE_CXX_FLAGS "-std=c++11" )

find_package(catkin REQUIRED COMPONENTS
  dynamixel_workbench_toolbox
  dynamixel_workbench_msgs
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arm_controller
  CATKIN_DEPENDS dynamixel_workbench_toolbox dynamixel_workbench_msgs roscpp std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
        arm_controller
        src/arm_controller.cpp
	src/kinematics.cpp
	src/ArmState.cpp
	src/OffState.cpp
	src/WaitingState.cpp
	src/PauseState.cpp
	src/GoState.cpp
	src/GoSynchronizedState.cpp
        include/state_machine/Error.cpp
        include/state_machine/
)

add_dependencies(
        arm_controller
        ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries( arm_controller
  ${catkin_LIBRARIES}
)
