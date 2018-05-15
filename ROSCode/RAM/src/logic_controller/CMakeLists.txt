cmake_minimum_required(VERSION 2.8.3)
project(logic_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  apriltags_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES logic_controller
  CATKIN_DEPENDS roscpp std_msgs apriltags_ros
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
	src/logic_controller.cpp
	src/trial.cpp
#	src/LogicState.cpp
#	src/WaitingState.cpp
#	src/LoadingState.cpp
#	src/VerifyState.cpp
	include/state_machine/Error.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)
