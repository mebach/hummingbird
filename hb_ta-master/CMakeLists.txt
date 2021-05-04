cmake_minimum_required(VERSION 2.8.3)
project(hb_ta)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  hb_common
  hb_msgs
)

# catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp rospy hb_common hb_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(pre_compiled_controller
  src/pid_controller.cpp
  src/pid_controller_node.cpp
)
add_dependencies(pre_compiled_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pre_compiled_controller
  ${catkin_LIBRARIES}
)

# catkin_install_python(PROGRAMS
#   scripts/my_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
