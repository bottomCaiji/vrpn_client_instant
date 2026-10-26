cmake_minimum_required(VERSION 2.8.3)
project(vrpn_client_instant)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp tf2_ros)
find_package(VRPN REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vrpn_client_instant
  CATKIN_DEPENDS geometry_msgs roscpp tf2_ros
  DEPENDS VRPN
)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${VRPN_INCLUDE_DIR}
)

add_library(vrpn_client_instant
  src/vrpn_client_instant.cpp
)
target_link_libraries(vrpn_client_instant
  ${catkin_LIBRARIES}
  ${VRPN_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(vrpn_tracker_instant_node
  src/vrpn_tracker_instant_node.cpp)
add_dependencies(vrpn_tracker_instant_node vrpn_client_instant)
target_link_libraries(vrpn_tracker_instant_node
  vrpn_client_instant
  ${catkin_LIBRARIES}
)

add_executable(vrpn_client_instant_node
  src/vrpn_client_instant_node.cpp)
add_dependencies(vrpn_client_instant_node vrpn_client_instant)
target_link_libraries(vrpn_client_instant_node
  vrpn_client_instant
  ${catkin_LIBRARIES}
)

install(TARGETS vrpn_client_instant vrpn_tracker_instant_node vrpn_client_instant_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_tracker_publisher test/tracker_publisher.test test/test_tracker_publisher.cpp)
  if(TARGET test_tracker_publisher)
    target_link_libraries(test_tracker_publisher ${catkin_LIBRARIES} ${VRPN_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
  endif()
  # C++11 threading is required, and namespace indentation follows this package's existing style.
  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,cpp" "--filter=-build/c++11,-runtime/indentation_namespace")
  roslint_cpp()
  roslint_add_test()
  roslaunch_add_file_check(launch)
endif()
