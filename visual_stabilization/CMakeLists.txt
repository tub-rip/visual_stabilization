cmake_minimum_required(VERSION 3.0.2)
project(visual_stabilization)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

cs_add_executable(frame_stabilization_node
  src/frame_stabilization.cpp
  src/camera_calibration.cpp
  src/camera_rotation_stabilizer.cpp
  src/vision_stabilizer.cpp
)
target_link_libraries(frame_stabilization_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES})

cs_add_executable(event_stabilization_node
   src/d_event.cpp
   src/event_stabilization.cpp
   src/camera_calibration.cpp
   src/event_windowing.cpp
   src/camera_rotation_stabilizer.cpp
   src/vision_stabilizer.cpp
)
target_link_libraries(event_stabilization_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES})

