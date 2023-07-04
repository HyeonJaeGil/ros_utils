cmake_minimum_required(VERSION 3.0.2)
project(ros_utils)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  pcl_conversions
  cv_bridge
)

catkin_package()

include_directories(
  ${PROJECT_SOURCE_DIR}/src/rosbag_utils
  ${PROJECT_SOURCE_DIR}/src/rostopic_utils
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

list(APPEND LIBS ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(concat_rosbag examples/concat_rosbag.cpp)
target_link_libraries(concat_rosbag ${LIBS})

add_executable(image_conversion examples/image_conversion.cpp)
target_link_libraries(image_conversion ${LIBS})
