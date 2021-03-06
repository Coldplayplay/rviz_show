cmake_minimum_required(VERSION 2.8.3)
project(rviz_show)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  octomap_msgs
  message_generation
  #moveit_ros_perception
  #moveit_kinematics
  #octomap_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  DEPENDS roscpp rospy smach smach_ros std_msgs sensor_msgs
)

## Build ##
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})


#add_executable(rviz_show_pcd src/rviz_show_pcd.cpp)
#add_executable(rviz_show_octo src/rviz_show_octo.cpp)
#add_executable(rviz_show_octo_from_raw src/rviz_show_octo_from_raw.cpp)
#add_executable(rviz_show_octo_from_sensor src/rviz_show_octo_from_sensor.cpp)
#add_executable(show_octo_from_raw_downsampling_filter src/show_octo_from_raw_downsampling_filter.cpp)
#add_executable(test_cloud_from_zed src/test_cloud_from_zed.cpp)

#target_link_libraries(rviz_show_pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
#target_link_libraries(rviz_show_octo ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
#target_link_libraries(rviz_show_octo_from_raw ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(rviz_show_octo_from_sensor ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(show_octo_from_raw_downsampling_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(test_cloud_from_zed ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenCV_LIBS})

add_executable(rviz_show_octo_from_pcd src/rviz_show_octo_from_pcd.cpp)
add_dependencies(rviz_show_octo_from_pcd ${PROJECT_NAME}_gencpp)
target_link_libraries(rviz_show_octo_from_pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(pub_scene_from_bt src/pub_PlanningScene_from_octo.cpp)
target_link_libraries(pub_scene_from_bt ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
add_dependencies(pub_scene_from_bt ${PROJECT_NAME}_gencpp)

add_executable(pub_scene_from_pcd src/pub_PlanningScene_from_pcd.cpp)
target_link_libraries(pub_scene_from_pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
add_dependencies(pub_scene_from_pcd ${PROJECT_NAME}_gencpp)

add_executable(pub_tcp src/sendPose.cpp)
target_link_libraries(pub_tcp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(pub_tcp ${PROJECT_NAME}_gencpp)

