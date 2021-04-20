cmake_minimum_required(VERSION 2.8.3)
project(lego_loam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs
  ndt_omp


)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
set(CMAKE_MODULE_PATH  ${PROJECT_SOURCE_DIR}/../Cmake/)

find_package(Glog REQUIRED)
find_package(Gflags REQUIRED)
find_package(GTest  REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cloud_msgs
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
    ${Glog_INLUCDE_DIRS}
     ${GFLAGS_INCLUDE_DIR}
)

link_directories(
  include
    ${GLOG_LIBRARIES}
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}


  ${GLOG_LIBRARIES}
  ${GTEST_LIBRARIES}

)

add_executable(imageProjection src/imageProjection.cpp)
add_dependencies(imageProjection ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(featureAssociation src/featureAssociation.cpp)
add_dependencies(featureAssociation ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(featureAssociation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(mapOptmization src/mapOptmization.cpp)
# target_link_libraries(mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)
add_executable(mapOptmization src/location_map.cpp)
 target_link_libraries(mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}
  ${GLOG_LIBRARIES}
   ${GFLAGS_LIBRARIES}
 gtsam)

add_executable(transformFusion src/transformFusion.cpp)
target_link_libraries(transformFusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(mapOptmization_gtest src/mapOptmization_gtest.cpp)
target_link_libraries(mapOptmization_gtest ${catkin_LIBRARIES}  gtsam -pthread   ${PCL_LIBRARIES}  ${GTEST_LIBRARIES} ${GLOG_LIBRARIES}
 gtest_main.a )