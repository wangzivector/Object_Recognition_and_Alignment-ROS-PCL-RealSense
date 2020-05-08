cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

find_package(catkin REQUIRED COMPONENTS 
dynamic_reconfigure
roscpp 
rostime 
sensor_msgs 
std_msgs 
geometry_msgs 
tf  
image_transport 
message_generation 
)
find_package(PCL 1.6.1 REQUIRED)
find_package(realsense2 2.29.0)

add_message_files(
  DIRECTORY msg
  FILES
  Shot352.msg
  Shot352_bundle.msg
	Shot1344.msg
	Shot1344_bundle.msg
	NARF.msg
	NARF_bundle.msg
  FPFH.msg
  FPFH_bundle.msg
)

generate_messages(DEPENDENCIES std_msgs object_recognition)

generate_dynamic_reconfigure_options(
  cfg/cconfig.cfg
  cfg/rconfig.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure roscpp rostime sensor_msgs std_msgs geometry_msgs tf image_transport message_runtime 
)


## Specify additional locations of header files

include_directories(
  include ${catkin_INCLUDE_DIRS}
    include
    # ${realsense2_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    )

set(DEPENDENCIES realsense2 ${PCL_LIBRARIES})


add_executable(kinect_pointcloud src/pcd_to_pointcloud.cpp)
# add_executable(object_recognition_node src/recognize.cpp)
# add_executable(pointcloud_descriptor src/pointcloud_to_descriptors.cpp)
# add_executable(filter_pointcloud src/segmentation.cpp)
# add_executable(NARF_descriptor src/depthimage_to_NARF.cpp)
set_property(TARGET kinect_pointcloud PROPERTY CXX_STANDARD 11)

add_dependencies(kinect_pointcloud  ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
# add_dependencies(pointcloud_descriptor ${PROJECT_NAME}_generate_messages_cpp)
# add_dependencies(object_recognition_node ${PROJECT_NAME}_generate_messages_cpp)
# add_dependencies(filter_pointcloud ${PROJECT_NAME}_generate_messages_cpp)
# add_dependencies(NARF_descriptor ${PROJECT_NAME}_generate_messages_cpp)

# link_directories(/usr/local/lib/)
target_link_libraries(kinect_pointcloud ${catkin_LIBRARIES} ${DEPENDENCIES} ${PCL_LIBRARIES} )
# target_link_libraries(object_recognition_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# target_link_libraries(pointcloud_descriptor ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# target_link_libraries(filter_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# target_link_libraries(NARF_descriptor ${catkin_LIBRARIES} ${PCL_LIBRARIES})


#install(TARGETS object_recognition_node pointcloud_descriptor filter_pointcloud NARF_descriptor
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


add_executable(feature_to_reco  src/feature_to_reco.cpp)

add_dependencies(feature_to_reco  ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(feature_to_reco  ${catkin_LIBRARIES}  ${DEPENDENCIES} ${PCL_LIBRARIES})


install(TARGETS kinect_pointcloud
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY pcds
				DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

