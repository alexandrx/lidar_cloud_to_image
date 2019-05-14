cmake_minimum_required(VERSION 3.1)
project(lidar_cloud_to_image)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

cmake_policy(SET CMP0045 OLD)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL QUIET COMPONENTS common io)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})


find_package(catkin COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  cv_bridge
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS src
 CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_ros cv_bridge
 DEPENDS PCL
)

###########
## Build ##
###########

##cloud_projector_lib
add_library(cloud_projection SHARED
        include/angles.h
        include/sensor_params.h
        src/sensor_params.cpp
        include/projection_params.h
        src/projection_params.cpp
        include/pcl_point_types.h
        include/cloud_projection.h
        src/cloud_projection.cpp
        )
target_include_directories(cloud_projection PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        include
        )
add_dependencies(cloud_projection
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(cloud_projection
        ${catkin_LIBRARIES} 
        ${OpenCV_LIBS}
        ${PCL_LIBS}
        ${YAML_CPP_LIBRARIES}
        )

##cloud2image
add_executable(cloud2image 
        src/cloud_to_image_node.cpp
        include/cloud_to_image.h
        src/cloud_to_image.cpp
        )
target_include_directories(cloud2image PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        include
        )
target_link_libraries(cloud2image
        cloud_projection
        ${catkin_LIBRARIES} 
        ${OpenCV_LIBS}
        ${PCL_LIBS}
        )

##Tests
##test_angular_range
add_executable(test_angular_range
        tests/test_angular_range.cpp
        )
target_include_directories(test_angular_range PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        include
        )
target_link_libraries(test_angular_range
        cloud_projection
        ${catkin_LIBRARIES} 
        ${OpenCV_LIBS}
        ${PCL_LIBS}
        )
##test_sensor_params
add_executable(test_sensor_params
        tests/test_sensor_params.cpp
        )
target_include_directories(test_sensor_params PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        include
        )
target_link_libraries(test_sensor_params
        cloud_projection
        ${catkin_LIBRARIES} 
        ${OpenCV_LIBS}
        ${PCL_LIBS}
        )
##test_projection_params
add_executable(test_projection_params
        tests/test_projection_params.cpp
        )
target_include_directories(test_projection_params PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        include
        )
target_link_libraries(test_projection_params
        cloud_projection
        ${catkin_LIBRARIES} 
        ${OpenCV_LIBS}
        ${PCL_LIBS}
        )
##test_cloud_projection
add_executable(test_cloud_projection
        tests/test_cloud_projection.cpp
        )
target_include_directories(test_cloud_projection PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        include
        )
target_link_libraries(test_cloud_projection
        cloud_projection
        ${catkin_LIBRARIES} 
        ${OpenCV_LIBS}
        ${PCL_LIBS}
        )

install(TARGETS cloud2image cloud_projection
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        PATTERN ".svn" EXCLUDE)
