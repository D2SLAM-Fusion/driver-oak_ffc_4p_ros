cmake_minimum_required(VERSION 3.0.2)
project(oakcam_ffc_4p_ros)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -Wno-write-strings -Wno-format-overflow ")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  roscpp
  sensor_msgs
  depthai_ros_msgs
  depthai_bridge
  # depthai
)

find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

#Find opencv
find_package(OpenCV 4.2 REQUIRED )
# find_package(Sanitizers)

#Find depthai SDK
find_package(depthai CONFIG REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  /root/3rd_party/depthai-core-v2.26.0/build/
  /root/3rd_party/XLink/build
  /root/3rd_party/spdlog-1.8.2/build
  /root/3rd_party/fmt-7.1.3/build
)

add_executable(${PROJECT_NAME}_test test.cpp)

target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  depthai::opencv
  depthai-core
  XLink
  spdlog
  fmt::fmt
  pthread
)

target_include_directories(${PROJECT_NAME}_test PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /root/3rd_party/depthai-core-v2.26.0/build/install/include
  /root/3rd_party/XLink/build/install/include
)

# add_library(oakcam_ffc_4p_ros STATIC
#   src/ffc_4p_driver.cpp
# )

# # add_dependencies(oakcam_ffc_4p_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Declare a C++ executable
# ## With catkin_make all packages are built within a single CMake context
# ## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ffc_4p_node.cpp)
# add_dependencies(${PROJECT_NAME}_node oakcam_ffc_4p_ros  ${catkin_EXPORTED_TARGETS})

# target_link_libraries(oakcam_ffc_4p_ros
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   depthai::opencv
# )

# target_link_libraries(${PROJECT_NAME}_node
#   oakcam_ffc_4p_ros
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   depthai::opencv
# )
