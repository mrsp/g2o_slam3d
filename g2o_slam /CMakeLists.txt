cmake_minimum_required(VERSION 2.8.3)
project(g2o_slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
)




list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(CSparse QUIET)  # Help g2o find SuiteSparse
find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)
LIST(APPEND G2O_LIBS
  cxsparse  
  g2o_cli g2o_ext_freeglut_minimal g2o_simulator
  g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
  g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
  g2o_types_sba g2o_types_slam3d g2o_csparse_extension
  g2o_opengl_helper g2o_solver_dense g2o_stuff
  g2o_types_sclam2d g2o_parser g2o_solver_pcg
  g2o_types_data g2o_types_sim3
)

include_directories( ${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR}    ${SUITESPARSE_INCLUDE_DIR})



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



catkin_package(
 INCLUDE_DIRS include
  LIBRARIES g2o_slam
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_filters nav_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}  
  ${CHOLMOD_INCLUDE_DIR} ${SUITESPARSE_INCLUDE_DIR}
)
## Declare a C++ executable
add_executable(g2o_slam3d src/g2o_slam3d_driver.cpp src/g2o_slam3d.cpp)

target_link_libraries(g2o_slam3d
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBS}
  ${G2O_LIBS}
  ${CHOLMOD_LIBRARIES})
