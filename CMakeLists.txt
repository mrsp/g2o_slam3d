cmake_minimum_required( VERSION 2.8 )
project( g2o_icp_ba )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
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

add_executable( icp_ba icp_ba.cpp )
target_link_libraries( icp_ba 
    ${OpenCV_LIBS}
    ${G2O_LIBS}
    ${CHOLMOD_LIBRARIES}
    )
