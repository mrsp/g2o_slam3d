/*
 * rgbd_odom_driver.cpp.
 *
 * Written by: Stylianos Piperakis.
 *
 * This file launches the g2o_vslam3d module for Sparse BA witj Depth.
 */
#include <g2o_slam/g2o_slam3d.h>
// for opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#ifdef USE_CONTRIB
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#endif
// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
using namespace std;



int main(int argc, char *argv[])
{

    ros::init(argc, argv, "g2o_slam3d");

    ros::NodeHandle n_p;
    g2o_slam3d bad(n_p);
    
    
    // Run the spinner in a separate thread to prevent lockups
    bad.run();
    
    //Done here
    ROS_INFO("Quitting... ");
    return 0;  
        
}
