// for std
#include <iostream>
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


#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/edge_se3.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <eigen3/Eigen/Dense>
// #include <g2o_slam/boolStamped.h>
#include <key_frame_publisher/boolStamped.h>


#include<g2o_slam/Queue.h>

#include <fstream>
#include <map>
using namespace std;
using namespace g2o;
using namespace Eigen;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, key_frame_publisher::boolStamped> MySyncPolicy;

struct ImageData
{
    cv_bridge::CvImagePtr depth;
    cv_bridge::CvImagePtr rgb;
    int frame;
};

class g2o_slam3d
{
private:
    /// ROS nodehanlder
    ros::NodeHandle nh;
    //g2o optimizer
    g2o::SparseOptimizer optimizer;
    vector<g2o::EdgeSE3PointXYZ *> edges;
    ///image dimensions
    int height, width;
    /// camera distorsion
    double k1, k2, k3, t1, t2;
    /// camera calibration
    double cx, cy, fx, fy;
    //Queues    
    Queue<nav_msgs::Odometry> odom_data;
    Queue<ImageData> image_data;        
    
    //last frame send
    int key_frame;    
    /// Flags for first Image Callback, first Camera Info Callback, and for new image callback
    std::map<int,int> oidx_map, vidx_map;
    
    int vidx, oidx, idx;
    double min_depth, max_depth;
    
    //max frequency
    double freq;
    
    /// Topics
    std::string image_topic, depth_topic, cam_info_topic, odom_topic, key_frame_topic;
    
    Eigen::Affine3d T_B_P;
    Eigen::Quaterniond q_B_P;
    
    /// Functions
    Eigen::Affine3d rosOdomToAffine(const nav_msgs::Odometry &odom) const;
    
    bool findCorrespondingPoints(const cv::Mat &img1, 
                                 const cv::Mat &img2,
                                 vector<cv::KeyPoint> &kp1, 
                                 vector<cv::KeyPoint> &kp2, 
                                 vector<cv::Point2f> &points1, 
                                 vector<cv::Point2f> &points2, 
                                 vector<cv::DMatch> &matches);
    
    void addMatchesToGraph(const vector<cv::DMatch> &corr,
                                   Eigen::Affine3d &odom_pose,
                                   const vector<cv::Point2f> &pts1,
                                   const vector<cv::Point2f> &pts2,
                                   const cv::Mat &prevDepthImage,
                                   const cv::Mat &currDepthImage,
                                   bool odom_inc);
    
    void imageDepthCb(const sensor_msgs::ImageConstPtr &img_msg,
                      const sensor_msgs::ImageConstPtr &depth_msg,
                      const key_frame_publisher::boolStampedConstPtr &keyFrame );

    
    /// Threads
    void processingThread();
    
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    message_filters::Subscriber<key_frame_publisher::boolStamped> kf_sub;
    
    ros::Subscriber odom_sub;
    message_filters::Synchronizer<MySyncPolicy> *ts_sync;
    
    ros::Publisher opt_odom_pub, opt_odom_path_pub, opt_pt_pub, drop_kf_pub;
    std::thread output_thread, processing_thread;
    int max_num_kfs;
    bool exit;
  
public:
    void optimize();
    void run();    
    g2o_slam3d(ros::NodeHandle nh_, double rate,int max_kfs_num);
    
    void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);        

    void addPoseVertex(Eigen::Affine3d pose, bool isFixed);
    void addObservationVertex(Eigen::Vector3d pos_, bool isMarginalized);
    void addPoseEdge(Eigen::Affine3d pose, Eigen::Matrix<double, 6, 6> cov, int vertexId);
    // edges == factors
    void addObservationEdges(Eigen::Vector3d p, Eigen::Matrix3d cov, int vertexId, int obsId);

    void solve(int num_iter, bool verbose);

    Eigen::Affine3d getPoseVertex(int vertexId);

    Eigen::Vector3d getObservationVertex(int obsId);
    Eigen::Vector3d projectuvXYZ(cv::Point2f pts, cv::Mat depthImg);
    void getInliers();
    
    int getPoseVertexId(int vidx_);
    int getObservationVertexId(int oidx_);
    void imdepthshow(cv::Mat map);
    void odomCb(const  nav_msgs::OdometryConstPtr &odom_msg);

};
