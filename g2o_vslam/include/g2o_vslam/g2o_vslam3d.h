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
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
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
#include <fstream>
#include <map>
using namespace std;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

class g2o_vslam3d
{
private:
    g2o::SparseOptimizer optimizer;
    bool isDense;
    vector<g2o::EdgeProjectXYZ2UV *> edges;
    std::map<int,int> oidx_map, vidx_map;  
    ///image dimensions
    int height, width;
    /// camera distorsion
    double k1, k2, k3, t1, t2;
    /// camera calibration
    double cx, cy, fx, fy;
    /// Flags for first Image Callback, first Camera Info Callback, and for new image callback
    bool firstImageCb, firstCameraInfoCb, img_inc;
    // Flags for  checking VO initialization
    bool  mm_to_meters, isFirst;
    /// ROS nodehanlder
    ros::NodeHandle nh;

public:
    Eigen::Affine3d odom_pose, T_B_P;
    Eigen::Quaterniond q_B_P;
    int kf_rate;
    int vidx, oidx, idx;
    double min_depth, max_depth;
     /// current image frame
    int frame;
    ///flag to indicate a new odometry measurement
    bool odom_inc;
    ros::Subscriber odom_sub;

    g2o_vslam3d(ros::NodeHandle nh_);
    bool keyframe;
    ///placeholders for previous and current Grayscale/RGB/Depth Image
    cv::Mat currImage, prevImage, currImageRGB, prevDepthImage, currDepthImage;
    ///ROS RGB Image Subscriber
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    ///ROS DEPTH Image Subscriber
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    /// ROS Synchronization for RGB and DEPTH msgs
    message_filters::Synchronizer<MySyncPolicy> *ts_sync;
    /// ROS image, depth and camera info topics
    std::string image_topic, depth_topic, cam_info_topic, odom_topic;
    void imageDepthCb(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::ImageConstPtr &depth_msg);
    /** @fn void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);
     * @brief Camera Info Callback
     */
    void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);

    void addPoseVertex(Eigen::Affine3d pose, bool isFixed);
    int findCorrespondingPoints(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &kp1, vector<cv::KeyPoint> &kp2, vector<cv::Point2f> &points1, vector<cv::Point2f> &points2, vector<cv::DMatch> &matches);
    //void addObservationVertexWithDepth(cv::Point2f pts,  cv::Mat depthImg,  bool isMarginalized);

    void addObservationVertex(Eigen::Vector3d pos_, bool isMarginalized);
    // edges == factors
    void addObservationEdges(cv::Point2f pts, Eigen::Matrix2d cov, int vertexId, int obsId);
    void addPoseEdge(Eigen::Affine3d pose, Eigen::Matrix<double, 6, 6> cov, int vertexId);

    void solve(int num_iter, bool verbose);

    Eigen::Affine3d getPoseVertex(int vertexId);

    Eigen::Vector3d getObservationVertex(int obsId);
    Eigen::Vector3d projectuvXYZ(cv::Point2f pts, cv::Mat depthImg);

    void getInliers();
    
    int getPoseVertexId(int vidx_);
    int getObservationVertexId(int oidx_);
    void imdepthshow(cv::Mat map);
    void odomCb(const nav_msgs::OdometryConstPtr &odom_msg);


};
