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
    ros::NodeHandle n;
    

//     ros::Publisher opt_odom_pub = n.advertise<nav_msgs::Odometry>("vipGPU/odom", 1000);
//     ros::Publisher opt_pt_pub = n.advertise<sensor_msgs::PointCloud>("vipGPU/pointcloud", 100);
//     sensor_msgs::PointCloud pt_msg;
//     ros::Publisher opt_odom_path_pub = n.advertise<nav_msgs::Path>("vipGPU/odom/path", 100);
//     nav_msgs::Path opt_odom_path_msg;
//     nav_msgs::Odometry opt_pose_msg;
//     Eigen::Quaterniond opt_pose_q;
//     Eigen::Affine3d opt_pose;
//     Eigen::Vector3d opt_point;

    
    ros::NodeHandle n_p("~");
    double image_freq, odom_freq;
    int max_num_kfs;
    
    n_p.param<double>("image_freq", image_freq, 30.0);
    n_p.param<double>("odom_freq", odom_freq, 30.0);
    n_p.param<int>("max_num_kfs", max_num_kfs, 10);
    
    double freq = fmax(image_freq,odom_freq);
    
    g2o_slam3d bad(n,freq,max_num_kfs);
    
    ros::Duration(0.25).sleep();
    // Run the spinner in a separate thread to prevent lockups
    bad.run();
    
    //Done here
    ROS_INFO("Quitting... ");
//     delete gsp;
    return 0;  
    
    
    
    
    
    /*
    
    
    
    int keyframes = 0;
    int max_num_kfs;
    n_p.param<int>("max_num_kfs", max_num_kfs, 10);

    static ros::Rate rate(2.0 * freq);
    while (ros::ok())
    {
        
        if (!bad.keyframe)
        {
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        // Keypoints placeholder
        vector<cv::Point2f> pts1, pts2;
        vector<cv::DMatch> corr;
        vector<cv::KeyPoint> kpts1, kpts2;
        if (bad.findCorrespondingPoints(bad.prevImage, bad.currImage, kpts1, kpts2, pts1, pts2, corr) == false)
        {
            cout << "no matches found" << endl;
            bad.prevImage =  bad.currImage.clone();
            bad.prevDepthImage =  bad.currDepthImage.clone();
            bad.keyframe = false;
            ros::spinOnce();
            rate.sleep();
            continue;
        }



        cout << "Found " << corr.size() << " matches" << endl;


        //vertices == nodes
        Eigen::Affine3d pose_0  =  bad.getPoseVertex(bad.vidx);
        if(bad.odom_inc)
        {
            Eigen::Affine3d rel_odom_pose =   pose_0.inverse() * bad.odom_pose;
            bad.addPoseVertex(bad.odom_pose,false);
            bad.addPoseEdge(rel_odom_pose, Eigen::Matrix<double, 6, 6>::Identity(), bad.vidx);
            cout<<"Got New Odom "<<endl<<bad.odom_pose.matrix()<<endl;
            cout<<"Got New REL Odom "<<endl<<rel_odom_pose.matrix()<<endl;

            // Eigen::Affine3d pose = Eigen::Affine3d::Identity();
            // pose.translation()(0) = 0.25;
            // pose.translation()(1) = 0.25;
            // pose.translation()(2) = 0.25;
            //bad.addPoseEdge(pose,bad.vidx);
            bad.odom_inc = false;
        }
        else
            bad.addPoseVertex(Eigen::Affine3d::Identity(), false); //Unknown pose;



        Eigen::Affine3d pose_1  =  bad.getPoseVertex(bad.vidx);
        Eigen::Vector3d pos1, rel_pos1, rel_pos0;
        // edges == factors
        for (unsigned int i = 0; i < corr.size(); i++)
        {
            rel_pos0 = bad.projectuvXYZ(pts1[i], bad.prevDepthImage);
            rel_pos1 = bad.projectuvXYZ(pts2[i], bad.currDepthImage);
            pos1 = pose_1 * rel_pos1;

            bad.addObservationVertex(pos1,true);
            bad.addObservationEdges(rel_pos0, Eigen::Matrix3d::Identity()*0.005, bad.vidx-1, bad.oidx);
            bad.addObservationEdges(rel_pos1, Eigen::Matrix3d::Identity()*0.005, bad.vidx, bad.oidx);
        }
        bad.prevImage =  bad.currImage.clone();
        bad.prevDepthImage =  bad.currDepthImage.clone();
        bad.keyframe = false;

        keyframes++;
        if(keyframes>max_num_kfs)
            break;

        ros::spinOnce();
        rate.sleep();
    }

    bad.solve(10, true); //6 iterations in G2O and verbose
    cout << " NUM OF POSE VERTICES " << bad.vidx << endl;
    cout << " NUM OF LANDMARK VERTICES " << bad.oidx << endl;
    
    for (unsigned int i = 0; i <=bad.vidx; i++)
    {
        opt_pose = bad.getPoseVertex(i);
        opt_pose_msg.pose.pose.position.x = opt_pose.translation()(0);
        opt_pose_msg.pose.pose.position.y = opt_pose.translation()(1);
        opt_pose_msg.pose.pose.position.z = opt_pose.translation()(2);
        opt_pose_q = Eigen::Quaterniond(opt_pose.linear());
        opt_pose_msg.pose.pose.orientation.x = opt_pose_q.x();
        opt_pose_msg.pose.pose.orientation.y = opt_pose_q.y();
        opt_pose_msg.pose.pose.orientation.z = opt_pose_q.z();
        opt_pose_msg.pose.pose.orientation.w = opt_pose_q.w();
        opt_pose_msg.header.stamp = ros::Time::now();
        opt_pose_msg.header.frame_id = "odom";
        opt_odom_pub.publish(opt_pose_msg);

        geometry_msgs::PoseStamped tmp_pose;
        tmp_pose.pose = opt_pose_msg.pose.pose;
        tmp_pose.header = opt_pose_msg.header;
		opt_odom_path_msg.poses.push_back(tmp_pose);
        //ros::Duration(0.25).sleep();
    }
    opt_odom_path_msg.header = opt_pose_msg.header;
    opt_odom_path_pub.publish(opt_odom_path_msg);

    for (unsigned int i = 0; i <=bad.oidx; i++)
    {
        opt_point = bad.getObservationVertex(i);
        geometry_msgs::Point32 point;
        point.x = opt_point(0);
        point.y = opt_point(1);
        point.z = opt_point(2);
        pt_msg.points.push_back(point);
    }
    pt_msg.header.stamp = ros::Time::now();
    pt_msg.header.frame_id = "odom";
    opt_pt_pub.publish(pt_msg);

    ros::Duration(0.25).sleep();

    cout << " Exiting... "<< endl;

    

    return 0;
    */
}
