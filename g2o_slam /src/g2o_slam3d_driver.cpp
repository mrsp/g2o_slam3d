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
using namespace std;



int main(int argc, char *argv[])
{

    ros::init(argc, argv, "g2o_slam3d");
    ros::NodeHandle n;
    g2o_slam3d bad(n);
    ros::NodeHandle n_p("~");
    double image_freq;
    n_p.param<double>("image_freq", image_freq, 100.0);
    static ros::Rate rate(2.0 * image_freq);
    while (ros::ok())
    {
        //if(bad.frame>501)
        //    break;

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

            continue;
        }

        cout << "Found " << corr.size() << " matches" << endl;
        // cv::Mat img_matched;
        // cv::drawMatches( bad.prevImage, kpts1,
        //                 bad.currImage, kpts2,
        //                 corr, img_matched,
        //                 cv::Scalar::all(-1),
        //                 cv::Scalar::all(-1),
        //                 std::vector<char>(),
        //                 cv::DrawMatchesFlags::DEFAULT);

        // cv::imshow("Knn Matches found", img_matched);
        // cv::waitKey(0);

        //vertices == nodes
        bad.addPoseVertex(false); //Unknown pose;
        // Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        // pose.translation()(0) = 0.25;
        // pose.translation()(1) = 0.25;
        // pose.translation()(2) = 0.25;
        // bad.addPoseEdge(pose,bad.vidx);
        // edges == factors
        for (unsigned int i = 0; i < corr.size(); i++)
        {
            bad.addObservationVertex(pts1[i], bad.prevDepthImage,true);
            bad.addObservationEdges(bad.projectuvXYZ(pts1[i], bad.prevDepthImage), bad.vidx-1, bad.oidx);
            bad.addObservationEdges(bad.projectuvXYZ(pts2[i], bad.currDepthImage), bad.vidx, bad.oidx);
        }
        bad.prevImage =  bad.currImage.clone();
        bad.prevDepthImage =  bad.currDepthImage.clone();
        bad.keyframe = false;
        break;
        ros::spinOnce();
        rate.sleep();
    }

    bad.solve(6, true); //6 iterations in G2O and verbose
    cout << " NUM OF POSE VERTICES " << bad.vidx << endl;
    cout << " NUM OF LANDMARK VERTICES " << bad.oidx << endl;
    for (unsigned int i = 0; i <=bad.vidx; i++)
    {
        bad.getPoseVertex(i);
    }

    return 0;
}
