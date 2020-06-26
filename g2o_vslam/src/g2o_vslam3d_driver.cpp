/*
 * rgbd_odom_driver.cpp.
 *
 * Written by: Stylianos Piperakis.
 *
 * This file launches the g2o_vslam3d module for Sparse BA witj Depth.
 */
#include <g2o_vslam/g2o_vslam3d.h>
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

    ros::init(argc, argv, "g2o_vslam3d");
    ros::NodeHandle n;
    g2o_vslam3d bad(n);
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

        // edges == factors
        for (unsigned int i = 0; i < corr.size(); i++)
        {
            bad.addObservationVertexWithDepth(pts1[i], bad.prevDepthImage, true);
            //bad.addObservationVertex(pts1[i], true);
            bad.addObservationEdges(pts1[i], bad.vidx-1, bad.oidx);
            bad.addObservationEdges(pts2[i], bad.vidx, bad.oidx);
        }
        bad.prevImage =  bad.currImage.clone();
        bad.prevDepthImage =  bad.currDepthImage.clone();
        bad.keyframe = false;
        ros::spinOnce();
        rate.sleep();
    }

    bad.solve(10, true); //10 iterations in G2O and verbose
    cout << " NUM OF POSE VERTICES " << bad.vidx << endl;
    cout << " NUM OF LANDMARK VERTICES " << bad.oidx << endl;
    for (unsigned int i = 0; i <=bad.vidx; i++)
    {
        bad.getPoseVertex(i);
    }

    return 0;
}
