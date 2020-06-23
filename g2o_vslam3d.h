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
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;


class g2o_vslam3d
{
    g2o::SparseOptimizer optimizer;
    bool isDense;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    g2o::OptimizationAlgorithmLevenberg *solver;
}


    g2o_vslam3d::g2o_vslam3d()
    {
        //Now g2o uses c++11 smart pointer instead of raw pointer
        if (isDense)
            linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
        else
            linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();

        solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);
        vidx = 0;
        odix = 0;
        // set Camera Intrinsics
        g2o::CameraParameters *camera = new g2o::CameraParameters(fx, Eigen::Vector2d(cx, cy), 0);
        camera->setId(0);
        optimizer.addParameter(camera);

    }

    void g2o_vslam3d::addPoseVertex(bool isFixed = false)
    {
        g2o::VertexSE3Expmap *v = new g2o::VertexSE3Expmap();
        v->setId(vidx+oidx);
        v->setFixed(isFixed);
        v->setEstimate(g2o::SE3Quat());
        optimizer.addVertex(v);
        vidx++
    }
    
    g2o_vslam3d::addObservationVertexWithDepth(vector<cv::Point2f> pts, cv::Mat depthImg, bool isMarginalized = true)
    {
        for (size_t i = 0; i < pts.size(); i++)
        {
            g2o::VertexSBAPointXYZ *v = new g2o::VertexSBAPointXYZ();
            v->setId(vidx+oidx);
            //Pinhole model to set Initial Point Estimate
            double z = depthImg.at<float>(cvRound(pts[i].x),cvRound(pts[i].y));
            if(z < 0.01 || z > 5.0)
                z = 1;

            cout<<"Depth at "<<cvRound(pts1[i].x)<<" "<<cvRound(pts1[i].y)<<" is "<<z<<endl;
            double x = (pts[i].x - cx) * z / fx;
            double y = (pts[i].y - cy) * z / fy;
            v->setMarginalized(isMarginalized);
            v->setEstimate(Eigen::Vector3d(x, y, z));
            optimizer.addVertex(v);
            oidx++;
        }
    }

    g2o_vslam3d::addObservationVertex(vector<cv::Point2f> pts, bool isMarginalized = true)
    {
        for (size_t i = 0; i < pts.size(); i++)
        {
            g2o::VertexSBAPointXYZ *v = new g2o::VertexSBAPointXYZ();
            v->setId(vidx);
            //Pinhole model to set Initial Point Estimate
            double z=1;
            double x = (pts[i].x - cx) * z / fx;
            double y = (pts[i].y - cy) * z / fy;
            v->setMarginalized(isMarginalized);
            v->setEstimate(Eigen::Vector3d(x, y, z));
            optimizer.addVertex(v);
            vidx++;
        }
    }

    // edges == factors
    g2o_vslam3d::addObservationEdges(vector<cv::Point2f> pts, int vertexId)
    {
        vector<g2o::EdgeProjectXYZ2UV *> edges;
        for (size_t i = 0; i < pts.size(); i++)
        {
            g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
            edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(vidx + oidx)));
            edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(vertxId)));
            edge->setMeasurement(Eigen::Vector2d(pts[i].x, pts[i].y));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setParameterId(0, 0);
            edge->setRobustKernel(new g2o::RobustKernelHuber());
            optimizer.addEdge(edge);
            edges.push_back(edge);
        }
    }

    void g2o_vslam3d::solve(int num_iter = 10, bool verbose = false)
    {
        optimizer.setVerbose(verbose);
        optimizer.initializeOptimization();
        optimizer.optimize(num_iter);
    }

    void g2o_vslam3d::getPoseVertex(int vertexId)
    {
            g2o::VertexSE3Expmap *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(vertexId));
            Eigen::Isometry3d pose = v->estimate();
            cout << "Pose=" << endl << pose.matrix() << endl;

    }


     void g2o_vslam3d::getObservationVertex(int obsId)
     {
        g2o::VertexSBAPointXYZ *v = dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(i + num_images));
        Eigen::Vector3d pos = v->estimate();
        cout << "vertex id " << i + num_images << ", pos = ";
        cout << pos(0) << "," << pos(1) << "," << pos(2) << endl;
     }
    
    void g2o_vslam3d::getInliers()
    {
        int inliers = 0;
        for (auto e : edges)
        {
            e->computeError();
            // chi2 error > Pixel uncertainty
            if (e->chi2() > 1.5)
            {
                //cout << "error = " << e->chi2() << endl;
            }
            else
            {
                inliers++;
            }
        }
        cout << "inliers in total points: " << inliers << "/" << edges.size() << endl;
    }
  
}
