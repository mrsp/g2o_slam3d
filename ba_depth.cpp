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

int findCorrespondingPoints(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint>& kp1, vector<cv::KeyPoint>& kp2, vector<cv::Point2f> &points1, vector<cv::Point2f> &points2, vector<cv::DMatch> &matches);

void imdepthshow(cv::Mat map);

//Xtion parameters
double cx = 319.5;
double cy = 239.5;
double fx = 570.3422241210938;
double fy = 570.3422241210938;

//Intel parameters
// double cx = 317.23138427734375;
// double cy = 252.4551239013672;
// double fx = 612.3971557617188;
// double fy = 612.1465454101562;






int main(int argc, char **argv)
{
    if (argc != 5)
    {
        cout << "Usage: icp_ba img1, img2 detph1 depth2" << endl;
        return 0;
    }

    // Read the two input images and convert to grayscale
    cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

    cv::Mat depth1 = cv::imread(argv[3], cv::IMREAD_ANYDEPTH);
    cv::Mat depth2 = cv::imread(argv[4], cv::IMREAD_ANYDEPTH);
 
    imdepthshow(depth1);
    imdepthshow(depth2);
    depth1.convertTo(depth1, CV_32F,0.001);
    depth2.convertTo(depth1, CV_32F,0.001);

    int num_images = 2;
    // Keypoints placeholder
    vector<cv::Point2f> pts1, pts2;
    vector<cv::DMatch> corr;
    vector<cv::KeyPoint> kpts1,  kpts2;
    if (findCorrespondingPoints(img1, img2, kpts1, kpts2, pts1, pts2, corr) == false)
    {
        cout << "no matches found" << endl;
        return 0;
    }

    cout << "Found " << corr.size() << " matches" << endl;
    cv::Mat img_matched;
	cv::drawMatches( img1, kpts1, 
                    img2, kpts2, 
                    corr, img_matched, 
                    cv::Scalar::all(-1),
                    cv::Scalar::all(-1),
                    std::vector<char>(),
                    cv::DrawMatchesFlags::DEFAULT);


    cv::imshow("Knn Matches found", img_matched);
    cv::waitKey(0);
    g2o::SparseOptimizer optimizer;

    //Now g2o uses c++11 smart pointer instead of raw pointer
    bool _isDense = false;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    if (_isDense)
        linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    else
        linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    //vertices == nodes
    for (int i = 0; i < num_images; i++)
    {
        g2o::VertexSE3Expmap *v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if (i == 0)
            v->setFixed(true);          // anchor the first pose
        v->setEstimate(g2o::SE3Quat()); // set to Identity TF
        optimizer.addVertex(v);
    }

    for (size_t i = 0; i < pts1.size(); i++)
    {
        g2o::VertexSBAPointXYZ *v = new g2o::VertexSBAPointXYZ();
        v->setId(num_images + i);
        //Pinhole model to set Initial Point Estimate
        double z = depth1.at<float>(cvRound(pts1[i].x),cvRound(pts1[i].y));
        cout<<"Depth at "<<cvRound(pts1[i].x)<<" "<<cvRound(pts1[i].y)<<" is "<<z<<endl;

        if(z<0.01 || z>5.0 || z!=z)
           z=1.0;

        double x = (pts1[i].x - cx) * z / fx;
        double y = (pts1[i].y - cy) * z / fy;
        v->setMarginalized(true);
        v->setEstimate(Eigen::Vector3d(x, y, z));
        optimizer.addVertex(v);
    }

    // set Camera Intrinsics
    g2o::CameraParameters *camera = new g2o::CameraParameters(fx, Eigen::Vector2d(cx, cy), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    // edges == factors
    vector<g2o::EdgeProjectXYZ2UV *> edges;
    for (size_t i = 0; i < pts1.size(); i++)
    {
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(i + num_images)));
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0)));
        edge->setMeasurement(Eigen::Vector2d(pts1[i].x, pts1[i].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setParameterId(0, 0);
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }

    for (size_t i = 0; i < pts2.size(); i++)
    {
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(i + num_images)));
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(1)));
        edge->setMeasurement(Eigen::Vector2d(pts2[i].x, pts2[i].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setParameterId(0, 0);
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);



    for (size_t i = 0; i < pts1.size(); i++)
    {
        g2o::VertexSBAPointXYZ *v = dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(i + num_images));
        Eigen::Vector3d pos = v->estimate();
        //cout << "vertex id " << i + num_images << ", pos = ";
        //cout << pos(0) << "," << pos(1) << "," << pos(2) << endl;
    }

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

    cout << "inliers in total points: " << inliers << "/" << pts1.size() + pts2.size() << endl;
    g2o::VertexSE3Expmap *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(1));
    Eigen::Isometry3d pose = v->estimate();
    cout << "Pose=" << endl
         << pose.matrix() << endl;
    optimizer.save("ba.g2o");
    return 0;
}

int findCorrespondingPoints(const cv::Mat &img1, const cv::Mat &img2,vector<cv::KeyPoint>& kp1, vector<cv::KeyPoint>& kp2,  vector<cv::Point2f> &points1, vector<cv::Point2f> &points2, vector<cv::DMatch> &matches)
{
    cv::Ptr<cv::Feature2D> fdetector = cv::ORB::create(5000);

    cv::Mat desp1, desp2;
    fdetector->detectAndCompute(img1, cv::Mat(), kp1, desp1);
    fdetector->detectAndCompute(img2, cv::Mat(), kp2, desp2);

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    double knn_match_ratio = 0.8;
    vector<vector<cv::DMatch>> matches_knn;
    matcher->knnMatch(desp1, desp2, matches_knn, 2);
    for (size_t i = 0; i < matches_knn.size(); i++)
    {
        if (matches_knn[i][0].distance < knn_match_ratio * matches_knn[i][1].distance)
            matches.push_back(matches_knn[i][0]);
    }

    if (matches.size() <= 20)
        return false;

    for (auto m : matches)
    {
        points1.push_back(kp1[m.queryIdx].pt);
        points2.push_back(kp2[m.trainIdx].pt);
    }

    return true;
}

void imdepthshow(cv::Mat map)
{
    double min;
    double max;
    cv::minMaxIdx(map, &min, &max);
    cv::Mat adjMap;
    // Histogram Equalization
    float scale = 255 / (max-min);
    map.convertTo(adjMap,CV_8UC1, scale, -min*scale); 

    // this is great. It converts your grayscale image into a tone-mapped one, 
    // much more pleasing for the eye
    // function is found in contrib module, so include contrib.hpp 
    // and link accordingly
    cv::Mat falseColorsMap;
    applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_AUTUMN);

    cv::imshow("Depth", falseColorsMap);
    cv::waitKey(0);
}