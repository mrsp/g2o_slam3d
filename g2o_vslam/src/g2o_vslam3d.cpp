#include <g2o_vslam/g2o_vslam3d.h>

g2o_vslam3d::g2o_vslam3d(ros::NodeHandle nh_)
{
    nh = nh_;
    img_inc = false;
    frame = 1;
    //Now g2o uses c++11 smart pointer instead of raw pointer
    isDense = false;
    if (isDense)
        linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    else
        linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();

    solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);
    vidx = -1;
    idx = -1;
    oidx = -1;

    ros::NodeHandle n_p("~");
    n_p.param<std::string>("image_topic", image_topic, "camera/rgb/image_rect_color");
    n_p.param<std::string>("depth_topic", depth_topic, "camera/depth_registered/sw_registered/image_rect");
    n_p.param<std::string>("cam_info_topic", cam_info_topic, "camera/rgb/camera_info");
    n_p.param<bool>("mm_to_meters", mm_to_meters, false);
    n_p.param<int>("kf_rate", kf_rate, 50);

    firstImageCb = true;
    keyframe = false;
    image_sub.subscribe(nh, image_topic, 1);
    depth_sub.subscribe(nh, depth_topic, 1);

    ts_sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_sub, depth_sub);

    ts_sync->registerCallback(boost::bind(&g2o_vslam3d::imageDepthCb, this, _1, _2));

    // set Camera Intrinsics
    ROS_INFO("Waiting camera info");
    while (ros::ok())
    {
        sensor_msgs::CameraInfoConstPtr cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic);
        if (cam_info)
        {
            cameraInfoCb(cam_info);
            break;
        }
    }
    ROS_INFO("Camera info received");

    //Setting camera
    g2o::CameraParameters *camera = new g2o::CameraParameters(fx, Eigen::Vector2d(cx, cy), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    //Initialize graph with an Identity Affine TF
    addPoseVertex(true);//Initial Pose is anchored 
}




int g2o_vslam3d::findCorrespondingPoints(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &kp1, vector<cv::KeyPoint> &kp2, vector<cv::Point2f> &points1, vector<cv::Point2f> &points2, vector<cv::DMatch> &matches)
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

void g2o_vslam3d::imageDepthCb(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    img_inc = true;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge RGB exception: %s", e.what());
        return;
    }

    cv_bridge::CvImagePtr cv_depth_ptr;
    try
    {
        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge DEPTH exception: %s", e.what());
        return;
    }

    if (mm_to_meters)
        cv_depth_ptr->image *= 0.001;

    if(frame%kf_rate==0)
    {
        if (firstImageCb)
        {
        
                ROS_INFO("Image and Depth Cb");
                //prevImage = cv_ptr->image;
                if (cv_ptr->image.channels() == 3)
                {
                    cvtColor(cv_ptr->image, prevImage, cv::COLOR_BGR2GRAY);
                }
                else
                {
                    prevImage = cv_ptr->image;
                }
                prevDepthImage = cv_depth_ptr->image;
                firstImageCb = false;
            
        }
        else
        {
        
                ROS_INFO("Image and Depth Cb");
                currImageRGB = cv_ptr->image;
                if (cv_ptr->image.channels() == 3)
                {
                    cvtColor(cv_ptr->image, currImage, cv::COLOR_BGR2GRAY);
                }
                else
                {
                    currImage = cv_ptr->image;
                }
                currDepthImage = cv_depth_ptr->image;
                if (!keyframe)
                    keyframe = true;
            
        }
    }
    frame++;
}


void g2o_vslam3d::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg)
{
        height = msg->height;
        width = msg->width;

        k1 = msg->D[0];
        k2 = msg->D[1];
        t1 = msg->D[2];
        t2 = msg->D[3];
        k3 = msg->D[4];

        fx = msg->K[0];
        cx = msg->K[2];
        fy = msg->K[4];
        cy = msg->K[5];
}

int g2o_vslam3d::getObservationVertexId(int oidx_)
{
    return  oidx_map[oidx_];
}

int g2o_vslam3d::getPoseVertexId(int vidx_)
{
    return  vidx_map[vidx_];
}


void g2o_vslam3d::addPoseVertex(bool isFixed = false)
{
    g2o::VertexSE3Expmap *v = new g2o::VertexSE3Expmap();
    vidx++;
    idx++;
    vidx_map[vidx]=idx;
    v->setId(idx);
    v->setFixed(isFixed);
    v->setEstimate(g2o::SE3Quat());
    optimizer.addVertex(v);


}

void g2o_vslam3d::addObservationVertexWithDepth(cv::Point2f pts,  cv::Mat depthImg,  bool isMarginalized = true)
{
        g2o::VertexSBAPointXYZ *v = new g2o::VertexSBAPointXYZ();
        oidx++;
        idx++;
        oidx_map[oidx]=idx;
        v->setId(idx);
        //Pinhole model to set Initial Point Estimate
        double z = depthImg.at<float>(cvRound(pts.x), cvRound(pts.y));
        if (z < 0.01 || z > 5.0 || z!=z)
            z = 1;

        double x = (pts.x - cx) * z / fx;
        double y = (pts.y - cy) * z / fy;
        v->setMarginalized(isMarginalized);
        v->setEstimate(Eigen::Vector3d(x, y, z));
        optimizer.addVertex(v);

}

void g2o_vslam3d::addObservationVertex(cv::Point2f pts,   bool isMarginalized = true)
{
        g2o::VertexSBAPointXYZ *v = new g2o::VertexSBAPointXYZ();
        idx++;
        oidx++;
        oidx_map[oidx] = idx;
        v->setId(idx);
        //Pinhole model to set Initial Point Estimate
        double z = 1;
        double x = (pts.x - cx) * z / fx;
        double y = (pts.y - cy) * z / fy;
        v->setMarginalized(isMarginalized);
        v->setEstimate(Eigen::Vector3d(x, y, z));
        optimizer.addVertex(v);
 
}

// edges == factors
void g2o_vslam3d::addObservationEdges(cv::Point2f pts, int vertexId, int obsId)
{
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(getObservationVertexId(obsId))));
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(getPoseVertexId(vertexId))));
        edge->setMeasurement(Eigen::Vector2d(pts.x, pts.y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setParameterId(0, 0);
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        edges.push_back(edge);
}

void g2o_vslam3d::solve(int num_iter = 10, bool verbose = false)
{
    optimizer.setVerbose(verbose);
    optimizer.initializeOptimization();
    optimizer.optimize(num_iter);
}

void g2o_vslam3d::getPoseVertex(int vertexId)
{
    int tmp_id = getPoseVertexId(vertexId);
    g2o::VertexSE3Expmap *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(tmp_id));
    Eigen::Isometry3d pose = v->estimate();
    cout << "Pose=" << endl << pose.matrix() << endl;
}

void g2o_vslam3d::getObservationVertex(int obsId)
{
    int tmp_id = getObservationVertexId(obsId);
    g2o::VertexSBAPointXYZ *v = dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(tmp_id));
    Eigen::Vector3d pos = v->estimate();
    cout << "vertex id " << tmp_id << ", pos = ";
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

void g2o_vslam3d::imdepthshow(cv::Mat map)
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