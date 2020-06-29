#include <g2o_slam/g2o_slam3d.h>

g2o_slam3d::g2o_slam3d(ros::NodeHandle nh_)
{
    nh = nh_;
    img_inc = false;
    frame = 0;
    //Now g2o uses c++11 smart pointer instead of raw pointer
    // std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    // bool _isDense = false;
    // if(_isDense)
    //     linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    // else
    //     linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
    
    // g2o::OptimizationAlgorithmLevenberg *solver =  new OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
    // //g2o::OptimizationAlgorithmGaussNewton *solver =  new OptimizationAlgorithmGaussNewton(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));


    // create the linear solver
    auto linearSolver = g2o::make_unique<LinearSolverCSparse<BlockSolverX::PoseMatrixType>>();

    // create the block solver on top of the linear solver
    auto blockSolver = g2o::make_unique<BlockSolverX>(std::move(linearSolver));
    OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(std::move(blockSolver));


    optimizer.setAlgorithm(solver);



    vidx = -1;
    idx = -1;
    oidx = -1;

    ros::NodeHandle n_p("~");
    n_p.param<std::string>("image_topic", image_topic, "camera/rgb/image_rect_color");
    n_p.param<std::string>("depth_topic", depth_topic, "camera/depth_registered/sw_registered/image_rect");
    n_p.param<std::string>("cam_info_topic", cam_info_topic, "camera/rgb/camera_info");
    n_p.param<std::string>("odom_topic", odom_topic, "/kfusion/odom");

    n_p.param<bool>("mm_to_meters", mm_to_meters, false);
    n_p.param<int>("kf_rate", kf_rate, 50);
    n_p.param<double>("max_depth", max_depth, 6.0);
    n_p.param<double>("min_depth", min_depth, 0.01);
    std::vector<double> affine_list;
    n_p.getParam("T_B_P", affine_list);
    if(affine_list.size()>0)
    {
        T_B_P(0, 0) = affine_list[0];
        T_B_P(0, 1) = affine_list[1];
        T_B_P(0, 2) = affine_list[2];
        T_B_P(0, 3) = affine_list[3];
        T_B_P(1, 0) = affine_list[4];
        T_B_P(1, 1) = affine_list[5];
        T_B_P(1, 2) = affine_list[6];
        T_B_P(1, 3) = affine_list[7];
        T_B_P(2, 0) = affine_list[8];
        T_B_P(2, 1) = affine_list[9];
        T_B_P(2, 2) = affine_list[10];
        T_B_P(2, 3) = affine_list[11];
        T_B_P(3, 0) = affine_list[12];
        T_B_P(3, 1) = affine_list[13];
        T_B_P(3, 2) = affine_list[14];
        T_B_P(3, 3) = affine_list[15];
        T_B_P = T_B_P.inverse();
    }
    else
    {
        T_B_P.Identity();
    }
    
    q_B_P = Quaterniond(T_B_P.linear());

    firstImageCb = true;
    keyframe = false;
    image_sub.subscribe(nh, image_topic, 1);
    depth_sub.subscribe(nh, depth_topic, 1);
    odom_sub = nh.subscribe(odom_topic, 1000, &g2o_slam3d::odomCb, this);

    ts_sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_sub, depth_sub);

    ts_sync->registerCallback(boost::bind(&g2o_slam3d::imageDepthCb, this, _1, _2));

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

    //add the camera parameters, caches are automatically resolved in the addEdge calls
    g2o::ParameterSE3Offset* cameraOffset = new g2o::ParameterSE3Offset;
    cameraOffset->setId(0);
    optimizer.addParameter(cameraOffset);
    //Initialize graph with an Identity Affine TF
    addPoseVertex(Eigen::Affine3d::Identity(),true);//Initial Pose is anchored 
    odom_pose = Eigen::Affine3d::Identity();
    opt_pose = Eigen::Affine3d::Identity();
}

void g2o_slam3d::odomCb(const nav_msgs::OdometryConstPtr &odom_msg)
{
    odom_inc = true;
    Eigen::Vector3d t_ = Vector3d(odom_msg->pose.pose.position.x,odom_msg->pose.pose.position.y,odom_msg->pose.pose.position.z);
    Quaterniond q_ = Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,odom_msg->pose.pose.orientation.y,odom_msg->pose.pose.orientation.z);
    
    t_ = T_B_P*t_;
    q_ = q_B_P* q_ * q_B_P;
    odom_pose.translation() = t_;
    odom_pose.linear() = q_.toRotationMatrix();

}
int g2o_slam3d::findCorrespondingPoints(const cv::Mat &img1, const cv::Mat &img2, vector<cv::KeyPoint> &kp1, vector<cv::KeyPoint> &kp2, vector<cv::Point2f> &points1, vector<cv::Point2f> &points2, vector<cv::DMatch> &matches)
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

void g2o_slam3d::imageDepthCb(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::ImageConstPtr &depth_msg)
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


void g2o_slam3d::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg)
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

int g2o_slam3d::getObservationVertexId(int oidx_)
{
    return  oidx_map[oidx_];
}

int g2o_slam3d::getPoseVertexId(int vidx_)
{
    return  vidx_map[vidx_];
}


void g2o_slam3d::addPoseVertex(Eigen::Affine3d pose_, bool isFixed = false)
{

    g2o::VertexSE3 *v = new g2o::VertexSE3();
    vidx++;
    idx++;
    vidx_map[vidx]=idx;
    v->setId(idx);
    v->setFixed(isFixed);
    v->setEstimate(g2o::SE3Quat(pose_.linear(),pose_.translation()));
    optimizer.addVertex(v);
    getPoseVertex(vidx);
}

void g2o_slam3d::addObservationVertex(cv::Point2f pts, cv::Mat depthImg, bool isMarginalized = true)
{
        g2o::VertexPointXYZ* v = new g2o::VertexPointXYZ();
        oidx++;
        idx++;
        oidx_map[oidx]=idx;
        v->setId(idx);
        v->setEstimate(projectuvXYZ(pts,depthImg));
        v->setMarginalized(true);
        optimizer.addVertex(v);

}

Eigen::Vector3d g2o_slam3d::projectuvXYZ(cv::Point2f pts, cv::Mat depthImg)
{
    Eigen::Vector3d ret = Eigen::Vector3d::Zero();
    //Pinhole model to set Initial Point Estimate
    double z = 1.0;
    int uu = cvRound(pts.y);
    int vv = cvRound(pts.x);
    if ((vv < width && vv >= 0 && uu >= 0 && uu < height))
    {
        z = depthImg.at<float>(uu, vv);
        if (z < min_depth || z > max_depth || z != z)
            z = 1.0;
    }

    ret(0) = (pts.x - cx) * z / fx;
    ret(1) = (pts.y - cy) * z / fy;
    ret(2) = z;

    return ret;
}

void g2o_slam3d::addPoseEdge(Eigen::Affine3d pose, int vertexId)
{
    //add odometry edge
    g2o::EdgeSE3 *odom=new g2o::EdgeSE3();
    // get two poses
    VertexSE3* vp0 = dynamic_cast<VertexSE3*>(optimizer.vertices().find(getPoseVertexId(vertexId-1))->second);
    VertexSE3* vp1 = dynamic_cast<VertexSE3*>(optimizer.vertices().find(getPoseVertexId(vertexId))->second);
    odom->setVertex(0,vp0);
    odom->setVertex(1,vp1);
    odom->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
    odom->setParameterId(0, 0);
    odom->setMeasurement(g2o::SE3Quat(pose.linear(),pose.translation()));
    optimizer.addEdge(odom);
}

// edges == factors
void g2o_slam3d::addObservationEdges(Eigen::Vector3d p, int vertexId, int obsId)
{
        g2o::EdgeSE3PointXYZ *edge = new g2o::EdgeSE3PointXYZ();
        VertexPointXYZ* vp0 = dynamic_cast<VertexPointXYZ*>(optimizer.vertices().find(getObservationVertexId(obsId))->second);
        VertexSE3* vp1 = dynamic_cast<VertexSE3*>(optimizer.vertices().find(getPoseVertexId(vertexId))->second);
        edge->setVertex(0, vp1);
        edge->setVertex(1, vp0);
        edge->setMeasurement(p);
        edge->setInformation(Eigen::Matrix3d::Identity()*0.1);
        edge->setParameterId(0, 0);
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        edges.push_back(edge);
}

void g2o_slam3d::solve(int num_iter = 10, bool verbose = false)
{
    optimizer.setVerbose(verbose);
    optimizer.initializeOptimization();
    optimizer.optimize(num_iter);
}

Eigen::Affine3d g2o_slam3d::getPoseVertex(int vertexId)
{
    int tmp_id = getPoseVertexId(vertexId);
    g2o::VertexSE3 *v = dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(tmp_id));
    Eigen::Isometry3d tmp_pose = v->estimate();
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.linear() = tmp_pose.linear();
    pose.translation() = tmp_pose.translation();
    cout << "Pose=" << endl << pose.matrix() << endl;
    return pose;
}

Eigen::Vector3d g2o_slam3d::getObservationVertex(int obsId)
{
    int tmp_id = getObservationVertexId(obsId);
    g2o::VertexPointXYZ *v = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(tmp_id));
    Eigen::Vector3d pos = v->estimate();
    cout << "vertex id " << tmp_id << ", pos = ";
    cout << pos(0) << "," << pos(1) << "," << pos(2) << endl;
    return pos;
}

void g2o_slam3d::getInliers()
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

void g2o_slam3d::imdepthshow(cv::Mat map)
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