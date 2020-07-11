#include <g2o_vslam/g2o_vslam3d.h>

g2o_vslam3d::g2o_vslam3d(ros::NodeHandle nh_)
{
    nh = nh_;
    img_inc = false;
    frame = 0;
    //Now g2o uses c++11 smart pointer instead of raw pointer
    //isDense = false;
    //if (isDense)
    //    auto linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    //else
        auto linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);
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
    if(affine_list.size() == 16)
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
    }
    else
    {
        T_B_P.Identity();
    }
    
    q_B_P = Eigen::Quaterniond(T_B_P.linear());

    firstImageCb = true;
    keyframe = false;
    image_sub.subscribe(nh, image_topic, 1);
    depth_sub.subscribe(nh, depth_topic, 1);
    odom_sub = nh.subscribe(odom_topic, 1000, &g2o_vslam3d::odomCb, this);

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
    odom_pose = Eigen::Affine3d::Identity();
    addPoseVertex(odom_pose,true);//Initial Pose is anchored

}

void g2o_vslam3d::odomCb(const nav_msgs::OdometryConstPtr &odom_msg)
{
    odom_inc = true;
    Eigen::Vector3d t_ = Eigen::Vector3d(odom_msg->pose.pose.position.x,odom_msg->pose.pose.position.y,odom_msg->pose.pose.position.z);
    Eigen::Quaterniond q_ = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,odom_msg->pose.pose.orientation.y,odom_msg->pose.pose.orientation.z);
    
    t_ = T_B_P*t_;
    q_ = q_B_P* q_ * q_B_P.inverse();
    odom_pose.translation() = t_;
    odom_pose.linear() = q_.toRotationMatrix();
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
        
                //ROS_INFO("Image and Depth Cb");
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
                {
                    keyframe = true;
                    ROS_INFO("Keyframe");
                }
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
        cout<<"Camera Parameters"<<endl;
        cout<<"fx "<<fx<<" fy "<<fy<<" cx "<<cx<<" cy "<<cy<<endl;
}

int g2o_vslam3d::getObservationVertexId(int oidx_)
{
    return  oidx_map[oidx_];
}

int g2o_vslam3d::getPoseVertexId(int vidx_)
{
    return  vidx_map[vidx_];
}


void g2o_vslam3d::addPoseVertex(Eigen::Affine3d pose_, bool isFixed = false)
{
    g2o::VertexSE3Expmap *v = new g2o::VertexSE3Expmap();
    vidx++;
    idx++;
    vidx_map[vidx]=idx;
    v->setId(idx);
    v->setFixed(isFixed);
    v->setEstimate(g2o::SE3Quat(pose_.linear(),pose_.translation()));
    optimizer.addVertex(v);
}

void g2o_vslam3d::addPoseEdge(Eigen::Affine3d pose, Eigen::Matrix<double, 6, 6> cov, int vertexId)
{
    //add odometry edge
    g2o::EdgeSE3Expmap *odom=new g2o::EdgeSE3Expmap();
    // get two poses
    g2o::VertexSE3Expmap* vp0 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertices().find(getPoseVertexId(vertexId-1))->second);
    g2o::VertexSE3Expmap* vp1 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertices().find(getPoseVertexId(vertexId))->second);
    odom->setVertex(0,vp0);
    odom->setVertex(1,vp1);
    odom->setInformation(cov);
    odom->setParameterId(0, 0);
    odom->setMeasurement(g2o::SE3Quat(pose.linear(),pose.translation()));
    optimizer.addEdge(odom);
}


// void g2o_vslam3d::addObservationVertexWithDepth(cv::Point2f pts,  cv::Mat depthImg,  bool isMarginalized = true)
// {
//         g2o::VertexSBAPointXYZ *v = new g2o::VertexSBAPointXYZ();
//         oidx++;
//         idx++;
//         oidx_map[oidx]=idx;
//         v->setId(idx);
//         //Pinhole model to set Initial Point Estimate
//         double z = 1.0;
//         int uu = cvRound(pts.y);
//         int vv = cvRound(pts.x);
//         if((vv < width &&  vv>=0 && uu >=0 && uu<height))
//         {
//             z = depthImg.at<float>(uu, vv);
//             if (z < min_depth || z > max_depth || z!=z)
//                z = 1.0;
//         }


//         double x = (pts.x - cx) * z / fx;
//         double y = (pts.y - cy) * z / fy;
//         v->setMarginalized(isMarginalized);
//         v->setEstimate(Eigen::Vector3d(x, y, z));
//         optimizer.addVertex(v);

// }

Eigen::Vector3d g2o_vslam3d::projectuvXYZ(cv::Point2f pts, cv::Mat depthImg)
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

// void g2o_vslam3d::addObservationVertex(cv::Point2f pts,   bool isMarginalized = true)
// {
//         g2o::VertexSBAPointXYZ *v = new g2o::VertexSBAPointXYZ();
//         idx++;
//         oidx++;
//         oidx_map[oidx] = idx;
//         v->setId(idx);
//         //Pinhole model to set Initial Point Estimate
//         double z = 1;
//         double x = (pts.x - cx) * z / fx;
//         double y = (pts.y - cy) * z / fy;
//         v->setMarginalized(isMarginalized);
//         v->setEstimate(Eigen::Vector3d(x, y, z));
//         optimizer.addVertex(v);
 
// }
void g2o_vslam3d::addObservationVertex(Eigen::Vector3d pos_, bool isMarginalized = true)
{
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        oidx++;
        idx++;
        oidx_map[oidx]=idx;
        v->setId(idx);
        v->setEstimate(pos_);
        v->setMarginalized(isMarginalized);
        optimizer.addVertex(v);
}



// edges == factors
void g2o_vslam3d::addObservationEdges(cv::Point2f pts, Eigen::Matrix2d cov,int vertexId, int obsId)
{
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
        //g2o::VertexSBAPointXYZ* vp0 = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertices().find(getObservationVertexId(obsId))->second);
        //g2o::VertexSE3Expmap* vp1 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertices().find(getPoseVertexId(vertexId))->second);
        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertices().find(getObservationVertexId(obsId))->second));
        edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertices().find(getPoseVertexId(vertexId))->second));
        edge->setMeasurement(Eigen::Vector2d(pts.x, pts.y));
        edge->setInformation(cov);
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

Eigen::Affine3d g2o_vslam3d::getPoseVertex(int vertexId)
{
    int tmp_id = getPoseVertexId(vertexId);
    g2o::VertexSE3Expmap *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(tmp_id));
    Eigen::Isometry3d tmp_pose = v->estimate();
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.linear() = tmp_pose.linear();
    pose.translation() = tmp_pose.translation();
    //cout << "Pose=" << endl << pose.matrix() << endl;
    return pose;
}

Eigen::Vector3d g2o_vslam3d::getObservationVertex(int obsId)
{
    int tmp_id = getObservationVertexId(obsId);
    g2o::VertexSBAPointXYZ *v = dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(tmp_id));
    Eigen::Vector3d pos = v->estimate();
    //cout << "vertex id " << tmp_id << ", pos = ";
    //cout << pos(0) << "," << pos(1) << "," << pos(2) << endl;
    return pos;
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