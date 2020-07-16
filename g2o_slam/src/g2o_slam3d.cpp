#include <g2o_slam/g2o_slam3d.h>
#include <sensor_msgs/PointCloud.h>

g2o_slam3d::g2o_slam3d(ros::NodeHandle nh_,double rate,int max_kfs_num)
    :freq(rate),
    max_num_kfs(max_kfs_num),
    exit(false)
{
    nh = nh_;
    processing_frame = 0;
    send_frame_num =0;
    intput_frame = 0;
    
    
    // create the linear solver
    auto linearSolver = g2o::make_unique<LinearSolverCSparse<BlockSolverX::PoseMatrixType>>();

    // create the block solver on top of the linear solver
    auto blockSolver = g2o::make_unique<BlockSolverX>(std::move(linearSolver));
    OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(std::move(blockSolver));

    opt_odom_pub = nh.advertise<nav_msgs::Odometry>("vipGPU/odom", 1000);
    opt_pt_pub = nh.advertise<sensor_msgs::PointCloud>("vipGPU/pointcloud", 100);
    opt_odom_path_pub = nh.advertise<nav_msgs::Path>("vipGPU/odom/path", 100);
    
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
    
    q_B_P = Quaterniond(T_B_P.linear());
    
    //subscribers
    image_sub.subscribe(nh, image_topic, 1);
    depth_sub.subscribe(nh, depth_topic, 1);
    odom_sub = nh.subscribe(odom_topic, 1000, &g2o_slam3d::odomCb, this);
    ts_sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_sub, depth_sub);
    
    //publishers
    image_pub =  nh.advertise<sensor_msgs::Image>("g2o/rgb/image_raw",1);    
    depth_pub = nh.advertise<sensor_msgs::Image>("g2o/depth/image_raw",1);
    kf_pub =nh.advertise<g2o_slam::boolStamped>("g2o/isKeyframe",10);
    

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
    //odom_pose = Eigen::Affine3d::Identity();
    
    ts_sync->registerCallback(boost::bind(&g2o_slam3d::imageDepthCb, this, _1, _2));

}


/** Main Loop **/
void g2o_slam3d::run()
{
    output_thread = std::thread([this]{this->outputPublishThread();});
    processing_thread = std::thread([this]{this->processingThread();});
    ros::spin();
}

void g2o_slam3d::outputPublishThread()
{
    ros::Rate rate(4.0*freq);
    while (ros::ok() && !exit)
    {
        if(input_data.size() > 0 && is_key_frame_data.size() > 0)
        {
            ImageData img=input_data.pop();
            g2o_slam::boolStamped bool_msg=is_key_frame_data.pop();
            
            img.rgb->header.stamp = ros::Time::now();            
            img.rgb->header.seq = send_frame_num;
            img.depth->header.stamp = img.rgb->header.stamp;
            img.depth->header.seq = send_frame_num;
            
            bool_msg.header.stamp = img.rgb->header.stamp;
            bool_msg.header.seq = send_frame_num;
            
            image_pub.publish(img.rgb->toImageMsg());
            depth_pub.publish(img.depth->toImageMsg());
            kf_pub.publish(bool_msg);
            
            send_frame_num++;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
}

void g2o_slam3d::processingThread()
{
    //cv::Mat currImage, prevImage, currImageRGB, prevDepthImage, currDepthImage;
    
    static cv::Mat currImage, prevImage, currDepthImage, prevDepthImage;
    bool firstTime=true;
    ros::Rate rate(4.0*freq);
    while (ros::ok() && !exit)
    {        
        if(key_frames_data.size() == 0)
        {
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        
        std::cout<<"Key frame:"<<processing_frame<<" "<<std::endl;
        
        ImageData img=key_frames_data.pop();        
        if(firstTime)
        {
            std::cout<<"First time"<<std::endl;
            if (img.rgb->image.channels() == 3)
            {
                cvtColor(img.rgb->image, currImage, cv::COLOR_BGR2GRAY);
            }
            else
            {
                currImage = img.rgb->image.clone();
            }
            currDepthImage = img.depth->image.clone();
            firstTime=false;
        }
        else
        {
            vector<cv::Point2f> pts1, pts2;
            vector<cv::DMatch> corr;
            vector<cv::KeyPoint> kpts1, kpts2;
         
            cv::swap(currImage, prevImage);
            cv::swap(currDepthImage, prevDepthImage);
            
            cvtColor(img.rgb->image, currImage, cv::COLOR_BGR2GRAY);
            currDepthImage = img.depth->image.clone();
            
            if (doFindCorrespondingPoints(prevImage, currImage, kpts1, kpts2, pts1, pts2, corr) == false)
            {
                cout << "no matches found" << endl;  
            }
            else
            {
                bool has_odom;
                Eigen::Affine3d odom;
                if(odom_data.size()>0)
                {
                    odom=odom_data.backAndClear();
                    has_odom=true;
                }
                else
                {
                    has_odom=false;
                }
                    
                addMatchesToGraph(corr,odom,pts1,pts2,prevDepthImage,currDepthImage,true);
            }            
        }
        
      
        if(processing_frame>=max_num_kfs)
        {
            optimize();
            ros::Duration(0.25).sleep();
            exit=true;            
        }
                          
        processing_frame++;
        ros::spinOnce();
        rate.sleep();
    }

}

void g2o_slam3d::addMatchesToGraph(const vector<cv::DMatch> &corr,
                                   Eigen::Affine3d &odom_pose,
                                   const vector<cv::Point2f> &pts1,
                                   const vector<cv::Point2f> &pts2,
                                   const cv::Mat &prevDepthImage,
                                   const cv::Mat &currDepthImage,
                                   bool odom_inc)
{    
    cout << "Found " << corr.size() << " matches" << endl;

    Eigen::Affine3d pose_0 = getPoseVertex(vidx);
    if(odom_inc)
    {
        Eigen::Affine3d rel_odom_pose =   pose_0.inverse() * odom_pose;
        addPoseVertex(odom_pose,false);
        addPoseEdge(rel_odom_pose, Eigen::Matrix<double, 6, 6>::Identity(), vidx);
        //cout<<"Got New Odom "<<endl<<odom_pose.matrix()<<endl;
        //cout<<"Got New REL Odom "<<endl<<rel_odom_pose.matrix()<<endl;

    }
    else
    {
        addPoseVertex(Eigen::Affine3d::Identity(), false); //Unknown pose;
    }

    Eigen::Affine3d pose_1  =  getPoseVertex(vidx);
    Eigen::Vector3d pos1, rel_pos1, rel_pos0;
    // edges == factors
    for (unsigned int i = 0; i < corr.size(); i++)
    {
        rel_pos0 = projectuvXYZ(pts1[i], prevDepthImage);
        rel_pos1 = projectuvXYZ(pts2[i], currDepthImage);
        pos1 = pose_1 * rel_pos1;

        addObservationVertex(pos1,true);
        addObservationEdges(rel_pos0, Eigen::Matrix3d::Identity()*0.005, vidx-1, oidx);
        addObservationEdges(rel_pos1, Eigen::Matrix3d::Identity()*0.005, vidx, oidx);
    }
}

void g2o_slam3d::odomCb(const nav_msgs::OdometryConstPtr &odom_msg)
{
    Eigen::Vector3d t_ = Vector3d(odom_msg->pose.pose.position.x,odom_msg->pose.pose.position.y,odom_msg->pose.pose.position.z);
    Quaterniond q_ = Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,odom_msg->pose.pose.orientation.y,odom_msg->pose.pose.orientation.z);
    
    t_ = T_B_P*t_;
    q_ = q_B_P* q_ * q_B_P.inverse();
    Eigen::Affine3d odom_pose;
    odom_pose.translation() = t_;
    odom_pose.linear() = q_.toRotationMatrix();
    
    odom_data.push(odom_pose);
}

bool g2o_slam3d::doFindCorrespondingPoints(const cv::Mat &img1, 
                                        const cv::Mat &img2, 
                                        vector<cv::KeyPoint> &kp1, 
                                        vector<cv::KeyPoint> &kp2, 
                                        vector<cv::Point2f> &points1, 
                                        vector<cv::Point2f> &points2, 
                                        vector<cv::DMatch> &matches)
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

    if (matches.size() <= 120)
        return false;

    for (auto m : matches)
    {
        points1.push_back(kp1[m.queryIdx].pt);
        points2.push_back(kp2[m.trainIdx].pt);
    }

    return true;
}

void g2o_slam3d::imageDepthCb(const sensor_msgs::ImageConstPtr &img_msg,
                              const sensor_msgs::ImageConstPtr &depth_msg)
{
    ImageData data;
    try
    {
        data.rgb = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge RGB exception: %s", e.what());
        return;
    }
    
    try
    {
        data.depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge DEPTH exception: %s", e.what());
        return;
    }
    if (mm_to_meters)
        data.depth->image *= 0.001;
    
    data.frame=intput_frame;
    input_data.push(data);
    
    bool b=isKeyFrame(intput_frame);
    
    g2o_slam::boolStamped isKeyFrameMsg;
    isKeyFrameMsg.indicator.data=b;
    isKeyFrameMsg.dropPrev.data=false;
    is_key_frame_data.push(isKeyFrameMsg);
    
    intput_frame++;
    if(b)
        key_frames_data.push(data);
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

void g2o_slam3d::addObservationVertex(Eigen::Vector3d pos_, bool isMarginalized = true)
{
        g2o::VertexPointXYZ* v = new g2o::VertexPointXYZ();
        oidx++;
        idx++;
        oidx_map[oidx]=idx;
        v->setId(idx);
        v->setEstimate(pos_);
        v->setMarginalized(isMarginalized);
        optimizer.addVertex(v);

}

Eigen::Vector3d g2o_slam3d::projectuvXYZ(cv::Point2f pts, cv::Mat depthImg)
{
    Eigen::Vector3d ret = Eigen::Vector3d::Zero();
    //Pinhole model to set Initial Point Estimate
    double z = 1.0;
    int uu = cvRound(pts.x);
    int vv = cvRound(pts.y);
    if ((uu < width && uu >= 0 && vv >= 0 && vv < height))
    {
        z = depthImg.at<float>(vv, uu);
        if (z < min_depth || z > max_depth || z != z)
            z = 1.0;
    }

    ret(0) = (pts.x - cx) * z / fx;
    ret(1) = (pts.y - cy) * z / fy;
    ret(2) = z;

    return ret;
}

void g2o_slam3d::addPoseEdge(Eigen::Affine3d pose, Eigen::Matrix<double, 6, 6> cov, int vertexId)
{
    //add odometry edge
    g2o::EdgeSE3 *odom=new g2o::EdgeSE3();
    // get two poses
    VertexSE3* vp0 = dynamic_cast<VertexSE3*>(optimizer.vertices().find(getPoseVertexId(vertexId-1))->second);
    VertexSE3* vp1 = dynamic_cast<VertexSE3*>(optimizer.vertices().find(getPoseVertexId(vertexId))->second);
    odom->setVertex(0,vp0);
    odom->setVertex(1,vp1);
    odom->setInformation(cov);
    odom->setParameterId(0, 0);
    odom->setMeasurement(g2o::SE3Quat(pose.linear(),pose.translation()));
    optimizer.addEdge(odom);
}

// edges == factors
void g2o_slam3d::addObservationEdges(Eigen::Vector3d p, Eigen::Matrix3d cov, int vertexId, int obsId)
{
        g2o::EdgeSE3PointXYZ *edge = new g2o::EdgeSE3PointXYZ();
        VertexPointXYZ* vp0 = dynamic_cast<VertexPointXYZ*>(optimizer.vertices().find(getObservationVertexId(obsId))->second);
        VertexSE3* vp1 = dynamic_cast<VertexSE3*>(optimizer.vertices().find(getPoseVertexId(vertexId))->second);
        edge->setVertex(0, vp1);
        edge->setVertex(1, vp0);
        edge->setMeasurement(p);
        edge->setInformation(cov);
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
    //cout << "Pose=" << endl << pose.matrix() << endl;
    return pose;
}

Eigen::Vector3d g2o_slam3d::getObservationVertex(int obsId)
{
    int tmp_id = getObservationVertexId(obsId);
    g2o::VertexPointXYZ *v = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(tmp_id));
    Eigen::Vector3d pos = v->estimate();
    //cout << "vertex id " << tmp_id << ", pos = ";
    //cout << pos(0) << "," << pos(1) << "," << pos(2) << endl;
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

void g2o_slam3d::optimize()
{
    solve(10, true); //6 iterations in G2O and verbose
    cout << " NUM OF POSE VERTICES " << vidx << endl;
    cout << " NUM OF LANDMARK VERTICES " << oidx << endl;
    
    nav_msgs::Path opt_odom_path_msg;
    nav_msgs::Odometry opt_pose_msg;
    Eigen::Quaterniond opt_pose_q;
    Eigen::Affine3d opt_pose;
    Eigen::Vector3d opt_point;
    
    sensor_msgs::PointCloud pt_msg;
    
    for (unsigned int i = 0; i <=vidx; i++)
    {
        opt_pose = getPoseVertex(i);
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

    for (unsigned int i = 0; i <=oidx; i++)
    {
        opt_point = getObservationVertex(i);
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
}
