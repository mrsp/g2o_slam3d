#include<g2o_slam3d.h>


#define MIN_LANDMARK_ID 1000
#define MIN_ODOM_ID 10000
g2o_slam3d::g2o_slam3d()
    :_isDense(false),
    poseId(0),
    bRobust(true),
    landmarkId(MIN_LANDMARK_ID),
    odomId(MIN_ODOM_ID),
    _params(params)
{
    nIterations=50;
}


void g2o_slam3d::addPoseConstrain(int p1,int p2,const sMatrix4 &pose, const Eigen::Matrix<double, 6, 6> &cov)
{
    g2o::VertexSE3 *v1=vertexes[p1];
    g2o::VertexSE3 *v2=vertexes[p2];


    //add odometry edge
    g2o::EdgeSE3 *odom=new g2o::EdgeSE3();
    odom->setId(odomId);
    odomId++;

    odom->setVertex(0,v1);
    odom->setVertex(1,v2);
    ///Sets the Information Matrix
    odom->setInformation(cov.inverse()); 
    g2o::SE3Quat isom=toG2oPose(pose);
    odom->setMeasurement(isom);

    odom->setParameterId(0, 0);
    if(bRobust)
    {
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        odom->setRobustKernel(rk);
        const float thHuber2D = sqrt(5.99);
        rk->setDelta(thHuber2D);
    }
    optimizer.addEdge(odom);
}

void g2o_slam3d::init(const sMatrix4 &initialPose, const sMatrix6 &cov)
{
    optimizer.setVerbose(true);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    if(_isDense)
        linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    else
         linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
    
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)) );
  
    optimizer.setAlgorithm(solver);

    g2o::ParameterSE3Offset* cameraOffset = new g2o::ParameterSE3Offset;
    cameraOffset->setId(0);
    optimizer.addParameter(cameraOffset);

    poseId=0;
    landmarkId=MIN_LANDMARK_ID;

    g2o::VertexSE3 *vert=new g2o::VertexSE3();
    vertexes.push_back(vert);
    vert->setFixed(true);

    g2o::SE3Quat g2oPose=toG2oPose(initialPose);
    vert->setEstimate(g2oPose);

    vert->setId(poseId);
    optimizer.addVertex(vert);
    poseId++;
    prevPose=initialPose;
    prevVertex=vert;
}

void g2o_slam3d::addFrame(const sMatrix4 &pose,const Eigen::Matrix<double, 6, 6> &cov)
{
    sMatrix4 delta=inverse(prevPose)*pose;
    prevPose=pose;

    g2o::VertexSE3 *vert=new g2o::VertexSE3();
    vert->setId(poseId);
    poseId++;
    vertexes.push_back(vert);    
    vert->setFixed(false);
    vert->setMarginalized(false);
    optimizer.addVertex(vert);    

    g2o::SE3Quat g2oPose=toG2oPose(pose);
    vert->setEstimate(g2oPose);

    //add odometry edge
    g2o::EdgeSE3 *odom=new g2o::EdgeSE3();
    odom->setId(odomId);
    odomId++;

    odom->setVertex(0,prevVertex);
    odom->setVertex(1,vert);



    odom->setInformation(cov.inverse());
    g2o::SE3Quat isom=toG2oPose(delta);
    odom->setMeasurement(isom);

    odom->setParameterId(0, 0);
    if(bRobust)
    {
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        odom->setRobustKernel(rk);
        const float thHuber2D = sqrt(5.99);
        rk->setDelta(thHuber2D);
    }
    optimizer.addEdge(odom);
    prevVertex=vert;

}

int g2o_slam3d::addLandmark(float3 pos)
{
    int id=landmarkId;
    float3 worldPose=prevPose*pos;    
    g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();

    vPoint->setEstimate(toG2oPoint(worldPose) );
    vPoint->setId(id);
    landmarkId++;
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    return id;
}

void g2o_slam3d::connectLandmark(float3 pos,int landIdx,int poseIdx, sMatrix3 &cov)
{
    if(poseIdx<0)
        poseIdx=poseId+poseIdx;

    g2o::EdgeSE3PointXYZ * e = new g2o::EdgeSE3PointXYZ();

    g2o::OptimizableGraph::Vertex *poseVert=optimizer.vertex(poseIdx);
    g2o::OptimizableGraph::Vertex *landmarkVert=optimizer.vertex(landIdx);

    auto vec=toG2oPoint(pos);

    e->setVertex(0,poseVert);
    e->setVertex(1,landmarkVert);
    //e->setMeasurement(toG2oPose(mat) );
    e->setMeasurement(vec);


    //Eigen::Matrix<double, 6, 6> eye= Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 3, 3> eigenCov= Eigen::Matrix<double, 3, 3>::Identity();

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            eigenCov(i,j)=cov(i,j);

    e->setInformation(eigenCov.inverse() );

    e->setParameterId(0, 0);
    if(bRobust)
    {
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        const float thHuber2D = sqrt(5.99);
        rk->setDelta(thHuber2D);
    }

    optimizer.addEdge(e);
}

void g2o_slam3d::connectLandmark(float2 pos, int lid, double Sigma2)
{

    std::cout<<"EDDSFSDF"<<std::endl;
//    return;
    Eigen::Matrix<double,2,1> obs;
    obs << pos.x, pos.y;

    //g2o::EdgeSE3 *e=new g2o::EdgeSE3();

    //Sigma2=1.0e-5;

    int vid=poseId-1;
    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
    g2o::OptimizableGraph::Vertex *v0=optimizer.vertex(vid);
    g2o::OptimizableGraph::Vertex *v1=optimizer.vertex(lid);
    e->setVertex(0,v0);
    e->setVertex(1,v1);
    e->setMeasurement(obs);
    e->setInformation(Eigen::Matrix2d::Identity()*(1/Sigma2) );
    
//    std::cout<<"Vid:"<<vid<<" Lid:"<<lid<<std::endl;
    
    if(bRobust)
    {
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        const float thHuber2D = sqrt(5.99);
        rk->setDelta(thHuber2D);
    }

    e->fx = _params.camera.x;
    e->fy = _params.camera.y;
    e->cx = _params.camera.z;
    e->cy = _params.camera.w;

    optimizer.addEdge(e);
    
}

void g2o_slam3d::clear()
{
    /*
    delete optimizer;
    optimizer=0;
    
    delete solver;
    solver=0;
    
    delete linearSolver;
    linearSolver=0;
    */
    optimizer.clear();
}

double g2o_slam3d::optimize(int frame)
{
    char buf[32];
    sprintf(buf,"graph_%d.g2o",frame);
    optimizer.save(buf);
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    double err=optimizer.activeChi2();
    std::cout<<"ERR:"<<err<<std::endl;

    sprintf(buf,"graph_%d_opt.g2o",frame);
    optimizer.save(buf);
    return err;
}

sMatrix4 g2o_slam3d::getPose(int id)
{
    g2o::VertexSE3* vSE3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(id));
    auto homogeneous = vSE3->estimate().matrix();

    sMatrix4 ret;
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            ret(i,j)=homogeneous(i,j);
        }
    }
    //return fromG2oPose(SE3quat);
    return ret;
}

uint g2o_slam3d::poseSize() const
{
    return poseId;
}

uint g2o_slam3d::landmarksSize() const
{
    return landmarkId-MIN_LANDMARK_ID;
}

//TODO
void g2o_slam3d::addFixPose(const sMatrix4 &fixPose) 
{

}

g2o::SE3Quat g2o_slam3d::toG2oPose(const sMatrix4 &pose)
{
    Eigen::Matrix3d rot; 
    Eigen::Vector3d trans;
    
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            rot(i,j)=pose(i,j);
        }
    }    
    
    for(int i=0;i<3;i++)
    {
        trans(i)=pose(i,3);
    }  
    
    return g2o::SE3Quat(rot,trans);
}
/*
g2o::SE3Quat g2o_slam3d::toG2oIsometry(const sMatrix4 &pose)
{
    return 
    Eigen::Matrix4d mat;

    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            mat(i,j)=pose(i,j);
        }
    }

    g2o::SE3Quat ret(mat);
    return ret;
}
*/

sMatrix4 g2o_slam3d::fromG2oPose(const g2o::SE3Quat &g2oPose)
{
    Eigen::Matrix<double,4,4> eigMat = g2oPose.to_homogeneous_matrix();
    sMatrix4 ret;
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            ret(i,j)=eigMat(i,j);
        }
    }    
    return ret;
}

Eigen::Matrix<double,3,1> g2o_slam3d::toG2oPoint(const float3 &vec)
{
    Eigen::Matrix<double,3,1> v;
    v << (double)vec.x, (double)vec.y, (double)vec.z;

    return v;
}
