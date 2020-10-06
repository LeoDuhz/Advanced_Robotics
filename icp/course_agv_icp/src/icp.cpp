#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include<pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/filters/passthrough.h>  //直通滤波器头文件
#include<pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include<pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件

using namespace std;
using namespace Eigen;

// structure of the nearest neighbor
typedef struct{
    std::vector<float> distances;
    std::vector<int> src_indices;
    std::vector<int> tar_indices;
} NeighBor;

class icp{

public:

    icp(ros::NodeHandle &n);
    ~icp();
    ros::NodeHandle& n;

    // robot init states
    double robot_x;
    double robot_y;
    double robot_theta;
    // sensor states = robot_x_y_theta
    Vector3d sensor_sta;
      
    // max iterations
    int max_iter;
    // distance threshold for filter the matching points
    double dis_th;
    // tolerance to stop icp
    double tolerance;
    // if is the first scan, set as the map/target
    bool isFirstScan;
    // src point cloud matrix
    MatrixXd src_pc;
    // target point cloud matrix
    MatrixXd tar_pc;
    
    //////////////////////////////////
    // used for point cloud calc
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Matrix3d PtTransform;
    void addVecToCloud(Eigen::MatrixXd moved_pc);
   //////////////////////////////////

    // ICP process function
    void process(sensor_msgs::LaserScan input);
    // transform the ros msg to Eigen Matrix
    Eigen::MatrixXd rosmsgToEigen(const sensor_msgs::LaserScan input);
    // fint the nearest points & filter
    NeighBor findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar);
    // get the transform from two point sets in one iteration
    Eigen::Matrix3d getTransform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
    // calc 2D Euclidean distance
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
    // transform vector states to matrix form
    Eigen::Matrix3d staToMatrix(const Vector3d sta);

    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber laser_sub;
    void publishResult(Matrix3d T);
 	tf::TransformBroadcaster odom_broadcaster;
 	ros::Publisher odom_pub;
};

icp::~icp()
{}

icp::icp(ros::NodeHandle& n):
    n(n)
{
    //////////////////
    Eigen::Matrix3d PtTransform = Eigen::MatrixXd::Identity(3,3);
    //////////////////

    
	// get the params
	n.getParam("/icp/robot_x", robot_x);
	n.getParam("/icp/robot_y", robot_y);
	n.getParam("/icp/robot_theta", robot_theta);
	sensor_sta << robot_x, robot_y, robot_theta;

	n.getParam("/icp/max_iter", max_iter);
	n.getParam("/icp/tolerance", tolerance);
	n.getParam("/icp/dis_th", dis_th);

    isFirstScan = true;
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &icp::process, this);
    //laser_sub = n.subscribe("/scan", 1, &icp::process, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("icp_odom", 1);
}

 void icp::addVecToCloud(Eigen::MatrixXd moved_pc){
     cout << "5" << endl;
     pcl::PointXYZ point;
     for(int i = 0; i < moved_pc.row(0).size(); i++){
        // cout << "i: " << i << endl;
        point.x = moved_pc(0, i);
        point.y = moved_pc(1, i);
        point.z = 0;
        cloud.push_back(point);
     }
 }

void icp::process(sensor_msgs::LaserScan input)
{
    cout<<"------seq:  "<<input.header.seq<<endl;

    // set the inital
    if(isFirstScan)
    {
        tar_pc = this->rosmsgToEigen(input);
        isFirstScan =false;
        return;
    }

	double time_0 = (double)ros::Time::now().toSec();

    // init some variables
    NeighBor neigh_;
    src_pc = this->rosmsgToEigen(input);
    cout<<"input cnt:  "<<src_pc.cols()<<endl;

    Eigen::MatrixXd src_pc_xy;
    Eigen::MatrixXd tar_chorder;
  Eigen::MatrixXd tempsrc;

    Eigen::Matrix3d Transform;
    Eigen::Matrix3d Transform_acc = Eigen::MatrixXd::Identity(3,3);
    double prev_error = 0;
    double mean_error;
    int iter_cnt = 0;

    int match_cnt = 0;

    // main LOOP
    for(int i=0; i<max_iter; i++)
    {
    	cout<<"---iter: "<<i<<endl;

/////////////////////////////////////////////////////src_pc.row(0).size()
         pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);
         pcl::PointXYZ point;
     for(int i = 0; i <src_pc.row(0).size()  ; i++){
        point.x = src_pc(0,i);
        point.y = src_pc(1,i);
        point.z = 0;
        clouds->push_back(point);
        // clouds->points[i].x =src_pc(0,i);
        // clouds->points[i].y =src_pc(1,i);
        // clouds->points[i].z =0;
    }
    
//       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);//
      
// cout<<src_pc.row(0).size()<<endl;
//   pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;  //创建滤波器
  
//   radiusoutlier.setInputCloud(clouds);    //设置输入点云
   
//   radiusoutlier.setRadiusSearch(1);     //设置半径为100的范围内找临近点
   
//   radiusoutlier.setMinNeighborsInRadius(10); //设置查询点的邻域点集数小于2的删除
    
//   radiusoutlier.filter(*cloud_after_Radius);
//    cout<<"7!!"<<endl;
//   Eigen::MatrixXd tempsrc = Eigen::MatrixXd::Ones(3,cloud_after_Radius->points.size());
//  cout<<"8!!"<<endl;
//   for(int i =0;i<cloud_after_Radius->points.size();i++)
//   {tempsrc(0,i) =cloud_after_Radius->points[i].x;
//      tempsrc(1,i) =cloud_after_Radius->points[i].y;
//     }
//      cout<<cloud_after_Radius->points.size()<<endl;
//     src_pc=tempsrc;
// //////////////////////////////////////////////////////


        neigh_ = this->findNearest(src_pc.transpose(), tar_pc.transpose());
        //cout << neigh_.src_indices.size() << endl;

    	src_pc_xy = Eigen::MatrixXd::Ones(2, neigh_.src_indices.size());
    	tar_chorder = Eigen::MatrixXd::Ones(2, neigh_.src_indices.size());

        // filter ...
        for(int j=0; j<neigh_.src_indices.size(); j++)
        {
        		src_pc_xy.block<2,1>(0,j) = src_pc.block<2,1>(0, neigh_.src_indices[j]);
	            tar_chorder.block<2,1>(0,j) = tar_pc.block<2,1>(0, neigh_.tar_indices[j]);
        }
        // filter finished ...

        Transform = this->getTransform(src_pc_xy.transpose(), tar_chorder.transpose());

        Transform_acc = Transform * Transform_acc;

        src_pc = Transform * src_pc;

        mean_error = std::accumulate(neigh_.distances.begin(),neigh_.distances.end(),0.0)/neigh_.distances.size();

        if (abs(prev_error - mean_error) < tolerance)
            break;

        prev_error = mean_error;
    	iter_cnt ++;
    }
    cout<<"total_iter: "<<iter_cnt<<endl;

    tar_pc = this->rosmsgToEigen(input);

    this->publishResult(Transform_acc);

	double time_1 = (double)ros::Time::now().toSec();
	cout<<"time_cost:  "<<time_1-time_0<<endl;

    //////////////////////////////////////////////////////
    //Point Cloud map building
    // cout << "1" << endl;
    // Eigen::MatrixXd moved_pc = PtTransform * src_pc;
    // cout << "2" << endl;
    // addVecToCloud(moved_pc);
    // cout << "3" << endl;
    // PtTransform = PtTransform * Transform_acc;
    // cout << "4" << endl;
    //////////////////////////////////////////////////////
}

Eigen::MatrixXd icp::rosmsgToEigen(const sensor_msgs::LaserScan input)
{
    int total_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3,total_num);

    float angle;
    for(int i=0; i<total_num; i++)
    {
        angle = input.angle_min + i * input.angle_increment;

        pc(0,i) = input.ranges[i] * std::cos(angle);
        pc(1,i) = input.ranges[i] * std::sin(angle);
    }
    return pc;
}

NeighBor icp::findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{
   
     
    // temp cpy
    int row_src = src.rows();
    int row_tar = tar.rows();
    Eigen::Vector2d vec_src;
    Eigen::Vector2d vec_tar;

    NeighBor neigh;
    float min = 100;
    int index = 0;
    float dist_temp = 0;

    for(int ii=0; ii < row_src; ii++){
        vec_src = src.block<1,2>(ii,0).transpose();
        min = 100;
        index = 0;
        dist_temp = 0;
        for(int jj=0; jj < row_tar; jj++){
            vec_tar = tar.block<1,2>(jj,0).transpose();
            dist_temp = calc_dist(vec_src,vec_tar);
            if (dist_temp < min){
                min = dist_temp;
                index = jj;
            }
        }

        //filter added
        if(min < dis_th)
        {
        	neigh.distances.push_back(min);
        	neigh.src_indices.push_back(ii);
        	neigh.tar_indices.push_back(index);
        }
    }
    return neigh;
}

Eigen::Matrix3d icp::getTransform(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{
    Eigen::Matrix3d T = Eigen::MatrixXd::Identity(3, 3);
    
    
    //#TODO: Please complete this function by yourself
	Vector2d p_src, p_tar;

    p_src(0) = src.col(0).mean();
    p_src(1) = src.col(1).mean();
    p_tar(0) = tar.col(0).mean();
    p_tar(1) = tar.col(1).mean();
    
    //test output
    // cout << "src:" << src << endl;
    // cout << "tar:" << tar << endl;  
    // cout << "p_src:" << p_src << endl;
    // cout << "p_tar:" << p_tar << endl;

    
    // cout << "src1" << p_src<1,2>(0,0) << endl;
    // cout << "tar1:" << p_tar<1,2>(0,0) << endl;

    int N = src.col(0).size();
    
    Eigen::MatrixXd demean_src(N, 2);
    Eigen::MatrixXd demean_tar(N, 2);

    for(int i = 0; i < N; i++){
        demean_src(i, 0) = src(i, 0) - p_src(0, 0);
        demean_src(i, 1) = src(i, 1) - p_src(1, 0);
        demean_tar(i, 0) = tar(i, 0) - p_tar(0, 0);
        demean_tar(i, 1) = tar(i, 1) - p_tar(1, 0);
    }

    //test output
    // cout << "demean_src:" << demean_src << endl;
    // cout << "demean_tar:" << demean_tar << endl;

    Eigen::Matrix2d matrixW;
    matrixW = demean_src.transpose() * demean_tar;

    //test output
    cout << "matrixW: " << matrixW << endl;

	Eigen::JacobiSVD<Eigen::Matrix2d> svd(matrixW, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix2d matrixU = svd.matrixU();
    Eigen::Matrix2d matrixV = svd.matrixV();

    if(matrixU.determinant() * matrixV.determinant() < 0){
        for(int x = 0; x < 2; x++){
            matrixU(x, 1) *= -1;
        }
    }
    
    // test output
    cout << "U=" << matrixU << endl;
    cout << "V=" << matrixV << endl;
    //test end

    Eigen::Matrix2d Rstar2d;
    Eigen::Vector2d tstar2d;
    Rstar2d = matrixV * matrixU.transpose();
    tstar2d = p_tar - Rstar2d * p_src;

    //test output
    cout << "Rstar2d: " << Rstar2d << endl;
    cout << "tstar2d: " << tstar2d << endl;

	T.block<2, 2>(0, 0) = Rstar2d;
    T.block<2, 1>(0, 2) = tstar2d;

    //test output
    cout << "T=" << T << endl;
    return T;
}

float icp::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{
    return std::sqrt((pta[0]-ptb[0])*(pta[0]-ptb[0]) + (pta[1]-ptb[1])*(pta[1]-ptb[1]));
}

Eigen::Matrix3d icp::staToMatrix(Eigen::Vector3d sta)
{
	Matrix3d RT;
    RT << cos(sta(2)), -sin(sta(2)), sta(0),
          sin(sta(2)), cos(sta(2)),sta(1),
          0, 0, 1;
    return RT;
}

void icp::publishResult(Eigen::Matrix3d T)
{
    float delta_yaw = atan2(T(1,0), T(0,0));
    cout<<"sensor-delta-xyt: "<<T(0,2)<<" "<<T(1,2)<<" "<<delta_yaw<<endl;

    sensor_sta(0) = sensor_sta(0) + cos(sensor_sta(2))*T(0,2) - sin(sensor_sta(2))*T(1,2);
    sensor_sta(1) = sensor_sta(1) + sin(sensor_sta(2))*T(0,2) + cos(sensor_sta(2))*T(1,2);
    sensor_sta(2) = sensor_sta(2) + delta_yaw;

    cout<<"sensor-global: "<<sensor_sta.transpose()<<endl;

    // tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(sensor_sta(2));

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "world_base";
    odom_trans.child_frame_id = "icp_odom";

    odom_trans.transform.translation.x = sensor_sta(0);
    odom_trans.transform.translation.y = sensor_sta(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    // odom
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world_base";

    odom.pose.pose.position.x = sensor_sta(0);
    odom.pose.pose.position.y = sensor_sta(1);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp");
    ros::NodeHandle n;

    icp icp_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();
    cout<<"before saved pcd!"<<endl;
    pcl::io::savePCDFile("/home/leodu/icpmap1.pcd", icp_.cloud);
    cout << "mapsaved" << endl;

    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // pcl::PointXYZ point;
    // for(int i = 0; i < 100; i++){
    //     cout << "i: " << i << endl;
    //     for(int j = 0; j < 100; j++){
    //         point.x =  i;
    //         point.y =10*j;
    //         point.z = 0;
    //         cloud.push_back(point);            
    //     }
    // }
    // pcl::io::savePCDFile("/home/leodu/icpmap.pcd", cloud);

    // pcl::io::loadPCDFile ("/home/leodu/icpmap.pcd", cloud);
    // for(int k = 0; k < (int)cloud.size(); k++){
    //     cout << "x: " << cloud[k].x << "y: " << cloud[k].y << "z" << cloud[k].z << endl;
    // }



    return 0;
}