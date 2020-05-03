#include <cmath>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Imu.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
using namespace Eigen;

pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
pcl::PointCloud<pcl::PointXYZ>::Ptr LiDAR (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr LiDAR_Before (new pcl::PointCloud<pcl::PointXYZ>);
int i,j,k,l,m;
float GPS_x,GPS_y,GPS_z;
double T,scan_x,scan_y,scan_z,scan_roll,scan_pitch,scan_yaw;
Quaterniond qimu,qImu2Lidar,Final_Quaternion;
Vector3d Final_EulerAngle;
Matrix3d RM,RMImu2Lidar,Final_RotationMatrix;
Matrix4f Initial_Guess,Imu2Lidar,Final_Transformation,LiDAR_Move_Transformation;



void LiDARCallBack(const sensor_msgs::PointCloud2::ConstPtr& lidar){
  T = lidar->header.stamp.toSec();
  // new cloud formation 
  pcl::fromROSMsg (*lidar, *LiDAR);
  k=1;
//debug
i=i+1;
ROS_INFO("Frame times : %i",i);
}



void GPSCallBack(const geometry_msgs::PointStamped::ConstPtr& GPS){
  GPS_x = GPS->point.x;  GPS_y = GPS->point.y;  GPS_z = GPS->point.z;
}



void IMUCallBack(const sensor_msgs::Imu::ConstPtr& IMU){
  qimu.x() = IMU->orientation.x;  qimu.y() = IMU->orientation.y;  qimu.z() = IMU->orientation.z;  qimu.w() = IMU->orientation.w;
  //RM = qimu.toRotationMatrix();

double roll = 0.0244197711, pitch = -0.0387098077, yaw = -2.1993103971;
Quaterniond q;
q = AngleAxisd(roll, Vector3d::UnitX())
    * AngleAxisd(pitch, Vector3d::UnitY())
    * AngleAxisd(yaw, Vector3d::UnitZ());
RM = q.toRotationMatrix();

}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {
  //Initialize ROS
  ros::init (argc, argv, "Midterm_16_easy");
  ros::NodeHandle n;

  //open the result csv file
  std::ofstream myfile;
  myfile.open ((ros::package::getPath("midterm_16")+"/src/16_easy_P.csv").c_str());

  //set IMU2LiDAR transformation
  qImu2Lidar.x() = -0.0051505;  qImu2Lidar.y() = 0.018102;  qImu2Lidar.z() = -0.019207 ;  qImu2Lidar.w() = 0.99964;
  RMImu2Lidar = qImu2Lidar.toRotationMatrix();
  //RMImu2Lidar = RMImu2Lidar.inverse();
  Imu2Lidar << RMImu2Lidar(0,0) , RMImu2Lidar(0,1) , RMImu2Lidar(0,2) , 0.46 ,
               RMImu2Lidar(1,0) , RMImu2Lidar(1,1) , RMImu2Lidar(1,2) ,  0.0 ,
               RMImu2Lidar(2,0) , RMImu2Lidar(2,1) , RMImu2Lidar(2,2) , 3.46 ,
                      0         ,        0         ,        0         ,   1  ;
  //Imu2Lidar = Imu2Lidar.inverse();

  //load map pcd file
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_Map (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (ros::package::getPath("midterm_16")+"/src/map.pcd", *pcd_Map) == -1)
  {
    PCL_ERROR ("Couldn't read .pcd file\n");
    return (-1);
  }
//pcl::transformPointCloud (*pcd_Map, *pcd_Map, Imu2Lidar);

  //Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
  sensor_msgs::PointCloud2 map;
  pcl::toROSMsg (*pcd_Map, map);
    
  //subscribe and publish datas
  ros::Subscriber LiDAR_sub = n.subscribe ("/lidar_points", 300, LiDARCallBack);
  ros::Subscriber GPS_sub = n.subscribe ("/fix", 300, GPSCallBack);
  ros::Subscriber IMU_sub = n.subscribe ("/imu/data", 300, IMUCallBack);
  ros::Publisher Map_pub = n.advertise<sensor_msgs::PointCloud2> ("/map", 300);
  ros::Publisher Scan_pub = n.advertise<sensor_msgs::PointCloud2> ("/scan", 300);
  //set tf2 topic
  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped Scan;
  static tf2_ros::StaticTransformBroadcaster stfb;
  geometry_msgs::TransformStamped MAP;

  //publish the tf frame of map
  MAP.header.frame_id = "base_link";
  MAP.child_frame_id = "/map";
  MAP.transform.translation.x = 0.0;
  MAP.transform.translation.y = 0.0;
  MAP.transform.translation.z = 0.0;
  MAP.transform.rotation.x = 0.0;
  MAP.transform.rotation.y = 0.0;
  MAP.transform.rotation.z = 0.0;
  MAP.transform.rotation.w = 1.0;
  MAP.header.stamp = ros::Time::now();
  stfb.sendTransform(MAP);
/*
  //set icp algorithm
  icp.setMaxCorrespondenceDistance(0.1);
  icp.setTransformationEpsilon(1e-10);
  icp.setEuclideanFitnessEpsilon(0.001);
  icp.setMaximumIterations (10000);
*/
  ros::Rate r(30);
  while(ros::ok())
  {
    if (m<50){
      //publish the PointCloud2 of map
      map.header.stamp = ros::Time::now();
      map.header.frame_id = "base_link";
      Map_pub.publish(map);
      m=m+1;
ROS_INFO("Upload map : %i %%",m*2);
    }

    if (k==1 & qimu.x()!=0){
    if (l==0) {
      //set initial guess for icp
/*
      Initial_Guess << RM(0,0) , RM(0,1) , RM(0,2) , GPS_x,
                       RM(1,0) , RM(1,1) , RM(1,2) , GPS_y,
                       RM(2,0) , RM(2,1) , RM(2,2) , GPS_z,
                          0    ,    0    ,    0    ,   1  ;
*/
      Initial_Guess << RM(0,0) , RM(0,1) , RM(0,2) , -263.5926208496,
                       RM(1,0) , RM(1,1) , RM(1,2) , -67.8579025269,
                       RM(2,0) , RM(2,1) , RM(2,2) , -9.8907089233,
                          0    ,    0    ,    0    ,   1  ;
      LiDAR_Before = LiDAR;
    }
    else if (l==1) {
      icp.setInputSource(LiDAR_Before);
      icp.setInputTarget(LiDAR);
      pcl::PointCloud<pcl::PointXYZ> LiDAR_Move;
      icp.align(LiDAR_Move);
      LiDAR_Move_Transformation << icp.getFinalTransformation();
      Initial_Guess = LiDAR_Move_Transformation * Final_Transformation;
      LiDAR_Before = LiDAR;
    }

std::cout << "Initial : \n" << RM.eulerAngles(2, 1, 0)/M_PI*180 << std::endl;
    icp.setInputSource(LiDAR);
    icp.setInputTarget(pcd_Map);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final,Initial_Guess);
    Final_Transformation << icp.getFinalTransformation();
    Final_RotationMatrix << Final_Transformation(0,0),Final_Transformation(0,1),Final_Transformation(0,2),
                            Final_Transformation(1,0),Final_Transformation(1,1),Final_Transformation(1,2),
                            Final_Transformation(2,0),Final_Transformation(2,1),Final_Transformation(2,2);
    Final_Quaternion = Eigen::Quaterniond(Final_RotationMatrix);  Final_EulerAngle = Final_RotationMatrix.eulerAngles(2, 1, 0);
    scan_x = Final_Transformation(0,3);  scan_y = Final_Transformation(1,3);  scan_z = Final_Transformation(2,3);
    scan_yaw = Final_EulerAngle(0)-M_PI;      scan_pitch = Final_EulerAngle(1)-M_PI;    scan_roll = Final_EulerAngle(2)+M_PI;
    //Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 scan;
    pcl::toROSMsg(Final,scan);
std::cout << "Final : \n" << Final_EulerAngle/M_PI*180 << std::endl;

    //publish the tf frame of scan
    Scan.header.frame_id = "base_link";
    Scan.child_frame_id = "/scan";
    Scan.transform.translation.x = scan_x;
    Scan.transform.translation.y = scan_y;
    Scan.transform.translation.z = scan_z;
    Scan.transform.rotation.x = Final_Quaternion.x();
    Scan.transform.rotation.y = Final_Quaternion.y();
    Scan.transform.rotation.z = Final_Quaternion.z();
    Scan.transform.rotation.w = Final_Quaternion.w();
    Scan.header.stamp = ros::Time::now();
    tfb.sendTransform(Scan);

    //publish the PointCloud2 of LiDAR
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "base_link";
    Scan_pub.publish(scan);

    //output data to result csv file
    //myfile.precision(14);
    myfile << std::fixed << std::setprecision(15) << T << "," << scan_x << "," << scan_y << "," << scan_z << "," << scan_yaw << "," << scan_pitch << "," << scan_roll << "\n";
    k=0;
    l=1;
//debug
j=j+1;
ROS_INFO("output times : %i   yaw : %f",j,scan_yaw);
  }
    ros::spinOnce(); 
    r.sleep();
  }

  myfile.close();
  return 0;
}
