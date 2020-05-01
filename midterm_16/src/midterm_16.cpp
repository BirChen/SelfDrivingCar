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

pcl::PointCloud<pcl::PointXYZ>::Ptr LiDAR (new pcl::PointCloud<pcl::PointXYZ>);
int i,j,k;
float GPS_x,GPS_y,GPS_z;
double T,scan_x,scan_y,scan_z,scan_roll,scan_pitch,scan_yaw;
Quaterniond qimu,qImu2Lidar,Final_Quaternion;
Vector3d Final_EulerAngle;
Matrix3d T1,T2,RM,RMImu2Lidar,Final_RotationMatrix;
Matrix4f Initial_Guess,Imu2Lidar,Final_Transformation;

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
  RM = qimu.toRotationMatrix();
/*
float roll =  * M_PI / 180, pitch = 0 * M_PI / 180, yaw = 0.707 * M_PI / 180;    
Quaternionf q;
q = AngleAxisf(roll, Vector3f::UnitX())
    * AngleAxisf(pitch, Vector3f::UnitY())
    * AngleAxisf(yaw, Vector3f::UnitZ());
std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
*/
/*
  T1 << 0.0225226, 0.999745, 0.0017194,
        0.0648765, -0.00317777, 0.997888,
        0.997639, -0.0223635, -0.0649315;
  T2 << 0, 0, 1,
       -1, 0, 0,
        0,-1, 0;
  RM = T2*T1*RM;
*/
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {
  //Initialize ROS
  ros::init (argc, argv, "Midterm_16");
  ros::NodeHandle n;

  //open the result csv file
  std::ofstream myfile;
  myfile.open ((ros::package::getPath("midterm_16")+"/src/16_easy_P.csv").c_str());

  //set IMU2LiDAR transformation
  qImu2Lidar.x() = -0.0051505;  qImu2Lidar.y() = 0.018102;  qImu2Lidar.z() = -0.019207 ;  qImu2Lidar.w() = 0.99964;
  RMImu2Lidar = qImu2Lidar.toRotationMatrix();
  Imu2Lidar << RMImu2Lidar(0,0) , RMImu2Lidar(0,1) , RMImu2Lidar(0,2) , 0.46 ,
               RMImu2Lidar(1,0) , RMImu2Lidar(1,1) , RMImu2Lidar(1,2) ,  0.0 ,
               RMImu2Lidar(2,0) , RMImu2Lidar(2,1) , RMImu2Lidar(2,2) , 3.46 ,
                      0         ,        0         ,        0         ,   1  ;

  //load map pcd file
  pcl::PointCloud<pcl::PointXYZ>::Ptr Map (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (ros::package::getPath("midterm_16")+"/src/map.pcd", *Map) == -1)
  {
    PCL_ERROR ("Couldn't read .pcd file\n");
    return (-1);
  }

  //Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
  sensor_msgs::PointCloud2 map;
  pcl::toROSMsg (*Map, map);
    
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

  ros::Rate r(300);
  while(ros::ok())
  {
    //set initial guess for icp
    Initial_Guess << RM(0,0) , RM(0,1) , RM(0,2) , GPS_x,
                     RM(1,0) , RM(1,1) , RM(1,2) , GPS_y,
                     RM(2,0) , RM(2,1) , RM(2,2) , GPS_z,
                        0    ,    0    ,    0    ,   1  ;
    //Initial_Guess = Imu2Lidar * Initial_Guess;

    //set icp algorithm
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
/*
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setTransformationEpsilon(1e-9);
    icp.setEuclideanFitnessEpsilon(0.01);
    icp.setMaximumIterations (10000);
*/
    icp.setInputSource(LiDAR);
    icp.setInputTarget(Map);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final,Initial_Guess);
    Final_Transformation << icp.getFinalTransformation();
    Final_RotationMatrix << Final_Transformation(0,0),Final_Transformation(0,1),Final_Transformation(0,2),
                            Final_Transformation(1,0),Final_Transformation(1,1),Final_Transformation(1,2),
                            Final_Transformation(2,0),Final_Transformation(2,1),Final_Transformation(2,2);
    Final_Quaternion = Eigen::Quaterniond(Final_RotationMatrix);  Final_EulerAngle = Final_RotationMatrix.eulerAngles(2, 1, 0);
    scan_x = Final_Transformation(0,3);  scan_y = Final_Transformation(1,3);  scan_z = Final_Transformation(2,3);
    scan_yaw = Final_EulerAngle(0);      scan_pitch = Final_EulerAngle(1);    scan_roll = Final_EulerAngle(2);
    //Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 scan;
    pcl::toROSMsg(Final,scan);

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

    //publish the PointCloud2 of map
    map.header.stamp = ros::Time::now();
    map.header.frame_id = "base_link";
    Map_pub.publish(map);
    //publish the PointCloud2 of LiDAR
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "base_link";
    Scan_pub.publish(scan);

    //output data to result csv file
    //myfile.precision(14);
  if (k==1){
    myfile << std::fixed << std::setprecision(15) << T << "," << scan_x << "," << scan_y << "," << scan_z << "," << scan_yaw << "," << scan_pitch << "," << scan_roll << "\n";
    k=0;
//debug
j=j+1;
ROS_INFO("output times : %d",j);
  }
    ros::spinOnce(); 
    r.sleep();
  }

  myfile.close();
  return 0;
}
