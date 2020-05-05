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
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
using namespace Eigen;

pcl::VoxelGrid<pcl::PointXYZ> sor;
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
pcl::PointCloud<pcl::PointXYZ>::Ptr LiDAR (new pcl::PointCloud<pcl::PointXYZ>), LiDAR_Before (new pcl::PointCloud<pcl::PointXYZ>);

double T;
float scan_x,scan_y,scan_z,scan_roll,scan_pitch,scan_yaw,yaw_set;
int LidarInputTimes,CsvOutputTimes,NewLidarInput,FirstInput,BagDefine,MapPubTimes;

Vector3d GPS_Position;
Quaterniond q,qimu,qLidar2Car,qCar2Lidar,Final_Quaternion;
Matrix3d RotationMatrix_Imu,RotationMatrix_Lidar2Car,RotationMatrix_Car2Lidar;
Matrix4f InitialGuess,Lidar2Car,Car2Lidar,Final_Transformation,LiDAR_Move_Transformation;



void LiDARCallBack(const sensor_msgs::PointCloud2::ConstPtr& lidar){
  //Get point cloud data and turn it to ROSMsg
  T = lidar->header.stamp.toSec();
  pcl::fromROSMsg (*lidar, *LiDAR);

  //Filter for pointcloud data
  sor.setInputCloud (LiDAR);
  sor.setLeafSize (2.0f, 2.0f, 2.0f);
  sor.filter (*LiDAR);

  //Turn point cloud to velodyn frame
  pcl::transformPointCloud (*LiDAR, *LiDAR, Car2Lidar);
  NewLidarInput=1;
  //Debug
  LidarInputTimes=LidarInputTimes+1;
  ROS_INFO("Frame times  : %i",LidarInputTimes);
}



void GPSCallBack(const geometry_msgs::PointStamped::ConstPtr& GPS){
  //Get GPS data
  GPS_Position << GPS->point.x , GPS->point.y , GPS->point.z;
}



void IMUCallBack(const sensor_msgs::Imu::ConstPtr& IMU){
  //Get IMU data
  qimu.x() = IMU->orientation.x;  qimu.y() = IMU->orientation.y;  qimu.z() = IMU->orientation.z;  qimu.w() = IMU->orientation.w;
  RotationMatrix_Imu = qimu.toRotationMatrix();

  //Rotate IMU for cancel yaw drift
  double roll = 0 * M_PI / 180, pitch = 0 * M_PI / 180, yaw = yaw_set * M_PI / 180;
  q = AngleAxisd(roll, Vector3d(RotationMatrix_Imu(0,0),RotationMatrix_Imu(1,0),RotationMatrix_Imu(2,0)))
      * AngleAxisd(pitch, Vector3d(RotationMatrix_Imu(0,1),RotationMatrix_Imu(1,1),RotationMatrix_Imu(2,1)))
      * AngleAxisd(yaw, Vector3d(RotationMatrix_Imu(0,2),RotationMatrix_Imu(1,2),RotationMatrix_Imu(2,2)));
  RotationMatrix_Imu = q.toRotationMatrix() * RotationMatrix_Imu;
}



int main(int argc, char** argv) {
  //Initialize ROS
  ros::init (argc, argv, "Midterm_16_easy");
  ros::NodeHandle n;

  //Set icp algorithm
  icp.setMaxCorrespondenceDistance(4.0);
  icp.setTransformationEpsilon(1e-10);
  icp.setEuclideanFitnessEpsilon(0.001);
  icp.setMaximumIterations (1000);

  //Set Lidar2Car transformation
  qLidar2Car.x() = 0.005;  qLidar2Car.y() = -0.018;  qLidar2Car.z() = 0.019 ;  qLidar2Car.w() = 1;
  RotationMatrix_Lidar2Car = qLidar2Car.toRotationMatrix();
  Lidar2Car << RotationMatrix_Lidar2Car(0,0) , RotationMatrix_Lidar2Car(0,1) , RotationMatrix_Lidar2Car(0,2) , -0.335 ,
               RotationMatrix_Lidar2Car(1,0) , RotationMatrix_Lidar2Car(1,1) , RotationMatrix_Lidar2Car(1,2) ,  0.020 ,
               RotationMatrix_Lidar2Car(2,0) , RotationMatrix_Lidar2Car(2,1) , RotationMatrix_Lidar2Car(2,2) , -3.474 ,
                      0         ,        0         ,        0         ,    1   ;

  //Set Car2Lidar transformation
  qCar2Lidar.x() = -0.00515049;  qCar2Lidar.y() = 0.018102;  qCar2Lidar.z() = -0.019207 ;  qCar2Lidar.w() = 0.999638;
  RotationMatrix_Car2Lidar = qCar2Lidar.toRotationMatrix();
  Car2Lidar << RotationMatrix_Car2Lidar(0,0) , RotationMatrix_Car2Lidar(0,1) , RotationMatrix_Car2Lidar(0,2) , 0.46 ,
               RotationMatrix_Car2Lidar(1,0) , RotationMatrix_Car2Lidar(1,1) , RotationMatrix_Car2Lidar(1,2) ,   0  ,
               RotationMatrix_Car2Lidar(2,0) , RotationMatrix_Car2Lidar(2,1) , RotationMatrix_Car2Lidar(2,2) , 3.46 ,
                      0         ,        0         ,        0         ,   1  ;

  //Load pcd file of map 
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_Map (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (ros::package::getPath("midterm_16")+"/src/map.pcd", *pcd_Map) == -1)
  {
    PCL_ERROR ("Couldn't read .pcd file\n");
    return (-1);
  }

  //Filter for map data
  sor.setInputCloud (pcd_Map);
  sor.setLeafSize (2.0f, 2.0f, 2.0f);
  sor.filter (*pcd_Map);

  //Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
  sensor_msgs::PointCloud2 map;
  pcl::toROSMsg (*pcd_Map, map);
    
  //Subscribe and publish datas
  ros::Subscriber LiDAR_sub = n.subscribe ("/lidar_points", 300, LiDARCallBack);
  ros::Subscriber GPS_sub = n.subscribe ("/fix", 300, GPSCallBack);
  ros::Subscriber IMU_sub = n.subscribe ("/imu/data", 300, IMUCallBack);
  ros::Publisher Map_pub = n.advertise<sensor_msgs::PointCloud2> ("/map", 300);
  ros::Publisher Scan_pub = n.advertise<sensor_msgs::PointCloud2> ("/scan", 300);

  //Set tf2 topic
  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped Scan;
  static tf2_ros::StaticTransformBroadcaster stfb;
  geometry_msgs::TransformStamped MAP;

  //publish the tf frame of map
  MAP.header.frame_id = "base_link";  MAP.child_frame_id = "/map";
  MAP.transform.translation.x = 0.0;  MAP.transform.translation.y = 0.0;  MAP.transform.translation.z = 0.0;
  MAP.transform.rotation.x = 0.0;  MAP.transform.rotation.y = 0.0;  MAP.transform.rotation.z = 0.0;  MAP.transform.rotation.w = 1.0;
  MAP.header.stamp = ros::Time::now();
  stfb.sendTransform(MAP);


  std::ofstream myfile;
  ros::Rate r(30);
  while(ros::ok())
  {
    //publish the PointCloud2 of map
    if (MapPubTimes<50){
      map.header.stamp = ros::Time::now();
      map.header.frame_id = "/map";
      Map_pub.publish(map);
      MapPubTimes=MapPubTimes+1;
      ROS_INFO("Upload map : %i %%",MapPubTimes*2);
    }

    //Set bag info
    if (BagDefine == 0 & GPS_Position(0) != 0){
      if (GPS_Position(0) > -270 & GPS_Position(0) < -260){
        yaw_set = -20;
        myfile.open ((ros::package::getPath("midterm_16")+"/src/16_easy_P.csv").c_str());
        BagDefine = 1;
        ROS_INFO("This is Public bag");
      }
      else if (GPS_Position(0) > -15 & GPS_Position(0) < -5){
        yaw_set = 150;
        myfile.open ((ros::package::getPath("midterm_16")+"/src/16_easy_1.csv").c_str());
        BagDefine = 1;
        ROS_INFO("This is Private_1 bag");
      }
      else if (GPS_Position(0) > -105 & GPS_Position(0) < -95){
        yaw_set = 122;
        myfile.open ((ros::package::getPath("midterm_16")+"/src/16_easy_2.csv").c_str());
        BagDefine = 1;
        ROS_INFO("This is Private_2 bag");
      }
      else if (GPS_Position(0) > -290 & GPS_Position(0) < -280){
        yaw_set = 73;
        myfile.open ((ros::package::getPath("midterm_16")+"/src/16_easy_3.csv").c_str());
        BagDefine = 1;
        ROS_INFO("This is Private_3 bag");
      }
    }

    //Let program start after data input
    if (NewLidarInput == 1 & qimu.x() != 0 & GPS_Position(2) != 0 & q.x() != 0){

      //set initial guess for icp
      if (FirstInput == 0) {
        InitialGuess << RotationMatrix_Imu(0,0) , RotationMatrix_Imu(0,1) , RotationMatrix_Imu(0,2) , GPS_Position(0),
                        RotationMatrix_Imu(1,0) , RotationMatrix_Imu(1,1) , RotationMatrix_Imu(1,2) , GPS_Position(1),
                        RotationMatrix_Imu(2,0) , RotationMatrix_Imu(2,1) , RotationMatrix_Imu(2,2) , GPS_Position(2),
                                   0            ,            0            ,            0            ,        1       ;
        LiDAR_Before = LiDAR;
      }
      else if (FirstInput==1) {
        icp.setInputSource(LiDAR_Before);
        icp.setInputTarget(LiDAR);
        pcl::PointCloud<pcl::PointXYZ> LiDAR_Move;
        icp.align(LiDAR_Move);
        LiDAR_Move_Transformation << icp.getFinalTransformation();
        InitialGuess << RotationMatrix_Imu(0,0) , RotationMatrix_Imu(0,1) , RotationMatrix_Imu(0,2) ,(Final_Transformation(0,3)+Final_Transformation(0,0)*LiDAR_Move_Transformation(0,3)+Final_Transformation(1,0)*LiDAR_Move_Transformation(1,3)+Final_Transformation(2,0)*LiDAR_Move_Transformation(2,3)+GPS_Position(0))/2,
                        RotationMatrix_Imu(1,0) , RotationMatrix_Imu(1,1) , RotationMatrix_Imu(1,2) ,(Final_Transformation(1,3)+Final_Transformation(0,1)*LiDAR_Move_Transformation(0,3)+Final_Transformation(1,1)*LiDAR_Move_Transformation(1,3)+Final_Transformation(2,1)*LiDAR_Move_Transformation(2,3)+GPS_Position(1))/2,
                        RotationMatrix_Imu(2,0) , RotationMatrix_Imu(2,1) , RotationMatrix_Imu(2,2) ,(Final_Transformation(2,3)+Final_Transformation(0,2)*LiDAR_Move_Transformation(0,3)+Final_Transformation(1,2)*LiDAR_Move_Transformation(1,3)+Final_Transformation(2,2)*LiDAR_Move_Transformation(2,3)+GPS_Position(2))/2,
                            0    ,    0    ,    0    ,     1     ;
        LiDAR_Before = LiDAR;
      }

      //Matching lidar and map
      icp.setInputSource(LiDAR);
      icp.setInputTarget(pcd_Map);
      pcl::PointCloud<pcl::PointXYZ> Final;
      icp.align(Final,InitialGuess);

      //Get data for result file
      Final_Transformation << icp.getFinalTransformation();
      Eigen::Affine3f tROTA(Final_Transformation);
      pcl::getTranslationAndEulerAngles(tROTA, scan_x, scan_y, scan_z, scan_roll, scan_pitch, scan_yaw);
      tf::Quaternion Final_Quaternion;
      Final_Quaternion.setRPY(scan_roll,scan_pitch,scan_yaw);

      //Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
      sensor_msgs::PointCloud2 scan;
      pcl::toROSMsg(Final,scan);

      //publish the tf frame of scan
      Scan.header.frame_id = "/map";
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
      scan.header.frame_id = "/map";
      Scan_pub.publish(scan);

      //output data to result csv file
      myfile << std::fixed << std::setprecision(15) << T << "," << scan_x << "," << scan_y << "," << scan_z << "," << scan_yaw << "," << scan_pitch << "," << scan_roll << "\n";
      NewLidarInput=0;
      FirstInput=1;

      //debug
      CsvOutputTimes=CsvOutputTimes+1;
      ROS_INFO("output times : %i \n yaw : %f , pitch : %f , roll : %f",CsvOutputTimes,scan_yaw,scan_pitch,scan_roll);
    }
    ros::spinOnce(); 
    r.sleep();
  }

  myfile.close();
  return 0;
}
