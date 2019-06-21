#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tf.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "gps_utm.cpp"
#include <visualization_msgs/Marker.h>


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <iostream>
#include <fstream>
#include <string>

#include <ros/package.h>




double pos[3], quat[4], yaw;

int count_gps, count_imu;
double cx_gps, cy_gps, center_yaw;


void gps_callback(const sensor_msgs::NavSatFixConstPtr& gps)
{

     // Transform to UTM reference system
     double northing, easting;
     char zone;
     LLtoUTM(gps->latitude, gps->longitude,  northing, easting , &zone);



     if (count_gps < freq){
       count_gps++;
       cx_gps += easting/freq;
       cy_gps += northing/frrq;
     }
     else{
       pos[0] = easting-cx_gps;
       pos[1] = northing-cy_gps;
       pos[2] = 0.0;
     }



     //cout << easting-cx_gps << "\t\t"<< northing-cy_gps << endl;

}


void imu_callback(const sensor_msgs::ImuConstPtr& imu)
{

     quat[0] = imu->orientation.x;
     quat[1] = imu->orientation.y;
     quat[2] = imu->orientation.z;
     quat[3] = imu->orientation.w;

     double r, p, y;
     tf::Matrix3x3 rotMatrix(tf::Quaternion(quat[0],quat[1],quat[2],quat[3]));

     // Get roll, pitch and yaw
     rotMatrix.getRPY(r, p, y);
     yaw = y;


     if (count_imu < freq){
       count_imu++;
       center_yaw += yaw/freq;
     }
     else{
       yaw = y - center_yaw;
     }

     //cout << "yaw = " << yaw*180/3.141592 << endl;
     //cout << "roll, pitch, yaw = " << "\t" << r*180/3.141592 << "\t" << p*180/3.141592 << "\t" << yaw*180/3.141592 << endl;
     //printf("roll, pitch, yaw = \t %.4f\t%.4f\t%.4f\n",r*180/3.141592,p*180/3.141592,y*180/3.141592);
}



// Main
int main(int argc, char **argv) {

  ros::init(argc, argv, "pose_constructor");
  ros::NodeHandle nh;

  double freq = 20.0;


  ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1, gps_callback);
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, imu_callback);
  //ros::Subscriber ground_truth_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1, getStates);
  //ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	//ros::Publisher thrust_rate_pub = nh.advertise<mav_msgs::RateThrust>("/uav/input/rateThrust", 1);
  //ros::Publisher curve_now_pub = nh.advertise<std_msgs::Int32>("/path/curve_now", 1);
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("/espeleo/pose_gps_imu", 1);
  ros::Publisher rviz_pose_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker_espeleo_ufmg", 1);
  ros::Rate loop_rate(freq);


  int i = 0;
  count_gps = 0;
  count_imu = 0;
  cx_gps = 0; cy_gps = 0; center_yaw = 0;

  geometry_msgs::Pose espeleo_pose;

  visualization_msgs::Marker espeleo_marker;


  int state = 0; // 0 - take off; 1 - follow vector field

  //pos[0] = 1.0; pos[1] = 2.0; pos[2] = 3.0;
  //quat[0] = 0.0; quat[1] = 0.0; quat[2] = 0.0; quat[3] = 0.0;


  FILE *f;
  f = fopen("/home/adrianomcr/ROS_projects/vale_ws/src/ufmg_experiments/text/results.txt", "w");



  while (ros::ok())
  {

    // Read the callbacks
    ros::spinOnce();


    //cout << i << endl;
    i++;


    fprintf(f,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",pos[0],pos[1],pos[2],quat[0],quat[1],quat[2],quat[3],yaw);


    // Uptade the rost variable to be publishe
    espeleo_pose.position.x = pos[0];
    espeleo_pose.position.y = pos[1];
    espeleo_pose.position.z = pos[2];
    espeleo_pose.orientation.x = quat[0];
    espeleo_pose.orientation.y = quat[1];
    espeleo_pose.orientation.z = quat[2];
    espeleo_pose.orientation.w = quat[3];

    // Publish rateThrust command
    pose_pub.publish(espeleo_pose);








    espeleo_marker.header.frame_id = "/world";
    espeleo_marker.header.stamp = ros::Time::now();
    espeleo_marker.id = 0;
    espeleo_marker.type = espeleo_marker.CUBE;
    espeleo_marker.action = espeleo_marker.ADD;
    espeleo_marker.scale.x = 0.50;
    espeleo_marker.scale.y = 0.30;
    espeleo_marker.scale.z = 0.12;
    espeleo_marker.color.a = 0.9;
    espeleo_marker.color.r = 0.9;
    espeleo_marker.color.g = 0.9;
    espeleo_marker.color.b = 0.0;
    espeleo_marker.pose.position.x = pos[0];
    espeleo_marker.pose.position.y = pos[1];
    espeleo_marker.pose.position.z = pos[2];
    //quaternio = [esp_q[0], esp_q[1], esp_q[2], esp_q[3]]
    espeleo_marker.pose.orientation.x = quat[0];
    espeleo_marker.pose.orientation.y = quat[1];
    espeleo_marker.pose.orientation.z = quat[2];
    espeleo_marker.pose.orientation.w = quat[3];

    rviz_pose_pub.publish(espeleo_marker);








    // Sleep program
    loop_rate.sleep();
  }



  //Terminate if ros is not ok
  if(!ros::ok()){
    // Close results file
    fclose(f);
  }



}
