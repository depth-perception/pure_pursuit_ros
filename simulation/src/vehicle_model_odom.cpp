#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <cassert>
#include <math.h>
#include <vector>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#define  L_VEHICLE  (2.64)       // the length of the vehicle (from the rear wheels to the front ones, m)

typedef struct VehicleState{
  double x = 0.0;          // m
  double y = 0.0;          // m
  double yaw = 0.0;        // degree
  double yaw_rate = 0.0;   // degree/s
  // double speed_x = 0;    // m/s
  // double speed_y = 0;    // m/s
  double speed = 0.0;      // m/s
}State;

State state;
double strAngle = 0.0; // degree

void update_strAngle(const std_msgs::Float64::ConstPtr& msg)
{
  strAngle = msg->data;
  if(strAngle>400){strAngle=400;}
  if(strAngle<-400){strAngle=-400;}
}

void update_speed(const std_msgs::Float64::ConstPtr& msg)
{
  state.speed = msg->data;
}

int main(int argc, char* argv[])
{
  ros::init(argc,argv,"vehicle_model");
  ros::NodeHandle n;

  ros::Subscriber sub_strAngle = n.subscribe("/decision/steering_angle",1,update_strAngle);
  ros::Subscriber sub_speed = n.subscribe("/decision/speed",1,update_speed);

  ros::Publisher pub_location = n.advertise<geometry_msgs::Point>("/sensor/IMU/location",1);
  ros::Publisher pub_speed = n.advertise<std_msgs::Float64>("/sensor/IMU/speed",1);
  ros::Publisher pub_yaw = n.advertise<std_msgs::Float64>("/sensor/IMU/yaw",1);

  ros::Publisher odom_pub = n.advertise< nav_msgs::Odometry >("odom", 10);
  tf::TransformBroadcaster odom_broadcaster;

  int r = 50;
  ros::Rate rate(r);
  double dt ;//= 1.0/r;
  double delta = 0.0;
  geometry_msgs::Point p;
  std_msgs::Float64 speed;
  std_msgs::Float64 yaw;

  double x0 = 5.0;
  double y0 = 5.0;
  // init vehicle state  32110373.62381314	7321357.722018659
  state.x = x0; //414.0;
  state.y = y0; //436.0;
  state.yaw = 10.0;
  state.speed = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time    = ros::Time::now();

  while(ros::ok()){
    current_time = ros::Time::now();

    dt       = (current_time - last_time).toSec();
    delta = strAngle / 10 * M_PI / 180;
    state.x = state.x + state.speed * cos(state.yaw*M_PI/180.0) * dt;
    state.y = state.y + state.speed * sin(state.yaw*M_PI/180.0) * dt;
    state.yaw_rate = state.speed * tan(delta) / L_VEHICLE;
    state.yaw = state.yaw + (state.yaw_rate * dt)*180.0/M_PI;

    p.x = state.x;
    p.y = state.y;
    pub_location.publish(p);

    speed.data = state.speed;
    pub_speed.publish(speed);

    yaw.data = state.yaw ;
    pub_yaw.publish(yaw);

    printf("DEBUG : delta = %.2f\n",delta);

    // setuo tf frame
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = state.x ;
    odom_trans.transform.translation.y = state.y ;
    odom_trans.transform.translation.z = 0.0;

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(state.yaw*M_PI/180);
    odom_trans.transform.rotation      = quat;

    // broadcast tf frame
    odom_broadcaster.sendTransform(odom_trans);

    //set up odom frame
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "base_link";

    odom.pose.pose.position.x = state.x ;
    odom.pose.pose.position.y = state.y ;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = quat;

    odom.child_frame_id = "odom";
    odom.twist.twist.linear.x = state.speed*cos(state.yaw*M_PI/180);
    odom.twist.twist.linear.y = state.speed*sin(state.yaw*M_PI/180);
    odom.twist.twist.angular.z = state.yaw_rate;

    // publish odom frame
    odom_pub.publish(odom);

    ros::spinOnce();
    last_time = current_time;
    rate.sleep();
  }

}
