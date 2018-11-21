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

#define  L_VEHICLE  (2.9)       // the length of the vehicle (from the rear wheels to the front ones, m)

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
  if(strAngle>300){strAngle=300;}
  if(strAngle<-300){strAngle=-300;}
}

int main(int argc, char* argv[])
{
  ros::init(argc,argv,"vehicle_model");
  ros::NodeHandle n;

  ros::Subscriber sub_strAngle = n.subscribe("/decision/steering_angle",1,update_strAngle);
  ros::Publisher pub_location = n.advertise<geometry_msgs::Point>("/sensor/IMU/location",1);
  ros::Publisher pub_speed = n.advertise<std_msgs::Float64>("/sensor/IMU/speed",1);
  ros::Publisher pub_yaw = n.advertise<std_msgs::Float64>("/sensor/IMU/yaw",1);

  int r = 50;
  ros::Rate rate(r);
  double dt = 1.0/r;
  double delta = 0.0;
  geometry_msgs::Point p;
  std_msgs::Float64 speed;
  std_msgs::Float64 yaw;

  // init vehicle state  32110373.62381314	7321357.722018659
  state.x = 32110373.62381314;
  state.y = 7321357.722018659;
  state.yaw = -70.0;
  state.speed = 5.0;

  while(ros::ok()){
    delta = strAngle / 10 * M_PI / 180;
    state.x = state.x + state.speed * cos(state.yaw*M_PI/180.0) * dt;
    state.y = state.y + state.speed * sin(state.yaw*M_PI/180.0) * dt;
    state.yaw = state.yaw + (state.speed / L_VEHICLE * tan(delta) * dt)*180.0/M_PI;

    p.x = state.x;
    p.y = state.y;
    pub_location.publish(p);

    speed.data = state.speed;
    pub_speed.publish(speed);

    yaw.data = state.yaw + 90;
    pub_yaw.publish(yaw);

    printf("DEBUG : delta = %.2f\n",delta);

    ros::spinOnce();
    rate.sleep();
  }

}
