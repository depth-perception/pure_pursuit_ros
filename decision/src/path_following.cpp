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
#include "sensor_msgs/Imu.h"

using namespace std;

#ifndef  DEBUG
#define  DEBUG       1
#endif

#define  K          (0.1)       // coefficient of front view distance
#define  LFC        (2.0)       // front view distance
#define  K_P        (1.0)       // coefficient of speed controler
#define  L_VEHICLE  (2.9)       // the length of the vehicle (from the rear wheels to the front ones, m)

int flag = 0;

typedef struct VehicleState{
  double x = 0;          // m
  double y = 0;          // m
  double yaw = 0;        // degree
  double yaw_rate = 0;   // degree/s
  // double speed_x = 0;    // m/s
  // double speed_y = 0;    // m/s
  double speed = 0;      // m/s
}State;

State state;

typedef struct LocationPoint{
  double x = 0;   // m
  double y = 0;   // m
}Point;

vector<Point> path;

void update_speed(const std_msgs::Float64::ConstPtr& msg)
{
  state.speed = msg->data;
  if(DEBUG){ROS_INFO("Update vehicle speed: [%f]",state.speed);}
}

void update_location(const geometry_msgs::Point::ConstPtr& msg)
{
  state.x = msg->x;
  state.y = msg->y;
  if(DEBUG){ROS_INFO("Update vehicle location: [%f , %f]",state.x,state.y);}
  flag = 1;
}

void update_yaw(const std_msgs::Float64::ConstPtr& msg)
{
  state.yaw = msg->data;
//  state.yaw = 90.0 - state.yaw;
  if(DEBUG){ROS_INFO("Update vehicle yaw: [%f]", state.yaw);}
}

void update_yaw_rate(const std_msgs::Float64::ConstPtr& msg)
{

}

void load_path(const string& path_file, vector<Point>& path)
{
  ifstream in;
  in.open(path_file,ios::in);
  if(!in.is_open()){
    cout << "ERROR: Open path file failed!" << endl;
    return;
  }
  string strOne;
  while(getline(in,strOne)){
    stringstream ss;
    ss << strOne;
    Point p;
    ss >> p.x >> p.y;
    path.push_back(p);
  }
  cout << "load path succeed!" << endl;
}

int get_goal_index(const State& s, const vector<Point>& path)
{
  vector<double> dx,dy,d;
  for(int i=0;i<path.size();i++)
  {
    dx.push_back(s.x - path[i].x);
    dy.push_back(s.y - path[i].y);
    d.push_back(sqrt(dx[i]*dx[i] + dy[i]*dy[i]));
    //cout << s.x << '\t' << s.y <<endl;
  }

  int index = 0;
  double d_min = d[0];
  for(int i=0;i<path.size();i++)
  {
    if(d_min>d[i])
    {
      d_min = d[i];
      index = i;
    }
  }

  printf("DEBUG 1 : index = %d\n",index);
  printf("DEBUG : d_min = %.2f\n",d_min);

  double l = 0;
  double lf = K * s.speed + LFC;
  double dx_ , dy_;

  //printf("DEBUG : lf = %f \n",lf);

  while(l<lf && index<path.size())
  {
    dx_ = path[index+1].x - path[index].x;
    dy_ = path[index+1].y - path[index].y;
    l += sqrt(dx_*dx_ + dy_*dy_);
  //  printf("DEBUG : l = %f \n",l);
    index++;
  }

  return index;
}

double pure_pursuit_control(const State& s, const vector<Point>& path,int *lastIndex)
{
  int index = get_goal_index(s,path);
  if(index <= *lastIndex) {index = *lastIndex;}  // make sure the vehicle move forward instead of backward

  Point goal;
  if(index<path.size()) {
    goal = path[index];
  }
  else {
    index = path.size() - 1;
    goal = path[index];
  }

  double alpha = atan2(goal.y-s.y , goal.x-s.x) - s.yaw * M_PI / 180;
  double lf =  K * s.speed + LFC;
  double delta = atan2((2.0 * L_VEHICLE * sin(alpha)) / lf, 1.0);

  cout <<setprecision(16)<< goal.x-s.x << '\t' <<setprecision(16)<< goal.y-s.y << endl;
//  cout << alpha << '\t' << s.yaw << endl;
  cout << delta << endl;

  *lastIndex = index;      // renew last goal index
  return delta;
}


int main(int argc,char **argv)
{
  ros::init(argc,argv,"path_following");
  ros::NodeHandle n;
  ros::Rate rate(20);

  ros::Subscriber sub_speed = n.subscribe("sensor/IMU/speed", 1000, update_speed);
  ros::Subscriber sub_yaw = n.subscribe("sensor/IMU/yaw", 1000, update_yaw);
  ros::Subscriber sub_location = n.subscribe("/sensor/IMU/location", 1000, update_location);
  ros::Subscriber sub_yaw_rate = n.subscribe("sensor/IMU/speed", 1000, update_yaw_rate);


  std_msgs::Float64 strAngle;
  ros::Publisher pub_strAngle = n.advertise<std_msgs::Float64>("/decision/steering_angle",1);

  // State state;
  // vector<Point> path;
  double delta;

  string path_file = "./src/decision/data/write.txt";

  load_path(path_file, path);
  if(path.size()==0){
    printf("ERROR: load path failed!");
    exit(1);
  }

 while(ros::ok())
 {
    ros::spinOnce();
    rate.sleep();
    if(flag) break;
 }

  int rearIndex = path.size() - 1;
  int goalIndex = get_goal_index(state,path);

  printf("DEBUG: goalIndex = %d; rearIdex = %d\n",goalIndex,rearIndex);

  while(ros::ok() && goalIndex<rearIndex)
  {
    delta = pure_pursuit_control(state, path, &goalIndex);
    cout << delta << '\t' << delta * 180/M_PI<<endl;
    strAngle.data = 10 * delta * 180 / M_PI;

    // if(strAngle.data > 300.0) strAngle.data = 300.0;
    // if(strAngle.data < -300.0) strAngle.data = -300.0;

    //strAngle.data = -strAngle.data;

    cout<<"DEBUG: strAngle = "<<strAngle.data<<endl;
    cout<<"DEBUG: goalIndex = "<<goalIndex<<endl;

    pub_strAngle.publish(strAngle);
    ros::spinOnce();
    rate.sleep();
  }

  if(goalIndex == rearIndex)
  {
    ROS_INFO("Vehicle has reached the target point! Node stop!");
  }

  return 0;
}
