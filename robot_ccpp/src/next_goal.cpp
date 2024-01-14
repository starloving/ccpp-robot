
/*This code is used to plan the trajectory of the robot  
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"

using namespace std;
using namespace tf2;

float x_current;
float y_current;

float normeNextGoal;

class quaternion_ros
{
public:
  float w;
  float x;
  float y;
  float z;

  quaternion_ros();

  void toQuaternion(float pitch, float roll, float yaw);
};

void quaternion_ros::toQuaternion(float pitch, float roll, float yaw)
{

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  w = cy * cr * cp + sy * sr * sp;
  x = cy * sr * cp - sy * cr * sp;
  y = cy * cr * sp + sy * sr * cp;
  z = sy * cr * cp - cy * sr * sp;
}

quaternion_ros::quaternion_ros()
{
  w = 1;
  x = 0;
  y = 0;
  z = 0;
}

class Path_planned
{
public:
  struct Goal
  {
    float x;
    float y;
    bool visited;
  };

  vector<Goal> Path;

  Path_planned();
  //Path_planned(float x, float y, bool visited);
  void addGoal(float X, float Y, bool visit);
};

Path_planned::Path_planned()
{
}

//Path_planned(float x, float y, bool visited)

void Path_planned::addGoal(float X, float Y, bool visit)
{
  Path_planned::Goal newGoal;
  newGoal.x = X;
  newGoal.y = Y;
  newGoal.visited = visit;
  Path.push_back(newGoal);
}

Path_planned planned_path;
nav_msgs::msg::Path passed_path;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_passed_path ;

void pose_callback(const nav_msgs::msg::Odometry::SharedPtr poses)
{ //마일리지 콜백 함수, 현재 로봇 위치와 앞의 목표점과의 거리를 계산하여, 새로운 커튼 포인트 발행 여부를 판단하는 데 사용된다.
  x_current = poses->pose.pose.position.x;
  y_current = poses->pose.pose.position.y;
  passed_path.header = poses->header;
  geometry_msgs::msg::PoseStamped p;
  p.header = poses->header;
  p.pose = poses->pose.pose;
  passed_path.poses.emplace_back(p);
  // pub_passed_path->publish(passed_path);
}

int taille_last_path = 0;
bool new_path = false;

//계획을 수락하는 경로로 받아들임
void path_callback(const nav_msgs::msg::Path::SharedPtr path)
{
  //주의 rviz를 위해 편리한 경로를 계속 보내고 있습니다. 그러나 여기서는 한 번만 수락하면 됩니다. 계획된 경로가 변경될 때 다시 로드
  if ((planned_path.Path.size() == 0) || (path->poses.size() != taille_last_path))
  {
    planned_path.Path.clear();
    new_path = true;
    for (int i = 0; i < path->poses.size(); i++)
    {
      planned_path.addGoal(path->poses[i].pose.position.x, path->poses[i].pose.position.y, false);

      cout << path->poses[i].pose.position.x << " " << path->poses[i].pose.position.y << endl;
    }
    cout << "Recv path size:" << path->poses.size() << endl;
    taille_last_path = path->poses.size();
  }
}

// int **count_antonin(char *)

int main(int argc, char *argv[]) {
  // srand(time(0));
  rclcpp::init(argc, argv) ;
  // ros::NodeHandle next_goal;
  auto next_goal = std::make_shared<rclcpp::Node>("next_goal_node");

  auto pub1 = next_goal->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", rclcpp::QoS(rclcpp::KeepLast(10))) ;

  auto pub_passed_path = next_goal->create_publisher<nav_msgs::msg::Path>(
    "/clean_robot/passed_path", rclcpp::QoS(rclcpp::KeepLast(10))) ;

  auto sub1 = next_goal->create_subscription<nav_msgs::msg::Odometry>(
          "/odom",
          // rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
          10,
          pose_callback
      );

  auto sub2 = next_goal->create_subscription<nav_msgs::msg::Path>(
          "/cleaning_path",
          // rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
          10,
          path_callback
      );

  // ros::Subscriber sub1 = next_goal.subscribe("/odom", 1000, pose_callback);
  // ros::Subscriber sub2 = next_goal.subscribe("/path_planning_node/cleaning_plan_nodehandle/cleaning_path", 1000, path_callback);

  // ros::Publisher pub1 = next_goal.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
  // pub_passed_path = next_goal.advertise<nav_msgs::Path>("/clean_robot/passed_path", 1000);

  // ros::Rate loop_rate(10);
  rclcpp::Rate loop_rate(10) ;  // 1 Hz

  geometry_msgs::msg::PoseStamped goal_msgs;
  int count = 0;
  double angle;
  bool goal_reached = false;
  // 다음 점을 보낼 임계값을 가져옵니다.
  // if (!next_goal.getParam("/NextGoal/tolerance_goal", normeNextGoal))
  // {
  //   ROS_ERROR("Please set your tolerance_goal");
  //   return 0;
  // }
  // ROS_INFO("tolerance_goal=%f", normeNextGoal);
  normeNextGoal = 1 ;
  next_goal->declare_parameter<float>("tolerance_goal" , normeNextGoal);
  next_goal->get_parameter("tolerance_goal" , normeNextGoal) ;

  while(rclcpp::ok()) {
    rclcpp::spin_some(next_goal) ;
    if (new_path) {
      count = 0;
      new_path = false;
    }
    // 현재 처리 지점
    cout << " count : " << count << endl;
    if (!planned_path.Path.empty())
    {
      // 현재 거리가 도달했다.
      if (sqrt(pow(x_current - planned_path.Path[count].x, 2) + pow(y_current - planned_path.Path[count].y, 2)) <= normeNextGoal)
      {
        count++;
        goal_reached = false;
      }
      if (goal_reached == false)
      {
        goal_msgs.header.frame_id = "odom";
        goal_msgs.header.stamp = next_goal->get_clock().get()->now();
        goal_msgs.pose.position.x = planned_path.Path[count].x;
        goal_msgs.pose.position.y = planned_path.Path[count].y;
        goal_msgs.pose.position.z = 0;
        if (count < planned_path.Path.size())
        {// 계산하여 배포한 yaw, 단, bug도 있지만 사용에 지장을 주지 않으며 yaw는 큰 영향을 미치지 않는다.
          angle = atan2(planned_path.Path[count + 1].y - planned_path.Path[count].y, planned_path.Path[count + 1].x - planned_path.Path[count].x);
        }
        else
        {
          angle = atan2(planned_path.Path[0].y - planned_path.Path[count].y, planned_path.Path[0].x - planned_path.Path[count].x);
        }
        cout << angle << endl;
        quaternion_ros q;
        q.toQuaternion(0, 0, float(angle));
        goal_msgs.pose.orientation.w = q.w;
        goal_msgs.pose.orientation.x = q.x;
        goal_msgs.pose.orientation.y = q.y;
        if (planned_path.Path[count].x < planned_path.Path[count + 1].x)
        {
          goal_msgs.pose.orientation.z = 0;
        }
        if (planned_path.Path[count].x > planned_path.Path[count + 1].x)
        {
          goal_msgs.pose.orientation.z = 2;
        }

        cout << " NEW GOAL " << endl;
        cout << " x = " << planned_path.Path[count].x << " y = " << planned_path.Path[count].y << endl;

        goal_reached = true;
        pub1->publish(goal_msgs);
      }
      cout << x_current << " " << y_current << endl;
      //현재
      cout << planned_path.Path[count].x << " " << planned_path.Path[count].y << endl;
      //목표
      cout << " DISTANCE : " << sqrt((x_current - planned_path.Path[count].x) * (x_current - planned_path.Path[count].x) + (y_current - planned_path.Path[count].y) * (y_current - planned_path.Path[count].y)) << endl;
      // 거리 공식
    }
    loop_rate.sleep();
  }
  rclcpp::shutdown() ;   
  return 0;
}
