#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

ros::Subscriber sub_sensor_pos;
ros::Publisher pub_sensor_pos;
geometry_msgs::PoseStamped sensor_pose;

void OdometryCallback(const nav_msgs::Odometry& odom)
{
   sensor_pose.header = odom.header;
   sensor_pose.pose.position.x    = odom.pose.pose.position.x;
   sensor_pose.pose.position.y    = odom.pose.pose.position.y;
   sensor_pose.pose.position.z    = odom.pose.pose.position.z;
   sensor_pose.pose.orientation.w = odom.pose.pose.orientation.w;
   sensor_pose.pose.orientation.x = odom.pose.pose.orientation.x;
   sensor_pose.pose.orientation.y = odom.pose.pose.orientation.y;
   sensor_pose.pose.orientation.z = odom.pose.pose.orientation.z;

   pub_sensor_pos.publish(sensor_pose);
}

int main(int argc, char** argv)
{  
   //从 mavros/local_position/odom 话题中，接收 nav_msgs::Odometry 类型的消息，
   //将其转化为 geometry_msgs::PoseStamped 类型消息，（不进行坐标系变换）
   //然后发送至 sensor_pose 话题

   ros::init(argc, argv, "pub_sensor_pose");
   ros::NodeHandle nh("~");
   string uav_name;
   nh.param<string>("uav_name", uav_name, "/uav0");
   if (uav_name == "/uav0")
      uav_name = "";

   sub_sensor_pos = nh.subscribe(uav_name + "/mavros/local_position/odom", 50, OdometryCallback);  
   pub_sensor_pos = nh.advertise<geometry_msgs::PoseStamped>("sensor_pose", 50);
   
   while(ros::ok())
   {
      nh.getParam("uav_name", uav_name);
      ros::spinOnce();
   }
   
}
