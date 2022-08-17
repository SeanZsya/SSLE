#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

ros::Subscriber sub_cam_pos;
ros::Publisher pub_cam_pos;
geometry_msgs::PoseStamped cam_pose;

void OdometryCallback(const nav_msgs::Odometry& odom)
{
   cam_pose.header = odom.header;
   cam_pose.pose.position.x    = odom.pose.pose.position.x;
   cam_pose.pose.position.y    = odom.pose.pose.position.y;
   cam_pose.pose.position.z    = odom.pose.pose.position.z;
   cam_pose.pose.orientation.w = odom.pose.pose.orientation.w;
   cam_pose.pose.orientation.x = odom.pose.pose.orientation.x;
   cam_pose.pose.orientation.y = odom.pose.pose.orientation.y;
   cam_pose.pose.orientation.z = odom.pose.pose.orientation.z;

   pub_cam_pos.publish(cam_pose);
}

int main(int argc, char** argv)
{  
   //从 mavros/local_position/odom 话题中，接收 nav_msgs::Odometry 类型的消息，
   //将其转化为 geometry_msgs::PoseStamped 类型消息，（不进行坐标系变换）
   //然后发送至 sensor_pose 话题

   ros::init(argc, argv, "pub_cam_pose");
   ros::NodeHandle nh("~");

   sub_cam_pos = nh.subscribe("mavros/local_position/odom", 50, OdometryCallback);  
   pub_cam_pos = nh.advertise<geometry_msgs::PoseStamped>("sensor_pose", 50);

   ros::spin();
}
