#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

using namespace Eigen;
using namespace std;

ros::Subscriber sub_cam_pos;
ros::Publisher pub_cam_pos;
geometry_msgs::PoseStamped camera_pose;
Matrix4d cam2body;
Matrix4d cam2world;
Eigen::Quaterniond cam2world_quat;

void OdometryCallback(const nav_msgs::Odometry& odom)
{
   Eigen::Vector3d request_position = Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
   Eigen::Quaterniond request_pose = Eigen::Quaterniond(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);

   Matrix4d pose_receive = Matrix4d::Identity();
   pose_receive.block<3,3>(0,0) = request_pose.toRotationMatrix();
   pose_receive(0,3) = request_position(0);
   pose_receive(1,3) = request_position(1);
   pose_receive(2,3) = request_position(2);

   Matrix4d body2world = pose_receive;
   cam2world = body2world * cam2body;
   cam2world_quat = cam2world.block<3,3>(0,0); 

   camera_pose.header = odom.header;
   camera_pose.pose.position.x    = cam2world(0,3);
   camera_pose.pose.position.y    = cam2world(1,3);
   camera_pose.pose.position.z    = cam2world(2,3);
   camera_pose.pose.orientation.w = cam2world_quat.w();
   camera_pose.pose.orientation.x = cam2world_quat.x();
   camera_pose.pose.orientation.y = cam2world_quat.y();
   camera_pose.pose.orientation.z = cam2world_quat.z();

   pub_cam_pos.publish(camera_pose);
}

int main(int argc, char** argv)
{  
   //从 mavros/local_position/odom 话题中，接收 nav_msgs::Odometry 类型的消息，
   //进行坐标变换，并将其转化为 geometry_msgs::PoseStamped 类型消息，
   //然后发送至 sensor_pose 话题

   ros::init(argc, argv, "pub_cam_pose");
   ros::NodeHandle nh("~");
   cam2body << 0.0,0.0,1.0,0.0,  -1.0,0.0,0.0,0.0,    0.0,-1.0,0.0,0.0,    0.0,0.0,0.0,1.0; 

   sub_cam_pos = nh.subscribe("mavros/local_position/odom", 50, OdometryCallback);  
   pub_cam_pos = nh.advertise<geometry_msgs::PoseStamped>("sensor_pose", 50);

   ros::spin();
}
