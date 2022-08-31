#include <ros/ros.h>
#include <iostream>
#include <prometheus_msgs/ControlCommand.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include "state_from_mavros.h"
#include "message_utils.h"
#include "command_to_mavros.h"
#include "prometheus_control_utils.h" 
// #include "controller_test.h"
#include "KeyboardEvent.h"
#include "quadrotor_msgs/PositionCommand.h"


#define NODE_NAME "fuel_control"
#define VEL_XY_STEP_SIZE 0.1
#define VEL_Z_STEP_SIZE 0.1
#define YAW_STEP_SIZE 0.08
#define TRA_WINDOW 2000

using namespace std;


prometheus_msgs::ControlCommand Command_to_pub;//即将发布的command

ros::Publisher move_pub;
ros::Subscriber fuel_cmd_sub;

//callback function
void cmd_transfer_cb(const quadrotor_msgs::PositionCommand cmd)
{
   
    Command_to_pub.header.stamp = ros::Time::now();
    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1; 
    Command_to_pub.source = NODE_NAME;
    Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_ALL;
    Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_to_pub.Reference_State.position_ref[0] = cmd.position.x;
    Command_to_pub.Reference_State.position_ref[1] = cmd.position.y;
    Command_to_pub.Reference_State.position_ref[2] = cmd.position.z;
    Command_to_pub.Reference_State.velocity_ref[0] = cmd.velocity.x;
    Command_to_pub.Reference_State.velocity_ref[1] = cmd.velocity.y;
    Command_to_pub.Reference_State.velocity_ref[2] = cmd.velocity.z;
    Command_to_pub.Reference_State.acceleration_ref[0] = cmd.acceleration.x;
    Command_to_pub.Reference_State.acceleration_ref[1] = cmd.acceleration.y;
    Command_to_pub.Reference_State.acceleration_ref[2] = cmd.acceleration.z;
    Command_to_pub.Reference_State.yaw_ref  = cmd.yaw;

    move_pub.publish(Command_to_pub);
    
}

//mian function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_px4_cmd");
    ros::NodeHandle nh("~");
    KeyboardEvent keyboardcontrol;
    char key_now, key_wait;
    char key_last;
    string uav_name;
    nh.param<string>("uav_name", uav_name, "/uav0");
    if (uav_name == "/uav0")
        uav_name = "";

    //　【发布】　控制指令
    move_pub = nh.advertise<prometheus_msgs::ControlCommand>(uav_name + "/prometheus/control_command", 10);

    fuel_cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(uav_name + "/planning/pos_cmd", 10, cmd_transfer_cb);


    // 初始化命令 - Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_to_pub.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_to_pub.Command_ID                          = 0;
    Command_to_pub.source = NODE_NAME;
    Command_to_pub.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_to_pub.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_to_pub.Reference_State.position_ref[0]     = 0;
    Command_to_pub.Reference_State.position_ref[1]     = 0;
    Command_to_pub.Reference_State.position_ref[2]     = 0;
    Command_to_pub.Reference_State.velocity_ref[0]     = 0;
    Command_to_pub.Reference_State.velocity_ref[1]     = 0;
    Command_to_pub.Reference_State.velocity_ref[2]     = 0;
    Command_to_pub.Reference_State.acceleration_ref[0] = 0;
    Command_to_pub.Reference_State.acceleration_ref[1] = 0;
    Command_to_pub.Reference_State.acceleration_ref[2] = 0;
    Command_to_pub.Reference_State.yaw_ref             = 0;
    

    cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<"<< endl;
    cout << "ENTER key to control the drone: " <<endl;
    cout << "1 for Arm, Space for Takeoff, L for Land, H for Hold, 0 for Disarm, r for runing FUEL onboard" <<endl;
    cout << "Move mode is fixed (XYZ_VEL,BODY_FRAME): w/s for body_x, a/d for body_y, k/m for z, q/e for body_yaw" <<endl;
    cout << "CTRL-C to quit." <<endl;

    while (ros::ok())
    {
      nh.getParam("uav_name", uav_name);
      keyboardcontrol.RosWhileLoopRun();
      key_now = keyboardcontrol.GetPressedKey();

      switch (key_now)
      {

        //悬停, 应当只发送一次, 不需要循环发送
        case U_KEY_NONE:

          sleep(0.5);

          break;

        // 数字1（非小键盘数字）：解锁及切换到OFFBOARD模式
        case U_KEY_1:
          cout << " " <<endl;
          cout << "Arm and Switch to OFFBOARD." <<endl;
      
          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.source = NODE_NAME;
          Command_to_pub.Reference_State.yaw_ref = 999;
          move_pub.publish(Command_to_pub);
          sleep(1.0);
          break;

        // 空格：起飞
        case U_KEY_SPACE:
          cout << " " <<endl;
          cout << "Switch to Takeoff Mode." <<endl;

          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.Reference_State.yaw_ref = 0.0;
          Command_to_pub.source = NODE_NAME;
          move_pub.publish(Command_to_pub);

          sleep(1.0);

          break;

        // 键盘L：降落
        case U_KEY_L:
          cout << " " <<endl;
          cout << "Switch to Land Mode." <<endl;
      
          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Land;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.source = NODE_NAME;
          move_pub.publish(Command_to_pub);

          break;

        // 键盘0（非小键盘数字）：紧急停止
        case U_KEY_0:
          cout << " " <<endl;
          cout << "Switch to Disarm Mode." <<endl;
      
          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Disarm;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.source = NODE_NAME;
          move_pub.publish(Command_to_pub);

          break;

        //起飞要维持起飞的模式?
        case U_KEY_T:
          cout << " " <<endl;
          cout << "Switch to Takeoff Mode." <<endl;

          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.source = NODE_NAME;
          move_pub.publish(Command_to_pub);

          sleep(2.0);
        
          break;

        //起飞要维持起飞的模式?
        case U_KEY_H:
          cout << " " <<endl;
          cout << "Switch to Hold Mode." <<endl;

          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Hold;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.source = NODE_NAME;
          Command_to_pub.Reference_State.position_ref[0]     = 0;
          Command_to_pub.Reference_State.position_ref[1]     = 0;
          Command_to_pub.Reference_State.position_ref[2]     = 0;
          Command_to_pub.Reference_State.velocity_ref[0]     = 0;
          Command_to_pub.Reference_State.velocity_ref[1]     = 0;
          Command_to_pub.Reference_State.velocity_ref[2]     = 0;
          Command_to_pub.Reference_State.acceleration_ref[0] = 0;
          Command_to_pub.Reference_State.acceleration_ref[1] = 0;
          Command_to_pub.Reference_State.acceleration_ref[2] = 0;
          move_pub.publish(Command_to_pub);

          sleep(1.0);
        
          break;

        // 向前匀速运动
        case U_KEY_W:
          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.source = NODE_NAME;
          Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_to_pub.Reference_State.velocity_ref[0]     += VEL_XY_STEP_SIZE;
          move_pub.publish(Command_to_pub);
          
          cout << " " <<endl;
          cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

          sleep(0.1);
        
          break;
        
        // 向后匀速运动
        case U_KEY_S:
          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.source = NODE_NAME;
          Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_to_pub.Reference_State.velocity_ref[0]     -= VEL_XY_STEP_SIZE;
          move_pub.publish(Command_to_pub);
          cout << " " <<endl;
          cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

          sleep(0.1);

          break;

        // 向左匀速运动
        case U_KEY_A:
          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.source = NODE_NAME;
          Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_to_pub.Reference_State.velocity_ref[1]     += VEL_XY_STEP_SIZE;
          move_pub.publish(Command_to_pub);
        
          cout << " " <<endl;
          cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

          sleep(0.1);

          break;

        // 向右匀速运动
        case U_KEY_D:    
          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.source = NODE_NAME;
          Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_to_pub.Reference_State.velocity_ref[1]     -= VEL_XY_STEP_SIZE;
          move_pub.publish(Command_to_pub);

          cout << " " <<endl;
          cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

          sleep(0.1);

          break;

        // 向上运动
        case U_KEY_K:
          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.source = NODE_NAME;
          Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_to_pub.Reference_State.velocity_ref[2]     += VEL_Z_STEP_SIZE;
          move_pub.publish(Command_to_pub);

          cout << " " <<endl;
          cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

          sleep(0.1);

          break;

        // 向下运动
        case U_KEY_M:
          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.source = NODE_NAME;
          Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_to_pub.Reference_State.velocity_ref[2]     -= VEL_Z_STEP_SIZE;
          move_pub.publish(Command_to_pub);

          cout << " " <<endl;
          cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] "<<endl;

          sleep(0.1);
        
          break;

        // 偏航运动，左转 （这个里偏航控制的是位置 不是速度）
        case U_KEY_Q:
          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.source = NODE_NAME;
          Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_VEL;
          Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_to_pub.Reference_State.yaw_ref             += YAW_STEP_SIZE;
          move_pub.publish(Command_to_pub);
          
          cout << " " <<endl;
          cout << "Increase the Yaw angle." <<endl;

          sleep(0.1);
        
          break;

        // 偏航运动，右转
        case U_KEY_E:
          Command_to_pub.header.stamp = ros::Time::now();
          Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
          Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
          Command_to_pub.source = NODE_NAME;
          Command_to_pub.Reference_State.Move_mode       = prometheus_msgs::PositionReference::XYZ_POS;
          Command_to_pub.Reference_State.Move_frame      = prometheus_msgs::PositionReference::BODY_FRAME;
          Command_to_pub.Reference_State.yaw_ref             += YAW_STEP_SIZE;
          move_pub.publish(Command_to_pub);
          
          cout << " " <<endl;
          cout << "Decrease the Yaw angle." <<endl;

          sleep(0.1);
        
          break;
        

        // Run with FUEL command!
        case U_KEY_R:

          cout << " " <<endl;
          cout << "FUEL exploration starts." <<endl;
          ros::spin();

          // sleep(0.5);

          // keyboardcontrol.RosWhileLoopRun();
          // key_wait = keyboardcontrol.GetPressedKey();
          // while(key_wait == U_KEY_NONE){
          //   keyboardcontrol.RosWhileLoopRun();
          //   key_wait = keyboardcontrol.GetPressedKey();
          //   ros::spinOnce();
          // }
          
          break;
    
      sleep(0.1);
      }
    }
    return 0;
}