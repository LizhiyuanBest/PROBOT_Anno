#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <cstdlib>
using namespace ikfast_kinematics_plugin;
#define pi 3.142
bool call = true;

void chatterback(const std_msgs::String::ConstPtr& msg)
{
    call = false;
    string show = msg->data;
    std::cout << show <<endl;
}

std::vector<Eigen::Vector4f> center;//创建存储点云质心的对象
//接收到订阅的消息后，会进入消息回调函数
void SubscribeAndPublish::positionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	//将接收到的消息打印出来
    ROS_INFO("got position");
    std_msgs::Float64MultiArray position_msg = *msg;
    std::cout<<position_msg.data.size()<<std::endl;
    std_msgs::Float64MultiArray temp_msg;

    for(int i=0;i<position_msg.data.size();i=i+5){
        
        Eigen::Vector4f temp_vector;
        temp_vector(0)=position_msg.data[i+1];
        temp_vector(1)=position_msg.data[i+2];
        temp_vector(2)=position_msg.data[i+3];
        temp_vector(3)=position_msg.data[i+0];
        std::cout<<"position"<<temp_vector<<std::endl;
        center.push_back(temp_vector);
    }

}

int main(int argc, char **argv)
{
    //初始化
    bool ret;
    ros::init(argc, argv, "ik");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);

    ros::Publisher speed_code_pub = node_handle.advertise<std_msgs::String>("speed_code",100);
    ros::Subscriber sub = node_handle.subscribe("chatter",100,chatterback);
    ros::Subscriber position_sub = node_handle.subscribe("/process/position",1,chatterback);
    std::stringstream sol_send;
    std_msgs::String machinecode;

    spinner.start();
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
     //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);
    

    //创建运动学类实例
    IKFastKinematicsPlugin ik;
    //初始化对象
    ret = ik.IKFastKinematicsPlugin::initialize("robot_description","manipulator","base_link",arm.getEndEffectorLink(),0.01);
    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.2683;
    target_pose.position.y = 0.1;
    target_pose.position.z = 0.1387;
    target_pose.orientation.x = 1.0;

    /*此为门位置（笛卡尔空间运动准备位置）位姿
    target_pose.position.x = 0.22886;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.432;
    target_pose.orientation.x = 1.0;
     */

    //pose1为vector变量，装入一个位姿
    std::vector<geometry_msgs::Pose> pose1;
    pose1.push_back(target_pose);

    //设定参考点
    std::vector<double> seed1;
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    //std::vector<std::vector<double> > sol_rad(1,vector<double>(6,0));
    //std::vector<std::vector<double> > sol_ang(1,vector<double>(6,0));
    //std::vector<std::vector<int> > sol_int_ang(1,vector<int>(6,0));

    std::vector<std::vector<double>> sol_rad;//逆解的变量，弧度制

    kinematics::KinematicsResult kinematic_result;

    //计算逆解
    ret = ik.getPositionIK(pose1, seed1, sol_rad, kinematic_result, kinematics::KinematicsQueryOptions());
    double sol_rad_export[sol_rad.size()][6];
    double sol_ang[sol_rad.size()][6];//逆解的变量，角度制
    int sol_int_ang[sol_rad.size()][6];//将角度取整
    string sol_string_ang[sol_rad.size()][6];;//需要发送的字符
    /*
    sol_ang[0][0] =45;
    sol_ang[0][1] =0;
    sol_ang[0][2] =0;
    sol_ang[0][3] =0;
    sol_ang[0][4] =0;
    sol_ang[0][5] =0;*/
    if(ret)//若求解成功
    {
        std::cout << "IK solved successfully." << sol_rad.size() << endl;
        //角度制计算，并输出计算结果，rviz显示
        for(int q = 0; q < sol_rad.size(); q++)
        {
            if (!sol_rad.empty())
            {
                memcpy(sol_rad_export[q], &sol_rad[q][0], sol_rad[0].size() * sizeof(double));
            }
        }
        for(int i = 0;i<sol_rad.size();++i)
        {
            std::cout << "SOLUTION"<<i<<":"<<endl;

            for (int j = 0; j < 6; ++j)
            {
                //std::cout << " " << sol_rad[i][j];
                ROS_INFO_STREAM(" " << sol_rad[i][j]);
                sol_ang[i][j] = sol_rad_export[i][j] * 180 / pi;
                std::cout << " " << sol_ang[i][j]<<endl;
                sol_int_ang[i][j] = (int)(sol_ang[i][j]);
                sol_string_ang[i][j] = std::to_string(sol_int_ang[i][j]);
                //std::cout << "@@@" << endl;
            }
            std::cout << "Start simulating" << endl;
            std::cout << std::endl;
            //运动到第i个解的姿态
            arm.setJointValueTarget(sol_rad[i]);
            arm.move();
            sleep(2);
            //显示结束，回到准备位置
            arm.setNamedTarget("home");
            arm.move();
            sleep(1);
            sol_send.clear();
            sol_send.str("");//清空缓冲区
            for(int j = 0; j < 6; ++j)
            {

                std::cout << j << " "<<sol_string_ang[i][j] << endl;
                sol_send << sol_string_ang[i][j];
                switch(j)
                {
                    case 0:sol_send << ('a');break;
                    case 1:sol_send << ('b');break;
                    case 2:sol_send << ('c');break;
                    case 3:sol_send << ('d');break;
                    case 4:sol_send << ('e');break;
                    case 5:sol_send << ('f');break;
                }

            }

            machinecode.data = sol_send.str();

            while(ros::ok() && call)
            {
                ROS_INFO("%s", machinecode.data.c_str());
                speed_code_pub.publish(machinecode);
                //检查一否一致，或者把接收的信息返回（在callback中发回）
                ros::spinOnce();

                sleep(10);
            }
            call = true;
            sol_send.clear();
            sol_send.str("");//清空缓冲区
            for(int j = 0; j < 6; ++j)
            {

                std::cout << j << " "<<'0' << endl;
                sol_send << '0';
                switch(j)
                {
                    case 0:sol_send << ('a');break;
                    case 1:sol_send << ('b');break;
                    case 2:sol_send << ('c');break;
                    case 3:sol_send << ('d');break;
                    case 4:sol_send << ('e');break;
                    case 5:sol_send << ('f');break;
                }

            }

            machinecode.data = sol_send.str();

            while(ros::ok() && call)
            {
                ROS_INFO("%s", machinecode.data.c_str());
                speed_code_pub.publish(machinecode);
                //检查一否一致，或者把接收的信息返回（在callback中发回）
                ros::spinOnce();

                sleep(10);
            }
            call = true;

        }
    }
    else
    {
        std::cout << "IK failed." << endl;
    }

    ros::shutdown(); 

    return 0;
}
