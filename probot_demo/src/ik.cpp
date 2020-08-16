#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"
#include "std_msgs/String.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sstream>
#include <iostream>
#include <cstdlib>
using namespace ikfast_kinematics_plugin;
#define pi 3.142


int main(int argc, char **argv)
{
    //初始化
    bool ret;
    ros::init(argc, argv, "ik");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);

    ros::Publisher speed_code_pub = node_handle.advertise<std_msgs::Float32MultiArray>("position_chatter",1000);

    std_msgs::Float32MultiArray sol_send;
 

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
    target_pose.position.z = 0.23;
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

    std::vector<std::vector<double>> sol_rad;//逆解的变量，弧度制

    kinematics::KinematicsResult kinematic_result;

    //计算逆解
    ret = ik.getPositionIK(pose1, seed1, sol_rad, kinematic_result, kinematics::KinematicsQueryOptions());
    double sol_rad_export[sol_rad.size()][6];
    double sol_ang[sol_rad.size()][6];//逆解的变量，角度制
    int sol_int_ang[sol_rad.size()][6];//将角度取整
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
        for(int i = 0;i<1;++i)
        {
            std::cout << "SOLUTION"<<i<<":"<<endl;
            //运动到第i个解的姿态
            arm.setJointValueTarget(sol_rad[i]);
            arm.move();
            sleep(2);
            // //显示结束，回到准备位置
            // arm.setNamedTarget("home");
            // arm.move();
            // sleep(1);

            sol_send.data.clear();
            for (int j = 0; j < 6; ++j)
            {
                //std::cout << " " << sol_rad[i][j];
                ROS_INFO_STREAM(" " << sol_rad[i][j]);
                sol_ang[i][j] = sol_rad_export[i][j] * 180 / pi;
                std::cout << " " << sol_ang[i][j]<<endl;
                switch(j)
                {
                    case 0:sol_send.data.push_back(sol_ang[i][j]*30) ;break;
                    case 1:sol_send.data.push_back(sol_ang[i][j]*205/3) ;break;
                    case 2:sol_send.data.push_back(sol_ang[i][j]*50) ;break;
                    case 3:sol_send.data.push_back(sol_ang[i][j]*125/2) ;break;
                    case 4:sol_send.data.push_back(sol_ang[i][j]*125/2) ;break;
                    case 5:sol_send.data.push_back(sol_ang[i][j]*200/9) ;break;
                }
            }
            sol_send.data.push_back(1000.0) ;//speed
            std::cout << "Start simulating" << endl;
            std::cout << std::endl;

            speed_code_pub.publish(sol_send);
            sleep(10);

            

        }
    }
    else
    {
        std::cout << "IK failed." << endl;
    }

    //显示结束，回到准备位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    sol_send.data.clear();
    for (int j = 0; j < 6; ++j)
    {
        switch(j)
        {
            case 0:sol_send.data.push_back(0.0) ;break;
            case 1:sol_send.data.push_back(0.0) ;break;
            case 2:sol_send.data.push_back(0.0) ;break;
            case 3:sol_send.data.push_back(0.0) ;break;
            case 4:sol_send.data.push_back(0.0) ;break;
            case 5:sol_send.data.push_back(0.0) ;break;
        }
    }
    sol_send.data.push_back(1000.0) ;//speed
    speed_code_pub.publish(sol_send);
    sleep(10);

    ros::shutdown(); 

    return 0;
}
