#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <cstdlib>

#include <std_msgs/Float64MultiArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>
#include "myprocess/myprocess_pose.h"
#include "myprocess/myprocess_position.h"

using namespace ikfast_kinematics_plugin;
#define pi 3.142
bool call = true;

class myMatch {
    public:
        int model;
        Eigen::Matrix3d R;
        Eigen::Vector3d T;        
};

std::vector<Eigen::Vector4f> center;//创建存储点云质心的对象
std::vector<myMatch> match;//创建存储点云质心的对象

//手眼标定的平移矩阵
// const double T[3] = { -0.11,-0.343,-0.37 };//驱动机器人到中心位置记下此时机器人位置
const double T[3] = { -0.13,-0.313,-0.37 };

void chatterback(const std_msgs::String::ConstPtr& msg)
{
    call = false;
    string show = msg->data;
    std::cout << show <<endl;
}

Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w)  
{  
    Eigen::Quaterniond q;  
    q.x() = x;  
    q.y() = y;  
    q.z() = z;  
    q.w() = w;  
  
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();  
    cout << "Quaternion2RotationMatrix result is:" <<endl;  
    cout << "R = " << endl << R << endl<< endl;  
    return R;  
}  
  
  
Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R)  
{  
    Eigen::Quaterniond q = Eigen::Quaterniond(R);  
    q.normalize();  
    cout << "RotationMatrix2Quaterniond result is:" <<endl;  
    cout << "x = " << q.x() <<endl;  
    cout << "y = " << q.y() <<endl;  
    cout << "z = " << q.z() <<endl;  
    cout << "w = " << q.w() <<endl<<endl;  
    return q;  
}  

bool get_position = false;

//接收到订阅的消息后，会进入消息回调函数
void positionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    //将接收到的消息打印出来
    ROS_INFO("got position");
    std_msgs::Float64MultiArray position_msg = *msg;
    std::cout<<position_msg.data.size()<<std::endl;
    std_msgs::Float64MultiArray temp_msg;
    if(!get_position)
    {
        for(int i=0;i<position_msg.data.size();i=i+5){
            
            Eigen::Vector4f temp_vector;
            temp_vector(0)=position_msg.data[i+1]+T[0];
            temp_vector(1)=position_msg.data[i+2]+T[1];
            temp_vector(2)=position_msg.data[i+3]+T[2];  //position_msg.data[i+3];
            temp_vector(3)=position_msg.data[i+0];
            std::cout<<"position"<<temp_vector<<std::endl;
            center.push_back(temp_vector);
        }
        get_position = true;
    }
	

}

//接收到订阅的消息后，会进入消息回调函数
void poseCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	//将接收到的消息打印出来
    ROS_INFO("got pose");
    std_msgs::Float64MultiArray pose_msg = *msg;
    std::cout<<pose_msg.data.size()<<std::endl;
    Eigen::Matrix3d temp_R;


    for(int i=0;i<pose_msg.data.size();i=i+4){
        myMatch temp_match;
        std::cout<<pose_msg.data[i]<<std::endl;
        temp_match.model = (int) pose_msg.data[i+1];
        i += 4;
        for(int j=0;j<3;j++,i=i+4){
            temp_match.R(j,0) = pose_msg.data[i];
            temp_match.R(j,1) = pose_msg.data[i+1];
            temp_match.R(j,2) = pose_msg.data[i+2];
            temp_match.T(j) = pose_msg.data[i+3];
        }
        match.push_back(temp_match);
       
        std::cout<<"model:"<<temp_match.model<<std::endl;
        std::cout<<"R:"<<temp_match.R<<std::endl;
        std::cout<<"T:"<<temp_match.T<<std::endl;
    }
}



int main(int argc, char **argv)
{
    //初始化
    bool ret;
    ros::init(argc, argv, "ik");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(3);


    //Topic you want to subscribe  
    ros::Subscriber pose_sub = node_handle.subscribe("/process/pose",1,poseCallback);
    ros::Subscriber position_sub = node_handle.subscribe("/process/position",1,positionCallback);

    ros::Publisher speed_code_pub = node_handle.advertise<std_msgs::Float64MultiArray>("Float64_chatter",100);
    ros::Subscriber sub = node_handle.subscribe("chatter",100,chatterback);
    std_msgs::Float64MultiArray sol_send;

    spinner.start();
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
     //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    //创建运动学类实例
    IKFastKinematicsPlugin ik;
    //初始化对象
    ret = ik.IKFastKinematicsPlugin::initialize("robot_description","manipulator","base_link",arm.getEndEffectorLink(),0.01);
    

    //设置几个点
    std::vector<geometry_msgs::Pose> mytgt_pose;
    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose;

    /*此为门位置（笛卡尔空间运动准备位置）位姿
    target_pose.position.x = 0.22886;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.432;
    target_pose.orientation.x = 1.0;
     */

    // target_pose.position.x = 0.26;
    // target_pose.position.y = 0.0;
    // target_pose.position.z = 0.23;
    // target_pose.orientation.x = 1.0;
    // mytgt_pose.push_back(target_pose);

    // target_pose.position.x = 0.325;
    // target_pose.position.y = -0.12;
    // target_pose.position.z = 0.23;
    // target_pose.orientation.x = 1.0;
    // mytgt_pose.push_back(target_pose);

    // target_pose.position.x = -0.105;
    // target_pose.position.y = -0.347;
    // target_pose.position.z = 0.21;
    // target_pose.orientation.x = 1.0;
    // mytgt_pose.push_back(target_pose);

    // target_pose.position.x = -0.000942;
    // target_pose.position.y = -0.01111;
    // target_pose.position.z = 0.2387;
    // target_pose.orientation.x = 1.0;
    // mytgt_pose.push_back(target_pose);

    // target_pose.position.x = -0.0804;
    // target_pose.position.y = -0.0266;
    // target_pose.position.z = 0.2387;
    // target_pose.orientation.x = 1.0;
    // mytgt_pose.push_back(target_pose);


    if(get_position)
    {
        for(int i=0;i<center.size();i++)
        {
            geometry_msgs::Pose target_position;
            Eigen::Vector4f temp_vector;
            temp_vector=center.at(i);
            target_position.position.x=temp_vector(0);
            target_position.position.y=temp_vector(1);
            target_position.position.z=temp_vector(2);
            target_position.orientation.x=1.0;
            mytgt_pose.push_back(target_position);
        }

    }


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


 
    // 循环执行
    for(int i=0;i<mytgt_pose.size();i++){

        //pose1为vector变量，装入一个位姿
        std::vector<geometry_msgs::Pose> pose1;
        pose1.push_back(mytgt_pose[i]);

        std::vector<std::vector<double>> sol_rad;//逆解的变量，弧度制
        kinematics::KinematicsResult kinematic_result;
        //计算逆解
        ret = ik.getPositionIK(pose1, seed1, sol_rad, kinematic_result, kinematics::KinematicsQueryOptions());
        
        double sol_ang[sol_rad.size()][6];//逆解的变量，角度制
        double sol_rad_export[sol_rad.size()][6];
        
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
            // for(int i = 0;i<sol_rad.size();++i)
            for(int i = 0;i<1;++i)
            {
                std::cout << "SOLUTION"<<i<<":"<<endl;
                //运动到第i个解的姿态
                arm.setJointValueTarget(sol_rad[i]);
                //arm.getCurrentPose();
                arm.move();
                sleep(2);

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
                        case 5:sol_send.data.push_back(sol_ang[i][j]*200/9) ;break; //sol_ang[i][j] = 0; 
                    }
                }
                //sol_send.data.push_back(1000) ;
                std::cout << "Start simulating" << endl;
                std::cout << std::endl;
               

                //geometry_msgs::Pose cur_pose = arm.get;
                //std::cout<<cur_pose << std::endl;
                //显示结束，回到准备位置
                // arm.setNamedTarget("home");
                // arm.move();
                // sleep(1);
                speed_code_pub.publish(sol_send);
                sleep(10);
                call = true;
                sol_rad.clear();

            }


        }
        else
        {
            std::cout << "IK failed." << endl;
        }

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

    speed_code_pub.publish(sol_send);
    sleep(10);
    call = true;

    ros::shutdown(); 

    return 0;
}
