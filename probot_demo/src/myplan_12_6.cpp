#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <math.h>

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
        Eigen::Matrix3d R;  //rovate
        Eigen::Vector3d T;  //transform
        Eigen::Vector3d C;  //center 
        double ang;     
};

std::vector<Eigen::Vector4f> center;//创建存储点云质心的对象
std::vector<myMatch> match;//创建存储点云质心的对象
std::vector<geometry_msgs::Pose> pose_match;//创建存储点云质心的对象
//手眼标定的平移矩阵
// const double T[3] = { -0.11,-0.343,-0.37 };//驱动机器人到中心位置记下此时机器人位置
const double T[3] = { -0.145,-0.376,-0.37 };
const double Model_Pose[7] = { -0.11,-0.325,-0.37,1.0,0.0,0.0,0.0 };

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
            // std::cout<<"position"<<temp_vector<<std::endl;
            center.push_back(temp_vector);
        }
        get_position = true;
    }
	

}
bool get_pose = false;
//接收到订阅的消息后，会进入消息回调函数
void poseCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    geometry_msgs::Pose model_pose ;
    model_pose.position.x = Model_Pose[0];
    model_pose.position.y = Model_Pose[1];
    model_pose.position.z = Model_Pose[2];
    model_pose.orientation.x = Model_Pose[3];
    model_pose.orientation.y = Model_Pose[4];
    model_pose.orientation.z = Model_Pose[5];
    model_pose.orientation.w = Model_Pose[6];
    Eigen::Quaterniond model_q;
    model_q.x() = model_pose.orientation.x;
    model_q.y() = model_pose.orientation.y;
    model_q.z() = model_pose.orientation.z;
    model_q.w() = model_pose.orientation.w;
    
    Eigen::Vector3d model_euler = model_q.toRotationMatrix().eulerAngles(2, 1, 0);  
    cout << "model Quaterniond2Euler result is: " << std::endl;  
    cout <<" x = "<< model_euler[2]*180/pi<< " y = "<< model_euler[1]*180/pi<< " z = " << model_euler[0]*180/pi << std::endl ;  
	//将接收到的消息打印出来
    ROS_INFO("got pose");
    std_msgs::Float64MultiArray pose_msg = *msg;
    std::cout<<pose_msg.data.size()<<std::endl;
    Eigen::Matrix3d temp_R;
    if(!get_pose)
    {
        match.clear();
        for(int i=0;i<pose_msg.data.size();i=i+4){
            myMatch temp_match;
            // std::cout<<pose_msg.data[i]<<std::endl;
            temp_match.model = (int) pose_msg.data[i];
            i++;
            for(int k=0;k<3;k++,i++){
                temp_match.C(k) = pose_msg.data[i]+T[k];
            }
            for(int j=0;j<3;j++,i=i+4){
                temp_match.R(j,0) = pose_msg.data[i];
                temp_match.R(j,1) = pose_msg.data[i+1];
                temp_match.R(j,2) = pose_msg.data[i+2];
                temp_match.T(j) = pose_msg.data[i+3];
            }
            temp_match.ang = atan2(temp_match.R(0,1),temp_match.R(0,0)) *180/pi;
            // if(temp_match.ang<0){
            //     temp_match.ang=-temp_match.ang;
            // }
            // temp_match.R.normalize();
            match.push_back(temp_match);
        
            std::cout<<"model:"<<temp_match.model<<std::endl;
            std::cout<<"R:"<<temp_match.R<<std::endl;
            std::cout<<"T:"<<temp_match.T<<std::endl;
            std::cout<<"C:"<<temp_match.C<<std::endl;
            std::cout<<"ang:"<<temp_match.ang<<std::endl;
        }

        pose_match.clear();
        for(int i=0;i<match.size();i++)
        {
            geometry_msgs::Pose target_pose;
            Eigen::Quaterniond temp_q;
            myMatch temp_thispose = match.at(i);

            Eigen::Quaterniond q = Eigen::Quaterniond(temp_thispose.R); 
            Eigen::Quaterniond q_; 
            // q.normalize(); 
            q_.x() = -q.x();
            q_.y() = -q.y();
            q_.z() = -q.z();
            q_.w() =  q.w();
            temp_q = q*model_q;//*q_;
            temp_q.z() = 0;
            temp_q.w() = 0;
            temp_q.normalize();
            Eigen::Vector3d temp_euler = temp_q.toRotationMatrix().eulerAngles(2, 1, 0);  
            cout << "temp Quaterniond2Euler result is: " << std::endl;  
            cout <<" x = "<< temp_euler[2]*180/pi<< " y = "<< temp_euler[1]*180/pi<< " z = " << temp_euler[0]*180/pi << std::endl ;
            Eigen::Vector3d temp_euler_r = temp_thispose.R.eulerAngles(2, 1, 0);  
            cout << "temp Quaterniond2Euler result is: " << std::endl;  
            cout <<" x = "<< temp_euler_r[2]*180/pi<< " y = "<< temp_euler_r[1]*180/pi<< " z = " << temp_euler_r[0]*180/pi << std::endl ;
            
            // target_pose.position.x=model_pose.position.x + temp_thispose.T[0];
            // target_pose.position.y=model_pose.position.y + temp_thispose.T[1];
            // target_pose.position.z=model_pose.position.z; // + temp_thispose.T[2];
            // target_pose.orientation.x=temp_q.x();
            // target_pose.orientation.y=temp_q.y();
            // target_pose.orientation.z=temp_q.z();
            // target_pose.orientation.w=temp_q.w();
            target_pose.position.x=temp_thispose.C[0];
            target_pose.position.y=temp_thispose.C[1];
            target_pose.position.z=0.21; //temp_thispose.C[2];
            target_pose.orientation.x=temp_q.x();
            target_pose.orientation.y=temp_q.y();
            target_pose.orientation.z=temp_q.z();
            target_pose.orientation.w=temp_q.w();
            pose_match.push_back(target_pose);
            std::cout<<"target_pose: "<<std::endl<<target_pose<<std::endl;
        }
        get_pose = true;
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
    // ros::Subscriber position_sub = node_handle.subscribe("/process/position",1,positionCallback);

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

    // target_pose.position.x = -0.115;
    // target_pose.position.y = -0.327;
    // target_pose.position.z = 0.21;
    // target_pose.orientation.x = 1.0;
    // target_pose.orientation.y = 0.0;
    // target_pose.orientation.z = 0.0;
    // target_pose.orientation.w = 0.0;
    // mytgt_pose.push_back(target_pose);



    // geometry_msgs::Pose model_pose ;
    // model_pose.position.x = Model_Pose[0];
    // model_pose.position.y = Model_Pose[1];
    // model_pose.position.z = 0.21; //Model_Pose[2];
    // model_pose.orientation.x = Model_Pose[3];
    // model_pose.orientation.y = Model_Pose[4];
    // model_pose.orientation.z = Model_Pose[5];
    // model_pose.orientation.w = Model_Pose[6];
    // mytgt_pose.push_back(model_pose);

    // while(!get_position)
    // {
    //     sleep(1);
    // }
    // if(get_position )
    // {
    //     for(int i=0;i<center.size();i++)
    //     {
    //         geometry_msgs::Pose target_position;
    //         Eigen::Vector4f temp_vector;
    //         temp_vector=center.at(i);
    //         target_position.position.x=temp_vector(0);
    //         target_position.position.y=temp_vector(1);
    //         target_position.position.z=0.21; //temp_vector(2);
    //         target_position.orientation.x=1.0;
    //         mytgt_pose.push_back(target_position);
    //     }

    // }
   
    while(!get_pose)
    {
        sleep(1);
    }
    if(get_pose)
    {
        for(int i=0;i<pose_match.size();i++)
        {
            target_pose = pose_match.at(i);
            mytgt_pose.push_back(target_pose);
            // std::cout<<"target_pose: "<<std::endl<<target_pose<<std::endl;
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



    double angle = 0.0;
    // 循环执行
    for(int k=0;k<mytgt_pose.size();k++){

        //pose1为vector变量，装入一个位姿
        std::vector<geometry_msgs::Pose> pose1;
        pose1.push_back(mytgt_pose[k]);

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
            angle = match.at(k).ang - angle;
            if(angle>360)  angle -= 360;
            if(angle>180)  angle -= 360;
            if(angle<-360) angle += 360;
            if(angle<-180) angle += 360;
            if(angle>90&&angle<=180) angle = -180+angle;
            if(angle<-90&&angle>=-180) angle = 180+angle;

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
                        case 5:sol_ang[i][j] = angle;sol_send.data.push_back(sol_ang[i][j]*200/9) ;break;//sol_ang[i][j] = angle; //sol_ang[i][j] = 0; 
                    }
                }
                //sol_send.data.push_back(1000) ;
                std::cout << "Start simulating" << endl;
                std::cout << std::endl;

                //for(int i=0;i<20;i++)
                speed_code_pub.publish(sol_send);
                sleep(10);
                call = true;
                
            }
        }
        else
        {
            std::cout << "IK failed." << endl;
        }
        sol_rad.clear();
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
