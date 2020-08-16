#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"
#include <time.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv) {

    bool ret;
    ros::init(argc, argv, "ik");
    ros::NodeHandle node_handle;
    //吸盘控制
    ros::Publisher suction_pub = node_handle.advertise<std_msgs::Bool>("suction_chatter",10);
    //夹爪控制
    ros::Publisher gripper_pub = node_handle.advertise<std_msgs::Bool>("gripper_chatter",10);
    //定义bool变量并赋值
    std_msgs::Bool suction_signal;
    std_msgs::Bool gripper_signal;
    suction_signal.data = 0;
    gripper_signal.data = 0;
    //下面使吸盘和夹爪间歇切换工作
    while(1)
    {
        //发布0，夹爪张开，或吸盘释放
        suction_signal.data = 0;
        gripper_signal.data = 0;
        suction_pub.publish(suction_signal);
        gripper_pub.publish(gripper_signal);
        std::cout<<"stop"<<std::endl;
        sleep(2);
        //发布1，夹爪闭合，或吸盘吸取
        suction_signal.data = 1;
        gripper_signal.data = 1;
        suction_pub.publish(suction_signal);
        gripper_pub.publish(gripper_signal);
        std::cout<<"activate"<<std::endl;
        sleep(2);
    }
}


