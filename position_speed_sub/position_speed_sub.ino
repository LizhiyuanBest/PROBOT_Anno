#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <inttypes.h>

const int suction_io = 47; //suction cup signal
const int gripper_io = 39; //gripper signal
const int J1pul = 10; //一号电机脉冲
const int J1dir = 11; //一号电机转向

const int J2pul = 8; //一号电机脉冲
const int J2dir = 9; //一号电机转向

const int J3pul = 6; //一号电机脉冲
const int J3dir = 7; //一号电机转向

const int J4pul = 4; //一号电机脉冲
const int J4dir = 5; //一号电机转向

const int J5pul = 2; //一号电机脉冲
const int J5dir = 3; //一号电机转向

const int J6pul = 14; //一号电机脉冲
const int J6dir = 15; //一号电机转向

AccelStepper J6(1,J6pul,J6dir);//对象5
AccelStepper J5(1,J5pul,J5dir);//对象6
AccelStepper J4(1,J4pul,J4dir);//对象4
AccelStepper J3(1,J3pul,J3dir);//对象3
AccelStepper J2(1,J2pul,J2dir);//对象2
AccelStepper J1(1,J1pul,J1dir);//对象1


ros::NodeHandle  nh;

float pals[100][6];
double init_p[6];
int prePos[6] = {0,0,0,0,0,0};
double maxvel = 800.0;
double maxaccel = 800.0;
int i = 0;
int j = 0;
double diff[6];
double maxdiff = 0;
bool flag = 0;
bool run_flag = 0;
bool flag64 = 0;
bool run_flag64 = 0;
bool setmaxspeed_flag = 0;

void(* resetFunc) (void) = 0;

void speed_callback(const std_msgs::Float32MultiArray& speed_msg)
{
  J6.setAcceleration(1500); 
  J5.setAcceleration(1500); 
  J4.setAcceleration(1500); 
  J3.setAcceleration(1500); 
  J2.setAcceleration(1500); 
  J1.setAcceleration(1500); 
  for(i = 0; i < 1; i++)
  {
    for(j = 0; j < 6; j++)
    {
      pals[i][j] = speed_msg.data[j];
    }
  }
  flag = 1;
  setmaxspeed_flag = 1;
}

void position_callback(const std_msgs::Float32MultiArray& position_msg)
{
  for(j = 0; j < 6; j++)
    {
      init_p[j] = position_msg.data[j];
    }
  maxvel = position_msg.data[6];
  // maxvel = 1600;
  maxaccel = 1500;
  
  for(int i = 0; i < 6;i++)
    {
      diff[i] = abs(prePos[i] - init_p[i]);
      maxdiff = max(maxdiff,diff[i]);
    }
  J6.setAcceleration(maxaccel);// * diff[5]/maxdiff); 
  J5.setAcceleration(maxaccel);// * diff[4]/maxdiff); 
  J4.setAcceleration(maxaccel);// * diff[3]/maxdiff); 
  J3.setAcceleration(maxaccel);// * diff[2]/maxdiff); 
  J2.setAcceleration(maxaccel);// * diff[1]/maxdiff); 
  J1.setAcceleration(maxaccel);// * diff[0]/maxdiff); 
  
  J6.setMaxSpeed(maxvel * diff[5]/maxdiff); 
  J5.setMaxSpeed(maxvel * diff[4]/maxdiff); 
  J4.setMaxSpeed(maxvel * diff[3]/maxdiff); 
  J3.setMaxSpeed(maxvel * diff[2]/maxdiff); 
  J2.setMaxSpeed(maxvel * diff[1]/maxdiff); 
  J1.setMaxSpeed(maxvel * diff[0]/maxdiff);
  maxdiff = 0; // clear this
  /*
  J6.setSpeed(300); 
  J5.setSpeed(600); 
  J4.setSpeed(300); 
  J3.setSpeed(300); 
  J2.setSpeed(300); 
  J1.setSpeed(300);*/
  J6.moveTo(int(init_p[5])); 
  J5.moveTo(-int(init_p[4])); 
  J4.moveTo(int(init_p[3])); 
  J3.moveTo(int(init_p[2])); 
  J2.moveTo(int(init_p[1])); 
  J1.moveTo(int(init_p[0]));
  
  prePos[0] = int(init_p[0]);
  prePos[1] = int(init_p[1]);
  prePos[2] = int(init_p[2]);
  prePos[3] = int(init_p[3]);
  prePos[4] = -int(init_p[4]);
  prePos[5] = int(init_p[5]);
  flag64 = 1;
}

void suction_callback(const std_msgs::Bool& suction_msg)
{
  digitalWrite(suction_io, suction_msg.data);
}

void gripper_callback(const std_msgs::Bool& gripper_msg)
{
  digitalWrite(gripper_io, gripper_msg.data);
}

ros::Subscriber<std_msgs::Float32MultiArray>speed_sub("speed_chatter", &speed_callback);
ros::Subscriber<std_msgs::Float32MultiArray>position_sub("position_chatter", &position_callback);
ros::Subscriber<std_msgs::Bool>suction_sub("suction_chatter", &suction_callback);//D47
ros::Subscriber<std_msgs::Bool>gripper_sub("gripper_chatter", &gripper_callback);//D39

void setup()
{
  Serial.begin(9600);
  pinMode(J1pul, OUTPUT);
  pinMode(J1dir, OUTPUT);
  pinMode(J2pul, OUTPUT);
  pinMode(J2dir, OUTPUT);
  pinMode(J3pul, OUTPUT);
  pinMode(J3dir, OUTPUT);
  pinMode(J4pul, OUTPUT);
  pinMode(J4dir, OUTPUT);
  pinMode(J5pul, OUTPUT);
  pinMode(J5dir, OUTPUT);
  pinMode(J6pul, OUTPUT);
  pinMode(J6dir, OUTPUT);
  pinMode(suction_io,OUTPUT);
  pinMode(gripper_io,OUTPUT);
  J6.setMaxSpeed(1200); 
  J5.setMaxSpeed(1200); 
  J4.setMaxSpeed(1200); 
  J3.setMaxSpeed(1200); 
  J2.setMaxSpeed(1200); 
  J1.setMaxSpeed(1200); 
 
  J6.setAcceleration(500); 
  J5.setAcceleration(500); 
  J4.setAcceleration(500); 
  J3.setAcceleration(500); 
  J2.setAcceleration(500); 
  J1.setAcceleration(500); 

  J6.setSpeed(0); 
  J5.setSpeed(0); 
  J4.setSpeed(0); 
  J3.setSpeed(0); 
  J2.setSpeed(0); 
  J1.setSpeed(0);
  
  nh.initNode();
  nh.subscribe(speed_sub);
  nh.subscribe(position_sub);
  nh.subscribe(suction_sub);
  nh.subscribe(gripper_sub);
  
  //nh.subscribe(sub_reset);
}

void loop()
{
  nh.spinOnce();
  if(setmaxspeed_flag)
  {
    J6.setMaxSpeed(1600); 
    J5.setMaxSpeed(1600); 
    J4.setMaxSpeed(1600); 
    J3.setMaxSpeed(1600); 
    J2.setMaxSpeed(1600); 
    J1.setMaxSpeed(1600);
    setmaxspeed_flag = 0;
  }
  if(flag)
  {
    J6.setSpeed(pals[0][5]); 
    J5.setSpeed(-pals[0][4]); 
    J4.setSpeed(pals[0][3]); 
    J3.setSpeed(pals[0][2]); 
    J2.setSpeed(pals[0][1]); 
    J1.setSpeed(pals[0][0]);
    
    flag = 0;
    run_flag = 1;
    run_flag64 = 0;
  }
  if(flag64)
  {
    flag64 = 0;
    run_flag = 0;
    run_flag64 = 1;
  }
  if(run_flag)
  {
    J1.runSpeed();
    J2.runSpeed();
    J3.runSpeed();
    J4.runSpeed();
    J5.runSpeed();
    J6.runSpeed();
  }
  if(run_flag64)
  {
    J1.run();
    J2.run();
    J3.run();
    J4.run();
    J5.run();
    J6.run();
  }
}
