
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <bucketbot_driver/CliffEvent.h>
#include <bucketbot_driver/Dockir.h>

#include <boost/assign.hpp>
//#include <iostream>

#include <stdio.h>
#include <math.h>
#include <pigpio.h>
#include <bucketbot_driver/rotary_encoder.hpp>
#include "../include/bucketbot_driver/ir_hasher.h"

//编码器
#define LEFT_ENCODER_PINA      23
#define LEFT_ENCODER_PINB      24

#define RIGHT_ENCODER_PINA     20
#define RIGHT_ENCODER_PINB     21

//电机PWM和转向
#define LEFT_MOTOR_PWM         17
#define LEFT_MOTOR_DIRECTION   2
#define LEFT_MOTOR_EN          5

#define RIGHT_MOTOR_PWM        27
#define RIGHT_MOTOR_DIRECTION  3
#define RIGHT_MOTOR_EN         4

//ir_dropsensor
#define FL_IRCLIFFSENSOR       6
#define FC_IRCLIFFSENSOR       13
#define FR_IRCLIFFSENSOR       26 

//两轮子的间距-米
#define WHEEL_SEPARATION       0.3347    //0.320
//轮子半径-米
#define WHEEL_RADIUS           0.0625

#define KP 1.3
#define KI 0.0
#define KD 0.15

//轮子前进每米编码器产生的脉冲数
#define TICKSPERMETER  76394.4
//每弧度（轮子的，注意乘以减速比）编码器产生的脉冲数
#define TICKSPERRADIAN 4774.6

//------------------------------------ir callback---------------------------------------------

char dockir_flag_l=0, dockir_flag_r=0,dockir_flag_c=0; 
char cb_flag_l = 0, cb_flag_r = 0, cb_flag_c = 0;
uint32_t hash_l, hash_r, hash_c;

void callback_l_ir(uint32_t hash_l)
{ 
   cb_flag_l = 1;
   printf("hash_l=%u\n", hash_l);
   if (2878232560==hash_l){
      dockir_flag_l = 1;
   }
   else if (1869374056==hash_l){
      dockir_flag_l = 2;
      printf("-------------------------------2---------------------------");
   }
   else if (2759406791==hash_l){
      dockir_flag_l = 4;
   }
   else 
      dockir_flag_l = 0;
  
}

void callback_c_ir(uint32_t hash_c)
{
   printf("here--------------------------------------------");
   cb_flag_c = 1;
   printf("hash_c=%u\n", hash_c);
   if (2878232560==hash_c){
      dockir_flag_c = 1;
   }
   else if (1869374056==hash_c){
      dockir_flag_c = 2;
   }
   else if (2759406791==hash_c){
      dockir_flag_c = 4;
   }
   else
      dockir_flag_c = 0;
   
}

void callback_r_ir(uint32_t hash_r)
{
   cb_flag_r = 1;
   printf("hash_r=%u\n", hash_r);
   if (2878232560==hash_r){
      dockir_flag_r = 1;
   }
   else if (1869374056==hash_r){
      dockir_flag_r = 2;
   }
   else if (2759406791==hash_r){
      dockir_flag_r = 4;
   }
   else
      dockir_flag_r = 0;
   
}


//------------------------------------encoder callback----------------------------------------
//编码器读数
static long int left_encoder_count = 0;
static long int right_encoder_count = 0;
static long int lspeed_encoder_count = 0;
static long int rspeed_encoder_count = 0;
int rcallbackcount = 0;
float left_speed_cmd = 0.0;
float right_speed_cmd = 0.0;

void left_encoder_callback(int way)
{
  left_encoder_count += way;
  lspeed_encoder_count+=way;
  //std::cout << "posl=" << left_encoder_count << std::endl;
}

void right_encoder_callback(int way)
{
  right_encoder_count += way;
  rspeed_encoder_count+= way;
  //std::cout << "posr=" << right_encoder_count << std::endl;
  //if(rcallbackcount>51)
  //rcallbackcount=51;
  if(right_speed_cmd>0){
     rcallbackcount++;
     if(rcallbackcount==52)
     {
        right_encoder_count=right_encoder_count-1;
        rspeed_encoder_count=rspeed_encoder_count-1;
        rcallbackcount=0;
        
     }
  }

  if(right_speed_cmd<0){
     rcallbackcount++;
     if(rcallbackcount==50)
     {
        right_encoder_count=right_encoder_count+1;
        rspeed_encoder_count=rspeed_encoder_count+1;
        rcallbackcount=0;
     }
  }
}


//------------------------------------cmd_vel callback---------------------------------------
//上位机速度命令，弧度/秒


void cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  float cmd_x = cmd_vel->linear.x;
  float cmd_theta = cmd_vel->angular.z;	
  //根据线速度和角速度计算左右轮子转速，弧度/秒
  left_speed_cmd  = 100.0*(cmd_x - cmd_theta * WHEEL_SEPARATION / 2.0)/WHEEL_RADIUS;
  right_speed_cmd = 100.0*(cmd_x + cmd_theta * WHEEL_SEPARATION / 2.0)/WHEEL_RADIUS;			
}


//------------------------------------PID controller---------------------------------------
//last encoder ticks
long int last_left_encoder_count = 0;
long int last_right_encoder_count = 0;

//编码器测得轮子转速，rad/s
float left_speed = 0.0;
float last_left_speed = 0.0;
float right_speed = 0.0;
float last_right_speed = 0.0;

//速度偏差
float left_speed_integral = 0.0;
float right_speed_integral = 0.0;

float last_left_speed_error = 0.0;
float last_right_speed_error = 0.0;

void control_loop(const ros::TimerEvent& event)
{
  float control_dt = (event.current_real - event.last_real).toSec();

  //left_speed = ((left_encoder_count - last_left_encoder_count)/TICKSPERRADIAN)/control_dt;
  //right_speed = ((right_encoder_count - last_right_encoder_count)/TICKSPERRADIAN)/control_dt;
  left_speed = (lspeed_encoder_count/TICKSPERRADIAN)/control_dt;
  right_speed = (rspeed_encoder_count/TICKSPERRADIAN)/control_dt;  

  printf("lspeed:%d\n",lspeed_encoder_count);
  int left_pwm_cmd = 0;
  int right_pwm_cmd = 0;

  //!!编码器测得的轮子速度正负必须和上位机命令的正负一致
  //一般逆时针旋转为正
  float left_speed_error = left_speed_cmd - left_speed;
  float right_speed_error = right_speed_cmd - right_speed;

  left_speed_integral += left_speed_error * control_dt;
  right_speed_integral += right_speed_error * control_dt;

  float left_speed_derivative = (left_speed_error - last_left_speed_error) / control_dt;
  float right_speed_derivative = (right_speed_error - last_right_speed_error) / control_dt;

  //PID
  left_pwm_cmd = KP*left_speed_error + KI*left_speed_integral + KD*left_speed_derivative;
  right_pwm_cmd = KP*right_speed_error + KI*right_speed_integral + KD*right_speed_derivative;

  if(left_speed_cmd==0){
     gpioWrite(LEFT_MOTOR_EN, PI_HIGH);
  }
  else
     gpioWrite(LEFT_MOTOR_EN, PI_LOW);
  if(right_speed_cmd==0){
     gpioWrite(RIGHT_MOTOR_EN, PI_HIGH);
  }
  else
     gpioWrite(RIGHT_MOTOR_EN, PI_LOW);

  //设置电机转向，根据实际测试调整正反转
  if (left_speed_cmd < 0)
    gpioWrite(LEFT_MOTOR_DIRECTION, PI_LOW);
  else
    gpioWrite(LEFT_MOTOR_DIRECTION, PI_HIGH);

  if (right_speed_cmd < 0)
    gpioWrite(RIGHT_MOTOR_DIRECTION, PI_HIGH);
  else
    gpioWrite(RIGHT_MOTOR_DIRECTION, PI_LOW);

  //设置电机转速，pwm range 0-1000
  if (fabs(left_pwm_cmd) > 1000)
    left_pwm_cmd = 1000;
  gpioPWM(LEFT_MOTOR_PWM, fabs(left_pwm_cmd));

  if (fabs(right_pwm_cmd) > 1000)
    right_pwm_cmd = 1000;
  gpioPWM(RIGHT_MOTOR_PWM, fabs(right_pwm_cmd));


  last_left_speed_error = left_speed_error;
  last_right_speed_error = right_speed_error;


  lspeed_encoder_count = 0;
  rspeed_encoder_count = 0;
}


//------------------------------------calculate odom---------------------------------------
//位置
float px = 0.0;
float py = 0.0;
float theta = 0.0;

//速度
float vx = 0.0;
float vth = 0.0;

void calc_odom(float dt)
{
  float dleft = (left_encoder_count - last_left_encoder_count)/TICKSPERMETER;
  float dright = (right_encoder_count - last_right_encoder_count)/TICKSPERMETER;

  printf("left:%d\n",left_encoder_count-last_left_encoder_count);
  printf("right:%d\n",right_encoder_count-last_right_encoder_count);
  //printf("last:%d\n",last_left_encoder_count);
  printf("dleft:%f\n",dleft);
  printf("dright:%f\n",dright);
    
  printf("cleft:%d\n",left_encoder_count);
  printf("cright:%d\n",right_encoder_count);

  float dxy_ave = (dright + dleft) / 2.0;
  float dth = (dright - dleft) / WHEEL_SEPARATION;
  vx = dxy_ave / dt;
  printf("vx%f\n",vx);
  vth = dth / dt;

  if (dxy_ave != 0)
  {
    float dx = cos(dth) * dxy_ave;
    float dy = -sin(dth) * dxy_ave;
    px += (cos(theta) * dx - sin(theta) * dy);
    py += (sin(theta) * dx + cos(theta) * dy);
  }
    
  if (dth != 0)
    theta += dth; 
  //left_encoder_count = 0;
  //right_encoder_count = 0;
  last_left_encoder_count = left_encoder_count;
  last_right_encoder_count = right_encoder_count;
}


//--------------------------------------MAIN-----------------------------------------

int main (int argc, char *argv[])
{
  
  if (gpioInitialise()<0) return 1;
  //初始化编码器IO
//  gpioSetMode(25, PI_INPUT);
//  gpioWrite(25, PI_HIGH);
  Hasher ir_l(7, callback_l_ir);
  Hasher ir_c(16, callback_c_ir);
  Hasher ir_r(8, callback_r_ir);

  re_decoder left_encoder(LEFT_ENCODER_PINA, LEFT_ENCODER_PINB, left_encoder_callback);
  re_decoder right_encoder(RIGHT_ENCODER_PINA, RIGHT_ENCODER_PINB, right_encoder_callback);

  gpioSetMode(LEFT_MOTOR_PWM, PI_OUTPUT);
  gpioSetMode(RIGHT_MOTOR_PWM, PI_OUTPUT);

  gpioSetMode(FL_IRCLIFFSENSOR, PI_INPUT);
  gpioSetMode(FC_IRCLIFFSENSOR, PI_INPUT);
  gpioSetMode(FR_IRCLIFFSENSOR, PI_INPUT);

  //PWM频率2K，范围0-1000
  gpioSetPWMfrequency(LEFT_MOTOR_PWM, 2000);
  gpioSetPWMrange(LEFT_MOTOR_PWM, 1000);

  gpioSetPWMfrequency(RIGHT_MOTOR_PWM, 2000);
  gpioSetPWMrange(RIGHT_MOTOR_PWM, 1000);

  gpioWrite(LEFT_MOTOR_EN, PI_LOW);
  gpioWrite(RIGHT_MOTOR_EN, PI_LOW);

  //ROS
  ros::init(argc, argv, "bucketbot_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();  

  ros::Time current_time, last_time;
  tf::TransformBroadcaster odom_broadcaster;
  bool publish_tf;
  nh.param("publish_tf", publish_tf, true);

  nav_msgs::Odometry odom;
  sensor_msgs::JointState joint_state;
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.velocity.resize(2);
  joint_state.name[0] = "left_wheel_joint";
  joint_state.name[1] = "right_wheel_joint";

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  ros::Publisher CLIFF_pub = nh.advertise<bucketbot_driver::CliffEvent>("/cliffsensor", 10);
  ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmd_callback);

  //电机PID循环 1/0.005=200Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.005), control_loop);

  bucketbot_driver::Dockir dockir_code;
  ros::Publisher dockir_code_pub = nh.advertise<bucketbot_driver::Dockir>("dockir", 1000);

  float dt = 0.0;
  last_time = ros::Time::now();
  bucketbot_driver::CliffEvent cliffevent;
  
  gpioSetPullUpDown(FL_IRCLIFFSENSOR,PI_PUD_DOWN);
  gpioSetPullUpDown(FC_IRCLIFFSENSOR,PI_PUD_DOWN);
  gpioSetPullUpDown(FR_IRCLIFFSENSOR,PI_PUD_DOWN);

  while (ros::ok())
  {

    gpioWrite(FL_IRCLIFFSENSOR, PI_LOW);
    gpioWrite(FC_IRCLIFFSENSOR, PI_LOW);
    gpioWrite(FR_IRCLIFFSENSOR, PI_LOW);
  
    current_time = ros::Time::now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    
    dt = (current_time - last_time).toSec();
    //printf("time:%f\n",dt);
    calc_odom(dt);           

    if(publish_tf)
    {
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_footprint"; 

      odom_trans.transform.translation.x = px;
      odom_trans.transform.translation.y = py;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      odom_broadcaster.sendTransform(odom_trans);
    }

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint"; 
		
    odom.pose.pose.position.x = px;
    odom.pose.pose.position.y = py;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance =  boost::assign::list_of (1e-3)  (0) (0)  (0)  (0)  (0)
                                                   (0) (1e-3)  (0)  (0)  (0)  (0)
                                                   (0)   (0)  (1e6) (0)  (0)  (0)
                                                   (0)   (0)   (0) (1e6) (0)  (0)
                                                   (0)   (0)   (0)  (0) (1e6) (0)
                                                   (0)   (0)   (0)  (0)  (0)  (1);		
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = vth;
    odom.twist.covariance =  boost::assign::list_of (1e-3)  (0) (0)  (0)  (0)  (0)
                                                    (0) (1e-3)  (0)  (0)  (0)  (0)
                                                    (0)   (0)  (1e6) (0)  (0)  (0)
                                                    (0)   (0)   (0) (1e6) (0)  (0)
                                                    (0)   (0)   (0)  (0) (1e6) (0)
                                                    (0)   (0)   (0)  (0)  (0)  (1);
    odom_pub.publish(odom);	

    joint_state.header.stamp = current_time;
    joint_state.position[0] = left_encoder_count / TICKSPERRADIAN;      //m
    joint_state.position[1] = right_encoder_count / TICKSPERRADIAN;     //m
    joint_state.velocity[0] = left_speed;
    joint_state.velocity[1] = right_speed;
    joint_state_pub.publish(joint_state);
    last_time = current_time;
    
    //50Hz
    dockir_code.data.resize(3);
    dockir_code.header.stamp = current_time;
    dockir_code.data[0] = dockir_flag_l;
    dockir_code.data[1] = dockir_flag_c;
    dockir_code.data[2] = dockir_flag_r;

    //printf("here");
    dockir_code_pub.publish(dockir_code);
    
    gpioSleep(PI_TIME_RELATIVE, 0, 20000);
     
      if(gpioRead(FL_IRCLIFFSENSOR))
        {
 	   cliffevent.sensor = cliffevent.LEFT;
           cliffevent.state = cliffevent.CLIFF;
           cliffevent.bottom = 5;
           CLIFF_pub.publish(cliffevent);
        }
    
      if(gpioRead(FC_IRCLIFFSENSOR))
        {
 	   cliffevent.sensor = cliffevent.CENTER;
           cliffevent.state = cliffevent.CLIFF;
           cliffevent.bottom = 5;
           CLIFF_pub.publish(cliffevent);
        }

      if(gpioRead(FR_IRCLIFFSENSOR)) 
        {
 	   cliffevent.sensor = cliffevent.RIGHT;
           cliffevent.state = cliffevent.CLIFF;
           cliffevent.bottom = 5;
           CLIFF_pub.publish(cliffevent);
        }
   //   else if(0==gpioRead(FR_IRCLIFFSENSOR));
   //     {
 //	   cliffevent.sensor = cliffevent.RIGHT;
   //        cliffevent.state = cliffevent.FLOOR;
   //        cliffevent.bottom = 0;
   //        CLIFF_pub.publish(cliffevent);
   //     }  
  }
 
  left_encoder.re_cancel();
  right_encoder.re_cancel();
  gpioWrite(LEFT_MOTOR_EN, PI_HIGH);
  gpioWrite(RIGHT_MOTOR_EN, PI_HIGH);
  gpioTerminate();
}
