#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>

#include <stdio.h>

#include <pigpio.h>

#include "../include/bucketbot_driver/ir_hasher.h"

/*

REQUIRES

An IR receiver output pin connected to a Pi gpio.

TO BUILD

gcc -o ir_hash_c test_ir_hasher.c  ir_hasher.c -lpigpio -lrt -lpthread

TO RUN

sudo ./ir_hash_c

*/

char dockir_flag_l=0, dockir_flag_r=0; 
char cb_flag_l = 0, cb_flag_r = 0;
uint32_t hash_l, hash_r;

void callback_l_ir(uint32_t hash_l)
{ 
   cb_flag_l = 1;
   printf("hash_l=%u\n", hash_l);
   if (4189793112==hash_l){
      dockir_flag_l = 1;
   }
   else
      dockir_flag_l = 0;
  
}

void callback_r_ir(uint32_t hash_r)
{
   cb_flag_r = 1;
   printf("hash_r=%u\n", hash_r);
   if (4189793112==hash_r){
      dockir_flag_r = 1;
   }
   else
      dockir_flag_r = 0;
   
}

int main(int argc, char *argv[])
{
   char filter_array_l[3]={0};
   char filter_array_r[3]={0};
   if (gpioInitialise() < 0) return 1;
   Hasher ir_l(7, callback_l_ir);
   Hasher ir_r(8, callback_r_ir);

   ros::init(argc, argv, "dockir_node");
   ros::NodeHandle nh;
   geometry_msgs::Vector3 ircode;
   ros::Publisher ircode_pub = nh.advertise<geometry_msgs::Vector3>("dockir", 1000);

   //ros::AsyncSpinner spinner(1);
   //spinner.start();


   //Pi_Hasher_t *hasher_l;
   //Pi_Hasher_t *hasher_r;

  

   /* 
      This assumes the output pin of an IR receiver is
      connected to gpio 7.
   */

   while(ros::ok()){
       
        ircode.x = dockir_flag_l;
        ircode.y = dockir_flag_r;
        ircode.z = 0;
	//printf("here");
        ircode_pub.publish(ircode);
        //ros::spinOnce();
        //sleep(0.2);
           
        gpioSleep(PI_TIME_RELATIVE, 0 , 20000);

   }

   gpioTerminate();
}

