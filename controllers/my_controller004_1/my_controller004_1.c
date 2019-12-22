/*
 * File:          my_controller004_1.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <math.h>
#include <stdio.h>
#include <webots/position_sensor.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define rad_2_deg(X) ( X / pi * 180.0)
#define deg_2_rad(X) ( X / 180.0 * pi)
#define pi 3.1415926
#define MAX_SPEED 6.28

void rotate();
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 

    
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
WbDeviceTag position_sensor4
  = wb_robot_get_device("shoulder_pan_joint_sensor");
WbDeviceTag shoulder_pan_joint
  = wb_robot_get_device("shoulder_pan_joint");
WbDeviceTag position_sensor3
  = wb_robot_get_device("elbow_joint_sensor");
WbDeviceTag elbow_joint
  = wb_robot_get_device("elbow_joint");
WbDeviceTag position_sensor2
  = wb_robot_get_device("shoulder_lift_joint_sensor");
WbDeviceTag shoulder_lift_joint
  = wb_robot_get_device("shoulder_lift_joint");
WbDeviceTag position_sensor1
  = wb_robot_get_device("wrist_1_joint_sensor");
WbDeviceTag wrist_1_joint
  = wb_robot_get_device("wrist_1_joint");
WbDeviceTag position_sensor
  = wb_robot_get_device("wrist_2_joint_sensor");
WbDeviceTag wrist_2_joint
  = wb_robot_get_device("wrist_2_joint");
  
   
   wb_position_sensor_enable(position_sensor1, TIME_STEP);
   wb_motor_set_velocity(wrist_1_joint, 0.1 * MAX_SPEED);
   wb_position_sensor_enable(position_sensor, TIME_STEP);
   wb_motor_set_velocity(wrist_2_joint,  0.1 * MAX_SPEED);
   wb_position_sensor_enable(position_sensor2, TIME_STEP);
   wb_motor_set_velocity(shoulder_lift_joint,  0.05 * MAX_SPEED);
   wb_position_sensor_enable(position_sensor3, TIME_STEP);
   wb_motor_set_velocity(elbow_joint,  0.05 * MAX_SPEED);
   wb_position_sensor_enable(position_sensor4, TIME_STEP);
   wb_motor_set_velocity(shoulder_pan_joint,  0.1 * MAX_SPEED);
   
   
   int wrist2_state = 1;
   int wrist1_state = 1;
   int shoulder_state =1;
   int elbow_state = 1;
   int shoulder_pan_state = 1;
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
   
   if(wrist2_state == 1 &&
   wb_position_sensor_get_value(position_sensor) >=  2 * pi - 0.01)
     wrist2_state = 2;
   if(wrist2_state == 2 &&
   wb_position_sensor_get_value(position_sensor) <= 0.01)
     wrist2_state = 1;
     
   if(wrist2_state == 1)
   {
     wb_motor_set_position(wrist_2_joint, 2*pi);
   }
   else
   {
     wb_motor_set_position(wrist_2_joint, 0);
   }
   
   if(wrist1_state == 1 &&
   wb_position_sensor_get_value(position_sensor) <=  pi + 0.01)
     wrist1_state = 2;
   if(wrist1_state == 2 &&
   wb_position_sensor_get_value(position_sensor) >= 2 * pi - 0.01)
     wrist1_state = 1;
     
   if(wrist1_state == 1)
   {
     wb_motor_set_position(wrist_1_joint, -pi);
   }
   else
   {
     wb_motor_set_position(wrist_1_joint, 0);
   }
   
   if(shoulder_state == 1 &&
   wb_position_sensor_get_value(position_sensor2) <=  -(pi/2 - 0.01))
     shoulder_state = 2;
   if(shoulder_state == 2 &&
   wb_position_sensor_get_value(position_sensor2) >=  -(pi/4 + 0.01))
     shoulder_state = 1;
     
   if(shoulder_state == 1)
   {
     wb_motor_set_position(shoulder_lift_joint, -pi/2);
   }
   else
   {
     wb_motor_set_position(shoulder_lift_joint, -pi/4);
   }
   
   if(elbow_state == 1 &&
   wb_position_sensor_get_value(position_sensor3) >=  pi/2 - 0.01)
     elbow_state = 2;
   if(elbow_state == 2 &&
   wb_position_sensor_get_value(position_sensor3) <=  pi/4 + 0.01)
     elbow_state = 1;
     
   if(elbow_state == 1)
   {
     wb_motor_set_position(elbow_joint, pi/2);
   }
   else
   {
     wb_motor_set_position(elbow_joint, pi/4);
   }
   
   if(shoulder_pan_state == 1 &&
   wb_position_sensor_get_value(position_sensor4) >=  pi/2 - 0.01)
     shoulder_pan_state = 2;
   if(shoulder_pan_state == 2 &&
   wb_position_sensor_get_value(position_sensor4) <=  0 + 0.01)
     shoulder_pan_state = 1;
     
   if(shoulder_pan_state == 1)
   {
     wb_motor_set_position(shoulder_pan_joint, pi/2);
   }
   else
   {
     wb_motor_set_position(shoulder_pan_joint, 0);
   }
   
   
    /* Process sensor data here */

 
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}


