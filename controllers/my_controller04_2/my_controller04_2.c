/*
 * File:          my_controller04_2.c
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
#include <webots/inertial_unit.h>
#include <math.h>
#include <stdio.h>
#include <webots/led.h>
#include <webots/distance_sensor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 5
#define rad_2_deg(X) ( X / pi * 180.0)
#define deg_2_rad(X) ( X / 180.0 * pi)
#define pi 3.1415926
#define MAX_SPEED 100

double aimAngle = 0;
double p = 1.6;
double i = 0.3;
double d = 0;
double now_Angle;
double Eorror_Angle;
double fore_Eorror;
double Inter_Eorror;
double Diff_Eorror;
double output_v;
double turn_left;
double turn_right;
double speed_factor = 2;
double turn_factor = 0.05;

void PID_controller(WbDeviceTag Inertial);
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
WbDeviceTag inertial_unit
  = wb_robot_get_device("inertial_unit");
wb_inertial_unit_enable(inertial_unit,1);

WbDeviceTag ds[6];
char *name[] = {"ds1","ds2","ds3","ds4","ds5","ds6"};
for(int i=0;i<6;i++)
{
  ds[i] = wb_robot_get_device(name[i]);
}

for(int i=0;i<6;i++)
{
  wb_distance_sensor_enable(ds[i],TIME_STEP);
}

WbDeviceTag left_motor
  = wb_robot_get_device("left_motor");
  wb_motor_set_velocity(left_motor, MAX_SPEED);
wb_motor_set_position(left_motor, INFINITY);
wb_motor_set_velocity(left_motor,0);

 
WbDeviceTag right_motor
  = wb_robot_get_device("right_motor");
wb_motor_set_position(right_motor, INFINITY);
wb_motor_set_velocity(right_motor, 0);

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
     double right_distance = (wb_distance_sensor_get_value(ds[0])
     + wb_distance_sensor_get_value(ds[1]) 
     + wb_distance_sensor_get_value(ds[2]))/3;
     
     double left_distance = (wb_distance_sensor_get_value(ds[3])
     + wb_distance_sensor_get_value(ds[4]) 
     + wb_distance_sensor_get_value(ds[5]))/3;
     
     aimAngle =(right_distance + left_distance)/2.0/1000.0
     * (-speed_factor) ;
     
     
     printf("%f\n",right_distance + left_distance);
    // printf("%f\n",left_distance);
     if(right_distance>700 || left_distance>700)
     {
     if(right_distance > left_distance)
     {
       turn_left = -(right_distance - left_distance) * turn_factor;
       turn_right = (right_distance - left_distance) * turn_factor;
     }
     else if(right_distance < left_distance)
     {
       turn_left = (left_distance - right_distance) * turn_factor;
       turn_right = -(left_distance - right_distance) * turn_factor;
     }
     else
     {
       turn_left = 0;
       turn_right = 0;
     }
     }
     else 
     {
       aimAngle = -1;
       turn_left = 50;
       turn_right = -50;
     }
     
     
     
     
PID_controller(inertial_unit);
wb_motor_set_velocity(left_motor,-(output_v + turn_left));
wb_motor_set_velocity(right_motor, -(output_v + turn_right));
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

void PID_controller(WbDeviceTag Inertial)
{
  now_Angle = rad_2_deg(wb_inertial_unit_get_roll_pitch_yaw(Inertial)[1]);
  Eorror_Angle = now_Angle - aimAngle;
  Inter_Eorror += Eorror_Angle;
  Diff_Eorror = Eorror_Angle - fore_Eorror;
  fore_Eorror = Eorror_Angle;
  
  output_v = p * Eorror_Angle + i * Inter_Eorror
  + d * Diff_Eorror;
  if(output_v > MAX_SPEED)
    output_v = MAX_SPEED;
  if(output_v < - MAX_SPEED)
    output_v = - MAX_SPEED;
}
