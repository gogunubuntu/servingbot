/*******************************************************************************
  Copyright 2018 SEOULTECH CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

/* Authors: Donghyun Ko */

#ifndef MOBILEROBOT_CONFIG_H_
#define MOBILEROBOT_CONFIG_H_

// include msgs
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/Sound.h>
#include <turtlebot3_msgs/VersionInfo.h>

#include <TurtleBot3.h>

#include <math.h>

#define INIT_LOG_DATA "This core(v1.1.2) is compatible with TB3 Waffle or Waffle Pi"

#define HARDWARE_VER "1.0.0"
#define SOFTWARE_VER "1.0.0"
#define FIRMWARE_VER "1.1.2"

#include <avr/dtostrf.h>

//control frequency
#define CONTROL_MOTOR_SPEED_FREQUENCY 30 //Hz
#define IMU_PUBLISH_PERIOD 200 //Hz
#define CMD_VEL_PUBLISH_PERIOD 30
#define DRIVE_INFORMATION_PUBLISH_PERIOD 30 //Hz
#define VERSION_INFORMATION_PUBLISH_PERIOD 1 //hz 

//mobile robot's characteristic
#define WHEEL_NUM                        2
#define WHEEL_SEPARATION                 0.5810           // meter (BURGER : 0.160, WAFFLE : 0.287, mechatronics : 0.400)
#define WHEEL_RADIUS                     0.1015          //meter
//#define PI                               3.141592f

//direction
#define FORWARD 0
#define BACKWARD 1
#define STOP 2

#define LEFT 0
#define RIGHT 1

#define LINEAR 0
#define ANGULAR 1

#define MAX_LINEAR_VELOCITY              1.0   // m/s   (BURGER : 0.22, WAFFLE : 0.25)
#define MAX_ANGULAR_VELOCITY             1.0   // rad/s (BURGER : 2.84, WAFFLE : 1.82)

#define DEG2RAD 0.01745329252 // PI/180
#define RAD2DEG 57.2957795131 // 180/PI

// Callback function prototypes
void cmd_velCallback(const geometry_msgs::Twist & cmd_vel_msg);
void resetCallback(const std_msgs::Empty& reset_msg);

// Function prototypes
void publishDriveInformation(void);
void publishImuMsg(void);
void publishMagMsg(void);
//void publishSensorStateMsg(void);
void publishVersionInfoMsg(void);
//void publishBatteryStateMsg(void);
void publishDriveInformation(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros);

void updateVariable(void);
void updateTime(void);
void updateOdometry(void);
void updateJointStates(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGyroCali(void);

void initOdom(void);
void initJointStates(void);

bool calcOdometry(double diff_time);

void sendLogMsg(void);

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_velCallback );
ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
//turtlebot3_msgs::SensorState sensor_state_msg;
//ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// Version information of Turtlebot3
turtlebot3_msgs::VersionInfo version_info_msg;
ros::Publisher version_info_pub("version_info", &version_info_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Odometry of Turtlebot3
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint state of Turtlebot3
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field", &mag_msg);

/*******************************************************************************
* For debugging
*******************************************************************************/ 
std_msgs::Float32 angle_msg;
ros::Publisher pub_angle("angle", &angle_msg);
std_msgs::Float32 velocity_msg;
ros::Publisher pub_velocity("velocity", &velocity_msg);
std_msgs::Float32 pwm_msg;
ros::Publisher pub_pwm("pwm", &pwm_msg);

std_msgs::Float32 left_vel_check;
std_msgs::Float32 right_vel_check;


/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[5];

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
double  g_last_rad[WHEEL_NUM]       = {0.0, 0.0};
double g_last_diff_rad[WHEEL_NUM] = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
Turtlebot3Sensor sensors;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};

/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
Turtlebot3Diagnosis diagnosis;

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

// pwm
int pwm_pin[WHEEL_NUM] = {9, 11}; //motor_ pwm pin
int dir_pin[WHEEL_NUM] = {10, 12}; //motor direction pin
int dir[WHEEL_NUM] = {FORWARD, FORWARD}; //motor direction

//  hall sensor
int hall1[WHEEL_NUM] = {4, 2}; //hall1 pin
int hall2[WHEEL_NUM] = {7, 3}; //hall2 pin
int hall3[WHEEL_NUM] = {8, 75};//hall3 pin

int val_hall1[WHEEL_NUM] = {0, 0}; //hall1 data
int val_hall2[WHEEL_NUM] = {0, 0}; //hall2 data
int val_hall3[WHEEL_NUM] = {0, 0}; //hall3 data 

int val_hall1_prev[WHEEL_NUM] = {0, 0}; // post hall1 data
int val_hall2_prev[WHEEL_NUM] = {0, 0}; // post hall2 data
int val_hall3_prev[WHEEL_NUM] = {0, 0}; // post hall3 data

int hall_data_prev[WHEEL_NUM] = {0, 0}; // post hall data
int hall_data[WHEEL_NUM] = {0, 0}; // hall data

bool motor_start_flag[WHEEL_NUM] = {true, true}; 

float step_angle = 0.8823529412;//1.35294 ; //1.80392 = 0.90196 * 2

//anlge, velocity, radian/sec,rpm
double angle[WHEEL_NUM] = {0, 0};
double radian_per_sec[WHEEL_NUM] = {0, 0};
double ref_vel[WHEEL_NUM] = {0, 0};
double velocity[WHEEL_NUM] = {0, 0};
double rpm[WHEEL_NUM] = {0, 0};
unsigned long vel_time_prev[WHEEL_NUM]; //need to calculation velocity

//PID control
float Kp = 10; //10; //almost fix 20
float Ki = 200; //200
float Kd = 0.2; //02
float error[WHEEL_NUM];
float error_prev[WHEEL_NUM] = {0, 0};
float P_control[WHEEL_NUM], I_control[WHEEL_NUM], D_control[WHEEL_NUM];
float PID_control[WHEEL_NUM], PID_control_prev[WHEEL_NUM];
float duty_ratio[WHEEL_NUM];
bool cmd_vel_flag[WHEEL_NUM] = {true, true};

int offset[WHEEL_NUM] = {130,130};

//pre declaration of function
bool control_motor();
bool write_velocity();
void import_left_hall_data();
void import_right_hall_data();
void get_angle(int);
void get_velocity(int);
float pid_control(int);
void motor_pwm(int); 

/*******************************************************************************
* For safety
*******************************************************************************/ 
void resetGoalVelocity();

#endif //MOBILEROBOT_CONFIG_H_
