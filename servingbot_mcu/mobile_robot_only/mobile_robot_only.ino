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

/* Authors: jaewan choi */

#include "mobilerobot.h"

/*******************************************************************************
  Setup function
*******************************************************************************/
// 3 hall version
void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);
  nh.subscribe(reset_sub);
  
  //Serial.begin(9600);
  
  //nh.advertise(sensor_state_pub);
  nh.advertise(version_info_pub);
  nh.advertise(imu_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(mag_pub);

  nh.advertise(pub_angle);
  nh.advertise(pub_velocity);
  nh.advertise(pub_pwm);

  tf_broadcaster.init(nh);

  //setup motors, hallsensors
  motor_setup();
  hallsensor_setup();

  // first_posture_process();

  // Setting for IMU
  sensors.init();

  diagnosis.init();

  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();

  prev_update_time = millis();

  for (int side = 0; side < 2; side++)
  {
    vel_time_prev[side] = millis(); //because of velocity calculation
  }
}

void loop()
{
  uint32_t t = millis();
  updateTime();
  updateVariable();

  if ((t - tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    control_motor();
    tTime[0] = t;
  }
  if ((t - tTime[1]) >= (1000 / CMD_VEL_PUBLISH_PERIOD))
  {
    // subscribe sensors embedded in OpenCR and update odometry infomation.
    tTime[1] = t;
  }
  if ((t - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD))
  {
    //publishSensorStateMsg();
    //publishBatteryStateMsg();
    publishDriveInformation();
    tTime[2] = t;
  }
  if ((t - tTime[3]) >= (1000 / IMU_PUBLISH_PERIOD))
  {
    publishImuMsg();
    publishMagMsg();
    tTime[3] = t;
  }
  if ((t - tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_PERIOD))
  {
    publishVersionInfoMsg();
    tTime[4] = t;
  }

  // Send log message after ROS connection
  sendLogMsg();

  // Update the IMU unit
  sensors.updateIMU();

  // Start Gyro Calibration after ROS connection
  updateGyroCali();

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // give the serial link time to process
  delay(10);
}


/*******************************************************************************
  Callback function for cmd_vel msg
*******************************************************************************/
void cmd_velCallback(const geometry_msgs::Twist & cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(
                                      goal_velocity_from_cmd[LINEAR],
                                      (-1) * MAX_LINEAR_VELOCITY,
                                      MAX_LINEAR_VELOCITY
                                    );
  goal_velocity_from_cmd[ANGULAR] = constrain(
                                      goal_velocity_from_cmd[ANGULAR],
                                      (-1) * MAX_ANGULAR_VELOCITY,
                                      MAX_ANGULAR_VELOCITY
                                    );


  for (int side = 0; side < 2; side++)
  {
    if (side == LEFT)
    {
      goal_velocity[side] = goal_velocity_from_cmd[LINEAR] - (goal_velocity_from_cmd[ANGULAR] * WHEEL_SEPARATION / 2);
    }
    else if (side == RIGHT)
    {
      goal_velocity[side] = goal_velocity_from_cmd[LINEAR] + (goal_velocity_from_cmd[ANGULAR] * WHEEL_SEPARATION / 2);
    }

    if (goal_velocity[side] > 0)
    {
      dir[side] = FORWARD;
    }
    else if (goal_velocity[side] < 0)
    {
      dir[side] = BACKWARD;
    }

    if (goal_velocity[side] * velocity[side] < 0) //because of long rise time when sign is changed
    {
      cmd_vel_flag[side] = true;
    }
    
  }
}

/*******************************************************************************
  Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty & reset_msg)
{
  char log_msg[50];

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

  initOdom();

  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);
}

/*******************************************************************************
  Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = "imu_link";

  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
  Publish msgs (Magnetic data)
*******************************************************************************/
void publishMagMsg(void)
{
  mag_msg = sensors.getMag();

  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = "mag_link";

  mag_pub.publish(&mag_msg);
}

/*******************************************************************************
  Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = HARDWARE_VER;
  version_info_msg.software = SOFTWARE_VER;
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
}

/*******************************************************************************
  Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now  = millis();
  unsigned long step_time = time_now - prev_update_time; // dimension = [msec]

  prev_update_time = time_now;
  ros::Time stamp_now     = rosNow();

  // calculate odometry
  calcOdometry((double)(step_time * 0.001)); // dimension = [sec]

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(& odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(& joint_states);

}

/*******************************************************************************
  Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id       = "odom";
  odom.child_frame_id        = "base_link";

  odom.pose.pose.position.x  = odom_pose[0];
  odom.pose.pose.position.y  = odom_pose[1];
  odom.pose.pose.position.z  = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];

}

/*******************************************************************************
  Update the joint states
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] =
  {
    0.0,
    0.0
  };
  static float joint_states_vel[WHEEL_NUM] =
  {
    0.0,
    0.0
  };
  static float joint_states_eff[WHEEL_NUM] =
  {
    0.0,
    0.0
  };

  joint_states_pos[LEFT]                   = g_last_rad[LEFT];
  joint_states_pos[RIGHT]                  = g_last_rad[RIGHT];

  joint_states_vel[LEFT]                   = last_velocity[LEFT];
  joint_states_vel[RIGHT]                  = last_velocity[RIGHT];

  joint_states.position                    = joint_states_pos;
  joint_states.velocity                    = joint_states_vel;
  joint_states.effort                      = joint_states_eff;
}

/*******************************************************************************
  CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped & odom_tf)
{
  odom_tf.header                  = odom.header;
  odom_tf.child_frame_id          = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

/*******************************************************************************
  Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float * orientation;
  double wheel_l, wheel_r; // rotation value of wheel [rad]
  double delta_s,
         theta,
         delta_theta;
  static double last_theta = 0.0;
  double v, w; // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l   = wheel_r = 0.0;
  delta_s   = delta_theta = theta = 0.0;
  v         = w = 0.0;
  step_time = 0.0;

  step_time = diff_time; // dimension : [sec]

  if (step_time == 0)
    return false;

  wheel_l = (double)g_last_diff_rad[LEFT];
  wheel_r = (double)g_last_diff_rad[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;
  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s              = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;

  theta                = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;

  orientation          = sensors.getOrientation();
  theta                = atan2f(
                           orientation[1] * orientation[2] + orientation[0] * orientation[3],
                           0.5f - orientation[2] * orientation[2] - orientation[3] * orientation[3]
                         );
                         


  delta_theta          = theta - last_theta;

  v                    = delta_s / step_time;
  w                    = delta_theta / step_time;

  last_velocity[LEFT]  = velocity[LEFT];
  last_velocity[RIGHT] = velocity[RIGHT];

  // compute odometric pose
  odom_pose[0]         += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1]         += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2]         += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel[0]          = v;
  odom_vel[1]          = 0.0;
  odom_vel[2]          = w;

  last_theta           = theta;

  return true;
}

/*******************************************************************************
  Update variable (initialization)
*******************************************************************************/
void updateVariable(void)
{
  static bool variable_flag = false;

  if (nh.connected())
  {
    if (variable_flag == false)
    {
      sensors.initIMU();
      initOdom();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

/*******************************************************************************
  Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = micros();
  current_time = nh.now();
}
/*******************************************************************************
  ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return addMicros(current_time, micros() - current_offset);
}

/*******************************************************************************
  Time Interpolation function
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec  = _micros / 1000000 + t.sec;
  nsec = _micros % 1000000 + 1000 * (t.nsec / 1000);

  if (nsec >= 1e9)
  {
    sec++, nsec--;
  }
  return ros::Time(sec, nsec);
}

/*******************************************************************************
  Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(void)
{
  static bool isEnded = false;
  char log_msg[50];

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
  Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];
  const char* init_log_data = INIT_LOG_DATA;

  if (nh.connected())
  {
    if (log_flag == false)
    {
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to OpenCR board!");
      nh.loginfo(log_msg);

      sprintf(log_msg, init_log_data);
      nh.loginfo(log_msg);

      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      log_flag = true;

    }
  }
  else
  {
    log_flag = false;
  }
}
/*******************************************************************************
  Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}
/*******************************************************************************
  Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {"wheel_left_joint", "wheel_right_joint"};

  joint_states.header.frame_id = "base_link";
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}


void motor_setup()
{
  pinMode(dir_pin[LEFT], OUTPUT);
  drv_pwm_set_freq(pwm_pin[LEFT], 20000); //20kHz frequency

  pinMode(dir_pin[RIGHT], OUTPUT);
  drv_pwm_set_freq(pwm_pin[RIGHT], 20000);
}

void hallsensor_setup()
{
  pinMode(hall1[LEFT], INPUT_PULLDOWN);
  pinMode(hall2[LEFT], INPUT_PULLDOWN);
  pinMode(hall3[LEFT], INPUT_PULLDOWN);
  attachInterrupt(2, import_left_hall_data, CHANGE);
  attachInterrupt(3, import_left_hall_data, CHANGE);
  attachInterrupt(4, import_left_hall_data, CHANGE);
  
  pinMode(hall1[RIGHT], INPUT_PULLDOWN);
  pinMode(hall2[RIGHT], INPUT_PULLDOWN);
  pinMode(hall3[RIGHT], INPUT_PULLDOWN);
  attachInterrupt(0, import_right_hall_data, CHANGE);
  attachInterrupt(1, import_right_hall_data, CHANGE);
  attachInterrupt(8, import_right_hall_data, CHANGE);
}

void import_left_hall_data()
{
  if (motor_start_flag[LEFT])
  {
    val_hall1_prev[LEFT] = val_hall1[LEFT];
    val_hall2_prev[LEFT] = val_hall2[LEFT];
    val_hall3_prev[LEFT] = val_hall3[LEFT]; 
  }
  val_hall1[LEFT] = digitalRead(hall1[LEFT]);
  val_hall2[LEFT] = digitalRead(hall2[LEFT]);
  val_hall3[LEFT] = digitalRead(hall3[LEFT]);
  
  get_angle(LEFT);
}
void import_right_hall_data()
{
  if (motor_start_flag[RIGHT])
  {
    val_hall1_prev[RIGHT] = val_hall1[RIGHT];
    val_hall2_prev[RIGHT] = val_hall2[RIGHT];
    val_hall3_prev[RIGHT] = val_hall3[RIGHT];
  }
  val_hall1[RIGHT] = digitalRead(hall1[RIGHT]);
  val_hall2[RIGHT] = digitalRead(hall2[RIGHT]);
  val_hall3[RIGHT] = digitalRead(hall3[RIGHT]);
  
  get_angle(RIGHT);
}

void get_angle(int side)
{

  /*
    001
    011
    010
    110
    100
    101
  */

  if (motor_start_flag[side])
  {
    hall_data_prev[side] = (val_hall3_prev[side] << 2) | (val_hall2_prev[side] << 1) | (val_hall1_prev[side]);
  }

  hall_data[side] = (val_hall3[side] << 2) | (val_hall2[side] << 1) | (val_hall1[side]);

  if ((hall_data_prev[side] == B001 && hall_data[side] == B011) || 
      (hall_data_prev[side] == B011 && hall_data[side] == B010) || 
      (hall_data_prev[side] == B010 && hall_data[side] == B110) || 
      (hall_data_prev[side] == B110 && hall_data[side] == B100) ||
      (hall_data_prev[side] == B100 && hall_data[side] == B101) ||
      (hall_data_prev[side] == B101 && hall_data[side] == B001))
  {
    hall_data_prev[side] = hall_data[side];
    motor_start_flag[side] = false;
    angle[side] -= step_angle;
  }
  else if ((hall_data_prev[side] == B101 && hall_data[side] == B100) || 
           (hall_data_prev[side] == B100 && hall_data[side] == B110) || 
           (hall_data_prev[side] == B110 && hall_data[side] == B010) || 
           (hall_data_prev[side] == B010 && hall_data[side] == B011) ||
           (hall_data_prev[side] == B011 && hall_data[side] == B001) ||
           (hall_data_prev[side] == B001 && hall_data[side] == B101))
  {
    hall_data_prev[side] = hall_data[side];
    motor_start_flag[side] = false;
    angle[side] += step_angle;
  }
  else
  {
    motor_start_flag[side] = true;
  }
}
void get_velocity(int side)
{
  unsigned long vel_time_current = millis();
  unsigned long step_time = vel_time_current - vel_time_prev[side]; //demension [msec]
  vel_time_prev[side] = vel_time_current;

  g_last_diff_rad[side] = DEG2RAD * angle[side];
  g_last_rad[side] += g_last_diff_rad[side];

  radian_per_sec[side] = g_last_diff_rad[side] / (step_time * 0.001);
  velocity[side] = WHEEL_RADIUS * radian_per_sec[side];
  rpm[side] = 60 / (2 * PI) * radian_per_sec[side];

  velocity_msg.data = velocity[side];
  pub_velocity.publish(&velocity_msg);

  angle[side] = 0;
}

bool control_motor()
{
  write_velocity();

  return true;
}

bool write_velocity()
{
  for (int side = 0; side < 2; side++)
  {
    duty_ratio[side] = pid_control(side);
    motor_pwm(side);

    pwm_msg.data = duty_ratio[side];
    pub_pwm.publish(&pwm_msg);
  }

  return true;
}


float pid_control(int side)
{
  get_velocity(side);

  PID_control_prev[side] = PID_control[side];
  error[side] = goal_velocity[side] - velocity[side];

  P_control[side] = Kp * error[side];
  I_control[side] += Ki * error[side] / CONTROL_MOTOR_SPEED_FREQUENCY;
  D_control[side] = Kd * (error[side] - error_prev[side]) * CONTROL_MOTOR_SPEED_FREQUENCY;

  PID_control[side] = P_control[side] + I_control[side] + D_control[side];

  error_prev[side] = error[side];

  if (dir[side] == FORWARD)
  {
    PID_control[side] += offset[side];
  }
  else if (dir[side] == BACKWARD)
  {
    PID_control[side] -= offset[side];
  }
  /*
  if (abs(goal_velocity[side]) == 0)
  {
    P_control[side] = 0;
    I_control[side] = 0;
    D_control[side] = 0;
    PID_control[side] = 0;
    return 0;
  }

  if (abs(goal_velocity[side]) < 0.02)
  {
    P_control[side] = 0;
    I_control[side] = 0;
    D_control[side] = 0;
    PID_control[side] = 0;
    return 0;
  }
  */
  if (cmd_vel_flag[side] == true) //because problem of pid initialized
  {
    P_control[side] = 0;
    I_control[side] = 0;
    D_control[side] = 0;
    PID_control[side] = 0;
    if (goal_velocity[side]*velocity[side] >= 0)
    {
      cmd_vel_flag[side] = false;
    }
    return 0;
  }

  PID_control[side] = constrain(PID_control[side], -255, 255);

  return PID_control[side];
}

void motor_pwm(int side)
{
  //BLDC Motor need a 130 value of pwm at least
  if (duty_ratio[side] < 0)
  {
    digitalWrite(dir_pin[side], HIGH);
  }
  else if (duty_ratio[side] > 0)
  {
    digitalWrite(dir_pin[side], LOW);
  }
  analogWrite(pwm_pin[side], abs(duty_ratio[side]));

}

/*******************************************************************************
  For safety
*******************************************************************************/
void resetGoalVelocity()
{
  // Reset the goal velocities
  goal_velocity[LINEAR]  = 0.0f;
  goal_velocity[ANGULAR] = 0.0f;

}
