#ifndef CALLBACK_H
#define CALLBACK_H

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <boost/thread.hpp>

#include "dynamixel.hpp"
#include "Walkingpattern_generator.hpp"

#include "dynamixel_current_2port/SendMotion.h"

using Eigen::VectorXd;

class Callback
{
private:
  double turn_angle = 0;

public:
  enum Motion_Index
  {
    InitPose = 0,
    Forward_2step = 1,
    Left_2step = 2,
    Step_in_place = 3,
    Right_2step = 4,
    Forward_Nstep = 5,
    ForWard_fast4step = 7,
    FWD_UP = 8,
    BWD_UP = 9,
    Forward_Halfstep = 10,
    Left_Halfstep = 11,
    Right_Halfstep = 12,
    Back_Halfstep = 13,
    Forward_1step = 14,
    Left_6step = 15,
    Right_6step = 16,
    Shoot = 17,
    Ready_to_throw = 18,
    Right_1step = 19,
    Left_1step = 20,
    Grab = 30,
    START = 50,
    NONE = 99,
    FINISH = 100,
  };

    string Str_InitPose = "InitPose";
    string Str_Forward_2step = "Forward_2step";
    string Str_Forward_1step = "Forward_1step";
    string Str_Left_2step = "Left_2step";
    string Str_Step_in_place = "Step_in_place";
    string Str_Right_2step = "Right_2step";
    string Str_ForWard_fast4step = "ForWard_fast4step";
    string Str_Forward_Nstep = "Forward_Nstep";
    string Str_Shoot = "Shoot";
    string Str_Ready_to_throw = "Ready to throw";
    string Str_Forward_Halfstep = "Forward_Halfstep";
    string Str_Left_Halfstep = "Left_Halfstep";
    string Str_Right_Halfstep = "Right_Halfstep";
    string Str_Left_1step = "Left_1step";
    string Str_Right_1step = "Right_1step";
    string Str_Back_1step = "Back_Halfstep";
    string Str_Left_6step = "Left_6step";
    string Str_Right_6step = "Right_6step";
    string Str_FWD_UP = "FWD_UP";
    string Str_BWD_UP = "BWD_UP";
    string Str_NONE = "NONE";

  // Callback();
  Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr);

  Trajectory *trajectoryPtr;
  IK_Function *IK_Ptr;
  Dxl *dxlPtr;

  ros::NodeHandle nh;
  // Function

  // virtual void SelectMotion(const std_msgs::UInt8::ConstPtr &msg);

  virtual void Write_Leg_Theta();
  virtual void Write_Arm_Theta();
  virtual void Check_FSR();

  sensor_msgs::JointState joint_state;

  // ********************************************** IMU Thread ************************************************** //
  void IMUThread();
  ros::Subscriber IMU_Velocity_Complementary_x_subscriber_; ///< Gets IMU Sensor data from Move_Decision_node
  ros::Subscriber IMU_Velocity_Complementary_y_subscriber_; ///< Gets IMU Sensor data from Move_Decision_node
  ros::Subscriber IMU_Velocity_Complementary_z_subscriber_; ///< Gets IMU Sensor data from Move_Decision_node
  virtual void VelocityCallback(const std_msgs::Float32::ConstPtr &IMU);

  // ********************************************** Callback Thread ************************************************** //

  virtual void callbackThread();
  ros::Publisher joint_state_publisher_;    ///< Publishes joint states from reads
  ros::Subscriber joint_state_subscriber_;  ///< Gets joint states for writes
  ros::Subscriber FSR_L_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR_L
  ros::Subscriber FSR_R_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR_R
  ros::Subscriber Start_subscriber_;        ///< Start

  virtual void JointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_command);
  virtual void L_FSRsensorCallback(const std_msgs::UInt8::ConstPtr &FSR);
  virtual void R_FSRsensorCallback(const std_msgs::UInt8::ConstPtr &FSR);
  virtual void StartMode(const std_msgs::UInt8::ConstPtr &start);
  virtual void Set_Arm_Theta();
  /////////Service callbacek
  ros::ServiceClient client_SendMotion = nh.serviceClient<dynamixel_current_2port::SendMotion>("/Move_decision/SendMotion");
  dynamixel_current_2port::SendMotion srv_SendMotion;
  virtual int generateUniqueRequestID();

  virtual void SelectMotion();
  virtual void Move_UD_NeckAngle();
  virtual void Move_RL_NeckAngle();
  virtual void TATA();
  virtual void Emergency();
  virtual void Motion_Info();
  virtual void RecieveMotion();


  // Variable
  const int SPIN_RATE;
  VectorXd Goal_joint_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
  uint8_t L_value = 0;
  uint8_t R_value = 0;
  uint8_t fsr_value[2] = {L_value, R_value};
  double rl_neckangle = 0;
  double ud_neckangle = 0;
  double tmp_turn_angle = 0;
  bool emergency_ = 1; // True : Keep going , False : Emergency

  double vel_x = 0;
  double vel_y = 0;
  double vel_z = 0;

  // TEST
  bool a = false;
  int b = 1;

  // PRINT
  int error_counter = 0;
  bool error_printed = false;

  int8_t mode = 99; // NONE
  double walkfreq = 1.48114;
  double walktime = 2 / walkfreq;
  int freq = 100;
  int walktime_n = walktime * freq;
  int indext = 0;
  int check_indext = 0;
  int stop_indext = 0;
  bool turn_left = false;
  bool turn_right = false;
  int emergency = 99;
  bool on_emergency = false;
  double angle = 0;
  int index_angle = 0;

  MatrixXd RL_motion;
  MatrixXd LL_motion;
  MatrixXd RL_motion0;
  MatrixXd LL_motion0;
  MatrixXd RL_motion1;
  MatrixXd LL_motion1;
  MatrixXd RL_motion2;
  MatrixXd LL_motion2;
  MatrixXd RL_motion3;
  MatrixXd LL_motion3;
  MatrixXd RL_motion4;
  MatrixXd LL_motion4;
  MatrixXd RL_motion5;
  MatrixXd LL_motion5;
  MatrixXd RL_motion6;
  MatrixXd LL_motion6;
  MatrixXd RL_motion7;
  MatrixXd LL_motion7;
  VectorXd All_Theta = MatrixXd::Zero(NUMBER_OF_DYNAMIXELS, 1);

  // tf2::Quaternion quaternion;
};

#endif // CALLBACK_H