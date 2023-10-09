#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h> 
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
// #include <unordered_map> //자료구조 중 더 빠른 map탐색 key:value


//Protocol version
#define PROTOCOL_VERSION         2.0

//Default setting
#define NUMBER_OF_DYNAMIXELS     23
#define BAUDRATE                 4000000 
#define DEVICE_NAME              "/dev/ttyUSB0"


#define PI                       3.141592
#define TORQUE_TO_VALUE_MX_64    267.094     //mx-64 e-manual plot(not considering about efficiency)
#define TORQUE_TO_VALUE_MX_106   183.7155         
#define RAD_TO_VALUE             651.89878   //1rev = 4096 --> 4096/(2*PI)
#define RAD2DEG                  57.2958
#define DEG2RAD                  0.0174533


using Eigen::VectorXd;

// Operating Mode
enum DynamixelOperatingMode
{
    Current_Control_Mode = 0,
    Velocity_Control_Mode = 1,
    Position_Control_Mode = 3,
    Extended_Position_Control_Mode = 4,
    Current_based_Position_Control_Mode = 5,
    PWM_Control_Mode = 16
};

// Control table address
enum DynamixelStandardRegisterTable
{
  // EEPROM
  DxlReg_ModelNumber = 0,
  DxlReg_ModelInfo = 2,
  DxlReg_FirmwareVersion = 6,
  DxlReg_ID = 7,
  DxlReg_BaudRate = 8,
  DxlReg_ReturnDelayTime = 9,
  DxlReg_DriveMode = 10,        // !
  DxlReg_OperatingMode = 11,
  DxlReg_ShadowID = 12,
  DxlReg_ProtocolVersion = 13,
  DxlReg_HomingOffset = 20,
  DxlReg_MovingThreshold = 24,
  DxlReg_TemperatureLimit = 31,
  DxlReg_MaxVoltageLimit = 32,
  DxlReg_MinVoltageLimit = 34,
  DxlReg_PWMLimit = 36,
  DxlReg_CurrentLimit = 38,
  DxlReg_AccelerationLimit = 40,
  DxlReg_VelocityLimit = 44,
  DxlReg_MaxPositionLimit = 48,
  DxlReg_MinPositionLimit = 52, 
  DxlReg_DataPort1Mode = 56,
  DxlReg_DataPort2Mode = 57,
  DxlReg_DataPort3Mode = 58,
  DxlReg_Shutdown = 63,

  // RAM
  DxlReg_TorqueEnable = 64,
  DxlReg_LED = 65,
  DxlReg_StatusReturnLevel = 68,
  DxlReg_RegisteredInstruction = 69,
  DxlReg_HardwareErrorStatus = 70,
  DxlReg_VelocityIGain = 76,
  DxlReg_VelocityPGain = 78,
  DxlReg_PositionDGain = 80,
  DxlReg_PositionIGain = 82,
  DxlReg_PositionPGain = 84,
  DxlReg_Feedforward2ndGain = 88,
  DxlReg_Feedforward1stGain = 90,
  DxlReg_BusWatchdog = 98,
  DxlReg_GoalPWM = 100,
  DxlReg_GoalCurrent = 102,
  DxlReg_GoalVelocity = 104,
  DxlReg_ProfileAcceleration = 108,
  DxlReg_ProfileVelocity = 112,
  DxlReg_GoalPosition = 116,
  DxlReg_RealtimeTick = 120,
  DxlReg_Moving = 122,
  DxlReg_MovingStatus = 123,
  DxlReg_PresentPWM = 124,
  DxlReg_PresentCurrent = 126,
  DxlReg_PresentVelocity = 128,
  DxlReg_PresentPosition = 132,
  DxlReg_VelocityTrajectory = 136,
  DxlReg_PositionTrajectory = 140,
  DxlReg_PresentInputVoltage = 144,
  DxlReg_PresentTemperature = 146,
  DxlReg_DataPort1 = 152,
  DxlReg_DataPort2 = 154,
  DxlReg_DataPort3 = 156,
  DxlReg_IndirectAddress1 = 168,
  DxlReg_IndirectData1 = 224
};


class Dxl
{
    //Member Variable
    private:
        dynamixel::PortHandler* portHandler;
        dynamixel::PacketHandler* packetHandler;
        // const uint8_t dxl_id[NUMBER_OF_DYNAMIXELS] = {12,18,2};
        const uint8_t dxl_id[NUMBER_OF_DYNAMIXELS] = {10, 8, 6, 4, 2, 0, 11, 9, 7, 5, 3, 1, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 24};
        float zero_manual_offset[NUMBER_OF_DYNAMIXELS] = { 0 };
        uint32_t position[NUMBER_OF_DYNAMIXELS] = { 0 };
        uint32_t velocity[NUMBER_OF_DYNAMIXELS] = { 0 };
        int32_t ref_torque_value[NUMBER_OF_DYNAMIXELS] = { 0 };
        int32_t torque2value[NUMBER_OF_DYNAMIXELS] = { 0 };
        uint32_t current[NUMBER_OF_DYNAMIXELS] = { 0 };

        VectorXd ref_th_value_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd ref_th_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd ref_th_dot_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd ref_torque_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        // VectorXd th_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd th_last_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd th_dot_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd th_dot_est_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);

        int16_t Mode = 1; // Current = 0, Position = 1

        // //Spread Joint command
        // sensor_msgs::JointState joint_state;
        // sensor_msgs::JointState write_msg_;   ///< Stores the last message received from the write command topic
        // bool write_ready_ = false;            ///< Booleans indicating if we have received commands
        // int recv_queue_size_ = 1;             ///< Receive queue size for desired_joint_states topic
        // bool stop_motors_on_shutdown_;        ///< Indicates if the motors should be turned off when the controller stops

        // Member Function

        // ************************************ GETTERS ***************************************** //

        // virtual void syncReadTheta();  // rad_pos = (count-count_initial_position) * (range/360) * (2*PI/encoder_cpr)
        virtual void syncReadThetaDot();
        virtual void getParam(int32_t data, uint8_t *param);

// **************************** SETTERS ******************************** //

        // virtual void syncWriteTheta();
        virtual void syncWriteTorque();

// **************************** Function ******************************** //

        // virtual void initActuatorValues();
        float convertValue2Radian(int32_t value);
        int32_t torqueToValue(double torque, uint8_t index);

    // Member Function
    public:
        Dxl(); //생성자
        ~Dxl(); //소멸자

// ************************************ GETTERS ***************************************** //

        virtual VectorXd GetThetaAct();
        virtual VectorXd GetThetaDot();
        virtual VectorXd GetThetaDotEstimated();
        // virtual VectorXd GetPIDGain();
        virtual int16_t GetPresentMode();
        virtual void syncReadTheta();  // rad_pos = (count-count_initial_position) * (range/360) * (2*PI/encoder_cpr)
        VectorXd th_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        virtual void SyncReadCurrent();
        VectorXd cur_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        virtual VectorXd GetCurrent();



// **************************** SETTERS ******************************** //

        virtual void SetTorqueRef(VectorXd);
        virtual void SetThetaRef(VectorXd);
        // virtual void SetPIDGain(VectorXd);
        // Current = 0, Position = 1
        virtual int16_t SetPresentMode(int16_t Mode); 
        virtual void syncWriteTheta();

        

// **************************** Function ******************************** //

        virtual void Loop(bool RxTh, bool RxThDot, bool TxTorque);
        virtual void CalculateEstimatedThetaDot(int);
        virtual void initActuatorValues();
        // virtual void FSR_flag();
        // virtual void Quaternino2RPY();
        virtual float convertValue2Current(int32_t value);

};


#endif