#ifndef MOVE_DECISION_H
#define MOVE_DECISION_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
#include <string.h>

#include "dynamixel_current_2port/SendMotion.h"

#include "img_proc.hpp"

#define UD_MAX 90
#define UD_MIN 0
#define UD_CENTER 10

#define RL_MAX 90
#define RL_MIN -90
#define RL_CENTER 0

#define TURN_MAX 10
#define TURN_MIN -10
#define NOLINE_ANGLE 15

#define ADJUST_TURN 5
#define ADJUST_Y_MARGIN 125


using namespace std;

class Move_Decision
{
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

    enum Running_Mode
    {
        FAR_HOOP_MODE = 0,
        ADJUST_MODE = 1,
        STOP_MODE = 2,
        WAKEUP_MODE = 3,
        SHOOT_MODE = 4,
        NO_HOOP_MODE = 5,
    };

    enum Stand_Status
    {
        Stand = 0,
        Fallen_Forward = 1,
        Fallen_Back = 2,
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
    string Str_Back_Halfstep = "Back_Halfstep";
    string Str_Left_6step = "Left_6step";
    string Str_Right_6step = "Right_6step";
    string Str_FWD_UP = "FWD_UP";
    string Str_BWD_UP = "BWD_UP";
    string Str_NONE = "NONE";

    string Str_FAR_HOOP_MODE = "FAR_HOOP_MODE";
    string Str_ADJUST_MODE = "ADJUST_MODE";
    string Str_SHOOT_MODE = "SHOOT_MODE";
    string Str_NO_HOOP_MODE = "NO_HOOP_MODE";
    string Str_STOP_MODE = "STOP_MODE";
    string Str_WAKEUP_MODE = "WAKEUP_MODE";

    Move_Decision(Img_proc *img_procPtr);
    Img_proc *img_procPtr;

    // Move_Decision();
    ~Move_Decision();

    // ********************************************** PROCESS THREAD************************************************** //

    void process();
    void processThread();
    void FAR_HOOP_mode();
    void ADJUST_mode();
    void STOP_mode();
    void WAKEUP_mode();
    void SHOOT_mode();
    void NO_HOOP_mode();

    bool tmp_img_proc_far_hoop_flg_ = false;
    bool tmp_img_proc_adjust_flg_ = false;
    bool tmp_img_proc_shoot_flg_ = false;
    bool tmp_img_proc_no_hoop_flg_ = false;

    // ********************************************** CALLBACK THREAD ************************************************** //

    void Running_Mode_Decision();
    void callbackThread();
    void startMode();

    bool SendMotion(dynamixel_current_2port::SendMotion::Request &req, dynamixel_current_2port::SendMotion::Response &res);

    std::tuple<int8_t, double> playMotion();
    double turn_angle();
    double Move_UD_NeckAngle();
    double Move_RL_NeckAngle();
    bool Emergency();

    // Publish & Subscribe
    // ros::Publisher Emergency_pub_;
    // IMU
    void IMUsensorCallback(const std_msgs::Float32::ConstPtr &IMU);
    ros::Subscriber IMU_sensor_x_subscriber_; ///< Gets IMU Sensor data from Sensor_node
    ros::Subscriber IMU_sensor_y_subscriber_; ///< Gets IMU Sensor data from Sensor_node
    ros::Subscriber IMU_sensor_z_subscriber_; ///< Gets IMU Sensor data from Sensor_node

    bool stop_fallen_check_;
    double present_pitch_;
    double present_roll_;
    Eigen::VectorXd RPY = Eigen::VectorXd::Zero(3); // Roll Pitch Yaw

    // Server && Client
    ros::ServiceServer SendMotion_server_;

    // ********************************************** FUNCTION ************************************************** //

    Eigen::Vector3d convertRotationToRPY(const Eigen::Matrix3d &rotation);
    Eigen::Vector3d convertQuaternionToRPY(const Eigen::Quaterniond &quaternion);
    void Motion_Info();
    void Running_Info();

    void Send_Motion_Info(int8_t res_motion);
    void Send_Info(int8_t motion_, double turn_angle_, double ud, double rl, bool emg);

    void DistanceDecision(double distance_);
    void Test_service();

    /////////////////////// Sequence++ ///////////////////////
    bool finish_past = false;
    int8_t req_finish_count = 0;

    // check the variable sharing with multi thread
    int aaaa = 0;
    int ccc = 0;
    int abb = 0;
    int abc = 1;
    int b = aaaa % 2;

    // ********************************************** GETTERS ************************************************** //

    bool Get_Emergency_() const;
    int8_t Get_motion_index_() const;
    int8_t Get_stand_status_() const;
    int8_t Get_running_mode_() const;
    double Get_turn_angle_() const;
    double Get_distance_() const;

    bool Get_ProcessON() const;
    bool Get_MoveDecisionON() const;
    bool Get_CallbackON() const;

    // RUNNING MODE
    bool Get_Far_Hoop_flg() const;
    bool Get_Adjust_flg() const;
    bool Get_Shoot_flg() const;
    bool Get_No_Hoop_flg() const;
    bool Get_Stop_det_flg() const;

    bool Get_select_motion_on_flg() const;
    bool Get_turn_angle_on_flg() const;
    bool Get_emergency_on_flg() const;
    bool Get_distance_on_flg() const;

    double Get_RL_NeckAngle() const;
    double Get_UD_NeckAngle() const;
    bool Get_RL_Neck_on_flg() const;
    bool Get_UD_Neck_on_flg() const;

    bool Get_SM_req_finish() const;
    bool Get_TA_req_finish() const;
    bool Get_UD_req_finish() const;
    bool Get_RL_req_finish() const;
    bool Get_EM_req_finish() const;

    // ********************************************** SETTERS ************************************************** //

    void Set_Emergency_(bool Emergency);
    void Set_motion_index_(int8_t motion_index);
    void Set_stand_status_(int8_t stand_status);
    void Set_running_mode_(int8_t running_mode);
    void Set_turn_angle_(double turn_angle);
    void Set_distance_(double distance);

    void Set_ProcessON(bool ProcessON);
    void Set_MoveDecisionON(bool MoveDecisionON);
    void Set_CallbackON(bool CallbackON);

    void Set_Far_Hoop_flg(bool far_hoop_flg);
    void Set_Adjust_flg(bool adjust_flg);
    void Set_Shoot_flg(bool shoot_flg);
    void Set_No_Hoop_flg(bool no_hoop_flg);

    void Set_select_motion_on_flg(bool select_motion_on_flg);
    void Set_turn_angle_on_flg(bool turn_angle_on_flg);
    void Set_emergency_on_flg(bool emergency_on_flg);
    void Set_distance_on_flg(bool distance_on_flg);

    void Set_RL_NeckAngle(double RL_NeckAngle);
    void Set_UD_NeckAngle(double UD_NeckAngle);
    void Set_RL_Neck_on_flg(bool RL_Neck_on_flg);
    void Set_UD_Neck_on_flg(bool UD_Neck_on_flg);

    void Set_SM_req_finish(bool SM_req_finish);
    void Set_TA_req_finish(bool TA_req_finish);
    void Set_UD_req_finish(bool UD_req_finish);
    void Set_RL_req_finish(bool RL_req_finish);
    void Set_EM_req_finish(bool EM_req_finish);

    // ********************************************** IMG_PROC ************************************************** //

    /////////////////////// Far Hoop Mode ///////////////////////
    bool robot_forward = false;
    double Green_area_dis = 1;
    int8_t hoop_distance = 0;
    int8_t far_hoop_motion = 0;

    /////////////////////// Adjust Mode ///////////////////////
    // 0 : Approach to the Adjust --> Motion : Motion_Index::Forward_Halfstep (Until adjust center)
    // 1 : Pose Control (Posture(Gradient))
    // 2 : Motion : SHOOT
    // 3 : Initializing
    int8_t tmp_adjust_seq = 0;
    int8_t img_proc_adjust_delta_x = 0;
    int8_t adjust_motion = 0;
    int img_proc_contain_adjust_to_foot = 0; // adjust Y Point
    bool contain_adjust_X = false;                // adjust X Point
    bool contain_adjust_Y = false;                // adjust Y Point
    bool adjust_posture = false;                  // adjust gradient
    bool adjust_seq_finish = false;
    double img_proc_adjust_angle = 0;
    double adjust_actual_angle = 0;

    string Str_ADJUST_SEQUENCE_0 = "ADJUST_SEQUENCE_0 : POSTURE CONTROL";
    string Str_ADJUST_SEQUENCE_1 = "ADJUST_SEQUENCE_1 : POSITION CONTROL";
    string Str_ADJUST_SEQUENCE_2 = "ADJUST_SEQUENCE_2 : POSTURE CONTROL ONE MORE TIME";
    string Str_ADJUST_SEQUENCE_3 = "ADJUST_SEQUENCE_3 : Ready_to_throw";
    string Str_ADJUST_SEQUENCE_4 = "ADJUST_SEQUENCE_4 : SHOOT";
    string Str_ADJUST_SEQUENCE_5 = "ADJUST_SEQUENCE_5 : INITIALIZING";

    /////////////////////// No Hoop Mode ///////////////////////
    int8_t tmp_delta_x = 0;
    int8_t nohoop_motion = 0;
    double nohoop_actual_angle = 0;

    /////////////////////// WAKEUP_MODE ///////////////////////
    // WakeUp_seq = 0 : Initial
    // WakeUp_seq = 1 : FWD_UP or BWD_UP
    // WakeUp_seq = 2 : Motion_Index : Initial_pose
    // WakeUp_seq = 3 : Line_mode()
    int8_t WakeUp_seq = 0;
    int8_t tmp_stand_status = 0;
    int8_t wakeup_motion = 0;
    int8_t wakeup_running = 0;

    int warning_counter = 0;
    bool warning_printed = false;

private:
    ros::NodeHandle nh;
    ros::Publisher pub;

    bool response_sent_ = false;
    std::set<int> processed_requests_;

    void recordProcessedRequest(int request_id)
    {
        processed_requests_.insert(request_id);
    }

    // Check if the request ID has already been processed
    bool isRequestProcessed(int request_id)
    {
        return processed_requests_.find(request_id) != processed_requests_.end();
    }

    const double FALL_FORWARD_LIMIT;
    const double FALL_BACK_LIMIT;
    const int SPIN_RATE;

    int8_t motion_index_;
    int8_t stand_status_;
    int8_t running_mode_;

    // Body Angle
    // Counter Clock Wise(+)
    // LEFT(+) / RIGHT(-)
    double turn_angle_ = 0;

    // Distance
    double distance_ = 0;

    // Neck
    // Counter Clock Wise(+)
    // LEFT(+) / RIGHT(-)
    double RL_NeckAngle_ = 0;
    bool RL_Neck_on_flg_ = false;
    // Counter Clock Wise(+)
    // UP(+) / DOWN(-)
    double UD_NeckAngle_ = 0;
    bool UD_Neck_on_flg_ = false;

    // Running mode
    bool far_hoop_flg_ = false;
    bool adjust_flg_ = false;
    bool shoot_flg_ = false;
    bool no_hoop_flg_ = false;
    bool stop_det_flg_ = false;

    bool select_motion_on_flg_ = false;
    bool turn_angle_on_flg_ = false;
    bool emergency_on_flg_ = false;
    bool distance_on_flg_ = false;

    bool huddle_det_stop_flg_ = false;
    bool corner_det_stop_flg_ = false;
    int8_t Wall_mode = 0;

    bool Emergency_;

    /// Thread switch ///
    bool ProcessON_;
    bool MoveDecisionON_;
    bool CallbackON_;

    // true -> req.finish is true
    // false -> req.finish is false
    // bool req_finish = true;
    bool SM_req_finish_ = false;
    bool TA_req_finish_ = false;
    bool UD_req_finish_ = false;
    bool RL_req_finish_ = false;
    bool EM_req_finish_ = false;

    // ********************************************** MUTEX ************************************************** //
    mutable std::mutex mtx_far_hoop_flg;
    mutable std::mutex mtx_adjust_flg;
    mutable std::mutex mtx_shoot_flg;
    mutable std::mutex mtx_no_hoop_flg;
    mutable std::mutex mtx_stop_det_flg;

    mutable std::mutex mtx_RL_NeckAngle_;
    mutable std::mutex mtx_UD_NeckAngle_;

    mutable std::mutex mtx_RL_Neck_on_flg;
    mutable std::mutex mtx_UD_Neck_on_flg;

    mutable std::mutex mtx_turn_angle_;
    mutable std::mutex mtx_distance_;

    mutable std::mutex mtx_motion_index_;
    mutable std::mutex mtx_stand_status_;
    mutable std::mutex mtx_running_mode_;

    mutable std::mutex mtx_select_motion_on_flg_;
    mutable std::mutex mtx_turn_angle_on_flg_;
    mutable std::mutex mtx_emergency_on_flg_;
    mutable std::mutex mtx_distance_on_flg_;

    mutable std::mutex mtx_Emergency_;
    mutable std::mutex mtx_ProcessON_;
    mutable std::mutex mtx_MoveDecisionON_;
    mutable std::mutex mtx_CallbackON_;

    mutable std::mutex mtx_SM_req_finish_;
    mutable std::mutex mtx_TA_req_finish_;
    mutable std::mutex mtx_UD_req_finish_;
    mutable std::mutex mtx_RL_req_finish_;
    mutable std::mutex mtx_EM_req_finish_;
};

#endif // MOVE_DECISION_H
