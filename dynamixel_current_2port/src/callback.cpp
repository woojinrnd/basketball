#include "callback.hpp"

// Callback::callback()
Callback::Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr)
    : trajectoryPtr(trajectoryPtr),
      IK_Ptr(IK_Ptr),
      dxlPtr(dxlPtr),
      SPIN_RATE(100)
{
    ros::NodeHandle nh(ros::this_node::getName());
    boost::thread queue_thread = boost::thread(boost::bind(&Callback::callbackThread, this));
    boost::thread imu_thread = boost::thread(boost::bind(&Callback::IMUThread, this));
    trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 675);
    trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 675);
    trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 675);
    trajectoryPtr->Turn_Trajectory = VectorXd::Zero(135);
}

void Callback::JointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_command)
{
    for (int i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        Goal_joint_[i] = joint_command->position[i];
        dxlPtr->SetThetaRef(Goal_joint_);
    }
}

void Callback::L_FSRsensorCallback(const std_msgs::UInt8::ConstPtr &FSR)
{
    R_value = FSR->data; // Left_foot_FSR
}

void Callback::R_FSRsensorCallback(const std_msgs::UInt8::ConstPtr &FSR)
{
    L_value = FSR->data; // Right_foot_FSR
}

/////////////////////////////////////////////// About Subscribe IMUThread ///////////////////////////////////////////////

void Callback::VelocityCallback(const std_msgs::Float32::ConstPtr &IMU)
{
    vel_x = IMU->data;
    vel_y = IMU->data;
    vel_z = IMU->data;

    // ROS_ERROR("X : %f", vel_x);
    // ROS_ERROR("Y : %f", vel_y);
    // ROS_ERROR("Z : %f", vel_z);
}

void Callback::IMUThread()
{
    IMU_Velocity_Complementary_x_subscriber_ = nh.subscribe("/filtered/Velocity_Complementary/x", 1000, &Callback::VelocityCallback, this);
    IMU_Velocity_Complementary_y_subscriber_ = nh.subscribe("/filtered/Velocity_Complementary/y", 1000, &Callback::VelocityCallback, this);
    IMU_Velocity_Complementary_z_subscriber_ = nh.subscribe("/filtered/Velocity_Complementary/z", 1000, &Callback::VelocityCallback, this);

    ros::Rate loop_rate(200);
    while (nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// Function to generate a unique request ID
int Callback::generateUniqueRequestID()
{
    // Initialize a seed using srand function
    // Initializing the seed allows you to generate random numbers using rand() function
    srand(static_cast<unsigned int>(std::time(0)));

    // Generate a random request ID (e.g., a random integer between 1 and 10000)
    return (rand() % 10000) + 1;
}

void Callback::callbackThread()
{
    ros::NodeHandle nh(ros::this_node::getName());

    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("KWJ_joint_states", 100);
    joint_state_subscriber_ = nh.subscribe("KWJ_desired_joint_states", 1000, &Callback::JointStatesCallback, this);

    FSR_L_sensor_subscriber_ = nh.subscribe("FSR_L", 1000, &Callback::L_FSRsensorCallback, this);
    FSR_R_sensor_subscriber_ = nh.subscribe("FSR_R", 1000, &Callback::R_FSRsensorCallback, this);

    srv_SendMotion.request.SM_finish = true;
    srv_SendMotion.request.TA_finish = true;
    srv_SendMotion.request.UD_finish = true;
    srv_SendMotion.request.RL_finish = true;
    srv_SendMotion.request.EM_finish = true;

    ros::Rate loop_rate(SPIN_RATE);
    while (nh.ok())
    {

        srv_SendMotion.request.request_id = generateUniqueRequestID(); // generate a unique request ID

        if (client_SendMotion.call(srv_SendMotion))
        {
            if (srv_SendMotion.response.success)
            {
                RecieveMotion();
                Motion_Info();
            }
            else if (!error_printed)
            {
                if (error_counter < 50) // Check the counter
                {
                    //ROS_INFO("\n");
                    //ROS_ERROR("Failed to call service");
                    //ROS_INFO("\n");
                    error_printed = true; // Set the flag to true
                    error_counter++;      // Increase the counter
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
        // usleep(1000);
    }
}

/////////////////////////////////////////////// About Client Callback ///////////////////////////////////////////////

void Callback::RecieveMotion()
{

    // //ROS_INFO("#[MESSAGE] SM Request : %s#", srv_SendMotion.request.SM_finish ? "true" : "false");
    // //ROS_INFO("#[MESSAGE] TA Request : %s#", srv_SendMotion.request.TA_finish ? "true" : "false");
    // //ROS_INFO("#[MESSAGE] UD Request : %s#", srv_SendMotion.request.UD_finish ? "true" : "false");
    // //ROS_INFO("#[MESSAGE] RL Request : %s#", srv_SendMotion.request.RL_finish ? "true" : "false");
    // //ROS_INFO("#[MESSAGE] EM Request : %s#", srv_SendMotion.request.EM_finish ? "true" : "false");

    // ROS_WARN("RL_NECK : %f", rl_neckangle);
    // ROS_WARN("UD_NECK : %f", -ud_neckangle+90);
    // ROS_WARN("TURN_ANGLE : %f", turn_angle * RAD2DEG);
    // //ROS_ERROR("EMERGENCY : %s", emergency_ ? "True" : "False");

    SelectMotion();
    TATA();
    Move_UD_NeckAngle();
    Move_RL_NeckAngle();
    Emergency();
}

void Callback::Move_RL_NeckAngle()
{

    double res_rl_neck = srv_SendMotion.response.rl_neckangle;
    rl_neckangle = res_rl_neck;
    All_Theta[21] = rl_neckangle;
    // All_Theta[2] = rl_neckangle * DEG2RAD;
    ROS_WARN("RL_NECK : %f", rl_neckangle);
    ROS_INFO("------------------------- RL NECK Angle ----------------------------");
}

// 90[deg] : Initial Propose (Look straight)
// CCW (+)
void Callback::Move_UD_NeckAngle()
{
    double res_ud_neck = srv_SendMotion.response.ud_neckangle;
    ud_neckangle = 90 - res_ud_neck;
    // All_Theta[22] = ud_neckangle * DEG2RAD;
    ROS_WARN("UD_NECK : %f", res_ud_neck);
    ROS_INFO("------------------------- UD NECK Angle ----------------------------");
}

void Callback::TATA()
{
    double res_turn_angle = srv_SendMotion.response.turn_angle;

    if (res_turn_angle != 0 )
    {
        turn_angle = res_turn_angle * DEG2RAD;
        trajectoryPtr->Make_turn_trajectory(turn_angle);
        index_angle = 0;
       ROS_WARN("TURN_ANGLE : %f", res_turn_angle);
    }
    
    ROS_INFO("------------------------- TURN_ANGLE ----------------------------");
}

/// 재민이형 긴급정지에 대한 코드 여기다가 넣으면 됨 ///
void Callback::Emergency()
{
    bool res_emergency = srv_SendMotion.response.emergency;
    emergency_ = res_emergency;
    if (emergency_ == false)
    {
        on_emergency = false;
        stop_indext = 0;
        // mode = 0;
    }
    else if (emergency_ == true)
    {
        on_emergency = true;
        stop_indext = 0;
        // mode = 0;
    }
    ROS_ERROR("EMERGENCY : %s", emergency_ ? "True" : "False");
    ROS_INFO("------------------------- EMERGENCY ----------------------------");
}

void Callback::Check_FSR()
{
    if (IK_Ptr->check_index == 0.05 && L_value == 0) // 오른발 들때 왼발 착지 확인
    {
        indext -= 1;
    }
    else if (IK_Ptr->check_index > 0.55 && R_value == 0) // 왼발 들때 오른발 착지 확인
    {
        indext -= 1;
    }
}

void Callback::Motion_Info()
{
    int8_t res_mode = srv_SendMotion.response.select_motion;
    float res_distance = srv_SendMotion.response.distance;
    string tmp_motion;
    switch (res_mode)
    {
    case Motion_Index::InitPose:
        tmp_motion = Str_InitPose;
        break;

    case Motion_Index::Forward_4step:
        tmp_motion = Str_Forward_4step;
        break;

    case Motion_Index::Left_2step:
        tmp_motion = Str_Left_2step;
        break;

    case Motion_Index::Step_in_place:
        tmp_motion = Str_Step_in_place;
        break;

    case Motion_Index::Right_2step:
        tmp_motion = Str_Right_2step;
        break;

    case Motion_Index::ForWard_fast4step:
        tmp_motion = Str_ForWard_fast4step;
        break;

    case Motion_Index::Forward_Nstep:
        tmp_motion = Str_Forward_Nstep;
        break;

    case Motion_Index::Huddle_Jump:
        tmp_motion = Str_Huddle_Jump;
        break;

    case Motion_Index::Forward_Halfstep:
        tmp_motion = Str_Forward_Halfstep;
        break;

    case Motion_Index::Left_Halfstep:
        tmp_motion = Str_Left_Halfstep;
        break;

    case Motion_Index::Right_Halfstep:
        tmp_motion = Str_Right_Halfstep;
        break;

    case Motion_Index::Back_Halfstep:
        tmp_motion = Str_Back_Halfstep;
        break;

    case Motion_Index::FWD_UP:
        tmp_motion = Str_FWD_UP;
        break;

    case Motion_Index::BWD_UP:
        tmp_motion = Str_BWD_UP;
        break;

    case Motion_Index::NONE:
        tmp_motion = Str_NONE;
        break;
    }

    if (res_mode == Motion_Index::Forward_Nstep)
    {
        ROS_WARN("Motion_Index : %s", tmp_motion.c_str());
        ROS_WARN("Distance : %f", res_distance);
        ROS_INFO("------------------------- Select Motion ----------------------------");
    }

    else if (res_mode == Motion_Index::InitPose)
    {
        ROS_ERROR("Motion_Index : %s", tmp_motion.c_str());
        ROS_INFO("------------------------- Select Motion ----------------------------");
    }

    else
    {
        ROS_WARN("Motion_Index : %s", tmp_motion.c_str());
        ROS_INFO("------------------------- Select Motion ----------------------------");
    }
}

void Callback::SelectMotion()
{
    int8_t res_mode = srv_SendMotion.response.select_motion;
    float res_distance = srv_SendMotion.response.distance;
    if (res_mode == 0)
    {
        mode = 0;
        trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 675);
        trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 675);
        trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 675);
        trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 675);
        trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 675);
        trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 675);
        indext = 0;
    }
    else if (res_mode == 1)
    {
        mode = 1;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Go_Straight(0.05,0.5,0.05);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3.5,3.5,3.5,3.5,3.5,3.5);
        IK_Ptr->Set_Angle_Compensation(135);
        trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        indext = 0;
    }
    else if (res_mode == 2)
    {
        mode = 2;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Side_Left2();
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3.5,3.5,3.5,3.5,3.5,3.5);
        IK_Ptr->Set_Angle_Compensation(135);
        indext = 0;
    }
    else if (res_mode == 3)
    {
        mode = 3;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Step_in_place(0.05, 0.5);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3.5,3.5,3.5,3.5,3.5,3.5);
        IK_Ptr->Set_Angle_Compensation(135);
        indext = 0;
    }
    else if (res_mode == 4)
    {
        mode = 4;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Side_Right2();
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3.5,3.5,3.5,3.5,3.5,3.5);
        IK_Ptr->Set_Angle_Compensation(135);
        indext = 0;
    }
    else if (res_mode == 5)
    {
        mode = 5;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Freq_Change_Straight(0.05,0.5,0.05,1);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3.5,3.5,3.5,3.5,3.5,3.5);
        IK_Ptr->Set_Angle_Compensation(67);
        indext = 0;
    }
    else if (res_mode == 6)
    {
        mode = 6;
        IK_Ptr->Change_Com_Height(80);
        trajectoryPtr->Huddle_Motion(0.2, 0.12);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3.5,2.5,4.5,3.5,2.5,4.5);
        IK_Ptr->Set_Angle_Compensation(135);
        indext = 0;
    }
    // //ROS_INFO("mode(%d)", mode);
    // ROS_WARN("Distance(%f)", res_distance);
}

void Callback::Write_Leg_Theta()
{

    check_indext = indext % walktime_n;
    if (turn_angle != 0 && check_indext == 0 && indext > 0.5*walktime_n && indext < trajectoryPtr->Ref_RL_x.cols()-walktime_n*0.5)
    {
        on_angle = true;
    }

    if (on_angle == true)
    {
        srv_SendMotion.request.TA_finish = false;
    }
    else if (on_angle == false)
    {
        srv_SendMotion.request.TA_finish = true;
    }


    if (on_emergency == true)
    {
        if (check_indext == 0 && mode == 1)
        {
            emergency = 1;
        }
        else if (check_indext == 67 && mode == 3)
        {
            emergency = 2;
        }
    }
    else if (on_emergency == false)
    {
        emergency = 0;
    }
    // Check_FSR();
    if (emergency == 0)
    {
        indext += 1;
        if (mode == 0)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
        }

        else if (mode == 1)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext);
            if (on_angle == true)
            {
                if (turn_angle > 0)
                {
                    IK_Ptr->RL_th[0] = trajectoryPtr->Turn_Trajectory(index_angle);
                    index_angle += 1;
                    if (index_angle > walktime_n - 1)
                    {
                        turn_angle = 0;
                        on_angle = false;
                        srv_SendMotion.request.TA_finish = true;
                    }
                }
                else if (turn_angle < 0)
                { 
                    IK_Ptr->LL_th[0] = trajectoryPtr->Turn_Trajectory(index_angle);
                    index_angle += 1;
                    if (index_angle > walktime_n - 1)
                    {
                        turn_angle = 0;
                        on_angle = false;
                        srv_SendMotion.request.TA_finish = true;
                    }
                }
 
            }
        }

        else if (mode == 3)
        {

            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext);
            if (on_angle == true)
            {

                if (turn_angle > 0)
                {
                    IK_Ptr->RL_th[0] = trajectoryPtr->Turn_Trajectory(index_angle);
                    index_angle += 1;
                    if (index_angle > walktime_n - 1)
                    {
                        turn_angle = 0;
                        on_angle = false;
                        srv_SendMotion.request.TA_finish = true;
                    }
                }
                else if (turn_angle < 0)
                { 
                    IK_Ptr->LL_th[0] = trajectoryPtr->Turn_Trajectory(index_angle);
                    index_angle += 1;
                    if (index_angle > walktime_n - 1)
                    {
                        turn_angle = 0;
                        on_angle = false;
                        srv_SendMotion.request.TA_finish = true;
                    }
                }
            }
        }
        else if (mode == 2)
        {

            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation_Leftwalk(indext);
        }
        else if (mode == 4)
        {

            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation_Leftwalk(indext);
        }
        else if (mode == 5)
        {

            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Fast_Angle_Compensation(indext);
        }
        else if (mode == 6)
        {

            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation_Huddle(indext);
        }
    }
    else if (emergency == 1)
    {
        stop_indext += 1;
        //ROS_INFO("%d", emergency);
        IK_Ptr->BRP_Simulation(trajectoryPtr->lsRef_RL_x, trajectoryPtr->lsRef_RL_y, trajectoryPtr->lsRef_RL_z, trajectoryPtr->lsRef_LL_x, trajectoryPtr->lsRef_LL_y, trajectoryPtr->lsRef_LL_z, stop_indext);
        if (stop_indext > 133)
        {
            stop_indext -= 1;
            on_emergency = false;
        }
    }
    else if (emergency == 2)
    {
        indext -= 1;
    }

    All_Theta[0] = -IK_Ptr->RL_th[0];
    All_Theta[1] = IK_Ptr->RL_th[1] - 1 * DEG2RAD;
    All_Theta[2] = IK_Ptr->RL_th[2] - 10.74 * DEG2RAD;
    All_Theta[3] = -IK_Ptr->RL_th[3] + 38.34 * DEG2RAD;
    All_Theta[4] = -IK_Ptr->RL_th[4] + 24.22 * DEG2RAD;
    All_Theta[5] = -IK_Ptr->RL_th[5];
    All_Theta[6] = IK_Ptr->LL_th[0] + 2 * DEG2RAD;
    All_Theta[7] = IK_Ptr->LL_th[1];
    All_Theta[8] = -IK_Ptr->LL_th[2] + 10.74 * DEG2RAD;
    All_Theta[9] = IK_Ptr->LL_th[3] - 36.34 * DEG2RAD;
    All_Theta[10] = IK_Ptr->LL_th[4] - 24.22 * DEG2RAD;
    All_Theta[11] = -IK_Ptr->LL_th[5];

    if (indext >= trajectoryPtr->Ref_RL_x.cols() && indext != 0)
    {
        indext -= 1;
        srv_SendMotion.request.SM_finish = true;
    }
    else
    {
        srv_SendMotion.request.SM_finish = false;
    }

    // ROS_INFO("on_angle(%d) indext(%d) index_angle(%d) mode(%d)", on_angle , indext , index_angle, mode);


    // //ROS_INFO("%d  %lf %d", indext, All_Theta[3], emergency);
    Check_FSR();
}

void Callback::Write_Arm_Theta()
{
    All_Theta[12] = 0; // 허리
    All_Theta[13] = -90 * DEG2RAD;
    All_Theta[14] = 90 * DEG2RAD;
    All_Theta[15] = -60 * DEG2RAD;
    All_Theta[16] = 60 * DEG2RAD;
    All_Theta[17] = -90 * DEG2RAD;
    All_Theta[18] = -90 * DEG2RAD;
    All_Theta[19] = 0 * DEG2RAD;
    All_Theta[20] = 0 * DEG2RAD;
}