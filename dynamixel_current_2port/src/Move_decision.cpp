#include "Move_decision.hpp"

// Constructor
Move_Decision::Move_Decision(Img_proc *img_procPtr)
    : img_procPtr(img_procPtr),
      FALL_FORWARD_LIMIT(60),
      FALL_BACK_LIMIT(-60),
      SPIN_RATE(100),
      stand_status_(Stand_Status::Stand),
      motion_index_(Motion_Index::InitPose),
      stop_fallen_check_(false),
      Emergency_(false),
      turn_angle_(0)
{
    // Init ROS
    ros::NodeHandle nh(ros::this_node::getName());

    boost::thread process_thread = boost::thread(boost::bind(&Move_Decision::processThread, this));
    boost::thread queue_thread = boost::thread(boost::bind(&Move_Decision::callbackThread, this));
    boost::thread web_process_thread = boost::thread(boost::bind(&Img_proc::webcam_thread, img_procPtr));
    boost::thread depth_process_thread = boost::thread(boost::bind(&Img_proc::realsense_thread, img_procPtr));
}

Move_Decision::~Move_Decision()
{
}

// ********************************************** PROCESS THREAD************************************************** //

void Move_Decision::process()
{
    //////////////////////////////////////   DEBUG WINDOW    //////////////////////////////////////
    //// Switch line_det_flg | no_line_det_flg
    // if (aaaa % 2 == 0)
    // {
    //     Set_line_det_flg(true);
    //     Set_no_line_det_flg(false);
    // }
    // else
    // {
    //     Set_no_line_det_flg(true);
    //     Set_line_det_flg(false);
    // }
    // aaaa++;

    // FAR HOOP MODE
    // Set_Far_Hoop_flg(true);

    // NO HOOP MODE
    // Set_No_Hoop_flg(true);

    // AdJUST MODE
    // Set_Adjust_flg(true);

    //////////////////////////////////////   DEBUG WINDOW    //////////////////////////////////////

    tmp_img_proc_far_hoop_flg_ = img_procPtr->Get_img_proc_Far_Hoop_det();
    tmp_img_proc_adjust_flg_ = img_procPtr->Get_img_proc_Adjust_det();
    tmp_img_proc_no_hoop_flg_ = img_procPtr->Get_img_proc_No_Hoop_det();

    //////////////////////////////////////   FAR HOOP MODE    //////////////////////////////////////

    if (tmp_img_proc_far_hoop_flg_)
    {
        if (tmp_img_proc_adjust_flg_)
        {
            Set_Adjust_flg(true);
            Set_Far_Hoop_flg(false);
            Set_No_Hoop_flg(false);
        }
        else
        {
            Set_Far_Hoop_flg(true);
            Set_Adjust_flg(false);
            Set_No_Hoop_flg(false);
        }
    }

    //////////////////////////////////////   NO HOOP MODE    //////////////////////////////////////

    else if (tmp_img_proc_no_hoop_flg_)
    {
        if (tmp_img_proc_adjust_flg_)
        {
            Set_Adjust_flg(false);
            Set_Far_Hoop_flg(false);
            Set_No_Hoop_flg(true);
        }

        else
        {
            Set_No_Hoop_flg(true);
            Set_Far_Hoop_flg(false);
            Set_Adjust_flg(false);
        }
    }

    //////////////////////////////////////   ADJUST MODE    //////////////////////////////////////

    else if (tmp_img_proc_adjust_flg_)
    {
        if (tmp_img_proc_far_hoop_flg_)
        {
            Set_Adjust_flg(true);
            Set_Far_Hoop_flg(false);
            Set_No_Hoop_flg(false);
        }

        else if (tmp_img_proc_no_hoop_flg_)
        {
            Set_Adjust_flg(false);
            Set_Far_Hoop_flg(false);
            Set_No_Hoop_flg(true);
        }

        else
        {
            Set_Adjust_flg(true);
            Set_Far_Hoop_flg(false);
            Set_No_Hoop_flg(false);
        }
    }

    if (tmp_adjust_seq == 4 || tmp_adjust_seq == 5)
    {
        Set_Adjust_flg(true);
        Set_Far_Hoop_flg(false);
        Set_No_Hoop_flg(false);
    }
}

void Move_Decision::processThread()
{
    // set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    // node loop
    while (ros::ok())
    {
        // ROS_INFO("\n");
        // ROS_INFO("-------------------------PROCESSTHREAD----------------------------");
        process();
        // Running_Info();
        // Motion_Info();
        // ROS_INFO("Gradient : %f", img_procPtr->Get_gradient());
        // ROS_INFO("delta_x : %f", img_procPtr->Get_delta_x());
        // ROS_INFO("distance : %f", img_procPtr->Get_distance());
        // ROS_INFO("Move_decision img_proc_line_det : %d", img_procPtr->Get_img_proc_line_det());
        // ROS_INFO("-------------------------PROCESSTHREAD----------------------------");
        // ROS_INFO("\n");
        // relax to fit output rate
        loop_rate.sleep();
    }
}

// ********************************************** CALLBACK THREAD ************************************************** //

void Move_Decision::Running_Mode_Decision()
{
    if (running_mode_ == WAKEUP_MODE || stand_status_ == Fallen_Forward || stand_status_ == Fallen_Back)
    {
        running_mode_ = WAKEUP_MODE;
    }

    else if (running_mode_ != WAKEUP_MODE)
    {
        if (Get_Far_Hoop_flg()) // goal_line_detect_flg is true: goal_line detection mode
        {
            running_mode_ = FAR_HOOP_MODE;
        }
        else if (Get_Adjust_flg())
        {
            running_mode_ = ADJUST_MODE;
        }
        else if (Get_No_Hoop_flg())
        {
            running_mode_ = NO_HOOP_MODE;
        }
    }

    switch (running_mode_)
    {
    case FAR_HOOP_MODE:
        FAR_HOOP_mode();
        break;
    case ADJUST_MODE:
        ADJUST_mode();
        break;
    case STOP_MODE:
        STOP_mode();
        break;
    case WAKEUP_MODE:
        WAKEUP_mode();
        break;
    case NO_HOOP_MODE:
        NO_HOOP_mode();
        break;
    }
}

void Move_Decision::FAR_HOOP_mode()
{
    hoop_distance = img_procPtr->Get_distance();
    DistanceDecision(hoop_distance);
    far_hoop_motion = Get_motion_index_();

    if (!Get_select_motion_on_flg())
    {
        far_hoop_motion = Motion_Index::Forward_1step;
        Set_select_motion_on_flg(true);
        Set_motion_index_(far_hoop_motion);
    }
}

void Move_Decision::NO_HOOP_mode()
{
    // Counter Clock wise(+) (Turn Angle sign)
    // delta_x > 0 : LEFT Window  ->  Left turn (-)
    // delta_x < 0 : RIGHT window ->  Right turn  (+)

    tmp_delta_x = img_procPtr->Get_delta_x();
    nohoop_actual_angle = Get_turn_angle_();
    nohoop_motion = Get_motion_index_();

    if (tmp_delta_x < 0) // Right
    {
        if (!Get_turn_angle_on_flg())
        {
            nohoop_actual_angle -= 3;
            if (nohoop_actual_angle < -NOLINE_ANGLE)
            {
                nohoop_actual_angle = -NOLINE_ANGLE;
            }
            Set_turn_angle_(nohoop_actual_angle);
            Set_turn_angle_on_flg(true);
        }

        if (!Get_select_motion_on_flg())
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
        }

        ROS_WARN("RIGHT TURN");
        // ROS_INFO("turn angle : %d", Get_turn_angle_());
    }

    else if (tmp_delta_x > 0) // LEFT
    {
        if (!Get_turn_angle_on_flg())
        {
            nohoop_actual_angle += 3;
            if (nohoop_actual_angle > NOLINE_ANGLE)
            {
                nohoop_actual_angle = NOLINE_ANGLE;
            }
            Set_turn_angle_(nohoop_actual_angle);
            Set_turn_angle_on_flg(true);
        }

        if (!Get_select_motion_on_flg())
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
        }

        ROS_WARN("LEFT_TURN");
    }
}

void Move_Decision::ADJUST_mode()
{
    // 0 : Pose Control (Posture(Gradient))
    // --> Motion : Motion_Index::Step_In_Place && Turn Angle(Gradient)

    // 1 : Approach to the Line
    // --> Motion : Motion_Index::Left_Halfstep or Motion_Index::Right_Halfstep : About X diff
    // --> Motion : Motion_Index::Forward_Halfstep (Until Line center) : About Y diff

    // 2 : Motion : Ready_to_throw
    // --> Motion : Motion_Index::Step_In_Place && Turn Angle(Gradient)

    // 3 : Pose Control (Posture(Gradient))
    // --> Motion : Motion_Index::Step_In_Place && Turn Angle(Gradient)

    // 4 : Motion : Shoot

    // 5 : Initializing

    adjust_actual_angle = Get_turn_angle_();
    // adjust_motion = Motion_Index::InitPose;
    img_proc_adjust_angle = img_procPtr->Get_adjust_angle();
    ROS_ERROR("img_proc_adjust_angle : %lf", img_proc_adjust_angle);
    ROS_INFO("tmp_adjust_seq: %d", tmp_adjust_seq);


    // // 0 : Approach to the adjust + Pose Control (Posture(Gradient))
    // if (tmp_adjust_seq == 0)
    // {
    //     img_proc_adjust_angle = img_procPtr->Get_adjust_angle();
    //     ROS_ERROR("img_proc_adjust_angle : %lf", img_proc_adjust_angle);
    //     ROS_ERROR(Str_ADJUST_SEQUENCE_0.c_str());

    //     if (!Get_select_motion_on_flg())
    //     {
    //         adjust_motion = Motion_Index::Step_in_place;
    //         Set_motion_index_(adjust_motion);
    //         Set_select_motion_on_flg(true);
    //     }

    //     if (!Get_turn_angle_on_flg())
    //     {
    //         if (img_proc_adjust_angle > 10 || img_proc_adjust_angle < -10)
    //         {
    //             if (img_proc_adjust_angle >= 15)
    //             {
    //                 adjust_actual_angle = ADJUST_TURN;
    //             }
    //             else if (img_proc_adjust_angle <= -15)
    //             {
    //                 adjust_actual_angle = -ADJUST_TURN;
    //             }
    //             Set_turn_angle_(adjust_actual_angle);
    //             Set_turn_angle_on_flg(true);
    //         }
    //         else
    //         {
    //             adjust_posture = true;
    //         }
    //     }

    //     if (adjust_posture == true)
    //     {
    //         tmp_adjust_seq++;
    //         adjust_posture = false;
    //     }
    // }

    // 1 : Approach to the adjust + Pose Control (Position)
    if (tmp_adjust_seq == 0)
    {
        // Initializing
        img_proc_adjust_delta_x = img_procPtr->Get_delta_x();
        img_proc_contain_adjust_to_foot = img_procPtr->Get_contain_adjust_to_foot();

        ROS_ERROR(Str_ADJUST_SEQUENCE_1.c_str());
        ROS_ERROR("X diff :%d", img_proc_adjust_delta_x);
        ROS_WARN("Y diff : %d", img_proc_contain_adjust_to_foot);

        if (!Get_select_motion_on_flg())
        {
            // // adjust center X point
            // if (img_proc_adjust_delta_x < 0)
            // {
            //     adjust_motion = Motion_Index::Right_Halfstep;
            //     Set_motion_index_(adjust_motion);
            //     Set_select_motion_on_flg(true);
            //     contain_adjust_X = false;
            // }

            // else if (img_proc_adjust_delta_x > 0)
            // {
            //     adjust_motion = Motion_Index::Left_Halfstep;
            //     Set_motion_index_(adjust_motion);
            //     Set_select_motion_on_flg(true);
            //     contain_adjust_X = false;
            // }

            // // About adjust X point
            // else if (img_proc_adjust_delta_x == 0)
            // {
            //     ROS_WARN("X diff : %d", img_proc_adjust_delta_x);
            //     adjust_motion = Motion_Index::InitPose;
            //     Set_motion_index_(adjust_motion);
            //     Set_select_motion_on_flg(true);
            //     contain_adjust_X = true;
            //     ROS_WARN("X POSITION IS OK!!!!!!!!!!!!!!!!!!!");
            // }

            // adjust center X point
            if (img_proc_adjust_delta_x < 0)
            {
                adjust_motion =  Motion_Index::Step_in_place;
                Set_motion_index_(adjust_motion);
                Set_select_motion_on_flg(true);
                if (!Get_turn_angle_on_flg())
                {
                    Set_turn_angle_(-5);
                    Set_turn_angle_on_flg(true);
                }
                contain_adjust_X = false;
            }

            else if (img_proc_adjust_delta_x > 0)
            {
                adjust_motion = Motion_Index::Step_in_place;
                Set_motion_index_(adjust_motion);
                Set_select_motion_on_flg(true);
                if (!Get_turn_angle_on_flg())
                {
                    Set_turn_angle_(5);
                    Set_turn_angle_on_flg(true);
                }
                contain_adjust_X = false;
            }

            // About adjust X point
            else if (img_proc_adjust_delta_x == 0)
            {
                ROS_WARN("X diff : %d", img_proc_adjust_delta_x);
                adjust_motion = Motion_Index::InitPose;
                Set_motion_index_(adjust_motion);
                Set_select_motion_on_flg(true);
                contain_adjust_X = true;
                ROS_WARN("X POSITION IS OK!!!!!!!!!!!!!!!!!!!");
            }

            // About adjust Y point
            if (contain_adjust_X && img_proc_contain_adjust_to_foot > ADJUST_Y_MARGIN)
            {
                adjust_motion = Motion_Index::Forward_Halfstep;
                Set_motion_index_(adjust_motion);
                Set_select_motion_on_flg(true);
            }

            else if (img_proc_contain_adjust_to_foot < ADJUST_Y_MARGIN)
            {
                contain_adjust_Y = true;
                ROS_WARN("Y POSITION IS OK!!!!!!!!!!!!!!!!!!!");
            }

            if (contain_adjust_Y && contain_adjust_X)
            {
                adjust_motion = Motion_Index::InitPose;
                Set_motion_index_(adjust_motion);
                Set_select_motion_on_flg(true);

                // Sequence++
                if (finish_past != Get_SM_req_finish())
                {
                    contain_adjust_X = false;
                    contain_adjust_Y = false;
                    tmp_adjust_seq++;
                }
            }
        }
    }
    

    // // 2 : Approach to the adjust + Pose Control (Posture(Gradient))
    // else if (tmp_adjust_seq == 2)
    // {
    //     img_proc_adjust_angle = img_procPtr->Get_adjust_angle();
    //     ROS_ERROR("img_proc_adjust_angle : %lf", img_proc_adjust_angle);
    //     ROS_ERROR(Str_ADJUST_SEQUENCE_2.c_str());

    //     if (!Get_select_motion_on_flg())
    //     {
    //         adjust_motion = Motion_Index::Step_in_place;
    //         Set_motion_index_(adjust_motion);
    //         Set_select_motion_on_flg(true);
    //     }

    //     if (!Get_turn_angle_on_flg())
    //     {
    //         if (img_proc_adjust_angle > 10 || img_proc_adjust_angle < -10)
    //         {
    //             if (img_proc_adjust_angle >= 15)
    //             {
    //                 adjust_actual_angle = ADJUST_TURN;
    //             }
    //             else if (img_proc_adjust_angle <= -15)
    //             {
    //                 adjust_actual_angle = -ADJUST_TURN;
    //             }
    //             Set_turn_angle_(adjust_actual_angle);
    //             Set_turn_angle_on_flg(true);
    //         }
    //         else
    //         {
    //             adjust_posture = true;
    //         }
    //     }

    //     if (adjust_posture == true)
    //     {
    //         if (finish_past != Get_SM_req_finish())
    //         {
    //             tmp_adjust_seq++;
    //             adjust_posture = false;
    //         }
    //     }
    // }
    // 4 : Motion : Ready_to_throw
    else if (tmp_adjust_seq == 1)
    {
        ROS_ERROR(Str_ADJUST_SEQUENCE_3.c_str());
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            adjust_motion = Motion_Index::Ready_to_throw;
            Set_motion_index_(adjust_motion);
            Set_select_motion_on_flg(true);
            tmp_adjust_seq++;
        }
    }

    // 5 : Motion : Shoot
    else if (tmp_adjust_seq == 2)
    {
        ROS_ERROR(Str_ADJUST_SEQUENCE_4.c_str());
        if (!Get_select_motion_on_flg() /* && Get_SM_req_finish() */)
        {
            adjust_motion = Motion_Index::Shoot;
            Set_motion_index_(adjust_motion);
            Set_select_motion_on_flg(true);
            // tmp_adjust_seq =0;
        }
    }

    // // 5 : Initializing
    // else if (tmp_adjust_seq == 3)
    // {
    //     ROS_ERROR(Str_ADJUST_SEQUENCE_5.c_str());
    //     if (!Get_select_motion_on_flg())
    //     {
    //         adjust_motion = Motion_Index::InitPose;
    //         Set_motion_index_(adjust_motion);
    //         Set_select_motion_on_flg(true);
    //     }
    //     // Sequence++
    //     if (finish_past != Get_SM_req_finish())
    //     {
    //         tmp_adjust_seq = 0;
    //     }
    // }
}

void Move_Decision::STOP_mode()
{
    Set_Emergency_(true);
}

void Move_Decision::WAKEUP_mode()
{
    // WAKEUP_MODE
    // WakeUp_seq = 0 : Initial
    // WakeUp_seq = 1 : FWD_UP or BWD_UP
    // WakeUp_seq = 2 : Motion_Index : Initial_pose
    // WakeUp_seq = 3 : Line_mode()

    wakeup_motion = Get_motion_index_();
    wakeup_running = 0;

    tmp_stand_status = Get_stand_status_();

    if (WakeUp_seq == 0)
    {
        WakeUp_seq++;
    }

    else if (WakeUp_seq == 1)
    {
        if (tmp_stand_status == Stand_Status::Fallen_Back)
        {
            if (!Get_select_motion_on_flg() && Get_SM_req_finish())
            {
                wakeup_motion = Motion_Index::FWD_UP;
                Set_motion_index_(wakeup_motion);
                Set_select_motion_on_flg(true);
                WakeUp_seq++;
            }
        }

        else if (tmp_stand_status == Stand_Status::Fallen_Forward)
        {
            if (!Get_select_motion_on_flg() && Get_SM_req_finish())
            {
                wakeup_motion = Motion_Index::BWD_UP;
                Set_motion_index_(wakeup_motion);
                Set_select_motion_on_flg(true);
                WakeUp_seq++;
            }
        }
    }

    else if (WakeUp_seq == 2)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            wakeup_motion = InitPose;
            Set_motion_index_(wakeup_motion);
            Set_select_motion_on_flg(true);
            WakeUp_seq++;
        }
    }

    else if (WakeUp_seq == 3)
    {
        if (!Get_select_motion_on_flg() && Get_SM_req_finish())
        {
            wakeup_motion = Motion_Index::InitPose;
            Set_motion_index_(wakeup_motion);
            Set_select_motion_on_flg(true);
            WakeUp_seq++;
        }
    }

    else if (WakeUp_seq == 4)
    {
        wakeup_running = Get_running_mode_();
        Set_running_mode_(wakeup_running);
        WakeUp_seq = 0;
    }
    ROS_ERROR("WAKE_UP SEQ : %d", WakeUp_seq);
    // Motion_Info();
}

void Move_Decision::callbackThread()
{
    ros::NodeHandle nh(ros::this_node::getName());

    // Subscriber
    // IMU_sensor_x_subscriber_ = nh.subscribe("/Angle/x", 1000, &Move_Decision::IMUsensorCallback, this);
    IMU_sensor_y_subscriber_ = nh.subscribe("/Angle/y", 1000, &Move_Decision::IMUsensorCallback, this);
    // IMU_sensor_z_subscriber_ = nh.subscribe("/Angle/z", 1000, &Move_Decision::IMUsensorCallback, this);

    // Server
    SendMotion_server_ = nh.advertiseService("SendMotion", &Move_Decision::SendMotion, this);

    ros::Rate loop_rate(SPIN_RATE);
    startMode();
    while (nh.ok())
    {
        // ROS_INFO("-------------------------CALLBACKTHREAD----------------------------");
        // ROS_INFO("-------------------------------------------------------------------");
        Running_Mode_Decision();
        Running_Info();
        Motion_Info();
        // ROS_INFO("angle : %f", Get_turn_angle_());
        // ROS_INFO("RL_Neck : %f", Get_RL_NeckAngle());
        // ROS_INFO("UD_Neck : %f", Get_UD_NeckAngle());
        // ROS_INFO("Distance : %f", img_procPtr->Get_distance());
        // ROS_INFO("EMG : %s", Get_Emergency_() ? "true" : "false");
        // ROS_INFO("-------------------------------------------------------------------");
        // ROS_INFO("-------------------------CALLBACKTHREAD----------------------------");

        ros::spinOnce();
        loop_rate.sleep();
        // usleep(1000);
    }
}

//////////////////////////////////////////////////////////// Server Part ////////////////////////////////////////////////////////////////

bool Move_Decision::SendMotion(dynamixel_current_2port::SendMotion::Request &req, dynamixel_current_2port::SendMotion::Response &res)
{
    // Extract the unique ID included in the request
    int request_id = req.request_id;

    // Check if the request has already been processed
    if (isRequestProcessed(request_id) && !warning_printed)
    {
        if (warning_counter) // Check the counter
        {
            ROS_WARN("Duplicate service response prevented for request ID: %d", request_id);
            warning_printed = true; // Set the flag to true
            warning_counter++;      // Increase the counter
        }
        return false;
    }

    // Finish Check
    bool req_SM_finish = req.SM_finish;
    bool req_TA_finish = req.TA_finish;
    bool req_UD_finish = req.UD_finish;
    bool req_RL_finish = req.RL_finish;
    bool req_EM_finish = req.EM_finish;

    Set_SM_req_finish(req_SM_finish);
    Set_TA_req_finish(req_TA_finish);
    Set_UD_req_finish(req_UD_finish);
    Set_RL_req_finish(req_RL_finish);
    Set_EM_req_finish(req_EM_finish);

    // Response
    auto res_SM = playMotion();
    double res_TA = turn_angle();
    double res_UD = Move_UD_NeckAngle();
    double res_RL = Move_RL_NeckAngle();
    bool res_EM = Emergency();

    if (Get_SM_req_finish())
    {
        if (std::get<0>(res_SM) == Motion_Index::NONE)
        {
            std::get<0>(res_SM) = Get_motion_index_();
            Set_select_motion_on_flg(false);
        }

        res.select_motion = std::get<0>(res_SM);
        res.distance = std::get<1>(res_SM);
    }

    else if (!Get_SM_req_finish())
    {
        res.select_motion = Motion_Index::NONE;
        Set_select_motion_on_flg(false);
    }

    ROS_WARN("[MESSAGE] SM Request :   %s ", Get_SM_req_finish() ? "true" : "false");
    ROS_WARN("[MESSAGE] TA Request :   %s ", Get_TA_req_finish() ? "true" : "false");
    // ROS_WARN("[MESSAGE] RL Request :   %s ", Get_RL_req_finish() ? "true" : "false");
    // ROS_ERROR("[MESSAGE] EMG Request :   %s ", Get_EM_req_finish() ? "true" : "false");

    if (Get_TA_req_finish())
    {
        res.turn_angle = res_TA;
    }

    if (Get_UD_req_finish())
    {
        res.ud_neckangle = res_UD;
    }

    if (Get_RL_req_finish())
    {
        res.rl_neckangle = res_RL;
    }

    if (Get_EM_req_finish())
    {
        res.emergency = res_EM;
    }

    Send_Info(res.select_motion, res.turn_angle, res.ud_neckangle, res.rl_neckangle, res.emergency);

    res.success = true;
    recordProcessedRequest(request_id);

    return true;
}

std::tuple<int8_t, double> Move_Decision::playMotion()
{
    int8_t res_select_motion = 99;
    double res_distance = 0;
    // int8_t total = Get_TA_req_finish() + Get_UD_req_finish() + Get_RL_req_finish() + Get_EM_req_finish();

    if ((Get_stand_status_() == Stand_Status::Stand) && (Get_select_motion_on_flg() == true) /*&& total <=4*/)
    {
        switch (Get_motion_index_())
        {
        case Motion_Index::InitPose:
            res_select_motion = Motion_Index::InitPose;
            break;

        case Motion_Index::Forward_2step:
            res_select_motion = Motion_Index::Forward_2step;
            break;

        case Motion_Index::Left_2step:
            res_select_motion = Motion_Index::Left_2step;
            break;

        case Motion_Index::Step_in_place:
            res_select_motion = Motion_Index::Step_in_place;
            break;

        case Motion_Index::Right_2step:
            res_select_motion = Motion_Index::Right_2step;
            break;

        case Motion_Index::ForWard_fast4step:
            res_select_motion = Motion_Index::ForWard_fast4step;
            break;

        case Motion_Index::Forward_Nstep:
            res_select_motion = Motion_Index::Forward_Nstep;
            if (Get_distance_on_flg())
            {
                res_distance = Get_distance_();
                Set_distance_on_flg(false);
            }
            break;

        case Motion_Index::NONE:
            res_select_motion = Motion_Index::NONE;
            break;

        case Motion_Index::Forward_Halfstep:
            res_select_motion = Motion_Index::Forward_Halfstep;
            break;

        case Motion_Index::Right_Halfstep:
            res_select_motion = Motion_Index::Right_Halfstep;
            break;

        case Motion_Index::Left_Halfstep:
            res_select_motion = Motion_Index::Left_Halfstep;
            break;

        case Motion_Index::Back_Halfstep:
            res_select_motion = Motion_Index::Back_Halfstep;
            break;

        case Motion_Index::Forward_1step:
            res_select_motion = Motion_Index::Forward_1step;
            break;

        case Motion_Index::Shoot:
            res_select_motion = Motion_Index::Shoot;
            break;

        case Motion_Index::Ready_to_throw:
            res_select_motion = Motion_Index::Ready_to_throw;
            break;

        default:
            res_select_motion = Motion_Index::InitPose;
        }

        Set_select_motion_on_flg(false);
    }

    else if (((Get_stand_status_() == Stand_Status::Fallen_Back)) || (Get_stand_status_() == Stand_Status::Fallen_Forward) && (Get_select_motion_on_flg() == true))
    {
        switch (Get_motion_index_())
        {
        case Motion_Index::FWD_UP:
            res_select_motion = Motion_Index::FWD_UP;
            break;

        case Motion_Index::BWD_UP:
            res_select_motion = Motion_Index::BWD_UP;
            break;

        default:
            res_select_motion = Motion_Index::InitPose;
        }
        Set_select_motion_on_flg(false);
    }

    // ROS_WARN("[MESSAGE] SM Request :   %s ", Get_SM_req_finish() ? "true" : "false");
    // ROS_INFO("#[MESSAGE] SM Motion :   %d#", res_select_motion);
    // ROS_INFO("#[MESSAGE] SM Distance : %f#", res_distance);

    return std::make_tuple(res_select_motion, res_distance);
}

void Move_Decision::Test_service()
{
    if (aaaa == 0)
    {
        ROS_ERROR("ccc : %d", ccc);
        ROS_ERROR("aaaa : %d", aaaa);
        abc++;

        if (ccc == 0 && !Get_select_motion_on_flg() /*  && Get_SM_req_finish() */)
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);

            if (!Get_turn_angle_on_flg())
            {
                Set_turn_angle_(10);
                Set_turn_angle_on_flg(true);
            }

            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                ccc++;
            }
        }
        if (ccc == 1 && !Get_select_motion_on_flg() /*  && Get_SM_req_finish() */)
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
            if (!Get_turn_angle_on_flg())
            {
                Set_turn_angle_(10);
                Set_turn_angle_on_flg(true);
            }
            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                ccc++;
            }
        }
        if (ccc == 2 && !Get_select_motion_on_flg() /*  && Get_SM_req_finish() */)
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
            if (!Get_turn_angle_on_flg())
            {
                Set_turn_angle_(10);
                Set_turn_angle_on_flg(true);
            }
            // Sequence++
            if (finish_past != Get_SM_req_finish())
            {
                ccc = 0;
                aaaa++;
            }
        }

        ROS_ERROR("Loop NUMBER(%d)", abc);
    }

    else if (aaaa == 1 && !Get_select_motion_on_flg() /* && Get_SM_req_finish() */)
    {
        Set_motion_index_(Motion_Index::Forward_Halfstep);
        Set_select_motion_on_flg(true);

        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            aaaa++;
        }
    }

    else if (aaaa == 2 && !Get_select_motion_on_flg() /*  && Get_SM_req_finish() */)
    {
        Set_motion_index_(Motion_Index::Left_Halfstep);
        Set_distance_on_flg(true);
        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            aaaa++;
        }
    }

    else if (aaaa == 3 && !Get_select_motion_on_flg() /* && Get_SM_req_finish() */)
    {
        Set_motion_index_(Motion_Index::Right_Halfstep);
        Set_select_motion_on_flg(true);
        // Sequence++
        if (finish_past != Get_SM_req_finish())
        {
            aaaa++;
        }
    }

    else if (aaaa == 4 && !Get_select_motion_on_flg() /* && Get_SM_req_finish() */)
    {
        Set_motion_index_(Motion_Index::Right_2step);
        Set_select_motion_on_flg(true);
        if (finish_past != Get_SM_req_finish())
        {
            aaaa++;
        }
    }

    ROS_WARN("---------------------------------------");
    Motion_Info();
    ROS_WARN("---------------------------------------");
}

double Move_Decision::turn_angle()
{
    double res_turn_angle = 0;

    if ((Get_stand_status_() == Stand_Status::Stand) && Get_turn_angle_on_flg())
    {
        if (Get_motion_index_() == Motion_Index::Forward_2step || Get_motion_index_() == Motion_Index::Step_in_place || Get_motion_index_() == Motion_Index::Forward_Nstep)
        {
            res_turn_angle = this->Get_turn_angle_();
            if (res_turn_angle > TURN_MAX)
                res_turn_angle = TURN_MAX;
            else if (res_turn_angle < TURN_MIN)
                res_turn_angle = TURN_MIN;
            Set_turn_angle_on_flg(false);
        }
    }

    // ROS_WARN("[MESSAGE] TA Request :   %s ", Get_TA_req_finish() ? "true" : "false");
    // ROS_INFO("#[MESSAGE] TA Response : %f#", res_turn_angle);

    return res_turn_angle;
}

double Move_Decision::Move_UD_NeckAngle()
{

    double res_ud_neckangle = UD_CENTER;
    if (Get_running_mode_() == Running_Mode::FAR_HOOP_MODE)
    {
        res_ud_neckangle = 20;
        Set_UD_Neck_on_flg(false);
    }

    else if (!Get_UD_req_finish())
    {
        if (Get_running_mode_() == Running_Mode::FAR_HOOP_MODE)
        {
            res_ud_neckangle = 20;
        }
        else
        {
            res_ud_neckangle = this->Get_UD_NeckAngle();
            Set_UD_Neck_on_flg(false);
        }
    }

    else if ((Get_stand_status_() == Stand_Status::Stand) && Get_UD_Neck_on_flg())
    {
        res_ud_neckangle = this->Get_UD_NeckAngle();
        if (res_ud_neckangle > UD_MAX)
            res_ud_neckangle = UD_MAX;
        else if (res_ud_neckangle < UD_MIN)
            res_ud_neckangle = UD_MIN;
        Set_UD_Neck_on_flg(false);
    }

    // ROS_WARN("[MESSAGE] UD Request :   %s ", Get_UD_req_finish() ? "true" : "false");
    // ROS_INFO("#[MESSAGE] UD Response : %f#", res_ud_neckangle);

    return res_ud_neckangle;
}

double Move_Decision::Move_RL_NeckAngle()
{
    double res_rl_neckangle = 0;

    if ((Get_stand_status_() == Stand_Status::Stand) && Get_RL_Neck_on_flg())
    {
        // img_procssing
        res_rl_neckangle = this->Get_RL_NeckAngle();
        if (res_rl_neckangle > RL_MAX)
            res_rl_neckangle = RL_MAX;
        else if (res_rl_neckangle < RL_MIN)
            res_rl_neckangle = RL_MIN;
        Set_RL_Neck_on_flg(false);
    }

    // ROS_WARN("[MESSAGE] RL Request :   %s ", Get_RL_req_finish() ? "true" : "false");
    // ROS_INFO("#[MESSAGE] RL Response : %f#", res_rl_neckangle);

    return res_rl_neckangle;
}

bool Move_Decision::Emergency()
{
    bool res_emergency = false;

    if ((Get_stand_status_() == Stand_Status::Stand) && Get_emergency_on_flg())
    {
        // 1 : Stop
        // 0 : Keep Going (Option)
        res_emergency = Get_Emergency_();
        Set_emergency_on_flg(false);
    }

    // ROS_ERROR("[MESSAGE] EMG Request :   %s ", Get_EM_req_finish() ? "true" : "false");
    // ROS_ERROR("#[MESSAGE] EMG Response : %s#", res_emergency ? "true" : "false");

    return res_emergency;
}

///////////////////////////////////////// About Publish & Subscribe /////////////////////////////////////////

void Move_Decision::startMode()
{
    bool send_emergency = Get_Emergency_();
    // EmergencyPublish(send_emergency);
    Set_motion_index_(Motion_Index::InitPose);
}

void Move_Decision::IMUsensorCallback(const std_msgs::Float32::ConstPtr &IMU)
{
    if (stop_fallen_check_ == true)
        return;

    // RPY(0) = IMU->data;
    RPY(1) = IMU->data;
    // RPY(2) = IMU->data;

    double pitch = RPY(1) * 57.2958;

    // Quaternino2RPY();
    // sensorPtr->RPY *= (180 / M_PI);
    // double roll = sensorPtr->RPY(0);
    // double pitch = sensorPtr->RPY(1);
    // double yaw = sensorPtr->RPY(2);

    double alpha = 0.4;
    if (present_pitch_ == 0)
        present_pitch_ = pitch;
    else
        present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

    if (present_pitch_ > FALL_FORWARD_LIMIT)
        Set_stand_status_(Stand_Status::Fallen_Forward);
    else if (present_pitch_ < FALL_BACK_LIMIT)
        Set_stand_status_(Stand_Status::Fallen_Back);
    else
        Set_stand_status_(Stand_Status::Stand);

    // ROS_ERROR("STAND_STAUS : %d", Get_stand_status_());
    // ROS_ERROR("PITCH : %f", pitch);
}

// ********************************************** FUNCTION ************************************************** //

Eigen::Vector3d Move_Decision::convertRotationToRPY(const Eigen::Matrix3d &rotation)
{
    Eigen::Vector3d rpy; // = Eigen::MatrixXd::Zero(3,1);

    rpy.coeffRef(0, 0) = atan2(rotation.coeff(2, 1), rotation.coeff(2, 2));
    rpy.coeffRef(1, 0) = atan2(-rotation.coeff(2, 0), sqrt(pow(rotation.coeff(2, 1), 2) + pow(rotation.coeff(2, 2), 2)));
    rpy.coeffRef(2, 0) = atan2(rotation.coeff(1, 0), rotation.coeff(0, 0));

    return rpy;
}

Eigen::Vector3d Move_Decision::convertQuaternionToRPY(const Eigen::Quaterniond &quaternion)
{
    Eigen::Vector3d rpy = convertRotationToRPY(quaternion.toRotationMatrix());

    return rpy;
}

void Move_Decision::Motion_Info()
{
    string tmp_motion;
    switch (Get_motion_index_())
    {
    case Motion_Index::InitPose:
        tmp_motion = Str_InitPose;
        break;

    case Motion_Index::Forward_2step:
        tmp_motion = Str_Forward_2step;
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

    case Motion_Index::Shoot:
        tmp_motion = Str_Shoot;
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

    case Motion_Index::Forward_1step:
        tmp_motion = Str_Forward_1step;
        break;

    case Motion_Index::Left_1step:
        tmp_motion = Str_Left_1step;
        break;

    case Motion_Index::Right_1step:
        tmp_motion = Str_Right_1step;
        break;

    case Motion_Index::Right_6step:
        tmp_motion = Right_6step;
        break;

    case Motion_Index::Left_6step:
        tmp_motion = Left_6step;
        break;

    case Motion_Index::Ready_to_throw:
        tmp_motion = Str_Ready_to_throw;
        break;

    case Motion_Index::NONE:
        tmp_motion = Str_NONE;
        break;
    }
    ROS_INFO("Motion_Index : %s", tmp_motion.c_str());
}

void Move_Decision::Send_Motion_Info(int8_t res_motion)
{
    string tmp_motion;
    switch (res_motion)
    {
    case Motion_Index::InitPose:
        tmp_motion = Str_InitPose;
        break;

    case Motion_Index::Forward_2step:
        tmp_motion = Str_Forward_2step;
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

    case Motion_Index::Shoot:
        tmp_motion = Str_Shoot;
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

    case Motion_Index::Forward_1step:
        tmp_motion = Str_Forward_1step;
        break;

    case Motion_Index::Left_1step:
        tmp_motion = Str_Left_1step;
        break;
        
    case Motion_Index::Right_1step:
        tmp_motion = Str_Right_1step;
        break;

    case Motion_Index::Right_6step:
        tmp_motion = Right_6step;
        break;

    case Motion_Index::Left_6step:
        tmp_motion = Left_6step;
        break;

    case Motion_Index::Ready_to_throw:
        tmp_motion = Str_Ready_to_throw;
        break;

    case Motion_Index::NONE:
        tmp_motion = Str_NONE;
        break;
    }
    ROS_ERROR("SEND Motion_Index : %s", tmp_motion.c_str());
}

void Move_Decision::Send_Info(int8_t motion_, double turn_angle_, double ud, double rl, bool emg)
{
    ROS_INFO("------------------------- SEND ----------------------------");
    Send_Motion_Info(motion_);
    ROS_ERROR("#[MESSAGE] TA Response : %f#", turn_angle_);
    ROS_ERROR("#[MESSAGE] UD Response : %f#", ud);
    ROS_ERROR("#[MESSAGE] RL Response : %f#", rl);
    ROS_ERROR("#[MESSAGE] EMG Response : %s#", emg ? "true" : "false");
    ROS_INFO("------------------------- SEND ----------------------------");
}

void Move_Decision::Running_Info()
{
    string tmp_running;
    switch (Get_running_mode_())
    {
    case Running_Mode::FAR_HOOP_MODE:
        tmp_running = Str_FAR_HOOP_MODE;
        break;

    case Running_Mode::ADJUST_MODE:
        tmp_running = Str_ADJUST_MODE;
        break;

    case Running_Mode::STOP_MODE:
        tmp_running = Str_STOP_MODE;
        break;

    case Running_Mode::WAKEUP_MODE:
        tmp_running = Str_WAKEUP_MODE;
        break;

    case Running_Mode::SHOOT_MODE:
        tmp_running = Str_SHOOT_MODE;
        break;

    case Running_Mode::NO_HOOP_MODE:
        tmp_running = Str_NO_HOOP_MODE;
        break;

        ROS_INFO("Running_Mode : %s", tmp_running.c_str());
    }
}

void Move_Decision::DistanceDecision(double distance_)
{
    if (distance_ > Green_area_dis) // judged to be a straight line. If it exists between the slopes, it is a straight line.
    {
        robot_forward = true;
    }
    else if (distance_ < Green_area_dis)
    {
        robot_forward = false;
    }
}

// ********************************************** GETTERS ************************************************** //

bool Move_Decision::Get_Emergency_() const
{
    std::lock_guard<std::mutex> lock(mtx_Emergency_);
    return Emergency_;
}

bool Move_Decision::Get_emergency_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_emergency_on_flg_);
    return emergency_on_flg_;
}

int8_t Move_Decision::Get_motion_index_() const
{
    std::lock_guard<std::mutex> lock(mtx_motion_index_);
    return motion_index_;
}

int8_t Move_Decision::Get_stand_status_() const
{
    std::lock_guard<std::mutex> lock(mtx_stand_status_);
    return stand_status_;
}

int8_t Move_Decision::Get_running_mode_() const
{
    std::lock_guard<std::mutex> lock(mtx_running_mode_);
    return running_mode_;
}

double Move_Decision::Get_turn_angle_() const
{
    std::lock_guard<std::mutex> lock(mtx_turn_angle_);
    return turn_angle_;
}

double Move_Decision::Get_distance_() const
{
    std::lock_guard<std::mutex> lock(mtx_distance_);
    return distance_;
}

bool Move_Decision::Get_ProcessON() const
{
    std::lock_guard<std::mutex> lock(mtx_ProcessON_);
    return ProcessON_;
}

bool Move_Decision::Get_MoveDecisionON() const
{
    std::lock_guard<std::mutex> lock(mtx_MoveDecisionON_);
    return MoveDecisionON_;
}

bool Move_Decision::Get_CallbackON() const
{
    std::lock_guard<std::mutex> lock(mtx_CallbackON_);
    return CallbackON_;
}

bool Move_Decision::Get_Far_Hoop_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_far_hoop_flg);
    return far_hoop_flg_;
}

bool Move_Decision::Get_Adjust_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_adjust_flg);
    return adjust_flg_;
}

bool Move_Decision::Get_Shoot_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_shoot_flg);
    return shoot_flg_;
}

bool Move_Decision::Get_No_Hoop_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_no_hoop_flg);
    return no_hoop_flg_;
}

bool Move_Decision::Get_Stop_det_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_stop_det_flg);
    return stop_det_flg_;
}

bool Move_Decision::Get_select_motion_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_select_motion_on_flg_);
    return select_motion_on_flg_;
}

bool Move_Decision::Get_turn_angle_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_turn_angle_on_flg_);
    return turn_angle_on_flg_;
}

bool Move_Decision::Get_distance_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_distance_on_flg_);
    return distance_on_flg_;
}

double Move_Decision::Get_UD_NeckAngle() const
{
    std::lock_guard<std::mutex> lock(mtx_UD_NeckAngle_);
    return UD_NeckAngle_;
}

double Move_Decision::Get_RL_NeckAngle() const
{
    std::lock_guard<std::mutex> lock(mtx_RL_NeckAngle_);
    return RL_NeckAngle_;
}

bool Move_Decision::Get_UD_Neck_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_UD_Neck_on_flg);
    return UD_Neck_on_flg_;
}

bool Move_Decision::Get_RL_Neck_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_RL_Neck_on_flg);
    return RL_Neck_on_flg_;
}

bool Move_Decision::Get_SM_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_SM_req_finish_);
    return SM_req_finish_;
}

bool Move_Decision::Get_TA_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_TA_req_finish_);
    return TA_req_finish_;
}

bool Move_Decision::Get_UD_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_UD_req_finish_);
    return UD_req_finish_;
}

bool Move_Decision::Get_RL_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_RL_req_finish_);
    return RL_req_finish_;
}

bool Move_Decision::Get_EM_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_EM_req_finish_);
    return EM_req_finish_;
}

// ********************************************** SETTERS ************************************************** //

void Move_Decision::Set_Emergency_(bool Emergency)
{
    std::lock_guard<std::mutex> lock(mtx_Emergency_);
    this->Emergency_ = Emergency;
}

void Move_Decision::Set_emergency_on_flg(bool emergency_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_emergency_on_flg_);
    this->emergency_on_flg_ = emergency_on_flg;
}

void Move_Decision::Set_motion_index_(int8_t motion_index)
{
    std::lock_guard<std::mutex> lock(mtx_motion_index_);
    this->motion_index_ = motion_index;
}

void Move_Decision::Set_stand_status_(int8_t stand_status)
{
    std::lock_guard<std::mutex> lock(mtx_stand_status_);
    this->stand_status_ = stand_status;
}

void Move_Decision::Set_running_mode_(int8_t running_mode)
{
    std::lock_guard<std::mutex> lock(mtx_running_mode_);
    this->running_mode_ = running_mode;
}

void Move_Decision::Set_turn_angle_(double turn_angle)
{
    std::lock_guard<std::mutex> lock(mtx_turn_angle_);
    this->turn_angle_ = turn_angle;
}

void Move_Decision::Set_distance_(double distance)
{
    std::lock_guard<std::mutex> lock(mtx_distance_);
    this->distance_ = distance;
}

void Move_Decision::Set_ProcessON(bool ProcessON)
{
    std::lock_guard<std::mutex> lock(mtx_ProcessON_);
    this->ProcessON_ = ProcessON;
}

void Move_Decision::Set_MoveDecisionON(bool MoveDecisionON)
{
    std::lock_guard<std::mutex> lock(mtx_MoveDecisionON_);
    this->MoveDecisionON_ = MoveDecisionON;
}

void Move_Decision::Set_CallbackON(bool CallbackON)
{
    std::lock_guard<std::mutex> lock(mtx_CallbackON_);
    this->CallbackON_ = CallbackON;
}

void Move_Decision::Set_Far_Hoop_flg(bool far_hoop_flg)
{
    std::lock_guard<std::mutex> lock(mtx_far_hoop_flg);
    this->far_hoop_flg_ = far_hoop_flg;
}

void Move_Decision::Set_Adjust_flg(bool adjust_flg)
{
    std::lock_guard<std::mutex> lock(mtx_adjust_flg);
    this->adjust_flg_ = adjust_flg;
}

void Move_Decision::Set_Shoot_flg(bool shoot_flg)
{
    std::lock_guard<std::mutex> lock(mtx_shoot_flg);
    this->shoot_flg_ = shoot_flg;
}

void Move_Decision::Set_No_Hoop_flg(bool no_hoop_flg)
{
    std::lock_guard<std::mutex> lock(mtx_no_hoop_flg);
    this->no_hoop_flg_ = no_hoop_flg;
}

void Move_Decision::Set_select_motion_on_flg(bool select_motion_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_select_motion_on_flg_);
    this->select_motion_on_flg_ = select_motion_on_flg;
}

void Move_Decision::Set_turn_angle_on_flg(bool turn_angle_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_turn_angle_on_flg_);
    this->turn_angle_on_flg_ = turn_angle_on_flg;
}

void Move_Decision::Set_distance_on_flg(bool distance_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_distance_on_flg_);
    this->distance_on_flg_ = distance_on_flg;
}

void Move_Decision::Set_RL_NeckAngle(double RL_NeckAngle)
{
    std::lock_guard<std::mutex> lock(mtx_RL_NeckAngle_);
    this->RL_NeckAngle_ = RL_NeckAngle;
}

void Move_Decision::Set_UD_NeckAngle(double UD_NeckAngle)
{
    std::lock_guard<std::mutex> lock(mtx_UD_NeckAngle_);
    this->UD_NeckAngle_ = UD_NeckAngle;
}

void Move_Decision::Set_RL_Neck_on_flg(bool RL_Neck_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_RL_Neck_on_flg);
    this->RL_Neck_on_flg_ = RL_Neck_on_flg;
}

void Move_Decision::Set_UD_Neck_on_flg(bool UD_Neck_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_UD_Neck_on_flg);
    this->UD_Neck_on_flg_ = UD_Neck_on_flg;
}

void Move_Decision::Set_SM_req_finish(bool SM_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_SM_req_finish_);
    this->SM_req_finish_ = SM_req_finish;
}

void Move_Decision::Set_TA_req_finish(bool TA_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_TA_req_finish_);
    this->TA_req_finish_ = TA_req_finish;
}

void Move_Decision::Set_UD_req_finish(bool UD_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_UD_req_finish_);
    this->UD_req_finish_ = UD_req_finish;
}

void Move_Decision::Set_RL_req_finish(bool RL_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_RL_req_finish_);
    this->RL_req_finish_ = RL_req_finish;
}

void Move_Decision::Set_EM_req_finish(bool EM_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_EM_req_finish_);
    this->EM_req_finish_ = EM_req_finish;
}
