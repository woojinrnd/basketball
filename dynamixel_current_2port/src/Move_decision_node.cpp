#include <iostream>
#include <time.h>
#include "Move_decision.hpp"
// #include "img_proc.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Move_decision");
    ros::Time::init();
    ros::Rate loop_rate(1);
    ros::NodeHandle nh;

    Img_proc img_proc;
    Move_Decision move_decision(&img_proc);
    // Move_Decision move_decision;



    // ros::Subscriber Motion_Selector_; ///< Gets Motion number from motion_decision
    // Motion_Selector_ = nh.subscribe("/Move_decision/Select_Motion", 1000, &Callback::SelectMotion, &callback);

    // Timer
    struct timespec start, end;
    double run_time;
    clock_gettime(CLOCK_REALTIME, &start); // Wall-clock time

    // callback.Write_Arm_Theta();
    // callback.MotionMaker();

    while (ros::ok())
    {
        // cout << "woojin" << endl;
        // callback.Write_Leg_Theta();
        // dxl.SetThetaRef(callback.All_Theta);
        // dxl.syncWriteTheta();
        

        ros::spinOnce();
        loop_rate.sleep();
        
    }

    clock_gettime(CLOCK_REALTIME, &end);                                                         // Wall-clock time
    run_time = (end.tv_sec - start.tv_sec) * 1000.0 + (end.tv_nsec - start.tv_nsec) / 1000000.0; // 단위는 ms

    move_decision.~Move_Decision();
    cout << run_time << endl;
    return 0;
}

/////////////////////////////////THREAD_EXAMPLE/////////////////////////////////
// #include <ros/ros.h>
// #include <std_msgs/Empty.h>
// #include <boost/thread/thread.hpp>

// void do_stuff(int* publish_rate)
// {
//   ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
//   ros::Publisher pub_b = node->advertise<std_msgs::Empty>("topic_b", 10);

//   ros::Rate loop_rate(*publish_rate);
//   while (ros::ok())
//   {
//     std_msgs::Empty msg;
//     pub_b.publish(msg);
//     loop_rate.sleep();
//   }
// }

// int main(int argc, char** argv)
// {
//   int rate_b = 1; // 1 Hz

//   ros::init(argc, argv, "mt_node");

//   // spawn another thread
//   boost::thread thread_b(do_stuff, &rate_b);

//   ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
//   ros::Publisher pub_a = node->advertise<std_msgs::Empty>("topic_a", 10);

//   ros::Rate loop_rate(10); // 10 Hz
//   while (ros::ok())
//   {
//     std_msgs::Empty msg;
//     pub_a.publish(msg);
//     loop_rate.sleep();

//     // process any incoming messages in this thread
//     ros::spinOnce();
//   }

//   // wait the second thread to finish
//   thread_b.join();

//   return 0;
// }