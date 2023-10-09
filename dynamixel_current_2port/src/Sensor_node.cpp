#include <iostream>
#include <time.h>
#include "sensor.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Sensor_node");
    ros::Time::init();
    ros::Rate loop_rate(400);
    ros::NodeHandle nh;

    Sensor sensor;


    // callback.Write_Arm_Theta();
    // callback.MotionMaker();

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}