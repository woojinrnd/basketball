#ifndef SENSOR_H
#define SENSOR_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <boost/thread.hpp>
#include <tf/tf.h>

class Sensor
{
public:
    Sensor();
    ~Sensor();

    // ********************************************** FILTER ************************************************** //

    virtual float LPF(float x_k, float y_pre, float Ts, float tau_LPF);
    // virtual float LPF(const float& x_k, const float& y_pre, const float Ts, const float tau_LPF);
    virtual float HPF(float x_k, float x_pre, float y_pre, float Ts, float tau_HPF);
    // virtual float HPF(const float& x_k, const float& x_pre, const float& y_pre, const float Ts, const float tau_HPF);
    virtual float HPF_Integral(float x_k, float y_pre, float Ts, float tau_HPF_Integral);
    virtual float Integral(float x_k, float y_pre, float Ts);
    virtual float Complementary(float gyro, float HPF_Int, float alpha);

    // ********************************************** SUBSCRIBER ************************************************** //
    // **********************************************  TRHEAD ************************************************** //
    
    virtual void SensorPublishThread();
    Eigen::VectorXd quaternion = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd RPY = Eigen::VectorXd::Zero(3);   // Roll Pitch Yaw
    Eigen::VectorXd Accel = Eigen::VectorXd::Zero(3); // Accel_x, Accel_y, Accel_z
    Eigen::VectorXd Gyro = Eigen::VectorXd::Zero(3);  // Gyro_x, Gyro_y, Gyro_z

    // ********************************************** PUBLISHER ************************************************** //
    // **********************************************  TRHEAD ************************************************** //
    virtual void IMUcallbackThread();
    virtual void IMUsensorCallback(const sensor_msgs::Imu::ConstPtr &IMU);
    ros::Subscriber IMU_sensor_subscriber_; ///< Gets IMU Sensor data from XSENSE mti_driver_node

    virtual void Publish_Angle();
    virtual void Publish_Gyro_Origin();
    virtual void Publish_Accel_Origin();
    virtual void Publish_Gyro_LPF();
    virtual void Publish_Accel_HPF();
    virtual void Publish_Velocity_Integral();
    virtual void Publish_Velocity_HPF_Integral();
    virtual void Publish_Velocity_Complementary();

    geometry_msgs::Vector3 gyro;
    geometry_msgs::Vector3 accel;

    // Angle
    ros::Publisher IMU_Angle_X_publisher_;
    ros::Publisher IMU_Angle_Y_publisher_;
    ros::Publisher IMU_Angle_Z_publisher_;

    // Origin Gyro
    ros::Publisher IMU_Gryo_x_publisher_; ///< Publishes Imu/gyro.x from reads
    ros::Publisher IMU_Gryo_y_publisher_; ///< Publishes Imu/gyro.y from reads
    ros::Publisher IMU_Gryo_z_publisher_; ///< Publishes Imu/gyro.z from reads

    // Origin Accel
    ros::Publisher IMU_Accel_x_publisher_; ///< Publishes Imu/accel.x from reads
    ros::Publisher IMU_Accel_y_publisher_; ///< Publishes Imu/accel.y from reads
    ros::Publisher IMU_Accel_z_publisher_; ///< Publishes Imu/accel.z from reads

    // Filtered Accel (HPF)
    ros::Publisher IMU_Accel_filtered_x_publisher_; ///< Publishes Imu/accel.x from reads
    ros::Publisher IMU_Accel_filtered_y_publisher_; ///< Publishes Imu/accel.y from reads
    ros::Publisher IMU_Accel_filtered_z_publisher_; ///< Publishes Imu/accel.z from reads

    // Filterd Gyro (LPF)
    ros::Publisher IMU_Gryo_filtered_x_publisher_; ///< Publishes Imu/gyro.x from reads
    ros::Publisher IMU_Gryo_filtered_y_publisher_; ///< Publishes Imu/gyro.y from reads
    ros::Publisher IMU_Gryo_filtered_z_publisher_; ///< Publishes Imu/gyro.z from reads

    // Filterd Accel (Integral)
    ros::Publisher IMU_Velocity_x_publisher_; ///< Publishes Imu/accel.x from reads
    ros::Publisher IMU_Velocity_y_publisher_; ///< Publishes Imu/accel.y from reads
    ros::Publisher IMU_Velocity_z_publisher_; ///< Publishes Imu/accel.z from reads

    // Filterd Accel (HPF_Integral)
    ros::Publisher IMU_Velocity_filtered_x_publisher_; ///< Publishes Imu/accel.x from reads
    ros::Publisher IMU_Velocity_filtered_y_publisher_; ///< Publishes Imu/accel.y from reads
    ros::Publisher IMU_Velocity_filtered_z_publisher_; ///< Publishes Imu/accel.z from reads

    // Filterd Accel (Complementary Filter)
    ros::Publisher IMU_Velocity_Complementary_x_publisher_; ///< Publishes Imu/accel.x from reads
    ros::Publisher IMU_Velocity_Complementary_y_publisher_; ///< Publishes Imu/accel.y from reads
    ros::Publisher IMU_Velocity_Complementary_z_publisher_; ///< Publishes Imu/accel.z from reads

    // ********************************************** FUNCTION ************************************************** //
    void Quaternino2RPY();


    // virtual MatrixXd GetCapturePoint();
    // virtual MatrixXd Reference_CP_CM(MatrixXd &_motion);

private:
    ros::NodeHandle nh_;
    const int SPIN_RATE;

    // Sampling time, tau value
    float Ts = 0.002; // 1 / (loop_rate)
    float tau_LPF = 0.5;
    float tau_HPF = 0.5;
    float tau_HPF_Integral = 0.5;
    float alpha = 0.7;
    float L = 0.45; // foot ~ IMU [m]

    // geometry_msgs::Vector3 gyro;
    // geometry_msgs::Vector3 accel;

    // Eigen::VectorXd quaternion = Eigen::VectorXd::Zero(4);
    // Eigen::VectorXd RPY = Eigen::VectorXd::Zero(3);   // Roll Pitch Yaw
    // Eigen::VectorXd Accel = Eigen::VectorXd::Zero(3); // Accel_x, Accel_y, Accel_z
    // Eigen::VectorXd Gyro = Eigen::VectorXd::Zero(3);  // Gyro_x, Gyro_y, Gyro_z

    // Initialize previous filtered values (initial = 0)
    float lpf_y_pre_x = 0;
    float lpf_y_pre_y = 0;
    float lpf_y_pre_z = 0;
    // Initialize previous input values (initial = 0)
    float hpf_x_pre_x = 0;
    float hpf_x_pre_y = 0;
    float hpf_x_pre_z = 0;
    // Initialize previous filtered values (initial = 0)
    float hpf_y_pre_x = 0;
    float hpf_y_pre_y = 0;
    float hpf_y_pre_z = 0;
    // Initialize previous filtered values (initialized to 0)
    float hpf_yi_pre_x = 0;
    float hpf_yi_pre_y = 0;
    float hpf_yi_pre_z = 0;
};

#endif