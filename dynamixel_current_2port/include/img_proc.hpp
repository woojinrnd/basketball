#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <librealsense2/rs.hpp>
#include <vector>
#include <mutex>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>


#define TOP_BORDER_LINE 240    // == (IMG_H / 2)
#define BOTTOM_BORDER_LINE 460 // == (IMG_H - 20) change this value when Up-Down Neck Angle changed or to adjust with RV value
#define LEFT_BORDER_LINE 300   // == (IMG_W/2 - 20)
#define RIGHT_BORDER_LINE 260  // == (IMG_W/2 + 20)

#define CIRCLE_RADIUS 100 // 50 -> 60 -> 100

#define LEFT_EDGE_BORDER_LINE 0 + 15
#define RIGHT_EDGE_BORDER_LINE 640 - 15

#define RR_LINE_CURVATURE 0.004 // 0.005 -> 0.004
#define Y_VERTEX 90

#define NOISE_DELETE_DELTA_X 120
#define CONTOUR_AREA 200
#define MIN_CONTOUR_AREA 50
#define NO_LINE_DETECT_DX 160

#define IMG_W 848
#define IMG_H 480

#define PROP_EXPOSURE -6
#define PROP_GAIN 128
#define PROP_TEMPERATURE 4985
#define PROP_BRIGHTNESS 128
#define PROP_CONTRAST 128
#define PROP_SATURATION 128

#define ADJUST_X_MARGIN 40
#define CREEK 30

using namespace cv;
using namespace std;

class Img_proc
{
public:
    Img_proc();
    ~Img_proc(){};

    // ********************************************** 2D THREAD************************************************** //

    void webcam_thread();

    void topic_publish(const std::string &topic_name)
    {
        pub = nh.advertise<std_msgs::Float32>(topic_name, 1000);
    }

    void publish_message(float msg_data)
    {
        std_msgs::Float32 msg;
        msg.data = msg_data;
        pub.publish(msg);
    }

    // Cam set
    const int webcam_width = 640;
    const int webcam_height = 480;
    const int webcam_fps = 30;
    const int webcam_id = 0;

    int threshold_value_white = 180;
    int threshold_value_yellow = 127;
    int threshold_value_black = 50;

    const int max_value = 255;
    int hue_lower = 0;
    int hue_upper = 179;
    int saturation_lower = 0;
    int saturation_upper = 255;
    int value_lower = 0;
    int value_upper = 255;

    bool has_white_prev = false;
    bool has_yellow_prev = false;

    cv::Scalar blue_color = {255, 0, 0};
    cv::Scalar green_color = {0, 255, 0};
    cv::Scalar red_color = (0, 0, 255);

    cv::Scalar lower_bound_led = {160, 50, 20};
    cv::Scalar upper_bound_led = {179, 255, 255};

    cv::Scalar lower_bound_green = {35, 50, 50};
    cv::Scalar upper_bound_green = {85, 255, 255};

    cv::Scalar lower_bound_blue = {85, 150, 55};
    cv::Scalar upper_bound_blue = {125, 255, 255};


    bool a = 0;

    bool left = false;
    bool right = false;

    static void on_trackbar(int, void *);
    void create_threshold_trackbar_W(const std::string &window_name);
    void create_threshold_trackbar_Y(const std::string &window_name);
    void create_color_range_trackbar(const std::string &window_name);
    void create_threshold_trackbar_Black(const std::string &window_name);
    std::tuple<cv::Mat, cv::Mat> extract_color(const cv::Mat &input_frame, const cv::Scalar &lower_bound, const cv::Scalar &upper_bound);
    std::tuple<cv::Mat, bool, std::vector<cv::Point>, double> Is_AreaThreshold(const cv::Mat& image, cv::Scalar lower_bound, cv::Scalar upper_bound, int green_area);
    double MinDistanceBetweenContours(cv::Mat input_frame, const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2);


    // ********************************************** 3D THREAD************************************************** //
    int Hoop_Location(cv::Mat &color, cv::Point center);
    std::tuple<cv::Mat, cv::Point2f> Hoop_Detect(cv::Mat color, cv::Mat depth, cv::Mat depth_dist, int threshold_value);
    void realsense_thread();

    // Cam set
    const int realsense_width = 848;
    const int realsense_height = 480;
    const int realsense_fps = 30;

    // ********************************************** GETTERS ************************************************** //

    bool Get_img_proc_Far_Hoop_det() const;
    bool Get_img_proc_Adjust_det() const;
    bool Get_img_proc_Shoot_det() const;
    bool Get_img_proc_No_Hoop_det() const;
    bool Get_img_proc_stop_det() const;
    int8_t Get_img_proc_Adjust_number() const;

    double Get_gradient() const;
    double Get_delta_x() const;
    double Get_distance() const;
    double Get_adjust_angle() const;
    int Get_contain_adjust_to_foot() const;

    // ********************************************** SETTERS ************************************************** //

    void Set_img_proc_Far_Hoop_det(bool img_proc_far_hoop_det);
    void Set_img_proc_Adjust_det(bool img_proc_adjust_det);
    void Set_img_proc_Shoot_det(bool img_proc_adjust_det);
    void Set_img_proc_No_Hoop_det(bool img_proc_no_hoop_det);
    void Set_img_proc_stop_det(bool img_proc_stop_det);
    void Set_img_proc_adjust_number(int8_t img_proc_adjust_number);
    void Set_img_proc_wall_number(int8_t img_proc_adjust_number);

    void Set_gradient(double gradient);
    void Set_delta_x(double delta_x);
    void Set_distance(double set_distance);
    void Set_adjust_angle(double adjust_angle);
    void Set_contain_adjust_to_foot(int contain_adjust_to_foot);

    // ********************************************** running ************************************************** //

    cv::VideoCapture vcap;
    Mat Origin_img;

    void RGB2HSV(const cv::Mat &rgb_image, cv::Mat &hsv_image);
    void RGB2LAB(const cv::Mat &rgb_image, cv::Mat &lab_image);
    void saveParameters(const std::string &filename);
    void loadParameters(const std::string &filename);

    void running_process();
    // void extractAndDisplayObject2(cv::VideoCapture& cap, const cv::Scalar& hsv_lower, const cv::Scalar& hsv_upper, const cv::Scalar& lab_lower, const cv::Scalar& lab_upper);

    void init();
    /////////////////////LINE////////////////////
    Point tmp_point_target = Point(IMG_W / 2, IMG_H / 2);
    Point point_target = Point(IMG_W / 2, IMG_H);
    std::vector<std::vector<cv::Point>> contours_;
    double delta_x_list[3] = {0.f, 0.f, 0.f};
    // cv::Mat final_binary_mask;
    cv::Mat final_binary_mask = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC1);

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    const int SPIN_RATE;

    // HSV and LAB parameter values
    int h_min, h_max, s_min, s_max, v_min, v_max;
    int l_min, l_max, a_min, a_max, b_min, b_max;

    // Determine flg from img_proc
    bool img_proc_far_hoop_det_ = false;
    bool img_proc_adjust_det_ = false;
    bool img_proc_shoot_det_ = false;
    bool img_proc_no_hoop_det_ = false;
    bool img_proc_stop_det_ = false;
    int8_t img_proc_adjust_number_ = 0;

    // Hoop mode
    double gradient_ = 0; // Line_angle
    double distance_ = 0; // huddle / wall mode

    // No Line mode
    // delta_x : Center of window.x - Center of last captured line.x
    // delta_x > 0 : LEFT
    // delta_x < 0 : RIGHT
    double delta_x_ = 0;

    //Adjust mode
    double adjust_angle_ = 0;
    int contain_adjust_to_foot_ = 0;

    /////////////////////////////////////////// Mutex ///////////////////////////////////////////
    // LINE Determine flg from img_proc
    mutable std::mutex mtx_img_proc_far_hoop_det_;
    mutable std::mutex mtx_img_proc_adjust_det_;
    mutable std::mutex mtx_img_proc_shoot_det_;
    mutable std::mutex mtx_img_proc_no_hoop_det_;
    mutable std::mutex mtx_img_proc_stop_det_;
    mutable std::mutex mtx_img_proc_adjust_number_;

    // Far_Hoop Mode
    mutable std::mutex mtx_gradient;
    mutable std::mutex mtx_distance;
    // No Hoop Mode
    mutable std::mutex mtx_delta_x;
    //Adjust Mode
    mutable std::mutex mtx_adjust_angle;
    mutable std::mutex mtx_contain_adjust_to_foot;
};