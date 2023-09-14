#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <cmath>

#include "dynamixel.hpp"
#include "callback.hpp"
#include "dynamixel_controller.hpp"
#include "Walkingpattern_generator.hpp"
#include "sensor.hpp"

#include "dynamixel_current_2port/Select_Motion.h"
#include "dynamixel_current_2port/Turn_Angle.h"

int low_h = 0, low_s = 150, low_v = 100;
int high_h = 100, high_s = 255, high_v = 255;

void on_low_h_thresh_trackbar(int, void *)
{
    low_h = cv::min(high_h - 1, low_h);
    cv::setTrackbarPos("Low H", "Threshold", low_h);
}
void on_high_h_thresh_trackbar(int, void *)
{
    high_h = cv::max(high_h, low_h + 1);
    cv::setTrackbarPos("High H", "Threshold", high_h);
}
// ... 비슷한 함수들을 S와 V에 대해서도 생성 ...

int main(int argc, char **argv)
{
    ros::init(argc, argv, "daynmixel_current_2port");
    ros::Time::init();
    ros::Rate loop_rate(30);
    ros::NodeHandle nh;

    Dxl dxl;
    // Dxl_Controller dxl_ctrl(&dxl);
    // Motions motion;
    // Callback callback(&motion, &dxl);
    // Sensor sensor(&motion, &callback);

    VectorXd A(3);
    for (int i = 0; i < 3; i++)
    {
        A(i) = 45 * DEG2RAD;
    }

    rs2::pipeline pipe;
    rs2::config cfg;

    // Depth & Color Stream 설정
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // 파이프 라인 시작
    pipe.start(cfg);

    const auto window_name = "RealSense Color Stream";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    // 트랙바 생성
    cv::namedWindow("Threshold", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Low H", "Threshold", &low_h, 180, on_low_h_thresh_trackbar);
    cv::createTrackbar("High H", "Threshold", &high_h, 180, on_high_h_thresh_trackbar);
    // ... S와 V에 대한 트랙바도 생성 ...

    while (cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();

        cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);

        cv::Mat hsv_image;
        cv::cvtColor(color_image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat yellow_mask;
        cv::inRange(hsv_image, cv::Scalar(low_h, low_s, low_v), cv::Scalar(high_h, high_s, high_v), yellow_mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 컨투어 그리기
        cv::drawContours(color_image, contours, -1, cv::Scalar(0, 255, 0), 3);

        // 노란색 영역의 중심점 찾기
        cv::Moments m = cv::moments(yellow_mask, false);
        if (m.m00 != 0)
        { // 노란색 영역이 있는 경우
            cv::Point center(m.m10 / m.m00, m.m01 / m.m00);

            // 중심점에 대한 실제 거리 계산
            float distance = depth_frame.as<rs2::depth_frame>().get_distance(center.x, center.y);
            float distance_cos = distance * (std::sqrt(2.0) / 2.0);

            std::cout << "Distance : " << distance << " meters       "
                      << "Diatance_cos(30) : " << distance_cos << " meters" << std::endl;

            // 중심점 표시
            cv::circle(color_image, center, 10, cv::Scalar(0, 0, 255), -1);
        }
        else
        {
            std::cout << "No yellow area detected." << std::endl;
        }

        dxl.SetThetaRef(A);
        dxl.syncWriteTheta();

        ros::spinOnce();
        loop_rate.sleep();

        cv::imshow(window_name, color_image);
        cv::imshow("Threshold", yellow_mask); // 이진화 이미지 표시
        if (cv::waitKey(1) >= 0)
            break;
    }

    dxl.~Dxl();
    return 0;
}
