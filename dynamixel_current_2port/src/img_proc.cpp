#include "img_proc.hpp"

// Constructor
Img_proc::Img_proc()
    : SPIN_RATE(1),
      img_proc_far_hoop_det_(false),
      gradient_(0)
{
}

// ********************************************** 2D THREAD************************************************** //

void Img_proc::on_trackbar(int, void *)
{
    // Function body if required.
}

void Img_proc::create_threshold_trackbar_W(const std::string &window_name)
{
    cv::createTrackbar("Threshold_white", window_name, &threshold_value_white, max_value, on_trackbar);
}

void Img_proc::create_threshold_trackbar_Y(const std::string &window_name)
{
    cv::createTrackbar("Threshold_yellow", window_name, &threshold_value_yellow, max_value, on_trackbar);
}

void Img_proc::create_color_range_trackbar(const std::string &window_name)
{
    cv::createTrackbar("Hue Lower", window_name, &hue_lower, 179, on_trackbar);
    cv::createTrackbar("Hue Upper", window_name, &hue_upper, 179, on_trackbar);
    cv::createTrackbar("Saturation Lower", window_name, &saturation_lower, 255, on_trackbar);
    cv::createTrackbar("Saturation Upper", window_name, &saturation_upper, 255, on_trackbar);
    cv::createTrackbar("Value Lower", window_name, &value_lower, 255, on_trackbar);
    cv::createTrackbar("Value Upper", window_name, &value_upper, 255, on_trackbar);
}

void Img_proc::create_threshold_trackbar_Black(const std::string &window_name)
{
    cv::createTrackbar("Threshold_Black", window_name, &threshold_value_black, max_value, on_trackbar);
}

std::tuple<cv::Mat, bool, std::vector<cv::Point>, double> Img_proc::Is_AreaThreshold(const cv::Mat& image, cv::Scalar lower_bound, cv::Scalar upper_bound, int green_area)
{
    cv::Mat extract = image.clone();

    cv::Mat hsv;
    cv::cvtColor(extract, hsv, cv::COLOR_BGR2HSV);

    // 초록색 영역을 이진화
    cv::Mat mask;
    cv::inRange(hsv, lower_bound, upper_bound, mask);

    // 연결된 요소를 찾기
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool is_Green = false;
    std::vector<cv::Point> largestContour;

    double maxArea = 0.0;
    double angle_line = 0.0;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area >= green_area && area > maxArea) {
            maxArea = area;
            largestContour = contour;
            is_Green = true;
        }
    }

    if (is_Green) {
        cv::drawContours(extract, std::vector<std::vector<cv::Point>>{largestContour}, 0, cv::Scalar(0, 255, 0), 2);

        cv::Point startPoint = largestContour[0];
        cv::Point endPoint = largestContour[largestContour.size() - 1];
        double deltaY = static_cast<double>(endPoint.y - startPoint.y);
        double deltaX = static_cast<double>(endPoint.x - startPoint.x);
        double angleRadians = std::atan2(deltaY, deltaX);
        double angleDegrees = angleRadians * (180.0 / CV_PI);
        angle_line = angleDegrees;
    }
    return {extract, is_Green, largestContour, angle_line};
}

double Img_proc::MinDistanceBetweenContours(cv::Mat input_frame, const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2)
{
    double minDistance = 1000;
    cv::Point closestPoint1, closestPoint2;

    for (const auto& point1 : contour1) {
        for (const auto& point2 : contour2) {
            double distance = cv::norm(point1 - point2);
            if (distance < minDistance) {
                minDistance = distance;
                closestPoint1 = point1;
                closestPoint2 = point2;
            }
        }
    }
    putText(input_frame, "Distance_Foot_Green : " + std::to_string(minDistance), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    cv::line(input_frame, closestPoint1, closestPoint2, green_color, 4);

    return minDistance;
}

float computeEllipseAngle(const cv::Mat& input_frame, const std::vector<cv::Point>& contour)
{
    if (contour.size() < 5)
    {
        std::cerr << "활꼴을 찾을 수 없습니다" << std::endl;
        return 0.0f;
    }

    cv::RotatedRect minEllipse = cv::fitEllipse(contour);

    cv::Point2f points[2];
    minEllipse.points(points);
    cv::Point2f point1 = points[0];
    cv::Point2f point2 = points[1];

    float a = (point2.y - point1.y) / (point2.x - point1.x);

    float angle = atan(a) * 180.0 / CV_PI;

    putText(input_frame, "Angle_Green : " + std::to_string(angle), cv::Point(10, 75), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

    return angle;
}


void Img_proc::webcam_thread()
{
    // // // TEST
    // Set_img_proc_wall_det(true);
    // Set_img_proc_corner_number(1);

    // CAMERA INIT
    cv::VideoCapture cap(webcam_id);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, webcam_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, webcam_height);
    cap.set(cv::CAP_PROP_FPS, webcam_fps);
    cap.set(CAP_PROP_AUTOFOCUS, false);
    cap.set(CAP_PROP_AUTO_EXPOSURE, false);

    cap.set(CAP_PROP_EXPOSURE, PROP_EXPOSURE);
    cap.set(CAP_PROP_GAIN, PROP_GAIN);
    cap.set(CAP_PROP_BRIGHTNESS, PROP_BRIGHTNESS);
    cap.set(CAP_PROP_CONTRAST, PROP_CONTRAST);
    cap.set(CAP_PROP_SATURATION, PROP_SATURATION);
    cap.set(CAP_PROP_TEMPERATURE, PROP_TEMPERATURE);

    if (!cap.isOpened())
    {
        std::cerr << "Could not open the webcam\n";
        return;
    }

    const std::string window_Green = "Green_Extract";
    const std::string window_Blue = "Blue_Extract";

    cv::namedWindow(window_Green);
    cv::namedWindow(window_Blue);

    cv::Mat frame, hsv_frame_white, hsv_frame_yellow, thresh_frame_white, thresh_frame_yellow, gray;

    double Distance_foot_to_green = 1000;
    float angle_Green = 0;

    while (ros::ok())
    {
        cap >> frame;
        if (frame.empty())
            break;

        auto Green_Exis = Is_AreaThreshold(frame, lower_bound_green, upper_bound_green, 1000);
        auto Foot_Exis = Is_AreaThreshold(frame, lower_bound_led, upper_bound_led, 10);

        std::vector<cv::Point>Green_contour = std::get<2>(Green_Exis);
        std::vector<cv::Point>Blue_contour = std::get<2>(Foot_Exis);

        bool is_Green = std::get<1>(Green_Exis);

        if(is_Green)
        {
            this->Set_img_proc_Far_Hoop_det(false);
            this->Set_img_proc_Adjust_det(true);

            Distance_foot_to_green = MinDistanceBetweenContours(std::get<0>(Green_Exis), Green_contour, Blue_contour);
            angle_Green = computeEllipseAngle(std::get<0>(Green_Exis), Green_contour);

            this->Set_contain_adjust_to_foot(Distance_foot_to_green);
            this->Set_adjust_angle(angle_Green);
        }
        else
        {
            this->Set_img_proc_Far_Hoop_det(true);
            this->Set_img_proc_Adjust_det(false);
            this->Set_adjust_angle(0);
        }

        cv::imshow(window_Green, std::get<0>(Green_Exis));
        cv::imshow(window_Blue, std::get<0>(Foot_Exis));

        if (cv::waitKey(1) == 27)
            break;
        // loop_rate.sleep();
    }

    vcap.release();
    cv::destroyAllWindows();
}

// // ********************************************** 3D THREAD************************************************** //

std::tuple<cv::Mat, cv::Point2f> Img_proc::Hoop_Detect(cv::Mat color, cv::Mat depth, cv::Mat depth_dist, int threshold_value)
{
    cv::Mat mask = depth_dist < 1300;

    cv::Mat output;
    color.copyTo(output, mask);

    cv::Mat inverse_mask = depth_dist >= 1300;
    output.setTo(cv::Scalar(255, 255, 255), inverse_mask);

    cv::Rect roi(0, 100, output.cols, 200);

    output.rowRange(0, 100).setTo(cv::Scalar(255, 255, 255));
    output.rowRange(300, output.rows).setTo(cv::Scalar(255, 255, 255));

    cv::Mat red_mask, hsv;
    cv::cvtColor(output, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(160, 120, 100), cv::Scalar(179, 255, 255), red_mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::drawContours(output, contours, -1, cv::Scalar(0, 0, 255), 2);

    cv::Point2f black_center;

    cv::Mat grayscale;
    cv::cvtColor(output(roi), grayscale, cv::COLOR_BGR2GRAY);

    cv::Mat binary;
    cv::threshold(grayscale, binary, threshold_value, 255, cv::THRESH_BINARY_INV);

    std::vector<std::vector<cv::Point>> black_contours;
    cv::findContours(binary, black_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!black_contours.empty())
    {
        std::sort(black_contours.begin(), black_contours.end(), [](const std::vector<cv::Point> &c1, const std::vector<cv::Point> &c2)
                  { return cv::contourArea(c1, false) > cv::contourArea(c2, false); });

        cv::drawContours(output, black_contours, 0, cv::Scalar(255, 0, 0), 2, 8, cv::noArray(), INT_MAX, roi.tl());

        cv::Moments moments = cv::moments(black_contours[0]);
        if (moments.m00 != 0)
        {
            black_center = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);
            black_center += cv::Point2f(static_cast<float>(roi.x), static_cast<float>(roi.y));
            cv::circle(output, black_center, 5, cv::Scalar(0, 0, 255), -1);
        }
    }

    // cv::line(output, cv::Point(0, 100), cv::Point(640, 100), red_color, 3);
    // cv::line(output, cv::Point(0, 300), cv::Point(640, 300), red_color, 3);

    return std::make_tuple(output, black_center);
}

int Img_proc::Hoop_Location(cv::Mat &color, cv::Point center)
{
    int x_left_bound = (IMG_W / 2 + CREEK) - ADJUST_X_MARGIN;
    int x_right_bound = (IMG_W / 2 + CREEK) + ADJUST_X_MARGIN;
    int y_top_bound = 10;
    int y_bottom_bound = 470;

    int x_pixel_toShoot = 0;

    rectangle(color, cv::Point(x_left_bound, y_top_bound), cv::Point(x_right_bound, y_bottom_bound), cv::Scalar(255, 255, 0), 2);

    if (center.x < x_left_bound)
    {
        putText(color, "Left", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        x_pixel_toShoot = 1;
    }
    else if (center.x > x_right_bound)
    {
        putText(color, "Right", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        x_pixel_toShoot = -1;
    }

    else
    {
        putText(color, "Inside", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        x_pixel_toShoot = 0;
    }

    return x_pixel_toShoot;
}

void Img_proc::realsense_thread()
{
    rs2::colorizer color_map;
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, realsense_width, realsense_height, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, realsense_width, realsense_height, RS2_FORMAT_Z16, 30);

    try
    {
        pipe.start(cfg);
    }
    catch (const rs2::error &e)
    {
        std::cerr << "Failed to open the RealSense camera: " << e.what() << std::endl;
        return;
    }

    const auto window_name = "Distance Measurement";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    const auto window_name_real_color = "Realsense Color Frame";
    cv::namedWindow(window_name_real_color, cv::WINDOW_AUTOSIZE);

    create_threshold_trackbar_Black(window_name_real_color);

    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::spatial_filter spatial;
    rs2::temporal_filter temporal;
    rs2::hole_filling_filter hole_filling;

    try
    {
        while (ros::ok() && cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
        {
            rs2::frameset data = pipe.wait_for_frames();
            data = align_to.process(data);

            rs2::depth_frame depth_frame = data.get_depth_frame();

            depth_frame = hole_filling.process(depth_frame);

            rs2::frame depth = depth_frame;
            rs2::frame color = data.get_color_frame();

            color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);

            float depth_scale = pipe.get_active_profile().get_device().first<rs2::depth_sensor>().get_depth_scale();
            rs2_intrinsics intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

            const int w = depth.as<rs2::video_frame>().get_width();
            const int h = depth.as<rs2::video_frame>().get_height();

            cv::Mat colorMat(cv::Size(w, h), CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depthMat(cv::Size(w, h), CV_8UC3, (void *)depth.apply_filter(color_map).get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_dist(cv::Size(w, h), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            Eigen::Vector3f normal_vector;

            auto Hoop = Hoop_Detect(colorMat, depthMat, depth_dist, threshold_value_black);

            cv::Mat Hoop_extract = std::get<0>(Hoop);
            cv::Point2f Hoop_center = std::get<1>(Hoop);

            int hoop_center_error = Hoop_Location(Hoop_extract, Hoop_center);

            this->Set_delta_x(hoop_center_error);

            this->Set_distance(distance_);

            cv::imshow(window_name, depthMat);
            cv::imshow(window_name_real_color, Hoop_extract);

            // cv::imshow(window_name, depthMat);
            // cv::imshow(window_name_color, colorMat);
        }
    }
    catch (const rs2::error &e)
    {
        std::cerr << "An error occurred during streaming: " << e.what() << std::endl;
    }
}

// ********************************************** GETTERS ************************************************** //

bool Img_proc::Get_img_proc_Far_Hoop_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_far_hoop_det_);
    return img_proc_far_hoop_det_;
}

bool Img_proc::Get_img_proc_Adjust_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_adjust_det_);
    return img_proc_adjust_det_;
}

bool Img_proc::Get_img_proc_Shoot_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_shoot_det_);
    return img_proc_shoot_det_;
}

bool Img_proc::Get_img_proc_No_Hoop_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_no_hoop_det_);
    return img_proc_no_hoop_det_;
}

bool Img_proc::Get_img_proc_stop_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_stop_det_);
    return img_proc_stop_det_;
}

double Img_proc::Get_delta_x() const
{
    std::lock_guard<std::mutex> lock(mtx_delta_x);
    return delta_x_;
}

double Img_proc::Get_distance() const
{
    std::lock_guard<std::mutex> lock(mtx_distance);
    return distance_;
}

int8_t Img_proc::Get_img_proc_Adjust_number() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_adjust_number_);
    return img_proc_adjust_number_;
}

double Img_proc::Get_gradient() const
{
    std::lock_guard<std::mutex> lock(mtx_gradient);
    return gradient_;
}

double Img_proc::Get_adjust_angle() const
{
    std::lock_guard<std::mutex> lock(mtx_adjust_angle);
    return adjust_angle_;
}

int Img_proc::Get_contain_adjust_to_foot() const
{
    std::lock_guard<std::mutex> lock(mtx_contain_adjust_to_foot);
    return contain_adjust_to_foot_;
}

// ********************************************** SETTERS ************************************************** //

void Img_proc::Set_img_proc_Far_Hoop_det(bool img_proc_far_hoop_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_far_hoop_det_);
    this->img_proc_far_hoop_det_ = img_proc_far_hoop_det;
}

void Img_proc::Set_img_proc_Adjust_det(bool img_proc_adjust_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_adjust_det_);
    this->img_proc_adjust_det_ = img_proc_adjust_det;
}

void Img_proc::Set_img_proc_Shoot_det(bool img_proc_shoot_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_shoot_det_);
    this->img_proc_shoot_det_ = img_proc_shoot_det;
}

void Img_proc::Set_img_proc_No_Hoop_det(bool img_proc_no_hoop_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_no_hoop_det_);
    this->img_proc_no_hoop_det_ = img_proc_no_hoop_det;
}

void Img_proc::Set_img_proc_stop_det(bool img_proc_stop_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_stop_det_);
    this->img_proc_stop_det_ = img_proc_stop_det;
}

void Img_proc::Set_delta_x(double delta_x)
{
    std::lock_guard<std::mutex> lock(mtx_delta_x);
    this->delta_x_ = delta_x;
}

void Img_proc::Set_distance(double distance)
{
    std::lock_guard<std::mutex> lock(mtx_distance);
    this->distance_ = distance;
}

void Img_proc::Set_img_proc_adjust_number(int8_t img_proc_adjust_number)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_adjust_number_);
    this->img_proc_adjust_number_ = img_proc_adjust_number;
}

void Img_proc::Set_gradient(double gradient)
{
    std::lock_guard<std::mutex> lock(mtx_gradient);
    this->gradient_ = gradient;
}

void Img_proc::Set_adjust_angle(double adjust_angle)
{
    std::lock_guard<std::mutex> lock(mtx_adjust_angle);
    this->adjust_angle_ = adjust_angle;
}

void Img_proc::Set_contain_adjust_to_foot(int contain_adjust_to_foot)
{
    std::lock_guard<std::mutex> lock(mtx_contain_adjust_to_foot);
    this->contain_adjust_to_foot_ = contain_adjust_to_foot;
}