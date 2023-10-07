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

    // const std::string window_name1 = "hsv Frame_white";
    const std::string window_name2 = "thresh Frame_white";
    // const std::string window_name3 = "hsv Frame_yellow";
    const std::string window_name4 = "thresh Frame_yellow";
    // cv::namedWindow(window_name1);
    cv::namedWindow(window_name2);
    // cv::namedWindow(window_name3);
    cv::namedWindow(window_name4);

    // create_color_range_trackbar(window_name1);
    create_threshold_trackbar_W(window_name2);
    // create_color_range_trackbar(window_name3);
    create_threshold_trackbar_Y(window_name4);

    cv::Mat frame, hsv_frame_white, hsv_frame_yellow, thresh_frame_white, thresh_frame_yellow, gray;

    while (ros::ok())
    {
        cap >> frame;
        if (frame.empty())
            break;

        if (cv::waitKey(1) == 27)
            break;
        // loop_rate.sleep();
    }

    vcap.release();
    cv::destroyAllWindows();
}

// // ********************************************** 3D THREAD************************************************** //

std::tuple<cv::Mat, double, cv::Point> Img_proc::Hoop_Detect(cv::Mat color, cv::Mat depth, rs2::depth_frame depth_frame)
{
    cv::Mat gray, thresh, extracted_object;

    double distance_hoop = 0;

    cv::cvtColor(depth, gray, cv::COLOR_BGR2GRAY);
    int pos = cv::getTrackbarPos("pos", "Distance Measurement");
    cv::threshold(gray, thresh, pos, 255, cv::THRESH_BINARY);

    extracted_object = cv::Mat::zeros(color.size(), color.type());

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double max_area = 0;
    cv::Point center;
    std::vector<cv::Point> largest_contour;

    for (const auto &contour : contours)
    {
        double area = contourArea(contour);
        if (area > max_area)
        {
            max_area = area;
            largest_contour = contour;
        }
    }

    if (!largest_contour.empty() && max_area > 500)
    {
        cv::Moments m = cv::moments(largest_contour);
        if (m.m00 != 0)
        {
            center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
            cv::Mat singleContourMask = cv::Mat::zeros(thresh.size(), thresh.type());
            cv::drawContours(singleContourMask, std::vector<std::vector<cv::Point>>{largest_contour}, -1, cv::Scalar(255), cv::FILLED);

            cv::Mat singleExtractedObject;
            color.copyTo(singleExtractedObject, singleContourMask);
            singleExtractedObject.copyTo(extracted_object, singleContourMask);

            cv::circle(extracted_object, center, 1, cv::Scalar(0, 255, 0), 2);
            distance_hoop = depth_frame.get_distance(center.x, center.y);
            putText(extracted_object, "Distance: " + std::to_string(distance_hoop), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }
    }
    cv::cvtColor(depth, gray, cv::COLOR_BGR2GRAY);
    int pos2 = cv::getTrackbarPos("pos", "Distance Measurement");
    cv::threshold(gray, thresh, pos, 255, cv::THRESH_BINARY);

    return std::make_tuple(extracted_object, distance_hoop, center);
}

int Img_proc::Hoop_Location(cv::Mat &color, cv::Point center)
{
    int x_left_bound = 424;
    int x_right_bound = 500;
    int y_top_bound = 10;
    int y_bottom_bound = 470;

    rectangle(color, cv::Point(x_left_bound, y_top_bound), cv::Point(x_right_bound, y_bottom_bound), cv::Scalar(255, 255, 0), 2);

    if (center.x >= x_left_bound && center.x <= x_right_bound &&
        center.y >= y_top_bound && center.y <= y_bottom_bound)
    {
        putText(color, "Inside", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        return 1; // center가 영역 안에 있음을 나타내는 값 반환
    }
    else
    {
        putText(color, "Outside", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        return 0; // center가 영역 밖에 있음을 나타내는 값 반환
    }
}

void Img_proc::realsense_thread()
{
    rs2::colorizer color_map;
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, realsense_width, realsense_height, RS2_FORMAT_BGR8, realsense_fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, realsense_width, realsense_height, RS2_FORMAT_Z16, realsense_fps);

    try
    {
        pipe.start(cfg);
    }
    catch (const rs2::error &e)
    {
        std::cerr << "Failed to open the RealSense camera: " << e.what() << std::endl;
        return;
    }

    const auto window_name = "Realsense Depth Frame";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    const auto window_name_color = "Realsense Color Frame";
    cv::namedWindow(window_name_color, cv::WINDOW_AUTOSIZE);

    try
    {
        while (ros::ok() && cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
        {
            rs2::frameset data = pipe.wait_for_frames();

            rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
            rs2::frame color = data.get_color_frame();
            rs2::depth_frame depth_frame = data.get_depth_frame();

            color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);

            float depth_scale = pipe.get_active_profile().get_device().first<rs2::depth_sensor>().get_depth_scale();
            rs2_intrinsics intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

            const int w = depth.as<rs2::video_frame>().get_width();
            const int h = depth.as<rs2::video_frame>().get_height();

            cv::Mat colorMat(cv::Size(w, h), CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depthMat(cv::Size(w, h), CV_8UC3, (void *)depth.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_dist(cv::Size(w, h), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            Eigen::Vector3f normal_vector;

            auto Hoop = Hoop_Detect(colorMat, depthMat, depth_frame);

            colorMat = std::get<0>(Hoop);

            double distance_ = std::get<1>(Hoop);

            this->Set_distance(distance_);

            cv::imshow(window_name, depthMat);
            cv::imshow(window_name_color, colorMat);
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

