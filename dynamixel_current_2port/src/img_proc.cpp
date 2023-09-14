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
std::tuple<cv::Mat, double, cv::Point> Img_proc::Hoop_Detect(cv::Mat color, cv::Mat depth, rs2::depth_frame depth_frame) {
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

  for (const auto &contour : contours) {
      double area = contourArea(contour);
      if (area > max_area) {
          max_area = area;
          largest_contour = contour;
      }
  }

  if (!largest_contour.empty() && max_area > 500) {
      cv::Moments m = cv::moments(largest_contour);
      if (m.m00 != 0) {
          center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
          cv::Mat singleContourMask = cv::Mat::zeros(thresh.size(), thresh.type());
          cv::drawContours(singleContourMask, std::vector<std::vector<cv::Point>>{largest_contour}, -1, cv::Scalar(255), cv::FILLED);

          cv::Mat singleExtractedObject;
          color.copyTo(singleExtractedObject, singleContourMask);
          singleExtractedObject.copyTo(extracted_object, singleContourMask);

          cv::circle(extracted_object, center, 1, cv::Scalar(0,255,0), 2);
          distance_hoop = depth_frame.get_distance(center.x, center.y);
          putText(extracted_object, "Distance: " + std::to_string(distance_hoop), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
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

    if(center.x >= x_left_bound && center.x <= x_right_bound &&
           center.y >= y_top_bound && center.y <= y_bottom_bound) {
            putText(color, "Inside", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            return 1;  // center가 영역 안에 있음을 나타내는 값 반환
        } else {
            putText(color, "Outside", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            return 0;  // center가 영역 밖에 있음을 나타내는 값 반환
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
            cv::Mat depth_dist(cv::Size(w, h), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            Eigen::Vector3f normal_vector;

            auto Hoop = Hoop_Detect(colorMat, depthMat, depth_frame);

            colorMat = std::get<0>(Hoop);

            double distance_ = std::get<1>(Hoop);

            this->Set_distance(distance_);

            cv::imshow(window_name, depthMat);
            cv::imshow(window_name_color, colorMat);

            // cv::imshow(window_name, depthMat);
            // cv::imshow(window_name_color, colorMat);
        }
    }
    catch (const rs2::error &e)
    {
        std::cerr << "An error occurred during streaming: " << e.what() << std::endl;
    }
}

// *************************************************************************  *****************************************************************************//

void Img_proc::RGB2HSV(const cv::Mat &rgb_image, cv::Mat &hsv_image)
{
    cv::cvtColor(rgb_image, hsv_image, cv::COLOR_BGR2HSV);
    morphologyEx(hsv_image, hsv_image, MORPH_OPEN, Mat(), Point(-1, -1), 1);
    morphologyEx(hsv_image, hsv_image, MORPH_ERODE, Mat(), Point(-1, -1), -5);
}

void Img_proc::RGB2LAB(const cv::Mat &rgb_image, cv::Mat &lab_image)
{
    cv::cvtColor(rgb_image, lab_image, cv::COLOR_BGR2Lab);
    morphologyEx(lab_image, lab_image, MORPH_OPEN, Mat(), Point(-1, -1), 1);
}

void Img_proc::saveParameters(const std::string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open configuration file for writing." << std::endl;
        return;
    }

    fs << "HSVParams"
       << "{"
       << "h_min" << static_cast<int>(h_min) << "h_max" << static_cast<int>(h_max)
       << "s_min" << static_cast<int>(s_min) << "s_max" << static_cast<int>(s_max)
       << "v_min" << static_cast<int>(v_min) << "v_max" << static_cast<int>(v_max) << "}";

    fs << "LABParams"
       << "{"
       << "l_min" << static_cast<int>(l_min) << "l_max" << static_cast<int>(l_max)
       << "a_min" << static_cast<int>(a_min) << "a_max" << static_cast<int>(a_max)
       << "b_min" << static_cast<int>(b_min) << "b_max" << static_cast<int>(b_max) << "}";

    fs.release();
}

void Img_proc::loadParameters(const std::string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open configuration file for reading." << std::endl;
        return;
    }

    cv::FileNode hsvParams = fs["HSVParams"];
    h_min = static_cast<int>(hsvParams["h_min"]);
    h_max = static_cast<int>(hsvParams["h_max"]);
    s_min = static_cast<int>(hsvParams["s_min"]);
    s_max = static_cast<int>(hsvParams["s_max"]);
    v_min = static_cast<int>(hsvParams["v_min"]);
    v_max = static_cast<int>(hsvParams["v_max"]);

    cv::FileNode labParams = fs["LABParams"];
    l_min = static_cast<int>(labParams["l_min"]);
    l_max = static_cast<int>(labParams["l_max"]);
    a_min = static_cast<int>(labParams["a_min"]);
    a_max = static_cast<int>(labParams["a_max"]);
    b_min = static_cast<int>(labParams["b_min"]);
    b_max = static_cast<int>(labParams["b_max"]);

    fs.release();
}

void Img_proc::running_process()
{
    vcap >> Origin_img; // Read a frame from the camera

    if (Origin_img.empty())
    {
        std::cerr << "Empty frame received from the camera!" << std::endl;
        return;
    }

    cv::Mat hsv_image, lab_image;
    RGB2HSV(Origin_img, hsv_image);
    RGB2LAB(Origin_img, lab_image);

    // Create binary masks for the specified color ranges in HSV and LAB color spaces
    cv::Mat hsv_binary_mask, lab_binary_mask;

    // Get current HSV and LAB threshold values from trackbars
    int h_min = cv::getTrackbarPos("H min", "Threshold Adjustments");
    int h_max = cv::getTrackbarPos("H max", "Threshold Adjustments");
    int s_min = cv::getTrackbarPos("S min", "Threshold Adjustments");
    int s_max = cv::getTrackbarPos("S max", "Threshold Adjustments");
    int v_min = cv::getTrackbarPos("V min", "Threshold Adjustments");
    int v_max = cv::getTrackbarPos("V max", "Threshold Adjustments");
    int l_min = cv::getTrackbarPos("L min", "Threshold Adjustments");
    int l_max = cv::getTrackbarPos("L max", "Threshold Adjustments");
    int a_min = cv::getTrackbarPos("A min", "Threshold Adjustments");
    int a_max = cv::getTrackbarPos("A max", "Threshold Adjustments");
    int b_min = cv::getTrackbarPos("B min", "Threshold Adjustments");
    int b_max = cv::getTrackbarPos("B max", "Threshold Adjustments");

    cv::Scalar hsv_lower_thresh(h_min, s_min, v_min);
    cv::Scalar hsv_upper_thresh(h_max, s_max, v_max);
    cv::Scalar lab_lower_thresh(l_min, a_min, b_min);
    cv::Scalar lab_upper_thresh(l_max, a_max, b_max);

    cv::inRange(hsv_image, hsv_lower_thresh, hsv_upper_thresh, hsv_binary_mask);
    cv::inRange(lab_image, lab_lower_thresh, lab_upper_thresh, lab_binary_mask);

    // Combine binary masks from both color spaces
    // cv::Mat final_binary_mask;
    Mat field;
    cv::bitwise_and(hsv_binary_mask, lab_binary_mask, final_binary_mask);

    // Apply morphological operations if needed (e.g., to remove noise)
    cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(final_binary_mask, final_binary_mask, cv::MORPH_OPEN, morph_kernel);

    field = final_binary_mask.clone();

    // Calculate object area
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> hull;
    std::vector<cv::Point> approxpoly;
    // cv::findContours(final_binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    cv::findContours(field, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    int _size = static_cast<int>(contours.size());
    int area_max = 0;
    int label_num = 0;

    if (_size > 0)
    {
        for (int i = 0; i < _size; i++)
        {
            int area = cvRound(cv::contourArea(contours[i]));
            area_max = std::max(area_max, area);
        }

        if (area_max >= 2000)
        {
            // ROS_WARN("img_proc_line_det : %d", Get_img_proc_line_det());
        }

        else if (500 < area_max < 2000)
        {
            // ROS_ERROR("img_proc_line_det : %d", Get_img_proc_line_det());
        }

        cv::convexHull(cv::Mat(contours[label_num]), hull, false);
        // cv::fillConvexPoly(Origin_img, hull, cv::Scalar(255), 8);
        cv::fillConvexPoly(field, hull, cv::Scalar(255), 8);

        cv::Scalar color(0, 0, 255); // Red color for text
        std::string area_text = "Area: " + std::to_string(area_max) + " pixels";

        // Draw the area information on the frame
        cv::putText(Origin_img, area_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, color, 2);
    }

    if (contours.empty())
    {
        // ROS_ERROR("img_proc_no_line_det : %d", Get_img_proc_no_line_det());
        // cout << "no area" << endl;
    }
    cv::morphologyEx(field, field, cv::MORPH_ERODE, cv::Mat(), cv::Point(-1, -1), 5);
    cv::bitwise_and(field, ~final_binary_mask, field);
    cv::morphologyEx(field, field, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2);

    // Show the original frame and the final binary mask
    cv::imshow("Original Frame", Origin_img);
    cv::imshow("Final Binary Mask", final_binary_mask);
}

void Img_proc::init()
{
    // // set node loop rate
    // ros::Rate loop_rate(SPIN_RATE);
    // // vcap = VideoCapture(CAP_DSHOW + webcam_id);
    // vcap = VideoCapture(webcam_id);
    // vcap.set(cv::CAP_PROP_FRAME_WIDTH, webcam_width);
    // vcap.set(cv::CAP_PROP_FRAME_HEIGHT, webcam_height);
    // vcap.set(cv::CAP_PROP_FPS, webcam_fps);

    // if (!vcap.isOpened())
    // {
    //     std::cerr << "Could not open the webcam\n";
    //     return;
    // }

    // cv::namedWindow("Threshold Adjustments", cv::WINDOW_NORMAL);
    // cv::createTrackbar("H min", "Threshold Adjustments", nullptr, 255);
    // cv::createTrackbar("H max", "Threshold Adjustments", nullptr, 255);
    // cv::createTrackbar("S min", "Threshold Adjustments", nullptr, 255);
    // cv::createTrackbar("S max", "Threshold Adjustments", nullptr, 255);
    // cv::createTrackbar("V min", "Threshold Adjustments", nullptr, 255);
    // cv::createTrackbar("V max", "Threshold Adjustments", nullptr, 255);
    // cv::createTrackbar("L min", "Threshold Adjustments", nullptr, 255);
    // cv::createTrackbar("L max", "Threshold Adjustments", nullptr, 255);
    // cv::createTrackbar("A min", "Threshold Adjustments", nullptr, 255);
    // cv::createTrackbar("A max", "Threshold Adjustments", nullptr, 255);
    // cv::createTrackbar("B min", "Threshold Adjustments", nullptr, 255);
    // cv::createTrackbar("B max", "Threshold Adjustments", nullptr, 255);

    // cv::setTrackbarPos("H min", "Threshold Adjustments", 77);
    // cv::setTrackbarPos("H max", "Threshold Adjustments", 235);

    // cv::setTrackbarPos("S min", "Threshold Adjustments", 131);
    // cv::setTrackbarPos("S max", "Threshold Adjustments", 214);

    // cv::setTrackbarPos("V min", "Threshold Adjustments", 60);
    // cv::setTrackbarPos("V max", "Threshold Adjustments", 156);

    // cv::setTrackbarPos("L min", "Threshold Adjustments", 16);
    // cv::setTrackbarPos("L max", "Threshold Adjustments", 151);

    // cv::setTrackbarPos("A min", "Threshold Adjustments", 115);
    // cv::setTrackbarPos("A max", "Threshold Adjustments", 177);

    // cv::setTrackbarPos("B min", "Threshold Adjustments", 66);
    // cv::setTrackbarPos("B max", "Threshold Adjustments", 173);
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

void Img_proc::Set_img_proc_wall_number(int8_t img_proc_adjust_number)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_adjust_number_);
    this->img_proc_adjust_number_ = img_proc_adjust_number;
}

void Img_proc::Set_gradient(double gradient)
{
    std::lock_guard<std::mutex> lock(mtx_gradient);
    this->gradient_ = gradient;
}
