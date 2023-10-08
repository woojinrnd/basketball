#include <opencv2/opencv.hpp>
#include <cmath>

const int webcam_width = 640;
const int webcam_height = 480;
const int webcam_fps = 30;

// 트랙바 콜백 함수
void onTrackbar(int, void *) {}

int main()
{
    // 카메라 열기
    cv::VideoCapture cap(0); // 0번 카메라를 사용합니다. 다른 카메라를 사용하려면 인덱스를 조정하세요.
    cap.set(cv::CAP_PROP_FRAME_WIDTH, webcam_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, webcam_height);
    cap.set(cv::CAP_PROP_FPS, webcam_fps);

    if (!cap.isOpened())
    {
        std::cerr << "카메라를 열 수 없습니다." << std::endl;
        return -1;
    }

    // 윈도우 생성
    cv::namedWindow("Camera Feed");

    // 초기 HSV 범위 설정
    int hMin = 40, sMin = 40, vMin = 40;
    int hMax = 80, sMax = 255, vMax = 255;

    // 트랙바 생성
    cv::createTrackbar("Hue Min", "Camera Feed", &hMin, 179, onTrackbar);
    cv::createTrackbar("Saturation Min", "Camera Feed", &sMin, 255, onTrackbar);
    cv::createTrackbar("Value Min", "Camera Feed", &vMin, 255, onTrackbar);
    cv::createTrackbar("Hue Max", "Camera Feed", &hMax, 179, onTrackbar);
    cv::createTrackbar("Saturation Max", "Camera Feed", &sMax, 255, onTrackbar);
    cv::createTrackbar("Value Max", "Camera Feed", &vMax, 255, onTrackbar);

    while (true)
    {
        cv::Mat frame;
        cap >> frame; // 카메라에서 프레임 캡처

        // 프레임이 없을 경우 종료
        if (frame.empty())
        {
            std::cerr << "프레임을 받아올 수 없습니다." << std::endl;
            break;
        }

        // 이미지를 HSV 색 공간으로 변환
        cv::Mat hsvImage;
        cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);

        // HSV 범위 설정
        cv::Scalar lowerBound = cv::Scalar(hMin, sMin, vMin);
        cv::Scalar upperBound = cv::Scalar(hMax, sMax, vMax);

        // HSV 범위 내의 픽셀을 이진화
        cv::Mat binaryImage;
        cv::inRange(hsvImage, lowerBound, upperBound, binaryImage);

        // 이진화된 이미지에서 활꼴 찾기
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty())
        {
            // 가장 큰 활꼴을 선택
            double maxArea = 0;
            int maxIdx = -1;
            for (size_t i = 0; i < contours.size(); i++)
            {
                if (contours[i].size() < 5) // 5개 미만의 점을 가진 경계선은 무시
                    continue;
                double area = cv::contourArea(contours[i]);
                if (area > maxArea)
                {
                    maxArea = area;
                    maxIdx = i;
                }
            }

            if (maxIdx >= 0)
            {
                // 활꼴의 접선 방정식 계산
                cv::RotatedRect minEllipse = cv::fitEllipse(contours[maxIdx]);

                // 접선 방정식의 두 점 가져오기
                cv::Point2f points[2];
                minEllipse.points(points);
                cv::Point2f point1 = points[0];
                cv::Point2f point2 = points[1];

                // 기울기 계산
                float a = (point2.y - point1.y) / (point2.x - point1.x);

                // 기울기를 각도로 변환 (라디안에서도 동작)
                float angle = atan(a) * 180.0 / CV_PI;

                // 각도 출력
                std::cout << "각도: " << angle << "도" << std::endl;

                // 활꼴 그리기
                cv::drawContours(frame, contours, maxIdx, cv::Scalar(0, 255, 0), 2);
            }
        }
        else
        {
            std::cout << "활꼴을 찾을 수 없습니다." << std::endl;
        }

        // 프레임 표시
        cv::imshow("Camera Feed", frame);

        // 'q' 키를 누르면 루프 종료
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
