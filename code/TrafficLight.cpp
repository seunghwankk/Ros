#include <iostream>
#include <map>
#include <vector>

#define OPENCV_470
//ROS��
//#include <ros/ros.h>
//#include <sensor_msgs/Image.h>
//#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#ifdef OPENCV_470
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#endif // OPENCV_470

#ifdef _DEBUG
#pragma comment(lib,"opencv_world470d.lib")
#else	//RELEASE
#pragma comment(lib,"opencv_world470.lib")
#endif

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // �̹��� ���� �б�
    Mat img = imread("traffic.jpg", IMREAD_COLOR);
    if (img.empty()) {
        std::cerr << "Failed to read image." << std::endl;
        return -1;
    }

    // ����ũ �� ���� ����
    Scalar red_lower(0, 100, 100);
    Scalar red_upper(10, 255, 255);
    Scalar green_lower(50, 100, 100);
    Scalar green_upper(100, 255, 255);
    Scalar yellow_lower(20, 100, 100);
    Scalar yellow_upper(40, 255, 255);

    // �̹��� ũ�� ����
    resize(img, img, Size(640, 480));

    // �̹����� HSV�� ��ȯ
    Mat hsv;
    cvtColor(img, hsv, COLOR_BGR2HSV);

    // �� ���� �ش��ϴ� ����ũ ����
    Mat red_mask, green_mask, yellow_mask;
    inRange(hsv, red_lower, red_upper, red_mask);
    inRange(hsv, green_lower, green_upper, green_mask);
    inRange(hsv, yellow_lower, yellow_upper, yellow_mask);

    // ����ũ�� ����Ͽ� �̹������� �� ���� �и�
    Mat red_img, green_img, yellow_img;
    bitwise_and(img, img, red_img, red_mask);
    bitwise_and(img, img, green_img, green_mask);
    bitwise_and(img, img, yellow_img, yellow_mask);

    // �и��� �̹��� ���
    imshow("Red", red_img);
    imshow("Green", green_img);
    imshow("Yellow", yellow_img);

    // Ű �Է� ���
    waitKey(0);

    return 0;
}