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

double angle;

int main(int argc, char** argv) {
    Mat image = imread("level.jpg");
    if (image.empty()) {
        cout << "Could not open or find the image" << endl;
        return -1;
    }
    //ROI������ �����ϱ� ���� 4���� ����Ʈ ����
    Point pts[4];
    pts[0] = Point(0.2 * image.cols, 0.4 * image.rows);
    pts[1] = Point(0.7 * image.cols, 0.4 * image.rows);
    pts[2] = Point(0.7 * image.cols, 0.7 * image.rows);
    pts[3] = Point(0.2 * image.cols, 0.7 * image.rows);
    //HSV�̹����� ��ȯ -> �������� �߷����� ����
    Mat hsv_image;
    cvtColor(image, hsv_image, COLOR_BGR2HSV);
    //����ũ�� �����Ͽ� �������� ���� �� ROI���� �̹����� ����� ����ũ�� ���� -> ������ ���ܱⰡ ���̴� �κи� ����
    Mat mask;
    inRange(hsv_image, Scalar(0, 70, 50), Scalar(10, 255, 255), mask);
    Mat mask_roi = Mat::zeros(image.rows, image.cols, CV_8UC1);
    fillConvexPoly(mask_roi, pts, 4, Scalar(255, 0, 0));
    bitwise_and(mask, mask_roi, mask);
    //�������� ������ �簢���� ���ϰ� �� �簢���� �߽��� centers���Ϳ� ����
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Mat output = image.clone();
    vector<Point> centers;
    for (size_t i = 0; i < contours.size(); i++) {
        Rect rect = boundingRect(contours[i]);

        if (rect.width > 10 && rect.height > 10) {
            drawContours(output, contours, i, Scalar(0, 0, 255), 2);
            Point center(rect.x + rect.width / 2, rect.y + rect.height / 2);
            circle(output, center, 3, Scalar(0, 255, 0), -1);
            centers.push_back(center);
        }
    }

    // ���� ������ ���� ���� ������ ���� ����
    Point leftmost_center = centers[0];
    Point rightmost_center = centers[0];
    for (size_t i = 1; i < centers.size(); i++) {
        if (centers[i].x < leftmost_center.x) {
            leftmost_center = centers[i];
        }
        if (centers[i].x > rightmost_center.x) {
            rightmost_center = centers[i];
        }
    }

    // ���� �� ���� �ϳ��� ������ ���� -> ���ܱ��� �߽ɼ�
    line(output, leftmost_center, rightmost_center, Scalar(255, 0, 0), 2);

    // Calculate the angle of the line
    double dx = rightmost_center.x - leftmost_center.x;
    double dy = rightmost_center.y - leftmost_center.y;
    double angle = atan2(dy, dx) * 180 / CV_PI;

    // �߽ɼ��� ������ ���Ͽ� �������� �������� ����
    string shape;
    if (angle >= -10 && angle <= 10) {
        shape = "closed";
    }
    else if (angle >= 80 && angle <= 100) {
        shape = "open";
    }
    else {
        shape = "unknown";
    }

    // �̹����� ���ڿ��� ���
    Point text_position((leftmost_center.x + rightmost_center.x) / 2, leftmost_center.y - 10);
    putText(output, shape, text_position, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(170, 150, 255), 2);


    namedWindow("Output", WINDOW_NORMAL);
    imshow("Output", output);
    waitKey(0);

    return 0;
}