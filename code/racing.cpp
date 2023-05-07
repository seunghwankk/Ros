#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

// ���� ���� ���ϱ�
void get_steering_direction(Mat& binary, double& direction)
{
    // Hough Transform�� �̿��Ͽ� ���� ����
    vector<Vec2f> lines;
    HoughLines(binary, lines, 1, CV_PI / 180, 50);

    // ����� ���� �� ���� �� ���� ����
    double max_length = 0;
    for (size_t i = 0; i < lines.size(); i++)
    {
        double length = lines[i][0];
        if (length > max_length)
        {
            max_length = length;
            direction = lines[i][1];
        }
    }

    // ������ [-pi/2, pi/2] ������ ����
    if (direction > CV_PI / 2)
    {
        direction -= CV_PI;
    }
    else if (direction < -CV_PI / 2)
    {
        direction += CV_PI;
    }
}
