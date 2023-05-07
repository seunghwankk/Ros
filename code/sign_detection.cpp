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
    // object �̹��� �ε�
    Mat object = imread("object.jpg", IMREAD_GRAYSCALE);

    // scene �̹��� �ε�
    Mat scene = imread("scene.jpg", IMREAD_GRAYSCALE);

    // ORB ��ü ����
    Ptr<ORB> orb = ORB::create(400, 1.2f, 8, 31, 2, 2, ORB::HARRIS_SCORE, 31, 20);

    // Ű����Ʈ �� ��ũ���� ���
    vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    orb->detectAndCompute(object, Mat(), keypoints_object, descriptors_object);
    orb->detectAndCompute(scene, Mat(), keypoints_scene, descriptors_scene);

    // ��Ī ��ü ����
    BFMatcher matcher(NORM_HAMMING);
    vector<vector<DMatch>> matches;
    matcher.knnMatch(descriptors_object, descriptors_scene, matches, 2); //�ֱ��� �̿� ��Ī

    // �̿� �Ÿ� ������ �̿��Ͽ� ��ȿ�� ��Ī ����Ʈ�� ����
    vector<DMatch> good_matches;
    for (int i = 0; i < matches.size(); i++) {
        //�߸��� ��Ī�̳� ������ ����
        if (matches[i][0].distance < 0.75 * matches[i][1].distance) {
            good_matches.push_back(matches[i][0]);
        }
    }

    // good_matches�� ���� ���� �̻��� �� object �̹����� scene �̹������� ������� ���
    if (good_matches.size() >= 8) {
        cout << "object.jpg is detected in scene.jpg" << endl;

        // ���� �̹��� ���
        Mat img_matches;
        drawMatches(object, keypoints_object, scene, keypoints_scene,
            good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
            vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        imshow("Detected Object", img_matches);
        waitKey(0);
    }

    return 0;
}