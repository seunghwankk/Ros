#include <ros/ros.h> // ROS ���̺귯���� ����ϱ� ���� ��� ����
#include <image_transport/image_transport.h> // �̹��� �ۺ��� �� ������ ���� ��� ����
#include <cv_bridge/cv_bridge.h> // OpenCV�� ROS �̹��� ���� ��ȯ�� ���� ��� ����
#include <opencv2/highgui/highgui.hpp> // OpenCV GUI�� ���� ��� ����
#include <opencv2/imgproc/imgproc.hpp> // OpenCV ���� ó���� ���� ��� ����

#define ArraySize 100
#define PI 3.141592


using namespace cv;
using namespace std;

static const string OPENCV_WINDOW = "Image window"; // OpenCV â�� �̸�
Point* aPoint1 = new Point[ArraySize], * aPoint2 = new Point[ArraySize], * bPoint1 = new Point[ArraySize], * bPoint2 = new Point[ArraySize];
int aIndex = 0, bIndex = 0;

class LaneDetector
{
    ros::NodeHandle nh_; // ROS ��� �ڵ鷯
    image_transport::ImageTransport it_; // �̹��� ���� Ŭ����
    image_transport::Subscriber image_sub_; // �̹��� ������
    image_transport::Publisher image_pub_; // �̹��� ������

public:
    LaneDetector()
        : it_(nh_) // ������ �ʱ�ȭ ����� ����Ͽ� ��� �ڵ鷯 ��ü�� �̹��� ���� ��ü �ʱ�ȭ
    {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/jetbot_camera/raw", 1, // "/camera/image_raw" �������κ��� ���� ����
            &LaneDetector::imageCb, this); // ���� �ݹ� �Լ� ���
        image_pub_ = it_.advertise("/lane_detection/output_video", 1); // "/lane_detection/output_video" �������� �̹��� ����

        cv::namedWindow(OPENCV_WINDOW); // OpenCV â ����
    }

    ~LaneDetector()

        cv::destroyWindow(OPENCV_WINDOW); // �Ҹ��ڿ��� OpenCV â ����
}

Point IntersectionPoint1(Point p1, Point p2, Point p3, Point p4) { //�� ���� ������ ����Լ�
    Point ret;
    //�����Ǵ� ���� x ��ǥ �����
    ret.x = ((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p1.x - p2.x) * (p3.x * p4.y - p3.y * p4.x)) / ((p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x));

    //�����Ǵ� ���� y ��ǥ �����
    ret.y = ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x * p4.y - p3.y * p4.x)) / ((p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x));

    return ret;
}

float get_radian(float Val) //���� ����Ͽ� ��ȯ
{
    return PI * Val / 180;  //Val ���� ���� ������ ��ȯ
}

Mat drivingArea(Mat image, Point pt1, Point pt2, Point pt3, Point pt4)
{
    Point pty = IntersectionPoint1(pt2, pt3, pt1, pt4) + Point(0, 20); //��� ������ �߾����� �ణ �Ʒ��� ��ǥ�� ����
    Point a1 = pty + Point(200, 0); //x��� ������ ������ ����� ���� ���� ����
    Point a2 = pty + Point(-200, 0);
    Point Pt23Fix = IntersectionPoint1(a1, a2, pt1, pt4);  //���� ��� ������ ��ٸ��� ���¸� ����� ���� ���� ������ �������� ����
    Point Pt14Fix = IntersectionPoint1(pt2, pt3, a1, a2);
    Point points[4] = { pt3,pt4,Pt23Fix ,Pt14Fix }; // �� ���� ��ǥ�� �Լ��� �����ϱ����� ����
    const Point* ppt[1] = { points };

    int npt[] = { 4 };
    Mat result = image;
    Mat img_mask(image.size(), CV_8UC3, cv::Scalar(255, 255, 255)); //�Ͼ������ ����
    fillPoly(img_mask, ppt, npt, 1, Scalar(0, 255, 0), LINE_8); //�ʷϻ����� ��ٸ��� ��ĥ
    addWeighted(result, 0.8, img_mask, 0.2, 0, result); //  ���� �׸��� ��ٸ��� ������ �̿��� ����ġ�� �ξ� �׸��� ��ħ
    line(result, Pt14Fix, pt3, Scalar(255, 0, 0), 2, LINE_8); // ��ٸ����� ���κп� ���� ���� �׾���
    line(result, Pt23Fix, pt4, Scalar(0, 0, 255), 2, LINE_8);

    return result; //���� ��� �׸� ��ȯ
}

void imageCb(const sensor_msgs::ImageConstPtr& msg) // �̹��� ���� �ݹ� �Լ�
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // ROS �޽����� OpenCV �������� ��ȯ
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what()); // ���� ó��
        return;
    }

    cv::Mat src_img = cv_ptr->image; // OpenCV ���� �̹����κ��� ���� �̹��� ȹ��

    //ROI���� ������ ����Ʈ
    Point pts[4];
    pts[0] = Point(0.48 * src_img.cols, 0.6 * src_img.rows);
    pts[1] = Point(0.6 * src_img.cols, 0.6 * src_img.rows);
    pts[2] = Point(0.85 * src_img.cols, 0.9 * src_img.rows);
    pts[3] = Point(0.2 * src_img.cols, 0.9 * src_img.rows);

    // Convert to grayscale
    cv::Mat gray_img;
    cv::cvtColor(src_img, gray_img, cv::COLOR_BGR2GRAY); // ���� �̹����� �׷��̽����� �������� ��ȯ

    // Apply Gaussian blur to smooth the image and reduce noise
    cv::Mat blurred_img, hsv;
    cvtColor(src_img, hsv, COLOR_BGR2HSV);
    cv::GaussianBlur(gray_img, blurred_img, cv::Size(5, 5), 0); // ����þ� ���� �����Ͽ� �̹����� �ε巴�� ����

    Mat mask = Mat::zeros(src_img.rows, src_img.cols, CV_8UC1);
    fillConvexPoly(mask, pts, 4, Scalar(255, 0, 0));

    Mat binary, binary_y, binary_w;
    inRange(blurred_img, Scalar(20, 100, 100), Scalar(30, 255, 255), binary_y); // ����� ����ũ
    bitwise_and(binary_y, mask, binary_y); // ROI ����ũ ����
    inRange(blurred_img, Scalar(0, 0, 200), Scalar(255, 30, 255), binary_w); // ��� ����ũ
    bitwise_and(binary_w, mask, binary_w); // ROI ����ũ ����	
    // ����ŷ�� �̹��� ����
    bitwise_or(binary_w, binary_y, binary);
    Mat img_masked;
    bitwise_and(gray_img, binary, img_masked);

    // Detect lines using Hough transform
    vector<Vec2f> lines;
    HoughLines(img_masked, lines, 1, CV_PI / 180, 30, 30, 100);
    Mat HoughLineDetected_img(img_masked.rows, img_masked.cols, CV_8U, cv::Scalar(255));
    vector<Vec2f>::const_iterator it = lines.begin();

    while (it != lines.end()) {
        float rho = (*it)[0];   // ù ��° ��Ҵ� rho �Ÿ�
        float theta = (*it)[1]; // �� ��° ��Ҵ� ��Ÿ ����
        if (theta < get_radian(90) && theta >get_radian(35)) { // ���� ���� ���� 35 ~ 85 ���� ������ ����
            Point pt1(rho / cos(theta), 0); // ù ���������� �ش� ���� ������   
            aPoint1[(aIndex) % ArraySize] = pt1; //��ġ Queue�� ���ư��� ������ �ȿ����� ���� ������� ä��
            Point pt2((rho - HoughLineDetected_img.rows * sin(theta)) / cos(theta), HoughLineDetected_img.rows);
            // ������ ���������� �ش� ���� ������
            aPoint2[(aIndex++) % ArraySize] = pt2; // ���� �ε��� ������ �Ѿ�� ���� ����
            //line(img, pt1, pt2, cv::Scalar(255), 2); // ��� ���� �׸���
        }
        else if (theta<get_radian(145) && theta>get_radian(90)) { // ���� ���� ���� 95 ~ 145 ���� ������ ����
            cv::Point pt1(0, rho / sin(theta)); // ù ���������� �ش� ���� ������  
            bPoint1[(bIndex) % ArraySize] = pt1; //��ġ Queue�� ���ư��� ������ �ȿ����� ���� ������� ä��
            Point pt2(HoughLineDetected_img.cols, (rho - HoughLineDetected_img.cols * cos(theta)) / sin(theta));
            // ������ ���������� �ش� ���� ������
            bPoint2[(bIndex++) % ArraySize] = pt2; // ���� �ε��� ������ �Ѿ�� ���� ����
            //line(img, pt1, pt2, cv::Scalar(255), 2); // ��� ���� �׸���
        }
        ++it; //�ݺ��� ����
    }
    //--    ��� ���� ���� ---//
    Point aP1S(0, 0), aP2S(0, 0), bP1S(0, 0), bP2S(0, 0); //�� ������ ��հ��� ��� ���� ��
    int a1cnt = 0, a2cnt = 0, b1cnt = 0, b2cnt = 0;
    for (int i = 0; i < ArraySize; i++) {//�迭�� ����� ��� ���� ��ǥ ���� ����
        aP1S += aPoint1[i];
        aP2S += aPoint2[i];
        bP1S += bPoint1[i];
        bP2S += bPoint2[i];
        if (aPoint1[i] != Point(0, 0)) {
            a1cnt++;
        }
        if (aPoint2[i] != Point(0, 0)) {
            a2cnt++;
        }
        if (bPoint1[i] != Point(0, 0)) {
            b1cnt++;
        }
        if (bPoint2[i] != Point(0, 0)) {
            b2cnt++;
        }
    }
    aP1S /= a1cnt;//�迭�� ����� ��� ���� ��հ��� ����
    aP2S /= a2cnt;
    bP1S /= b1cnt;
    bP2S /= b2cnt;

    Mat result = drivingArea(img, bP1S, aP1S, aP2S, bP2S);

    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg(); // output_img�� ROS Image �޽����� ��ȯ
    image_pub_.publish(output_msg); // ��ȯ�� �̹����� topic���� publish

    cv::imshow(OPENCV_WINDOW, result); // �̹��� ���
    cv::waitKey(1); // imshow������ �Է��� ��ٸ��� ���� ��� �Լ� (1ms)
}
};

int main(int argc, char** argv)
{
    // "lane_detector"�� �� ����� �̸����� ���ȴ�.
    ros::init(argc, argv, "lane_detector");
    // LaneDetector Ŭ������ �ν��Ͻ��� �����.
    LaneDetector ld;
    // ROS �ý��ۿ��� �޽����� �ް�, �����ϸ�, �ݹ��� ȣ���Ѵ�.
    ros::spin();
    // ���α׷��� ���������� ������� ��Ÿ���� 0�� ��ȯ�Ѵ�.
    return 0;
}