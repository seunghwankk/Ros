//�����Լ�
int main(void) {

    Mat img = imread("image.png", IMREAD_COLOR);

    //ROI���� ������ ����Ʈ
    Point pts[4];
    pts[0] = Point(0.48 * img.cols, 0.6 * img.rows);
    pts[1] = Point(0.6 * img.cols, 0.6 * img.rows);
    pts[2] = Point(0.85 * img.cols, 0.9 * img.rows);
    pts[3] = Point(0.2 * img.cols, 0.9 * img.rows);

    Mat img_gray = img.clone();
    cvtColor(img_gray, img_gray, COLOR_BGR2GRAY);
    // �̹��� ��ó�� (������� ��ȯ, ����þ� �� ����)
    Mat hsv, blurred;
    cvtColor(img, hsv, COLOR_BGR2HSV);
    GaussianBlur(hsv, blurred, Size(5, 5), 0);

    // ����� ������ ��� ������ ���ļ� ����ŷ ��� ����
    Mat mask = Mat::zeros(img.rows, img.cols, CV_8UC1);
    fillConvexPoly(mask, pts, 4, Scalar(255, 0, 0));

    Mat binary, binary_y, binary_w;
    inRange(blurred, Scalar(20, 100, 100), Scalar(30, 255, 255), binary_y); // ����� ����ũ
    bitwise_and(binary_y, mask, binary_y); // ROI ����ũ ����
    inRange(blurred, Scalar(0, 0, 200), Scalar(255, 30, 255), binary_w); // ��� ����ũ
    bitwise_and(binary_w, mask, binary_w); // ROI ����ũ ����
    // ����ŷ�� �̹��� ����
    bitwise_or(binary_w, binary_y, binary);
    Mat img_masked;
    bitwise_and(img_gray, binary, img_masked);

    vector<Vec2f> lines;
    HoughLines(img_masked, lines, 1, CV_PI / 180, 30, 30, 100);
    Mat HoughLineDetected_img(img_masked.rows, img_masked.cols, CV_8U, cv::Scalar(255));
    std::vector<cv::Vec2f>::const_iterator it = lines.begin();

    while (it != lines.end()) { // it(iterator)�ݺ���
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
    //line(img, aP1S, aP2S, cv::Scalar(0,0,255), 2); // ������ ��� ������ ���� �׸���
    //line(img, bP1S, bP2S, cv::Scalar(0,0,255), 2); //������ ��� ������ ���� �׸���

    Mat result = drivingArea(img, bP1S, aP1S, aP2S, bP2S);

    //ROS��
    //ros::init(argc, argv, "image_publisher");
    //// �̹����� ������ ROS ��� �ڵ� ����
    //ros::NodeHandle nh;
    //// ������ �̹��� ���� ���
    //std::string filename = "path/to/image.png";
    //// OpenCV�� ����Ͽ� �̹��� ���� �ε�
    //cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);
    //// cv_bridge�� ����Ͽ� OpenCV �̹����� ROS �޽����� ��ȯ
    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    //// �̹��� ������ ���� ROS publisher ����
    //ros::Publisher pub = nh.advertise<sensor_msgs::Image>("image_topic", 1);
    //// ������ �̹��� ����
    //ros::Rate loop_rate(10);
    //while (ros::ok()){
    //    // �̹��� ����
    //    pub.publish(msg);
    //    // ���� �ӵ� ����
    //    loop_rate.sleep();
    //}

    return 0;
}