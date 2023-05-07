//�̹��� �ݹ� �Լ� - �̹��� �޽����� ���� ������ ����
void imageCallback(const sensor_msgs::ImageConstPtr& msg, ros::Publisher& pub)
{
	try
	{
		// ROS �޽����� OpenCV �̹����� ��ȯ
		Mat img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

		// �̹��� ��ó�� (������� ��ȯ, ����þ� �� ����)
		Mat hsv, blurred;
		cvtColor(img, hsv, COLOR_BGR2HSV);
		GaussianBlur(hsv, blurred, Size(5, 5), 0);

		// ROI ���� ����
		Mat mask = Mat::zeros(img.size(), CV_8UC1);
		Point roi_points[4] = { Point(0, img.rows), Point(img.cols, img.rows), Point(img.cols * 3 / 4, img.rows / 2),Point(img.cols / 4, img.rows / 2) };
		fillConvexPoly(mask, roi_points, 4, Scalar(255, 255, 255));

		// ROI ������ ������ �κ� ����
		Mat masked;
		bitwise_and(blurred, mask, masked);

		// ����ȭ
		Mat binary;
		inRange(masked, Scalar(0, 0, 200), Scalar(255, 100, 255), binary);

		// ���� ���� ���ϱ�
		double direction;
		get_steering_direction(binary, direction);

		// ���� ���⿡ ���� ��Ʋ�� ��� ����
		geometry_msgs::Twist twist;
		twist.linear.x = 0.15;   // ���ӵ� = �κ� �ӵ�
		twist.angular.z = direction * 0.01;  // ���ӵ� = �κ� ȸ�� ����

		// ��Ʋ�� ��� ����
		pub.publish(twist);

		// ����׿� �̹��� ǥ��
		imshow("binary", binary);
		waitKey(1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}