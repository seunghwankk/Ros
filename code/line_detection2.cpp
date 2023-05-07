//�� 4���� �̿��Ͽ� �簢�� �׸��� �Լ�
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