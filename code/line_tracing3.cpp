//�������� ���ϴ� �Լ�
Point IntersectionPoint1(Point p1, Point p2, Point p3, Point p4) { //�� ���� ������ ����Լ�
    Point ret;
    //�����Ǵ� ���� x ��ǥ �����
    ret.x = ((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p1.x - p2.x) * (p3.x * p4.y - p3.y * p4.x)) / ((p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x));

    //�����Ǵ� ���� y ��ǥ �����
    ret.y = ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x * p4.y - p3.y * p4.x)) / ((p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x));

    return ret;
}