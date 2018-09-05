#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

//�����ڵ�����С������ϳ�ֱ��
void LeastSquare(const vector<Point2f> & inliners, vector<Point2f> & optimalPoints)
{

	int size = inliners.size();    //��ȡ�ڵ�����

	CvMat *matX = cvCreateMat(1, size, CV_32FC1);     //ָ��洢�����ڵ��x���������
	CvMat *matY = cvCreateMat(1, size, CV_32FC1);     //ָ��洢�����ڵ��y���������
	CvMat *matTmp = cvCreateMat(1, size, CV_32FC1);   //�����洢�������ʱ���

	for (int i = 0; i < size; i++)
	{

		CV_MAT_ELEM(*matX, float, 0, i) = inliners[i].x;  //���������ڵ��x����
		CV_MAT_ELEM(*matY, float, 0, i) = inliners[i].y;  //���������ڵ��y����
	}

	float xSum = 0, xxSum = 0, ySum = 0, xySum = 0;

	cvMul(matX, matX, matTmp, 1);   //�õ�ÿ�����x*x���
	xxSum = cvSum(matTmp).val[0];   //�����е��x�����ƽ�����
	xSum = cvSum(matX).val[0];      //�����е��x��������
	ySum = cvSum(matY).val[0];      //�����е��y��������

	cvMul(matX, matY, matTmp, 1);   //�õ�ÿ�����x*y���
	xySum = cvSum(matTmp).val[0];   //�����е��x*y���

	float k, b, tmp = 0;
	tmp = xxSum * size - xSum * xSum;          //��С���˹�ʽ�ķ�ĸ
	k = (xySum * size - xSum * ySum) / tmp;    //�õ���ϵ�ֱ�ߵ�б��
	b = (xxSum * ySum - xySum * xSum) / tmp;   //�õ���ϵ�ֱ�ߵĽؾ�

	cout << "k = " << k << endl;
	cout << "b = " << b << endl;

	//������ϵ�ֱ�ߵ����ߵĶ˵�
	float y = k * inliners[0].x + b;
	optimalPoints.push_back(Point2f(inliners[0].x, y));
	y = k * inliners[size - 1].x + b;
	optimalPoints.push_back(Point2f(inliners[size - 1].x, y));  //�������Զ�������ڵ��������ʾ��ֱ��

	cvReleaseMat(&matX);
	cvReleaseMat(&matY);
	cvReleaseMat(&matTmp);
}