#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <opencv2/opencv.hpp>
#include "LeastSquare.h"

using namespace std;
using namespace cv;

//����ֱ�߷��̣�����ֱ�ߵ�һ��ʽ���̣�Ax��By��C = 0�е�A��B��C
//start������㣬end����һ���յ㣬������ʽ����ֱ�߷���
void GenerateLine(Point2f & start, Point2f & end, double * parameter)
{
	double k = double(start.y - end.y) / (start.x - end.x);   //ע��Ҫǿ��ת��Ϊdouble�ͣ����������������
	double A = sqrt(1 - 1 / (1 + pow(k, 2)));
	double B = sqrt(1 - pow(A, 2));  
	if (k > 0)
		B = -B;     //��ΪA,B��Զ������,����б��Ϊ��ʱ��A��B��������ŵģ�����BҪ��
	double C = -A * start.x - B * start.y;
	parameter[0] = A;
	parameter[1] = B;
	parameter[2] = C;
}

//RANSAC�㷨�������ҵ�������ֱ���ϵ���������
//ordin����һ��ͼƬ��������������꣬����Ч��;
//linePoint1����RANSAC�ҵ���ֱ���е�һ�㣬linePoint2����RANSAC�ҵ���ֱ���е���һ��
void RansacMade(vector<Point2f> & ordin, Point2f & linePoint1, Point2f & linePoint2)
{
	srand(time(0));
	int iterTime = 100;      //��������������
	double distance = 5;     //�����ڵ㵽ֱ�ߵ�������,��һ��Ҫ��FindInliner�е�distance����һ��
	int preTotal = 0;        //����������ģ�͵ĵ�ĸ���
	int size = ordin.size(); //��������ͼƬ��ĸ���
	Point2f samp1, samp2;    //���ѡȡ�������������ֱ��
	int k = 1;               //���浱ǰ���еĵ�������
	if (size != 0)           //��ʱͼƬ������һ�Ű�ͼ����û�б�Ե�������ordin�ǿյģ���һ���㶼û�У���û��Ҫ�����ˡ�
	{
		double dist = 0;
		int total = 0;       //���浱ǰ��ε����ڵ���
		while (preTotal < (size * 2 / 3) && k < iterTime) //��2/3�ĵ�������ģ�ͻ�ﵽ�����������Ϳ����˳�ѭ��
		{
			samp1 = ordin[rand() % size];
			int count1 = 0;
			while (samp1 == Point2f(-1, -1))  //��һ��RANSAC�����ڵ���������Ϊ(-1, -1)������ڶ���RANSAC
			{
				samp1 = ordin[rand() % size];
				++count1;
				if (count1 > 50)             //���ѡ��50�Σ������ϴε��ڵ㣬��Ҫѡ���ˣ���������п��ܿ����⡣
					break;
			}
			int count2 = 0;                  //��count1����һ������ֹ����ѡ�������ѭ��
			samp2 = ordin[rand() % size];
			while (samp1.x == samp2.x || samp2 == Point2f(-1, -1))    //ѡ��x��һ���ĵ㣬���������б�ʻ������;�����ǲ���ѡ����Ϊ(-1,-1)�ĵ�
			{
				samp2 = ordin[rand() % size];  //ֱ��ѡ���ĵ㲻��ͬһ��
				++count2;
				if (count2 > 50)
					break;
			}
			if (count1 > 50 || count2 > 50)
			{
				count1 = 0;
				count2 = 0;
				++k;
				continue;
			}

			total = 0;   //ÿ�����µ�����Ҫ���³�ʼ���������
			double linePara[3];
			GenerateLine(samp1, samp2, linePara);
			for (int i = 0; i < size; ++i)
			{
				if (ordin[i] == Point2f(-1, -1))  //��Ҫ���㵽(-1,-1)������롣
					continue;
				dist = fabs(linePara[0] * ordin[i].x + linePara[1] * ordin[i].y + linePara[2]);
				if (dist < distance)
				{
					total++;
				}
			}
			if (total > preTotal)
			{
				preTotal = total;
				linePoint1 = samp1;
				linePoint2 = samp2;
			}
			++k;   //���µ�������
		}
	}
}

//���������������ֱ�ߵ�����linePoint1��linePoint2���趨�ľ�����ֵ�����ֱ�ߵ������ڵ�
//ordin�������ͼƬ����һ������ֵ�����ص����ꣻinliners���������ڵ�����
void FindInliner(vector<Point2f> & ordin, Point2f & linePoint1, Point2f & linePoint2, vector<Point2f> & inliners)
{
	double parameter[3] = { 0 };
	GenerateLine(linePoint1, linePoint2, parameter);
	double distance = 10;    //�趨�ľ�����ֵ
	double  dist = 0;
	for (int i = 0; i < ordin.size(); ++i)
	{
		if (ordin[i] == Point2f(-1, -1))  //��Ҫ���㵽(-1,-1)������롣
			continue;
		dist = fabs(parameter[0] * ordin[i].x + parameter[1] * ordin[i].y + parameter[2]);
		if (dist < distance)
		{
			inliners.push_back(ordin[i]);
			ordin[i] = Point2f(-1, -1);
		}
	}
}

//��һ��ͼƬ������ֵ����һ����ֵ��ĵ�ȫ������������Ҫ����һ�ŻҶ�ͼ
void PickImgPoints(const Mat gray, vector<Point2f> & points)
{
	int rows = gray.rows;
	int cols = gray.cols;
	const uchar * data = nullptr;
	for (int i = 0; i < rows; i += 3)
	{
		data = gray.ptr<uchar>(i);
		for (int j = 0; j < cols; j += 3)
		{
			if (int(data[j]) > 127)
			{
				points.push_back(Point2f(j, i)); //ע�����ﲻ��ѹ���ˣ�cols���Ǻ����꣬rows����������
			}
		}
	}
}

//������һЩ���
void ClearNoise(vector<Point> & points, Mat grayImage)
{
	srand(time(0));
	int size = points.size();
	uchar * data = nullptr;
	int num = 0, count = 0 ;

	while (count < 50)
	{
		num = (rand() % size);
		if (points[num] == Point(-1, -1))
			continue;
		data = grayImage.ptr<uchar>(points[num].y);
		data[points[num].x] = 0;

		circle(grayImage, points[num], 10, Scalar(0, 0, 0), -1, 8);

		points[num] = Point(-1, -1);
		++count;
	}
	imshow("�����ͼ", grayImage);
}

//������ӵ�3�����ӵ�����
void ClearWheelOutline(Mat grayImage)
{
	//circle(grayImage, Point(340, 0), 50, Scalar(0), -1, 8); //������������
	circle(grayImage, Point(0, grayImage.rows), 180, Scalar(0), -1, 8);  //����ߵ������
	circle(grayImage, Point(grayImage.cols, grayImage.rows), 180, Scalar(0), -1, 8); //���ұߵ������
}

//���ݸ����ĵ�������
void ClearContour(Mat grayImage, const vector<Point2f>& contourPoints)
{
	for (int i = 0; i < contourPoints.size(); ++i)
	{
		circle(grayImage, contourPoints[i], 10, Scalar(0), -1, 8);
	}
}

//������ͼƬ�еĵ�
void RandomClear(Mat grayImage)
{
	int rows = grayImage.rows;
	int cols = grayImage.cols;
	srand(time(0)); 
	Point target;
	int x = 0, y = 0;
	for (int i = 0; i < 50; ++i)
	{
		x = rand() % cols;
		y = rand() % rows;
		target = Point(x, y);
		circle(grayImage, target, 25, Scalar(0), -1, 8);
	}
}

//RANSAC�ܵĵ��ú���
//oriImage��������RANSAC�;�����С���˴�����Ч��ͼ��caliPoints���뾭�����������ĵ㣬
//uvPointsָ��һ�����飬�ž���RANSAC����С������Ϻ������ֱ�ߵĶ˵�����
void CallRansac(Mat oriImage, vector<Point2f>& caliPoints, float * uvPoints)
{
	if (caliPoints.size() < 50)        //�������ͼƬ�ĵ�С��50���㣬����������Ͳ�Ҫִ��
	{
		imshow("RANSAC", oriImage);    //Ϊ����ʾԭ���Ĵ���
		return;
	}
	/***********************��һ��RANSAC***********************/
	Point2f x(0, 0), y(0, 0);
	RansacMade(caliPoints, x, y);  //x,y���ѡ����ֱ���е������������
	//double lineParameter[3];     //�洢ֱ�ߵĲ���
	//GenerateLine(x, y, lineParameter);
	//��������ʾ�����ڵ�

	vector<Point2f> inliners;
	FindInliner(caliPoints, x, y, inliners);  //���������ҵ����ڵ������С���˷�
	vector<Point2f> optimalPoints;
	LeastSquare(inliners, optimalPoints);     //���ڵ������С����
	uvPoints[0] = optimalPoints[0].x;         //uvPointsָ����������Ĵ�Ź�����Ҫ���еĺ���������ǰ�棬����������
	uvPoints[4] = optimalPoints[0].y;
	uvPoints[1] = optimalPoints[1].x;
	uvPoints[5] = optimalPoints[1].y;

	cout << "�ڵ�����" << inliners.size() << endl;
	if (inliners.size() > 40)
	{
		//for (int i = 0; i < inliners.size(); ++i)
		//{
		//	circle(ran_image, inliners[i], 3, Scalar(255, 255, 255), 1, 8);
		//}
		line(oriImage, optimalPoints[0], optimalPoints[1], Scalar(255, 255, 255), 1, 8);
	}
	/*************************�ڶ���RANSAC********************/
	x = y = Point(0, 0);
	inliners.clear();
	optimalPoints.clear();
	RansacMade(caliPoints, x, y);
	//GenerateLine(x, y, lineParameter);

	FindInliner(caliPoints, x, y, inliners);
	LeastSquare(inliners, optimalPoints);
	uvPoints[2] = optimalPoints[0].x;
	uvPoints[6] = optimalPoints[0].y;
	uvPoints[3] = optimalPoints[1].x;
	uvPoints[7] = optimalPoints[1].y;
	cout << "�ڵ�����" << inliners.size() << endl;
	if (inliners.size() > 40)
	{
		//for (int i = 0; i < inliners.size(); ++i)
		//{
		//	circle(ran_image, inliners[i], 3, Scalar(255, 255, 255), 1, 8);
		//}
		line(oriImage, optimalPoints[0], optimalPoints[1], Scalar(255, 255, 255), 1, 8);
	}
	imshow("RANSAC and Least Square", oriImage);
}