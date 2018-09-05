#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <opencv2/opencv.hpp>
#include "LeastSquare.h"

using namespace std;
using namespace cv;

//生成直线方程，返回直线的一般式方程：Ax＋By＋C = 0中的A，B，C
//start传入起点，end传入一个终点，由两点式给出直线方程
void GenerateLine(Point2f & start, Point2f & end, double * parameter)
{
	double k = double(start.y - end.y) / (start.x - end.x);   //注意要强制转换为double型，否则会变成整数除法
	double A = sqrt(1 - 1 / (1 + pow(k, 2)));
	double B = sqrt(1 - pow(A, 2));  
	if (k > 0)
		B = -B;     //因为A,B永远是正数,但当斜率为负时，A，B两个是异号的，所以B要求反
	double C = -A * start.x - B * start.y;
	parameter[0] = A;
	parameter[1] = B;
	parameter[2] = C;
}

//RANSAC算法，返回找到的最优直线上的两点坐标
//ordin传入一幅图片上所有亮点的坐标，即有效点;
//linePoint1保存RANSAC找到的直线中的一点，linePoint2保存RANSAC找到的直线中的另一点
void RansacMade(vector<Point2f> & ordin, Point2f & linePoint1, Point2f & linePoint2)
{
	srand(time(0));
	int iterTime = 100;      //设置最大迭代次数
	double distance = 5;     //设置内点到直线的最大距离,不一定要和FindInliner中的distance保持一致
	int preTotal = 0;        //储存符合拟合模型的点的个数
	int size = ordin.size(); //储存整张图片点的个数
	Point2f samp1, samp2;    //随机选取两个点用来拟合直线
	int k = 1;               //保存当前运行的迭代次数
	if (size != 0)           //有时图片可能是一张白图，即没有边缘，则可能ordin是空的，即一个点都没有，就没必要处理了。
	{
		double dist = 0;
		int total = 0;       //保存当前这次迭代内点数
		while (preTotal < (size * 2 / 3) && k < iterTime) //有2/3的点符合拟合模型或达到最大迭代次数就可以退出循环
		{
			samp1 = ordin[rand() % size];
			int count1 = 0;
			while (samp1 == Point2f(-1, -1))  //第一次RANSAC后会把内点坐标设置为(-1, -1)，方便第二次RANSAC
			{
				samp1 = ordin[rand() % size];
				++count1;
				if (count1 > 50)             //如果选了50次，还是上次的内点，则不要选择了，否则程序有可能卡在这。
					break;
			}
			int count2 = 0;                  //跟count1作用一样，防止下面选点进入死循环
			samp2 = ordin[rand() % size];
			while (samp1.x == samp2.x || samp2 == Point2f(-1, -1))    //选出x不一样的点，否则等下求斜率会无穷大;还有是不能选坐标为(-1,-1)的点
			{
				samp2 = ordin[rand() % size];  //直到选出的点不是同一点
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

			total = 0;   //每次重新迭代都要重新初始化这个变量
			double linePara[3];
			GenerateLine(samp1, samp2, linePara);
			for (int i = 0; i < size; ++i)
			{
				if (ordin[i] == Point2f(-1, -1))  //不要计算到(-1,-1)坐标距离。
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
			++k;   //更新迭代次数
		}
	}
}

//这个函数用来根据直线的两点linePoint1，linePoint2和设定的距离阈值求出该直线的所有内点
//ordin存放整张图片大于一定亮度值的像素的坐标；inliners存放输出的内点坐标
void FindInliner(vector<Point2f> & ordin, Point2f & linePoint1, Point2f & linePoint2, vector<Point2f> & inliners)
{
	double parameter[3] = { 0 };
	GenerateLine(linePoint1, linePoint2, parameter);
	double distance = 10;    //设定的距离阈值
	double  dist = 0;
	for (int i = 0; i < ordin.size(); ++i)
	{
		if (ordin[i] == Point2f(-1, -1))  //不要计算到(-1,-1)坐标距离。
			continue;
		dist = fabs(parameter[0] * ordin[i].x + parameter[1] * ordin[i].y + parameter[2]);
		if (dist < distance)
		{
			inliners.push_back(ordin[i]);
			ordin[i] = Point2f(-1, -1);
		}
	}
}

//把一幅图片的亮度值大于一定阈值后的点全部保存起来，要求传入一张灰度图
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
				points.push_back(Point2f(j, i)); //注意这里不能压反了，cols才是横坐标，rows才是纵坐标
			}
		}
	}
}

//随机清除一些噪点
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
	imshow("清噪点图", grayImage);
}

//清除车子的3个轮子的轮廓
void ClearWheelOutline(Mat grayImage)
{
	//circle(grayImage, Point(340, 0), 50, Scalar(0), -1, 8); //清万向轮轮廓
	circle(grayImage, Point(0, grayImage.rows), 180, Scalar(0), -1, 8);  //清左边电机轮廓
	circle(grayImage, Point(grayImage.cols, grayImage.rows), 180, Scalar(0), -1, 8); //清右边电机轮廓
}

//根据给定的点清轮廓
void ClearContour(Mat grayImage, const vector<Point2f>& contourPoints)
{
	for (int i = 0; i < contourPoints.size(); ++i)
	{
		circle(grayImage, contourPoints[i], 10, Scalar(0), -1, 8);
	}
}

//随机清除图片中的点
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

//RANSAC总的调用函数
//oriImage用来保存RANSAC和经过最小二乘处理后的效果图，caliPoints传入经过畸变矫正后的点，
//uvPoints指向一个数组，放经过RANSAC和最小二乘拟合后的两条直线的端点坐标
void CallRansac(Mat oriImage, vector<Point2f>& caliPoints, float * uvPoints)
{
	if (caliPoints.size() < 50)        //如果整幅图片的点小于50个点，则下面的语句就不要执行
	{
		imshow("RANSAC", oriImage);    //为了显示原来的窗口
		return;
	}
	/***********************第一次RANSAC***********************/
	Point2f x(0, 0), y(0, 0);
	RansacMade(caliPoints, x, y);  //x,y存放选出的直线中的两个点的坐标
	//double lineParameter[3];     //存储直线的参数
	//GenerateLine(x, y, lineParameter);
	//测试用显示所有内点

	vector<Point2f> inliners;
	FindInliner(caliPoints, x, y, inliners);  //在这后面对找到的内点采用最小二乘法
	vector<Point2f> optimalPoints;
	LeastSquare(inliners, optimalPoints);     //对内点进行最小二乘
	uvPoints[0] = optimalPoints[0].x;         //uvPoints指向的这个数组的存放规则是要所有的横坐标排在前面，再排纵坐标
	uvPoints[4] = optimalPoints[0].y;
	uvPoints[1] = optimalPoints[1].x;
	uvPoints[5] = optimalPoints[1].y;

	cout << "内点数：" << inliners.size() << endl;
	if (inliners.size() > 40)
	{
		//for (int i = 0; i < inliners.size(); ++i)
		//{
		//	circle(ran_image, inliners[i], 3, Scalar(255, 255, 255), 1, 8);
		//}
		line(oriImage, optimalPoints[0], optimalPoints[1], Scalar(255, 255, 255), 1, 8);
	}
	/*************************第二次RANSAC********************/
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
	cout << "内点数：" << inliners.size() << endl;
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