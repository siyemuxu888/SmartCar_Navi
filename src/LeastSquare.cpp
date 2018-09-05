#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

//根据内点用最小二乘拟合出直线
void LeastSquare(const vector<Point2f> & inliners, vector<Point2f> & optimalPoints)
{

	int size = inliners.size();    //获取内点数量

	CvMat *matX = cvCreateMat(1, size, CV_32FC1);     //指向存储所有内点的x坐标的数组
	CvMat *matY = cvCreateMat(1, size, CV_32FC1);     //指向存储所有内点的y坐标的数组
	CvMat *matTmp = cvCreateMat(1, size, CV_32FC1);   //用来存储运算的临时结果

	for (int i = 0; i < size; i++)
	{

		CV_MAT_ELEM(*matX, float, 0, i) = inliners[i].x;  //存入所有内点的x坐标
		CV_MAT_ELEM(*matY, float, 0, i) = inliners[i].y;  //存入所有内点的y坐标
	}

	float xSum = 0, xxSum = 0, ySum = 0, xySum = 0;

	cvMul(matX, matX, matTmp, 1);   //得到每个点的x*x结果
	xxSum = cvSum(matTmp).val[0];   //对所有点的x坐标的平方求和
	xSum = cvSum(matX).val[0];      //对所有点的x坐标的求和
	ySum = cvSum(matY).val[0];      //对所有点的y坐标的求和

	cvMul(matX, matY, matTmp, 1);   //得到每个点的x*y结果
	xySum = cvSum(matTmp).val[0];   //对所有点的x*y求和

	float k, b, tmp = 0;
	tmp = xxSum * size - xSum * xSum;          //最小二乘公式的分母
	k = (xySum * size - xSum * ySum) / tmp;    //得到拟合的直线的斜率
	b = (xxSum * ySum - xySum * xSum) / tmp;   //得到拟合的直线的截距

	cout << "k = " << k << endl;
	cout << "b = " << b << endl;

	//保存拟合的直线的两边的端点
	float y = k * inliners[0].x + b;
	optimalPoints.push_back(Point2f(inliners[0].x, y));
	y = k * inliners[size - 1].x + b;
	optimalPoints.push_back(Point2f(inliners[size - 1].x, y));  //把相距最远的两个内点存入来显示出直线

	cvReleaseMat(&matX);
	cvReleaseMat(&matY);
	cvReleaseMat(&matTmp);
}