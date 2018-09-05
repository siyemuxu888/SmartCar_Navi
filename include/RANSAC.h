#ifndef RANSAC_H
#define RANSAC_H
#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;

//生成直线方程，返回直线的一般式方程：Ax＋By＋C = 0中的A，B，C
//start传入起点，end传入一个终点，由两点式给出直线方程
void GenerateLine(Point2f & start, Point2f & end, double * parameter);

//RANSAC算法，返回找到的最优直线上的两点坐标
//ordin传入一幅图片上所有亮点的坐标，即有效点;
//linePoint1保存RANSAC找到的直线中的一点，linePoint2保存RANSAC找到的直线中的另一点
void RansacMade(vector<Point2f> & ordin, Point2f & linePoint1, Point2f & linePoint2);

//这个函数用来根据直线的两点linePoint1，linePoint2和设定的距离阈值求出该直线的所有内点
//ordin存放整张图片大于一定亮度值的像素的坐标；inliners存放输出的内点坐标
void FindInliner(vector<Point2f> & ordin, Point2f & linePoint1, Point2f & linePoint2, vector<Point2f> & inliners);

//把一幅图片的亮度值大于一定阈值后的点全部保存起来，要求传入一张灰度图
void PickImgPoints(const Mat gray, vector<Point2f> & points);

//随机清除一些噪点
void ClearNoise(vector<Point> & points, Mat grayImage);

//清除车子的3个轮子的轮廓
void ClearWheelOutline(Mat grayImage);

//随机清除图片中的点
void RandomClear(Mat grayImage);

//RANSAC总的调用函数
//oriImage用来保存RANSAC和经过最小二乘处理后的效果图，caliPoints传入经过畸变矫正后的点，
//uvPoints指向一个数组，放经过RANSAC和最小二乘拟合后的两条直线的端点坐标
void CallRansac(Mat oriImage, vector<Point2f>& caliPoints, float * uvPoints);

//根据给定的点清轮廓
void ClearContour(Mat grayImage, const vector<Point2f>& contourPoints);

#endif