#ifndef LEAST_SQUARE_H
#define LEAST_SQUARE_H

#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;

//根据内点用最小二乘拟合出直线
void LeastSquare(const vector<Point2f> & inliners, vector<Point2f> & optimalPoints);

#endif