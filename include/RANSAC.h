#ifndef RANSAC_H
#define RANSAC_H
#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;

void GenerateLine(Point2f & start, Point2f & end, double * parameter);
void RANSAC_MADE(vector<Point2f> & ordin, Point2f & best_line_start, Point2f & best_line_end);
void FindInliner(vector<Point2f> & ordin, Point2f & best_line_start, Point2f & best_line_end, vector<Point2f> & inliners);
void DrawPoint(const Mat gray, vector<Point2f> & points);
void ClearNoise(vector<Point> & points, Mat grayImage);
void ClearWheelOutline(Mat grayImage);
void RandomClear(Mat grayImage);
void Call_RANSAC(Mat ran_image, vector<Point2f>& cali_points, float * uvPoints);
void ClearContour(Mat grayImage, const vector<Point2f> contourPoints);

#endif