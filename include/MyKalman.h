#ifndef MYKALMAN_H
#define MYKALMAN_H
#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;

//1.kalman filter setup   
const int stateNum = 3;             //状态向量维数
const int measureNum = 2;           //观测向量维数，先假设跟输入向量一样
const float floorWidth = 500.0f;    //500mm,地板的尺寸
const float D = 147.0f;             //147mm,小车两个轮子之间的距离

//卡尔曼滤波器初始化
void KalmanInit(KalmanFilter & KF); 

//根据给出的直线两点计算直线在x和y轴的截距
void SolveIntercept(const Point2f & startPoint, const Point2f & endPoint, vector<float> & result);

//根据小车左右两个轮子编码器的差值进行卡尔曼滤波器的时间更新
void KalmanPredict(KalmanFilter & KF, float deltaS_l, float deltaS_r);

//卡尔曼滤波器的测量更新
void KalmanCorrect(KalmanFilter & KF, const Mat& measurement);

#endif