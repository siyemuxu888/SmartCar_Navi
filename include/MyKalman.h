#ifndef MYKALMAN_H
#define MYKALMAN_H
#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;

//1.kalman filter setup   
const int stateNum = 3;             //״̬����ά��
const int measureNum = 2;           //�۲�����ά�����ȼ������������һ��
const float floorWidth = 500.0f;    //500mm,�ذ�ĳߴ�
const float D = 147.0f;             //147mm,С����������֮��ľ���

//�������˲�����ʼ��
void KalmanInit(KalmanFilter & KF); 

//���ݸ�����ֱ���������ֱ����x��y��Ľؾ�
void SolveIntercept(const Point2f & startPoint, const Point2f & endPoint, vector<float> & result);

//����С�������������ӱ������Ĳ�ֵ���п������˲�����ʱ�����
void KalmanPredict(KalmanFilter & KF, float deltaS_l, float deltaS_r);

//�������˲����Ĳ�������
void KalmanCorrect(KalmanFilter & KF, const Mat& measurement);

#endif