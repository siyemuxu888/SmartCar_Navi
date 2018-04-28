#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;

//1.kalman filter setup   
const int stateNum = 3;            //״̬����ά��
const int measureNum = 2;          //�۲�����ά�����ȼ������������һ��
const float floorWidth = 500.0f;   //500mm,�ذ�ĳߴ�
const float D = 147.0f;            //147mm,С����������֮��ľ���

void KalmanInit(KalmanFilter & KF);
void solveIntercept(const Point2f & startPoint, const Point2f & endPoint, vector<float> & result);
void KalmanPredict(KalmanFilter & KF, float deltaS_l, float deltaS_r);
void KalmanCorrect(KalmanFilter & KF, const Mat& measurement);