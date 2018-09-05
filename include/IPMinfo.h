#ifndef IPMINFO_H
#define IPMINFO_H
#include <opencv2/opencv.hpp>
using namespace cv;
typedef struct CameraInfo
{
	//焦距x和y
	CvPoint2D32f focalLength;
	//图像帧中的光学中心坐标（原点为（0,0）在左上角）
	CvPoint2D32f opticalCenter;
	//摄像机离地面的高度
	float cameraHeight;
	//俯仰角以弧度为单位（向下）
	float pitch;
	//偏航角以弧度表示 (顺时针)
	float yaw;
	//图像的宽度
	//int imageWidth;
	//图像的高度
	//int imageHeight;
}CameraInfo;

//摄像头内参矩阵
const Matx33d intrinsic_matrix =
{
	288.9274806169603, 0, 320.1509020795118,
	0, 289.9074502886161, 260.9787613714533,
	0, 0, 1
};
const Vec4d distortion_coeffs = { -0.0739035, 0.0518974, -0.0328777, 0.0039241 };//摄像头畸变系数

//逆透视变换函数
void TransformImage2Ground(const CvMat *inPoints, CvMat *outPoints, const CvMat * transMat);

//生成透视变换矩阵函数
float * CreateMatrix(const CameraInfo * cameraInfo);

//计算摄像头到两条直线的距离还有两条直线在摄像头坐标系下的倾角
void RealLineParameter(Point2f & start, Point2f & end, float * parameter);

#endif