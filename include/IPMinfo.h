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

//摄像头内参矩阵跟畸变系数
static const Matx33d intrinsic_matrix =
{
	288.9274806169603, 0, 320.1509020795118,
	0, 289.9074502886161, 260.9787613714533,
	0, 0, 1
};
static const Vec4d distortion_coeffs = { -0.0739035, 0.0518974, -0.0328777, 0.0039241 };

void TransformImage2Ground(const CvMat *inPoints,
	CvMat *outPoints, const CvMat * transMat);

float * CreateMatrix(const CameraInfo * cameraInfo);
void RealLineParameter(Point2f & start, Point2f & end, float * parameter);

#endif