#ifndef IPMINFO_H
#define IPMINFO_H
#include <opencv2/opencv.hpp>
using namespace cv;
typedef struct CameraInfo
{
	//����x��y
	CvPoint2D32f focalLength;
	//ͼ��֡�еĹ�ѧ�������꣨ԭ��Ϊ��0,0�������Ͻǣ�
	CvPoint2D32f opticalCenter;
	//����������ĸ߶�
	float cameraHeight;
	//�������Ի���Ϊ��λ�����£�
	float pitch;
	//ƫ�����Ի��ȱ�ʾ (˳ʱ��)
	float yaw;
	//ͼ��Ŀ��
	//int imageWidth;
	//ͼ��ĸ߶�
	//int imageHeight;
}CameraInfo;

//����ͷ�ڲξ��������ϵ��
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