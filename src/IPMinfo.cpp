#include <opencv2/opencv.hpp>
#include <iostream>
#include "IPMinfo.h"
using namespace cv;

#define PI 3.14159

//创建变换矩阵
float * CreateMatrix(const CameraInfo * cameraInfo)
{
	//创建变换矩阵
	float c1 = cos(cameraInfo->pitch * PI / 180);  //转化为弧度
	float s1 = sin(cameraInfo->pitch * PI / 180);
	float c2 = cos(cameraInfo->yaw * PI / 180);
	float s2 = sin(cameraInfo->yaw * PI / 180);
	static float matp[] = {
		-cameraInfo->cameraHeight*c2 / cameraInfo->focalLength.x,
		cameraInfo->cameraHeight*s1*s2 / cameraInfo->focalLength.y,
		(cameraInfo->cameraHeight*c2*cameraInfo->opticalCenter.x /
		cameraInfo->focalLength.x) -
		(cameraInfo->cameraHeight *s1*s2* cameraInfo->opticalCenter.y /
		cameraInfo->focalLength.y) - cameraInfo->cameraHeight *c1*s2,

		cameraInfo->cameraHeight *s2 / cameraInfo->focalLength.x,
		cameraInfo->cameraHeight *s1*c2 / cameraInfo->focalLength.y,
		(-cameraInfo->cameraHeight *s2* cameraInfo->opticalCenter.x
		/ cameraInfo->focalLength.x) - (cameraInfo->cameraHeight *s1*c2*
		cameraInfo->opticalCenter.y / cameraInfo->focalLength.y) -
		cameraInfo->cameraHeight *c1*c2,

		0,
		cameraInfo->cameraHeight *c1 / cameraInfo->focalLength.y,
		(-cameraInfo->cameraHeight *c1* cameraInfo->opticalCenter.y /
		cameraInfo->focalLength.y) + cameraInfo->cameraHeight *s1,

		0,
		-c1 / cameraInfo->focalLength.y,
		(c1* cameraInfo->opticalCenter.y / cameraInfo->focalLength.y) - s1,
	};

	return matp;
}

//从每一帧图像中的点（uv坐标）变换到地平面上的真实的世界坐标框架上
//参数inpoint为图像的输入点，参数outpoint为图像的输出点，transMat传入IPM变换矩阵
void TransformImage2Ground(const CvMat *inPoints, CvMat *outPoints, const CvMat * transMat)
{	
	//向输入点矩阵中加入前2行
	CvMat *inPoints4 = cvCreateMat(inPoints->rows + 2, inPoints->cols, cvGetElemType(inPoints));

	//将inpoints复制到前2行中去
	CvMat inPoints2, inPoints3, inPointsr4, inPointsr3;
	cvGetRows(inPoints4, &inPoints2, 0, 2);  //将inPoints2指向inPoints4的前2行
	cvGetRows(inPoints4, &inPoints3, 0, 3);  //将inPoints3指向inPoints4的前3行
	cvGetRow(inPoints4, &inPointsr3, 2);     //将inPointsr3指向inPoints4中第3行
	cvGetRow(inPoints4, &inPointsr4, 3);     //将inPointsr4指向inPoints4中第4行
	cvSet(&inPointsr3, cvRealScalar(1));     //相当于将inPoints4中的第3行全置为1
	cvCopy(inPoints, &inPoints2);            //相当于将输入的点复制到inPoints4中的前2行
	
	cvMatMul(transMat, &inPoints3, inPoints4); //IPM变换矩阵跟inPoints3相乘，结果放到inPoints4

	//下面是除以inPoints4矩阵的最后一行
	for (int i = 0; i< inPoints->cols; i++)
	{
		float div = CV_MAT_ELEM(inPointsr4, float, 0, i);  //得到inPointsr4中的第i个元素
		CV_MAT_ELEM(*inPoints4, float, 0, i) =
			CV_MAT_ELEM(*inPoints4, float, 0, i) / div;
		CV_MAT_ELEM(*inPoints4, float, 1, i) =
			CV_MAT_ELEM(*inPoints4, float, 1, i) / div;    //因为得到的是一个齐次坐标，所以要除于第四行的元素才能得到真实的坐标
	}

	cvCopy(&inPoints2, outPoints);     //将最终IPM变换后的坐标放入到outPoints中
	cvReleaseMat(&inPoints4);
}

//计算摄像头到直线的距离还有直线的倾角
void RealLineParameter(Point2f & start, Point2f & end, float * parameter)
{
	float k = (start.y - end.y) / (start.x - end.x);   //求斜率
	float A = sqrt(1 - 1 / (1 + pow(k, 2)));
	float B = sqrt(1 - pow(A, 2));
	float angle = 0.0;
	if (k > 0)
	{
		B = -B;                    //因为A,B永远是正数,但当斜率为负时，A，B两个是异号的，所以B要求反
		angle = atan(k);
		angle = angle * 180 / PI;  //因为图像坐标原点是左上角，跟我们实际看到的直线正好上下翻转了
	}                              //所以要用180减去真正的角度，与我们看到的才相符
	else
	{
		angle = atan(k);
		angle = angle * 180 / PI;
		angle = 180 + angle;                 //理由同上，因为angle是负的，所以是加180
	}
	double C = -A * start.x - B * start.y;   //因为摄像头是原点，所以C就是摄像头离直线的距离

	parameter[0] = angle;      //求得的直线倾角
	parameter[1] = fabs(C);    //摄像头到直线的距离
}