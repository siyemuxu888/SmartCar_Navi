#include <opencv2/opencv.hpp>
#include <iostream>
#include "IPMinfo.h"
using namespace cv;

#define PI 3.14159

//�����任����
float * CreateMatrix(const CameraInfo * cameraInfo)
{
	//�����任����
	float c1 = cos(cameraInfo->pitch * PI / 180);  //ת��Ϊ����
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

//��ÿһ֡ͼ���еĵ㣨uv���꣩�任����ƽ���ϵ���ʵ��������������
//����inpointΪͼ�������㣬����outpointΪͼ�������㣬transMat����IPM�任����
void TransformImage2Ground(const CvMat *inPoints, CvMat *outPoints, const CvMat * transMat)
{	
	//�����������м���ǰ2��
	CvMat *inPoints4 = cvCreateMat(inPoints->rows + 2, inPoints->cols, cvGetElemType(inPoints));

	//��inpoints���Ƶ�ǰ2����ȥ
	CvMat inPoints2, inPoints3, inPointsr4, inPointsr3;
	cvGetRows(inPoints4, &inPoints2, 0, 2);  //��inPoints2ָ��inPoints4��ǰ2��
	cvGetRows(inPoints4, &inPoints3, 0, 3);  //��inPoints3ָ��inPoints4��ǰ3��
	cvGetRow(inPoints4, &inPointsr3, 2);     //��inPointsr3ָ��inPoints4�е�3��
	cvGetRow(inPoints4, &inPointsr4, 3);     //��inPointsr4ָ��inPoints4�е�4��
	cvSet(&inPointsr3, cvRealScalar(1));     //�൱�ڽ�inPoints4�еĵ�3��ȫ��Ϊ1
	cvCopy(inPoints, &inPoints2);            //�൱�ڽ�����ĵ㸴�Ƶ�inPoints4�е�ǰ2��
	
	cvMatMul(transMat, &inPoints3, inPoints4); //IPM�任�����inPoints3��ˣ�����ŵ�inPoints4

	//�����ǳ���inPoints4��������һ��
	for (int i = 0; i< inPoints->cols; i++)
	{
		float div = CV_MAT_ELEM(inPointsr4, float, 0, i);  //�õ�inPointsr4�еĵ�i��Ԫ��
		CV_MAT_ELEM(*inPoints4, float, 0, i) =
			CV_MAT_ELEM(*inPoints4, float, 0, i) / div;
		CV_MAT_ELEM(*inPoints4, float, 1, i) =
			CV_MAT_ELEM(*inPoints4, float, 1, i) / div;    //��Ϊ�õ�����һ��������꣬����Ҫ���ڵ����е�Ԫ�ز��ܵõ���ʵ������
	}

	cvCopy(&inPoints2, outPoints);     //������IPM�任���������뵽outPoints��
	cvReleaseMat(&inPoints4);
}

//��������ͷ��ֱ�ߵľ��뻹��ֱ�ߵ����
void RealLineParameter(Point2f & start, Point2f & end, float * parameter)
{
	float k = (start.y - end.y) / (start.x - end.x);   //��б��
	float A = sqrt(1 - 1 / (1 + pow(k, 2)));
	float B = sqrt(1 - pow(A, 2));
	float angle = 0.0;
	if (k > 0)
	{
		B = -B;                    //��ΪA,B��Զ������,����б��Ϊ��ʱ��A��B��������ŵģ�����BҪ��
		angle = atan(k);
		angle = angle * 180 / PI;  //��Ϊͼ������ԭ�������Ͻǣ�������ʵ�ʿ�����ֱ���������·�ת��
	}                              //����Ҫ��180��ȥ�����ĽǶȣ������ǿ����Ĳ����
	else
	{
		angle = atan(k);
		angle = angle * 180 / PI;
		angle = 180 + angle;                 //����ͬ�ϣ���Ϊangle�Ǹ��ģ������Ǽ�180
	}
	double C = -A * start.x - B * start.y;   //��Ϊ����ͷ��ԭ�㣬����C��������ͷ��ֱ�ߵľ���

	parameter[0] = angle;      //��õ�ֱ�����
	parameter[1] = fabs(C);    //����ͷ��ֱ�ߵľ���
}