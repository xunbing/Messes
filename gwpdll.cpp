// getUnWrapedPhaseDLL.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.H"
#include <stdio.H>
#include <tchar.H>
#include <opencv2\opencv.hpp>
#include <iostream>
#include <Windows.H> 
#include <time.H>
#include <fstream>
#define getUnWrapedPhaseDLL_API extern "C" _declspec(dllexport)
#include "getUnWrapedPhaseDLL.H"
/*==========================
变量声明
===========================*/

double PI = 3.14159265358979;
int W = 0;
int H = 0;
int nstep = 5;
int p1 = 11;
int p2 = 12;
int p3 = 13;

getUnWrapedPhaseDLL_API int Init(int w, int h, int step)
{
	W = w;
	H = h;
	nstep = step;
	return 1;
}
getUnWrapedPhaseDLL_API int createProjImg(int index, unsigned char*data)
{
	if (index >= 1 && index <= 5)
	{
		for (int i = 0; i < W; i++)
		{
			int remainer1 = i % p1;
			int result1 = int((sin((2 * PI / p1 * remainer1 + 2 * PI*(index - 1) / 5.0)) + 1) / 2.0 * 255 + 0.5f);
			data[i] = result1;
		}
	}
	else if (index >= 6 && index <= 10)
	{
		for (int i = 0; i < W; i++)
		{
			int remainer2 = i % p2;
			int result2 = int((sin((2 * PI / p2 * remainer2 + 2 * PI*(index - 6) / 5.0)) + 1) / 2.0 * 255 + 0.5f);
			data[i] = result2;
		}
	}
	else if (index >= 11 && index <= 15)
	{
		for (int i = 0; i < W; i++)
		{
			int remainer3 = i % p3;
			int result3 = int((sin((2 * PI / p3 * remainer3 + 2 * PI*(index - 11) / 5.0)) + 1) / 2.0 * 255 + 0.5f);
			data[i] = result3;
		}
	}
	else if (index == 16)
	{
		for (int i = 0; i < W; i++)
		{
			data[i] = 0;
		}
	}
	else if (index == 17)
	{
		for (int i = 0; i < W; i++)
		{
			data[i] = 255;
		}
	}
	return 0;
}
getUnWrapedPhaseDLL_API int removeBg(unsigned char *bgImgb, unsigned char *bgImgw, double *PHASE)
{
	cv::Mat *bg = new cv::Mat[2];
	bg[0] = cv::Mat(H, W, CV_8UC1, bgImgb);
	bg[1] = cv::Mat(H, W, CV_8UC1, bgImgw);
	bool *judge = new bool[W*H];

	//黑白相减
	for (int i = 0; i < H; i++)
	{
		for (int j = 0; j < W; j++)
		{
			int ij = i*W + j;
			if (bgImgw[ij] - bgImgb[ij] > 5)
			{
				bgImgw[ij] = bgImgw[ij] - bgImgb[ij];
			}
			else
				bgImgw[ij] = 0;
			//if (bg[1].at<uchar>(i, j) - bg[0].at<uchar>(i, j) > 5)
			//	bg[1].at<uchar>(i, j) = bg[1].at<uchar>(i, j) - bg[0].at<uchar>(i, j);
			//else
			//	bg[1].at<uchar>(i, j) = 0;
		}
	}
	//中值滤波减少椒盐噪声
	cv::medianBlur(bg[1], bg[1], 3);

	cv::Mat bg_copy1 = cv::Mat(H, W, CV_8UC1, bgImgw);
	//二值化
	cv::threshold(bg_copy1, bg_copy1, 0, 255, CV_THRESH_BINARY);

	blur(bg_copy1, bg_copy1, cv::Size(3, 3));
	cv::Mat detected_edges;
	/// 运行Canny算子
	Canny(bg_copy1, detected_edges, 40, 120, 3);

	/// 使用 Canny算子输出边缘作为掩码显示原图像
	cv::Mat dst;
	dst = cv::Scalar::all(0);
	bg_copy1.copyTo(dst, detected_edges);

	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * 1 + 1, 2 * 1 + 1),
		cv::Point(1, 1));
	///膨胀操作
	dilate(dst, dst, element);
	threshold(dst, dst, 10, 255, CV_THRESH_BINARY);
	for (int i = 0; i < H; i++)
	{
		for (int j = 0; j < W; j++)
		{
			if (dst.data[i*W + j] == 255)
			{
				PHASE[i*W + j] = 0;
			}
		}
	}
	cv::Mat element2 = cv::getStructuringElement(0, cv::Size(3, 3));
	// 腐蚀操作
	cv::erode(bg_copy1, bg_copy1, element2);
	for (int i = 1; i < H; i++)
	{
		for (int j = 1; j < W; j++)
		{
			int weight = 0;
			if (bgImgw[i*W + j] == 255)
			{
				//八连通区域检测
				if (i < H - 1 && j < W - 1)
				{
					if (bgImgw[i*W + j - 1] == 0)
						weight++;
					if (bgImgw[i*W + j + 1] == 0)
						weight++;
					if (bgImgw[(i - 1)*W + j] == 0)
						weight++;
					if (bgImgw[(i + 1)*W + j] == 0)
						weight++;
					if (bgImgw[(i - 1)*W + j - 1] == 0)
						weight++;
					if (bgImgw[(i + 1)*W + j + 1] == 0)
						weight++;
					if (bgImgw[(i - 1)*W + j + 1] == 0)
						weight++;
					if (bgImgw[(i + 1)*W + j - 1] == 0)
						weight++;
				}
				if (weight <= 3)
					judge[i*W + j] = TRUE;
				else
					judge[i*W + j] = FALSE;
			}
			else
			{
				judge[i*W + j] = FALSE;
				//bg[1].at<uchar>(i, j) = 0;
			}
		}
	}
	for (int k = 0; k < 17; k++)
	{
		for (int i = 0; i < H; i++)
		{
			for (int j = 0; j < W; j++)
			{
				if (i < H - 1 && j < W - 1)
				{
					if (!judge[i*W + j])
					{
						PHASE[i*W + j] = 0;
					}
				}
			}
		}
	}
	delete[]judge;
	return 0;
}
getUnWrapedPhaseDLL_API int phaseflitter(double *PHASE)
{
	///上下左右八连通滤波
	double *phasej = new double[W*H];
	for (int i = 0; i < W*H; i++)
	{
		phasej[i] = 0;
	}
	double aver = 0;

	for (int i = 1; i < H - 1; i++)
	{
		for (int j = 1; j < W - 1; j++)
		{
			aver = (PHASE[i*W + j] + PHASE[(i - 1)*W + j] + PHASE[(i + 1)*W + j] + PHASE[i*W + j - 1] + PHASE[i*W + j + 1] + PHASE[(i - 1)*W + j - 1] + PHASE[(i - 1)*W + j + 1] + PHASE[(i + 1)*W + j + 1] + PHASE[(i + 1)*W + j - 1]) / 9;
			if (fabs(aver - PHASE[i*W + j]) > 0.1)
				phasej[i*W + j] = 1;
		}
	}

	for (int i = 1; i < H - 1; i++)
	{
		for (int j = 1; j < W - 1; j++)
		{
			if (phasej[i*W + j] == 1)
				PHASE[i*W + j] = 0;
		}
	}
	delete[]phasej;
	return 0;
}
getUnWrapedPhaseDLL_API int getWrappedPhase(int step, unsigned char **Arrayofphase, double *PHASE, int size, int index, int row)
{

	double* sinValue = new double[step];
	double* cosValue = new double[step];
	double interMove = 2 * PI / step;

	for (int k = 0; k < step; k++)
	{
		sinValue[k] = -1 * sin((k)*interMove);
		cosValue[k] = cos((k)*interMove);
	}

	for (int i = 0; i < size; i++)
	{
		double deltax = 0;
		double deltay = 0;

		for (int k = 0; k < step; k++)
		{
			deltay += Arrayofphase[k + index][row*W + i] * sinValue[k];
			deltax += Arrayofphase[k + index][row*W + i] * cosValue[k];
		}
		{
			if (deltax == 0 && deltay >= 0)
				PHASE[i] = PI / 2;
			else if (deltax == 0 && deltay < 0)
				PHASE[i] = -PI / 2;
			else
				PHASE[i] = atan2(deltay, deltax);

			//Phase[i] = (Phase[i] + PI)/(2*PI);
		}
	}
	delete[]sinValue;
	delete[]cosValue;
	return 0;
}
getUnWrapedPhaseDLL_API int phaseResult(unsigned char **Img, double *phase)
{
	double *Phase;
	double *Phase2;
	double *Phase3;
	double *abPhase;
	double *abPhase2;
	double *abPhase22;
	double *abPhase3;

	double *ababPhase;
	double *ababPhase2;
	double *ababPhase22;
	double *ababPhase3;

	//临时存储φ1，φ2和φ2，φ3的周期数
	int *C1;
	int *C2;
	int *C3;
	int *C4;
	C1 = new int[W];
	C2 = new int[W];
	C3 = new int[W];
	C4 = new int[W];
	//相移矩阵5*W的二维数组


	Phase = new double[W];
	Phase2 = new double[W];
	Phase3 = new double[W];
	abPhase = new double[W];
	abPhase2 = new double[W];
	abPhase22 = new double[W];
	abPhase3 = new double[W];
	ababPhase = new double[W];
	ababPhase2 = new double[W];
	ababPhase22 = new double[W];
	ababPhase3 = new double[W];
	//delete[]arrayofphase3;
	for (int i = 0; i < H; i++)
	{
		//求包裹相位
		getWrappedPhase(nstep, Img, Phase, W, 0, i);
		getWrappedPhase(nstep, Img, Phase2, W, 5, i);
		getWrappedPhase(nstep, Img, Phase3, W, 10, i);

		//解次展开相位
		for (int l = 0; l < W; l++)
		{

			double delta = 0;
			double xi1 = 0, xi2 = 0;
			double d1 = 0, d2 = 0;

			if (Phase[l] >= Phase2[l])
			{
				delta = Phase[l] - Phase2[l];
				d1 = p2*delta;
				d2 = p1*delta;
				C1[l] = (int)((p2 / (p2 - p1)*delta - Phase[l]) / (2 * PI));
				C2[l] = (int)((p1 / (p2 - p1)*delta - Phase2[l]) / (2 * PI));

				abPhase[l] = 2 * PI*C1[l] + Phase[l];
				abPhase2[l] = 2 * PI*C2[l] + Phase2[l];
				//if (C1[l] == 12)	C1[l] = 0;
				//if (C2[l] == 11)	C2[l] = 0;

				if (C1[l - 1] == -1)
				{
					abPhase[l - 1] = (abPhase[l] + abPhase[l - 2]) / 2;
				}
				if (C2[l - 1] == -1)
				{
					abPhase2[l - 1] = (abPhase2[l] + abPhase2[l - 2]) / 2;
				}
				//
				if (d1 - abPhase[l] > 4.8)
				{
					//if (abPhase[l]<abPhase[l - 1])
					{
						abPhase[l] += 2 * PI; C1[l] += 1;
					}
				}
				if (d2 - abPhase2[l] > 4.8)
				{
					//if (abPhase2[l] < abPhase2[l - 1])
					{
						abPhase2[l] += 2 * PI; C2[l] += 1;
					}
				}
				xi1 = abPhase[l] - d1;
				xi2 = abPhase2[l] - d2;
				abPhase[l] -= -p1*xi1 / (p1*p1 + p2*p2);
				abPhase2[l] -= p2*xi2 / (p1*p1 + p2*p2);
			}
			else
			{
				delta = Phase[l] - Phase2[l] + 2 * PI;
				d1 = p2*delta;
				d2 = p1*delta;
				C1[l] = (int)((p2 / (p2 - p1)*delta - Phase[l]) / (2 * PI));
				C2[l] = (int)((p1 / (p2 - p1)*delta - Phase2[l]) / (2 * PI));

				/*if (i == 943)
				outfile123 << C1[l] << endl;*/


				abPhase[l] = 2 * PI*C1[l] + Phase[l];
				abPhase2[l] = 2 * PI*C2[l] + Phase2[l];
				//if (C1[l] == 12)	C1[l] = 0;
				//if (C2[l] == 11)	C2[l] = 0;

				if (C1[l - 1] == -1)
				{
					abPhase[l - 1] = (abPhase[l] + abPhase[l - 2]) / 2;
				}
				if (C2[l - 1] == -1)
				{
					abPhase2[l - 1] = (abPhase2[l] + abPhase2[l - 2]) / 2;
				}
				if (d1 - abPhase[l] > 4.8)
				{
					//if (abPhase[l]<abPhase[l - 1])
					{
						abPhase[l] += 2 * PI; C1[l] += 1;
					}
				}
				if (d2 - abPhase2[l] > 4.8)
				{
					//if (abPhase2[l] < abPhase2[l - 1])
					{
						abPhase2[l] += 2 * PI; C2[l] += 1;
					}
				}


				xi1 = abPhase[l] - d1;
				xi2 = abPhase2[l] - d2;
				abPhase[l] -= -p1*xi1 / (p1*p1 + p2*p2);
				abPhase2[l] -= p2*xi2 / (p1*p1 + p2*p2);
			}
			if (Phase2[l] >= Phase3[l])
			{
				delta = Phase2[l] - Phase3[l];
				d1 = p3*delta;
				d2 = p2*delta;

				C3[l] = (int)((p3 / (p3 - p2)*delta - Phase2[l]) / (2 * PI));
				C4[l] = (int)((p2 / (p3 - p2)*delta - Phase3[l]) / (2 * PI));

				abPhase22[l] = 2 * PI*C3[l] + Phase2[l];
				abPhase3[l] = 2 * PI*C4[l] + Phase3[l];


				//if (C3[l] == 13)	C3[l] = 0;
				//if (C4[l] == 12)	C4[l] = 0;


				if (C3[l - 1] == -1)
				{
					abPhase22[l - 1] = (abPhase22[l] + abPhase22[l - 2]) / 2;
				}
				if (C4[l - 1] == -1)
				{
					abPhase3[l - 1] = (abPhase3[l] + abPhase3[l - 2]) / 2;
				}


				if (d1 - abPhase22[l] > 4.8)
				{
					//if (abPhase22[l]<abPhase22[l - 1])
					{
						abPhase22[l] += 2 * PI; C3[l] += 1;
					}
				}
				if (d2 - abPhase3[l] > 4.8)
				{
					//if (abPhase22[l] < abPhase22[l - 1])
					{
						abPhase3[l] += 2 * PI; C4[l] += 1;
					}
				}

				xi1 = abPhase22[l] - d1;
				xi2 = abPhase3[l] - d2;
				abPhase22[l] -= -p2*xi1 / (p3*p3 + p2*p2);
				abPhase3[l] -= p3*xi2 / (p3*p3 + p2*p2);

			}
			else
			{
				delta = Phase2[l] - Phase3[l] + 2 * PI;
				d1 = p3*delta;
				d2 = p2*delta;
				C3[l] = (int)((p3 / (p3 - p2)*delta - Phase2[l]) / (2 * PI));
				C4[l] = (int)((p2 / (p3 - p2)*delta - Phase3[l]) / (2 * PI));

				abPhase22[l] = 2 * PI*C3[l] + Phase2[l];
				abPhase3[l] = 2 * PI*C4[l] + Phase3[l];
				//if (C3[l] == 13)	C3[l] = 0;
				//if (C4[l] == 12)	C4[l] = 0;


				if (C3[l - 1] == -1)
				{
					abPhase22[l - 1] = (abPhase22[l] + abPhase22[l - 2]) / 2;
				}
				if (C4[l - 1] == -1)
				{
					abPhase3[l - 1] = (abPhase3[l] + abPhase3[l - 2]) / 2;
				}



				if (d1 - abPhase22[l] > 4.8)
				{
					//if (abPhase22[l]<abPhase22[l - 1])
					{
						abPhase22[l] += 2 * PI; C3[l] += 1;
					}
				}
				if (d2 - abPhase3[l] > 4.8)
				{
					//if (abPhase22[l] < abPhase22[l - 1])
					{
						abPhase3[l] += 2 * PI; C4[l] += 1;
					}
				}

				xi1 = abPhase22[l] - d1;
				xi2 = abPhase3[l] - d2;
				abPhase22[l] -= -p2*xi1 / (p3*p3 + p2*p2);
				abPhase3[l] -= p3*xi2 / (p3*p3 + p2*p2);
			}
		}
		//对特殊点取平均值
		for (int j = 1; j < W - 1; j++)
		{
			if (abPhase2[j] - abPhase2[j - 1] > PI&&abPhase2[j] - abPhase2[j - 1] < 3 * PI)
			{
				abPhase2[j] = (abPhase2[j - 1] + abPhase2[j + 1]) / 2;
			}
			if (abPhase3[j] - abPhase3[j - 1] > PI&&abPhase3[j] - abPhase3[j - 1] < 3 * PI)
			{
				abPhase3[j] = (abPhase3[j - 1] + abPhase3[j + 1]) / 2;
			}
			if (abPhase2[j] - abPhase2[j - 1]<-PI&&abPhase2[j] - abPhase2[j - 1]>-3 * PI)
			{
				abPhase2[j] = (abPhase2[j - 1] + abPhase2[j + 1]) / 2;
			}
			if (abPhase3[j] - abPhase3[j - 1]<-PI&&abPhase3[j] - abPhase3[j - 1]>-3 * PI)
			{
				abPhase3[j] = (abPhase3[j - 1] + abPhase3[j + 1]) / 2;
			}
		}
		//解全局相位
		for (int l = 0; l < W; l++)
		{
			double delta = 0;
			//11
			abPhase[l] = (abPhase[l] + PI / 2) / (p2) - PI;
			//12
			abPhase2[l] = (abPhase2[l] + PI / 2) / (p1) - PI;
			//12
			abPhase22[l] = (abPhase22[l] + PI / 2) / (p3) - PI;
			//13
			abPhase3[l] = (abPhase3[l] + PI / 2) / (p2) - PI;
			//
			if (abPhase[l] >= abPhase22[l])
			{
				delta = abPhase[l] - abPhase22[l];
				ababPhase[l] = 2 * PI*(int)((double(p2*p3/(p3-p2)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase[l]) / (2 * PI)) + abPhase[l];
				ababPhase22[l] = 2 * PI*(int)((double(p2*p1/(p2-p1)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase22[l]) / (2 * PI)) + abPhase22[l];
				//t1 = (int)((double(p2*p3/(p3-p2)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase[l]) / (2 * PI));
				//t2 = (int)((double(p2*p1/(p2-p1)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase22[l]) / (2 * PI));
			}
			else
			{
				delta = abPhase[l] - abPhase22[l] + 2 * PI;
				ababPhase[l] = 2 * PI*(int)((double(p2*p3/(p3-p2)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase[l]) / (2 * PI)) + abPhase[l];
				ababPhase22[l] = 2 * PI*(int)((double(p2*p1/(p2-p1)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase22[l]) / (2 * PI)) + abPhase22[l];
				//t1 = (int)((double(p2*p3/(p3-p2)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase[l]) / (2 * PI));
				//t2 = (int)((double(p2*p1/(p2-p1)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase22[l]) / (2 * PI));

			}
			if (abPhase2[l] >= abPhase3[l])
			{
				delta = abPhase2[l] - abPhase3[l];
				ababPhase2[l] = 2 * PI*(int)((double(p2*p3/(p3-p2)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase2[l]) / (2 * PI)) + abPhase2[l];
				ababPhase3[l] = 2 * PI*(int)((double(p2*p1/(p2-p1)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase3[l]) / (2 * PI)) + abPhase3[l];
				//t1 = (int)((double(p2*p3/(p3-p2)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase2[l]) / (2 * PI));
				//t2 = (int)((double(p2*p1/(p2-p1)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase3[l]) / (2 * PI));

			}
			else
			{
				delta = abPhase2[l] - abPhase3[l] + 2 * PI;

				ababPhase2[l] = 2 * PI*(int)((double(p2*p3/(p3-p2)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase2[l]) / (2 * PI)) + abPhase2[l];
				ababPhase3[l] = 2 * PI*(int)((double(p2*p1/(p2-p1)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase3[l]) / (2 * PI)) + abPhase3[l];
				//t1 = (int)((double(p2*p3/(p3-p2)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase2[l]) / (2 * PI));
				//t2 = (int)((double(p2*p1/(p2-p1)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase3[l]) / (2 * PI));
			}
			//归一化
			/*ababPhase3[l] = (ababPhase3[l] + PI) / (132 * 2 * PI) * (p2*p3/(p3-p2)-p2*p1/(p2-p1));*/
			phase[i*W + l] = ababPhase3[l];
		}
	}
	delete[]C1;
	delete[]C2;
	delete[]C3;
	delete[]C4;
	delete[]Phase;
	delete[]Phase2;
	delete[]Phase3;
	delete[]abPhase;
	delete[]abPhase2;
	delete[]abPhase22;
	delete[]abPhase3;
	delete[]ababPhase;
	delete[]ababPhase2;
	delete[]ababPhase22;
	delete[]ababPhase3;
	C1=NULL;
	C2=NULL;
	C3=NULL;
	C4=NULL;
	Phase = NULL;
	Phase2 = NULL;
	Phase3 = NULL;
	abPhase = NULL;
	abPhase2 = NULL;
	abPhase22 = NULL;
	abPhase3 = NULL;
	ababPhase = NULL;
	ababPhase2 = NULL;
	ababPhase22 = NULL;
	ababPhase3 = NULL;
	//disConstruction(arrayofphase, arrayofphase2, arrayofphase3, tmp, img);
	return 0;
}

