// ComparePhase0928.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <time.h>
#include <opencv2\opencv.hpp>
#include <fstream>
#include "CameraDriver310Dll.h"
#pragma comment(lib,"CameraDriver310Dll.lib")
#include <Windows.h>
#pragma comment(lib,"D:\\program\\opencv\\build\\x64\\vc12\\lib\\opencv_world310.lib")
#include "extern.h"
#include "fbScanner.h"
#pragma comment(lib,"waqGeneral.lib")
#pragma comment(lib,"waqHardware.lib")
#include "waqMark.h"
#pragma comment(lib,"waqMark.lib")
using namespace cv;
using namespace std;
double PI = 3.14159265358979;
int W = 0;
int H = 0;
int nstep = 5;
int p1 = 13;
int p2 = 14;
int p3 = 15;
double *x = 0;
double *y = 0;
double *z = 0;

double *nx = 0;
double *ny = 0;
double *nz = 0;

unsigned char**pImgL = 0;
unsigned char**pImgR = 0;
unsigned char**proImg = 0;
unsigned char*gpImgL[17] = { 0 };
unsigned char*gpImgR[17] = { 0 };
unsigned char*gpproImg[17] = { 0 };

int gxbInit(int w, int h, int step);
int gxbGetWrappedPhase(int step, unsigned char **&Arrayofphase, double *PHASE, int size, int index, int row);
int gxbPhaseResult(unsigned char **&Img, double *phase);
int gxbCreateProjImg(int index, unsigned char*data, int proW);
int gxbPhaseflitter(double *PHASE);
int gxbRemoveBg(unsigned char *bgImgb, unsigned char *bgImgw, double *PHASE);
int gxbRegionGrow(unsigned char**&pImg, double *Phase);
int gxbPointCloudFilter(unsigned char*imgMask, double *X, double *Y, double *Z);
int gxbGetPhase(unsigned char**img, double *phase);
int gxbPro_Pho(int proW, int proH, unsigned char**&proImg, unsigned char**&pImgL, unsigned char**&pImgR);
int writemyphase(int index, int w, int h, double *phasel, double *phaser);
int gxbReduceUselessPhase(double *finalPhase, double *abPhase1, double *abPhase2);
int main()
{
	gxbInit(1280, 1024, 5);
	double *phaseL = new double[W*H];
	double *phaseR = new double[W*H];
	
	int proW = 1280, proH = 720;
	for (int i = 0; i < 17; i++)
	{
		if (gpImgL[i] == 0)
		{
			gpImgL[i] = new unsigned char[W*H];
			gpImgR[i] = new unsigned char[W*H];
			gpproImg[i] = new unsigned char[proW*proH];
		}
	}
	x = new double[W*H];
	y = new double[W*H];
	z = new double[W*H];

	nx = new double[W*H];
	ny = new double[W*H];
	nz = new double[W*H];
	double R[9] = { 0 };
	double T[3] = { 0 };
	double X[128] = { 0 };
	double Y[128] = { 0 };
	double Z[128] = { 0 };
	unsigned char *rgbL = new unsigned char[W*H * 3];
	unsigned char *rgbR = new unsigned char[W*H * 3];
	Mat Img;
	cv::Mat MarkrgbL, MarkrgbR;
	clock_t start, end;
	int markn1, markn2;
	char *cloudname = new char[50];
	int time =1;
	while (1)
	{
		gxbPro_Pho(proW, proH, proImg, pImgL, pImgR);
		Img = Mat(H, W, CV_8UC1, pImgL[16]);
		imwrite("pImgL16.bmp", Img);
		unsigned char *Img17L = new unsigned char[W*H];
		unsigned char *Img17R = new unsigned char[W*H];
		memcpy(Img17L, pImgL[16], W*H);
		memcpy(Img17R, pImgR[16], W*H);
		start = clock();
		gxbGetPhase(pImgL, phaseL);
		gxbGetPhase(pImgR, phaseR);
		end = clock();
		cout << "解相位耗时：" << end - start << "ms" << endl;
		int index = 350;
		writemyphase(index, W, H, phaseL, phaseR);
		memcpy(pImgL[1], Img17L, W*H);
		memcpy(pImgR[1], Img17R, W*H);
		fbInit("CamCCF.txt", "ProjCCF.txt", W, H);
		fbPhaseFilter(phaseL, 0.04, 50);
		fbPhaseFilter(phaseR, 0.04, 50);
		fbPointCloud(phaseL, phaseR, x, y, z, nx, ny, nz);
		//gxbPointCloudFilter(pImgL[0], x, y, z);
		waqReadCalibration("CamCCF.txt", "ProjCCF.txt", W, H);
		waqSegMarkParameter(100, 50,17 ,7);
		///获取标记点
		markn1 = waqGetMark3d(pImgL[1], pImgR[1], rgbL, rgbR, X, Y, Z);
		cout <<"mark:  " <<markn1 << endl;
		MarkrgbL = cv::Mat(H, W, CV_8UC3, rgbL);
		cv::imwrite("rgbL1.bmp", MarkrgbL);
		MarkrgbR = cv::Mat(H, W, CV_8UC3, rgbR);
		cv::imwrite("rgbR1.bmp", MarkrgbR);
		///标记点对齐
		waqMarkAlign(markn1, X, Y, Z, R, T, 2);
		for (int i = 0; i < 9; i++)
			cout << "R:" << R[i] << " ";
		cout << endl;
		for (int i = 0; i < 3; i++)
			cout << "T:" << T[i] << " ";
		cout << endl;
		///给点云应用RT
		waqSetRT(R, T, W*H, x, y, z);
		fbPointCloudFilter(x, y, z, nx, ny, nz, 0.1, 50);
		sprintf(cloudname, "%dcloud.txt", time);
		time++;
		fbSaveData(cloudname, W*H, x, y, z, nx, ny, nz);
		printf("随便输入个字符\n");
		getchar();
	}
	
	
	return 0;
}
//初始化
int gxbInit(int w, int h, int step)
{
	W = w;
	H = h;
	nstep = step;
	return 0;
}
//创建光栅
int gxbCreateProjImg(int index, unsigned char*data, int proW)
{
	int RangeColor = 180;
	if (index >= 1 && index <= 5)
	{
		for (int i = 0; i < proW; i++)
		{
			int remainer1 = i % p1;
			int result1 = int((sin((2 * PI / p1 * remainer1 + 2 * PI*(index - 1) / 5.0)) + 1) / 2.0 * RangeColor + 0.5f);
			data[i] = result1;
		}
	}
	else if (index >= 6 && index <= 10)
	{
		for (int i = 0; i < proW; i++)
		{
			int remainer2 = i % p2;
			int result2 = int((sin((2 * PI / p2 * remainer2 + 2 * PI*(index - 6) / 5.0)) + 1) / 2.0 * RangeColor + 0.5f);
			data[i] = result2;
		}
	}
	else if (index >= 11 && index <= 15)
	{
		for (int i = 0; i < proW; i++)
		{
			int remainer3 = i % p3;
			int result3 = int((sin((2 * PI / p3 * remainer3 + 2 * PI*(index - 11) / 5.0)) + 1) / 2.0 * RangeColor + 0.5f);
			data[i] = result3;
		}
	}
	else if (index == 16)
	{
		for (int i = 0; i < proW; i++)
		{
			data[i] = 0;
		}
	}
	else if (index == 17)
	{
		for (int i = 0; i < proW; i++)
		{
			data[i] = RangeColor;
		}
	}
	return 0;
}
//消除背景
int gxbRemoveBg(unsigned char *bgImgb, unsigned char *bgImgw, double *PHASE)
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
//相位滤波
int gxbPhaseflitter(double *PHASE)
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
//求包裹相位
int gxbGetWrappedPhase(int step, unsigned char **&Arrayofphase, double *PHASE, int size, int index, int row)
{
	//Mat getPhaseImg = Mat(H, W, CV_8UC1, Arrayofphase[1]);
	//imshow("getPhaseimg", getPhaseImg);
	//waitKey(-1);
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
//求展开相位
int gxbPhaseResult(unsigned char **&Img, double *phase)
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
	string filel = "myphase/Phase.txt";
	string filer = "myphase/Phase2.txt";
	string file3 = "myphase/Phase3.txt";
	string filecp1 = "myphase/cp_abPhase1.txt";
	string filecp2 = "myphase/cp_abPhase2.txt";
	string filecp22 = "myphase/cp_abPhase22.txt";
	string filecp3 = "myphase/cp_abPhase3.txt";
	ofstream outl, outr, out3,outcp1,outcp2,outcp22,outcp3;
	outl.open(filel.c_str(), ofstream::out);
	outr.open(filer.c_str(), ofstream::out);
	out3.open(file3.c_str(), ofstream::out);

	outcp1.open(filecp1.c_str(), ofstream::out);
	outcp2.open(filecp2.c_str(), ofstream::out);
	outcp22.open(filecp22.c_str(), ofstream::out);
	outcp3.open(filecp3.c_str(), ofstream::out);

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
	//保存下次展开真实相位
	double *cp_abPhase = new double[W];
	double *cp_abPhase2 = new double[W];
	double *cp_abPhase22 = new double[W];
	double *cp_abPhase3 = new double[W];

	for (int i = 0; i < H; i++)
	{
		//求包裹相位
		gxbGetWrappedPhase(nstep, Img, Phase, W, 0, i);
		gxbGetWrappedPhase(nstep, Img, Phase2, W, 5, i);
		gxbGetWrappedPhase(nstep, Img, Phase3, W, 10, i);

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
				cp_abPhase[l] = abPhase[l];
				cp_abPhase2[l] = abPhase2[l];
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
				cp_abPhase[l] = abPhase[l];
				cp_abPhase2[l] = abPhase2[l];
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
				cp_abPhase22[l] = abPhase22[l];
				cp_abPhase3[l] = abPhase3[l];
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
				cp_abPhase22[l] = abPhase22[l];
				cp_abPhase3[l] = abPhase3[l];
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
			cp_abPhase2[j] = abPhase2[j];
			cp_abPhase3[j] = abPhase3[j];
		}
		//解全局相位
		for (int l = 0; l < W; l++)
		{
			double delta = 0;
			//11
			abPhase[l] = (abPhase[l] + PI / 2) / (p2)-PI;
			//12
			abPhase2[l] = (abPhase2[l] + PI / 2) / (p1)-PI;
			//12
			abPhase22[l] = (abPhase22[l] + PI / 2) / (p3)-PI;
			//13
			abPhase3[l] = (abPhase3[l] + PI / 2) / (p2)-PI;
			//
			if (abPhase[l] >= abPhase22[l])
			{
				delta = abPhase[l] - abPhase22[l];
				ababPhase[l] = 2 * PI*(int)((double(p2*p3 / (p3 - p2)) / (p2*p3 / (p3 - p2) - p2*p1 / (p2 - p1)) * delta - abPhase[l]) / (2 * PI)) + abPhase[l];
				ababPhase22[l] = 2 * PI*(int)((double(p2*p1 / (p2 - p1)) / (p2*p3 / (p3 - p2) - p2*p1 / (p2 - p1)) * delta - abPhase22[l]) / (2 * PI)) + abPhase22[l];
				//t1 = (int)((double(p2*p3/(p3-p2)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase[l]) / (2 * PI));
				//t2 = (int)((double(p2*p1/(p2-p1)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase22[l]) / (2 * PI));
			}
			else
			{
				delta = abPhase[l] - abPhase22[l] + 2 * PI;
				ababPhase[l] = 2 * PI*(int)((double(p2*p3 / (p3 - p2)) / (p2*p3 / (p3 - p2) - p2*p1 / (p2 - p1)) * delta - abPhase[l]) / (2 * PI)) + abPhase[l];
				ababPhase22[l] = 2 * PI*(int)((double(p2*p1 / (p2 - p1)) / (p2*p3 / (p3 - p2) - p2*p1 / (p2 - p1)) * delta - abPhase22[l]) / (2 * PI)) + abPhase22[l];
				//t1 = (int)((double(p2*p3/(p3-p2)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase[l]) / (2 * PI));
				//t2 = (int)((double(p2*p1/(p2-p1)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase22[l]) / (2 * PI));

			}
			if (abPhase2[l] >= abPhase3[l])
			{
				delta = abPhase2[l] - abPhase3[l];
				ababPhase2[l] = 2 * PI*(int)((double(p2*p3 / (p3 - p2)) / (p2*p3 / (p3 - p2) - p2*p1 / (p2 - p1)) * delta - abPhase2[l]) / (2 * PI)) + abPhase2[l];
				ababPhase3[l] = 2 * PI*(int)((double(p2*p1 / (p2 - p1)) / (p2*p3 / (p3 - p2) - p2*p1 / (p2 - p1)) * delta - abPhase3[l]) / (2 * PI)) + abPhase3[l];
				//t1 = (int)((double(p2*p3/(p3-p2)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase2[l]) / (2 * PI));
				//t2 = (int)((double(p2*p1/(p2-p1)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase3[l]) / (2 * PI));

			}
			else
			{
				delta = abPhase2[l] - abPhase3[l] + 2 * PI;

				ababPhase2[l] = 2 * PI*(int)((double(p2*p3 / (p3 - p2)) / (p2*p3 / (p3 - p2) - p2*p1 / (p2 - p1)) * delta - abPhase2[l]) / (2 * PI)) + abPhase2[l];
				ababPhase3[l] = 2 * PI*(int)((double(p2*p1 / (p2 - p1)) / (p2*p3 / (p3 - p2) - p2*p1 / (p2 - p1)) * delta - abPhase3[l]) / (2 * PI)) + abPhase3[l];
				//t1 = (int)((double(p2*p3/(p3-p2)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase2[l]) / (2 * PI));
				//t2 = (int)((double(p2*p1/(p2-p1)) / (p2*p3/(p3-p2)-p2*p1/(p2-p1)) * delta - abPhase3[l]) / (2 * PI));
			}
			//归一化
			/*ababPhase3[l] = (ababPhase3[l] + PI) / (132 * 2 * PI) * (p2*p3/(p3-p2)-p2*p1/(p2-p1));*/

			phase[i*W + l] = ababPhase3[l];
		}
		gxbReduceUselessPhase(phase,abPhase2,abPhase3);
		if (i == 350)
		{
			for (int j = 0; j < W; j++)
			{
				outl << Phase[j] << endl;
				outr << Phase2[j] << endl;
				out3 << Phase3[j] << endl;
				outcp1 << cp_abPhase[j] << endl;
				outcp2 << cp_abPhase2[j] << endl;
				outcp22 << cp_abPhase22[j] << endl;
				outcp3 << cp_abPhase3[j] << endl;
			}
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

	//disConstruction(arrayofphase, arrayofphase2, arrayofphase3, tmp, img);
	return 0;
}
//边缘处理
int gxbRegionGrow(unsigned char**&pImg, double *Phase)
{
	int *Judge = new int[W*H];
	memset(Judge, 0, W * H);
	double *tmpPhaselr = new double[W*H];
	double *tmpPhaseud = new double[W*H];
	unsigned char *data = new unsigned char[W*H];
	unsigned char *result = new unsigned char[W*H];
	memset(data, 0, W*H);
	for (int i = 1; i < H - 1; i++)
	{
		for (int j = 1; j < W - 1; j++)
		{
			//cout << Phase[i*W + j] << endl;
			tmpPhaselr[i*W + j] = Phase[i*W + j + 1] - Phase[i*W + j];
			if (fabs(tmpPhaselr[i*W + j]) > 5)
				data[i*W + j] = 255;
			/*data[i*W + j] = uchar((tmpPhaselr[i*W + j]+PI) / (132 * 2 * PI)*24 * 255);*/
		}
	}
	for (int i = 1; i < H - 1; i++)
	{
		for (int j = 1; j < W - 1; j++)
		{
			tmpPhaseud[i*W + j] = Phase[i*W + j] - Phase[(i - 1)*W + j];
			if (fabs(tmpPhaseud[i*W + j]) > 5)
				data[i*W + j] = 255;
		}
	}
	cv::Mat showCont = cv::Mat(H, W, CV_8UC1, data);

	cv::threshold(showCont, showCont, 50, 255, 0);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * 1 + 1, 2 * 1 + 1),
		cv::Point(1, 1));
	///膨胀操作
	cv::dilate(showCont, showCont, element);
	unsigned char*Img = showCont.data;

	memset(result, 0, W*H);
	cv::Point2i pt = cv::Point2i(1, 1);
	cv::Point2i ptGrowing;                              //待生长点位置
	int nGrowLable = 0;                             //标记是否生长过
	int nSrcValue = 0;                              //生长起点灰度值
	int nCurValue = 0;                              //当前生长点灰度值

													//生长方向顺序数据
	int DIR[8][2] = { { -1,-1 },{ 0,-1 },{ 1,-1 },{ 1,0 },{ 1,1 },{ 0,1 },{ -1,1 },{ -1,0 } };
	std::vector<cv::Point2i> vcGrowPt;						//生长点栈 
	std::vector<cv::Point2i> vcSize;
	for (int p = 1; p < H - 1; p++)
	{
		for (int k = 1; k < W - 1; k++)
		{
			if (Img[p*W + k] == 255 && result[p*W + k] == 0)//种子点满足要求且未被访问过
			{
				pt = cv::Point2i(k, p);
				vcGrowPt.push_back(pt);//将生长点压入栈中 
				vcSize.push_back(pt);//记录生长区域的面积用于判断

				result[pt.y*W + pt.x] = 255;               //标记生长点  
				nSrcValue = Img[pt.y*W + pt.x];            //记录生长点的灰度值  

				while (!vcGrowPt.empty())                       //生长栈不为空则生长  
				{
					pt = vcGrowPt.back();                       //取出一个生长点  
					vcGrowPt.pop_back();

					//分别对八个方向上的点进行生长  
					for (int i = 0; i < 9; ++i)
					{
						ptGrowing.x = pt.x + DIR[i][0];
						ptGrowing.y = pt.y + DIR[i][1];
						//检查是否是边缘点  
						if (ptGrowing.x < 0 || ptGrowing.y < 0 || ptGrowing.x >(W - 1) || (ptGrowing.y > H - 1))
							continue;

						nGrowLable = result[ptGrowing.y*W + ptGrowing.x];      //当前待生长点的灰度值  

						if (nGrowLable == 0)                    //如果标记点还没有被生长  
						{
							nCurValue = Img[ptGrowing.y*W + ptGrowing.x];
							//cout << "生长点:" << ptGrowing << "  灰度值:" << nCurValue << endl;
							if (nCurValue == 255)                  //在阈值范围内则生长  
							{
								result[ptGrowing.y*W + ptGrowing.x] = 255;     //标记为白色  
								Judge[ptGrowing.y*W + ptGrowing.x] = 1;
								vcGrowPt.push_back(ptGrowing);                  //将下一个生长点压入栈中
								vcSize.push_back(ptGrowing);
							}
						}
					}
				}
				//如果生长区域的像素少于20个，则放弃该边缘，蒙版Mask的相关区域重新归0
				if (vcSize.size() < 30)
				{
					for (int i = 0; i < vcSize.size(); i++)
					{
						Judge[vcSize[i].y*W + vcSize[i].x] = 0;
					}
				}
				vcSize.clear();
			}
			else
				continue;
		}
	}
	for (int i = 0; i < H; i++)
	{
		for (int j = 0; j < W; j++)
		{
			int ij = i*W + j;
			if (Judge[ij] == 1)
			{
				pImg[0][ij] = 255;
			}
			else
			{
				pImg[0][ij] = 0;
			}
		}
	}
	delete[]Judge;
	delete[]data;
	delete[]tmpPhaseud;
	delete[]tmpPhaselr;
	return 0;
}
int gxbPointCloudFilter(unsigned char*imgMask, double *X, double *Y, double *Z)
{
	for (int i = 0; i < H; i++)
	{
		for (int j = 0; j < W; j++)
		{
			int ij = i*W + j;
			if (imgMask[ij] == 255)
			{
				X[ij] = NAN;
				Y[ij] = NAN;
				Z[ij] = NAN;
			}
		}
	}
	return 0;
}
//放在一个函数内
int gxbGetPhase(unsigned char**img, double *phase)
{

	gxbPhaseResult(img, phase);
	//gxbPhaseflitter(phase);
	gxbRemoveBg(img[15], img[16], phase);
	gxbRegionGrow(img, phase);


	return 0;
}
//投影拍照
int gxbPro_Pho(int proW, int proH, unsigned char**&proImg, unsigned char**&pImgL, unsigned char**&pImgR)
{

	pImgL = gpImgL;
	pImgR = gpImgR;
	proImg = gpproImg;
	unsigned char*data = new unsigned char[proW];
	double timeExpose = 0;
	int timeFlag = 0;
	unsigned char * rgbL = 0;
	unsigned char * rgbR = 0;
	CameraDeviceOpen();
	CameraDeviceSetGain(0, 0);
	while (1)
	{
		cin >> timeExpose >> timeFlag;

		CameraDeviceSetExposureValue(timeExpose * 1000, timeExpose * 1000);
		if (timeFlag == -1)
			break;
		Mat raster = imread("raster.bmp");
		namedWindow("raster", WINDOW_AUTOSIZE);
		setWindowProperty("raster", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		moveWindow("raster", 1920, 0);
		imshow("raster", raster);
		waitKey(400);
		destroyWindow("raster");
		CameraGetOnceImage(rgbL, rgbR);
		Mat testExposeImg = Mat(H, W, CV_8UC1, rgbL);
		imshow("testExposeImg", testExposeImg);
		waitKey(-1);
	}
	char* wndname = "img";
	for (int i = 0; i < 17; i++)
	{
		gxbCreateProjImg(i + 1, data, proW);
		for (int k = 0; k < proH; k++)
		{
			for (int j = 0; j < proW; j++)
			{
				proImg[i][k*proW + j] = data[j];
			}
		}
		Mat proShow = Mat(proH, proW, CV_8UC1, proImg[i]);
		imwrite("ras.bmp", proShow);
		namedWindow(wndname, WINDOW_AUTOSIZE);
		setWindowProperty(wndname, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		moveWindow(wndname, 1920, 0);
		imshow(wndname, proShow);
		if (i == 0)
			waitKey(400);
		else
			waitKey(300);
		CameraGetOnceImage(rgbL, rgbR);
		for (int j = 0; j < W*H; j++)
		{
			pImgL[i][j] = rgbL[j];
			pImgR[i][j] = rgbR[j];
		}
		Mat photo1 = Mat(H, W, CV_8UC1, pImgL[i]);
		resize(photo1, photo1, Size(640, 512));
		namedWindow("photo1", WINDOW_AUTOSIZE);
		setWindowProperty("photo1", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		moveWindow("photo1", 320, 284);
		imshow("photo1", photo1);
		Mat photo2 = Mat(H, W, CV_8UC1, pImgL[i]);
		resize(photo2, photo2, Size(640, 512));
		namedWindow("photo2", WINDOW_AUTOSIZE);
		setWindowProperty("photo2", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		moveWindow("photo2", 960, 284);
		imshow("photo2", photo2);
		waitKey(200);
	}
	destroyWindow("photo1");
	destroyWindow("photo2");
	destroyWindow(wndname);
	CameraDeviceClose();
	return 0;
}
//文件写
int writemyphase(int index,int w, int h, double *phasel, double *phaser)
{
	cout << "正在把相位写入文件" << endl;
	clock_t start = clock();
	string filel = "myphase/phaseL.txt";
	string filer = "myphase/phaseR.txt";
	ofstream outl, outr;
	outl.open(filel.c_str(), ofstream::out);
	outr.open(filer.c_str(), ofstream::out);
	if (index != -1)
	{
		for (int i = 0; i < 1; i++)
		{
			for (int j = 0; j < w; j++)
			{
				outl << phasel[index*w + j] << endl;
				outr << phaser[index*w + j] << endl;
			}
			//outl << endl;
			//outr << endl;
		}
	}
	
	clock_t end = clock();
	cout << "写文件耗时：" << end - start << "ms" << endl;
	return 0;
}
int gxbReduceUselessPhase(double *finalPhase,double *abPhase1,double *abPhase2)
{
	//保存差值数组用来判断混乱度
	double *Diff = new double[W*H];

	return 0;
}
