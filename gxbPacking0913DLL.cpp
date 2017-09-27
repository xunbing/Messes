// gxbPacking0913DLL.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include <stdio.h>
#include <tchar.h>
#include <vector>
#include <opencv2\opencv.hpp>
using namespace cv;
#include <iostream>
using namespace std;
#include <Windows.h> 
#include <time.h>
#include <fstream>
#define gxbPacking0913DLL_API extern "C" _declspec(dllexport)
//解相模块
#include "getUnWrapedPhaseDLL.h"
#pragma comment(lib,"getUnWrapedPhaseDLL.lib")
//点云处理模块
#include "fbScanner.h"
#include "extern.h"
#pragma comment(lib,"waqGeneral.lib")
#pragma comment(lib,"waqHardware.lib")
//黑白相机驱动模块
#include "CameraDriver310Dll.h"
#pragma comment(lib,"CameraDriver310Dll.lib")
#pragma comment(lib,"opencv_world310.lib")
#include "waqMark.h"
#pragma comment(lib,"waqMark.lib")
/*==========================
		  变量声明
===========================*/
double PI = 3.14159265358979;
int proW = 1280;
int proH = 720;
int W = 0;
int H = 0;
int nstep = 5;
int p1 = 13;
int p2 = 14;
int p3 = 15;

unsigned char*gpImgL[17] = { 0 };
unsigned char*gpImgR[17] = { 0 };

/***************************************************************************************
Function:  初始化标定+初始化解相
Input:     strL strR 左右相机的已标定文件 w 照片宽度 h 照片高度(1280 x 1024)
Output:   控制台输出初始化信息
Description: 调用点云模块和解相模块的初始化函数
Return:    失败返回-1，成功返回0
Others:    NULL
***************************************************************************************/
gxbPacking0913DLL_API int gxbCalibrationInitial(char *strL, char *strR, int w, int h,int PROW,int PROH)
{
	W = w;
	H = h;
	proH = PROH;
	proW = PROW;
	//解相位前的初始化
	Init(w, h, nstep);
	std::cout << "相位初始化完成..." << std::endl;
	//生成点云前的初始化
	fbInit(strL, strR, w, h);
	
	std::cout << "标定初始化完成..." << std::endl;
	return 0;
}
gxbPacking0913DLL_API int createProImg(int index, unsigned char*&data)
{
	if (index >= 0 && index <= 4)
	{
		for (int j = 0; j < proH; j++)
		{
			for (int i = 0; i < proW; i++)
			{
				int remainer1 = i % p1;
				int result1 = int((sin((2 * PI / p1 * remainer1 + 2 * PI*(index) / 5.0)) + 1) / 2.0 * 255 + 0.5f);
				data[j*proW + i] = result1;
			}
		}
	}
	else if (index >= 5 && index <= 9)
	{
		for (int j = 0; j < proH; j++)
		{
			for (int i = 0; i < proW; i++)
			{
				int remainer2 = i % p2;
				int result2 = int((sin((2 * PI / p2 * remainer2 + 2 * PI*(index - 5) / 5.0)) + 1) / 2.0 * 255 + 0.5f);
				data[j*proW + i] = result2;
			}
		}

	}
	else if (index >= 10 && index <= 14)
	{
		for (int j = 0; j < proH; j++)
		{
			for (int i = 0; i < proW; i++)
			{
				int remainer3 = i % p3;
				int result3 = int((sin((2 * PI / p3 * remainer3 + 2 * PI*(index - 10) / 5.0)) + 1) / 2.0 * 255 + 0.5f);
				data[j*proW + i] = result3;
			}
		}

	}
	else if (index == 15)
	{
		for (int j = 0; j < proH; j++)
		{
			for (int i = 0; i < proW; i++)
			{
				data[j*proW + i] = 0;
			}
		}

	}
	else if (index == 16)
	{
		for (int j = 0; j < proH; j++)
		{
			for (int i = 0; i < proW; i++)
			{
				data[j*proW + i] = 255;
			}
		}
	}
	return 0;
}
/***************************************************************************************
Function:  拍摄一组相移图片(同时配合投影仪投影)
Input:     pImgL pImgR 左右相机相移图片指针(内部申请内存) 【appendExpose曝光增益(0-14.9) timeExpose曝光时间(单位：千微秒)】(外部传入)
Output:    pImgL pImgR的引用，在gxbGetPointCloud函数中直接使用，最后释放
Description: 调用相机驱动模块，软硬触发方式打开，函数执行完关闭相机，多次拍摄需重复调用该函数
Return:    失败返回-1，成功返回0
Others:    NULL
***************************************************************************************/
gxbPacking0913DLL_API int gxbGetPhaseImage(unsigned char**&pImgL, unsigned char**&pImgR, float appendExpose, float timeExpose)
{

	//pImgL = new unsigned char*[17];
	//pImgR = new unsigned char*[17];
	for (int i = 0; i < 17; i++)
	{
		if (gpImgL[i] == 0)
		{
			gpImgL[i] = new unsigned char[W*H];
			gpImgR[i] = new unsigned char[W*H];
		}
	}
	pImgL = gpImgL;
	pImgR = gpImgR;

	///记得释放
	unsigned char *data = new unsigned char[proW*proH];
	char* wndname = "img";

	//float appendExpose = 0,timeExpose = 20000;
	//std::cout << "请输入曝光增益(0 - 14.9)" << std::endl;
	//std::cin >> appendExpose;
	//std::cout << "请输入曝光时间(0 - 25)(千微秒)" << std::endl;
	//std::cin >> timeExpose;
	CameraDeviceOpen();
	CameraDeviceSetGain(appendExpose, appendExpose);
	CameraDeviceSetExposureValue(timeExpose * 1000, timeExpose * 1000);
	for (int i = 0; i < 17; i++)
	{
		unsigned char * rgbL = 0;
		unsigned char * rgbR = 0;
		createProImg(i, data);
		cv::Mat tmp_img = cv::Mat(proH, proW, CV_8UC1, data);

		namedWindow(wndname, WINDOW_AUTOSIZE);
		setWindowProperty(wndname, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		moveWindow(wndname, 1920, 0);
		imshow(wndname, tmp_img);
		if (i == 0)
			waitKey(400);
		else
			waitKey(200);
		CameraGetOnceImage(rgbL, rgbR);
		for (int j = 0; j < W*H; j++)
		{
			pImgL[i][j] = rgbL[j];
			pImgR[i][j] = rgbR[j];
		}
		/*
		tmpL = Mat(H, W, CV_8UC1, rgbL);
		tmpR = Mat(H, W, CV_8UC1, rgbR);
		if (i < 12)
		{
			sprintf(name1, "p/testImg1/a%ximg_left.bmp", i + 1);
			sprintf(name2, "p/testImg2/a%ximg_right.bmp", i + 1);
			imwrite(name1, tmpL);
			imwrite(name2, tmpR);
		}
		else if (i >= 12 && i < 17)
		{
			sprintf(name1, "p/testImg1/b%ximg_left.bmp", i + 1 - 12);
			sprintf(name2, "p/testImg2/b%ximg_right.bmp", i + 1 - 12);
			imwrite(name1, tmpL);
			imwrite(name2, tmpR);
		}
		else if (i >= 17 && i < 22)
		{
			sprintf(name1, "p/testImg1/c%ximg_left.bmp", i + 1 - 17);
			sprintf(name2, "p/testImg2/c%ximg_right.bmp", i + 1 - 17);
			imwrite(name1, tmpL);
			imwrite(name2, tmpR);
		}
		else if (i >= 22 && i < 27)
		{
			sprintf(name1, "p/testImg1/d%ximg_left.bmp", i + 1 - 22);
			sprintf(name2, "p/testImg2/d%ximg_right.bmp", i + 1 - 22);
			imwrite(name1, tmpL);
			imwrite(name2, tmpR);
		}
		else if (i >= 27 && i < 29)
		{
			sprintf(name1, "p/testImg1/e%ximg_left.bmp", i + 1 - 27);
			sprintf(name2, "p/testImg2/e%ximg_right.bmp", i + 1 - 27);
			imwrite(name1, tmpL);
			imwrite(name2, tmpR);
		}*/
	}
	delete[]data;
	destroyWindow(wndname);

	CameraDeviceClose();

	return 0;
}
/***************************************************************************************
Function:  区域生长的边缘处理
Input:     
Output:    
Description: 
Return:    
Others:    
***************************************************************************************/
gxbPacking0913DLL_API int RegionGrow(unsigned char**&pImgL,double *Phase, int *Judge)
{

	double *tmpPhaselr = new double[W*H];
	double *tmpPhaseud = new double[W*H];
	double PI = 3.14159265358979;
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
	Mat showCont = Mat(H, W, CV_8UC1, data);

	threshold(showCont, showCont, 50, 255, 0);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * 1 + 1, 2 * 1 + 1),
		cv::Point(1, 1));
	///膨胀操作
	dilate(showCont, showCont, element);
	unsigned char*Img = showCont.data;

	memset(result, 0, W*H);
	Point2i pt = Point2i(1, 1);
	Point2i ptGrowing;                              //待生长点位置
	int nGrowLable = 0;                             //标记是否生长过
	int nSrcValue = 0;                              //生长起点灰度值
	int nCurValue = 0;                              //当前生长点灰度值

													//生长方向顺序数据
	int DIR[8][2] = { { -1,-1 },{ 0,-1 },{ 1,-1 },{ 1,0 },{ 1,1 },{ 0,1 },{ -1,1 },{ -1,0 } };
	vector<Point2i> vcGrowPt;						//生长点栈 
	vector<Point2i> vcSize;
	for (int p = 1; p < H - 1; p++)
	{
		for (int k = 1; k < W - 1; k++)
		{
			if (Img[p*W + k] == 255 && result[p*W + k] == 0)//种子点满足要求且未被访问过
			{
				pt = Point2i(k, p);
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
				pImgL[0][ij] = 255;
			}
			else
			{
				pImgL[0][ij] = 0;
			}
		}
	}
	
	delete[]data;
	delete[]tmpPhaseud;
	delete[]tmpPhaselr;
	return 0;
}
/***************************************************************************************
Function:  解相位+点云生成
Input:     pImgL pImgR 左右相机相移图片 X, Y, Z, Nx, Ny, Nz 三维坐标和法线坐标(内部申请内存)
Output:    cloud.txt 点云数据
Description: 调用点云+解相模块，控制台输出耗时信息
Return:    失败返回-1，成功返回0
Others:    NULL
***************************************************************************************/
double *gX = 0;
double *gY = 0;
double *gZ = 0;
double *gNx = 0;
double *gNy = 0;
double *gNz = 0;

gxbPacking0913DLL_API int gxbGetPointCloud(unsigned char**pImgL, unsigned char**pImgR, double *&X, double *&Y, double *&Z, double *&Nx, double *&Ny, double *&Nz)
{
	///记得释放
	double *phasel = new double[W*H];
	double *phaser = new double[W*H];
	///释放
	unsigned char *Img17L = new unsigned char[W*H];
	unsigned char *Img17R = new unsigned char[W*H];
	memcpy(Img17L, pImgL[16], W*H);
	memcpy(Img17R, pImgR[16], W*H);
	//计时开始
	clock_t start = clock();
	int time1 = 0, time2 = 0, time3 = 0;
	//解相位
	{
		phaseResult(pImgL, phasel);
		phaseResult(pImgR, phaser);
	}
	//计时结束
	clock_t end = clock(); time1 = end - start; std::cout << "解相位耗时   " << end - start << "ms" << std::endl;

	//计时开始
	start = clock();
	//去除背景
	removeBg(pImgL[15], pImgL[16], phasel);
	removeBg(pImgR[15], pImgR[16], phaser);
	//计时结束
	end = clock(); time3 = end - start; std::cout << "去除背景耗时   " << end - start << "ms" << std::endl;
	//计时开始
	start = clock();
	//相位滤波
	phaseflitter(phasel);
	phaseflitter(phaser);
	//计时结束
	end = clock(); time2 = end - start; std::cout << "相位滤波耗时   " << end - start << "ms" << std::endl;
	std::cout << "解相位+滤波+去背景 总耗时   " << time1 + time2 + time3 << "ms" << std::endl << std::endl;
	///相位滤波
	fbPhaseFilter(phasel, 0.04, 50);
	fbPhaseFilter(phaser, 0.04, 50);
	///生成点云

	if (gX == 0)
	{
		gX = new double[W*H];
		gY = new double[W*H];
		gZ = new double[W*H];

		gNx = new double[W*H];
		gNy = new double[W*H];
		gNz = new double[W*H];
	}
	X = gX;
	Y = gY;
	Z = gZ;
	Nx = gNx;
	Ny = gNy;
	Nz = gNz;
	//计算点云
	fbPointCloud(phasel, phaser, X, Y, Z, Nx, Ny, Nz);


	int *judge = new int[W * H];
	int *judger = new int[W * H];
	memset(judge, 0, W * H);
	memset(judger, 0, W * H);
	//利用左相机进行边缘处理
	RegionGrow(pImgL,phasel, judge);
	for (int i = 0; i < H; i++)
	{
		for (int j = 0; j < W; j++)
		{
			int ij = i*W + j;
			if (judge[ij] == 1)
			{
				X[ij] = NAN;
				Y[ij] = NAN;
				Z[ij] = NAN;
			}
		}
	}
	memcpy(pImgL[1], Img17L, W*H);
	memcpy(pImgR[1], Img17R, W*H);
	cout << "已执行边缘处理" << endl;
	delete[]judge;
	delete[]judger;
	fbPointCloudFilter(X, Y, Z, Nx, Ny, Nz, 0.1, 50);
	std::cout << "hello_cloud" << std::endl;
	//保存点云
	if (1)
	{
		fbSaveData("cloud.txt", W*H, X, Y, Z, Nx, Ny, Nz);
	}
	std::cout << "hello_save" << std::endl;
	//写入二进制文件
	ofstream rs("cloud.bin", ios::binary);
	for (int i = 0; i<W*H; i++)
	{
		rs.write((char*)(&X[i]), sizeof(X[i]));//将数据写到二进制文件
		rs.write((char*)(&Y[i]), sizeof(Y[i]));//将数据写到二进制文件
		rs.write((char*)(&Z[i]), sizeof(Z[i]));//将数据写到二进制文件
	}
	rs.close();
	delete[]phaser;
	delete[]phasel;

	if(0)
	{
		for (int i = 0; i < 17; i++)
		{
			delete[]pImgR[i];
			delete[]pImgL[i];
		}
		delete[]pImgL;
		delete[]pImgR;
		delete[]X;
		delete[]Y;
		delete[]Z;
		delete[]Nx;
		delete[]Ny;
		delete[]Nz;
	}

	return 0;
}
gxbPacking0913DLL_API int saveCloud(double *&X, double *&Y, double *&Z, double *&Nx, double *&Ny, double *&Nz)
{
	fbSaveData("cloud.txt", W*H, X, Y, Z, Nx, Ny, Nz);
	return 0;
}
