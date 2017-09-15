// findContours.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <opencv2\opencv.hpp>
#include <vector>
#include <iostream>
using namespace std;
using namespace cv;
int w = 1024, h = 768;
int *judge = new int [w*h];
int RegionGrow(unsigned char *Img, unsigned char *&result,int w,int h);
int main()
{
	memset(judge, 0, w*h);
	unsigned char *result = new unsigned char[w*h];
	Mat img = imread("right.bmp",IMREAD_GRAYSCALE);
	
	RegionGrow(img.data, result,w,h);
	Mat Mask = Mat(h, w, CV_8UC1, result);
	imshow("right1", img);
	imshow("Mask", Mask);
	imwrite("Mask.bmp", Mask);
	waitKey(-1);
	return 0;
}
int RegionGrow(unsigned char *Img,unsigned char *&result,int w,int h)
{
	memset(result, 0, w*h);
	Point2i pt = Point2i(1, 1);
	Point2i ptGrowing;                              //待生长点位置
	int nGrowLable = 0;                             //标记是否生长过
	int nSrcValue = 0;                              //生长起点灰度值
	int nCurValue = 0;                              //当前生长点灰度值
	
													//生长方向顺序数据
	int DIR[8][2] = { { -1,-1 },{ 0,-1 },{ 1,-1 },{ 1,0 },{ 1,1 },{ 0,1 },{ -1,1 },{ -1,0 } };
	vector<Point2i> vcGrowPt;						//生长点栈 
	vector<Point2i> vcSize;
	for (int p = 1; p < h-1; p++)
	{
		for (int k = 1; k < w-1; k++)
		{
			if (Img[p*w+k] == 255 && result[p*w+k] == 0)//种子点满足要求且未被访问过
			{
				pt = Point2i(k, p);
				vcGrowPt.push_back(pt);//将生长点压入栈中 
				vcSize.push_back(pt);//记录生长区域的面积用于判断

				result[pt.y*w+ pt.x] = 255;               //标记生长点  
				nSrcValue = Img[pt.y*w+ pt.x];            //记录生长点的灰度值  

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
						if (ptGrowing.x < 0 || ptGrowing.y < 0 || ptGrowing.x >(w - 1) || (ptGrowing.y > h - 1))
							continue;

						nGrowLable = result[ptGrowing.y*w+ ptGrowing.x];      //当前待生长点的灰度值  

						if (nGrowLable == 0)                    //如果标记点还没有被生长  
						{
							nCurValue = Img[ptGrowing.y*w + ptGrowing.x];
							//cout << "生长点:" << ptGrowing << "  灰度值:" << nCurValue << endl;
							if (nCurValue == 255)                  //在阈值范围内则生长  
							{
								result[ptGrowing.y*w +ptGrowing.x] = 255;     //标记为白色  
								judge[ptGrowing.y*w+ptGrowing.x] = 1;
								vcGrowPt.push_back(ptGrowing);                  //将下一个生长点压入栈中
								vcSize.push_back(ptGrowing);
							}
						}
					}
				}
				//如果生长区域的像素少于20个，则放弃该边缘，蒙版Mask的相关区域重新归0
				if (vcSize.size() < 20)
				{
					for (int i = 0; i < vcSize.size(); i++)
					{
						judge[vcSize[i].y*w+vcSize[i].x] = 0;
					}
				}
				vcSize.clear();
			}
			else
				continue;
		}
	}
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			int ij = i*w + j;
			if (judge[ij] == 1)
				result[ij] = 255;
			else 
				result[ij] = 0;

		}
	}
	return 0;
}
