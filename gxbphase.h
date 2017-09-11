#pragma once

/*
初始化
*/
int gxbInit(int w, int h, int step);
/*====================================
解相位
Img：15张1024*768，三频五步相移图片
phase：绝对相位，大小1024*768
====================================*/
int gxbphaseResult(unsigned char **Img, double *phase);
/*====================================
生成投影图像
index：编号（1-17）
p1,p2,p3:11,12,13
data:854*480
====================================*/
int gxbcreateProjImg(int index, unsigned char*data);
/*====================================
相位滤波
phaseL,phaseR位解得的绝对相位
====================================*/
int gxbphaseflitter(double *PHASE);
/*====================================
去除背景
bgImgb:黑
bgImgw:白
====================================*/
int gxbremoveBg(unsigned char *bgImgb, unsigned char *bgImgw, double *PHASE);

//
int gxbGetPhase(unsigned char**img, double *phase);


