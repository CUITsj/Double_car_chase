#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "common.h"
#include "include.h"

extern uint8 start_flag;   //用于检测起跑线
extern uint8 check_flag;   //用于检测起跑线
extern uint8 road_mid;     //图像处理最终得到的赛道中点
extern uint8 circle_count;   //用于圆环计数
extern uint8 front_flag;    //判断车在前或在后
extern uint8 left_in_circle_flag; //左入环标志
extern uint8 right_in_circle_flag; //右入环标志
extern uint8 img[]; 


uint8 Image_analyze(uint8 *img);                                //图像处理
uint8 leftout_circle(uint8 *img);                               //左入圆环出圆环专用函数
uint8 rightout_circle(uint8 *img);                              //右入圆环出圆环专用函数
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);        //解压

#endif