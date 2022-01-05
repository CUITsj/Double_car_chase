#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "common.h"
#include "include.h"

extern uint8 start_flag;   //���ڼ��������
extern uint8 check_flag;   //���ڼ��������
extern uint8 road_mid;     //ͼ�������յõ��������е�
extern uint8 circle_count;   //����Բ������
extern uint8 front_flag;    //�жϳ���ǰ���ں�
extern uint8 left_in_circle_flag; //���뻷��־
extern uint8 right_in_circle_flag; //���뻷��־
extern uint8 img[]; 


uint8 Image_analyze(uint8 *img);                                //ͼ����
uint8 leftout_circle(uint8 *img);                               //����Բ����Բ��ר�ú���
uint8 rightout_circle(uint8 *img);                              //����Բ����Բ��ר�ú���
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);        //��ѹ

#endif