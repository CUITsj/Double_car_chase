#include "rudder.h"

int S3010_DUTY = S3010_MID;  

void S3010_pid()//�������
{
  int D_value = 40 - road_mid; //�����е��ͼ���е�Ĳ�ֵ
  
  if(road_mid > 50 || road_mid < 30)
  {
    S3010_DUTY = S3010_MID + D_value * 28; //40��28
  }
  else
  {
    S3010_DUTY = S3010_MID + D_value * 20;
  }
  
  if(S3010_DUTY > S3010_MID+700)
  {
    S3010_DUTY = S3010_MID+700;
  }
  if(S3010_DUTY < S3010_MID-700)
  {
    S3010_DUTY = S3010_MID-700;
  }
}