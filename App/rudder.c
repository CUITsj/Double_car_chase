#include "rudder.h"

int S3010_DUTY = S3010_MID;  

void S3010_pid()//舵机调节
{
  int D_value = 40 - road_mid; //赛道中点和图像中点的差值
  
  if(road_mid > 50 || road_mid < 30)
  {
    S3010_DUTY = S3010_MID + D_value * 28; //40改28
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