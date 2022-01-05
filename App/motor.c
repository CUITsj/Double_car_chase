#include "motor.h"

int16 MOTOR1_speed = 0;//编码器测试左轮速度
int16 MOTOR2_speed = 0;//编码器测试右轮速度

int Set_speed = 0; //设定的电机速度

int MOTOR1_DUTY = 0;
int MOTOR2_DUTY = 0;
int MOTOR3_DUTY = 0;
int MOTOR4_DUTY = 0;

void MOTOR_measure()
{
  int16 Pulses1, Pulses2;

  Pulses1 = ftm_quad_get(FTM1);
  ftm_quad_clean(FTM1);
  Pulses2 = ftm_quad_get(FTM2);
  ftm_quad_clean(FTM2);

  MOTOR1_speed = Pulses1; 
  MOTOR2_speed = -Pulses2;
}

void MOTOR_pid(int16 SetSpeed)
{
  int NowSpeed1 = MOTOR1_speed, NowSpeed2 = MOTOR2_speed;
  int SetSpeed1 = 0, SetSpeed2 = 0;
  int left_duty = 0, right_duty = 0;
  int iError1 = 0, iError2 = 0;
  static float sum_error1 = 0,sum_error2 = 0;
  static int LastError1 = 0;
  static int LastError2 = 0;

  if(!SetSpeed)
  {
    stop_pid();
    return;
  }
//左右轮差速算法
  if(road_mid < 40)
  {
    SetSpeed1 = (int)(SetSpeed - ((float)(40 - road_mid) * 4.5));
    SetSpeed2 = SetSpeed;
  }
  if(road_mid > 40)
  {
    SetSpeed1 = SetSpeed;
    SetSpeed2 = (int)(SetSpeed - ((float)(road_mid - 40) * 4.5));
  }
  if(road_mid == 40)
  {
    SetSpeed1 = SetSpeed;
    SetSpeed2 = SetSpeed;
  }
//左轮PID调节
  
//1当前差、差积分、限幅
  iError1 = (SetSpeed1 - NowSpeed1);
  sum_error1 = sum_error1 + iError1;
  if (sum_error1 > 10000) 
  {
    sum_error1 = 10000;
  }
  if (sum_error1 < -10000) 
  {
    sum_error1 = -10000;
  }
  
//2速度差大于-80进行正常PID，否则直接满转减速，（减速更迅猛）
  if(iError1 > -50)
  {
    left_duty = iError1 * 100 + (int32)(sum_error1 * 0.5) + (iError1 - LastError1) * 0;
  }
  else
  {
    left_duty = -9999;
  }
  
//3限幅
  if (left_duty > 9999)
  {
    left_duty = 9999;
  }
  if (left_duty < -9999) 
  {
    left_duty = -9999; 
  }

//4记录上一次差
  LastError1 = iError1;

//5输出
  if(left_duty >= 0)
  {
    MOTOR1_DUTY = left_duty;
    MOTOR2_DUTY = 0;
  }
  else
  {
    MOTOR1_DUTY = 0;
    MOTOR2_DUTY = -left_duty;
  }
  
//右轮
  
//1当前差、差积分、限幅
  iError2 = (SetSpeed2 - NowSpeed2);
  sum_error2 = sum_error2 + iError2;
  if (sum_error2 > 10000)
  {
    sum_error2 = 10000;
  }
  if (sum_error2 < -10000)
  {
    sum_error2 = -10000;
  }
  
//2速度差绝大于-80进行正常PID，否则直接满转减速，减速更迅猛
  if(iError2 > -80)
  {
    right_duty = iError2 * 100 + (int32)(sum_error2 * 0.5) + (iError2 - LastError2) * 0;
  }
  else
  {
    right_duty = -9999;
  }

//3限幅  
  if (right_duty > 9999)
  {
    right_duty = 9999;
  }
  if (right_duty < -9999)
  { 
    right_duty = -9999;
  }
  
//4记录上一次差
  LastError2 = iError2;
  
//5输出  
  if(right_duty >= 0)
  {
    MOTOR3_DUTY = right_duty;
    MOTOR4_DUTY = 0;
  }
  else
  {
    MOTOR3_DUTY = 0;
    MOTOR4_DUTY = -right_duty;
  }
}

void stop_pid()
{
  int NowSpeed1 = MOTOR1_speed, NowSpeed2 = MOTOR2_speed;
  int SetSpeed1 = 0, SetSpeed2 = 0;
  int left_duty = 0, right_duty = 0;
  int iError1 = 0, iError2 = 0;
  static float sum_error1 = 0,sum_error2 = 0;
  static int LastError1 = 0;
  static int LastError2 = 0;

  //左轮PID调节
  
//1当前差、差积分、限幅
  iError1 = (SetSpeed1 - NowSpeed1);
  sum_error1 = sum_error1 + iError1;
  if (sum_error1 > 1000) 
  {
    sum_error1 = 1000;
  }
  if (sum_error1 < -1000) 
  {
    sum_error1 = -1000;
  }
  
//2速度差大于-80进行正常PID，否则直接满转减速，（减速更迅猛）
  if(iError1 > -40)
  {
    left_duty = iError1 * 70 + (int32)(sum_error1 * 0.1) + (iError1 - LastError1) * 25;
  }
  else
  {
    left_duty = -9999;
  }
  
//3限幅
  if (left_duty > 9999)
  {
    left_duty = 9999;
  }
  if (left_duty < -9999) 
  {
    left_duty = -9999; 
  }

//4记录上一次差
  LastError1 = iError1;

//5输出
  if(left_duty >= 0)
  {
    MOTOR1_DUTY = left_duty;
    MOTOR2_DUTY = 0;
  }
  else
  {
    MOTOR1_DUTY = 0;
    MOTOR2_DUTY = -left_duty;
  }
  
  //右轮
  
  if (color == 1)  //红车
  {
    //1当前差、差积分、限幅
    iError2 = (SetSpeed2 - NowSpeed2);
    sum_error2 = sum_error2 + iError2;
    if (sum_error2 > 1000)
    {
      sum_error2 = 1000;
    }
    if (sum_error2 < -1000)
    {
      sum_error2 = -1000;
    }
    
    //2速度差绝大于-80进行正常PID，否则直接满转减速，减速更迅猛
    if(iError2 > -40)
    {
      right_duty = iError2 * 100 + (int32)(sum_error2 * 0.2) + (iError2 - LastError2) * 35;
    }
    else
    {
      right_duty = -9999;
    }
    
    //3限幅  
    if (right_duty > 9999)
    {
      right_duty = 9999;
    }
    if (right_duty < -9999)
    { 
      right_duty = -9999;
    }
    
    //4记录上一次差
    LastError2 = iError2;
    
    //5输出  
    if(right_duty >= 0)
    {
      MOTOR3_DUTY = right_duty;
      MOTOR4_DUTY = 0;
    }
    else
    {
      MOTOR3_DUTY = 0;
      MOTOR4_DUTY = -right_duty;
    }
  }
  else if (color == 0)  //黑车
  {
        //1当前差、差积分、限幅
    iError2 = (SetSpeed2 - NowSpeed2);
    sum_error2 = sum_error2 + iError2;
    if (sum_error2 > 1000)
    {
      sum_error2 = 1000;
    }
    if (sum_error2 < -1000)
    {
      sum_error2 = -1000;
    }
    
    //2速度差绝大于-80进行正常PID，否则直接满转减速，减速更迅猛
    if(iError2 > -40)
    {
      right_duty = iError2 * 70 + (int32)(sum_error2 * 0.1) + (iError2 - LastError2) * 25;
    }
    else
    {
      right_duty = -9999;
    }
    
    //3限幅  
    if (right_duty > 9999)
    {
      right_duty = 9999;
    }
    if (right_duty < -9999)
    { 
      right_duty = -9999;
    }
    
    //4记录上一次差
    LastError2 = iError2;
    
    //5输出  
    if(right_duty >= 0)
    {
      MOTOR3_DUTY = right_duty;
      MOTOR4_DUTY = 0;
    }
    else
    {
      MOTOR3_DUTY = 0;
      MOTOR4_DUTY = -right_duty;
    }
  }
}
