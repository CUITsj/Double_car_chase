/*!
*     COPYRIGHT NOTICE
*     Copyright (c) 2013,山外科技
*     All rights reserved.
*     技术讨论：山外论坛 http://www.vcan123.com
*
*     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
*     修改内容时必须保留山外科技的版权声明。
*
* @file       MK60_it.c
* @brief      山外K60 平台中断服务函数
* @author     山外科技
* @version    v5.0
* @date       2013-06-26
*/
#include    "MK60_it.h"

uint8 check_flag = 1;//用于检测起跑线的标志变量
uint8 delay_to_out_circle_flag = 0;
uint8 delay_sendMSG_flag = 0;
uint8 start_sendMSG_flag = 0;


/*********************************中断服务函数********************************/

/*!
*  @brief      PORTA中断服务函数
*  @since      v5.0
*/
void PORTA_IRQHandler()
{
  uint8  n;    //引脚号
  uint32 flag;
  
  flag = PORTA_ISFR;
  PORTA_ISFR  = ~0;                                   //清中断标志位
  
  n = 29;                                             //场中断
  if(flag & (1 << n))                                 //PTA29触发中断
  {
    camera_vsync();
  }
#if ( CAMERA_USE_HREF == 1)                            //使用行中断
  n = 28;
  if(flag & (1 << n))                                 //PTA28触发中断
  {
    camera_href();
  }
#endif
}

/*!
*  @brief      PORTE中断服务函数
*  @since      v5.0
*/
void PORTE_IRQHandler()
{
  uint8  n;    //引脚号
  uint32 flag;
  
  flag = PORTE_ISFR;
  PORTE_ISFR  = ~0;                                   //清中断标志位
  
  n = 27;
  if(flag & (1 << n))                                 //PTE27触发中断
  {
    nrf_handler();
  }
}

/*!
*  @brief      DMA0中断服务函数
*  @since      v5.0
*/
void DMA0_IRQHandler()
{
  camera_dma();
}

/*!
*  @brief      PIT0中断服务函数
*  @since      v5.0
*/
void PIT0_IRQHandler()	// 10ms进一次中断
{
  MOTOR_measure();
  MOTOR_pid(Set_speed);  
  ftm_pwm_duty(MOTOR_FTM, MOTOR1_CH, MOTOR1_DUTY);//左轮前
  ftm_pwm_duty(MOTOR_FTM, MOTOR2_CH, MOTOR2_DUTY);//左轮后  
  ftm_pwm_duty(MOTOR_FTM, MOTOR3_CH, MOTOR3_DUTY);//右轮前
  ftm_pwm_duty(MOTOR_FTM, MOTOR4_CH, MOTOR4_DUTY);//右轮后 
//  if((!MOTOR1_speed || !MOTOR2_speed) && Set_speed != 0 && check_flag == 1 && behind_check_in_circle == 1)//防止堵转
//  {
//    Set_speed = 0;
//  }  
  PIT_Flag_Clear(PIT0);//清除定时器中断标志位
}


/*!
*  @brief      PIT1中断服务函数
*  @since      v5.0
*/
void PIT1_IRQHandler()
{
  static uint8 delay_stop_ms0 = 0;  //后车
  static uint8 delay_stop_ms1 = 0;  //前车
  static uint8 delay_to_out_circle_time = 0;
  static uint8 delay_sendMSG_time = 0;
  
  if (check_flag == 1 && start_flag > 0 && front_flag == 0)//后车停车
  {
    delay_stop_ms0++;
    if (delay_stop_ms0 == 6)  //识别到起跑线后车延迟50ms给0占空比
    {
      delay_stop_ms0 = 0;  
      while(1)
      {
        Set_speed = 0;
        camera_get_img();                            // 摄像头获取图像
        while (!Image_Flag);                        // 采集完毕
        Image_Flag = 0;
        img_extract(img, imgbuff,CAMERA_SIZE);	// 解压
        road_mid = Image_analyze(img);
        S3010_pid();
        ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//更新舵机控制PWM 
      }
    }
  }
  if (check_flag == 1 && start_flag > 0 && front_flag == 1)//前车停车
  {
    delay_stop_ms1++;
    if (delay_stop_ms1 == 17)  //识别到起跑线前车延迟100ms给0占空比
    {
      delay_stop_ms1 = 0;
      while(1)
      {
        Set_speed = 0;
        camera_get_img();                            // 摄像头获取图像
        while (!Image_Flag);                        // 采集完毕
        Image_Flag = 0;
        img_extract(img, imgbuff,CAMERA_SIZE);	// 解压
        road_mid = Image_analyze(img);
        S3010_pid();
        ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//更新舵机控制PWM 
      }
    } 
  }
  if (delay_to_out_circle_flag == 1)  //延时，直到完全出圆环，才开始检测入圆环，并且前车变为后车，圆环数加一
  {
    delay_to_out_circle_time++;
    if (delay_to_out_circle_time == 60)
    {
      led_turn(LED_MAX);
      delay_to_out_circle_flag = 0;
      delay_to_out_circle_time = 0;
      front_in_circle_flag = 0;         //前车已经完全出圆环，开始检测入圆环
      front_flag = 0;//到此前车变为后车
      circle_count++;
    }
  }
  if (delay_sendMSG_flag == 1)//延迟一段时间保证完全出圆环一段距离再发消息给后车，并通过此延时来控制两车距离
  {
    delay_sendMSG_time++;
    if (delay_sendMSG_time == 20)
    {
      led_turn(LED_MAX);
      delay_sendMSG_flag = 0;
      delay_sendMSG_time = 0;
      start_sendMSG_flag = 1;
    }
  }
  
  PIT_Flag_Clear(PIT1);//清除定时器中断标志位
}

/*!
*  @brief      PIT2中断服务函数
*  @since      v5.0
*/
void PIT2_IRQHandler()
{
  static uint8 time_ns = 0; //定时n秒

  if(check_flag == 0)
  {
    time_ns++;
  }
  //3秒后启动
  if(time_ns == 30)
  {
    Set_speed = (int)(CMD_hmi_flash >> 24);
  }
  //再5秒后启动起跑线检测
  if(time_ns == 80)
  {
    time_ns = 0;
    start_flag = 0;
    check_flag = 1;
  }
  
  PIT_Flag_Clear(PIT2);//清除定时器中断标志位
}





