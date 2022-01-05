/*!
*     COPYRIGHT NOTICE
*     Copyright (c) 2013,ɽ��Ƽ�
*     All rights reserved.
*     �������ۣ�ɽ����̳ http://www.vcan123.com
*
*     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
*     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
*
* @file       MK60_it.c
* @brief      ɽ��K60 ƽ̨�жϷ�����
* @author     ɽ��Ƽ�
* @version    v5.0
* @date       2013-06-26
*/
#include    "MK60_it.h"

uint8 check_flag = 1;//���ڼ�������ߵı�־����
uint8 delay_to_out_circle_flag = 0;
uint8 delay_sendMSG_flag = 0;
uint8 start_sendMSG_flag = 0;


/*********************************�жϷ�����********************************/

/*!
*  @brief      PORTA�жϷ�����
*  @since      v5.0
*/
void PORTA_IRQHandler()
{
  uint8  n;    //���ź�
  uint32 flag;
  
  flag = PORTA_ISFR;
  PORTA_ISFR  = ~0;                                   //���жϱ�־λ
  
  n = 29;                                             //���ж�
  if(flag & (1 << n))                                 //PTA29�����ж�
  {
    camera_vsync();
  }
#if ( CAMERA_USE_HREF == 1)                            //ʹ�����ж�
  n = 28;
  if(flag & (1 << n))                                 //PTA28�����ж�
  {
    camera_href();
  }
#endif
}

/*!
*  @brief      PORTE�жϷ�����
*  @since      v5.0
*/
void PORTE_IRQHandler()
{
  uint8  n;    //���ź�
  uint32 flag;
  
  flag = PORTE_ISFR;
  PORTE_ISFR  = ~0;                                   //���жϱ�־λ
  
  n = 27;
  if(flag & (1 << n))                                 //PTE27�����ж�
  {
    nrf_handler();
  }
}

/*!
*  @brief      DMA0�жϷ�����
*  @since      v5.0
*/
void DMA0_IRQHandler()
{
  camera_dma();
}

/*!
*  @brief      PIT0�жϷ�����
*  @since      v5.0
*/
void PIT0_IRQHandler()	// 10ms��һ���ж�
{
  MOTOR_measure();
  MOTOR_pid(Set_speed);  
  ftm_pwm_duty(MOTOR_FTM, MOTOR1_CH, MOTOR1_DUTY);//����ǰ
  ftm_pwm_duty(MOTOR_FTM, MOTOR2_CH, MOTOR2_DUTY);//���ֺ�  
  ftm_pwm_duty(MOTOR_FTM, MOTOR3_CH, MOTOR3_DUTY);//����ǰ
  ftm_pwm_duty(MOTOR_FTM, MOTOR4_CH, MOTOR4_DUTY);//���ֺ� 
//  if((!MOTOR1_speed || !MOTOR2_speed) && Set_speed != 0 && check_flag == 1 && behind_check_in_circle == 1)//��ֹ��ת
//  {
//    Set_speed = 0;
//  }  
  PIT_Flag_Clear(PIT0);//�����ʱ���жϱ�־λ
}


/*!
*  @brief      PIT1�жϷ�����
*  @since      v5.0
*/
void PIT1_IRQHandler()
{
  static uint8 delay_stop_ms0 = 0;  //��
  static uint8 delay_stop_ms1 = 0;  //ǰ��
  static uint8 delay_to_out_circle_time = 0;
  static uint8 delay_sendMSG_time = 0;
  
  if (check_flag == 1 && start_flag > 0 && front_flag == 0)//��ͣ��
  {
    delay_stop_ms0++;
    if (delay_stop_ms0 == 6)  //ʶ�������ߺ��ӳ�50ms��0ռ�ձ�
    {
      delay_stop_ms0 = 0;  
      while(1)
      {
        Set_speed = 0;
        camera_get_img();                            // ����ͷ��ȡͼ��
        while (!Image_Flag);                        // �ɼ����
        Image_Flag = 0;
        img_extract(img, imgbuff,CAMERA_SIZE);	// ��ѹ
        road_mid = Image_analyze(img);
        S3010_pid();
        ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//���¶������PWM 
      }
    }
  }
  if (check_flag == 1 && start_flag > 0 && front_flag == 1)//ǰ��ͣ��
  {
    delay_stop_ms1++;
    if (delay_stop_ms1 == 17)  //ʶ��������ǰ���ӳ�100ms��0ռ�ձ�
    {
      delay_stop_ms1 = 0;
      while(1)
      {
        Set_speed = 0;
        camera_get_img();                            // ����ͷ��ȡͼ��
        while (!Image_Flag);                        // �ɼ����
        Image_Flag = 0;
        img_extract(img, imgbuff,CAMERA_SIZE);	// ��ѹ
        road_mid = Image_analyze(img);
        S3010_pid();
        ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//���¶������PWM 
      }
    } 
  }
  if (delay_to_out_circle_flag == 1)  //��ʱ��ֱ����ȫ��Բ�����ſ�ʼ�����Բ��������ǰ����Ϊ�󳵣�Բ������һ
  {
    delay_to_out_circle_time++;
    if (delay_to_out_circle_time == 60)
    {
      led_turn(LED_MAX);
      delay_to_out_circle_flag = 0;
      delay_to_out_circle_time = 0;
      front_in_circle_flag = 0;         //ǰ���Ѿ���ȫ��Բ������ʼ�����Բ��
      front_flag = 0;//����ǰ����Ϊ��
      circle_count++;
    }
  }
  if (delay_sendMSG_flag == 1)//�ӳ�һ��ʱ�䱣֤��ȫ��Բ��һ�ξ����ٷ���Ϣ���󳵣���ͨ������ʱ��������������
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
  
  PIT_Flag_Clear(PIT1);//�����ʱ���жϱ�־λ
}

/*!
*  @brief      PIT2�жϷ�����
*  @since      v5.0
*/
void PIT2_IRQHandler()
{
  static uint8 time_ns = 0; //��ʱn��

  if(check_flag == 0)
  {
    time_ns++;
  }
  //3�������
  if(time_ns == 30)
  {
    Set_speed = (int)(CMD_hmi_flash >> 24);
  }
  //��5������������߼��
  if(time_ns == 80)
  {
    time_ns = 0;
    start_flag = 0;
    check_flag = 1;
  }
  
  PIT_Flag_Clear(PIT2);//�����ʱ���жϱ�־λ
}





