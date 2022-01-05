#include "common.h"
#include "include.h"
//2
uint8 img[CAMERA_W*CAMERA_H]; //����װ��ѹ���ͼ�񣬼�����ͼ�������ͼ��
uint8 *imgbuff = (uint8 *)(((uint8 *)&nrf_tx_buff) + COM_LEN);	// ���ڴ洢NRF�����ͼ��

uint8 road_mid = 40;     //ͼ������õ��������е�
uint8 circle_count = 0;  //Բ������
uint8 behind_in_circle_flag = 0;        //����Բ����־
uint8 front_in_circle_flag = 0;         //ǰ���뻷��־
uint8 dont_stop_flag = 0;               //��ͣ����־


uint8 front_flag = 1; //1��ʾ��ʼ��Ϊǰ����0��ʾ��ʼ��Ϊ��***
uint8 color = 1;   //�������ֲ�ͬ�����ɫ�ĳ���0��ʾ��ɫ��1��ʾ��ɫ����Ϊ�������ı�������ͬ����ͣ��PID��ͬ***

uint8 front_circle_speed = 100;         //ǰ���뻷�ٶ�***

uint8 behind_circle_speed = 140;        //���뻷�ٶ�***
  
/*********** main����***********/
void main(void)
{
  uint8 Rxbuff[DATA_PACKET] = {0};
  uint8 Txbuff[DATA_PACKET] = {0};
  
  check_flag = 1;
  System_Init();	        //��ʼ������ģ��
  check_flag = 0;             //�����ڿ�ʼ��ʱ3���������
  while(1)               //�ȴ�����һ�����ݰ������ݴ洢��Rxbuff��
  {
    camera_get_img();                            // ����ͷ��ȡͼ��
    while (!Image_Flag);                        // �ɼ����
    Image_Flag = 0;
    img_extract(img, imgbuff,CAMERA_SIZE);	// ��ѹ
    road_mid = Image_analyze(img);
    S3010_pid();
    ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//���¶������PWM 
  }
  while(1)
  {
    /*************************************************************** ǰ������ **************************************************/
    if (front_flag == 1)
    {
      /************************ ͼ��ɼ�����ʾ  ***********************/
      camera_get_img();                            // ����ͷ��ȡͼ��
      while (!Image_Flag);                        // �ɼ����
      Image_Flag = 0;
      img_extract(img, imgbuff,CAMERA_SIZE);	// ��ѹ
      
      /************************* ͼ���㷨���� ************************/ 
      road_mid = Image_analyze(img);
      S3010_pid();
      ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//���¶������PWM 
      
      if (front_in_circle_flag == 1)//�Ѿ���⵽һ����Բ����Ҫ��֤��ȫ��Բ���ټ����Բ��,������ȫ����ǰ��⵽���뻷
      {
        left_in_circle_flag = 0;
        right_in_circle_flag = 0;
      }
      
      //ǰ��ʶ������Բ��
      if (left_in_circle_flag == 1)
      {
        front_in_circle_flag = 1;//ǰ��ʶ��һ�ν���Բ�����ͺ���֮����뻷���ֱ����ȫ����
        //���ܽ�left_in_circle_flag��־������������Ϊ�ڽ��뻷·�����һ��ʱ�䣬ֻ�е���ȫ����ʱ������
        Set_speed = front_circle_speed;      //ǰ����Բ���ͼ����Ա�ͣ��
        //����Բ����ʼ����Բ��
        //��һ������⵽�����뻷ʱ�ĳ�����־
        while (leftout_circle(img))
        {
          camera_get_img();                            // ����ͷ��ȡͼ��
          while (!Image_Flag);                        // �ɼ����
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// ��ѹ
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//���¶������PWM 
          
          if (nrf_rx(Rxbuff,DATA_PACKET) != 0)          //˵���յ���Ϣ
          {
            if (Rxbuff[20] == 0xff)                           //�յ���ȷ��Ϣ���ָ��ٶ�
            {
              Rxbuff[20] = 0x00; 
              dont_stop_flag = 1;
            }
          }
        }
        //�ڶ�������ȫ����Բ������ⲻ��������־
        while (!leftout_circle(img))
        {
          camera_get_img();                            // ����ͷ��ȡͼ��
          while (!Image_Flag);                        // �ɼ����
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// ��ѹ
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//���¶������PWM 
          
          if (nrf_rx(Rxbuff,DATA_PACKET) != 0)          //˵���յ���Ϣ
          {
            if (Rxbuff[20] == 0xff)                           //�յ���ȷ��Ϣ���ָ��ٶ�
            {
              Rxbuff[20] = 0x00; 
              dont_stop_flag = 1;
            }
          }
        }
        //���������ٴμ�⵽������־�����������ĳ�����־
        if (dont_stop_flag == 0)        //��û�յ���Ϣ��ͣ���ȴ���Ϣ
        {
          Set_speed = 0;
          while(nrf_rx(Rxbuff,DATA_PACKET) == 0)               //�ȴ�����һ�����ݰ������ݴ洢��Rxbuff��
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
          if (Rxbuff[20] == 0xff)                           //�յ���ȷ��Ϣ���ָ��ٶ�
          {
            Rxbuff[20] = 0x00;    
            Set_speed = (int)(CMD_hmi_flash >> 24);       //�ָ������ٶ�
            delay_to_out_circle_flag = 1;    //�����ڿ�ʼ��ʱ��ֱ����ȫ��Բ�����ſ�ʼ�����Բ��������ǰ����Ϊ�󳵣�Բ������һ
            //��ʱֱ����ȫ�����ڼ䡰���Ա��֡���Բ����־Ϊ0����ֹ����·ʱ�ٴ�ʶ���뻷
          }    
        }
        else if (dont_stop_flag == 1)   //�Ѿ��յ���Ϣ������Ҫͣ�����ָ��ٶ�
        {
          dont_stop_flag = 0;
          Set_speed = (int)(CMD_hmi_flash >> 24);       //�ָ������ٶ�
          delay_to_out_circle_flag = 1;    //�����ڿ�ʼ��ʱ��ֱ����ȫ��Բ�����ſ�ʼ�����Բ��������ǰ����Ϊ�󳵣�Բ������һ
          //��ʱֱ����ȫ�����ڼ䡰���Ա��֡���Բ����־Ϊ0����ֹ����·ʱ�ٴ�ʶ���뻷
        }
      }
      
      //ǰ��ʶ������Բ��
      else if (right_in_circle_flag == 1)
      {
        front_in_circle_flag = 1;//ǰ��ʶ��һ�ν���Բ�����ͺ���֮����뻷���ֱ����ȫ����
        //���ܽ�left_in_circle_flag��־������������Ϊ�ڽ��뻷·�����һ��ʱ�䣬ֻ�е���ȫ����ʱ������
        Set_speed = front_circle_speed;        //ǰ����Բ���ͼ����Ա�ͣ��
        //��⵽��Բ��ʱ��ʼ����Բ��
        //��һ������⵽�����뻷ʱ�ĳ�����־
        while (rightout_circle(img))
        {
          camera_get_img();                            // ����ͷ��ȡͼ��
          while (!Image_Flag);                        // �ɼ����
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// ��ѹ
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//���¶������PWM 
          
          if (nrf_rx(Rxbuff,DATA_PACKET) != 0)          //˵���յ���Ϣ
          {
            if (Rxbuff[20] == 0xff)                           //�յ���ȷ��Ϣ���ָ��ٶ�
            {
              Rxbuff[20] = 0x00; 
              dont_stop_flag = 1;
            }
          }
        }
        //�ڶ�������ȫ����Բ������ⲻ��������־
        while (!rightout_circle(img))
        {
          camera_get_img();                            // ����ͷ��ȡͼ��
          while (!Image_Flag);                        // �ɼ����
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// ��ѹ
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//���¶������PWM 
          
          if (nrf_rx(Rxbuff,DATA_PACKET) != 0)          //˵���յ���Ϣ
          {
            if (Rxbuff[20] == 0xff)                           //�յ���ȷ��Ϣ���ָ��ٶ�
            {
              Rxbuff[20] = 0x00; 
              dont_stop_flag = 1;
            }
          }
        }
        //���������ٴμ�⵽������־�����������ĳ�����־
        if (dont_stop_flag == 0)        //��û�յ���Ϣ��ͣ���ȴ���Ϣ
        {
          Set_speed = 0;
          while(nrf_rx(Rxbuff,DATA_PACKET) == 0)               //�ȴ�����һ�����ݰ������ݴ洢��Rxbuff��
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
          if (Rxbuff[20] == 0xff)                           //�յ���ȷ��Ϣ���ָ��ٶ�
          {
            Rxbuff[20] = 0x00;    
            Set_speed = (int)(CMD_hmi_flash >> 24);       //�ָ������ٶ�
            delay_to_out_circle_flag = 1;    //�����ڿ�ʼ��ʱ��ֱ����ȫ��Բ�����ſ�ʼ�����Բ��������ǰ����Ϊ�󳵣�Բ������һ
            //��ʱֱ����ȫ�����ڼ䡰���Ա��֡���Բ����־Ϊ0����ֹ����·ʱ�ٴ�ʶ���뻷
          }    
        }
        else if (dont_stop_flag == 1)   //�Ѿ��յ���Ϣ������Ҫͣ�����ָ��ٶ�
        {
          dont_stop_flag = 0;
          Set_speed = (int)(CMD_hmi_flash >> 24);       //�ָ������ٶ�
          delay_to_out_circle_flag = 1;    //�����ڿ�ʼ��ʱ��ֱ����ȫ��Բ�����ſ�ʼ�����Բ��������ǰ����Ϊ�󳵣�Բ������һ
          //��ʱֱ����ȫ�����ڼ䡰���Ա��֡���Բ����־Ϊ0����ֹ����·ʱ�ٴ�ʶ���뻷
        }         
      }    
    }
    
    /************************************************************ �󳵴��� *************************************************************/
    else if (front_flag == 0)
    {
      /************************ ͼ��ɼ�����ʾ  ***********************/
      camera_get_img();                            // ����ͷ��ȡͼ��
      while (!Image_Flag);                        // �ɼ����
      Image_Flag = 0;
      img_extract(img, imgbuff,CAMERA_SIZE);	// ��ѹ
      
      /************************* ͼ���㷨���� ************************/ 
      road_mid = Image_analyze(img);
      S3010_pid();
      ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//���¶������PWM 
      
      if (behind_in_circle_flag == 1)  //�Ѿ���⵽һ����Բ����Ҫ��֤��ȫ��Բ���ټ����Բ����������ȫ����ǰ��⵽���뻷
      {
        left_in_circle_flag = 0;
        right_in_circle_flag = 0;
      }
      
      //��ʶ������Բ��
      if (left_in_circle_flag == 1)
      {
        behind_in_circle_flag = 1;
        //���ܽ�left_in_circle_flag��־������������Ϊ�ڽ��뻷·�����һ��ʱ�䣬ֻ�е���ȫ����ʱ������
        Set_speed = behind_circle_speed;      //����Բ���ͼ��ٷ�ֹ�������
        //����Բ����ʼ����Բ��
        //��һ������⵽�����뻷ʱ�ĳ�����־
        while (leftout_circle(img))
        {
          camera_get_img();                            // ����ͷ��ȡͼ��
          while (!Image_Flag);                        // �ɼ����
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// ��ѹ
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//���¶������PWM 
        }
        //�ڶ�������ȫ����Բ������ⲻ��������־
        while (!leftout_circle(img))
        {
          camera_get_img();                            // ����ͷ��ȡͼ��
          while (!Image_Flag);                        // �ɼ����
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// ��ѹ
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//���¶������PWM 
        }
        //���������ٴμ�⵽������־�����������ĳ�����־
        //��ʼ��ʱһ��ʱ�䷢��Ϣ
        delay_sendMSG_flag = 1;         //��⵽��Բ�����ӳ�һ��ʱ�䱣֤��ȫ��Բ��һ�ξ����ٷ���Ϣ����
      }
      //��ʶ������Բ��
      else if (right_in_circle_flag == 1)
      {
        behind_in_circle_flag = 1;
        //���ܽ�left_in_circle_flag��־������������Ϊ�ڽ��뻷·�����һ��ʱ�䣬ֻ�е���ȫ����ʱ������
        Set_speed = behind_circle_speed;        //����Բ���ͼ��ٷ�ֹ�������
        //��⵽��Բ��ʱ��ʼ����Բ��
        //��һ������⵽�����뻷ʱ�ĳ�����־
        while (rightout_circle(img))
        {
          camera_get_img();                            // ����ͷ��ȡͼ��
          while (!Image_Flag);                        // �ɼ����
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// ��ѹ
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//���¶������PWM 
        }
        //�ڶ�������ȫ����Բ������ⲻ��������־
        while (!rightout_circle(img))
        {
          camera_get_img();                            // ����ͷ��ȡͼ��
          while (!Image_Flag);                        // �ɼ����
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// ��ѹ
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//���¶������PWM 
        }
        //���������ٴμ�⵽������־�����������ĳ�����־
        //��ʼ��ʱһ��ʱ�䷢��Ϣ
        delay_sendMSG_flag = 1;         //��⵽��Բ�����ӳ�һ��ʱ�䱣֤��ȫ��Բ��һ�ξ����ٷ���Ϣ����
      }
      
      if (start_sendMSG_flag == 1)//�߼��󳵿�ʼ����Ϣ���߼�ǰ��
      {
        Txbuff[20] = 0xff;
        //�˴����붨ʱһ��ʱ�䣬Ȼ����NRF��Ϣʹ������
        
        behind_in_circle_flag = 0; //���Ѿ���ȫ��Բ��
        
        start_sendMSG_flag = 0;         //ֻ����һ����Ϣ
        
        Set_speed = (int)(CMD_hmi_flash >> 24);  //�ָ������ٶ�
        
        if(nrf_tx(Txbuff,DATA_PACKET) == 1) 
        {
          while(nrf_tx_state() == NRF_TXING);         //�ȴ��������
          //�󳵷�����Ϣ�ɹ�
          front_flag = 1; //��Ϊ�߼�ǰ��
          circle_count++;//һ��Բ����һ�γ� 
        }
      }
    }
    if (circle_count >= 4)
    {
      circle_count = 0;
    }  
    /************************ ���߷��ͺͽ�������  ***********************/
    //nrf_msg_tx(COM_IMG,nrf_tx_buff);
    //while(nrf_tx_state() == NRF_TXING);             //�ȴ��������
  }
}
