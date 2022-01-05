#include "common.h"
#include "include.h"
//2
uint8 img[CAMERA_W*CAMERA_H]; //用于装解压后的图像，即用于图像分析的图像
uint8 *imgbuff = (uint8 *)(((uint8 *)&nrf_tx_buff) + COM_LEN);	// 用于存储NRF传输的图像

uint8 road_mid = 40;     //图像分析得到的赛道中点
uint8 circle_count = 0;  //圆环计数
uint8 behind_in_circle_flag = 0;        //后车入圆环标志
uint8 front_in_circle_flag = 0;         //前车入环标志
uint8 dont_stop_flag = 0;               //不停车标志


uint8 front_flag = 1; //1表示初始化为前车，0表示初始化为后车***
uint8 color = 1;   //用来区分不同舵机颜色的车，0表示黑色，1表示红色，因为两个车的编码器不同导致停车PID不同***

uint8 front_circle_speed = 100;         //前车入环速度***

uint8 behind_circle_speed = 140;        //后车入环速度***
  
/*********** main函数***********/
void main(void)
{
  uint8 Rxbuff[DATA_PACKET] = {0};
  uint8 Txbuff[DATA_PACKET] = {0};
  
  check_flag = 1;
  System_Init();	        //初始化所有模块
  check_flag = 0;             //从现在开始定时3秒启动电机
  while(1)               //等待接收一个数据包，数据存储在Rxbuff里
  {
    camera_get_img();                            // 摄像头获取图像
    while (!Image_Flag);                        // 采集完毕
    Image_Flag = 0;
    img_extract(img, imgbuff,CAMERA_SIZE);	// 解压
    road_mid = Image_analyze(img);
    S3010_pid();
    ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//更新舵机控制PWM 
  }
  while(1)
  {
    /*************************************************************** 前车代码 **************************************************/
    if (front_flag == 1)
    {
      /************************ 图像采集和显示  ***********************/
      camera_get_img();                            // 摄像头获取图像
      while (!Image_Flag);                        // 采集完毕
      Image_Flag = 0;
      img_extract(img, imgbuff,CAMERA_SIZE);	// 解压
      
      /************************* 图像算法分析 ************************/ 
      road_mid = Image_analyze(img);
      S3010_pid();
      ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//更新舵机控制PWM 
      
      if (front_in_circle_flag == 1)//已经检测到一次入圆环，要保证完全出圆环再检测入圆环,忽略完全出环前检测到的入环
      {
        left_in_circle_flag = 0;
        right_in_circle_flag = 0;
      }
      
      //前车识别到左入圆环
      if (left_in_circle_flag == 1)
      {
        front_in_circle_flag = 1;//前车识别到一次进入圆环，就忽略之后的入环检测直到完全出环
        //不能将left_in_circle_flag标志清零放在这里，因为在进入环路会持续一段时间，只有当完全进入时在清零
        Set_speed = front_circle_speed;      //前车入圆环就减速以便停车
        //进入圆环开始检测出圆环
        //第一步，检测到的是入环时的出环标志
        while (leftout_circle(img))
        {
          camera_get_img();                            // 摄像头获取图像
          while (!Image_Flag);                        // 采集完毕
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// 解压
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//更新舵机控制PWM 
          
          if (nrf_rx(Rxbuff,DATA_PACKET) != 0)          //说明收到消息
          {
            if (Rxbuff[20] == 0xff)                           //收到正确消息，恢复速度
            {
              Rxbuff[20] = 0x00; 
              dont_stop_flag = 1;
            }
          }
        }
        //第二步，完全进入圆环，检测不到出环标志
        while (!leftout_circle(img))
        {
          camera_get_img();                            // 摄像头获取图像
          while (!Image_Flag);                        // 采集完毕
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// 解压
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//更新舵机控制PWM 
          
          if (nrf_rx(Rxbuff,DATA_PACKET) != 0)          //说明收到消息
          {
            if (Rxbuff[20] == 0xff)                           //收到正确消息，恢复速度
            {
              Rxbuff[20] = 0x00; 
              dont_stop_flag = 1;
            }
          }
        }
        //第三步，再次检测到出环标志，就是真正的出环标志
        if (dont_stop_flag == 0)        //还没收到消息，停车等待消息
        {
          Set_speed = 0;
          while(nrf_rx(Rxbuff,DATA_PACKET) == 0)               //等待接收一个数据包，数据存储在Rxbuff里
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
          if (Rxbuff[20] == 0xff)                           //收到正确消息，恢复速度
          {
            Rxbuff[20] = 0x00;    
            Set_speed = (int)(CMD_hmi_flash >> 24);       //恢复正常速度
            delay_to_out_circle_flag = 1;    //从现在开始延时，直到完全出圆环，才开始检测入圆环，并且前车变为后车，圆环数加一
            //延时直到完全出环期间“绝对保持”入圆环标志为0，防止出环路时再次识别到入环
          }    
        }
        else if (dont_stop_flag == 1)   //已经收到消息，不需要停车，恢复速度
        {
          dont_stop_flag = 0;
          Set_speed = (int)(CMD_hmi_flash >> 24);       //恢复正常速度
          delay_to_out_circle_flag = 1;    //从现在开始延时，直到完全出圆环，才开始检测入圆环，并且前车变为后车，圆环数加一
          //延时直到完全出环期间“绝对保持”入圆环标志为0，防止出环路时再次识别到入环
        }
      }
      
      //前车识别到右入圆环
      else if (right_in_circle_flag == 1)
      {
        front_in_circle_flag = 1;//前车识别到一次进入圆环，就忽略之后的入环检测直到完全出环
        //不能将left_in_circle_flag标志清零放在这里，因为在进入环路会持续一段时间，只有当完全进入时在清零
        Set_speed = front_circle_speed;        //前车入圆环就减速以便停车
        //检测到入圆环时开始检测出圆环
        //第一步，检测到的是入环时的出环标志
        while (rightout_circle(img))
        {
          camera_get_img();                            // 摄像头获取图像
          while (!Image_Flag);                        // 采集完毕
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// 解压
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//更新舵机控制PWM 
          
          if (nrf_rx(Rxbuff,DATA_PACKET) != 0)          //说明收到消息
          {
            if (Rxbuff[20] == 0xff)                           //收到正确消息，恢复速度
            {
              Rxbuff[20] = 0x00; 
              dont_stop_flag = 1;
            }
          }
        }
        //第二步，完全进入圆环，检测不到出环标志
        while (!rightout_circle(img))
        {
          camera_get_img();                            // 摄像头获取图像
          while (!Image_Flag);                        // 采集完毕
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// 解压
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//更新舵机控制PWM 
          
          if (nrf_rx(Rxbuff,DATA_PACKET) != 0)          //说明收到消息
          {
            if (Rxbuff[20] == 0xff)                           //收到正确消息，恢复速度
            {
              Rxbuff[20] = 0x00; 
              dont_stop_flag = 1;
            }
          }
        }
        //第三步，再次检测到出环标志，就是真正的出环标志
        if (dont_stop_flag == 0)        //还没收到消息，停车等待消息
        {
          Set_speed = 0;
          while(nrf_rx(Rxbuff,DATA_PACKET) == 0)               //等待接收一个数据包，数据存储在Rxbuff里
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
          if (Rxbuff[20] == 0xff)                           //收到正确消息，恢复速度
          {
            Rxbuff[20] = 0x00;    
            Set_speed = (int)(CMD_hmi_flash >> 24);       //恢复正常速度
            delay_to_out_circle_flag = 1;    //从现在开始延时，直到完全出圆环，才开始检测入圆环，并且前车变为后车，圆环数加一
            //延时直到完全出环期间“绝对保持”入圆环标志为0，防止出环路时再次识别到入环
          }    
        }
        else if (dont_stop_flag == 1)   //已经收到消息，不需要停车，恢复速度
        {
          dont_stop_flag = 0;
          Set_speed = (int)(CMD_hmi_flash >> 24);       //恢复正常速度
          delay_to_out_circle_flag = 1;    //从现在开始延时，直到完全出圆环，才开始检测入圆环，并且前车变为后车，圆环数加一
          //延时直到完全出环期间“绝对保持”入圆环标志为0，防止出环路时再次识别到入环
        }         
      }    
    }
    
    /************************************************************ 后车代码 *************************************************************/
    else if (front_flag == 0)
    {
      /************************ 图像采集和显示  ***********************/
      camera_get_img();                            // 摄像头获取图像
      while (!Image_Flag);                        // 采集完毕
      Image_Flag = 0;
      img_extract(img, imgbuff,CAMERA_SIZE);	// 解压
      
      /************************* 图像算法分析 ************************/ 
      road_mid = Image_analyze(img);
      S3010_pid();
      ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//更新舵机控制PWM 
      
      if (behind_in_circle_flag == 1)  //已经检测到一次入圆环，要保证完全出圆环再检测入圆，环忽略完全出环前检测到的入环
      {
        left_in_circle_flag = 0;
        right_in_circle_flag = 0;
      }
      
      //后车识别到左入圆环
      if (left_in_circle_flag == 1)
      {
        behind_in_circle_flag = 1;
        //不能将left_in_circle_flag标志清零放在这里，因为在进入环路会持续一段时间，只有当完全进入时在清零
        Set_speed = behind_circle_speed;      //后车入圆环就减速防止冲出赛道
        //进入圆环开始检测出圆环
        //第一步，检测到的是入环时的出环标志
        while (leftout_circle(img))
        {
          camera_get_img();                            // 摄像头获取图像
          while (!Image_Flag);                        // 采集完毕
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// 解压
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//更新舵机控制PWM 
        }
        //第二步，完全进入圆环，检测不到出环标志
        while (!leftout_circle(img))
        {
          camera_get_img();                            // 摄像头获取图像
          while (!Image_Flag);                        // 采集完毕
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// 解压
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//更新舵机控制PWM 
        }
        //第三步，再次检测到出环标志，就是真正的出环标志
        //开始延时一段时间发消息
        delay_sendMSG_flag = 1;         //检测到出圆环，延迟一段时间保证完全出圆环一段距离再发消息给后车
      }
      //后车识别到右入圆环
      else if (right_in_circle_flag == 1)
      {
        behind_in_circle_flag = 1;
        //不能将left_in_circle_flag标志清零放在这里，因为在进入环路会持续一段时间，只有当完全进入时在清零
        Set_speed = behind_circle_speed;        //后车入圆环就减速防止冲出赛道
        //检测到入圆环时开始检测出圆环
        //第一步，检测到的是入环时的出环标志
        while (rightout_circle(img))
        {
          camera_get_img();                            // 摄像头获取图像
          while (!Image_Flag);                        // 采集完毕
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// 解压
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//更新舵机控制PWM 
        }
        //第二步，完全进入圆环，检测不到出环标志
        while (!rightout_circle(img))
        {
          camera_get_img();                            // 摄像头获取图像
          while (!Image_Flag);                        // 采集完毕
          Image_Flag = 0;
          img_extract(img, imgbuff,CAMERA_SIZE);	// 解压
          road_mid = Image_analyze(img);
          S3010_pid();
          ftm_pwm_duty(S3010_FTM, S3010_CH, S3010_DUTY);//更新舵机控制PWM 
        }
        //第三步，再次检测到出环标志，就是真正的出环标志
        //开始延时一段时间发消息
        delay_sendMSG_flag = 1;         //检测到出圆环，延迟一段时间保证完全出圆环一段距离再发消息给后车
      }
      
      if (start_sendMSG_flag == 1)//逻辑后车开始发消息给逻辑前车
      {
        Txbuff[20] = 0xff;
        //此处加入定时一段时间，然后发送NRF消息使后车运行
        
        behind_in_circle_flag = 0; //后车已经完全出圆环
        
        start_sendMSG_flag = 0;         //只发送一次消息
        
        Set_speed = (int)(CMD_hmi_flash >> 24);  //恢复正常速度
        
        if(nrf_tx(Txbuff,DATA_PACKET) == 1) 
        {
          while(nrf_tx_state() == NRF_TXING);         //等待发送完成
          //后车发送消息成功
          front_flag = 1; //变为逻辑前车
          circle_count++;//一个圆环超一次车 
        }
      }
    }
    if (circle_count >= 4)
    {
      circle_count = 0;
    }  
    /************************ 无线发送和接收数据  ***********************/
    //nrf_msg_tx(COM_IMG,nrf_tx_buff);
    //while(nrf_tx_state() == NRF_TXING);             //等待发送完成
  }
}
