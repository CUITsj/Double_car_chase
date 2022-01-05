#include "init.h"

uint8  nrf_rx_buff[CAMERA_SIZE + 2*COM_LEN + DATA_PACKET];         //Ԥ��
uint8  nrf_tx_buff[CAMERA_SIZE + 2*COM_LEN + DATA_PACKET];         //Ԥ��

uint32 CMD_hmi_flash = 0;

void System_Init()
{
  char HMI_Cmd_Tmp = 0;
  uint8 HMI_Cmd = 0;
  uint8 HMI_Cmd_Tmp_YH1 = 0;
  uint8 HMI_Cmd_Tmp_YH2 = 0;
  uint8 HMI_Cmd_Tmp_YH3 = 0;
  uint8 HMI_Cmd_Tmp_YH4 = 0;
  uint8 relen = 0;
  uint8 buff[DATA_PACKET] = {0};
  
  buff[15] = 0xff;
  
  /************************  ��ʼ����ʼ  ************************************/
  led_init(LED_MAX);
  led(LED_MAX,LED_ON);
  
  /************************ �����ʼ��  ***********************/
  ftm_pwm_init(S3010_FTM, S3010_CH, S3010_HZ, S3010_MID);//ռ�ձ�Ϊ S3010_MID/FTM3_PRECISON,FTM3_PRECISONΪ10000u,��MK60_FTM.h�ж���
  
  /************************ �����ʼ�� ************************************/
  ftm_pwm_init(MOTOR_FTM, MOTOR1_CH, MOTOR_HZ, 0);//����ǰ
  ftm_pwm_init(MOTOR_FTM, MOTOR2_CH, MOTOR_HZ, 0);//���ֺ�  
  ftm_pwm_init(MOTOR_FTM, MOTOR3_CH, MOTOR_HZ, 0);//����ǰ
  ftm_pwm_init(MOTOR_FTM, MOTOR4_CH, MOTOR_HZ, 0);//���ֺ�
  
  /************************ ���� K60 �����ȼ�  *****************************/
  NVIC_SetPriorityGrouping(4);      //������Χ��0~4��4��ʾ4bitȫΪ��ռ���ȼ���û��ѹ���ȼ�
  NVIC_SetPriority(PORTA_IRQn,0);         // ����ԽС���ȼ�Խ�ߣ�����ͷ
  NVIC_SetPriority(PORTE_IRQn,1);         // NRF
  NVIC_SetPriority(DMA0_IRQn,2);          // ����ͷ
  NVIC_SetPriority(PIT0_IRQn,3);          // ��ʱ��
  NVIC_SetPriority(PIT1_IRQn,4);          // ��ʱ��
  NVIC_SetPriority(PIT2_IRQn,5);          // ��ʱ��
  NVIC_SetPriority(UART4_RX_TX_IRQn,6);   // ����
  
  /************************ ����ͷ ��ʼ�� **********************************/
  camera_init(imgbuff);                                   // ����ͷ��ʼ������ͼ��ɼ��� ImageBuff ��ַ
  set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);	// ���� PORTA ���жϷ�����Ϊ PORTA_IRQHandler
  set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      // ���� DMA0 ���жϷ�����Ϊ DMA0_IRQHandler
  
  /************************ ��������ʼ�� **********************************/ 
  // ʹ�ö�ʱ��0������10msִ��һ��  
  ftm_quad_init(FTM1);	// ����
  ftm_quad_init(FTM2);	// ����
  
  /************************** UART���� ��ʼ��  ***********************/
  uart_init(UART4, 9600); //���ڴ�����
  
  /************************ ����ģ�� NRF ��ʼ��  ***********************/
  #if 1
    uint32 i = 20;
    while(!nrf_init());
    //�����жϷ�����
    set_vector_handler(PORTE_VECTORn, PORTE_IRQHandler);
    enable_irq(PORTE_IRQn);
    nrf_msg_init();
    
    while(i--)
    {
      nrf_msg_tx(COM_RETRAN, nrf_tx_buff);// COM_RETRAN��λ���䣬����֮ǰ���յ�������
    }
  #endif
  
  /************************ FLASH�ʹ�������ʼ�� *****************************/
  CMD_hmi_flash = flash_read(SECTOR_NUM, 0, uint32);
  //��ʾ��ǰ������ٶ�
  printf("t7.txt=\"%d\"",(uint8)(CMD_hmi_flash>>24));
  uart_putchar(UART4,0xff);
  uart_putchar(UART4,0xff);
  uart_putchar(UART4,0xff);  
  //��ʾ��ǰ�������ֵ
  printf("t8.txt=\"%d\"",(uint8)(CMD_hmi_flash>>16));
  uart_putchar(UART4,0xff);
  uart_putchar(UART4,0xff);
  uart_putchar(UART4,0xff);  
  
  while(HMI_Cmd != 0x17)
  {
    relen = nrf_rx(buff,DATA_PACKET);               //�ȴ�����һ�����ݰ������ݴ洢��buff��
    if(relen != 0)
    {
      if (buff[15] == 0xff)
      {
        HMI_Cmd = 0x17;
      }
    }
    if(uart_querychar(UART4,&HMI_Cmd_Tmp)==1)     //��ѯ����1���ַ������浽 ch��
    {
      HMI_Cmd = HMI_Cmd_Tmp;
      if(HMI_Cmd == 0x01)
      {
        CMD_hmi_flash += (1<<24);
      }
      if(HMI_Cmd == 0x02)
      {
        CMD_hmi_flash -= (1<<24);
      }
      if(HMI_Cmd == 0x03)
      {
        CMD_hmi_flash += (1<<16);
      }
      if(HMI_Cmd == 0x04)
      {
        CMD_hmi_flash -= (1<<16);
      }
      if(HMI_Cmd == 0x05)
      {
        HMI_Cmd_Tmp_YH1 = ~HMI_Cmd_Tmp_YH1;
        if(HMI_Cmd_Tmp_YH1)
          CMD_hmi_flash |= (1<<4);
        else      
          CMD_hmi_flash &= ~(1<<4);
      }
      if(HMI_Cmd == 0x06)
      {
        HMI_Cmd_Tmp_YH2 = ~HMI_Cmd_Tmp_YH2;
        if(HMI_Cmd_Tmp_YH2)
          CMD_hmi_flash |= (1<<3);
        else      
          CMD_hmi_flash &= ~(1<<3);
      }
      if(HMI_Cmd == 0x07)
      {
        HMI_Cmd_Tmp_YH3 = ~HMI_Cmd_Tmp_YH3;
        if(HMI_Cmd_Tmp_YH3)
          CMD_hmi_flash |= (1<<2);
        else      
          CMD_hmi_flash &= ~(1<<2);
      }
      if(HMI_Cmd == 0x08)
      {
        HMI_Cmd_Tmp_YH4 = ~HMI_Cmd_Tmp_YH4;
        if(HMI_Cmd_Tmp_YH4)
          CMD_hmi_flash |= (1<<1);
        else      
          CMD_hmi_flash &= ~(1<<1);
      }
      //��ʾ��ǰ�ٶ�
      printf("t7.txt=\"%d\"",(uint8)(CMD_hmi_flash>>24));
      uart_putchar(UART4,0xff);
      uart_putchar(UART4,0xff);
      uart_putchar(UART4,0xff);
      //��ʾ��ǰ�������ֵ
      printf("t8.txt=\"%d\"",(uint8)(CMD_hmi_flash>>16));
      uart_putchar(UART4,0xff);
      uart_putchar(UART4,0xff);
      uart_putchar(UART4,0xff);
      
      //��ʾԲ������
      printf("t3.txt=\"%d\"",(uint8)(CMD_hmi_flash>>4) & 0x01);
      uart_putchar(UART4,0xff);
      uart_putchar(UART4,0xff);
      uart_putchar(UART4,0xff);   
      printf("t4.txt=\"%d\"",(uint8)(CMD_hmi_flash>>3) & 0x01);
      uart_putchar(UART4,0xff);
      uart_putchar(UART4,0xff);
      uart_putchar(UART4,0xff);   
      printf("t5.txt=\"%d\"",(uint8)(CMD_hmi_flash>>2) & 0x01);
      uart_putchar(UART4,0xff);
      uart_putchar(UART4,0xff);
      uart_putchar(UART4,0xff);   
      printf("t6.txt=\"%d\"",(uint8)(CMD_hmi_flash>>1) & 0x01);
      uart_putchar(UART4,0xff);
      uart_putchar(UART4,0xff);
      uart_putchar(UART4,0xff);   
    }
  }
  if(nrf_tx(buff,DATA_PACKET) == 1) 
  {
    while(nrf_tx_state() == NRF_TXING);         //�ȴ��������
  }
  
  while(!flash_erase_sector(SECTOR_NUM));                     //��������
  while(!flash_write(SECTOR_NUM, 0, CMD_hmi_flash));  //д�����ݵ�������ƫ�Ƶ�ַΪ0������һ��д��4�ֽ� 
  
  /************************ ����ͷ��ֵ��ʼ�� *********************************/
  SCCB_WriteByte(OV7725_CNST, (CMD_hmi_flash>>16));	//�ı�ͼ����ֵ  
  
  /************************ ͼ��ɼ� ***************************************/
  camera_get_img();	// �Ȳɼ�һ��ͼ��
  
  /************************ ��ʱ�� ��ʼ��  *********************************/ 
  pit_init_ms(PIT0, 10);                              	// ��ʼ��PIT0����ʱʱ��Ϊ�� 10ms
  set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);	// ����PIT0���жϷ�����Ϊ PIT0_IRQHandler
  enable_irq (PIT0_IRQn); 				// ʹ���ж�
  
  //����ʹPIT1��PIT2�Ķ�ʱʱ��󣬲���ʹ��ʱ����׼ȷ
  pit_init_ms(PIT1, 30);                           	// ��ʼ��PIT1����ʱʱ��Ϊ�� 20ms
  set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler); 	// ����PIT1���жϷ�����Ϊ PIT1_IRQHandler
  enable_irq (PIT1_IRQn); 				// ʹ���ж�
  
  pit_init_ms(PIT2, 100);                              // ��ʼ��PIT2����ʱʱ��Ϊ�� 1s
  set_vector_handler(PIT2_VECTORn, PIT2_IRQHandler);    // ����PIT1���жϷ�����Ϊ PIT2_IRQHandler
  enable_irq(PIT2_IRQn);                                // ʹ���ж�
  
  /************************  ��ʼ������  ************************************/
  led(LED_MAX,LED_OFF);// ��LEDָʾ��		ȫ����ʼ���ɹ�
}