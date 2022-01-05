#include "camera.h"

uint8 colour[2] = {255, 0}; //0��ʾ��255��ʾ�ף��˴����Ըı��ʾ�ڰ׵�����

uint8 start_flag = 0;//���ڼ�������ߵı�־����
 
uint8 right_in_circle_flag = 0;  
uint8 left_in_circle_flag = 0;

//��Ȩƽ������
uint8 Weight[60] = {	0,  0,  0,  0,  0,    0,  0,  0,  0,  0,
			0,  0,  0,  0,  0,    0,  0,  0,  0,  0,
                        7,  6,  6,  5,  5,     5,  4,  3,  3,  3,  
                        5,  4,  3,  3,  3,     5,  4,  3,  3,  3,
                       1,  1,  1,  1,  1,    1,  1,  1,  1,  1,
			1,  1,  1,  1,  1,    1,  1,  1,  1,  1   };	//��Ȩƽ������

uint8 leftout_circle(uint8 *img)
{
  int8 i, j, k, left_bound[61];
  uint8 rightmost_leftbound = 0, rightmost_leftboundi, scan_point, scan_upbound, get_left_flag[61];
  uint8 out_circle_up = 40, out_circle_down = 10;
  
  rightmost_leftboundi = out_circle_down;
  scan_upbound = out_circle_down;
  for(i=out_circle_up,scan_upbound = out_circle_down; i>=scan_upbound; i--)//��59��scan_upbound����ɨ��
  {
    if (i == out_circle_up) //���õ�һ������߽����ʼ��
    {
      j = 45;
      scan_point = 45;
    }
    else if (i < out_circle_up)
    {
      for (k=i+1; k<=out_circle_up; k++)
      {
        if (get_left_flag[k] == 1)  //��ǰ�����߽�Ϊ�ο�
        {
          j = left_bound[k] + 5;
          scan_point = left_bound[k] + 5;   //�ڳ�һ����϶���ⶪ��,��ֵһ��Ҫ���ڵ���5
          break;
        }
      }
      if (k == out_circle_up + 1)  //ǰ��û���ҵ�����߽�
      {
        j = 45;
        scan_point = 45;
      }    
    }
    for(j=j,scan_point=scan_point; j>=2; j--)//��ʼ����߽�
    {
      if (img[i*80 + j] == 255 && img[i*80 + j - 1] == 0)//�ҵ��˱߽�
      {
        if (i < out_circle_up)
        {
          for (k=i+1; k<=out_circle_up; k++)
          {
            if (k-i-1<3 && get_left_flag[k] == 1)
            {                                                           
              if (j - 1 > left_bound[k] + 5)//��Ծ���ߣ�j - 1 < left_bound[k] - 4��ֹ��S������ߡ����Ż�����ȥ��-4
              {
                get_left_flag[i] = 0;
                left_bound[i] = 0;
                break;            
              }
              else  //û�����䶪��
              {
                left_bound[i] = j - 1;
                get_left_flag[i] = 1;
                break;
              }
            }
          }     
        }
        else
        {             
          left_bound[i] = j - 1;//�ҵ�����߽�
          get_left_flag[i] = 1;          
          break;
        }
      }
      else if (img[i*80 + scan_point] == 0 && img[i*80 + scan_point - 1] == 0)//ȫ��
      {
        if (scan_point == 79)  //����û����߽�
        {
          get_left_flag[i] = 0;
          left_bound[i] = 0;    //ȫ�ڶ��ߣ�Ĭ��0
          break;
        }
        scan_point++;
        j = scan_point + 1;
      }
      else if(img[i*80 + scan_point] == 255 && img[i*80 + scan_point - 1] == 255)//ȫ��
      {
        if (scan_point == 2)//����û����߽�
        {
          get_left_flag[i] = 0;
          left_bound[i] = 0;   //ȫ�׶��ߣ�Ĭ��0
          break;
        }
        scan_point--;
      }
    }
  }
   for (i=out_circle_up; i>=out_circle_down; i--)
  {
    if (left_bound[i] > rightmost_leftbound)
    {
      rightmost_leftbound = left_bound[i];
      rightmost_leftboundi = i;
    }
  }
  if (rightmost_leftboundi >= out_circle_down + 5 && rightmost_leftboundi <= out_circle_up - 5)
  {
    if (((left_bound[rightmost_leftboundi - 1] <= rightmost_leftbound && get_left_flag[rightmost_leftboundi - 1] == 1) && (left_bound[rightmost_leftboundi - 3] <= rightmost_leftbound && get_left_flag[rightmost_leftboundi - 3] == 1) && (left_bound[rightmost_leftboundi - 5] <= rightmost_leftbound && get_left_flag[rightmost_leftboundi - 5] == 1)) && ((left_bound[rightmost_leftboundi + 1] <= rightmost_leftbound && get_left_flag[rightmost_leftboundi + 1] == 1) && (left_bound[rightmost_leftboundi + 3] < rightmost_leftbound && get_left_flag[rightmost_leftboundi + 3] == 1) && (left_bound[rightmost_leftboundi + 5] < rightmost_leftbound && get_left_flag[rightmost_leftboundi + 5] == 1)))
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
  else
  {
     return 0;  //û�г�Բ��
  }
}

//ע������Խ�磡������
uint8 rightout_circle(uint8 *img)
{
  int8 i, j, k, right_bound[61];
  uint8 leftmost_rightbound = 79, leftmost_rightboundi, scan_point, scan_upbound, get_right_flag[61];
  uint8 out_circle_up = 40, out_circle_down = 10;
  
  leftmost_rightboundi = out_circle_down;
  scan_upbound = out_circle_down;
  for(i=out_circle_up, scan_upbound= out_circle_down; i>=scan_upbound; i--)//��59��scan_upbound����ɨ��
  {
    if (i == out_circle_up) //���õ�һ�����ұ߽����ʼ��
    {
      j = 35;
      scan_point = 35;
    }
    else if (i < out_circle_up)
    {
      for (k=i+1; k<=out_circle_up; k++) //��ǰ���ҵ����ұ߽�Ϊ�ο�
      {
        if (get_right_flag[k] == 1)
        {
          j = right_bound[k] - 5;           //�ڳ�һ����϶��ɨ��߽�,��ֵһ��Ҫ���ڵ���5
          scan_point = right_bound[k] - 5;
          break;
        }
      }
      if (k == out_circle_up + 1) //ǰ��û���ҵ����ұ߽�
      {
        j = 35;
        scan_point = 35;
      }    
    }
    for(j=j,scan_point=scan_point; j<=78; j++)//��ʼ���ұ߽�
    {
      if (img[i*80 + j] == 255 && img[i*80 + j + 1] == 0)
      {        
        if (i < out_circle_up)
        {
          for (k=i+1; k<=out_circle_up; k++)
          {
            if (k-i-1<3 && get_right_flag[k] == 1)
            {                              
              if (j + 1 < right_bound[k] - 5)//��Ծ���ߣ�j + 1 > right_bound[k] + 4 ��ֹ��S������ߡ����Ż�����ȥ��+4
              {
                get_right_flag[i] = 0;
                right_bound[i] = 79;
                break;            
              }
              else
              {
                get_right_flag[i] = 1;
                right_bound[i] = j + 1;
                break;
              }
            }
          }         
        }
        else
        {          
          right_bound[i] = j + 1;
          get_right_flag[i] = 1;    //�ҵ����ұ߽�
          break;  
        }
      }
      else if (img[i*80 + scan_point] == 0 && img[i*80 + scan_point + 1] == 0)//ȫ��
      {
        if (scan_point == 0) // �������ұ߽�
        {
          right_bound[i] = 79;//ȫ�ڶ��ߣ�Ĭ��79
          get_right_flag[i] = 0;      
          break;
        }
        scan_point--;
        j = scan_point - 1;
      }
      else if (img[i*80 + scan_point] == 255 && img[i*80 + scan_point + 1] == 255)//ȫ��
      {
        if (scan_point == 78)
        {
          right_bound[i] = 79;//ȫ�׶��ߣ�Ĭ��79
          get_right_flag[i] = 0;                
          break;
        }
        scan_point++;
      }
    }
  }
                  
  
  
  for (i=out_circle_up; i>=out_circle_down; i--)
  {
    if (right_bound[i] < leftmost_rightbound)
    {
      leftmost_rightbound = right_bound[i];
      leftmost_rightboundi = i;
    }
  }

  if (leftmost_rightboundi >= out_circle_down + 5 && leftmost_rightboundi <= out_circle_up - 5)
  {
    if (((right_bound[leftmost_rightboundi - 1] >= leftmost_rightbound && get_right_flag[leftmost_rightboundi - 1] == 1) && (right_bound[leftmost_rightboundi - 3] >= leftmost_rightbound && get_right_flag[leftmost_rightboundi - 3] == 1) && (right_bound[leftmost_rightboundi - 5] >= leftmost_rightbound && get_right_flag[leftmost_rightboundi - 5] == 1)) && ((right_bound[leftmost_rightboundi + 1] >= leftmost_rightbound && get_right_flag[leftmost_rightboundi + 1] == 1) && (right_bound[leftmost_rightboundi + 3] > leftmost_rightbound && get_right_flag[leftmost_rightboundi + 3] == 1) && (right_bound[leftmost_rightboundi + 5] > leftmost_rightbound && get_right_flag[leftmost_rightboundi + 5] == 1)))
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
  else
  {
     return 0;  //û�г�Բ��
  }
}

uint8 Image_analyze(uint8 *img)//0��ʾ��255��ʾ��
{
  uint8 circle_under = 0, circle_under1 = 0, circle_under2 = 0;
  uint8 left_cross_flag, left_jump_flag, right_cross_flag, right_jump_flag;
  uint8 get_right_flag[61], get_left_flag[61], get_mid_flag[61];//���߱�־
  uint8 left_white_flag = 0, right_white_flag = 0;//ȫ�׻�ȫ�ڶ���
  uint8 scan_point, scan_upbound = 10, scan_uppoint = 0, find_black = 0;
  int8 i, j, k, k1, circle_tier;//Բ����
  int8 left_bound[61], right_bound[61], mid_point[61];
  uint16 weight_sum = 1, midpoint_sum = 0, mid_pointcount = 1;
  
  uint8 road_midpoint, start_flag1 = 0, start_flag2 = 0, start_flag3 = 0, uncircle_flag1 = 0, uncircle_flag2 = 0;
  
  uint8 direction = 0; //����Բ��ת��
  
  for(i=59,scan_upbound = 10; i>=scan_upbound; i--)//��59��scan_upbound����ɨ��
  {
    left_cross_flag = 0;
    left_jump_flag = 0;
    right_cross_flag = 0;
    right_jump_flag = 0;
    get_right_flag[i] = 0;
    get_left_flag[i] = 0;
    get_mid_flag[i] = 0;
    left_white_flag = 0;
    right_white_flag = 0; 
    uncircle_flag1 = 0;
    uncircle_flag2 = 0;//��ʼ����־����     
   
/********************************** ��ʼ����߽� **************************************/
    if (i == 59) //���õ�һ������߽����ʼ��
    {
      j = 30;
      scan_point=30;
    }
    else if (i < 59)
    {
      for (k=i+1; k<=59; k++)
      {
        if (get_left_flag[k] == 1)  //��ǰ�����߽�Ϊ�ο�
        {
          j = left_bound[k] + 5;
          scan_point = left_bound[k] + 5;   //�ڳ�һ����϶���ⶪ��,��ֵһ��Ҫ���ڵ���5
          break;
        }
      }
      if (k == 60)  //ǰ��û���ҵ�����߽�
      {
        j = 30;
        scan_point=30;
      }    
    }
    for(j=j,scan_point=scan_point; j>=2; j--)//��ʼ����߽�
    {
      if (img[i*80 + j] == 255 && img[i*80 + j - 1] == 0)//�ҵ��˱߽�
      {
        if (i < 59)
        {
          for (k=i+1; k<=59; k++)
          {
            if (get_right_flag[k] == 1)
            {
              if (j - 1 >= right_bound[k])//���涪��
              {
                left_cross_flag = 1;        
                break;
              }
              else
              {
                left_cross_flag = 0;     
                break;
              }
            }
          }
          for (k=i+1; k<=59; k++)
          {
            if (k-i-1<10 && get_left_flag[k] == 1)
            { 
              if (j - 1 > left_bound[k] + 8 || j - 1 < left_bound[k] - 2)//��Ծ���ߣ�j - 1 < left_bound[k] - 4��ֹ��S������ߡ����Ż�����ȥ��-4
              {
                left_jump_flag = 1;
                break;
              }
              else
              {
                left_jump_flag = 0;
                break;
              }
            }
          }     
          if (left_cross_flag == 1 || left_jump_flag == 1)//����
          {
            get_left_flag[i] = 0;
            left_bound[i] = 0;
            break;
          }
          else if (left_cross_flag == 0 && left_jump_flag == 0)//û�ж���
          {
            left_bound[i] = j - 1;
            get_left_flag[i] = 1;
            if (left_bound[i] >= 70)           //�����Ͻ�
            {
              scan_upbound = i;
            }
            break;
          }
        }
        else
        {             
            left_bound[i] = j - 1;//�ҵ�����߽�
            get_left_flag[i] = 1;          
            break;
        }
      }
      else if (img[i*80 + scan_point] == 0 && img[i*80 + scan_point - 1] == 0)//ȫ��
      {
        if (scan_point == 79)  //����û����߽�
        {
          get_left_flag[i] = 0;
          left_bound[i] = 0;    //ȫ�ڶ��ߣ�Ĭ��0
          break;
        }
        scan_point++;
        j = scan_point + 1;
      }
      else if(img[i*80 + scan_point] == 255 && img[i*80 + scan_point - 1] == 255)//ȫ��
      {
        if (scan_point == 2)//����û����߽�
        {
          get_left_flag[i] = 0;
          left_white_flag = 1;  //ȫ�׶���
          left_bound[i] = 0;   //ȫ�׶��ߣ�Ĭ��0
          break;
        }
        scan_point--;
      }
    }
/********************************** ����߽���� **************************************/
    
/********************************** ��ʼ���ұ߽� **************************************/
    if (i == 59) //���õ�һ�����ұ߽����ʼ��
    {
      j = 50;
      scan_point=50;
    }
    else if (i < 59)
    {
      for (k=i+1; k<=59; k++) //��ǰ���ҵ����ұ߽�Ϊ�ο�
      {
        if (get_right_flag[k] == 1)
        {
          j = right_bound[k]-5;           //�ڳ�һ����϶��ɨ��߽�,��ֵһ��Ҫ���ڵ���5
          scan_point = right_bound[k]-5;
          break;
        }
      }
      if (k == 60) //ǰ��û���ҵ����ұ߽�
      {
        j = 50;
        scan_point=50;
      }    
    }
    for(j=j,scan_point=scan_point; j<=78; j++)//��ʼ���ұ߽�
    {
      if (img[i*80 + j] == 255 && img[i*80 + j + 1] == 0)
      {        
        if (i < 59)
        {
          for (k=i+1; k<=59; k++)
          {
            if (get_left_flag[k] == 1)
            {
              if (j + 1 <= left_bound[k])//���涪��
              {
                right_cross_flag = 1;                              
                break;
              }
              else
              {
                right_cross_flag = 0;                              
                break;
              }
            }
          }
          for (k=i+1; k<=59; k++)
          {
            if (k-i-1<10 && get_right_flag[k] == 1)
            {
              if (j + 1 > right_bound[k] || j + 1 < right_bound[k] - 8)//��Ծ���ߣ�j + 1 > right_bound[k] + 4 ��ֹ��S������ߡ����Ż�����ȥ��+4
              {
                right_jump_flag = 1;                
                break;
              }
              else
              {
                right_jump_flag = 0;
                break;
              }
            }
          }
          if (right_cross_flag == 1 || right_jump_flag == 1)//����
          {
            get_right_flag[i] = 0;
            right_bound[i] = 79;
            break;
          }
          else if (right_cross_flag == 0 && right_jump_flag == 0)//û�ж���
          {
            get_right_flag[i] = 1;
            right_bound[i] = j + 1;
            if (right_bound[i] <= 10)//�����Ͻ�
            {
              scan_upbound = i;
            }
            break;
          }
          
        }
        else
        {          
            right_bound[i] = j + 1;
            get_right_flag[i] = 1;    //�ҵ����ұ߽�
            break;  
        }
      }
      else if (img[i*80 + scan_point] == 0 && img[i*80 + scan_point + 1] == 0)//ȫ��
      {
        if (scan_point == 0) // �������ұ߽�
        {
          right_bound[i] = 79;//ȫ�ڶ��ߣ�Ĭ��79
          get_right_flag[i] = 0;
          break;
        }
        scan_point--;
        j = scan_point - 1;
      }
      else if (img[i*80 + scan_point] == 255 && img[i*80 + scan_point + 1] == 255)//ȫ��
      {
        if (scan_point == 78)
        {
          right_bound[i] = 79;//ȫ�׶��ߣ�Ĭ��79
          get_right_flag[i] = 0;
          right_white_flag = 1;//ȫ�׶���
          break;
        }
        scan_point++;
      }
    }            
/********************************** ���ұ߽���� **************************************/   
          
/********************************** ����������߽�� **************************************/
    
    if (get_left_flag[i] == 0 && get_right_flag[i] == 1)//��߶���
    {
      mid_point[i] = right_bound[i]-38*i/100-14;
      get_mid_flag[i] = 1;
        
      if (right_bound[i] < 65)
      {
        for (scan_uppoint = i; scan_uppoint>=2; scan_uppoint--)   //������߶��� ����ɨ���Ͻ�
        {
          if (img[scan_uppoint*80+1] == 255 && img[(scan_uppoint-1)*80+1] == 0)
          {
            if (i - scan_uppoint > 30) //��߶��߿�ȴ���30����Ϊ�ҵ��µ�ɨ���Ͻ�
            {
              scan_upbound = scan_uppoint;
            }
            break;
          }
        }
      }
    }
    if (get_left_flag[i] == 1 && get_right_flag[i] == 0)//�ұ߶���
    {
      mid_point[i] = left_bound[i]+38*i/100+14;
      get_mid_flag[i] = 1;
        
      if (left_bound[i] > 15)
      {
        for (scan_uppoint = i; scan_uppoint>=2; scan_uppoint--)
        {
          if (img[scan_uppoint*80 + 79] == 255 && img[(scan_uppoint-1)*80 + 79] == 0)
          {
            if (i - scan_uppoint > 30)  //�ұ߶��߿�ȴ���30����Ϊ�ҵ��µ�ɨ���Ͻ�
            {
              scan_upbound = scan_uppoint;
            }
            break;
          }
        }
      }
    }
    if (get_left_flag[i] == 1 && get_right_flag[i] == 1)              //û�ж���
    {
      mid_point[i] = (left_bound[i] + right_bound[i])/2;
      get_mid_flag[i] = 1;
    }
    if (get_left_flag[59] == 1 && get_right_flag[59] == 1)////////////////////////////////////////////
    {
      if (right_bound[59] - left_bound[59] < 50)
      {
        return (right_bound[59] + left_bound[59])/2;
      }
    }
    if (left_white_flag == 1 && right_white_flag ==1 || left_white_flag == 1 && right_jump_flag == 1 || right_white_flag == 1 && left_jump_flag == 1 || right_jump_flag == 1 && left_jump_flag == 1)
    {
      /***************************************�˴�����ʮ�ֺͻ�����������************************************************/      
      find_black = 40;
      for (circle_tier = i; circle_tier>10; circle_tier--)
      {
        if (img[circle_tier*80 + find_black] == 255 && img[(circle_tier-1)*80 + find_black] == 0)//�п����ǻ���
        {
          circle_under = circle_tier - 1;
          if (circle_under <= 19)/////////////////�ӳ�ת��
          {
            break;
          }
          //��ֹ��ʮ�ֳ�ͻ
          for (k=circle_under+7; k<59 && k>10; k--)
          {
            if (img[k*80 + find_black - 7] == 0)
            {
              circle_under1 = k;
              break;
            }
          }
          for (k=circle_under+7; k<59 && k>10; k--)
          {
            if (img[k*80 + find_black + 7] == 0)
            {
              circle_under2 = k;
              break;
            }
          }
          if(circle_under1 == 0 || circle_under2 == 0)
          {
            break;
          }
          
          if (circle_under1 <= circle_under)
          {
            if (circle_under - circle_under1 > 5)//�ǻ���
            {
              break;
            }
          }
          else
          {
            if (circle_under1 - circle_under > 5)//�ǻ��� /////////////
            {
              break;
            }
          }
          if (circle_under2 <= circle_under)
          {
            if (circle_under - circle_under2 > 5)//�ǻ���
            {
              break;
            }
          }
          else
          {
            if (circle_under2 - circle_under > 5)//�ǻ���
            {
              break;
            }
          }   
          //��ֹ�������ͻ
          for (k=find_black; k<find_black + 20 && k<79; k++)
          {
            if (img[(circle_tier+3)*80 + k] == 0)//�ǻ���
            {
              goto skip1;
            }
          }
          for (k=find_black; k>find_black - 20 && k>0; k--)
          {
            if (img[(circle_tier+3)*80 + k] == 0)//�ǻ���
            {
              goto skip1;
            }
          }
          //��ֹ�������߳�ͻ
          for (k=find_black ; k>find_black - 6; k--)
          {
            if (img[(circle_under-5)*80 + k] == 255)
            {
              uncircle_flag1 = 1;
              break;
            }          
          }
          for (k=find_black ; k<find_black + 6; k++)
          {
            if (img[(circle_under-5)*80 + k] == 255)
            {
              uncircle_flag2 = 1;
              break;
            }          
          }
          if (uncircle_flag1 == 1 && uncircle_flag2 == 1) //������
          {           
            break;//return 40;
          }
          //ȷ����Բ��
          switch(circle_count)
          {
          case 0:
            if (((CMD_hmi_flash>>4) & 0x01) == 1)
            {
              direction = 1;
            }
            else if (((CMD_hmi_flash>>4) & 0x01) == 0)
            {
              direction = 0;
            }
          break;
          case 1:
            if (((CMD_hmi_flash>>3) & 0x01) == 1)
            {
              direction = 1;
            }
            else if (((CMD_hmi_flash>>3) & 0x01) == 0)
            {
              direction = 0;
            }
          break;
          case 2:
            if (((CMD_hmi_flash>>2) & 0x01) == 1)
            {
              direction = 1;
            }
            else if (((CMD_hmi_flash>>2) & 0x01) == 0)
            {
              direction = 0;
            }
           break;
          case 3:
            if (((CMD_hmi_flash>>1) & 0x01) == 1)
            {
              direction = 1;
            }
            else if (((CMD_hmi_flash>>1) & 0x01) == 0)
            {
              direction = 0;
            }
           break;
          default:
           break;
          }              
          if(direction == 0) //��ת
          {  
            left_in_circle_flag = 1;  
            for (i=circle_under; i>10; i--)
            {        
              for(j=1; j<=find_black; j++)//��ʼ���ұ߽�
              {
                if (img[i*80 + j] == 255 && img[i*80 + j + 1] == 0)//�ҵ��˱߽�
                {                  
                  right_bound[i] = j + 1;                   
                  mid_point[i] = right_bound[i]-38*i/100+4;
                  get_mid_flag[i] = 1;                  
                  break;
                }               
              }   
            }  
            for (i=circle_under; i>10; i--)
            {
              if (get_mid_flag[i] == 1 && mid_point[i] > 0 && mid_point[i] < 79)
              {
                midpoint_sum += mid_point[i];
                mid_pointcount++;
              }
            }
            if (midpoint_sum/mid_pointcount == 0)
            {
              return 20;  //60��20
            }
            else
            {
              mid_pointcount = mid_pointcount - 1;
              return midpoint_sum/mid_pointcount;
            }
          }
          else if (direction == 1)//��ת
          { 
            right_in_circle_flag = 1;  
            for (i=circle_under; i>10; i--)
            {        
              for(j=79; j>=find_black; j--)//��ʼ����߽�
              {
                if (img[i*80 + j] == 255 && img[i*80 + j - 1] == 0)//�ҵ��˱߽�
                {                  
                  left_bound[i] = j - 1;                   
                  mid_point[i] = left_bound[i]+38*i/100-4;
                  get_mid_flag[i] = 1;                  
                  break;
                }               
              }   
            }  
            for (i=circle_under; i>10; i--)
            {
              if (get_mid_flag[i] == 1 && mid_point[i] > 0 && mid_point[i] < 79)
              {
                midpoint_sum += mid_point[i];
                mid_pointcount++;
              }
            }
            if (midpoint_sum/mid_pointcount == 0)
            {
              return 60;  //60��20
            }
            else
            {
              mid_pointcount = mid_pointcount - 1;
              return midpoint_sum/mid_pointcount;
            }
          }
        }
        if (0)
        {
        skip1:break;
        }
      } 
    }
    /*****************************************ʮ�ֺͻ�������    �������***********************************************/      
     /********************************** �������߽��������� **************************************/
  }//������һ�� 
    weight_sum = 1;
 for (i=scan_upbound; i<=59; i++)
 {
     if (get_mid_flag[i] == 1 && mid_point[i] > 0 && mid_point[i] < 79)
      {
        midpoint_sum += mid_point[i]*Weight[i];
        weight_sum += Weight[i];
      }
 }
 if (midpoint_sum/weight_sum == 0)
 {
   road_midpoint = 40;
 }
 else
 {
   weight_sum = weight_sum - 1;
   road_midpoint = midpoint_sum/weight_sum;
 }
 if (road_midpoint > 30 && road_midpoint < 50)
 {
       for (i=59; i>21; i--)
       {
         for (j=road_midpoint; j>road_midpoint-8 && j>0; j--)  //����߽� 8�ɸ�
         {
           if (img[i*80 + j] == 255 && img[i*80 + j - 1] == 0) //�ҵ�����߽�
           {                        
             for (k=j-1; k>j-6 && k>0; k--)
             {
               if (img[(i-4)*80 + k] == 0 && img[(i-4)*80 + k - 1] == 255)//������/////i-1��i-5
               {
                 start_flag1 =  1; 
                 break;
               }              
             }
             for (k=j; k<j+6 && k<79; k++)
             {
               if (img[(i-4)*80 + k] == 255 && img[(i-4)*80 + k + 1] == 0)//������
               {
                 start_flag2 =  1;     
                 break;
               }
             }
             if (start_flag2 == 1)
             {
               for (k1=k; k1<k+6 && k<79; k++)
               {         
                   if (img[(i-4)*80 + k] == 0 && img[(i-4)*80 + k + 1] == 255)//������
                   {
                     start_flag3 =  1;     
                     break;
                   }
               }
             }           
             if (start_flag1 == 1 && start_flag2 == 1 && start_flag3 == 1)
             {
               start_flag = start_flag + 1;
               return road_midpoint;
             }
             else
             {
               start_flag1 = 0;
               start_flag2 = 0;
               start_flag3 = 0;
             }
             break;
           }
         }    
       }
       for (i=30; i<57; i++)//���ұ߽�Ϊ��׼���ϰ� �ұ߽� ����Ϊ��y=79+0.3x-29 ��߽纯����ϵ y=38-x/2 ���������� y=0.4x+6.5
       {
         if (get_right_flag[i] == 1 && right_bound[i] > 40+0.3*i)
         {
            for (j = right_bound[i] - i*4/10 - 6 ; j > right_bound[i] - i*4/10 - 6 - 15; j--)
            {
              if (img[i*80 + j] == 255 && img[i*80 + j - 1] == 0)
              {
                return (j - 1 + right_bound[i])/2 + 5;
              }
            }
         }
         else
         {
           break;
         }
       }
       for (i=30; i<57; i++)//����߽�Ϊ��׼���ϰ� �ұ߽� ����Ϊ��y=79+0.3x-29 ��߽纯����ϵ y=38-x/2 ���������� y=0.4x+6.5
       {
         if (get_left_flag[i] == 1 && left_bound[i] < 48-i/2)
         {
            for (j = left_bound[i] + i*4/10 + 6 ; j < left_bound[i] + i*4/10 + 6 + 15; j++)
            {
              if (img[i*80 + j] == 255 && img[i*80 + j + 1] == 0)
              {
                return (j + 1 + left_bound[i])/2 - 5;
              }
            }
         }
         else
         {
           break;
         }
       }
       return road_midpoint;
 }
 else
 {
   return road_midpoint;
 }
}
/*!
*  @brief      ��ֵ��ͼ���ѹ���ռ� �� ʱ�� ��ѹ��
*  @param      dst             ͼ���ѹĿ�ĵ�ַ
*  @param      src             ͼ���ѹԴ��ַ
*  @param      srclen          ��ֵ��ͼ���ռ�ÿռ��С
*  @since      v5.0            img_extract(img, imgbuff,CAMERA_SIZE);
*  Sample usage:
*/
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
  uint8 tmpsrc;
  while(srclen --)
  {
    tmpsrc = *src++;
    *dst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
    *dst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
    *dst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
    *dst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
    *dst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
    *dst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
    *dst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
    *dst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
  }
}
