#include "camera.h"

uint8 colour[2] = {255, 0}; //0表示黑255表示白，此处可以改变表示黑白的数字

uint8 start_flag = 0;//用于检测起跑线的标志变量
 
uint8 right_in_circle_flag = 0;  
uint8 left_in_circle_flag = 0;

//加权平均参数
uint8 Weight[60] = {	0,  0,  0,  0,  0,    0,  0,  0,  0,  0,
			0,  0,  0,  0,  0,    0,  0,  0,  0,  0,
                        7,  6,  6,  5,  5,     5,  4,  3,  3,  3,  
                        5,  4,  3,  3,  3,     5,  4,  3,  3,  3,
                       1,  1,  1,  1,  1,    1,  1,  1,  1,  1,
			1,  1,  1,  1,  1,    1,  1,  1,  1,  1   };	//加权平均参数

uint8 leftout_circle(uint8 *img)
{
  int8 i, j, k, left_bound[61];
  uint8 rightmost_leftbound = 0, rightmost_leftboundi, scan_point, scan_upbound, get_left_flag[61];
  uint8 out_circle_up = 40, out_circle_down = 10;
  
  rightmost_leftboundi = out_circle_down;
  scan_upbound = out_circle_down;
  for(i=out_circle_up,scan_upbound = out_circle_down; i>=scan_upbound; i--)//从59到scan_upbound进行扫描
  {
    if (i == out_circle_up) //设置第一行找左边界的起始点
    {
      j = 45;
      scan_point = 45;
    }
    else if (i < out_circle_up)
    {
      for (k=i+1; k<=out_circle_up; k++)
      {
        if (get_left_flag[k] == 1)  //以前面的左边界为参考
        {
          j = left_bound[k] + 5;
          scan_point = left_bound[k] + 5;   //腾出一定空隙以免丢线,该值一定要大于等于5
          break;
        }
      }
      if (k == out_circle_up + 1)  //前面没有找到的左边界
      {
        j = 45;
        scan_point = 45;
      }    
    }
    for(j=j,scan_point=scan_point; j>=2; j--)//开始找左边界
    {
      if (img[i*80 + j] == 255 && img[i*80 + j - 1] == 0)//找到了边界
      {
        if (i < out_circle_up)
        {
          for (k=i+1; k<=out_circle_up; k++)
          {
            if (k-i-1<3 && get_left_flag[k] == 1)
            {                                                           
              if (j - 1 > left_bound[k] + 5)//跳跃丢线，j - 1 < left_bound[k] - 4防止左S弯道丢线、、优化环形去掉-4
              {
                get_left_flag[i] = 0;
                left_bound[i] = 0;
                break;            
              }
              else  //没有跳变丢线
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
          left_bound[i] = j - 1;//找到了左边界
          get_left_flag[i] = 1;          
          break;
        }
      }
      else if (img[i*80 + scan_point] == 0 && img[i*80 + scan_point - 1] == 0)//全黑
      {
        if (scan_point == 79)  //该行没有左边界
        {
          get_left_flag[i] = 0;
          left_bound[i] = 0;    //全黑丢线，默认0
          break;
        }
        scan_point++;
        j = scan_point + 1;
      }
      else if(img[i*80 + scan_point] == 255 && img[i*80 + scan_point - 1] == 255)//全白
      {
        if (scan_point == 2)//该行没有左边界
        {
          get_left_flag[i] = 0;
          left_bound[i] = 0;   //全白丢线，默认0
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
     return 0;  //没有出圆环
  }
}

//注意数组越界！！！！
uint8 rightout_circle(uint8 *img)
{
  int8 i, j, k, right_bound[61];
  uint8 leftmost_rightbound = 79, leftmost_rightboundi, scan_point, scan_upbound, get_right_flag[61];
  uint8 out_circle_up = 40, out_circle_down = 10;
  
  leftmost_rightboundi = out_circle_down;
  scan_upbound = out_circle_down;
  for(i=out_circle_up, scan_upbound= out_circle_down; i>=scan_upbound; i--)//从59到scan_upbound进行扫描
  {
    if (i == out_circle_up) //设置第一行找右边界的起始点
    {
      j = 35;
      scan_point = 35;
    }
    else if (i < out_circle_up)
    {
      for (k=i+1; k<=out_circle_up; k++) //以前面找到的右边界为参考
      {
        if (get_right_flag[k] == 1)
        {
          j = right_bound[k] - 5;           //腾出一定空隙来扫描边界,该值一定要大于等于5
          scan_point = right_bound[k] - 5;
          break;
        }
      }
      if (k == out_circle_up + 1) //前面没有找到的右边界
      {
        j = 35;
        scan_point = 35;
      }    
    }
    for(j=j,scan_point=scan_point; j<=78; j++)//开始找右边界
    {
      if (img[i*80 + j] == 255 && img[i*80 + j + 1] == 0)
      {        
        if (i < out_circle_up)
        {
          for (k=i+1; k<=out_circle_up; k++)
          {
            if (k-i-1<3 && get_right_flag[k] == 1)
            {                              
              if (j + 1 < right_bound[k] - 5)//跳跃丢线，j + 1 > right_bound[k] + 4 防止右S弯道丢线、、优化环形去掉+4
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
          get_right_flag[i] = 1;    //找到了右边界
          break;  
        }
      }
      else if (img[i*80 + scan_point] == 0 && img[i*80 + scan_point + 1] == 0)//全黑
      {
        if (scan_point == 0) // 该行无右边界
        {
          right_bound[i] = 79;//全黑丢线，默认79
          get_right_flag[i] = 0;      
          break;
        }
        scan_point--;
        j = scan_point - 1;
      }
      else if (img[i*80 + scan_point] == 255 && img[i*80 + scan_point + 1] == 255)//全白
      {
        if (scan_point == 78)
        {
          right_bound[i] = 79;//全白丢线，默认79
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
     return 0;  //没有出圆环
  }
}

uint8 Image_analyze(uint8 *img)//0表示黑255表示白
{
  uint8 circle_under = 0, circle_under1 = 0, circle_under2 = 0;
  uint8 left_cross_flag, left_jump_flag, right_cross_flag, right_jump_flag;
  uint8 get_right_flag[61], get_left_flag[61], get_mid_flag[61];//丢线标志
  uint8 left_white_flag = 0, right_white_flag = 0;//全白或全黑丢线
  uint8 scan_point, scan_upbound = 10, scan_uppoint = 0, find_black = 0;
  int8 i, j, k, k1, circle_tier;//圆环行
  int8 left_bound[61], right_bound[61], mid_point[61];
  uint16 weight_sum = 1, midpoint_sum = 0, mid_pointcount = 1;
  
  uint8 road_midpoint, start_flag1 = 0, start_flag2 = 0, start_flag3 = 0, uncircle_flag1 = 0, uncircle_flag2 = 0;
  
  uint8 direction = 0; //控制圆环转向
  
  for(i=59,scan_upbound = 10; i>=scan_upbound; i--)//从59到scan_upbound进行扫描
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
    uncircle_flag2 = 0;//初始化标志变量     
   
/********************************** 开始找左边界 **************************************/
    if (i == 59) //设置第一行找左边界的起始点
    {
      j = 30;
      scan_point=30;
    }
    else if (i < 59)
    {
      for (k=i+1; k<=59; k++)
      {
        if (get_left_flag[k] == 1)  //以前面的左边界为参考
        {
          j = left_bound[k] + 5;
          scan_point = left_bound[k] + 5;   //腾出一定空隙以免丢线,该值一定要大于等于5
          break;
        }
      }
      if (k == 60)  //前面没有找到的左边界
      {
        j = 30;
        scan_point=30;
      }    
    }
    for(j=j,scan_point=scan_point; j>=2; j--)//开始找左边界
    {
      if (img[i*80 + j] == 255 && img[i*80 + j - 1] == 0)//找到了边界
      {
        if (i < 59)
        {
          for (k=i+1; k<=59; k++)
          {
            if (get_right_flag[k] == 1)
            {
              if (j - 1 >= right_bound[k])//交叉丢线
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
              if (j - 1 > left_bound[k] + 8 || j - 1 < left_bound[k] - 2)//跳跃丢线，j - 1 < left_bound[k] - 4防止左S弯道丢线、、优化环形去掉-4
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
          if (left_cross_flag == 1 || left_jump_flag == 1)//丢线
          {
            get_left_flag[i] = 0;
            left_bound[i] = 0;
            break;
          }
          else if (left_cross_flag == 0 && left_jump_flag == 0)//没有丢线
          {
            left_bound[i] = j - 1;
            get_left_flag[i] = 1;
            if (left_bound[i] >= 70)           //更新上界
            {
              scan_upbound = i;
            }
            break;
          }
        }
        else
        {             
            left_bound[i] = j - 1;//找到了左边界
            get_left_flag[i] = 1;          
            break;
        }
      }
      else if (img[i*80 + scan_point] == 0 && img[i*80 + scan_point - 1] == 0)//全黑
      {
        if (scan_point == 79)  //该行没有左边界
        {
          get_left_flag[i] = 0;
          left_bound[i] = 0;    //全黑丢线，默认0
          break;
        }
        scan_point++;
        j = scan_point + 1;
      }
      else if(img[i*80 + scan_point] == 255 && img[i*80 + scan_point - 1] == 255)//全白
      {
        if (scan_point == 2)//该行没有左边界
        {
          get_left_flag[i] = 0;
          left_white_flag = 1;  //全白丢线
          left_bound[i] = 0;   //全白丢线，默认0
          break;
        }
        scan_point--;
      }
    }
/********************************** 找左边界结束 **************************************/
    
/********************************** 开始找右边界 **************************************/
    if (i == 59) //设置第一行找右边界的起始点
    {
      j = 50;
      scan_point=50;
    }
    else if (i < 59)
    {
      for (k=i+1; k<=59; k++) //以前面找到的右边界为参考
      {
        if (get_right_flag[k] == 1)
        {
          j = right_bound[k]-5;           //腾出一定空隙来扫描边界,该值一定要大于等于5
          scan_point = right_bound[k]-5;
          break;
        }
      }
      if (k == 60) //前面没有找到的右边界
      {
        j = 50;
        scan_point=50;
      }    
    }
    for(j=j,scan_point=scan_point; j<=78; j++)//开始找右边界
    {
      if (img[i*80 + j] == 255 && img[i*80 + j + 1] == 0)
      {        
        if (i < 59)
        {
          for (k=i+1; k<=59; k++)
          {
            if (get_left_flag[k] == 1)
            {
              if (j + 1 <= left_bound[k])//交叉丢线
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
              if (j + 1 > right_bound[k] || j + 1 < right_bound[k] - 8)//跳跃丢线，j + 1 > right_bound[k] + 4 防止右S弯道丢线、、优化环形去掉+4
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
          if (right_cross_flag == 1 || right_jump_flag == 1)//丢线
          {
            get_right_flag[i] = 0;
            right_bound[i] = 79;
            break;
          }
          else if (right_cross_flag == 0 && right_jump_flag == 0)//没有丢线
          {
            get_right_flag[i] = 1;
            right_bound[i] = j + 1;
            if (right_bound[i] <= 10)//更新上界
            {
              scan_upbound = i;
            }
            break;
          }
          
        }
        else
        {          
            right_bound[i] = j + 1;
            get_right_flag[i] = 1;    //找到了右边界
            break;  
        }
      }
      else if (img[i*80 + scan_point] == 0 && img[i*80 + scan_point + 1] == 0)//全黑
      {
        if (scan_point == 0) // 该行无右边界
        {
          right_bound[i] = 79;//全黑丢线，默认79
          get_right_flag[i] = 0;
          break;
        }
        scan_point--;
        j = scan_point - 1;
      }
      else if (img[i*80 + scan_point] == 255 && img[i*80 + scan_point + 1] == 255)//全白
      {
        if (scan_point == 78)
        {
          right_bound[i] = 79;//全白丢线，默认79
          get_right_flag[i] = 0;
          right_white_flag = 1;//全白丢线
          break;
        }
        scan_point++;
      }
    }            
/********************************** 找右边界结束 **************************************/   
          
/********************************** 处理该行找线结果 **************************************/
    
    if (get_left_flag[i] == 0 && get_right_flag[i] == 1)//左边丢线
    {
      mid_point[i] = right_bound[i]-38*i/100-14;
      get_mid_flag[i] = 1;
        
      if (right_bound[i] < 65)
      {
        for (scan_uppoint = i; scan_uppoint>=2; scan_uppoint--)   //由于左边丢线 更新扫描上界
        {
          if (img[scan_uppoint*80+1] == 255 && img[(scan_uppoint-1)*80+1] == 0)
          {
            if (i - scan_uppoint > 30) //左边丢线宽度大于30则认为找到新的扫描上界
            {
              scan_upbound = scan_uppoint;
            }
            break;
          }
        }
      }
    }
    if (get_left_flag[i] == 1 && get_right_flag[i] == 0)//右边丢线
    {
      mid_point[i] = left_bound[i]+38*i/100+14;
      get_mid_flag[i] = 1;
        
      if (left_bound[i] > 15)
      {
        for (scan_uppoint = i; scan_uppoint>=2; scan_uppoint--)
        {
          if (img[scan_uppoint*80 + 79] == 255 && img[(scan_uppoint-1)*80 + 79] == 0)
          {
            if (i - scan_uppoint > 30)  //右边丢线宽度大于30则认为找到新的扫描上界
            {
              scan_upbound = scan_uppoint;
            }
            break;
          }
        }
      }
    }
    if (get_left_flag[i] == 1 && get_right_flag[i] == 1)              //没有丢线
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
      /***************************************此处加入十字和环形赛道处理************************************************/      
      find_black = 40;
      for (circle_tier = i; circle_tier>10; circle_tier--)
      {
        if (img[circle_tier*80 + find_black] == 255 && img[(circle_tier-1)*80 + find_black] == 0)//有可能是环形
        {
          circle_under = circle_tier - 1;
          if (circle_under <= 19)/////////////////延迟转弯
          {
            break;
          }
          //防止与十字冲突
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
            if (circle_under - circle_under1 > 5)//非环形
            {
              break;
            }
          }
          else
          {
            if (circle_under1 - circle_under > 5)//非环形 /////////////
            {
              break;
            }
          }
          if (circle_under2 <= circle_under)
          {
            if (circle_under - circle_under2 > 5)//非环形
            {
              break;
            }
          }
          else
          {
            if (circle_under2 - circle_under > 5)//非环形
            {
              break;
            }
          }   
          //防止与弯道冲突
          for (k=find_black; k<find_black + 20 && k<79; k++)
          {
            if (img[(circle_tier+3)*80 + k] == 0)//非环形
            {
              goto skip1;
            }
          }
          for (k=find_black; k>find_black - 20 && k>0; k--)
          {
            if (img[(circle_tier+3)*80 + k] == 0)//非环形
            {
              goto skip1;
            }
          }
          //防止与起跑线冲突
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
          if (uncircle_flag1 == 1 && uncircle_flag2 == 1) //起跑线
          {           
            break;//return 40;
          }
          //确定是圆环
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
          if(direction == 0) //左转
          {  
            left_in_circle_flag = 1;  
            for (i=circle_under; i>10; i--)
            {        
              for(j=1; j<=find_black; j++)//开始找右边界
              {
                if (img[i*80 + j] == 255 && img[i*80 + j + 1] == 0)//找到了边界
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
              return 20;  //60改20
            }
            else
            {
              mid_pointcount = mid_pointcount - 1;
              return midpoint_sum/mid_pointcount;
            }
          }
          else if (direction == 1)//右转
          { 
            right_in_circle_flag = 1;  
            for (i=circle_under; i>10; i--)
            {        
              for(j=79; j>=find_black; j--)//开始找左边界
              {
                if (img[i*80 + j] == 255 && img[i*80 + j - 1] == 0)//找到了边界
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
              return 60;  //60改20
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
    /*****************************************十字和环形赛道    处理完毕***********************************************/      
     /********************************** 该行找线结果处理完毕 **************************************/
  }//进入下一行 
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
         for (j=road_midpoint; j>road_midpoint-8 && j>0; j--)  //找左边界 8可改
         {
           if (img[i*80 + j] == 255 && img[i*80 + j - 1] == 0) //找到了左边界
           {                        
             for (k=j-1; k>j-6 && k>0; k--)
             {
               if (img[(i-4)*80 + k] == 0 && img[(i-4)*80 + k - 1] == 255)//起跑线/////i-1改i-5
               {
                 start_flag1 =  1; 
                 break;
               }              
             }
             for (k=j; k<j+6 && k<79; k++)
             {
               if (img[(i-4)*80 + k] == 255 && img[(i-4)*80 + k + 1] == 0)//起跑线
               {
                 start_flag2 =  1;     
                 break;
               }
             }
             if (start_flag2 == 1)
             {
               for (k1=k; k1<k+6 && k<79; k++)
               {         
                   if (img[(i-4)*80 + k] == 0 && img[(i-4)*80 + k + 1] == 255)//起跑线
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
       for (i=30; i<57; i++)//以右边界为基准找障碍 右边界 函数为：y=79+0.3x-29 左边界函数关系 y=38-x/2 半边赛道宽度 y=0.4x+6.5
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
       for (i=30; i<57; i++)//以左边界为基准找障碍 右边界 函数为：y=79+0.3x-29 左边界函数关系 y=38-x/2 半边赛道宽度 y=0.4x+6.5
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
*  @brief      二值化图像解压（空间 换 时间 解压）
*  @param      dst             图像解压目的地址
*  @param      src             图像解压源地址
*  @param      srclen          二值化图像的占用空间大小
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
