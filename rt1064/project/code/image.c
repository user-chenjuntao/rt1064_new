#include "image.h"
#include "motor.h"
#include "config.h"
#include "zf_device_ips200.h"
#include "math.h"
#include "data_handle.h"
#include "color_detection.h"
#include "imu963ra_handle.h"
#include "path_follow.h"


uint8 image_otsuThreshold_less;
uint8 PostProcessing_image[MT9V03X_H][MT9V03X_W];                                      //用于存放二值化处理后的图像数组
uint8 Left_Lost_Time = 0;                                                              //记录左边界无效点
uint8 Right_Lost_Time = 0;                                                             //记录右边界无效点
uint8 Zebra_stop_flag =0;
float line_error = 0;
uint8 low_high_choose = 0;
uint8 huandao_num_flag = 0;
uint8 crossing_flag_help = 0;
int straight_speed = 100;//200   240
int turn_speed = 90;//160       185
int yuanhuan_speed = 210;//125
int left_right_choose = 1;                                                             //-1 表示进左圆环，即反方向发车  1 表示进右圆环，即正方向发车
uint8 qianzhan_value = 60;
uint8 qianzhan_change_value = 44;
uint8 Zebra_end_flag = 0;
uint8 yuanhuan_flag_4 = 0;
int start_accelerate = 0;
int end_decelerate = -15;
int slow_turn_speed = -20;
int turn_num = 3;
uint8 car_go_flag = 0;
uint8 going_flag = 0;
extern int speed_base;

volatile uint8 push_box_flag = 0;
uint8 Direction_promotion = 1;
volatile float now_rotate_angle = 0;

/*********************************************************
** //图像权重数组  //权重越靠上，车转弯越早
*********************************************************/
uint8 mid_weight_list[120] = 
{
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,6,6,
    6,8,10,12,13,14,15,16,17,18,
    19,20,21,24,25,26,27,28,24,22,
    21,18,18,18,17,17,16,16,16,16,
    16,15,14,13,12,11,10,9,8,7,
    6,6,6,6,6,6,6,6,6,6,
    6,5,4,3,2,1,1,1,1,1,
    1,1,1,1,1,0,0,0,0,0,
};



uint8 low_weight_list[120] = 
{
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    1,1,1,2,2,2,2,3,3,3,
    4,4,5,6,7,8,9,10,11,12,
    13,14,15,16,17,18,17,16,15,14,
    13,12,11,10,9,8,7,6,6,6,
    6,6,5,5,5,4,4,3,2,1,
    1,1,1,1,1,1,1,1,1,1,
};

float final_mid_line = MID_W;   // 最终输出的中线值
float last_mid_line = MID_W;    // 上次中线值

//-------------------------------------------------------------------------------------
//使用大津法进行二值化处理
//-------------------------------------------------------------------------------------
//uint8 otsuThreshold_less(uint8 *image, uint16 width, uint16 height)
//{
//    #define GrayScale 256
//    int pixel_count[GrayScale] = {0};//每个灰度值所占像素个数
//    float pixel_account[GrayScale] = {0};//每个灰度值所占总像素比例
//    int i,j;   
//    int pixel_sum = width * height;   //总像素点
//    uint8 threshold = 0;
//    uint8* pixel_data = image;  //指向像素数据的指针


//    //统计灰度级中每个像素在整幅图像中的个数  
//    for (i = 0; i < height; i++)
//    {
//        for (j = 0; j < width; j++)
//        {
//            pixel_count[(int)pixel_data[i * width + j]]++;  //将像素值作为计数数组的下标

//        }
//    }
//    float u = 0;  
//    for (i = 0; i < GrayScale; i++)
//    {
//        pixel_account[i] = (float)pixel_count[i] / pixel_sum;   //计算每个像素在整幅图像中的比例  
//        u += i * pixel_account[i];  //总平均灰度
//    }

//      
//    float variance_max=0.0;  //最大类间方差
//	float variance;
//    float w0 = 0, avgValue  = 0;  //w0 前景比例 ，avgValue 前景平均灰度
//    for(int i = 0; i < 256; i++)     //每一次循环都是一次完整类间方差计算 (两个for叠加为1个)
//    {  
//        w0 += pixel_account[i];  //假设当前灰度i为阈值, 0~i 灰度像素所占整幅图像的比例即前景比例
//        avgValue  += i * pixel_account[i];        
//        if (w0 <= 0 || w0 >= 1)
//			continue;
//        variance = pow((avgValue/w0 - u), 2) * w0 /(1 - w0);    //类间方差   
//        if(variance > variance_max) 
//        {  
//            variance_max = variance;  
//            threshold = i;  
//        }  
//    } 

//    return threshold;

//}

int My_Adapt_Threshold(uint8*image,uint16 width, uint16 height)   //大津算法，注意计算阈值的一定要是原图像
{
    #define GrayScale 256
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    int  threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    for (i = 0; i < height; i+=2)//统计灰度级中每个像素在整幅图像中的个数
    {
        for (j = 0; j <width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
        }
    }
    for (i = 0; i < GrayScale; i++) //计算每个像素值的点在整幅图像中的比例
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)//遍历灰度级[0,255]
    {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);//平方
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;//最大类间方差法
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    if(threshold>255)
        threshold=255;
    if(threshold<0)
        threshold=0;
  return threshold;
}


void image_postprocess(void)
{
	uint8 i = 0, j = 0;
	image_otsuThreshold_less = My_Adapt_Threshold(*mt9v03x_image,MT9V03X_W,MT9V03X_H);
	for (i = 0; i < MT9V03X_H; i++)
	{
		for (j = 0; j < MT9V03X_W; j++)
		{
			PostProcessing_image[i][j] = (mt9v03x_image[i][j] > image_otsuThreshold_less) ? WHITE_PIXEL : BLACK_PIXEL;
		}
	}
//	image_filter(PostProcessing_image);
}

//目的是为了出去一些噪点，防止对于元素判断出现问题
void image_filter(uint8(*image)[MT9V03X_W])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
	uint16 i, j;
	uint32 num = 0;


	for (i = 1; i < MT9V03X_H - 1; i++)
	{
		for (j = 1; j < (MT9V03X_W - 1); j++)
		{
			//统计八个方向的像素值
			num =
				image[i - 1][j - 1] + image[i - 1][j] + image[i - 1][j + 1]
				+ image[i][j - 1] + image[i][j + 1]
				+ image[i + 1][j - 1] + image[i + 1][j] + image[i + 1][j + 1];


			if (num >= THRESHOLD_MAX && image[i][j] == 0)
			{

				image[i][j] = WHITE_PIXEL;//白  可以搞成宏定义，方便更改

			}
			if (num <= THRESHOLD_MIN && image[i][j] == 255)
			{

				image[i][j] = BLACK_PIXEL;//黑

			}

		}
	}

}

uint8 car_stop_flag = 0;

//void car_stop(void)
//{
//	uint8 i, j;
//	uint16 black_num = 0;
//	
//	for (i = MT9V03X_H - 1; i > MT9V03X_H - 6; i--)
//	{
//		for (j = 30; j < MT9V03X_W-30; j += 2)
//		{
//			if (PostProcessing_image[i][j] == BLACK_PIXEL)
//				black_num++;
//		}
//	}// || (Left_Lost_Time > 85 && Right_Lost_Time > 85)
//	if (black_num > 285)
//	{
//		car_stop_flag = 1;
//	}
//	else
//	{
//		car_stop_flag = 0;
//	}
//}

void car_stop(void)
{
	uint8 i, j;
	uint16 black_num = 0;
	uint8 black_row_num = 0;
	
	for (i = MT9V03X_H - 1; i > MT9V03X_H - 7; i--)
	{
		for (j = 20; j < MT9V03X_W-20; j ++)
		{
			if (PostProcessing_image[i][j] == BLACK_PIXEL)
				black_num++;
		}
		if (black_num >= 90)
		{
			black_row_num++;
		}
		black_num = 0;
	}
	if (black_row_num >= 4)
	{
		car_stop_flag = 1;
	}
}

uint8 research_start_point(void)
{
	if (PostProcessing_image[MT9V03X_H - 1][MT9V03X_W/2] == WHITE_PIXEL && PostProcessing_image[MT9V03X_H - 1][MT9V03X_W/2-1] == WHITE_PIXEL && PostProcessing_image[MT9V03X_H - 1][MT9V03X_W/2+1] == WHITE_PIXEL)
	{
		return 2;
	}
	else if (PostProcessing_image[MT9V03X_H - 1][MT9V03X_W/4] == WHITE_PIXEL && PostProcessing_image[MT9V03X_H - 1][MT9V03X_W/4-1] == WHITE_PIXEL && PostProcessing_image[MT9V03X_H - 1][MT9V03X_W/4+1] == WHITE_PIXEL)
	{
		return 1;
	}
	else if (PostProcessing_image[MT9V03X_H - 1][MT9V03X_W/4*3] == WHITE_PIXEL && PostProcessing_image[MT9V03X_H - 1][MT9V03X_W/4*3-1] == WHITE_PIXEL && PostProcessing_image[MT9V03X_H - 1][MT9V03X_W/4*3+1] == WHITE_PIXEL)
	{
		return 3;
	}
	else
	{
		return 2;
	}
}
	
	

 //&& PostProcessing_image[MT9V03X_H - 1][i - 2] == BLACK_PIXEL&& PostProcessing_image[MT9V03X_H - 1][i + 2] == BLACK_PIXEL
uint8 get_left_start(uint8 num)
{
	uint8 i = 0;
	for(i = MT9V03X_W*(num/4.0f); i > 1; i--)
	{
		if ((PostProcessing_image[MT9V03X_H - 1][i] == WHITE_PIXEL && PostProcessing_image[MT9V03X_H - 1][i - 1] == BLACK_PIXEL) || i == 2)
		{
			return i;
		}
	}
	return 2;
}


uint8 left_point,right_point;
uint8 get_right_start(uint8 num)
{
	uint8 i = 0;
	for(i = MT9V03X_W*(num/4.0f); i < MT9V03X_W - 2; i++)
	{
		if ((PostProcessing_image[MT9V03X_H - 1][i] == WHITE_PIXEL && PostProcessing_image[MT9V03X_H - 1][i + 1] == BLACK_PIXEL ) || i == 185)
		{
			return i;
		}
	}
	return 185;
}





uint8 longest_line_number;           //记录最长白列的列位置，x方向上
uint8 line;                          //记录最长白列的长度，0代表最长，y方向，行

void research_longest_line(void)
{
    uint8 i, j;
    uint8 longest_length = 0;  // 记录最长白色列的长度
    uint8 pl = research_start_point();
    left_point = get_left_start(pl);
    right_point = get_right_start(pl);
    
    longest_line_number = MT9V03X_W/2;  // 默认值
    line = 119;  // 默认值（119是一个任意选择，代表较长的长度）
    
    // 确保左右边界合理
    if (left_point+1 >= right_point)
	{
		// 设置安全默认值或添加错误处理
		longest_line_number = MT9V03X_W / 2;
		line = 60;
		return;
	}
	
	uint8 exit_cnt_research = 0;
		
    
    for (i = left_point+1; i < right_point; i++)
    {

        uint8 bottom_continuous_length = 0;
		
		
        // 查找当前列白色区域的起始行
        for (j = MT9V03X_H - 1; j > 0; j--)
        {
			if (PostProcessing_image[j][i] == WHITE_PIXEL)
			{
				bottom_continuous_length++;
			}
			else
			{
				break;
			}

        }
		if (bottom_continuous_length < 15)  // 可根据需求调整（如至少3行才视为有效）
        {
            continue;
        }
        
        // 如果没有找到黑白边界，但整列都是白色 && bottom_continuous_length>0.8*MT9V03X_H
        if (j == 0 && PostProcessing_image[0][i] == WHITE_PIXEL && PostProcessing_image[MT9V03X_H/2][i] == WHITE_PIXEL && PostProcessing_image[MT9V03X_H-2][i] == WHITE_PIXEL)
        {
			line = 1;
			longest_line_number = i;
			break;
        }
		
//		if (longest_length >= MT9V03X_H*0.70f && bottom_continuous_length < longest_length && timer_3s_flag == 0)
//		{
//			exit_cnt_research++;
//			if (exit_cnt_research >= 5)
//			{
//				break;
//			}
//		}
//		else
//		{
//			exit_cnt_research = 0;
//		}
        
        // 更新最长白色列信息
        if (bottom_continuous_length > longest_length)
        {
            longest_length = bottom_continuous_length;
            longest_line_number = i;
            line = MT9V03X_H-bottom_continuous_length + 1;  // 记录白色区域的起始行
        }
    }
	if (longest_line_number >= RIGHT_MAX)
	{
		longest_line_number = RIGHT_MAX;
	}
	else if (longest_line_number <= LEFT_MAX)
	{
		longest_line_number = LEFT_MAX;
	}
}
uint8 road_left[120] = {0};         //记录x，或列数
uint8 break_point_l[6][2] ={{0}};
uint8 break_num_left;
uint8 break_point_r[6][2] = {{0}};
uint8 break_num_right;
uint8 road_right[120] = {0};       
uint8 left_flag_break;
uint8 right_flag_break;
void research_road(void)
{
	uint8 i,j;
	left_flag_break = 0;
	right_flag_break = 0;
	break_num_left = 0;
	break_num_right = 0; ////
	for (i = MT9V03X_H - 1; i >= line; i--)
	{
		for (j = longest_line_number; j > 1; j--)       //列
		{
			if ((PostProcessing_image[i][j] == WHITE_PIXEL && PostProcessing_image[i][j-1] == BLACK_PIXEL && PostProcessing_image[i][j-2] == BLACK_PIXEL) || j <= 2)
			{
				
				road_left[i] = j;
				if (road_left[i] <= 2)
				{
					Left_Lost_Time++;
				}

				left_flag_break++;
				if (left_flag_break >1)
					left_flag_break = 1;
				break;
			}
		}
		for (j = longest_line_number; j < MT9V03X_W - 2; j++)
		{
			if ((PostProcessing_image[i][j] == WHITE_PIXEL && PostProcessing_image[i][j+1] == BLACK_PIXEL && PostProcessing_image[i][j+2] == BLACK_PIXEL) || j >= 185)
			{
				road_right[i] = j;
				if (road_right[i] >= 185)
				{
					Right_Lost_Time++;
				}

				right_flag_break++;
				if (right_flag_break >1)
					right_flag_break = 1;
				break;
			}
		}
	}
//	merge_close_breakpoints();
}



// 合并左拐点的邻近拐点
void merge_left_breakpoints(void)
{
    if (break_num_left <= 1) return;  // 不足2个拐点，无需合并

    uint8 temp_l[6][2] = {{0}};  // 临时存储合并后的左拐点
    uint8 temp_num = 0;          // 合并后的左拐点数量

    // 保留第一个拐点作为初始值
    temp_l[temp_num][0] = break_point_l[0][0];
    temp_l[temp_num][1] = break_point_l[0][1];
    temp_num++;

    // 遍历剩余拐点，判断是否需要合并
    for (uint8 i = 1; i < break_num_left; i++)
    {
        // 计算当前拐点与上一个保留的拐点的行数差
        uint8 row_diff = break_point_l[i][0] - temp_l[temp_num-1][0];

        if (row_diff > BREAKPOINT_MERGE_THRESHOLD)
        {
            // 行数差超过阈值，保留当前拐点
            temp_l[temp_num][0] = break_point_l[i][0];
            temp_l[temp_num][1] = break_point_l[i][1];
            temp_num++;
            if (temp_num >= 6) break;  // 不超过数组最大容量
        }
        // 否则（行数差≤阈值），视为邻近拐点，不保留（合并到上一个）
    }

    // 更新左拐点数组和数量
    memcpy(break_point_l, temp_l, sizeof(temp_l));
    break_num_left = temp_num;
}

// 合并右拐点的邻近拐点（逻辑与左拐点完全一致）
void merge_right_breakpoints(void)
{
    if (break_num_right <= 1) return;  // 不足2个拐点，无需合并

    uint8 temp_r[6][2] = {{0}};  // 临时存储合并后的右拐点
    uint8 temp_num = 0;          // 合并后的右拐点数量

    // 保留第一个拐点作为初始值
    temp_r[temp_num][0] = break_point_r[0][0];
    temp_r[temp_num][1] = break_point_r[0][1];
    temp_num++;

    // 遍历剩余拐点，判断是否需要合并
    for (uint8 i = 1; i < break_num_right; i++)
    {
        // 计算当前拐点与上一个保留的拐点的行数差
        uint8 row_diff = break_point_r[i][0] - temp_r[temp_num-1][0];

        if (row_diff > BREAKPOINT_MERGE_THRESHOLD)
        {
            // 行数差超过阈值，保留当前拐点
            temp_r[temp_num][0] = break_point_r[i][0];
            temp_r[temp_num][1] = break_point_r[i][1];
            temp_num++;
            if (temp_num >= 6) break;  // 不超过数组最大容量
        }
        // 否则（行数差≤阈值），视为邻近拐点，不保留（合并到上一个）
    }

    // 更新右拐点数组和数量
    memcpy(break_point_r, temp_r, sizeof(temp_r));
    break_num_right = temp_num;
}

// 统一调用左、右拐点合并函数
void merge_close_breakpoints(void)
{
    merge_left_breakpoints();
    merge_right_breakpoints();
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     左下角点检测
  @param     起始行，终止行
  @return    返回角点所在的行数，找不到返回0
  Sample     left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Down_Point(int start,int end, uint8 mode)//找左下角点，返回值是角点所在的行数
{
    int i,t;
    int left_down_line=0;
    if(Left_Lost_Time>=0.8*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return left_down_line;
	if (mode == 0)
	{
		if(start<end)//--访问，要保证start>end
		{
			t=start;
			start=end;
			end=t;
		}
		if(start>=MT9V03X_H-1-5)//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
			start=MT9V03X_H-1-5;//另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
		if(end<=line)
			end=line;
		if(end<=5)
		   end=5;
		for(i=start;i>=end;i--)
		{
			if(left_down_line==0&&//只找第一个符合条件的点
			   abs(road_left[i]-road_left[i+1])<=5&&//角点的阈值可以更改
			   abs(road_left[i+1]-road_left[i+2])<=5&&  
			   abs(road_left[i+2]-road_left[i+3])<=5&&
				  (road_left[i]-road_left[i-2])>=8&&
				  (road_left[i]-road_left[i-3])>=12&&
				  (road_left[i]-road_left[i-4])>=15&&
			       road_left[i] < (MT9V03X_W/2 + OFFSET))
			{
				left_down_line=i;//获取行数即可
				break;
			}
		}
	}
	else if (mode == 1)
	{
		if(start>end)//--访问，要保证start<end
		{
			t=start;
			start=end;
			end=t;
		}
		if(end>=MT9V03X_H-1-5)//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
			end=MT9V03X_H-1-5;//另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
		if(start<=line)
			start=line;
		if(start<=5)
		   start=5;
		for(i=start;i<=end;i++)
		{
			if(left_down_line==0&&//只找第一个符合条件的点
			   abs(road_left[i]-road_left[i+1])<=5&&//角点的阈值可以更改
			   abs(road_left[i+1]-road_left[i+2])<=5&&  
			   abs(road_left[i+2]-road_left[i+3])<=5&&
				  (road_left[i]-road_left[i-2])>=8&&
				  (road_left[i]-road_left[i-3])>=12&&
				  (road_left[i]-road_left[i-4])>=15&&
			       road_left[i] < (MT9V03X_W/2 + OFFSET))
			{
				left_down_line=i;//获取行数即可
				break;
			}
		}
	}
    
    return left_down_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左上角点检测
  @param     起始行，终止行
  @return    返回角点所在的行数，找不到返回0
  Sample     left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Up_Point(int start,int end,uint8 mode)//找左下角点，返回值是角点所在的行数
{
    int i,t;
    int left_up_line=0;
    if(Left_Lost_Time>=0.8*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return left_up_line;
	if (mode == 0)  //0从下往上找
	{
		if(start<end)//--访问，要保证start>end
		{
			t=start;
			start=end;
			end=t;
		}
		if(start>=MT9V03X_H-1-5)//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
			start=MT9V03X_H-1-5;//另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
		if(end<=line)
			end=line;
		if(end<=5)
		   end=5;
		for(i=start;i>=end;i--)
		{
			if(left_up_line==0&&//只找第一个符合条件的点
			   abs(road_left[i]-road_left[i-1])<=5&&//角点的阈值可以更改
			   abs(road_left[i-1]-road_left[i-2])<=5&&
			   abs(road_left[i-2]-road_left[i-3])<=5&&
				  (road_left[i]-road_left[i+2])>=8&&
				  (road_left[i]-road_left[i+3])>=12&&
				  (road_left[i]-road_left[i+4])>=15&&
			       road_left[i] < (MT9V03X_W/2 + OFFSET))
			{
				left_up_line=i;//获取行数即可
				break;
			}
		}
		
	}
	else if (mode == 1)  //1从上往下找
	{
		if(start>end)//--访问，要保证start>end
		{
			t=start;
			start=end;
			end=t;
		}
		if(end>=MT9V03X_H-1-5)//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
			end=MT9V03X_H-1-5;//另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
		if(start<=line)
			start=line;
		if(start<=5)
		   start=5;
		for(i=start;i<=end;i++)
		{
			if(left_up_line==0&&//只找第一个符合条件的点
			   abs(road_left[i]-road_left[i-1])<=5&&//角点的阈值可以更改
			   abs(road_left[i-1]-road_left[i-2])<=5&&  
			   abs(road_left[i-2]-road_left[i-3])<=5&&
				  (road_left[i]-road_left[i+2])>=8&&
				  (road_left[i]-road_left[i+3])>=12&&
				  (road_left[i]-road_left[i+4])>=15&&
			       road_left[i] < (MT9V03X_W/2 + OFFSET))
			{
				left_up_line=i;//获取行数即可
				break;
			}
		}

	}
    return left_up_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右下角点检测
  @param     起始行，终止行
  @return    返回角点所在的行数，找不到返回0
  Sample     left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Down_Point(int start,int end, uint8 mode)//找左下角点，返回值是角点所在的行数
{
    int i,t;
    int right_down_line=0;
    if(Right_Lost_Time>=0.8*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return right_down_line;
	if (mode == 0)
	{
		if(start<end)//--访问，要保证start>end
		{
			t=start;
			start=end;
			end=t;
		}
		if(start>=MT9V03X_H-1-5)//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
			start=MT9V03X_H-1-5;//另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
		if(end<=line)
			end=line;
		if(end<=5)
		   end=5;
		for(i=start;i>=end;i--)
		{
			if(right_down_line==0&&//只找第一个符合条件的点
			   abs(road_right[i]-road_right[i+1])<=5&&//角点的阈值可以更改
			   abs(road_right[i+1]-road_right[i+2])<=5&&  
			   abs(road_right[i+2]-road_right[i+3])<=5&&
				  (road_right[i-2]-road_right[i])>=8&&
				  (road_right[i-3]-road_right[i])>=12&&
				  (road_right[i-4]-road_right[i])>=15&&
			       road_right[i] > (MT9V03X_W/2 - OFFSET))
			{
				right_down_line=i;//获取行数即可
				break;
			}
		}
	}
	else if (mode == 1)
	{
		if(start>end)//--访问，要保证start<end
		{
			t=start;
			start=end;
			end=t;
		}
		if(end>=MT9V03X_H-1-5)//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
			end=MT9V03X_H-1-5;//另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
		if(start<=line)
			start=line;
		if(start<=5)
		   start=5;
		for(i=start;i<=end;i++)
		{
			if(right_down_line==0&&//只找第一个符合条件的点
			   abs(road_right[i]-road_right[i+1])<=5&&//角点的阈值可以更改
			   abs(road_right[i+1]-road_right[i+2])<=5&&  
			   abs(road_right[i+2]-road_right[i+3])<=5&&
				  (road_right[i-2]-road_right[i])>=8&&
				  (road_right[i-3]-road_right[i])>=12&&
				  (road_right[i-4]-road_right[i])>=15&&
			       road_right[i] > (MT9V03X_W/2 - OFFSET))
			{
				right_down_line=i;//获取行数即可
				break;
			}
		}
	}
    
    return right_down_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右上角点检测
  @param     起始行，终止行
  @return    返回角点所在的行数，找不到返回0
  Sample     left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Up_Point(int start,int end,uint8 mode)//找左下角点，返回值是角点所在的行数
{
    int i,t;
    int right_up_line=0;
    if(Right_Lost_Time>=0.8*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return right_up_line;
	if (mode == 0)
	{
		if(start<end)//--访问，要保证start>end
		{
			t=start;
			start=end;
			end=t;
		}
		if(start>=MT9V03X_H-1-5)//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
			start=MT9V03X_H-1-5;//另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
		if(end<=line)
			end=line;
		if(end<=5)
		   end=5;
		for(i=start;i>=end;i--)
		{
			if(right_up_line==0&&//只找第一个符合条件的点
			   abs(road_right[i]-road_right[i-1])<=5&&//角点的阈值可以更改
			   abs(road_right[i-1]-road_right[i-2])<=5&&  
			   abs(road_right[i-2]-road_right[i-3])<=5&&
				  (road_right[i+2]-road_right[i])>=8&&
				  (road_right[i+3]-road_right[i])>=12&&
				  (road_right[i+4]-road_right[i])>=15&&
			       road_right[i] > (MT9V03X_W/2 - OFFSET))
			{
				right_up_line=i;//获取行数即可
				break;
			}
		}
	}
	else if (mode == 1)
	{
		if(start>end)//--访问，要保证start>end
		{
			t=start;
			start=end;
			end=t;
		}
		if(end>=MT9V03X_H-1-5)//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
			end=MT9V03X_H-1-5;//另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
		if(start<=line)
			start=line;
		if(start<=5)
		   start=5;
		for(i=start;i<=end;i++)
		{
			if(right_up_line==0&&//只找第一个符合条件的点
			   abs(road_right[i]-road_right[i-1])<=5&&//角点的阈值可以更改
			   abs(road_right[i-1]-road_right[i-2])<=5&&  
			   abs(road_right[i-2]-road_right[i-3])<=5&&
				  (road_right[i+2]-road_right[i])>=8&&
				  (road_right[i+3]-road_right[i])>=12&&
				  (road_right[i+4]-road_right[i])>=15&&
			       road_right[i] > (MT9V03X_W/2 - OFFSET))
			{
				right_up_line=i;//获取行数即可
				break;
			}
		}
	}
    return right_up_line;
}




uint8 huandao_flag = 0;

uint8 l_d_num = 0;
uint8 l_u_num = 0;
uint8 r_d_num = 0;
uint8 r_u_num = 0;
uint8 center_line[120];
int num_123;
uint8 see_flag = 0;
void image_process(void)
{

	uint16 value_count;
	uint8 curve_num = 0;
	image_postprocess();                                                          //图像处理
	
	research_longest_line();                                                      //寻找赛道边界
	research_road();                                                              //
	color_binary_detection();
//	if (blob_info.valid == 0)
//	{
//		car_stop();                                                                   //冲出保护与斑马线检测
//	}
	
	Zebra_crossing_handle();                                                      //
	
	l_d_num = Find_Left_Down_Point(MT9V03X_H-1,line,0);                           //寻找四大拐点
	l_u_num = Find_Left_Up_Point(MT9V03X_H-40,line,0);                             //
	r_d_num = Find_Right_Down_Point(MT9V03X_H-1,line,0);
	r_u_num = Find_Right_Up_Point(MT9V03X_H-40,line,0);
//	if (l_d_num > 0 && l_d_num < l_u_num && l_u_num >= MT9V03X_H-40)
//	{
//		l_u_num = Find_Left_Up_Point(l_d_num,line,0);
//	}
	if (l_d_num > 0 && l_d_num < l_u_num)
	{
		l_d_num = 0;
	}
//	if (r_d_num > 0 && r_d_num < r_u_num && r_u_num >= MT9V03X_H-40)
//	{
//		r_u_num = Find_Right_Up_Point(r_d_num,line,0);
//	}
	if (r_d_num > 0 && r_d_num < r_u_num && r_u_num)
	{
		r_d_num = 0;
	}
	

	crossing_add(l_d_num,l_u_num,r_d_num,r_u_num);                                //十字
	
	if (crossing_flag_help == 0)                                                  //圆环
	{                                                                             //
		if (left_right_choose == -1)
		{
			yuanhuan_left_in_handle();
		}
		else if (left_right_choose == 1)
		{
			yuanhuan_right_in_handle();
		}
		
	}
	
	
	for (uint8 i = line; i < MT9V03X_H; i++)                                      //求中线
	{                                                                             //
		center_line[i] = (road_left[i] + road_right[i]) >> 1;
	}

//	curve_num =  Detect_S_Curve(center_line, line);
	
	final_mid_line = find_mid_line_weight();
	line_error = final_mid_line - MID_W;
		
}

uint8 left_test = 0;
uint8 right_test = 0;

void image_show(void)
{
	ips200_show_gray_image(0, 50, PostProcessing_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
	ips200_draw_line(longest_line_number, 50, longest_line_number, 50+MT9V03X_H, RGB565_YELLOW);
	ips200_draw_line(0, 50+line, MT9V03X_W, 50+line, RGB565_RED);
	ips200_draw_line(0, 50+qianzhan_value, MT9V03X_W, 50+qianzhan_value, RGB565_PINK);
	for (uint8 i = line; i < MT9V03X_H; i++)
	{
		ips200_draw_point(road_left[i], i+50,RGB565_GREEN);
		ips200_draw_point(road_right[i], i+50,RGB565_BLUE);
		ips200_draw_point(center_line[i], i+50,RGB565_RED);
	}
	ips200_draw_point(road_left[l_d_num],l_d_num+50,RGB565_RED);
	ips200_draw_point(road_left[l_u_num],l_u_num+50,RGB565_RED);
	ips200_draw_point(road_right[r_d_num],r_d_num+50,RGB565_RED);
	ips200_draw_point(road_right[r_u_num],r_u_num+50,RGB565_RED);
	
	ips200_show_uint(0,208,l_d_num,3);
	ips200_show_uint(30,208,l_u_num,3);
	ips200_show_uint(60,208,r_d_num,3);
	ips200_show_uint(90,208,r_u_num,3);
	
	ips200_show_uint(120,208,final_mid_line,3);

//	ips200_show_uint(150,208,left_test,1);
//	ips200_show_uint(170,208,right_test,1);
	
//	ips200_show_uint(150, 208, line,3);
//	ips200_show_uint(170, 208, Zebra_end_flag,1);
	ips200_show_int(150, 208, (int)blob_info.distance, 3);
	ips200_show_uint(180, 208, blob_info.valid, 1);

	ips200_show_uint(200,208,push_box_flag,1);
//	ips200_show_uint(150,208,l_d_num,3);
//	ips200_show_uint(180,208,r_d_num,3);
}


void image_data_clear(void)
{
	break_num_left = 0;
	break_num_right = 0;
	Left_Lost_Time = 0;
	Right_Lost_Time = 0;
//	huandao_flag = 0;
	crossing_flag_help = 0;

	
	memset(road_left, 0, sizeof(road_left));
	memset(road_right, 0, sizeof(road_right));
	
	memset(center_line, 0, sizeof(center_line));
}


void huandao_clear(void)
{
	huandao_flag = 0;
	huandao_num_flag = 0;
}



void speed_strategy(void)
{
    path_follow_output_t pf = {0};

    path_follow_update(yaw_angle, &pf);

    if (pf.active)
    {
        speed_three_array[0] = pf.vx_cmd;
        speed_three_array[1] = pf.vy_cmd;
        speed_three_array[2] = pf.omega_cmd;
    }
    else
    {
        speed_three_array[0] = 0;
        speed_three_array[1] = 0;
        speed_three_array[2] = 0;
    }

    Kinematics_Inverse(speed_three_array, speed_encoder);
}



//------------------------------------------------------------------------------------------------------------------
//函数名称：uint8 crossing_handle(void)
//功能说明：最小二乘法函数
//参数说明：无
//函数返回：
//------------------------------------------------------------------------------------------------------------------

void least_squares(uint8 begin, uint8 end, uint8 *border, float *xielv, float *jieju)
{
	float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
	int16 i = 0;
	*xielv = 0;
	*jieju = 0;

	for (i = begin; i < end; i++)
	{
		sum_x += i;
		sum_y += border[i];
		sum_xy += i * (border[i]);
		sum_x2 += i * i;

	}
	if ((end - begin)*sum_x2 - sum_x * sum_x) //判断除数是否为零
	{
		*xielv = ((end - begin)*sum_xy - sum_x * sum_y) / ((end - begin)*sum_x2 - sum_x * sum_x);

	}
	*jieju = (sum_y - (*xielv) * sum_x) / (end - begin);
}
void xieji(uint8 begin, uint8 end, uint8 y_begin, uint8 y_end,uint8 arr[])
{
	uint8 i = 0;
	float xielv;
	float jieju;
    if (end == begin)  // 避免除零错误
    {
        xielv = 0;
        jieju = y_begin;
		return;
    }
    
    // 计算斜率和截距，使用浮点数运算
    xielv = (float)(y_end - y_begin) / (float)(end - begin);
    jieju = y_begin - (xielv) * begin;
	
	for (i = begin; i < end; i++)
    {
		arr[i] = (uint8)(xielv * i + jieju + 0.5f);
        if (arr[i] <= 1)
			arr[i] = 1;
        else if (arr[i] >= MT9V03X_W - 2)
            arr[i] = MT9V03X_W - 2;
    }
}

//十字补线
void crossing_add(uint8 num_d_l, uint8 num_u_l, uint8 num_d_r, uint8 num_u_r)
{
    float xielv_l, jieju_l;
    float xielv_r, jieju_r;
    uint16 i;
	int temp;
    
	
	
    // 正入十字
    if (num_d_l&& num_u_l&&num_d_r&&num_u_r && huandao_flag == 0 && Zebra_stop_flag == 0 && road_left[num_u_l] < road_right[num_u_r] && road_left[num_d_l] < road_right[num_d_r]
		&& abs(num_d_l - num_d_r) < 20 && abs(num_u_l-num_u_r) < 20 && num_d_l - num_u_l > 15 && num_d_r - num_u_r > 15 && (road_left[num_u_l] >= road_left[num_d_l] - 5)
	&& (road_right[num_u_r] <= road_right[num_d_r] + 5))
    {//abs(num_u_l-num_u_r) <=10 && abs(num_d_l-num_d_r) <=10 && 

        if (abs(num_d_l-num_u_l) >= 10 && abs(num_d_r - num_u_r) >= 10 && Continuity_Change_Left(MT9V03X_H-5, line + 10,line) != 0 && Continuity_Change_Right(MT9V03X_H-5, line + 10,line) != 0)// num_l >= 2
        {
            // 使用最后两个断点进行线性插值
            xieji(num_u_l, num_d_l, road_left[num_u_l], road_left[num_d_l],road_left);
            
			xieji(num_u_r, num_d_r, road_right[num_u_r], road_right[num_d_r],road_right);
            
			huandao_clear();
			crossing_flag_help = 1;
        }
		
    }
	 //正入十字后
	else if (num_u_l >= MT9V03X_H/2-50 && num_u_r >= MT9V03X_H/2-50 && huandao_flag == 0 && Zebra_stop_flag == 0 && (road_right[num_u_r]-road_left[num_u_l] > 15))//////////////
	{
		if (num_d_l > 0 && num_d_r == 0 && abs(num_d_l-num_u_l) >= 20 && (road_left[num_u_l] >= road_left[num_d_l] - 5) && abs(num_u_l - num_u_r) <= 15)                        //斜入十字，三点补线
		{
			xieji(num_u_l, num_d_l, road_left[num_u_l], road_left[num_d_l],road_left);
			least_squares(num_u_r-6,num_u_r-1,road_right,&xielv_r,&jieju_r);
            for (i = num_u_r - 2; i < MT9V03X_H - 1; i++)
            {
                road_right[i] = xielv_r * i + jieju_r;
                if (road_right[i] <= 1)
                    road_right[i] = MT9V03X_W - 2;
                else if (road_right[i] >= MT9V03X_W - 2)
                    road_right[i] = MT9V03X_W - 2;
            }
			crossing_flag_help = 1;
		}
		else if (num_d_l == 0 && num_d_r > 0 && abs(num_d_r - num_u_r) >= 20 && (road_right[num_u_r] <= road_right[num_d_r] + 5) && abs(num_u_l - num_u_r) <= 15)                 //斜入十字，三点补线
		{
			xieji(num_u_r, num_d_r, road_right[num_u_r], road_right[num_d_r],road_right);
			least_squares(num_u_l-6,num_u_l-1,road_left,&xielv_l,&jieju_l);
			for (i = num_u_l - 2; i < MT9V03X_H - 1; i++)
            {
                road_left[i] = xielv_l * i + jieju_l;
                if (road_left[i] <= 1)
                    road_left[i] = 1;
                else if (road_left[i] >= MT9V03X_W - 2)
                    road_left[i] = 1;
            }
			crossing_flag_help = 1;
		}
		if (abs(num_u_l - num_u_r) < 20 && num_d_l == 0 && num_d_r == 0)
		{
			least_squares(num_u_l-8,num_u_l-1,road_left,&xielv_l,&jieju_l);
            // 补线并限制边界
            for (i = num_u_l - 5; i < MT9V03X_H - 1; i++)
            {
                road_left[i] = xielv_l * i + jieju_l;
                if (road_left[i] <= 1)
                    road_left[i] = 1;
                else if (road_left[i] >= MT9V03X_W - 2)
                    road_left[i] = 1;
            }
			least_squares(num_u_r-8,num_u_r-1,road_right,&xielv_r,&jieju_r);
            for (i = num_u_r - 5; i < MT9V03X_H - 1; i++)
            {
                road_right[i] = xielv_r * i + jieju_r;
                if (road_right[i] <= 1)
                    road_right[i] = MT9V03X_W - 2;
                else if (road_right[i] >= MT9V03X_W - 2)
                    road_right[i] = MT9V03X_W - 2;
            }
			uint8 left_num_down_add = Find_Left_Down_Point(num_u_l-5,line+2,0);
	        uint8 left_num_up_add = Find_Left_Up_Point(num_u_l-5,line+2,0);
	        uint8 right_num_down_add = Find_Right_Down_Point(num_u_r-5,line+2,0);
	        uint8 right_num_up_add = Find_Right_Up_Point(num_u_r-5,line+2,0);
			if (left_num_down_add && left_num_up_add && right_num_down_add && right_num_up_add && road_left[left_num_down_add] > 35
				&& road_left[left_num_up_add] > 45 && road_right[right_num_down_add] < MT9V03X_W - 35 && road_right[right_num_up_add] < MT9V03X_W-45)
			{
				xieji(left_num_up_add, left_num_down_add, road_left[left_num_up_add], road_left[left_num_down_add],road_left);
				xieji(right_num_up_add, right_num_down_add, road_right[right_num_up_add], road_right[right_num_down_add],road_right);
			}
			crossing_flag_help = 1;
		}
	}
	else if (num_u_l && num_d_l && num_u_r == 0 && num_d_r == 0 && huandao_flag == 0 && Zebra_stop_flag == 0 && Right_Lost_Time > (MT9V03X_H-line)*0.65 && (road_left[num_u_l] >= road_left[num_d_l] - 5)
		&& num_d_l - num_u_l > 15)
	{
		xieji(num_u_l, num_d_l, road_left[num_u_l], road_left[num_d_l],road_left);
	}
	else if (num_u_l == 0 && num_d_l == 0 && num_u_r && num_d_r && huandao_flag == 0 && Zebra_stop_flag == 0 && Left_Lost_Time > (MT9V03X_H-line)*0.65 && (road_right[num_u_r] <= road_right[num_d_r] + 5)
		&& num_d_r - num_u_r > 15)
	{
		xieji(num_u_r, num_d_r, road_right[num_u_r], road_right[num_d_r],road_right);
	}
}

uint8 my_max(uint8 num1, uint8 num2)
{
	if (num1 > num2)
		return num1;
	else
		return num2;
}
uint8 num_line = 60;
/*********************************************************
** 备注：权重可通过图像权重数组调节
*********************************************************/
float find_mid_line_weight(void)
{
    float mid_line_value = MID_W;       // 最终中线输出值
    float mid_line = MID_W;             // 本次中线值
    float weight_midline_sum = 0;      // 加权中线累加值
    float weight_sum = 0;              // 权重累加值
	num_line = MT9V03X_H/2;
//	if (low_high_choose == 0)
//	{
//		for(uint8 i = MT9V03X_H - 1; i > line; i--)
//		{
//			weight_midline_sum += center_line[i] * mid_weight_list[i];
//			weight_sum += mid_weight_list[i];
//		}
////	}
////	else if (low_high_choose == 1)
////	{
////		for(uint8 i = MT9V03X_H - 1; i > line; i--)
////		{
////			weight_midline_sum += center_line[i] * low_weight_list[i];
////			weight_sum += low_weight_list[i];
////		}
////	}
//    







//	for (uint8 i = MT9V03X_H-15; i > ((MT9V03X_H/2-10)>line?(MT9V03X_H/2-10):line); i--)
//	{
//		if (abs(center_line[i] - center_line[i-1]) < 11)
//		{
//			weight_midline_sum += center_line[i] * mid_weight_list[i];
//			weight_sum += mid_weight_list[i];
//		}
//		else
//		{
//			num_line = i;
//			break;
//		}
//	}

//    mid_line = (float)(weight_midline_sum / weight_sum);
//    mid_line_value = last_mid_line * 0.2 + mid_line * 0.8; // 互补滤波
//    last_mid_line = mid_line_value;
////	mid_line_value = center_line[70];
	
	if (line > qianzhan_value && line < MT9V03X_H-6)
	{
		for (uint8 i = line+1; i < line+6; i++)
		{
			weight_midline_sum += center_line[i];
		}
	}
	else
	{
			for (uint8 i = qianzhan_value; i < qianzhan_value+5; i++)
		{
			weight_midline_sum += center_line[i];
		}
	}

	
	mid_line = (float)(weight_midline_sum / 5);
	mid_line_value = last_mid_line * 0.2 + mid_line * 0.8; // 互补滤波
	last_mid_line = mid_line_value;
	
    return mid_line_value;
}


void Zebra_crossing_handle(void)
{
	uint8 i ,j;
	uint8 Zebra_crossing_num = 0;
	for (i = MT9V03X_H/2; i < MT9V03X_H; i+=3)
	{
		for (j = 20; j < 168; j++)
		{
			if ((PostProcessing_image[i][j+1] == BLACK_PIXEL && PostProcessing_image[i][j] == BLACK_PIXEL && PostProcessing_image[i][j-1] == WHITE_PIXEL && PostProcessing_image[i][j-2] == WHITE_PIXEL) 
				|| (PostProcessing_image[i][j-1] == BLACK_PIXEL && PostProcessing_image[i][j] == BLACK_PIXEL && PostProcessing_image[i][j+1] == WHITE_PIXEL && PostProcessing_image[i][j+2] == WHITE_PIXEL))
			{
				Zebra_crossing_num++;
			}
		}
		if (Zebra_crossing_num >= 10)
		{
			Zebra_stop_flag = 1;
			break;
		}
		else
		{
			Zebra_crossing_num = 0;
			Zebra_stop_flag = 0;
		}
	}

}

//// 优化参数（进一步减少计算）
//#define ZEBRA_ROW_STEP 2         // 行间隔采样（每2行扫1行）
//#define ZEBRA_COL_STEP 2         // 列间隔采样（每2列扫1列）
//#define ZEBRA_ROW_RATIO_START 0.4f  // 检测行范围上移（更聚焦中下部）
//#define ZEBRA_ROW_RATIO_END   0.7f
//#define MIN_EDGES_PER_ROW 10      // 降低边缘计数阈值（因间隔采样）
//#define MIN_CONTINUOUS_ROWS 2
//#define MIN_EDGE_GAP 4           // 增大间距阈值（间隔采样后边缘更稀疏）

//uint8 Zebra_stop_flag = 0;

//void Zebra_crossing_handle(void)
//{
//    // 1. 动态计算更小的检测范围
//    uint8 start_row = (uint8)(MT9V03X_H * ZEBRA_ROW_RATIO_START);
//    uint8 end_row = (uint8)(MT9V03X_H * ZEBRA_ROW_RATIO_END);
//    uint8 start_col = (uint8)(MT9V03X_W * 0.15f);  // 进一步缩小列范围
//    uint8 end_col = (uint8)(MT9V03X_W * 0.85f);

//    uint8 continuous_valid_rows = 0;

//    // 2. 行间隔采样（每ZEBRA_ROW_STEP行扫1行，减少50%行遍历）
//    for (uint8 i = start_row; i <= end_row; i += ZEBRA_ROW_STEP)
//    {
//        uint8 edge_count = 0;
//        uint8 last_edge_col = 0;

//        // 3. 列间隔采样（每ZEBRA_COL_STEP列扫1列，减少50%列遍历）
//        for (uint8 j = start_col; j < end_col - ZEBRA_COL_STEP; j += ZEBRA_COL_STEP)
//        {
//            // 简化边缘判断：只检测间隔后的列跳变（等效原逻辑但计算更少）
//            if (PostProcessing_image[i][j] != PostProcessing_image[i][j + ZEBRA_COL_STEP])
//            {
//                // 过滤近邻噪声（因间隔采样，边缘间距阈值可增大）
//                if (edge_count == 0 || (j - last_edge_col) >= MIN_EDGE_GAP)
//                {
//                    edge_count++;
//                    last_edge_col = j;
//                    // 提前退出
//                    if (edge_count > MIN_EDGES_PER_ROW) break;
//                }
//            }
//        }

//        // 4. 简化判定逻辑
//        if (edge_count >= MIN_EDGES_PER_ROW)
//        {
//            if (++continuous_valid_rows >= MIN_CONTINUOUS_ROWS)
//            {
//                Zebra_stop_flag = 1;
//                return;  // 找到后立即退出，减少后续计算
//            }
//        }
//        else
//        {
//            continuous_valid_rows = 0;
//        }
//    }

//    Zebra_stop_flag = 0;
//}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     左赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     continuity_change_flag=Continuity_Change_Left(int start,int end)
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Left(int start,int end, uint8 ref_line)
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Right_Lost_Time>=0.8*(MT9V03X_H - ref_line))//大部分都丢线，没必要判断了
       return 1;
    if(start>=MT9V03X_H-5)//数组越界保护
        start=MT9V03X_H-5;
    if(end<=15)
       end=15;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
 
    for(i=start;i>=end;i--)
    {
        if(abs(road_left[i]-road_left[i-1])>=5)//连续性阈值是5，可更改
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     continuity_change_flag=Continuity_Change_Right(int start,int end)
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Right(int start,int end, uint8 ref_line)
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Right_Lost_Time>=0.8*(MT9V03X_H - ref_line))//大部分都丢线，没必要判断了
       return 1;
    if(start>=MT9V03X_H-5)//数组越界保护
        start=MT9V03X_H-5;
    if(end<=15)
       end=15;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
       t=start;
       start=end;
       end=t;
    }
 
    for(i=start;i>=end;i--)
    {
        if(abs(road_right[i]-road_right[i-1])>=5)//连续性阈值是5，可更改
       {
            continuity_change_flag=i;
            break;
       }
    }
    return continuity_change_flag;
}


int find_min(uint8 arr[], int start, int end) {
	//从下往上找点
	int circle_num = 0;
	
    if (start < end || start < 0 || end < 0) {
        return -1; // 错误处理
    }
	if (start >= MT9V03X_H - 8)
	{
		start = MT9V03X_H - 8;
	}

    for (int i = start - 1; i >= end; i--) {
        if (arr[i] <= arr[i-1] && 
			arr[i-1] <= arr[i-2] && 
			arr[i-2] < arr[i-3] &&
			arr[i-3] < arr[i-4] &&
			arr[i] <= arr[i+1] &&
			arr[i+1] <= arr[i+2] &&
			arr[i+2] < arr[i+3] &&
			arr[i+3] < arr[i+4])
		{
            circle_num = i;
			return circle_num;
        }
    }
    return circle_num;
}

int find_max(uint8 arr[], int start, int end) {
	//从下往上找点
	int circle_num = 0;
	
    if (start < end || start < 0 || end < 0) {
        return -1; // 错误处理
    }
	if (start >= MT9V03X_H - 5)
	{
		start = MT9V03X_H - 5;
	}

    for (int i = start - 1; i >= end; i--) {
        if (arr[i] >= arr[i-1] && 
			arr[i-1] >= arr[i-2] && 
			arr[i-2] >= arr[i-3] &&
			arr[i-3] > arr[i-4] &&
			arr[i] >= arr[i+1] &&
			arr[i+1] >= arr[i+2] &&
			arr[i+2] >= arr[i+3] &&
			arr[i+3] > arr[i+4])
		{
            circle_num = i;
			return circle_num;
        }
    }
    return circle_num;
}




uint8 research_20point(uint8 num,uint8 arr[])
{
	if (num >= MT9V03X_H - 5)
	{
		return 0;
	}
	uint8 i = 0;
	for (i = MT9V03X_H - 5; i > num; i--)
	{
		if (arr[i] < MT9V03X_W - 15)
		{
			return i;
			
		}
	}
	return 0;
}


int find_min_min(uint8 arr[], int start, int end) {
	//从下往上找点
	int circle_num = 0;
	
    if (start < end || start < 0 || end < 0) {
        return -1; // 错误处理
    }
	if (start >= MT9V03X_H - 8)
	{
		start = MT9V03X_H - 8;
	}

    for (int i = start - 1; i >= end; i--) {
        if (arr[i] <= arr[i-1] && 
			arr[i-1] <= arr[i-2] && 
			arr[i-2] <= arr[i-3] &&
			arr[i-3] <= arr[i-4] &&
			arr[i] <= arr[i+1] &&
			arr[i+1] <= arr[i+2] &&
			arr[i+2] <= arr[i+3] &&
			arr[i+3] <= arr[i+4 ]&&
			arr[i] < MT9V03X_W-6) 
		{
            circle_num = i;
			return circle_num;
        }
    }
    return circle_num;
}

int find_max_max(uint8 arr[], int start, int end) {
	//从下往上找点
	int circle_num = 0;
	
    if (start < end || start < 0 || end < 0) {
        return -1; // 错误处理
    }
	if (start >= MT9V03X_H - 8)
	{
		start = MT9V03X_H - 8;
	}

    for (int i = start - 1; i >= end; i--) {
        if (arr[i] >= arr[i-1] && 
			arr[i-1] >= arr[i-2] && 
			arr[i-2] >= arr[i-3] &&
			arr[i-3] >= arr[i-4] &&
			arr[i] >= arr[i+1] &&
			arr[i+1] >= arr[i+2] &&
			arr[i+2] >= arr[i+3] &&
			arr[i+3] >= arr[i+4 ]&&
			arr[i] > 6) 
		{
            circle_num = i;
			return circle_num;
        }
    }
    return circle_num;
}

uint8 find_max_limit(uint8 num1,uint8 num2)
{
	if (num1 > num2)
	{
		return num1;
	}
	else
	{
		return num2;	
	}
}

uint8 Continuity_reduce_left(uint8 arr[])
{
	uint8 i = 0;
	uint8 reduce_left = 0;
	uint8 wrong_num = 0;
	for (i = MT9V03X_H - 5; i > MT9V03X_H - 40; i--)
	{
//		if (arr[i] - arr[i-3] < -)
		if (arr[i] -arr[i-1] > 0)
		{
			wrong_num++;
		}
		if (wrong_num > 2)
		{
			reduce_left = 1;
			break;
		}
	}
	return reduce_left;
}

/**
 * @brief 检测左边界是否存在圆弧（从下往上处理行：start_row > end_row）
 * @param arr       边界列坐标数组（如 road_left）
 * @param start_row 起始行（更靠下，行号大）
 * @param end_row   结束行（更靠上，行号小）
 * @return arr是左边界，arr_another是右边界
 */
uint8 detect_left_arc(uint8 arr[],uint8 arr_another[], uint8 start_row, uint8 end_row) {
    // 1. 合法性检查：必须从下往上（start_row > end_row）
    if (start_row <= end_row) return 0; 

    int8 diff1[MT9V03X_H] = {0}; // 一阶差分：当前行（下） - 上一行（上）的列变化
    int8 diff2[MT9V03X_H] = {0}; // 二阶差分：一阶差分的变化率（反映弯曲）
	uint8 num = 0;
	uint8 wrong_num = 0;
	uint8 sum = 0;

    // 2. 计算一阶差分（从下往上遍历行，i递减）
    for (uint8 i = start_row; i > end_row; i--) {
        if (i == 0) break; // 防止 i-1 越界（行号0是最顶部）
        diff1[i] = arr[i] - arr[i-1]; // 当前行（下） - 上一行（上）的列变化
		if (diff1[i] > 0)
		{
			wrong_num++;
		}
		if (wrong_num > 2)
		{
			return 0;
		}
		if (arr_another[i] >= MT9V03X_W-3)
		{
			sum++;
		}
    }

    // 3. 计算二阶差分（反映斜率的变化，i递减遍历）
    for (uint8 i = start_row - 1; i > end_row; i--) {
        if (i + 1 >= MT9V03X_H) break; // 防止 i+1 越界
        diff2[i] = diff1[i] - diff1[i+1]; // 当前差分 - 下一行（更靠下）的差分
		if (abs(diff2[i]) >=1)
		{
			num++;
		}
    }
	if (num > 5 && sum >= (start_row-end_row-10))
	{
		return 1;
	}
	else
	{
		return 0;
	}
	

}

uint8 research_black_point(void)
{
	uint8 i = 0;
	for (i = MT9V03X_H-5; i > 5; i--)
	{
		if (PostProcessing_image[i][MT9V03X_W-4]==WHITE_PIXEL && PostProcessing_image[i-1][MT9V03X_W-4]==BLACK_PIXEL && PostProcessing_image[i-2][MT9V03X_W-4]==BLACK_PIXEL)
		{
			return i;
		}
	}
	return 0;
}


//uint8  yuanhuan_see_handle(void)
//{
//	uint8 yuanhuan_qiedian = 0;
//	uint8 see_huanyuan_flag = 0;
////	float xielv,jieju;
//	if (Continuity_Change_Left(MT9V03X_H, line + 2) == 0 && Continuity_Change_Right(MT9V03X_H, line + 2) != 0 && r_u_num > 0 && r_d_num > 0)
//	{
//		see_huanyuan_flag = 1;
//		yuanhuan_qiedian = find_min(road_right, r_u_num+5, 10);
//		huandao_flag = 1;
//		if (yuanhuan_qiedian && huandao_flag == 1)
//		{
//			xieji(yuanhuan_qiedian, r_d_num, road_right[yuanhuan_qiedian], road_right[r_d_num], road_right);
//            
////            for (uint8 i = yuanhuan_qiedian; i < r_d_num; i++)
////            {
////                road_right[i] = xielv * i + jieju;
////                if (road_right[i] <= 1)
////                    road_right[i] = 1;
////                else if (road_right[i] >= MT9V03X_W - 2)
////                    road_right[i] = MT9V03X_W - 2;
////            }
//		}
//		
//	}
//	return see_huanyuan_flag;
//}


static uint8 exit_cnt_1 = 0;
static uint8 exit_cnt_2_1 = 0;
static uint8 exit_cnt_2_2 = 0;
static uint8 exit_cnt_3 = 0;
static uint8 exit_cnt_4 = 0;
static uint8 exit_cnt_0 = 0;

void yuanhuan_right_in_handle(void)
{
	float xielv,jieju;
	uint8 u_num = 0;
	
	u_num = Find_Right_Up_Point(MT9V03X_H-1,line,1);
	if (huandao_flag == 0 && huandao_num_flag == 0)
	{
		
		if (Continuity_Change_Left(MT9V03X_H-25, line + 2,line) == 0 && Continuity_Change_Right(MT9V03X_H, line + 2,line) != 0 && r_u_num > 0 && r_d_num > 0 && line < 10 && Left_Lost_Time <= MT9V03X_H/8 && Right_Lost_Time >= MT9V03X_H/3)
		{
			exit_cnt_1++;
			if (exit_cnt_1 >= 1)
			{
				speed_base = yuanhuan_speed;
				huandao_flag = 1;
				exit_cnt_1 = 0;
				exit_cnt_2_1 = 0;
			}
			
			
		}
		else if (r_d_num == 0 && r_u_num != 0 && Continuity_Change_Left(MT9V03X_H-20, line + 10,line) == 0 && Continuity_Change_Right(MT9V03X_H, line + 2,line) != 0 && line < 10 && Left_Lost_Time <=MT9V03X_H/8 && Right_Lost_Time >= MT9V03X_H/3)
		{
			exit_cnt_2_1++;
			if (exit_cnt_2_1 >= 3)
			{
				speed_base = yuanhuan_speed;
				huandao_flag = 2;
				exit_cnt_1 = 0;
				exit_cnt_2_1 = 0;
			}
			
		}
		else
		{
			exit_cnt_1 = 0;
			exit_cnt_2_1 = 0;
		}
	}
	else if (huandao_flag == 1)
	{// && Continuity_Change_Left(MT9V03X_H-15, line + 10,line) == 0
		uint8 yuanhuan_qiedian = 0;
		
		yuanhuan_qiedian = find_min(road_right, r_u_num+5, 10);
		if (r_d_num == 0 && r_u_num != 0)
		{
			exit_cnt_2_2++;
			if (exit_cnt_2_2 >= 1)
			{
				speed_base = yuanhuan_speed;
				huandao_flag = 2;
				exit_cnt_2_2 = 0;
			}
			
			
		}
		else
		{
			exit_cnt_2_2 = 0;
		}
		if (yuanhuan_qiedian)
		{
			xieji(yuanhuan_qiedian, r_d_num, road_right[yuanhuan_qiedian], road_right[r_d_num], road_right);
            
		}
		
	}
	else if (huandao_flag == 2)
	{// && detect_left_arc(road_left,road_right,MT9V03X_H-3,MT9V03X_H-38)
		uint8 qiebuxian = 0;
		going_flag = 0;
		
		qiebuxian = find_min_min(road_right, MT9V03X_H-5, u_num - 5);
		
		if (r_d_num == 0 && r_u_num == 0)
		{
			exit_cnt_3++;
			if (exit_cnt_3 >= 1)
			{
				huandao_flag = 3;
				exit_cnt_3 = 0;
			}
			
		}
		else
		{
			exit_cnt_3 = 0;
		}
		
		if (qiebuxian > 0)
		{
			xieji(qiebuxian, MT9V03X_H-5,road_right[qiebuxian], MT9V03X_W-20, road_right);
		}
		if (u_num > 10 && r_d_num == 0 && abs(u_num - 45) <= 20)
		{
			xieji(u_num, MT9V03X_H-5, road_right[u_num], 20,road_left);
			xieji(5, u_num, MT9V03X_W-2, road_right[u_num],road_left);
			for (uint8 i = 10; i <= u_num; i++)
			{
				road_right[i]=MT9V03X_W-2;
			}
//			low_high_choose = 1;
		}
		
//		if (going_flag == 1 && Continuity_reduce_left(road_left))
//		{
//			huandao_flag = 2;
//		}
	}
	else if (huandao_flag == 3)
	{
		speed_base = yuanhuan_speed;
		uint8 left_new_point = find_max(road_left, MT9V03X_H-5, 20);
//		qianzhan_value = line + 10;
		if (Continuity_Change_Left(MT9V03X_H-10, line + 10,line) == 0 && Continuity_Change_Right(MT9V03X_H, line + 2,line) != 0 && u_num > 0)
		{
			exit_cnt_4++;
			if (exit_cnt_4 >= 1)
			{
				huandao_flag = 4;
				exit_cnt_4 = 0;
			}
			
			
		}
		else
		{
			exit_cnt_4 = 0;
		}
//		uint8 right_add_point = research_black_point();
		if (left_new_point > 0)
		{
			if (line > 10)
			{
				xieji(line-5, line, MT9V03X_W-5, MT9V03X_W-15,road_left);
			}
			xieji(line, left_new_point, MT9V03X_W-15, road_left[left_new_point],road_left);
			for (uint8 i = 10; i < MT9V03X_H-60; i++)
			{
				road_right[i] = MT9V03X_W-2;
			}
			going_flag = 1;
		}
		else if (left_new_point == 0 && going_flag == 1)
		{
			if (line > 10)
			{
				xieji(line-5, line, MT9V03X_W-15, MT9V03X_W-25,road_left);
			}
			xieji(line, MT9V03X_H-15, MT9V03X_W-25, 10, road_left);
			for (uint8 i = 10; i < MT9V03X_H-60; i++)
			{
				road_right[i] = MT9V03X_W-2;
			}
		}
		
	}
	else if (huandao_flag == 4)
	{
//		qianzhan_value = line + 10;
		left_test = Continuity_Change_Left(MT9V03X_H-10, line+25, line);
		right_test = Continuity_Change_Right(MT9V03X_H-10, line+25, line);// && l_d_num == 0 && r_d_num == 0
		if (left_test == 0 && right_test == 0)
		{
			exit_cnt_0++;
			if (exit_cnt_0 >=1)
			{
				huandao_flag = 0;
				yuanhuan_flag_4 = 0;
//				huandao_num_flag = 10;
				speed_base = straight_speed;
				exit_cnt_0 = 0;
			}
			
		}
		else
		{
			exit_cnt_0 = 0;
		}
		if (huandao_flag != 0 && u_num > 0)
		{
			xieji(u_num, MT9V03X_H-5, road_right[u_num], MT9V03X_W-road_left[MT9V03X_H-5],road_right);
			yuanhuan_flag_4 = 1;
		}
//		else if (huandao_flag != 0 && u_num == 0 && yuanhuan_flag_4 == 0)
//		{
//			xieji(40, MT9V03X_H-5, MT9V03X_W-30, MT9V03X_W-15,road_right);// || u_num > 140
//		}
		
	}
}


void yuanhuan_left_in_handle(void)
{
	float xielv,jieju;
	uint8 u_num = 0;
	
	u_num = Find_Left_Up_Point(MT9V03X_H-1,line,1);
	if (huandao_flag == 0 && huandao_num_flag == 0)
	{
		
		if (Continuity_Change_Left(MT9V03X_H, line + 2,line) != 0 && Continuity_Change_Right(MT9V03X_H-25, line + 2,line) == 0 && l_u_num > 0 && l_d_num > 0 && line < 10 && Right_Lost_Time <= MT9V03X_H/8 && Left_Lost_Time >= MT9V03X_H/3)
		{
			exit_cnt_1++;
			if (exit_cnt_1 >= 1)
			{
				huandao_flag = 1;
				speed_base = yuanhuan_speed;
				exit_cnt_1 = 0;
				exit_cnt_2_1 = 0;
			}
			
			
		}
		else if (l_d_num == 0 && l_u_num != 0 && Continuity_Change_Left(MT9V03X_H, line + 2,line) != 0 && Continuity_Change_Right(MT9V03X_H-20, line + 10,line) == 0 && line < 10 && Right_Lost_Time <=MT9V03X_H/8 && Left_Lost_Time >= MT9V03X_H/3)
		{
			exit_cnt_2_1++;
			if (exit_cnt_2_1 >= 3)
			{
				huandao_flag = 2;
				speed_base = yuanhuan_speed;
				exit_cnt_1 = 0;
				exit_cnt_2_1 = 0;
			}
			
		}
		else
		{
			exit_cnt_1 = 0;
			exit_cnt_2_1 = 0;
		}
	}
	else if (huandao_flag == 1)
	{// && Continuity_Change_Left(MT9V03X_H-15, line + 10,line) == 0
		uint8 yuanhuan_qiedian = 0;
		
		yuanhuan_qiedian = find_max(road_left, l_u_num+5, 10);
		if (l_d_num == 0 && l_u_num != 0)
		{
			exit_cnt_2_2++;
			if (exit_cnt_2_2 >= 1)
			{
				speed_base = yuanhuan_speed;
				huandao_flag = 2;
				exit_cnt_2_2 = 0;
			}
			
			
		}
		else
		{
			exit_cnt_2_2 = 0;
		}
		if (yuanhuan_qiedian)
		{
			xieji(yuanhuan_qiedian, l_d_num, road_left[yuanhuan_qiedian], road_left[l_d_num], road_left);
            
		}
		
	}
	else if (huandao_flag == 2)
	{// && detect_left_arc(road_left,road_right,MT9V03X_H-3,MT9V03X_H-38)
		uint8 qiebuxian = 0;
		going_flag = 0;
		
		qiebuxian = find_max_max(road_left, MT9V03X_H-5, u_num - 5);
		
		if (l_d_num == 0 && l_u_num == 0)
		{
			exit_cnt_3++;
			if (exit_cnt_3 >= 1)
			{
				huandao_flag = 3;
				exit_cnt_3 = 0;
			}
			
		}
		else
		{
			exit_cnt_3 = 0;
		}
		
		if (qiebuxian > 0)
		{
			xieji(qiebuxian, MT9V03X_H-5,road_left[qiebuxian], 20, road_left);
		}
		if (u_num > 10 && l_d_num == 0 && abs(u_num - 45) <= 20)
		{
			xieji(u_num, MT9V03X_H-5, road_left[u_num], MT9V03X_W-20,road_right);
			xieji(10, u_num, 2, road_left[u_num],road_right);
			for (uint8 i = 10; i <= u_num; i++)
			{
				road_left[i]=2;
			}
//			low_high_choose = 1;
		}
		
//		if (going_flag == 1 && Continuity_reduce_left(road_left))
//		{
//			huandao_flag = 2;
//		}
	}
	else if (huandao_flag == 3)
	{
		
		uint8 right_new_point = find_min(road_right, MT9V03X_H-5, 20);
		if (Continuity_Change_Left(MT9V03X_H, line + 2,line) != 0 && Continuity_Change_Right(MT9V03X_H-10, line + 10,line) == 0 && u_num > 0)
		{
			exit_cnt_4++;
			if (exit_cnt_4 >= 1)
			{
				huandao_flag = 4;
				exit_cnt_4 = 0;
			}
			
			
		}
		else
		{
			exit_cnt_4 = 0;
		}
//		uint8 right_add_point = research_black_point();
		if (right_new_point > 0)
		{
			xieji(line, right_new_point, 20, road_right[right_new_point],road_right);
			for (uint8 i = 20; i < MT9V03X_H-60; i++)
			{
				road_left[i] = 2;
			}
			going_flag = 1;
		}
		else if (right_new_point == 0 && going_flag == 1)
		{
			xieji(line, MT9V03X_H-20, 30, MT9V03X_W-5, road_right);
			for (uint8 i = 20; i < MT9V03X_H-60; i++)
			{
				road_left[i] = 2;
			}
		}
		
	}
	else if (huandao_flag == 4)
	{
		
		left_test = Continuity_Change_Left(MT9V03X_H-10, line+25, line);
		right_test = Continuity_Change_Right(MT9V03X_H-10, line+25, line);
		if (left_test == 0 && right_test == 0)
		{
			exit_cnt_0++;
			if (exit_cnt_0 >=1)
			{
				huandao_flag = 0;
				speed_base = straight_speed;
//				huandao_num_flag = 10;
				exit_cnt_0 = 0;
			}
		}
		else
		{
			exit_cnt_0 = 0;
		}
		if (huandao_flag != 0 && u_num > 0)
		{
			xieji(u_num, MT9V03X_H-5, road_left[u_num], MT9V03X_W-road_right[MT9V03X_H-5],road_left);// || u_num > 140
		}
	}
}

void smooth_boundary(uint8 arr[], uint8 start, uint8 end)
{
	if (start < end)
	{
		uint8 temp[MT9V03X_H] = {0};
		for (uint8 i = start; i < end; i++)
		{
			if (i == start) {
				
				temp[i] = (arr[i] + arr[i+1]) / 2;
			} else if (i == end - 1) {
				
				temp[i] = (arr[i-1] + arr[i]) / 2;
			} else {
				
				temp[i] = (arr[i-1] + arr[i] + arr[i+1]) / 3;
			}
		}
		
		for (uint8 i = start; i < end; i++) {
			arr[i] = temp[i];
		}
	}
    
}
//Detect_S_Curve(center_line, line);
uint8 Detect_S_Curve(uint8 *center, uint8 start)
{
	uint8 centerline[MT9V03X_H] = {0};
	memcpy(centerline, center, MT9V03X_H * sizeof(uint8_t));
    smooth_boundary(centerline, start+1, MT9V03X_H - 5);

    uint8 start_row = start+1;
    uint8 end_row = MT9V03X_H - 5;
    if (start_row >= end_row) return 0;

    float center_diff1[MT9V03X_H] = {0}; // 中线一阶差分
    float center_diff2[MT9V03X_H] = {0}; // 中线二阶差分
    uint8 center_sign_changes = 0;       // 中线符号变化次数

    for (uint8 i = start_row; i < end_row; i++)
    {
        if (i + 2 >= end_row) break;
        center_diff1[i] = centerline[i] - centerline[i+2]; // 中线i行 - i+2行
        if (i + 2 < end_row) {
            center_diff2[i] = center_diff1[i] - center_diff1[i+2];
            if (i > start_row + 2) {
                if ((center_diff2[i-2] > 0 && center_diff2[i] < 0) || 
                    (center_diff2[i-2] < 0 && center_diff2[i] > 0)) {
                    center_sign_changes++;
                }
            }
        }
    }

    int center_diff_sum = 0;
    for (uint8 i = start_row; i < end_row; i++) {
        center_diff_sum += abs(center_diff1[i]);
    }
    if (center_diff_sum < 15) {
        return 0; // 整体变化太小，不是S弯
    }
	
    uint8 is_s_curve = 0;
	if (center_sign_changes >= 2)
		return is_s_curve;
    else
		return 0;
}

uint8 find_color_width(uint8 num,int line)
{
	uint8 i = 5;
	uint8 start_num = 0;
	uint8 end_num = 0;
	uint8 black_num = 0;
	for (i = 10; i < MT9V03X_W-10; i++)
	{
		if (PostProcessing_image[qianzhan_value-line][i] == WHITE_PIXEL && PostProcessing_image[qianzhan_value-line][i+1] == BLACK_PIXEL && PostProcessing_image[qianzhan_value-line][i+2] == BLACK_PIXEL
			&& start_num == 0)
		{
			start_num = i;
		}
		else if (start_num != 0 && PostProcessing_image[qianzhan_value-line][i] == BLACK_PIXEL)
		{
			black_num++;
		}
		else if (PostProcessing_image[qianzhan_value-line][i] == BLACK_PIXEL && PostProcessing_image[qianzhan_value-line][i+1] == WHITE_PIXEL && PostProcessing_image[qianzhan_value-line][i+2] == WHITE_PIXEL
			&& start_num != 0 && black_num >= num)
		{
			return 1;
		}
	}
	return 0;
}

uint8 push_box_cnt0 = 0;
uint8 push_box_cnt1 = 0;
uint8 push_box_cnt2 = 0;
uint8 push_box_cnt3 = 0;
float now_distance = 0,last_distance = 0;
int now_cx = 0,last_cx = 0;

void color_binary_detection(void)
{
	if(blob_info.valid == 1 && blob_info.distance <= 100 && push_box_flag == 0)
	{
		if (blob_info.area > 1600)
		{
			push_box_flag = 1;
		}
	}
	else if (push_box_flag == 1)
	{//
		if (fabs(blob_info.distance - TARGET_DISTANCE) < 10 && abs(blob_info.cx - TARGET_X) < 10)
		{
			
			last_cx = now_cx;
			last_distance = now_distance;
			now_cx = blob_info.cx;
			now_distance = blob_info.distance;
			if (fabs(now_distance - last_distance) < 3 && abs(now_cx - last_cx) <= 3)
			{
				push_box_cnt0++;
				if (push_box_cnt0 > 10)
				{
					push_box_flag = 2;      //状态2说明车已经停稳
					now_rotate_angle = yaw_angle + 95.0f;
					push_box_cnt0 = 0;
				}
			}
			else
			{
				push_box_cnt0 = 0;
			}
		}
	}
	else if (push_box_flag == 2)
	{
		if (fabs(now_rotate_angle-yaw_angle) <= 5)
		{
			push_box_cnt1++;
			if (push_box_cnt1 > 10)
			{
				push_box_flag = 3;
				now_distance = 0;
				last_distance = 0;
				now_cx = 0;
				last_cx = 0;
				push_box_cnt1 = 0;
			}
		}
		else
		{
			push_box_cnt1 = 0;
		}
	}
	else if (push_box_flag == 3)
	{
		if (fabs(blob_info.distance - TARGET_DISTANCE) < 10 && abs(blob_info.cx - TARGET_X) < 10)
		{
			
			last_cx = now_cx;
			last_distance = now_distance;
			now_cx = blob_info.cx;
			now_distance = blob_info.distance;
			if (fabs(now_distance - last_distance) < 3 && abs(now_cx - last_cx) <= 3)
			{
				push_box_cnt2++;
				if (push_box_cnt2 > 10)
				{
					push_box_flag = 4;      //状态4说明车已经停稳
					push_box_cnt2 = 0;
				}
			}
			else
			{
				push_box_cnt2 = 0;
			}
		}
	}
	else if (push_box_flag == 6)
	{
		if (fabs(now_rotate_angle-yaw_angle) <= 3)
		{
			push_box_cnt3++;
			if (push_box_cnt3 > 10)
			{
				push_box_flag = 0;
				now_distance = 0;
				last_distance = 0;
				now_cx = 0;
				last_cx = 0;
				push_box_cnt3 = 0;
			}
		}
		else
		{
			push_box_cnt3 = 0;
		}
	}
}
