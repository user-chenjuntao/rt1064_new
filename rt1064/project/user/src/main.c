/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"


#define IPS200_TYPE     (IPS200_TYPE_SPI)                                 // 并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8
                                                                                // SPI 两寸屏 这里宏定义填写 IPS200_TYPE_SPI
#define PIT_PRIORITY_0    (PIT_IRQn)
#define PIT_PRIORITY_1    (PIT_IRQn)
#define PIT_PRIORITY_2    (PIT_IRQn)



// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库移植用空工程

int v1= 0;
int v2= 0;
int speed_base = 350;//122 245
float speed_k = 0.95;//0.45
int speed_limit = 110;//40
int servo_test = 0;
uint8_t get_data = 0;
int car_stop_array[4] = {0};
uint16 timer_box_cnt = 0;


//int car_xy[2]={5,1};
//int boxes_xy_test[8][2]={
//	{0,1},{0,2},{0,3},{1,0},{4,7},
//	{3,7},{6,2},{6,6}
//};

PlannerPointV3_BFS car = {1, 2};
PlannerPointV3_BFS boxes[] = {
	{5, 3},
    {6, 3},    
	{3, 3},
};
PlannerPointV3_BFS targets[] = {
    {9, 7},  
	{9, 6},
    {14, 2},  
};  

PlannerPointV3_BFS obstacles[] = {

	{0, 7}, {2, 7},
	{3, 7}, {4, 7}, {4, 6}, 
	{7, 5}, {8, 5}, {8, 6},
	{8, 7}, {8, 8}, {9, 8},
	{10, 8},{11, 8},{12, 8},
	{12, 7},{12, 6},{12, 5},
	{12, 4},{12, 3},{11, 3},
	{10, 3},{10, 6},{10, 5},
	{1, 9},{1,7}

};//

PlannerPointV3_Bomb bombs[] = {{3, 3}};
	//
//Point car = {5,1};
size_t steps;
int res;
PlannerPointV3_BFS path[GREEDY_AREA];
size_t used_bomb_count = 0;
size_t box_target_mapping[100];
PlannerChainInfo chain_info;
PlannerAllBoxPaths first_paths, final_paths;
size_t used_bombs[1];
PlannerBoxOverlap overlaps[PLANNER_V3_BFS_MAX_BOXES];

Point corner_path[50];   // 拐点缓冲区
size_t corner_steps = 0;   // 拐点数量

// 重构链状态信息（在assigned_box_planner_greedy_2.c中定义）
extern PlannerRebuildChainStatus g_rebuild_chain_status;

// 数组数量变量
size_t boxes_count = 0;
size_t targets_count = 0;
size_t obstacles_count = 0;
size_t bombs_count = 0;

// 读取数组数量的函数
void read_array_counts(void)
{
    boxes_count = sizeof(boxes) / sizeof(boxes[0]);
    targets_count = sizeof(targets) / sizeof(targets[0]);
    obstacles_count = sizeof(obstacles) / sizeof(obstacles[0]);
    bombs_count = sizeof(bombs) / sizeof(bombs[0]);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    debug_init();                   // 调试端口初始化

    // 读取数组数量
    read_array_counts();
    
    // 此处编写用户代码 例如外设初始化代码等
    system_delay_ms(300);           //等待主板其他外设上电完成
    uart_blob_init();
	flash_init();
	menu_init();
	uart_init(UART_2, 115200, UART2_TX_B18, UART2_RX_B19);
    ips200_show_string(0, 0, "mt9v03x init.");
/*    while(1)
    {
        if(mt9v03x_init())
            ips200_show_string(0, 16, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(500);                                                   // 短延时快速闪灯表示异常
    }*/
	key_init(20);
	motor_init();
	encoder_init();
	
	uint8 load_result = param_load_from_flash();
	PID_Init(&ULpid, &ULPidInitStruct);
	PID_Init(&URpid, &URPidInitStruct);
	PID_Init(&DLpid, &DLPidInitStruct);
	PID_Init(&DRpid, &DRPidInitStruct);
	PID_Init(&Yawpid, &YawPidInitStruct);
	PID_Init(&Camera_x_pid, &Camera_x_PidInitStruct);
	PID_Init(&Camera_y_pid, &Camera_y_PidInitStruct);
	PID_Init(&Gyro_rotate_pid, &Gyro_Rotate_PidInitStruct);
	Kinematics_Init();

	
	res = plan_boxes_greedy_v3_bfs(15, 11, car, boxes,boxes_count,targets,targets_count,obstacles, obstacles_count, path, GREEDY_AREA, &steps, box_target_mapping, &chain_info, &first_paths, &final_paths, overlaps);
	path_follow_init(0.40f, (float)pulse_per_meter);
	
	//res = plan_boxes_greedy_v3_bfs(15, 11, car, boxes, 3,targets,3,obstacles, 25, path, GREEDY_AREA, &steps, box_target_mapping, &chain_info, &first_paths, &final_paths, overlaps);
	if (res == 0)
	{
		ips200_show_string(0, 16, "path init.");
	}
	else
	{
		ips200_show_string(0, 16, "path reinit.");
	}
	corner_steps = path_follow_extract_corners(path, steps, corner_path, 50);

	if (corner_steps > 0)
	{
		// 使用提取的拐点设置路径
		path_follow_set_path(corner_path, corner_steps);
	}

	imu963ra_init();
	Attitude_Init();
	pit_ms_init(PIT_CH0, 20);                                                  // 初始化 PIT_CH0 为周期中断 20ms 周期
    interrupt_set_priority(PIT_PRIORITY_0, 2);                                    // 设置 PIT0 对周期中断的中断优先级为 2
	pit_ms_init(PIT_CH1, 10);                                                  // 初始化 PIT_CH1 为周期中断 10ms 周期
    interrupt_set_priority(PIT_PRIORITY_1, 1);                                    // 设置 PIT1 对周期中断的中断优先级为 1
	pit_ms_init(PIT_CH2, 2);                                                  // 初始化 PIT_CH2 为周期中断 2ms 周期
    interrupt_set_priority(PIT_PRIORITY_2, 0);                                    // 设置 PIT2 对周期中断的中断优先级为 0
    interrupt_set_priority(LPUART1_IRQn, 3);  // 设置中断优先级（中等）
    interrupt_global_enable(0);
	ips200_show_string(0, 16, "init success.");
//	system_delay_ms(1000);
    // 此处编写用户代码 例如外设初始化代码等
    while(1)
    {
		process_blob_data();
		color_distance_handle();
        // 此处编写需要循环执行的代码
		/*
		if(mt9v03x_finish_flag)
		{
			image_process();
			mt9v03x_finish_flag=0;
		}
		*/
		
		
		
        menu_switch();
		
//		if (car_go_flag == 0)
//		{
			menu_display();
//		}
		image_data_clear();
        // 此处编写需要循环执行的代码
    }
}

void pit_0_handler (void)
{
    key_scanner();                                                              // 周期中断触发 标志位置位
}

void pit_1_handler (void)
{
//    encoder_read_filtered(&encoder_data_1, &encoder_data_2, &encoder_data_3, &encoder_data_4);
	encoder_get();
    distance_speed_strategy();//计算出的四个速度从0到3，分别是上左，上右，下左，下右
	// speed_encoder[0]=25;
    if (car_go_flag == 1 && car_stop_flag == 0)
    {//修改位置在这里
        motor_control(speed_encoder);
		// motor_pwm(straight_speed,0,0,0);
    }
    else if (car_go_flag == 1 && car_stop_flag == 1)
    {
        motor_control(car_stop_array);
    }
    else
    {
        motor_pwm(0,0,0,0);
    }
    
}

void pit_2_handler (void)
{
	//ICM_hubu();
	Attitude_Calculate();
	

	                                                                 
}


void uart_rx_interrupt_handler(void) {
    
    // 查询式读取1字节（无数据则直接退出，确保中断快速执行）
    if (uart_query_byte(UART_INDEX, &get_data)) {
        // 用fifo_write_buffer写入1字节（length=1，地址为get_data的地址）
        fifo_write_buffer(&uart_data_fifo, &get_data, 1);
    }
}