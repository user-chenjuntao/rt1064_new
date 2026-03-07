/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ?(RT1064DVL6A ???)??????? SDK ?????????
* Copyright (c) 2022 SEEKFREE ????
* 
* ???? RT1064DVL6A ???????
* 
* RT1064DVL6A ??? ?????
* ??????????????? GPL(GNU General Public License,? GNU???????)???
* ? GPL ??3?(? GPL3.0)?(????)???????,?????/????
* 
* ????????????????,???????????
* ????????????????????
* ??????? GPL
* 
* ????????????????? GPL ???
* ????,???<https://www.gnu.org/licenses/>
* 
* ????:
* ?????? GPL3.0 ??????? ???????????
* ???????? libraries/doc ????? GPL3_permission_statement.txt ???
* ?????? libraries ???? ??????? LICENSE ??
* ???????????? ???????????????????(????)
* 
* ????          main
* ????          ??????????
* ????          ?? libraries/doc ???? version ?? ????
* ????          IAR 8.32.4 or MDK 5.33
* ????          RT1064DVL6A
* ????          https://seekfree.taobao.com/
* 
* ????
* ??              ??                ??
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "assigned_box_planner_greedy_3.h"


#define IPS200_TYPE     (IPS200_TYPE_SPI)                                 // ????? ??????? IPS200_TYPE_PARALLEL8
                                                                                // SPI ??? ??????? IPS200_TYPE_SPI
#define PIT_PRIORITY_0    (PIT_IRQn)
#define PIT_PRIORITY_1    (PIT_IRQn)
#define PIT_PRIORITY_2    (PIT_IRQn)



// ???????????????????????
// ??? ???????????
// ??? project->clean  ?????????

// ?????????????
uint16_t t=0;
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
/*PlannerPointV3_BFS car = {1, 2};
PlannerPointV3_BFS boxes[] = {
    {1, 7},
    {7, 4},
    {10, 4},
};
PlannerPointV3_BFS targets[] = {
    {9, 5},
    {9, 6},
    {9, 7},
};
PlannerPointV3_BFS obstacles[] = {
    {5, 1},  {6, 1},  {7, 1},  {8, 1},
    {0, 7},  {2, 7},  {3, 7},  {4, 7},
    {4, 6},  {4, 5},  {5, 5},  {6, 5},
    {7, 5},  {8, 5},  {8, 6},  {8, 7},
    {8, 8},  {9, 8},  {10, 8}, {11, 8},
    {12, 8}, {12, 7}, {12, 6}, {12, 5},
    {12, 4}, {12, 3}, {11, 3}, {10, 3},
    {10, 5}, {10, 6}, {1, 9},
};*/
PlannerPointV3_BFS car = {6,0};

PlannerPointV3_BFS boxes[] = {
    {7,2}, {7,10}, {8,1}
};
PlannerPointV3_BFS targets[] = {
    {0,13}, {3,1}, {9,5}
};  // ?????????????

PlannerPointV3_BFS obstacles[] = {

    {0,1}, {0,5}, {0,8}, {1,3}, {1,6}, {1,8}, {1,9}, {1,10}, {1,11}, {2,0}, {2,2}, {2,3}, {2,4}, {2,6}, {2,8}, {3,2}, {3,4}, {3,6}, {3,8}, {3,11}, {4,1}, {4,2}, {4,4}, {5,2}, {5,10}, {5,11}, {6,11}, {7,4}, {8,4}, {8,5}, {8,6}, {8,7}, {8,8}, {8,9}, {8,10}, {8,11}, {9,4}


};//
PlannerPointV3_BFS bombs[] = {
	{6,1}, {6,9}, {7,11}

};
	//
//Point car = {5,1};
size_t steps;
int res=100;
PlannerPointV3_BFS path[GREEDY_AREA];
size_t used_bomb_count = 0;
size_t box_target_mapping[100];
//PlannerChainInfo chain_info;
PlannerAllBoxPaths first_paths, final_paths;
//PlannerBoxOverlap overlaps[PLANNER_V3_BFS_MAX_BOXES];

Point corner_path[50];   // ?????
size_t corner_steps = 0;   // ????

// ???????(?assigned_box_planner_greedy_2.c???)
//extern PlannerRebuildChainStatus g_rebuild_chain_status;

// ??????
size_t boxes_count = 0;
size_t targets_count = 0;
size_t obstacles_count = 0;
size_t bombs_count = 0;

// ?????????
void read_array_counts(void)
{
    boxes_count = sizeof(boxes) / sizeof(boxes[0]);
    targets_count = sizeof(targets) / sizeof(targets[0]);
    obstacles_count = sizeof(obstacles) / sizeof(obstacles[0]);
    bombs_count = sizeof(bombs) / sizeof(bombs[0]);
}

// 生成 size_t 数组的下一个排列（字典序），返回 1 表示有下一排列，0 表示已是最后一个
static int next_permutation(size_t *arr, size_t n)
{
    if (n <= 1) return 0;
    size_t i = n - 1;
    while (i > 0 && arr[i - 1] >= arr[i]) i--;
    if (i == 0) return 0;
    size_t j = n - 1;
    while (arr[j] <= arr[i - 1]) j--;
    size_t t = arr[i - 1];
    arr[i - 1] = arr[j];
    arr[j] = t;
    for (size_t k = i, e = n - 1; k < e; k++, e--) {
        t = arr[k];
        arr[k] = arr[e];
        arr[e] = t;
    }
    return 1;
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // ????
    debug_init();                   // ???????

    // ??????
    read_array_counts();
    
    // ???????? ??????????
    system_delay_ms(300);           //????????????
	flash_init();
	menu_init();
//	uart_init(UART_2, 115200, UART2_TX_B18, UART2_RX_B19);
    ips200_show_string(0, 0, "mt9v03x init.");
/*    while(1)
    {
        if(mt9v03x_init())
            ips200_show_string(0, 16, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(500);                                                   // ???????????
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
	imu963ra_init();
	Attitude_Init();
	pit_ms_init(PIT_CH0, 20);                                                  // ??? PIT_CH0 ????? 20ms ??
    interrupt_set_priority(PIT_PRIORITY_0, 2);                                    // ?? PIT0 ???????????? 2
	pit_ms_init(PIT_CH1, 10);                                                  // ??? PIT_CH1 ????? 10ms ??
    interrupt_set_priority(PIT_PRIORITY_1, 1);                                    // ?? PIT1 ???????????? 1
	pit_ms_init(PIT_CH2, 1);                                                  // ??? PIT_CH2 ????? 2ms ??
    interrupt_set_priority(PIT_PRIORITY_2, 0);                                    // ?? PIT2 ???????????? 0
    interrupt_set_priority(LPUART1_IRQn, 3);  // ???????(??)
    interrupt_global_enable(0);

	// 对 manual 的排列全部尝试，取 steps 最小的成功结果
	size_t manual[boxes_count];  // 至少容纳 boxes_count，此处取 8
	size_t best_manual[boxes_count];
	size_t best_steps = (size_t)-1;   // 未成功时保持最大值
	int best_res = -1;
	PlannerPointV3_BFS best_path[GREEDY_AREA];
	size_t best_box_target_mapping[100];
	static PlannerAllBoxPaths best_final_paths;
	static PlannerAllBoxPaths best_special_paths;
	for (size_t i = 0; i < boxes_count; i++)
		manual[i] = i;
	do {
		res = plan_boxes_greedy_v3_manual_assignment(
							10, 14, car,
	                        boxes, boxes_count,
	                        targets, targets_count,
	                        bombs, bombs_count,
	                        obstacles, obstacles_count,
							manual,
	                        path, GREEDY_AREA, &steps,
	                        box_target_mapping, &final_paths);
		if (res == 0 && steps < best_steps) {
			best_steps = steps;
			memcpy(best_manual, manual, sizeof(manual));
			best_res = 0;
			for (size_t k = 0; k < GREEDY_AREA; k++) best_path[k] = path[k];
			for (size_t k = 0; k < boxes_count; k++) best_box_target_mapping[k] = box_target_mapping[k];
			best_final_paths = final_paths;
		}
	} while (next_permutation(manual, boxes_count));
	// 若至少有一次成功，采用步数最少的那次
	if (best_res == 0) {
		res = best_res;
		steps = best_steps;
		for (size_t k = 0; k < GREEDY_AREA; k++) path[k] = best_path[k];
		for (size_t k = 0; k < boxes_count; k++) box_target_mapping[k] = best_box_target_mapping[k];
		final_paths = best_final_paths;
		/*res = plan_boxes_greedy_v3_manual_assignment(
							10, 14, car,
	                        boxes, boxes_count,
	                        targets, targets_count,
	                        bombs, bombs_count,
	                        obstacles, obstacles_count,
							best_manual,
	                        path, GREEDY_AREA, &steps,
	                        box_target_mapping, &final_paths);*/
	}
	/*manual[0] = 2; manual[1] = 0; manual[2] = 1;  // 固定分配，测试用
	res = plan_boxes_greedy_v3_manual_assignment(
							10, 14, car,
	                        boxes, boxes_count,
	                        targets, targets_count,
	                        bombs, bombs_count,
	                        obstacles, obstacles_count,
							manual,
	                        path, GREEDY_AREA, &steps,
	                        box_target_mapping, &final_paths);*/
// ret == 0 成功；ret == -11 表示分配无效（目标越界或重复）
	// ????????????? v3
	/*res = plan_boxes_greedy_v3(10, 14, car,
	                        boxes, boxes_count,
	                        targets, targets_count,
	                       	bombs, bombs_count,
	                        obstacles, obstacles_count,
	                        path, GREEDY_AREA, &steps,
	                        box_target_mapping, &final_paths);*/

	//res = plan_boxes_greedy_v3_bfs(14, 10, car, boxes, boxes_count,targets,targets_count,obstacles, obstacles_count, path, GREEDY_AREA, &steps, box_target_mapping, &final_paths);

	path_follow_init(0.40f, (float)pulse_per_meter);
	
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
		// ???????????
		path_follow_set_path(corner_path, corner_steps);
	}

	
	ips200_show_string(0, 32, "init success.");
//	system_delay_ms(1000);
    // ???????? ??????????
    while(1)
    {
		ips200_show_uint(0, 100, t , 5);
		color_distance_handle();
        // ?????????????
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
        // ?????????????
    }
}

void pit_0_handler (void)
{
    key_scanner();                                                              // ?????? ?????
}

void pit_1_handler (void)
{
//    encoder_read_filtered(&encoder_data_1, &encoder_data_2, &encoder_data_3, &encoder_data_4);
	encoder_get();
    distance_speed_strategy();//?????????0?3,?????,??,??,??
	// speed_encoder[0]=25;
    if (car_go_flag == 1 && car_stop_flag == 0)
    {//???????
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
	//Attitude_Calculate();
	if(res!=0){

		t+=1;

	}

	                                                                 
}
