#ifndef __DATA_HANDLE_H_
#define __DATA_HANDLE_H_

#include "zf_common_typedef.h"
#include "zf_common_fifo.h"
#include "zf_driver_uart.h"
#include "assigned_box_planner_greedy_2.h"

// 1. 串口配置（与调试端口分离，使用独立UART）
#define UART_INDEX              UART_1                                          // 使用UART5接收OpenART数据
#define UART_BAUDRATE           115200                                          // 与OpenART波特率一致
#define UART_TX_PIN             UART1_TX_B12                                     // 发送引脚（可选，用于调试回传）
#define UART_RX_PIN             UART1_RX_B13                                     // 接收引脚（接OpenART的TX）

//#define UART_PRIORITY           LPUART1_IRQn                                     // UART5对应的中断号（MIMXRT1064.h中定义）

// 2. FIFO与缓冲区配置
#define FIFO_SIZE               256                                               // FIFO缓冲区大小

#define FIFO_READ_BUF_SIZE      256                                               // FIFO批量读取缓冲区大小

#define FRAME_BUF_SIZE          256                                               // 单帧数据缓冲区大小

#define MID_CAMERA_WIDTH        160
#define MID_CAMERA_HEIGHT       120
#define CAMERA_WIDTH        320
#define CAMERA_HEIGHT       240

#define MAX_SELECT_SIZE     1
#define INIT_MAP_MIN_OBSTACLES 8
#define INIT_MAP_MIN_BOXES 3
#define INIT_MAP_MIN_TARGETS 3

// 3. 色块数据结构（解析后的数据）
typedef struct {
    int16_t cx, cy;  // 中心坐标
    int16_t w, h;    // 宽高
    int32_t area;    // 面积
    uint8_t valid;   // 数据有效性（1=有效，0=无效）
	int w_error;
	float distance;
} BlobInfo;

// 小车位置结构体（使用float类型以提高精度）
typedef struct {
    float row;  // 行坐标
    float col;  // 列坐标
} CarPosition;

// 数组最大容量定义（与main.c中的数组大小一致）
#define MAX_OBSTACLES 50
#define MAX_BOXES 10
#define MAX_TARGETS 10
#define MAX_CAR_PATH 250

// 像素到网格的转换比例（每9个像素为1格）
#define PIXEL_TO_GRID_RATIO 1

// 地图数据接收状态
typedef enum {
    MAP_STATE_INIT,      // 初始地图阶段（最多接收12次完整数据）
    MAP_STATE_DYNAMIC    // 动态地图阶段（只接收小车和箱子位置）
} map_state_t;

void uart_blob_init(void);
void parse_blob_frame(void);
void process_blob_data(void);

// 像素坐标转网格坐标（每9个像素为1格，返回float类型以提高精度）
void pixel_to_grid(int pixel_row, int pixel_col, float* grid_row, float* grid_col);

// 地图状态标志（供外部使用）
extern map_state_t map_state;
extern bool initial_map_ready;  // 初始地图是否已就绪
extern bool uart_data_processing_enabled;  // 串口数据处理启用标志

// 实际接收到的数据计数（供外部使用）
extern size_t actual_obstacles_count;
extern size_t actual_boxes_count;
extern size_t actual_targets_count;
extern size_t actual_car_path_count;

extern PlannerPointV3_BFS obstacles[MAX_OBSTACLES];                                               // 障碍物数组（在main.c中定义）
extern PlannerPointV3_BFS boxes[MAX_BOXES];                                                   // 箱子数组（在main.c中定义）
extern PlannerPointV3_BFS targets[MAX_TARGETS];                                                 // 目标点数组（在main.c中定义）
extern PlannerPointV3_BFS cat_turth_path[MAX_CAR_PATH];
extern PlannerPointV3_BFS car;  // 车的位置（初始地图阶段使用，在main.c中定义）
extern CarPosition car_position;  // 小车位置（动态地图阶段使用，float类型）

// 数据接收完成标志（所有格式的数据都已接收）
extern bool data_reception_complete;

extern uint8_t get_data_1;
extern uint8_t get_data_2;
extern uint8_t get_data_3;
extern uint8_t get_data_4;

extern fifo_struct uart_data_fifo;
extern BlobInfo blob_info;
extern uint32 format_count;


#endif
