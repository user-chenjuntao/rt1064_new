#include "data_handle.h"
#include <string.h>
#include <stdio.h>
#include <math.h>



uint32 format_count = 0;                                                      // 格式计数器
uint8_t uart_fifo_buf[FIFO_SIZE];                                                // FIFO挂载的缓冲区
fifo_struct uart_data_fifo;                                                      // FIFO对象
uint8_t fifo_read_buf[FIFO_READ_BUF_SIZE] = {0};                                 // 存储FIFO批量读取的数据
uint8_t frame_buf[FRAME_BUF_SIZE] = {0};                                         // 存储完整帧数据
uint16_t frame_idx = 0;                                                          // 帧缓冲区索引
volatile bool is_frame_complete = false;                                         // 帧完整标志
volatile bool data_format_change = false;                                         // 帧完整标志
static uint8 data_format_flag = 0;                                               // 数据格式标志（备用）
static uint8 data_format_flag_remember = 0;                                         // 上次数据格式标志（备用）
PlannerPointV3_BFS obstacles[MAX_OBSTACLES] = {{0}};                                               // 障碍物数组（在main.c中定义）
PlannerPointV3_BFS boxes[MAX_BOXES] = {{0}};                                                   // 箱子数组（在main.c中定义）
PlannerPointV3_BFS targets[MAX_TARGETS] = {{0}};                                                 // 目标点数组（在main.c中定义）
PlannerPointV3_BFS cat_turth_path[MAX_CAR_PATH] = {{0}};
PlannerPointV3_BFS car = {1, 2};  // 车的位置（初始地图阶段使用，在main.c中定义）
CarPosition car_position = {0.0f, 0.0f};  // 小车位置（动态地图阶段使用，float类型）

// 实际接收到的数据计数
size_t actual_obstacles_count = 0;
size_t actual_boxes_count = 0;
size_t actual_targets_count = 0;
size_t actual_car_path_count = 0;

// 数据接收完成标志（需要接收：车位置、障碍物、箱子、目标，共4种格式）
bool data_reception_complete = false;

BlobInfo blob_info = {0};                                                         // 初始化为无效

// 地图状态管理
map_state_t map_state = MAP_STATE_INIT;                                          // 初始状态为初始地图阶段
bool initial_map_ready = false;                                                  // 初始地图是否已就绪
bool uart_data_processing_enabled = false;                                       // 串口数据处理启用标志（默认关闭）

// 初始地图阶段：临时存储12次完整数据
typedef struct {
    PlannerPointV3_BFS obstacles[MAX_OBSTACLES];
    PlannerPointV3_BFS boxes[MAX_BOXES];
    PlannerPointV3_BFS targets[MAX_TARGETS];
    PlannerPointV3_BFS car;
    size_t obstacles_count;
    size_t boxes_count;
    size_t targets_count;
    size_t total_count;  // 总数据量（用于选择最完整的数据）
} init_map_data_t;

static init_map_data_t init_map_buffer[MAX_SELECT_SIZE] = {{{0}}};  // 存储12次完整数据
static uint8_t init_map_received_count = 0;          // 已接收的完整地图次数（0-12）
static bool current_round_complete = false; // Round complete when car frame arrives.

// 当前正在接收的一轮数据的临时存储（初始地图阶段使用）
static PlannerPointV3_BFS temp_obstacles[MAX_OBSTACLES] = {{0}};
static PlannerPointV3_BFS temp_boxes[MAX_BOXES] = {{0}};
static PlannerPointV3_BFS temp_targets[MAX_TARGETS] = {{0}};
static PlannerPointV3_BFS temp_car = {0, 0};
static size_t temp_obstacles_count = 0;
static size_t temp_boxes_count = 0;
static size_t temp_targets_count = 0;
static bool temp_car_received = false;



// 重置临时数据缓冲区（用于初始地图阶段的每一轮）
static void reset_temp_data(void) {
    memset(temp_obstacles, 0, sizeof(temp_obstacles));
    memset(temp_boxes, 0, sizeof(temp_boxes));
    memset(temp_targets, 0, sizeof(temp_targets));
    temp_car.row = 0;
    temp_car.col = 0;
    temp_obstacles_count = 0;
    temp_boxes_count = 0;
    temp_targets_count = 0;
    temp_car_received = false;
}

// 保存当前临时数据到初始地图缓冲区
static void save_temp_data_to_buffer(uint8_t buffer_idx) {
    if (buffer_idx >= MAX_SELECT_SIZE) return;
    
    // 复制数据
    memcpy(init_map_buffer[buffer_idx].obstacles, temp_obstacles, sizeof(temp_obstacles));
    memcpy(init_map_buffer[buffer_idx].boxes, temp_boxes, sizeof(temp_boxes));
    memcpy(init_map_buffer[buffer_idx].targets, temp_targets, sizeof(temp_targets));
    init_map_buffer[buffer_idx].car = temp_car;
    init_map_buffer[buffer_idx].obstacles_count = temp_obstacles_count;
    init_map_buffer[buffer_idx].boxes_count = temp_boxes_count;
    init_map_buffer[buffer_idx].targets_count = temp_targets_count;
    init_map_buffer[buffer_idx].total_count = temp_obstacles_count + temp_boxes_count + temp_targets_count + (temp_car_received ? 1 : 0);
}

// 从初始地图缓冲区中选择满足最低数量的最完整数据并应用到全局变量
static bool init_map_round_meets_min(const init_map_data_t *data) {
    return data->obstacles_count >= INIT_MAP_MIN_OBSTACLES
        && data->boxes_count >= INIT_MAP_MIN_BOXES
        && data->targets_count >= INIT_MAP_MIN_TARGETS;
}

static void select_and_apply_best_init_map(void) {
    uint8_t best_idx = 0;
    size_t max_count = 0;
    bool found_valid = false;

    for (uint8_t i = 0; i < init_map_received_count; i++) {
        if (init_map_round_meets_min(&init_map_buffer[i])) {
            if (!found_valid || init_map_buffer[i].total_count > max_count) {
                best_idx = i;
                max_count = init_map_buffer[i].total_count;
                found_valid = true;
            }
        }
    }

    if (!found_valid) {
        for (uint8_t i = 0; i < init_map_received_count; i++) {
            if (init_map_buffer[i].total_count > max_count) {
                best_idx = i;
                max_count = init_map_buffer[i].total_count;
            }
        }
    }

    memcpy(obstacles, init_map_buffer[best_idx].obstacles, sizeof(obstacles));
    memcpy(boxes, init_map_buffer[best_idx].boxes, sizeof(boxes));
    memcpy(targets, init_map_buffer[best_idx].targets, sizeof(targets));
    car = init_map_buffer[best_idx].car;
    actual_obstacles_count = init_map_buffer[best_idx].obstacles_count;
    actual_boxes_count = init_map_buffer[best_idx].boxes_count;
    actual_targets_count = init_map_buffer[best_idx].targets_count;

    initial_map_ready = true;
    map_state = MAP_STATE_DYNAMIC;
}

//串口与FIFO初始化
void uart_blob_init(void) {
    // 初始化FIFO（8位数据，挂载缓冲区）
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_fifo_buf, FIFO_SIZE);
    fifo_clear(&uart_data_fifo);  // 清空FIFO

    // 初始化串口
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);  // 开启接收中断
    
    // 初始化地图状态
    map_state = MAP_STATE_INIT;
    initial_map_ready = false;
    init_map_received_count = 0;
    current_round_complete = false;
    reset_temp_data();
    data_format_flag = 0;
    data_format_flag_remember = 0;
    format_count = 0;
}

//传入BlobInfo结构体指针，将其所有成员清0
void clearBlobInfo(BlobInfo* blob) {
    // 空指针判断，避免访问非法内存
    if (blob == NULL) {
        return;
    }
    
    // 结构体各成员清0
    blob->cx = 0;       // 中心X坐标清0
    blob->cy = 0;       // 中心Y坐标清0
    blob->w = 0;        // 宽度清0
    blob->h = 0;        // 高度清0
    blob->area = 0;     // 面积清0
    blob->valid = 0;    // 有效性标记清0（0表示无效）
	blob->w_error = 0;
	blob->distance = 0;
}

// 每种数据格式的索引计数器
static size_t parse_car_path_i = 0;
static size_t parse_obstacles_i = 0;
static size_t parse_boxes_i = 0;
static size_t parse_targets_i = 0;
uint8_t get_data_1 = 0;
uint8_t get_data_2 = 0;
uint8_t get_data_3 = 0;
uint8_t get_data_4 = 0; 

// 像素坐标转网格坐标（每9个像素为1格，使用float类型以提高精度）
void pixel_to_grid(int pixel_row, int pixel_col, float* grid_row, float* grid_col) {
    if (grid_row != NULL && grid_col != NULL) {
        *grid_row = (float)pixel_row / (float)PIXEL_TO_GRID_RATIO;
        *grid_col = (float)pixel_col / (float)PIXEL_TO_GRID_RATIO;
    }
}

//#是障碍，!是箱子，*是车，@目标
//数据解析函数（从完整帧中提取位置信息）
void parse_blob_frame(void) {
    int ret = 0;
    int pixel_row = 0, pixel_col = 0;  // 临时变量用于解析像素坐标
    float grid_row = 0.0f, grid_col = 0.0f;    // 转换后的网格坐标（使用float以提高精度）

    // if (map_state == MAP_STATE_INIT) {
        // 初始地图阶段：存储到临时缓冲区
        if (data_format_flag == 1) {  // 车位置 '*'
            ret = sscanf((char*)frame_buf, "*%d,%d\r\n", &pixel_row, &pixel_col);
            if (ret == 2 && !temp_car_received) {
                pixel_to_grid(pixel_row, pixel_col, &grid_row, &grid_col);
                get_data_1 = 1;
                temp_car.row = (int)roundf(grid_row);  // 四舍五入
                temp_car.col = (int)roundf(grid_col);  // 四舍五入
                temp_car_received = true;
                current_round_complete = true; // Round ends on car frame.
            }
        }
        else if (data_format_flag == 2) {  // 障碍物 '#'
            ret = sscanf((char*)frame_buf, "#%d,%d\r\n", &pixel_row, &pixel_col);
            if (ret == 2 && temp_obstacles_count < MAX_OBSTACLES) {
                pixel_to_grid(pixel_row, pixel_col, &grid_row, &grid_col);
                get_data_2 = 2;
                temp_obstacles[temp_obstacles_count].row = (int)roundf(grid_row);  // 四舍五入
                temp_obstacles[temp_obstacles_count].col = (int)roundf(grid_col);  // 四舍五入
                temp_obstacles_count++;
            }
        }
        else if (data_format_flag == 3) {  // 箱子 '!'
            ret = sscanf((char*)frame_buf, "!%d,%d\r\n", &pixel_row, &pixel_col);
            if (ret == 2 && temp_boxes_count < MAX_BOXES) {
                pixel_to_grid(pixel_row, pixel_col, &grid_row, &grid_col);
                get_data_3 = 3;
                temp_boxes[temp_boxes_count].row = (int)roundf(grid_row);  // 四舍五入
                temp_boxes[temp_boxes_count].col = (int)roundf(grid_col);  // 四舍五入
                temp_boxes_count++;
            }
        }
        else if (data_format_flag == 4) {  // 目标 '@'
            ret = sscanf((char*)frame_buf, "@%d,%d\r\n", &pixel_row, &pixel_col);
            if (ret == 2 && temp_targets_count < MAX_TARGETS) {
                pixel_to_grid(pixel_row, pixel_col, &grid_row, &grid_col);
                get_data_4 = 4;
                temp_targets[temp_targets_count].row = (int)roundf(grid_row);  // 四舍五入
                temp_targets[temp_targets_count].col = (int)roundf(grid_col);  // 四舍五入
                temp_targets_count++;
            }
        }
    // }
    // else {  // MAP_STATE_DYNAMIC - 动态地图阶段
    //     // 只处理小车和箱子位置
    //     if (data_format_flag == 1) {  // 车位置 '*'
    //         ret = sscanf((char*)frame_buf, "*%d,%d\r\n", &pixel_row, &pixel_col);
    //         if (ret == 2) {
    //             pixel_to_grid(pixel_row, pixel_col, &grid_row, &grid_col);
    //             get_data_2 = 22;
    //             car_position.row = grid_row;  // 直接存储float类型，保留小数精度
    //             car_position.col = grid_col;  // 直接存储float类型，保留小数精度
    //         }
    //     }
    //     else if (data_format_flag == 3) {  // 箱子 '!'
    //         ret = sscanf((char*)frame_buf, "!%d,%d\r\n", &pixel_row, &pixel_col);
    //         if (ret == 2 && actual_boxes_count < MAX_BOXES) {
    //             pixel_to_grid(pixel_row, pixel_col, &grid_row, &grid_col);
    //             get_data_3 = 33;
    //             boxes[actual_boxes_count].row = (int)roundf(grid_row);  // 四舍五入
    //             boxes[actual_boxes_count].col = (int)roundf(grid_col);  // 四舍五入
    //             actual_boxes_count++;
    //         }
    //     }
    //     // 在动态阶段，忽略障碍物和目标（它们不会改变）
    // }

    // 清空帧缓冲区，准备下一帧
    memset(frame_buf, 0, FRAME_BUF_SIZE);
    frame_idx = 0;
    is_frame_complete = false;
}
//#是障碍，!是箱子，*是车，@目标
//主循环数据处理（用fifo_read_buffer批量读取FIFO，提取完整帧）
void process_blob_data(void) {
    // Skip if UART data processing is disabled.
    if (!uart_data_processing_enabled) {
        return;
    }
    
    uint32_t read_len = 0;
    fifo_state_enum read_state;
    
    // Step 1: read all available FIFO data in one batch.
    read_len = fifo_used(&uart_data_fifo);  // Max read length (capped by buffer size).
    if (read_len != 0) {
        read_state = fifo_read_buffer(&uart_data_fifo, fifo_read_buf, &read_len, FIFO_READ_AND_CLEAN);
        // Step 2: parse all complete frames from the buffer.
        if (read_state == FIFO_SUCCESS && read_len > 0) {
            for (uint32_t i = 0; i < read_len; i++) {
                uint8_t data = fifo_read_buf[i];

                if (frame_idx >= FRAME_BUF_SIZE - 1) {
                    frame_idx = 0;
                    is_frame_complete = false;
                }

                // Drop data until a valid frame header is found.
                if (frame_idx == 0 && (data != '*' && data != '#' && data != '!' && data != '@')) {
                    continue;
                }

                // Detect data format (frame header).
                if (data == '*') {
                    data_format_flag = 1;
                }
                else if (data == '#') {
                    data_format_flag = 2;
                }
                else if (data == '!') {
                    data_format_flag = 3;
                }
                else if (data == '@') {
                    data_format_flag = 4;
                }

                // Detect format change.
                if (data_format_flag != data_format_flag_remember) {
                    data_format_change = true;
                    data_format_flag_remember = data_format_flag;
                } else {
                    data_format_change = false;
                }

                // Store data into the frame buffer.
                frame_buf[frame_idx++] = data;

                // Find frame tail '\r\n' (last 2 bytes).
                if (frame_idx >= 2 
                    && frame_buf[frame_idx-2] == '\r' 
                    && frame_buf[frame_idx-1] == '\n') {
                    is_frame_complete = true;  // Mark frame complete.
                    parse_blob_frame();
                }
            }
        }
    }

    if (current_round_complete) {
        if (init_map_received_count < MAX_SELECT_SIZE) {
            save_temp_data_to_buffer(init_map_received_count);
            init_map_received_count++; 
        }
        bool round_meets_min = (temp_obstacles_count >= INIT_MAP_MIN_OBSTACLES
            && temp_boxes_count >= INIT_MAP_MIN_BOXES
            && temp_targets_count >= INIT_MAP_MIN_TARGETS);

        if (round_meets_min || init_map_received_count >= MAX_SELECT_SIZE) {
            select_and_apply_best_init_map();
            data_reception_complete = true;
            reset_temp_data();
        } else {
            reset_temp_data();
        }
        current_round_complete = false;
    }
}
//  else {
//         }

//         current_round_complete = true;
//     }

//     if (current_round_complete) {
//         if (init_map_received_count < MAX_SELECT_SIZE) {
//             save_temp_data_to_buffer(init_map_received_count);
//             init_map_received_count++; 
//         }
//         bool round_meets_min = (temp_obstacles_count >= INIT_MAP_MIN_OBSTACLES
//             && temp_boxes_count >= INIT_MAP_MIN_BOXES
//             && temp_targets_count >= INIT_MAP_MIN_TARGETS);

//         if (round_meets_min || init_map_received_count >= MAX_SELECT_SIZE) {
//             select_and_apply_best_init_map();
//             data_reception_complete = true;
//             reset_temp_data();
//         } else {
//             reset_temp_data();
//         }
//         current_round_complete = false;
//     }
// }

