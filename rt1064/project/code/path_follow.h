#ifndef PATH_FOLLOW_H
#define PATH_FOLLOW_H

#include <stddef.h>
#include "assigned_box_planner_greedy.h"
#include "zf_common_typedef.h"
#include "pid.h"

typedef struct
{
    float vx_cmd;       // 车体坐标系前进方向，单位0.01 m/s（传 Kinematics_Inverse 前的值）
    float vy_cmd;       // 车体坐标系左向，单位0.01 m/s
    float omega_cmd;    // 车体角度，单位：rad/s * 20（沿speed_three_array 的刻度）
    uint8 active;       // 1 表示正在跟随
    uint8 reached;      // 1 表示路径已走完
    size_t target_idx;  // 当前目标路径点下标
} path_follow_output_t;

typedef struct
{
    float x_m;          // 当前世界坐标 X，单位 m
    float y_m;          // 当前世界坐标 Y，单位 m
    float yaw_deg;      // 当前航向角，单位度
    float target_x_m;   // 目标点世界坐标 X，单位 m
    float target_y_m;   // 目标点世界坐标 Y，单位 m
    uint8 active;       // 1 跟随中
    uint8 reached;      // 1 路径完成
    size_t target_idx;  // 当前目标点索引
} path_follow_status_t;

void path_follow_init(float grid_size_m, float pulses_per_meter);
void path_follow_set_path(const Point *path, size_t steps);
void path_follow_reset_pose(float x_m, float y_m, float yaw_deg);
void path_follow_update(float yaw_deg, path_follow_output_t *out);
void path_follow_get_status(path_follow_status_t *status);
void path_follow_draw_status(void);
float path_follow_heading_deg(Point from, Point to);
void distance_speed_strategy(void);

/**
 * 提取路径中的拐点（方向发生变化的点）
 * @param path 原始路径数组
 * @param path_steps 原始路径长度
 * @param corner_buffer 输出拐点数组缓冲区
 * @param corner_capacity 拐点缓冲区容量
 * @return 实际提取的拐点数量，如果缓冲区不足则返回0
 */
size_t path_follow_extract_corners(const Point *path, size_t path_steps, 
                                   Point *corner_buffer, size_t corner_capacity);

extern tagPID_T pid_world_x;
extern tagPID_T pid_world_y;
extern tagPID_T pid_yaw;
extern tagPID_T pid_accel_yaw;

#endif
