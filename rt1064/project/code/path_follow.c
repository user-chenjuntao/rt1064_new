#include "path_follow.h"
#include "motor.h"
#include "pid.h"
#include "zf_device_ips200.h"
#include <math.h>
#include "Attitude.h"



#ifndef M_PI
#define M_PI 3.1415926
#endif

#define DX_FAST_DISTANCE 0.6f  // 使用快速距离计算的阈值 m
#define DX_SLOW_DISTANCE 0.2f  // 使用慢速距离计算的阈值 m
#define DY_FAST_DISTANCE 0.6f  // 使用快速距离计算的阈值 m
#define DY_SLOW_DISTANCE 0.2f  // 使用慢速距离计算的阈值 m

#define CAR_DRIVER_MAX_SPEED 250.0f // 最大驱动速度 cm/s
#define CAR_DRIVER_MAX_ACCELERATE 2.5f // 最大驱动加速度 cm/s

#define CAR_DRIVER_MIN_SPEED 40.0f  // 最小驱动速度 cm/s

typedef struct
{
    float x_m;      // 世界坐标系 X，单位 m
    float y_m;      // 世界坐标系 Y，单位 m
    float yaw_deg;  // 航向角，度
} pose2d_t;

typedef struct
{
    const Point *path;     // 规划路径指针
    size_t steps;          // 路径长度
    size_t idx;            // 当前目标点索引
    float grid_m;          // 网格边长，m/格
    float pulses_per_meter;// 编码器每米脉冲数
    float pos_tol_m;       // 位置容差
    float yaw_tol_deg;     // 航向容差（保留，当前未用）
    float max_v_mps;       // 线速度上限 m/s
    float max_w_rad;       // 角速度上限 rad/s
    pose2d_t pose;         // 当前里程计位姿
    uint8 active;          // 1 表示跟随中
} path_follow_ctx_t;

static path_follow_ctx_t g_ctx = {0};
tagPID_T pid_world_x;     // X 轴位置 -> 速度 PID
tagPID_T pid_world_y;     // Y 轴位置 -> 速度 PID
tagPID_T pid_stay;         // 停留时位置 -> 速度 PID
tagPID_T pid_yaw;         // 航向 -> 角速度 PID
tagPID_T pid_accel_yaw;         // 航向 -> 角加速度 PID
static PIDInitStruct pid_world_init;
static PIDInitStruct pid_stay_init;
static PIDInitStruct pid_yaw_init;
static PIDInitStruct pid_accel_yaw_init;
uint8 car_direction = 0;  //0 表示停止；1表示x；2表示y；3表示同时x和y

// 对称限幅
static float clamp_sym(float v, float limit)
{
    if (v > limit)
    {
        return limit;
    }
    if (v < -limit)
    {
        return -limit;
    }
    return v;
}

// 将弧度限制在 (-pi, pi]
static float wrap_pi(float rad)
{
    while (rad > (float)M_PI)
    {
        rad -= 2.0f * (float)M_PI;
    }
    while (rad < -(float)M_PI)
    {
        rad += 2.0f * (float)M_PI;
    }
    return rad;
}

static void path_follow_update_pid_limits(void)
{
    // 运行时更新限幅，便于修改 max_v/max_w 后自动跟随
    pid_world_init.fMax_Iout = g_ctx.max_v_mps * 100.0f;
    pid_world_init.fMax_Out = g_ctx.max_v_mps * 100.0f;
    PID_Update(&pid_world_x, &pid_world_init);
    PID_Update(&pid_world_y, &pid_world_init);

    pid_yaw_init.fMax_Iout = g_ctx.max_w_rad * 20.0f;
    pid_yaw_init.fMax_Out = g_ctx.max_w_rad * 20.0f;
    PID_Update(&pid_yaw, &pid_yaw_init);
}

static int8_t fsignal(float num)
{
    if (num > 0)
        return 1;
    else if (num < 0)
        return -1;
    else
        return 0;
}

static float f_my_max(float a, float b)
{
    float a_abs = fabsf(a);
    float b_abs = fabsf(b);
    return (a_abs > b_abs) ? a_abs : b_abs;
}

static float f_my_min(float a, float b)
{
    float a_abs = fabsf(a);
    float b_abs = fabsf(b);
    return (a_abs < b_abs) ? a_abs : b_abs;
}

void path_follow_init(float grid_size_m, float pulses_per_meter)
{
    // 初始化基础参数与 PID
    g_ctx.grid_m = grid_size_m;
    g_ctx.pulses_per_meter = pulses_per_meter;
    g_ctx.pos_tol_m = 0.02f;   // 位置容差，可按需要调整
    g_ctx.yaw_tol_deg = 3.0f;  // 角度容差
    g_ctx.max_v_mps = 2.5f;    // 最大线速度
    g_ctx.max_w_rad = 360.0f;    // 最大角速度
    g_ctx.pose.x_m = 0.0f;
    g_ctx.pose.y_m = 0.0f;
    g_ctx.pose.yaw_deg = 0.0f;
    g_ctx.path = NULL;
    g_ctx.steps = 0;
    g_ctx.idx = 0;
    g_ctx.active = 0;

    pid_world_init.fKp = 2.2f;  // m 误差 -> 0.01 m/s
    pid_world_init.fKi = 0.0f;
    pid_world_init.fKd = 1.8f;
    pid_world_init.fMax_Iout = g_ctx.max_v_mps*100.0f;
    pid_world_init.fMax_Out = g_ctx.max_v_mps*100.0f;
    pid_world_init.alpha = 0.9f;

    pid_stay_init.fKp = 0.7f;  // m 误差 -> 0.01 m/s
    pid_stay_init.fKi = 0.0f;
    pid_stay_init.fKd = 0.3f;
    pid_stay_init.fMax_Iout = 200.0f;
    pid_stay_init.fMax_Out = 200.0f;
    pid_stay_init.alpha = 0.9f;

    pid_yaw_init.fKp = 6.2f;  // 度误差 -> rad/s*20
    pid_yaw_init.fKi = 0.0f;
    pid_yaw_init.fKd = 10.5f;
    pid_yaw_init.fMax_Iout = g_ctx.max_w_rad;
    pid_yaw_init.fMax_Out = g_ctx.max_w_rad;
    pid_yaw_init.alpha = 0.9f;

    pid_accel_yaw_init.fKp = 1.2f;  // 度误差 -> rad/s*20
    pid_accel_yaw_init.fKi = 0.0f;
    pid_accel_yaw_init.fKd = 2.1f;
    pid_accel_yaw_init.fMax_Iout = g_ctx.max_w_rad;
    pid_accel_yaw_init.fMax_Out = g_ctx.max_w_rad;
    pid_accel_yaw_init.alpha = 0.9f;

    PID_Init(&pid_world_x, &pid_world_init);
    PID_Init(&pid_stay, &pid_stay_init);
    PID_Init(&pid_world_y, &pid_world_init);
    PID_Init(&pid_yaw, &pid_yaw_init);
    PID_Init(&pid_accel_yaw, &pid_accel_yaw_init);
}

// 外部重置里程计起点
void path_follow_reset_pose(float x_m, float y_m, float yaw_deg)
{
    g_ctx.pose.x_m = x_m;
    g_ctx.pose.y_m = y_m;
    g_ctx.pose.yaw_deg = yaw_deg;
}

// 绑定新的规划路径并复位索引
void path_follow_set_path(const Point *path, size_t steps)
{
    g_ctx.path = path;
    g_ctx.steps = steps;
    g_ctx.idx = 0;
    if (path && steps > 0)
    {
        g_ctx.pose.x_m = path[0].row * g_ctx.grid_m;
        g_ctx.pose.y_m = path[0].col * g_ctx.grid_m;
        g_ctx.active = 1;
    }
    else
    {
        g_ctx.active = 0;
    }
}

// 从四轮编码器速度推算车体位姿，使用 yaw_angle 方向
static void update_odometry(float yaw_deg)
{
    if (g_ctx.pulses_per_meter <= 0.0f)
    {
        return;
    }

    float count_to_mps = ((float)PID_RATE) / g_ctx.pulses_per_meter;
    float w_ul = (float)up_L_all * count_to_mps;
    float w_ur = (float)up_R_all * count_to_mps;
    float w_dl = (float)down_L_all * count_to_mps;
    float w_dr = (float)down_R_all * count_to_mps;

    float vx_body = 0.25f * (w_ul + w_ur + w_dl + w_dr);
    float vy_body = 0.25f * (-w_ul + w_ur + w_dl - w_dr);
    float omega_body = (-w_ul + w_ur - w_dl + w_dr) / (2*D_X + 2*D_Y);
    // if (rx_plus_ry_cali != 0.0f)
    // {
    //     omega_body /= rx_plus_ry_cali;
    // }

    float dt = 1.0f / (float)PID_RATE;
    float yaw_rad = yaw_deg * ((float)M_PI / 180.0f);

    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    float vx_world = vx_body * cos_yaw - vy_body * sin_yaw;
    float vy_world = vx_body * sin_yaw + vy_body * cos_yaw;

    g_ctx.pose.x_m += vx_world * dt;
    g_ctx.pose.y_m += vy_world * dt;
    g_ctx.pose.yaw_deg = yaw_deg;
}

// 跟随路径生成三轴速度指令（车体系）
void path_follow_update(float yaw_deg, path_follow_output_t *out)
{
    if (out)
    {
        out->active = 0;
        out->reached = 0;
        out->vx_cmd = 0;
        out->vy_cmd = 0;
        out->omega_cmd = 0;
        out->target_idx = g_ctx.idx;
    }

    if (!g_ctx.active || NULL == g_ctx.path || 0 == g_ctx.steps)
    {
        return;
    }

    update_odometry(yaw_deg);
    // path_follow_update_pid_limits();

    Point target = g_ctx.path[g_ctx.idx];
    float target_x = target.row * g_ctx.grid_m;
    float target_y = target.col * g_ctx.grid_m;

    float dx = target_x - g_ctx.pose.x_m;
    float dx_fabs = fabsf(dx);
    float dy = target_y - g_ctx.pose.y_m;
    float dy_fabs = fabsf(dy);
    float dist = sqrtf(dx * dx + dy * dy);

    if (g_ctx.idx > 0 && g_ctx.active == 1)
    {
        if (g_ctx.path[g_ctx.idx].row - g_ctx.path[g_ctx.idx-1].row == 0)
        {
            car_direction = 2; //y方向
        }
        else if (g_ctx.path[g_ctx.idx].col - g_ctx.path[g_ctx.idx-1].col == 0)
        {
            car_direction = 1; //x方向
        }
        else
        {
            car_direction = 3; //xy方向
        }
    }
    else
    {
        car_direction = 0; //停止
    }

    if (dist < g_ctx.pos_tol_m)
    {
        if (g_ctx.idx + 1 < g_ctx.steps)
        {
            g_ctx.idx++;
            target = g_ctx.path[g_ctx.idx];
            target_x = target.row * g_ctx.grid_m;
            target_y = target.col * g_ctx.grid_m;
            dx = target_x - g_ctx.pose.x_m;
            dy = target_y - g_ctx.pose.y_m;
            dist = sqrtf(dx * dx + dy * dy);
        }
        else
        {
            g_ctx.active = 0;
            if (out)
            {
                out->reached = 1;
            }
            return;
        }
    }

    // float desired_heading_rad = atan2f(dy, dx);
    // float desired_heading_deg = desired_heading_rad * (180.0f / (float)M_PI);
    float yaw_rad = yaw_deg * ((float)M_PI / 180.0f);
    int v_world_x_cmd = 0;
    int v_world_y_cmd = 0;
    int v_world_x = 0;
    int v_world_y = 0;

    if (car_direction == 1)
    {
        if (dx_fabs > DX_FAST_DISTANCE)
        {
            v_world_x_cmd = fsignal(dx) * f_my_min(CAR_DRIVER_MAX_SPEED,100*sqrtf(2*CAR_DRIVER_MAX_ACCELERATE*dx_fabs)) + 0.1*PID_Location_Calculate(&pid_world_x, g_ctx.pose.x_m*100, target_x*100);
        }
        else if (dx_fabs > DX_SLOW_DISTANCE && dx_fabs <= DX_FAST_DISTANCE)
        {
            v_world_x_cmd = fsignal(dx) * f_my_max(PID_Location_Calculate(&pid_world_x, g_ctx.pose.x_m*100, target_x*100), CAR_DRIVER_MIN_SPEED);    
        }
        else if (dx_fabs <= DX_SLOW_DISTANCE)
        {
            v_world_x_cmd = PID_Location_Calculate(&pid_stay, g_ctx.pose.x_m*100, target_x*100) + fsignal(dx) * 90*20*(-1*(dx_fabs - 0.1f)*(dx_fabs - 0.1f)+0.01f);
        }

        v_world_y_cmd = PID_Location_Calculate(&pid_stay, g_ctx.pose.y_m*100, target_y*100);
    }
    else if (car_direction == 2)
    {
        if (dy_fabs > DY_FAST_DISTANCE)
        {
            v_world_y_cmd = fsignal(dy) * f_my_min(CAR_DRIVER_MAX_SPEED,100*sqrtf(2*CAR_DRIVER_MAX_ACCELERATE*dy_fabs)) + 0.1*PID_Location_Calculate(&pid_world_y, g_ctx.pose.y_m*100, target_y*100);
        }
        else if (dy_fabs > DY_SLOW_DISTANCE && dy_fabs <= DY_FAST_DISTANCE)
        {
            v_world_y_cmd = fsignal(dy) * f_my_max(PID_Location_Calculate(&pid_world_y, g_ctx.pose.y_m*100, target_y*100), CAR_DRIVER_MIN_SPEED);    
        }
        else if (dy_fabs <= DY_SLOW_DISTANCE)
        {
            v_world_y_cmd = PID_Location_Calculate(&pid_stay, g_ctx.pose.y_m*100, target_y*100)+ fsignal(dy) * 90*20*(-1*(dy_fabs - 0.1f)*(dy_fabs - 0.1f)+0.01f);
        }

        v_world_x_cmd = PID_Location_Calculate(&pid_stay, g_ctx.pose.x_m*100, target_x*100);
    }
    else
    {
        v_world_x_cmd = 0;
        v_world_y_cmd = 0;
    }
    
    v_world_x = (float)v_world_x_cmd;
    v_world_y = (float)v_world_y_cmd;


    // if (v_world_x > 0.0f && v_world_x < CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_x = CAR_DRIVER_MIN_SPEED;
    // }
    // else if (v_world_x < 0.0f && v_world_x > -CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_x = -CAR_DRIVER_MIN_SPEED;
    // }
    
    // if (v_world_y > 0.0f && v_world_y < CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_y = CAR_DRIVER_MIN_SPEED;
    // }
    // else if (v_world_y < 0.0f && v_world_y > -CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_y = -CAR_DRIVER_MIN_SPEED;
    // }

    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);
    float v_body_x = v_world_x * cos_yaw + v_world_y * sin_yaw;
    float v_body_y = -v_world_x * sin_yaw + v_world_y * cos_yaw;

    float omega_cmd = PID_Location_Calculate(&pid_yaw, yaw_deg, 0);
    // float omega_accel_cmd = PID_Location_Calculate(&pid_accel_yaw, omega_cmd, 0);
    omega_cmd = clamp_sym(omega_cmd, g_ctx.max_w_rad);

    if (out)
    {
        out->vx_cmd = (v_body_x);   // m/s -> 0.01 m/s
        out->vy_cmd = (v_body_y);
        out->omega_cmd = omega_cmd*(M_PI / 180.0f);               // rad/s*20 刻度
        out->active = 1;
        out->target_idx = g_ctx.idx;
    }
}

// 跟随路径生成三轴速度指令（车体系）
void path_follow_update_test(float yaw_deg, path_follow_output_t *out)
{
    if (out)
    {
        out->active = 0;
        out->reached = 0;
        out->vx_cmd = 0;
        out->vy_cmd = 0;
        out->omega_cmd = 0;
        out->target_idx = g_ctx.idx;
    }

    if (!g_ctx.active || NULL == g_ctx.path || 0 == g_ctx.steps)
    {
        return;
    }

    update_odometry(yaw_deg);
    // path_follow_update_pid_limits();

    Point target = g_ctx.path[g_ctx.idx];
    float target_x = target.row * g_ctx.grid_m;
    float target_y = target.col * g_ctx.grid_m;

    float dx = target_x - g_ctx.pose.x_m;
    float dx_fabs = fabsf(dx);
    float dy = target_y - g_ctx.pose.y_m;
    float dy_fabs = fabsf(dy);
    float dist = sqrtf(dx * dx + dy * dy);


    // float desired_heading_rad = atan2f(dy, dx);
    // float desired_heading_deg = desired_heading_rad * (180.0f / (float)M_PI);
    float yaw_rad = yaw_deg * ((float)M_PI / 180.0f);
    int v_world_x_cmd = 0;
    int v_world_y_cmd = 0;
    int v_world_x = 0;
    int v_world_y = 0;


    v_world_y_cmd = PID_Location_Calculate(&pid_stay, g_ctx.pose.y_m*100, target_y*100);
    v_world_x_cmd = PID_Location_Calculate(&pid_stay, g_ctx.pose.x_m*100, target_x*100);

    // if (abs(v_world_x_cmd) < 8 && abs(v_world_x_cmd) > 3)
    // {
    //     v_world_x = (float)v_world_x_cmd + fsignal(v_world_x_cmd)*1.0f;
    // }
    // else
    // {
        v_world_x = (float)v_world_x_cmd;
    // }

    // if (abs(v_world_y_cmd) < 8 && abs(v_world_y_cmd) > 3)
    // {
    //     v_world_y = (float)v_world_y_cmd + fsignal(v_world_y_cmd)*1.0f;
    // }
    // else
    // {
        v_world_y = (float)v_world_y_cmd;
    // }

    // if (v_world_x > 0.0f && v_world_x < CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_x = CAR_DRIVER_MIN_SPEED;
    // }
    // else if (v_world_x < 0.0f && v_world_x > -CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_x = -CAR_DRIVER_MIN_SPEED;
    // }
    
    // if (v_world_y > 0.0f && v_world_y < CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_y = CAR_DRIVER_MIN_SPEED;
    // }
    // else if (v_world_y < 0.0f && v_world_y > -CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_y = -CAR_DRIVER_MIN_SPEED;
    // }

    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);
    float v_body_x = v_world_x * cos_yaw + v_world_y * sin_yaw;
    float v_body_y = -v_world_x * sin_yaw + v_world_y * cos_yaw;
    // float v_body_x = 0;
    // float v_body_y = 0;

    float omega_cmd = PID_Location_Calculate(&pid_yaw, yaw_deg, 0);
    omega_cmd = clamp_sym(omega_cmd, g_ctx.max_w_rad);

    if (out)
    {
        out->vx_cmd = (v_body_x);   // m/s -> 0.01 m/s
        out->vy_cmd = (v_body_y);
        out->omega_cmd = omega_cmd*(M_PI / 180.0f);               // rad/s*20 刻度
        out->active = 1;
        out->target_idx = g_ctx.idx;
    }
}

void distance_speed_strategy(void)
{
    path_follow_output_t pf = {0};

    path_follow_update(eulerAngle.yaw, &pf);

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

// 查询当前里程计与目标信息
void path_follow_get_status(path_follow_status_t *status)
{
    if (NULL == status)
    {
        return;
    }

    status->x_m = g_ctx.pose.x_m;
    status->y_m = g_ctx.pose.y_m;
    status->yaw_deg = g_ctx.pose.yaw_deg;
    status->active = g_ctx.active;
    status->reached = (g_ctx.active == 0);
    status->target_idx = g_ctx.idx;

    if (g_ctx.path && g_ctx.idx < g_ctx.steps)
    {
        status->target_x_m = g_ctx.path[g_ctx.idx].row * g_ctx.grid_m;
        status->target_y_m = g_ctx.path[g_ctx.idx].col * g_ctx.grid_m;
    }
    else
    {
        status->target_x_m = 0.0f;
        status->target_y_m = 0.0f;
    }
}

// 在 IPS200 上显示当前位姿与目标点
void path_follow_draw_status(void)
{
    path_follow_status_t st = {0};
    path_follow_get_status(&st);

    // ips200_show_string(x, y, "Pose x y yaw:");
    ips200_show_string(0,80,"st_x_m");
    ips200_show_float(70, 80, st.x_m, 2, 4);
    ips200_show_string(0,96,"st_y_m");
    ips200_show_float(70, 96, st.y_m, 2, 4);
    ips200_show_string(0,112,"st_yaw");
    ips200_show_float(70, 112, st.yaw_deg, 3, 2);

    // ips200_show_string(x, y + 16, "Target idx/x/y:");
    ips200_show_string(0,128,"target_idx");
    ips200_show_uint(100, 128, st.target_idx, 3);
    ips200_show_string(0,144,"target_x_m");
    ips200_show_float(100, 144, st.target_x_m, 2, 4);
    ips200_show_string(0,160,"target_y_m");
    ips200_show_float(100, 160, st.target_y_m, 2, 4);
}

// 前向为0度（+X 方向），左侧为0~180，右侧为0~-180
float path_follow_heading_deg(Point from, Point to)
{
    float dx = (float)(to.row - from.row);
    float dy = (float)(to.col - from.col);
    if (dx == 0.0f && dy == 0.0f)
    {
        return 0.0f;
    }
    float angle_rad = atan2f(dy, dx);
    return angle_rad * (180.0f / (float)M_PI);
}

// 提取路径中的拐点（方向发生变化的点）
size_t path_follow_extract_corners(const Point *path, size_t path_steps, 
                                   Point *corner_buffer, size_t corner_capacity)
{
    if (NULL == path || 0 == path_steps || NULL == corner_buffer || 0 == corner_capacity)
    {
        return 0;
    }

    // 如果路径只有一个点，直接返回
    if (path_steps == 1)
    {
        if (corner_capacity >= 1)
        {
            corner_buffer[0] = path[0];
            return 1;
        }
        return 0;
    }

    size_t corner_count = 0;

    // 第一个点总是拐点（起点）
    if (corner_count >= corner_capacity)
    {
        return 0;  // 缓冲区不足
    }
    corner_buffer[corner_count++] = path[0];

    // 检查中间点是否为拐点
    for (size_t i = 1; i < path_steps - 1; i++)
    {
        // 计算从上一个点到当前点的方向向量
        int dx1 = path[i].col - path[i - 1].col;
        int dy1 = path[i].row - path[i - 1].row;

        // 计算从当前点到下一个点的方向向量
        int dx2 = path[i + 1].col - path[i].col;
        int dy2 = path[i + 1].row - path[i].row;

        // 如果方向向量不同，则当前点是拐点
        if (dx1 != dx2 || dy1 != dy2)
        {
            if (corner_count >= corner_capacity)
            {
                return 0;  // 缓冲区不足
            }
            corner_buffer[corner_count++] = path[i];
        }
    }

    // 最后一个点总是拐点（终点）
    if (corner_count >= corner_capacity)
    {
        return 0;  // 缓冲区不足
    }
    corner_buffer[corner_count++] = path[path_steps - 1];

    return corner_count;
}
