#include "zf_common_headfile.h"
#include <string.h>

#define FLASH_SECTION_INDEX            127
#define FLASH_PAGE_INDEX               FLASH_PAGE_3
#define PATH_ITEMS_PER_PAGE            32

#define FLASH_RESULT_NONE    0
#define FLASH_RESULT_SUCCESS 1
#define FLASH_RESULT_FAIL    2
#define FLASH_RESULT_CLEARED 3

typedef enum
{
    MENU_MODE_NAV = 0,
    MENU_MODE_EDIT,
    MENU_MODE_VIEW
} menu_mode_t;

typedef struct
{
    menu_item_t *current;
    uint8 selected_index;
    menu_mode_t mode;
    menu_item_t *active_item;
    uint8 need_clear;
    uint8 path_page;
    uint8 flash_result;
} menu_runtime_t;

static const float level[] = {100, 10, 1, 0.1f, 0.01f};
static uint8 level_i = 2;                // 默认步进设为1，避免上电误调过大
static menu_runtime_t menu_rt = {0};     // 菜单运行时状态
static uint8 display_mode = 0;           // 0:调试模式(刷新) 1:高速模式(发车后不刷新)
static uint8 path_index_visible = 1;     // 路径序号显示开关


extern uint8 car_go_flag;
extern uint8 car_stop_flag;
extern volatile uint8 push_box_flag;
extern volatile float now_rotate_angle;

extern size_t steps;
extern Point path[GREEDY_AREA];
extern int res;
extern size_t box_target_mapping[3];
extern PlannerChainInfo chain_info;

extern float speed_k;
extern int speed_limit;

extern int straight_speed;
extern int turn_speed;
extern int yuanhuan_speed;
extern int start_accelerate;
extern int end_decelerate;
extern int slow_turn_speed;
extern int turn_num;

extern uint8 huandao_flag;
extern uint8 crossing_flag_help;
extern uint8 line;
extern uint8 Left_Lost_Time;
extern uint8 Right_Lost_Time;
extern uint8 num_line;

// menu helpers
void menu_bind_parents(menu_item_t *parent);
menu_item_t *menu_get_selected_item(void);
void menu_move_up(void);
void menu_move_down(void);
void menu_enter_selected(void);
void menu_exit(void);
void menu_toggle_step(void);
void menu_handle_edit_adjust(int8 direction);
void menu_handle_view_keys(key_state_enum k1, key_state_enum k2, key_state_enum k3, key_state_enum k4);
void menu_draw_header(void);
void menu_draw_list(void);
void menu_draw_detail(void);
void menu_draw_level_hint(void);
void menu_draw_hint(void);
uint16 menu_total_path_pages(void);
uint8 param_clear_flash(void);

// actions
void action_start_car(void);
void action_stop_car(void);
void action_save_flash(void);
void action_load_flash(void);
void action_clear_flash(void);
void action_set_debug_mode(void);
void action_set_high_mode(void);

// draw helpers
void draw_main_info(void);
void draw_cargo_info(void);
void draw_status_camera(void);
void draw_status_path(void);
void draw_status_pid(void);
void draw_status_imu(void);
void draw_flash_status(void);
void draw_image_preview(void);
void draw_display_mode(void);

// menu tree definitions
static menu_item_t main_menu;
static menu_item_t cargo_menu;
static menu_item_t param_menu;
static menu_item_t param_speed_menu;
static menu_item_t param_servo_menu;
static menu_item_t param_pid_menu;
static menu_item_t status_menu;
static menu_item_t status_flash_menu;
static menu_item_t status_imu_view;
static menu_item_t image_menu;
static menu_item_t display_menu;

static menu_item_t cargo_start_item;
static menu_item_t cargo_stop_item;

static menu_item_t base_speed_item;
static menu_item_t turn_speed_item;
static menu_item_t speed_k_item;
static menu_item_t speed_limit_item;
static menu_item_t yuanhuan_speed_item;
static menu_item_t pid_world_x_kp_item;
static menu_item_t pid_world_x_ki_item;
static menu_item_t pid_world_x_kd_item;

static menu_item_t up_left_speed_item;
static menu_item_t up_right_speed_item;
static menu_item_t down_left_speed_item;
static menu_item_t down_right_speed_item;
static menu_item_t start_acc_item;
static menu_item_t end_decel_item;
static menu_item_t slow_turn_item;
static menu_item_t turn_num_item;

static menu_item_t pid_world_y_kp_item;
static menu_item_t pid_world_y_ki_item;
static menu_item_t pid_world_y_kd_item;
static menu_item_t pid_yaw_kp_item;
static menu_item_t pid_yaw_ki_item;
static menu_item_t pid_yaw_kd_item;
static menu_item_t pid_accel_yaw_kp_item;
static menu_item_t pid_accel_yaw_ki_item;
static menu_item_t pid_accel_yaw_kd_item;

static menu_item_t status_camera_view;
static menu_item_t status_path_view;
static menu_item_t status_pid_view;
static menu_item_t flash_save_item;
static menu_item_t flash_load_item;
static menu_item_t flash_clear_item;

static menu_item_t image_binary_view;
static menu_item_t image_variable_view;
static menu_item_t display_debug_mode_item;
static menu_item_t display_high_mode_item;

static const menu_item_t *const main_children[] = {&cargo_menu, &param_menu, &status_menu, &image_menu, &display_menu};
static const menu_item_t *const cargo_children[] = {&cargo_start_item, &cargo_stop_item};
static const menu_item_t *const param_children[] = {&param_speed_menu, &param_servo_menu, &param_pid_menu};
static const menu_item_t *const speed_children[] = {&base_speed_item, &turn_speed_item, &speed_k_item, &speed_limit_item, &yuanhuan_speed_item};
static const menu_item_t *const servo_children[] = {&up_left_speed_item, &up_right_speed_item, &down_left_speed_item, &down_right_speed_item, &start_acc_item, &end_decel_item, &slow_turn_item, &turn_num_item};
static const menu_item_t *const pid_children[] = {
    &pid_world_x_kp_item, &pid_world_x_ki_item, &pid_world_x_kd_item,
    &pid_world_y_kp_item, &pid_world_y_ki_item, &pid_world_y_kd_item,
    &pid_yaw_kp_item, &pid_yaw_ki_item, &pid_yaw_kd_item,
    &pid_accel_yaw_kp_item, &pid_accel_yaw_ki_item, &pid_accel_yaw_kd_item};
static const menu_item_t *const status_children[] = {&status_camera_view, &status_path_view, &status_pid_view, &status_imu_view, &status_flash_menu};
static const menu_item_t *const flash_children[] = {&flash_save_item, &flash_load_item, &flash_clear_item};
static const menu_item_t *const image_children[] = {&image_binary_view, &image_variable_view};
static const menu_item_t *const display_children[] = {&display_debug_mode_item, &display_high_mode_item};
static menu_item_t main_menu = {
    .title = "Main",
    .type = MENU_ITEM_PAGE,
    .children = (const menu_item_t *const *)main_children,
    .child_count = sizeof(main_children) / sizeof(main_children[0]),
    .parent = NULL,
    .draw = draw_main_info,
};

static menu_item_t cargo_menu = {
    .title = "CarGo",
    .type = MENU_ITEM_PAGE,
    .children = (const menu_item_t *const *)cargo_children,
    .child_count = sizeof(cargo_children) / sizeof(cargo_children[0]),
    .draw = draw_cargo_info,
};

static menu_item_t param_menu = {
    .title = "Parameter",
    .type = MENU_ITEM_PAGE,
    .children = (const menu_item_t *const *)param_children,
    .child_count = sizeof(param_children) / sizeof(param_children[0]),
};

static menu_item_t param_speed_menu = {
    .title = "Speed",
    .type = MENU_ITEM_PAGE,
    .children = (const menu_item_t *const *)speed_children,
    .child_count = sizeof(speed_children) / sizeof(speed_children[0]),
};

static menu_item_t param_servo_menu = {
    .title = "Servo",
    .type = MENU_ITEM_PAGE,
    .children = (const menu_item_t *const *)servo_children,
    .child_count = sizeof(servo_children) / sizeof(servo_children[0]),
};

static menu_item_t param_pid_menu = {
    .title = "PID",
    .type = MENU_ITEM_PAGE,
    .children = (const menu_item_t *const *)pid_children,
    .child_count = sizeof(pid_children) / sizeof(pid_children[0]),
};

static menu_item_t status_menu = {
    .title = "Status",
    .type = MENU_ITEM_PAGE,
    .children = (const menu_item_t *const *)status_children,
    .child_count = sizeof(status_children) / sizeof(status_children[0]),
};

static menu_item_t status_flash_menu = {
    .title = "Flash",
    .type = MENU_ITEM_PAGE,
    .children = (const menu_item_t *const *)flash_children,
    .child_count = sizeof(flash_children) / sizeof(flash_children[0]),
    .draw = draw_flash_status,
};

static menu_item_t image_menu = {
    .title = "Image",
    .type = MENU_ITEM_PAGE,
    .children = (const menu_item_t *const *)image_children,
    .child_count = sizeof(image_children) / sizeof(image_children[0]),
};

static menu_item_t cargo_start_item = {
    .title = "Start",
    .type = MENU_ITEM_ACTION,
    .action = action_start_car,
};

static menu_item_t cargo_stop_item = {
    .title = "Stop",
    .type = MENU_ITEM_ACTION,
    .action = action_stop_car,
};

static menu_item_t base_speed_item = {
    .title = "base_speed",
    .type = MENU_ITEM_PARAM_INT,
    .param = {.value_ptr = &straight_speed, .min = -2000, .max = 2000, .width = 5},
};

static menu_item_t turn_speed_item = {
    .title = "turn_speed",
    .type = MENU_ITEM_PARAM_INT,
    .param = {.value_ptr = &turn_speed, .min = -2000, .max = 2000, .width = 5},
};

static menu_item_t speed_k_item = {
    .title = "speed_k",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = {.value_ptr = &speed_k, .min = 0, .max = 5, .precision = 2, .width = 3},
};

static menu_item_t speed_limit_item = {
    .title = "speed_limit",
    .type = MENU_ITEM_PARAM_INT,
    .param = {.value_ptr = &speed_limit, .min = 0, .max = 2000, .width = 5},
};

static menu_item_t yuanhuan_speed_item = {
    .title = "yuanhuan_speed",
    .type = MENU_ITEM_PARAM_INT,
    .param = {.value_ptr = &yuanhuan_speed, .min = -2000, .max = 2000, .width = 5},
};

static menu_item_t pid_world_x_kp_item = {
    .title = "wx_kp",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = {.value_ptr = &pid_world_x.fKp, .min = 0, .max = 10, .precision = 3, .width = 4},
};

static menu_item_t pid_world_x_ki_item = {
    .title = "wx_ki",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = {.value_ptr = &pid_world_x.fKi, .min = 0, .max = 10, .precision = 3, .width = 4},
};

static menu_item_t pid_world_x_kd_item = {
    .title = "wx_kd",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = {.value_ptr = &pid_world_x.fKd, .min = 0, .max = 10, .precision = 3, .width = 4},
};

static menu_item_t up_left_speed_item = {
    .title = "up_left_speed",
    .type = MENU_ITEM_PARAM_INT,
    .param = {.value_ptr = &speed_encoder[0], .read_only = 1, .width = 5},
};

static menu_item_t up_right_speed_item = {
    .title = "up_right_speed",
    .type = MENU_ITEM_PARAM_INT,
    .param = {.value_ptr = &speed_encoder[1], .read_only = 1, .width = 5},
};

static menu_item_t down_left_speed_item = {
    .title = "down_left_speed",
    .type = MENU_ITEM_PARAM_INT,
    .param = {.value_ptr = &speed_encoder[2], .read_only = 1, .width = 5},
};

static menu_item_t down_right_speed_item = {
    .title = "down_right_speed",
    .type = MENU_ITEM_PARAM_INT,
    .param = {.value_ptr = &speed_encoder[3], .read_only = 1, .width = 5},
};

static menu_item_t start_acc_item = {
    .title = "start_acc",
    .type = MENU_ITEM_PARAM_INT,
    .param = {.value_ptr = &start_accelerate, .min = 0, .max = 4000, .width = 5},
};

static menu_item_t end_decel_item = {
    .title = "end_decel",
    .type = MENU_ITEM_PARAM_INT,
    .param = {.value_ptr = &end_decelerate, .min = 0, .max = 4000, .width = 5},
};

static menu_item_t slow_turn_item = {
    .title = "slow_turn",
    .type = MENU_ITEM_PARAM_INT,
    .param = {.value_ptr = &slow_turn_speed, .min = -2000, .max = 2000, .width = 5},
};

static menu_item_t turn_num_item = {
    .title = "turn_num",
    .type = MENU_ITEM_PARAM_INT,
    .param = {.value_ptr = &turn_num, .min = 0, .max = 255, .width = 3},
};

static menu_item_t pid_world_y_kp_item = {
    .title = "wy_kp",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = {.value_ptr = &pid_world_y.fKp, .min = 0, .max = 10, .precision = 3, .width = 4},
};

static menu_item_t pid_world_y_ki_item = {
    .title = "wy_ki",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = {.value_ptr = &pid_world_y.fKi, .min = 0, .max = 10, .precision = 3, .width = 4},
};

static menu_item_t pid_world_y_kd_item = {
    .title = "wy_kd",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = {.value_ptr = &pid_world_y.fKd, .min = 0, .max = 10, .precision = 3, .width = 4},
};

static menu_item_t pid_yaw_kp_item = {
    .title = "yaw_kp",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = {.value_ptr = &pid_yaw.fKp, .min = 0, .max = 10, .precision = 3, .width = 4},
};

static menu_item_t pid_yaw_ki_item = {
    .title = "yaw_ki",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = {.value_ptr = &pid_yaw.fKi, .min = 0, .max = 10, .precision = 3, .width = 4},
};

static menu_item_t pid_yaw_kd_item = {
    .title = "yaw_kd",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = {.value_ptr = &pid_yaw.fKd, .min = 0, .max = 10, .precision = 3, .width = 4},
};

static menu_item_t pid_accel_yaw_kp_item = {
    .title = "ay_kp",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = {.value_ptr = &pid_accel_yaw.fKp, .min = 0, .max = 10, .precision = 3, .width = 4},
};

static menu_item_t pid_accel_yaw_ki_item = {
    .title = "ay_ki",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = {.value_ptr = &pid_accel_yaw.fKi, .min = 0, .max = 10, .precision = 3, .width = 4},
};

static menu_item_t pid_accel_yaw_kd_item = {
    .title = "ay_kd",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = {.value_ptr = &pid_accel_yaw.fKd, .min = 0, .max = 10, .precision = 3, .width = 4},
};

static menu_item_t status_camera_view = {
    .title = "Camera",
    .type = MENU_ITEM_VIEW,
    .draw = draw_status_camera,
};

static menu_item_t status_path_view = {
    .title = "Path",
    .type = MENU_ITEM_VIEW,
    .draw = draw_status_path,
};

static menu_item_t status_pid_view = {
    .title = "PID",
    .type = MENU_ITEM_VIEW,
    .draw = draw_status_pid,
};

static menu_item_t status_imu_view = {
    .title = "IMU",
    .type = MENU_ITEM_VIEW,
    .draw = draw_status_imu,
};

static menu_item_t flash_save_item = {
    .title = "Save",
    .type = MENU_ITEM_ACTION,
    .action = action_save_flash,
};

static menu_item_t flash_load_item = {
    .title = "Load",
    .type = MENU_ITEM_ACTION,
    .action = action_load_flash,
};

static menu_item_t flash_clear_item = {
    .title = "Clear",
    .type = MENU_ITEM_ACTION,
    .action = action_clear_flash,
};

static menu_item_t image_binary_view = {
    .title = "BinaryImage",
    .type = MENU_ITEM_VIEW,
    .draw = draw_image_preview,
};

static menu_item_t image_variable_view = {
    .title = "Variable",
    .type = MENU_ITEM_VIEW,
    .draw = draw_image_preview,
};

static menu_item_t display_debug_mode_item = {
    .title = "DebugMode",
    .type = MENU_ITEM_ACTION,
    .action = action_set_debug_mode,
};

static menu_item_t display_high_mode_item = {
    .title = "HighSpeedMode",
    .type = MENU_ITEM_ACTION,
    .action = action_set_high_mode,
};

static menu_item_t display_menu = {
    .title = "Display",
    .type = MENU_ITEM_PAGE,
    .children = (const menu_item_t *const *)display_children,
    .child_count = sizeof(display_children) / sizeof(display_children[0]),
    .draw = draw_display_mode,
};
void menu_bind_parents(menu_item_t *parent)
{
    if (NULL == parent || NULL == parent->children)
    {
        return;
    }
    for (uint8 i = 0; i < parent->child_count; i++)
    {
        menu_item_t *child = (menu_item_t *)parent->children[i];
        child->parent = parent;
        if (MENU_ITEM_PAGE == child->type)
        {
            menu_bind_parents(child);
        }
    }
}

menu_item_t *menu_get_selected_item(void)
{
    if (NULL == menu_rt.current || 0 == menu_rt.current->child_count)
    {
        return NULL;
    }
    if (menu_rt.selected_index >= menu_rt.current->child_count)
    {
        menu_rt.selected_index = 0;
    }
    return (menu_item_t *)menu_rt.current->children[menu_rt.selected_index];
}

void menu_move_up(void)
{
    if (0 == menu_rt.current->child_count)
    {
        return;
    }
    if (menu_rt.selected_index == 0)
    {
        menu_rt.selected_index = menu_rt.current->child_count - 1;
    }
    else
    {
        menu_rt.selected_index--;
    }
}

void menu_move_down(void)
{
    if (0 == menu_rt.current->child_count)
    {
        return;
    }
    menu_rt.selected_index++;
    if (menu_rt.selected_index >= menu_rt.current->child_count)
    {
        menu_rt.selected_index = 0;
    }
}

void menu_enter_selected(void)
{
    menu_item_t *item = menu_get_selected_item();
    if (NULL == item)
    {
        return;
    }

    switch (item->type)
    {
        case MENU_ITEM_PAGE:
            if (item->child_count > 0)
            {
                menu_rt.current = item;
                menu_rt.selected_index = 0;
                menu_rt.active_item = NULL;
                menu_rt.mode = MENU_MODE_NAV;
                menu_rt.need_clear = 1;
            }
            else if (item->draw)
            {
                menu_rt.active_item = item;
                menu_rt.mode = MENU_MODE_VIEW;
                menu_rt.need_clear = 1;
            }
            break;
        case MENU_ITEM_PARAM_INT:
        case MENU_ITEM_PARAM_FLOAT:
            if (item->param.read_only)
            {
                break;
            }
            menu_rt.active_item = item;
            menu_rt.mode = MENU_MODE_EDIT;
            menu_rt.need_clear = 1;
            break;
        case MENU_ITEM_ACTION:
            if (item->action)
            {
                item->action();
                menu_rt.need_clear = 1;
            }
            break;
        case MENU_ITEM_VIEW:
            menu_rt.active_item = item;
            menu_rt.mode = MENU_MODE_VIEW;
            menu_rt.need_clear = 1;
            break;
        default:
            break;
    }
}

void menu_exit(void)
{
    if (MENU_MODE_EDIT == menu_rt.mode || MENU_MODE_VIEW == menu_rt.mode)
    {
        menu_rt.mode = MENU_MODE_NAV;
        menu_rt.active_item = NULL;
        menu_rt.need_clear = 1;
        return;
    }

    if (menu_rt.current && menu_rt.current->parent)
    {
        menu_item_t *parent = menu_rt.current->parent;
        uint8 idx = 0;
        for (uint8 i = 0; i < parent->child_count; i++)
        {
            if (parent->children[i] == menu_rt.current)
            {
                idx = i;
                break;
            }
        }
        menu_rt.current = parent;
        menu_rt.selected_index = idx;
        menu_rt.need_clear = 1;
    }
}

void menu_toggle_step(void)
{
    level_i++;
    if (level_i >= sizeof(level) / sizeof(level[0]))
    {
        level_i = 0;
    }
}

float menu_clamp_float(float value, float min, float max)
{
    if (min < max)
    {
        if (value < min)
        {
            return min;
        }
        if (value > max)
        {
            return max;
        }
    }
    return value;
}

void menu_handle_edit_adjust(int8 direction)
{
    if (NULL == menu_rt.active_item)
    {
        return;
    }
    menu_item_t *item = menu_rt.active_item;
    if (item->param.read_only)
    {
        return;
    }

    float step = level[level_i] * direction;

    if (MENU_ITEM_PARAM_INT == item->type)
    {
        int32 *p = (int32 *)item->param.value_ptr;
        float value = *p + step;
        value = menu_clamp_float(value, item->param.min, item->param.max);
        *p = (int32)value;
    }
    else if (MENU_ITEM_PARAM_FLOAT == item->type)
    {
        float *p = (float *)item->param.value_ptr;
        float value = *p + step;
        value = menu_clamp_float(value, item->param.min, item->param.max);
        *p = value;
    }
}

uint16 menu_total_path_pages(void)
{
    if (0 == steps)
    {
        return 1;
    }
    uint16 total = (steps + PATH_ITEMS_PER_PAGE - 1) / PATH_ITEMS_PER_PAGE;
    return (0 == total) ? 1 : total;
}

void menu_handle_view_keys(key_state_enum k1, key_state_enum k2, key_state_enum k3, key_state_enum k4)
{
    if (&status_path_view == menu_rt.active_item)
    {
        uint16 total_pages = menu_total_path_pages();
        if (k1 == KEY_SHORT_PRESS || k1 == KEY_LONG_PRESS)
        {
            if (menu_rt.path_page > 0)
            {
                menu_rt.path_page--;
                menu_rt.need_clear = 1;
            }
        }
        if (k2 == KEY_SHORT_PRESS || k2 == KEY_LONG_PRESS)
        {
            if (menu_rt.path_page + 1 < total_pages)
            {
                menu_rt.path_page++;
                menu_rt.need_clear = 1;
            }
        }
        if (k3 == KEY_SHORT_PRESS || k3 == KEY_LONG_PRESS)
        {
            path_index_visible = !path_index_visible;
            menu_rt.need_clear = 1;
        }
    }

    if (k4 == KEY_SHORT_PRESS || k4 == KEY_LONG_PRESS)
    {
        menu_rt.mode = MENU_MODE_NAV;
        menu_rt.active_item = NULL;
        menu_rt.need_clear = 1;
    }
}
void menu_draw_header(void)
{
    const char *title = "Menu";
    if (MENU_MODE_VIEW == menu_rt.mode && menu_rt.active_item && menu_rt.active_item->title)
    {
        title = menu_rt.active_item->title;
    }
    else if (menu_rt.current && menu_rt.current->title)
    {
        title = menu_rt.current->title;
    }
    ips200_show_string(0, 0, title);
    switch (menu_rt.mode)
    {
        case MENU_MODE_NAV:
            ips200_show_string(192, 0, "NAV");
            break;
        case MENU_MODE_EDIT:
            ips200_show_string(192, 0, "EDIT");
            break;
        case MENU_MODE_VIEW:
            ips200_show_string(192, 0, "VIEW");
            break;
        default:
            break;
    }
}

void menu_draw_value(menu_item_t *item, uint16 y)
{
    uint8 width = item->param.width ? item->param.width : 4;
    if (MENU_ITEM_PARAM_INT == item->type)
    {
        int32 value = *((int32 *)item->param.value_ptr);
        ips200_show_int(140, y, value, width);
    }
    else if (MENU_ITEM_PARAM_FLOAT == item->type)
    {
        float value = *((float *)item->param.value_ptr);
        uint8 precision = item->param.precision ? item->param.precision : 2;
        ips200_show_float(132, y, value, width, precision);
    }

    if (item->param.read_only)
    {
        ips200_show_string(210, y, "R");
    }
    else if (menu_rt.active_item == item && MENU_MODE_EDIT == menu_rt.mode)
    {
        ips200_show_string(210, y, "*");
    }
    else
    {
        ips200_show_string(210, y, " ");
    }
}

void menu_draw_list(void)
{
    if (NULL == menu_rt.current)
    {
        return;
    }

    for (uint8 i = 0; i < menu_rt.current->child_count; i++)
    {
        menu_item_t *item = (menu_item_t *)menu_rt.current->children[i];
        uint16 y = 16 + i * 16;
        if (i == menu_rt.selected_index)
        {
            ips200_show_string(0, y, "->");
        }
        else
        {
            ips200_show_string(0, y, "  ");
        }
        ips200_show_string(16, y, item->title);

        if (MENU_ITEM_PARAM_INT == item->type || MENU_ITEM_PARAM_FLOAT == item->type)
        {
            menu_draw_value(item, y);
        }
    }
}

void menu_draw_detail(void)
{
    if (MENU_MODE_NAV == menu_rt.mode && menu_rt.current && menu_rt.current->draw)
    {
        menu_rt.current->draw();
    }

    if (MENU_MODE_VIEW == menu_rt.mode && menu_rt.active_item && menu_rt.active_item->draw)
    {
        menu_rt.active_item->draw();
    }
}

void menu_draw_level_hint(void)
{
    ips200_show_string(0, 288, "step:");
    ips200_show_float(40, 288, level[level_i], 3, 2);
}

void menu_draw_hint(void)
{
    if (MENU_MODE_NAV == menu_rt.mode)
    {
        ips200_show_string(0, 304, "K1:UP K2:DOWN K3:OK K4:BACK");
    }
    else if (MENU_MODE_EDIT == menu_rt.mode)
    {
        ips200_show_string(0, 304, "K1:- K2:+ K3:STEP K4:EXIT");
    }
    else
    {
        if (&status_path_view == menu_rt.active_item)
        {
            ips200_show_string(0, 304, "K1:PREV K2:NEXT K3:IDX K4:BACK");
        }
        else
        {
            ips200_show_string(0, 304, "K4:BACK");
        }
    }
}

void menu_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE_SPI);

    menu_bind_parents(&main_menu);
    menu_rt.current = &main_menu;
    menu_rt.selected_index = 0;
    menu_rt.mode = MENU_MODE_NAV;
    menu_rt.active_item = NULL;
    menu_rt.need_clear = 1;
    menu_rt.path_page = 0;
    menu_rt.flash_result = FLASH_RESULT_NONE;
    display_mode = 0;
    path_index_visible = 1;
}

void menu_display(void)
{
    if (display_mode == 1 && car_go_flag == 1)
    {
        return;
    }

    if (menu_rt.need_clear)
    {
        ips200_clear();
        menu_rt.need_clear = 0;
    }

    if (!(MENU_MODE_VIEW == menu_rt.mode && (menu_rt.active_item == &status_path_view || menu_rt.active_item == &status_imu_view)))
    {
        menu_draw_header();
        menu_draw_list();
    }

    menu_draw_detail();
    menu_draw_level_hint();
    menu_draw_hint();
}

void menu_switch(void)
{
    key_state_enum k1 = key_get_state(KEY_1);
    key_state_enum k2 = key_get_state(KEY_2);
    key_state_enum k3 = key_get_state(KEY_3);
    key_state_enum k4 = key_get_state(KEY_4);

    if (MENU_MODE_NAV == menu_rt.mode)
    {
        if (k1 == KEY_SHORT_PRESS)
        {
            menu_move_up();
        }
        else if (k2 == KEY_SHORT_PRESS)
        {
            menu_move_down();
        }

        if (k3 == KEY_SHORT_PRESS)
        {
            menu_enter_selected();
        }
        else if (k4 == KEY_SHORT_PRESS)
        {
            menu_exit();
        }
    }
    else if (MENU_MODE_EDIT == menu_rt.mode)
    {
        if (k1 == KEY_SHORT_PRESS || k1 == KEY_LONG_PRESS)
        {
            menu_handle_edit_adjust(-1);
        }
        if (k2 == KEY_SHORT_PRESS || k2 == KEY_LONG_PRESS)
        {
            menu_handle_edit_adjust(1);
        }
        if (k3 == KEY_SHORT_PRESS)
        {
            menu_toggle_step();
        }
        if (k4 == KEY_SHORT_PRESS)
        {
            menu_exit();
        }
    }
    else if (MENU_MODE_VIEW == menu_rt.mode)
    {
        menu_handle_view_keys(k1, k2, k3, k4);
    }

    key_clear_state(KEY_1);
    key_clear_state(KEY_2);
    key_clear_state(KEY_3);
    key_clear_state(KEY_4);
}
void action_start_car(void)
{
    car_go_flag = 1;
    car_stop_flag = 0;
}

void action_stop_car(void)
{
    car_go_flag = 0;
    car_stop_flag = 1;
    memset(speed_encoder, 0, sizeof(speed_encoder));
    motor_control(speed_encoder);
}

void action_save_flash(void)
{
    menu_rt.flash_result = (param_save_to_flash() == 1) ? FLASH_RESULT_SUCCESS : FLASH_RESULT_FAIL;
}

void action_load_flash(void)
{
    menu_rt.flash_result = (param_load_from_flash() == 1) ? FLASH_RESULT_SUCCESS : FLASH_RESULT_FAIL;
}

void action_clear_flash(void)
{
    menu_rt.flash_result = (param_clear_flash() == 1) ? FLASH_RESULT_CLEARED : FLASH_RESULT_FAIL;
}

void action_set_debug_mode(void)
{
    display_mode = 0;
    menu_rt.need_clear = 1;
}

void action_set_high_mode(void)
{
    display_mode = 1;
    menu_rt.need_clear = 1;
}

void draw_main_info(void)
{
    ips200_show_string(140, 16, "plan");
    ips200_show_int(180, 16, res, 3);
    ips200_show_string(140, 32, "steps");
    ips200_show_int(180, 32, steps, 4);
    ips200_show_string(140, 48, "b-t");
    for (int i=0;i<3;i++){
        ips200_show_int(180+i*8, 48, box_target_mapping[i], 3);
    }
    ips200_show_string(140, 64, "chain");
    for (size_t i = 0; i < chain_info.chain_count; i++) {
        for (size_t j = 0; j < chain_info.chain_lengths[i]; j++) {
            ips200_show_int(180+j*8, 64+i*16, chain_info.chain_indices[i][j], 3);
        }
    }
}

void draw_cargo_info(void)
{
    ips200_show_string(140, 16, "enc1");
    ips200_show_int(180, 16, up_L_all, 4);
    ips200_show_string(140, 32, "enc2");
    ips200_show_int(180, 32, up_R_all, 4);
    ips200_show_string(140, 48, "enc3");
    ips200_show_int(180, 48, down_L_all, 4);
    ips200_show_string(140, 64, "enc4");
    ips200_show_int(180, 64, down_R_all, 4);

    path_follow_draw_status();

    // ips200_show_string(0, 96, "huan");
    // ips200_show_uint(40, 96, huandao_flag, 1);
    // ips200_show_string(0, 112, "cross");
    // ips200_show_uint(40, 112, crossing_flag_help, 1);
    // ips200_show_string(0, 128, "line");
    // ips200_show_uint(40, 128, line, 3);

    // ips200_show_string(0, 144, "lostL");
    // ips200_show_uint(48, 144, Left_Lost_Time, 3);
    // ips200_show_string(0, 160, "lostR");
    // ips200_show_uint(48, 160, Right_Lost_Time, 3);

    // ips200_show_string(0, 176, "dist");
    // ips200_show_float(48, 176, blob_info.distance, 3, 1);
    // ips200_show_string(0, 192, "cx");
    // ips200_show_int(32, 192, blob_info.cx, 4);
    // ips200_show_string(100, 192, "push");
    // ips200_show_uint(140, 192, push_box_flag, 1);
    // ips200_show_string(0, 208, "yaw");
    // ips200_show_float(32, 208, now_rotate_angle, 3, 1);

}

void draw_status_camera(void)
{
    ips200_show_string(0, 176, "cx cy w h");
    ips200_show_int(0, 192, blob_info.cx, 4);
    ips200_show_int(48, 192, blob_info.cy, 4);
    ips200_show_int(96, 192, blob_info.w, 4);
    ips200_show_int(144, 192, blob_info.h, 4);

    ips200_show_string(0, 208, "area");
    ips200_show_int(40, 208, blob_info.area, 5);
    ips200_show_string(100, 208, "w_err");
    ips200_show_int(148, 208, blob_info.w_error, 4);

    ips200_show_string(0, 224, "dist");
    ips200_show_float(40, 224, blob_info.distance, 3, 1);
}

void draw_status_path(void)
{
    uint16 total_pages = menu_total_path_pages();
    if (menu_rt.path_page >= total_pages)
    {
        menu_rt.path_page = (total_pages > 0) ? total_pages - 1 : 0;
    }

    uint16 start = menu_rt.path_page * PATH_ITEMS_PER_PAGE;
    uint16 end = start + PATH_ITEMS_PER_PAGE;
    if (end > steps)
    {
        end = steps;
    }

    const uint16 rows_per_col = PATH_ITEMS_PER_PAGE / 2; // 16
    const uint16 col_width = 112;
    const uint16 x_right = col_width + 8; // 120

    ips200_show_string(0, 0, "Path page");
    ips200_show_uint(72, 0, menu_rt.path_page + 1, 2);
    ips200_show_string(88, 0, "/");
    ips200_show_uint(96, 0, total_pages, 2);

    ips200_draw_line(col_width, 16, col_width, 16 + rows_per_col * 16, RGB565_WHITE);

    if (path_index_visible)
    {
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    }

    for (uint16 row = 0; row < rows_per_col; row++)
    {
        uint16 left_idx = start + row;
        uint16 right_idx = start + rows_per_col + row;
        uint16 y = 16 + row * 16;

        if (left_idx < end)
        {
            if (path_index_visible)
            {
                ips200_show_uint(0, y, left_idx, 3);
            }
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
            ips200_show_uint(32, y, path[left_idx].row, 3);
            ips200_show_uint(64, y, path[left_idx].col, 3);
            if (path_index_visible)
            {
                ips200_set_color(RGB565_CYAN, RGB565_BLACK);
            }
        }

        if (right_idx < end)
        {
            if (path_index_visible)
            {
                ips200_show_uint(x_right, y, right_idx, 3);
            }
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
            ips200_show_uint(x_right + 32, y, path[right_idx].row, 3);
            ips200_show_uint(x_right + 64, y, path[right_idx].col, 3);
            if (path_index_visible)
            {
                ips200_set_color(RGB565_CYAN, RGB565_BLACK);
            }
        }
    }

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

void draw_status_pid(void)
{
    ips200_show_string(0, 176, "Yaw Kp/Ki/Kd");
    ips200_show_float(0, 192, Yawpid.fKp, 3, 2);
    ips200_show_float(48, 192, Yawpid.fKi, 3, 2);
    ips200_show_float(96, 192, Yawpid.fKd, 3, 2);
    ips200_show_string(0, 208, "MaxI/MaxO/a");
    ips200_show_float(0, 224, Yawpid.fMax_Iout, 3, 2);
    ips200_show_float(48, 224, Yawpid.fMax_Out, 3, 2);
    ips200_show_float(96, 224, Yawpid.alpha, 2, 1);
}

void draw_status_imu(void)
{
    ips200_show_string(0, 0, "IMU");
    ips200_show_string(0, 24, "Roll");
    ips200_show_float(56, 24, eulerAngle.roll, 3, 2);
    ips200_show_string(0, 40, "Pitch");
    ips200_show_float(56, 40, eulerAngle.pitch, 3, 2);
    ips200_show_string(0, 56, "Yaw");
    ips200_show_float(56, 56, eulerAngle.yaw, 3, 2);
	
	ips200_show_string(0, 72, "GY1");
    ips200_show_float(56, 72, gyroscope[0], 3, 2);
    ips200_show_string(0, 88, "GY2");
    ips200_show_float(56, 88, gyroscope[1], 3, 2);
    ips200_show_string(0, 104, "GY3");
    ips200_show_float(56, 104, gyroscope[2], 3, 2);

	
	
    ips200_show_string(128, 24, "Gx0");
    ips200_show_float(176, 24, gyroscopeOffset[0], 2, 3);
    ips200_show_string(128, 40, "Gy0");
    ips200_show_float(176, 40, gyroscopeOffset[1], 2, 3);
    ips200_show_string(128, 56, "Gz0");
    ips200_show_float(176, 56, gyroscopeOffset[2], 2, 3);
	
	ips200_show_string(128, 72, "AC1");
    ips200_show_float(176, 72, accelerometer[0], 3, 2);
    ips200_show_string(128, 88, "AC2");
    ips200_show_float(176, 88, accelerometer[1], 3, 2);
    ips200_show_string(128, 104, "AC3");
    ips200_show_float(176, 104, accelerometer[2], 3, 2);
}

void draw_flash_status(void)
{
    switch (menu_rt.flash_result)
    {
        case FLASH_RESULT_SUCCESS:
            ips200_show_string(140, 16, "Flash OK");
            break;
        case FLASH_RESULT_FAIL:
            ips200_show_string(140, 16, "Flash Fail");
            break;
        case FLASH_RESULT_CLEARED:
            ips200_show_string(140, 16, "Flash Clear");
            break;
        default:
            ips200_show_string(140, 16, "Flash --");
            break;
    }
}

void draw_image_preview(void)
{
    image_show();

    ips200_show_string(0, 224, "line");
    ips200_show_uint(32, 224, num_line, 3);
    ips200_show_string(0, 240, "enc1");
    ips200_show_int(40, 240, up_L_all, 4);
    ips200_show_string(120, 240, "enc2");
    ips200_show_int(160, 240, up_R_all, 4);
    ips200_show_string(0, 256, "enc3");
    ips200_show_int(40, 256, down_L_all, 4);
    ips200_show_string(120, 256, "enc4");
    ips200_show_int(160, 256, down_R_all, 4);
    ips200_show_string(0, 272, "all");
    ips200_show_int(60, 272, all, 5);
}

void draw_display_mode(void)
{
    ips200_show_string(0, 176, "Mode:");
    if (display_mode == 0)
    {
        ips200_show_string(48, 176, "Debug (refresh)");
    }
    else
    {
        ips200_show_string(48, 176, "HighSpeed (freeze)");
    }
    ips200_show_string(0, 192, "Tips:");
    ips200_show_string(48, 192, "Debug: refresh on run");
    ips200_show_string(48, 208, "HighSpeed: freeze");
}
uint8 param_save_to_flash(void)
{
    flash_buffer_clear();

    if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))
    {
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    }

    flash_union_buffer[0].int32_type = straight_speed;
    flash_union_buffer[1].int32_type = turn_speed;
    flash_union_buffer[2].float_type = speed_k;
    flash_union_buffer[3].int32_type = speed_limit;
    flash_union_buffer[4].int32_type = yuanhuan_speed;
    flash_union_buffer[5].float_type = pid_world_x.fKp;
    flash_union_buffer[6].float_type = pid_world_x.fKi;
    flash_union_buffer[7].float_type = pid_world_x.fKd;

    flash_union_buffer[8].int32_type = start_accelerate;
    flash_union_buffer[9].int32_type = end_decelerate;
    flash_union_buffer[10].int32_type = slow_turn_speed;
    flash_union_buffer[11].int32_type = turn_num;

    flash_union_buffer[12].float_type = pid_world_y.fKp;
    flash_union_buffer[13].float_type = pid_world_y.fKi;
    flash_union_buffer[14].float_type = pid_world_y.fKd;
    flash_union_buffer[15].float_type = pid_yaw.fKp;
    flash_union_buffer[16].float_type = pid_yaw.fKi;
    flash_union_buffer[17].float_type = pid_yaw.fKd;
    flash_union_buffer[18].float_type = pid_accel_yaw.fKp;
    flash_union_buffer[19].float_type = pid_accel_yaw.fKi;
    flash_union_buffer[20].float_type = pid_accel_yaw.fKd;

    if (flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) != 0)
    {
        return 0;
    }
    return 1;
}

uint8 param_load_from_flash(void)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);

    if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) == 0)
    {
        return 0;
    }

    straight_speed = flash_union_buffer[0].int32_type;
    turn_speed = flash_union_buffer[1].int32_type;
    speed_k = flash_union_buffer[2].float_type;
    speed_limit = flash_union_buffer[3].int32_type;
    yuanhuan_speed = flash_union_buffer[4].int32_type;
    pid_world_x.fKp = flash_union_buffer[5].float_type;
    pid_world_x.fKi = flash_union_buffer[6].float_type;
    pid_world_x.fKd = flash_union_buffer[7].float_type;

    start_accelerate = flash_union_buffer[8].int32_type;
    end_decelerate = flash_union_buffer[9].int32_type;
    slow_turn_speed = flash_union_buffer[10].int32_type;
    turn_num = flash_union_buffer[11].int32_type;

    pid_world_y.fKp = flash_union_buffer[12].float_type;
    pid_world_y.fKi = flash_union_buffer[13].float_type;
    pid_world_y.fKd = flash_union_buffer[14].float_type;
    pid_yaw.fKp = flash_union_buffer[15].float_type;
    pid_yaw.fKi = flash_union_buffer[16].float_type;
    pid_yaw.fKd = flash_union_buffer[17].float_type;
    pid_accel_yaw.fKp = flash_union_buffer[18].float_type;
    pid_accel_yaw.fKi = flash_union_buffer[19].float_type;
    pid_accel_yaw.fKd = flash_union_buffer[20].float_type;

    return 1;
}

uint8 param_clear_flash(void)
{
    flash_buffer_clear();
    if (flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) != 0)
    {
        return 0;
    }
    return 1;
}
