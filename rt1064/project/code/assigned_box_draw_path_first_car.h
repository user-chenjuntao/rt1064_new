#ifndef ASSIGNED_BOX_DRAW_PATH_FIRST_CAR_H
#define ASSIGNED_BOX_DRAW_PATH_FIRST_CAR_H

#include "assigned_box_planner_greedy_3.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRAW_PATH_FIRST_CAR_STAGE 4

#define DRAW_PATH_FIRST_CAR_MAX_ITEMS 10
#define DRAW_PATH_FIRST_CAR_FULL_PATH_CAPACITY 10000
#define DRAW_PATH_FIRST_CAR_ERR_INVALID_ARG       601
#define DRAW_PATH_FIRST_CAR_ERR_TOO_MANY_ITEMS    602
#define DRAW_PATH_FIRST_CAR_ERR_GRID_TOO_LARGE    603
#define DRAW_PATH_FIRST_CAR_ERR_NO_CANDIDATE      604
#define DRAW_PATH_FIRST_CAR_ERR_UNREACHABLE_ITEM  605
#define DRAW_PATH_FIRST_CAR_ERR_PATH_OVERFLOW     606
#define DRAW_PATH_FIRST_CAR_ERR_ASTAR_FAILED      607
#define DRAW_PATH_FIRST_CAR_ERR_SPECIAL_PUSH_FAIL 608

extern PlannerAllBoxPaths draw_path_first_car_paths;
extern PlannerPointV3_BFS draw_path_first_car_full_path[DRAW_PATH_FIRST_CAR_FULL_PATH_CAPACITY];
extern size_t draw_path_first_car_full_steps;
extern size_t draw_path_first_car_visit_order[DRAW_PATH_FIRST_CAR_MAX_ITEMS];
extern size_t draw_path_first_car_visit_count;

void clear_draw_path_first_car_paths(void);

int build_draw_path_first_car_paths_v3(
    int rows, int cols, PlannerPointV3_BFS car,
    const PlannerPointV3_BFS *boxes, size_t box_count,
    const PlannerPointV3_BFS *targets, size_t target_count,
    const PlannerPointV3_BFS *bombs, size_t bomb_count,
    const PlannerPointV3_BFS *obstacles, size_t obstacle_count);

#ifdef __cplusplus
}
#endif

#endif
