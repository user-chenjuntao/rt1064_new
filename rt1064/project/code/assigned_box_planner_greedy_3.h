#ifndef ASSIGNED_BOX_PLANNER_GREEDY_3_H
#define ASSIGNED_BOX_PLANNER_GREEDY_3_H

#include <stddef.h>
#include <stdint.h>
#include "assigned_box_planner_greedy.h"
#include "assigned_box_planner_greedy_2.h"

#ifdef __cplusplus
extern "C" {
#endif

extern int last_err_stage;
extern int last_err_detail;

/* last_err_detail v3 stage-2 codes */
#define LAST_ERR_DETAIL_V3_BOX_TO_TARGET_INF                     101
#define LAST_ERR_DETAIL_V3_GOAL_COUNT_MISMATCH                   102
#define LAST_ERR_DETAIL_V3_SOURCE_BOX_NOT_FOUND                  103
#define LAST_ERR_DETAIL_V3_SOURCE_INDEX_INVALID                  104
#define LAST_ERR_DETAIL_V3_MANUAL_TARGET_INDEX_INVALID           105
#define LAST_ERR_DETAIL_V3_MANUAL_TARGET_DUPLICATED              106
#define LAST_ERR_DETAIL_V3_BOX_PATH_STEP_NOT_ADJACENT            107
#define LAST_ERR_DETAIL_V3_BOX_PATH_STEP_OUT_OF_BOUNDS           108
#define LAST_ERR_DETAIL_V3_BOX_OR_PUSH_ON_OBSTACLE               109
#define LAST_ERR_DETAIL_V3_BOX_OR_PUSH_COLLIDE_BOX               110
#define LAST_ERR_DETAIL_V3_BOX_INTERMEDIATE_UNREACHABLE          111
#define LAST_ERR_DETAIL_V3_CAR_CANNOT_REACH_PUSH_POS             112
#define LAST_ERR_DETAIL_V3_PUSH_EXEC_CAR_MISMATCH                113
#define LAST_ERR_DETAIL_V3_PUSH_EXEC_NOT_ADJACENT                114
#define LAST_ERR_DETAIL_V3_PUSH_EXEC_PATH_NOT_CONTINUOUS         115
#define LAST_ERR_DETAIL_V3_SCORE_NO_FEASIBLE_DIRECTION           116
#define LAST_ERR_DETAIL_V3_BOX_FINAL_NOT_AT_TARGET               117
#define LAST_ERR_DETAIL_V3_OVERALL_PATH_NOT_CONTINUOUS           118
#define LAST_ERR_DETAIL_V3_RETURN_PATH_NOT_CONTINUOUS            119
#define LAST_ERR_DETAIL_V3_BOUNDARY_PATH_INVALID                 120
#define LAST_ERR_DETAIL_V3_BOX_TARGET_BFS_FAILED                 121
#define LAST_ERR_DETAIL_V3_BOX_PATH_EXHAUSTED                    122
#define LAST_ERR_DETAIL_V3_SCORE_CAR_CANNOT_REACH_PUSH_POS       123
#define LAST_ERR_DETAIL_V3_PATH_ONLY_INVALID                     124
#define LAST_ERR_DETAIL_V3_SCORE_EXEC_NOT_ADJACENT               125
#define LAST_ERR_DETAIL_V3_SCORE_EXEC_PATH_NOT_CONTINUOUS        126
#define LAST_ERR_DETAIL_V3_SCORE_DIR_SELECTION_FAILED            127
#define LAST_ERR_DETAIL_V3_BOX_TARGET_DIST_BFS_FAILED            128
#define LAST_ERR_DETAIL_V3_BOX_PATH_ASTAR_FAILED                 129
#define LAST_ERR_DETAIL_V3_SCORE_EXEC_CAR_MISMATCH               130
#define LAST_ERR_DETAIL_V3_SPECIAL_PATH_FAILED                   131
#define LAST_ERR_DETAIL_V3_SPECIAL_PATH_BOX_REPLAN_FAILED        132
#define LAST_ERR_DETAIL_V3_SCORE_EXEC_STEPS_OVERFLOW             133
#define LAST_ERR_DETAIL_V3_PATH_AND_SCORE_BOTH_FAILED            134
#define LAST_ERR_DETAIL_V3_SELECTED_PATH_INVALID                 135
#define LAST_ERR_DETAIL_V3_SELECTED_SCORE_INVALID                136
#define LAST_ERR_DETAIL_V3_SPECIAL_PATH_COMBO_FAILED             137
#define LAST_ERR_DETAIL_V3_SPECIAL_PATH_PUSH_FAIL                138
#define LAST_ERR_DETAIL_V3_FIRST_PUSH_ASSIST_FAILED              139

/* last_err_detail v3 stage-3 codes */
#define LAST_ERR_DETAIL_V3_BOMB_PUSH_SAVE_PATH_OVERFLOW          300
#define LAST_ERR_DETAIL_V3_BOMB_PUSH_FAILED                      301
#define LAST_ERR_DETAIL_V3_BLOCKED_BOMB_REPLAN_FAILED            302
#define LAST_ERR_DETAIL_V3_BOMB_PUSH_TARGET_BFS_FAILED           310
#define LAST_ERR_DETAIL_V3_BOMB_PATH_AND_SCORE_BOTH_FAILED       311
#define LAST_ERR_DETAIL_V3_BOMB_PATH_EXHAUSTED                   312
#define LAST_ERR_DETAIL_V3_BOMB_PATH_STEP_NOT_ADJACENT           313
#define LAST_ERR_DETAIL_V3_BOMB_PATH_STEP_OUT_OF_BOUNDS          314
#define LAST_ERR_DETAIL_V3_BOMB_PATH_OBSTACLE_CONFLICT           315
#define LAST_ERR_DETAIL_V3_BOMB_PATH_BOX_CONFLICT                316
#define LAST_ERR_DETAIL_V3_BOMB_PATH_CAR_MOVE_FAILED             317
#define LAST_ERR_DETAIL_V3_BOMB_PATH_EXEC_CAR_MISMATCH           318
#define LAST_ERR_DETAIL_V3_BOMB_PATH_EXEC_NOT_ADJACENT           319
#define LAST_ERR_DETAIL_V3_BOMB_PATH_EXEC_NOT_CONTINUOUS         320
#define LAST_ERR_DETAIL_V3_BOMB_SCORE_NO_FEASIBLE_DIRECTION      321
#define LAST_ERR_DETAIL_V3_BOMB_SCORE_CAR_MOVE_FAILED            322
#define LAST_ERR_DETAIL_V3_BOMB_SCORE_EXEC_CAR_MISMATCH          323
#define LAST_ERR_DETAIL_V3_BOMB_SCORE_EXEC_NOT_ADJACENT          324
#define LAST_ERR_DETAIL_V3_BOMB_SCORE_EXEC_NOT_CONTINUOUS        325
#define LAST_ERR_DETAIL_V3_BOMB_FINAL_NOT_AT_TARGET              326
#define LAST_ERR_DETAIL_V3_BOMB_BOUNDARY_PATH_INVALID            327
#define LAST_ERR_DETAIL_V3_BOMB_PATH_ONLY_INVALID                328
#define LAST_ERR_DETAIL_V3_BOMB_SCORE_DIR_SELECTION_FAILED       329
/* ??last_err_detail == LAST_ERR_DETAIL_V3_SPECIAL_PATH_FAILED ???????????????????? */
extern int last_special_path_fail_reason;
extern int last_first_push_bomb_special_fail_reason;

/*
 * ??/??????????BFS+A* ???? * 0: ????????????????????????? * 1: ????BFS+A* ?????????????? */
extern int planner_v3_push_only_bfs_astar_path;

/* ?????????????????????????*/
extern PlannerAllBoxPaths special_paths;

/* Requirement bitmask for manual-assignment per-box control */
#define PLANNER_V3_REQ_MASK_REQ1 0x01u
#define PLANNER_V3_REQ_MASK_REQ2 0x02u
#define PLANNER_V3_REQ_MASK_REQ3 0x04u
#define PLANNER_V3_REQ_MASK_ALL  (PLANNER_V3_REQ_MASK_REQ1 | PLANNER_V3_REQ_MASK_REQ2 | PLANNER_V3_REQ_MASK_REQ3)
#define PLANNER_V3_RETURN_TO_START_DISABLE 0
#define PLANNER_V3_RETURN_TO_START_ENABLE  1

/* include_return_to_start_path:
 *   1 -> append the path from final car pose back to the initial car pose.
 *   0 -> do not append that return path segment.
 */
int plan_boxes_greedy_v3_with_return_control(int rows, int cols, PlannerPointV3_BFS car,
                         const PlannerPointV3_BFS *boxes, size_t box_count,
                         const PlannerPointV3_BFS *targets, size_t target_count,
                         const PlannerPointV3_BFS *bombs, size_t bomb_count,
                         const PlannerPointV3_BFS *obstacles,
                         size_t obstacle_count, PlannerPointV3_BFS *path_buffer,
                         size_t path_capacity, size_t *out_steps,
                         int include_return_to_start_path,
                         size_t *out_box_target_indices,
                         PlannerAllBoxPaths *out_final_paths);

/**
 * ?????? v3????????????
 *
 * @param rows/cols                 ?????
 * @param car                       ????
 * @param boxes/box_count           ???????
 * @param targets/target_count      ????????
 * @param bombs/bomb_count          ???????
 * @param obstacles/obstacle_count  ????????
 * @param path_buffer/path_capacity ???????????
 * @param out_steps                 ?????
 * @param out_box_target_indices    ?????????????? NULL?
 * @param out_final_paths           ?????????????? NULL?
 *
 * ???
 * - ??????????? path_buffer == NULL ? path_capacity == 0 ??
 *   ??????????? out_steps??????????
 *
 * ??????????????????
 * @return 0
 *   ?????? path_buffer == NULL????? out_steps ???
 * @return -1
 *   ???????????????????
 *   - ???????boxes / targets / out_steps ??
 *   - target_count ???????????????????????
 * @return -2
 *   ????????????
 *   - box_count == 0 ? target_count == 0
 *   - ??? row/col < 0 ?????????????? 0
 * @return -3
 *   ?????box_count ????????? v3 ???? 5 ????????
 * @return -4
 *   path_buffer ??? path_capacity == 0??????????????????
 * @return -6
 *   ??????????????????????????? / ??????????
 *   ????????????????
 *   - last_err_stage??????????2=?????&?????3=???&????????
 *   - last_err_detail?????????
 *   - ? last_err_detail == LAST_ERR_DETAIL_V3_SPECIAL_PATH_FAILED???????????? last_special_path_fail_reason
 * @return -7
 *   ??????????path_capacity ??????????
 *   ????????????????????
 * @return -8
 *   ?????????????target_count < usable_boxes??
 *   ??????????????
 * @return -9
 *   ???????? MAX_BOMBS?????? 5?
 * @return -10
 *   ????????????????????? out_steps / out_final_paths ????????
 */
int plan_boxes_greedy_v3(int rows, int cols, PlannerPointV3_BFS car,
                         const PlannerPointV3_BFS *boxes, size_t box_count,
                         const PlannerPointV3_BFS *targets, size_t target_count,
                         const PlannerPointV3_BFS *bombs, size_t bomb_count,
                         const PlannerPointV3_BFS *obstacles,
                         size_t obstacle_count, PlannerPointV3_BFS *path_buffer,
                         size_t path_capacity, size_t *out_steps,
                         size_t *out_box_target_indices,
                         PlannerAllBoxPaths *out_final_paths);

/**
 * ?????? v3?????????????? *
 * ??plan_boxes_greedy_v3 ??????????? * @param box_target_indices
 *   box_target_indices[????] = ?????? *   ?????????????????? [0, target_count)?? *
 * ???? * - ????????path_buffer == NULL ??path_capacity == 0?? *
 * ????????????????
 * @return 0
 *   ?????? path_buffer == NULL??????out_steps ????? * @return -1
 *   ???????????????????? *   - ???????boxes/targets/out_steps/box_target_indices ??
 * @return -2
 *   ????????????
 *   - box_count == 0 ??target_count == 0
 *   - ????????row/col < 0 ?????????????? 0
 * @return -3
 *   ?????box_count ??????????v3 ?????5 ?????????? * @return -4
 *   path_buffer ????path_capacity == 0?? * @return -6
 *   ????????????????????????? *   ???????? last_err_stage / last_err_detail?????? last_special_path_fail_reason?? * @return -7
 *   ??????????path_capacity ????? * @return -9
 *   ???????? MAX_BOMBS?????? 5??? * @return -10
 *   ??????????????????????out_steps / out_final_paths??? * @return -11
 *   ???????? *   - ???????box_target_indices[i] >= target_count?? *   - ????????????????????
 */
/* box_req_masks: optional per-box requirement mask (length=box_count).
 * When non-NULL, each box uses the exact mask and internal downgrade is disabled. */
int plan_boxes_greedy_v3_manual_assignment(int rows, int cols, PlannerPointV3_BFS car,
                                           const PlannerPointV3_BFS *boxes, size_t box_count,
                                           const PlannerPointV3_BFS *targets, size_t target_count,
                                           const PlannerPointV3_BFS *bombs, size_t bomb_count,
                                           const PlannerPointV3_BFS *obstacles,
                                           size_t obstacle_count,
                                           const size_t *box_target_indices,
                                           const uint8_t *box_req_masks,
                                           PlannerPointV3_BFS *path_buffer,
                                           size_t path_capacity, size_t *out_steps,
                                           size_t *out_box_target_indices,
                                           PlannerAllBoxPaths *out_final_paths);

int plan_boxes_greedy_v3_manual_assignment_with_return_control(int rows, int cols, PlannerPointV3_BFS car,
                                           const PlannerPointV3_BFS *boxes, size_t box_count,
                                           const PlannerPointV3_BFS *targets, size_t target_count,
                                           const PlannerPointV3_BFS *bombs, size_t bomb_count,
                                           const PlannerPointV3_BFS *obstacles,
                                           size_t obstacle_count,
                                           const size_t *box_target_indices,
                                           const uint8_t *box_req_masks,
                                           PlannerPointV3_BFS *path_buffer,
                                           size_t path_capacity, size_t *out_steps,
                                           int include_return_to_start_path,
                                           size_t *out_box_target_indices,
                                           PlannerAllBoxPaths *out_final_paths);

#ifdef __cplusplus
}
#endif

#endif
