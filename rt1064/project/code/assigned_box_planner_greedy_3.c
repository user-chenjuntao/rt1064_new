/*********************************************************************************************************************
* 文件名称 : assigned_box_planner_greedy_3.c
* 功能概述 : 基于 BFS / A* / 特殊路径策略的炸弹推箱规划 V3 实现
*
* 主要特性 :
*   1. 支持多箱、多目标、多炸弹与障碍物的综合路径规划
*   2. 通过贪心策略进行箱子→目标的自动分配（也支持手动映射）
*   3. 使用 BFS 预计算距离图，并结合 A* 进行小车及箱子（炸弹）路径搜索
*   4. 内置多种“特殊路径”和回滚机制，以提升在复杂场景下的成功率与可解释性
*********************************************************************************************************************/
/*********************************************************************************************************************
* 旧版本头注释（保留以兼容历史，建议阅读上方新的中文说明）
*********************************************************************************************************************/
#include "assigned_box_planner_greedy_3.h"
#include "assigned_box_planner_greedy_2.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

// 最近一次规划失败的大阶段与细分错误码（在头文件中有详细说明）
extern int last_err_stage;
extern int last_err_detail;
extern int last_err_detail_v3_stage3;

// 是否在推箱/推炸弹时仅使用 BFS + A* 路径；默认 0 表示多策略综合评估
int planner_v3_push_only_bfs_astar_path = 0;

// 规划算法使用的规模上限：最多箱子数、网格总格子数、贪心推箱最大步数
#define PLANNER_V3_BFS_MAX_BOXES 5
#define PLANNER_V3_BFS_MAX_CELLS 140
#define PLANNER_V3_BFS_MAX_GREEDY_STEPS 250
// A* 中 f=g+h 的桶最大索引（用于简单桶队列实现）
#define PLANNER_V3_ASTAR_F_BUCKET_MAX (PLANNER_V3_BFS_MAX_CELLS * 2 + 5)

// 占用网格中的比特标记
#define OCC_OBSTACLE      0x01
#define OCC_BOMB          0x02
#define OCC_BLOCKED_BOMB  0x04
#define OCC_BOX           0x08
#define OCC_BOX_INDEX_MASK 0xF0
#define OCC_BOX_INDEX_SHIFT 4
#define OCC_NO_BOX_INDEX  15
#define MAX_BOMBS 5          // 支持的最大炸弹数量

#define PLANNER_V3_BFS_FORBIDDEN_EDGE_BITS  (PLANNER_V3_BFS_MAX_CELLS * 4)
#define PLANNER_V3_BFS_FORBIDDEN_EDGE_WORDS ((PLANNER_V3_BFS_FORBIDDEN_EDGE_BITS + 31) / 32)

static inline int planner_v3_bfs_forbidden_edge_test(const uint32_t *bits, int edge_idx) {
  if (!bits || edge_idx < 0 || edge_idx >= PLANNER_V3_BFS_FORBIDDEN_EDGE_BITS) {
    return 0;
  }
  int word_idx = edge_idx >> 5;
  int bit_idx = edge_idx & 31;
  return (bits[word_idx] & (1u << bit_idx)) != 0;
}

static inline void planner_v3_bfs_forbidden_edge_set(uint32_t *bits, int edge_idx) {
  if (!bits || edge_idx < 0 || edge_idx >= PLANNER_V3_BFS_FORBIDDEN_EDGE_BITS) {
    return;
  }
  int word_idx = edge_idx >> 5;
  int bit_idx = edge_idx & 31;
  bits[word_idx] |= (1u << bit_idx);
}

/* Optional O(1) forbidden-edge bitset used by special-path A* replanning. */
static const uint32_t *s_astar_forbidden_edge_bits = NULL;


static Point s_blocked_bombs[MAX_BOMBS];
static size_t s_blocked_bomb_count = 0;


static uint32_t s_bfs_visit_stamp = 0;
static uint32_t s_bfs_visit_mark[PLANNER_V3_BFS_MAX_CELLS];
static uint32_t s_astar_run_stamp = 0;
static uint32_t s_astar_mark[PLANNER_V3_BFS_MAX_CELLS];

typedef struct PlannerV3BfsScratch {
  uint8_t occ_buf[PLANNER_V3_BFS_MAX_CELLS];
  int queue[PLANNER_V3_BFS_MAX_CELLS];
} PlannerV3BfsScratch;
/* Shared BFS scratch for helper-level BFS routines. Not re-entrant. */
static PlannerV3BfsScratch s_bfs_scratch;

/* A* scratch workspace: avoid re-allocating large 400-scale arrays on stack.
 * Note: planner_v3_bfs_astar_with_dist is not re-entrant with this workspace. */
typedef struct PlannerV3AstarScratch {
  uint8_t occ_buf[PLANNER_V3_BFS_MAX_CELLS];
  int g_score[PLANNER_V3_BFS_MAX_CELLS];
  int parent[PLANNER_V3_BFS_MAX_CELLS];
  int f_score[PLANNER_V3_BFS_MAX_CELLS];
  int car_to_push_best[PLANNER_V3_BFS_MAX_CELLS];
  int bucket_head[PLANNER_V3_ASTAR_F_BUCKET_MAX];
  int bucket_next[PLANNER_V3_BFS_MAX_CELLS];
  int path_indices[PLANNER_V3_BFS_MAX_CELLS];
} PlannerV3AstarScratch;
static PlannerV3AstarScratch s_astar_scratch;

typedef struct PlannerV3PlanLoopScratch {
  Point saved_obstacles_a[200];
  Point saved_obstacles_b[200];
  int target_dist[PLANNER_V3_BFS_MAX_CELLS];
  Point box_path[PLANNER_V3_BFS_MAX_CELLS];
  int box_path_dist[PLANNER_V3_BFS_MAX_CELLS];
} PlannerV3PlanLoopScratch;
static PlannerV3PlanLoopScratch s_plan_loop_scratch;

typedef struct PlannerV3ProbeScratch {
  Point temp_obstacles[200];
  Point temp_bombs[MAX_BOMBS];
  Point temp_path[PLANNER_V3_BFS_MAX_CELLS];
} PlannerV3ProbeScratch;
/* Shared probe workspace for high-frequency feasibility checks. Not re-entrant. */
static PlannerV3ProbeScratch s_probe_scratch;

// 构建给定状态下的占用网格：障碍物 / 炸弹 / 已阻塞炸弹 / 箱子 等
static void planner_v3_bfs_build_occupancy_grid(int rows, int cols,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count,
    int use_blocked_bombs,
    uint8_t *grid);
static int planner_v3_bfs_adjacent_blockers_occ(const uint8_t *occ, int rows, int cols,
    size_t skip_idx, int row, int col);
static int planner_v3_bfs_is_deadlock_occ(const uint8_t *occ, int rows, int cols,
    size_t skip_idx, int row, int col);

// 根据当前障碍 / 炸弹 / 箱子信息构建占用网格
static void planner_v3_bfs_build_occupancy_grid(int rows, int cols,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count,
    int use_blocked_bombs,
    uint8_t *grid) {
  size_t n = (size_t)(rows * cols);
  if (n > PLANNER_V3_BFS_MAX_CELLS) n = PLANNER_V3_BFS_MAX_CELLS;
  memset(grid, 0, n);
  for (size_t i = 0; i < obstacle_count; ++i) {
    int r = obstacles[i].row, c = obstacles[i].col;
    if (r >= 0 && r < rows && c >= 0 && c < cols)
      grid[(size_t)(r * cols + c)] |= OCC_OBSTACLE;
  }
  for (size_t i = 0; i < bomb_count; ++i) {
    int r = bombs[i].row, c = bombs[i].col;
    if (r >= 0 && r < rows && c >= 0 && c < cols)
      grid[(size_t)(r * cols + c)] |= OCC_BOMB;
  }
  if (use_blocked_bombs) {
    for (size_t i = 0; i < s_blocked_bomb_count; ++i) {
      int r = s_blocked_bombs[i].row, c = s_blocked_bombs[i].col;
      if (r >= 0 && r < rows && c >= 0 && c < cols)
        grid[(size_t)(r * cols + c)] |= OCC_BLOCKED_BOMB;
    }
  }
  for (size_t i = 0; i < box_count; ++i) {
    int r = boxes[i].row, c = boxes[i].col;
    if (r < 0 || c < 0) continue;
    if (r >= 0 && r < rows && c >= 0 && c < cols) {
      size_t idx = (size_t)(r * cols + c);
      grid[idx] |= OCC_BOX;
      grid[idx] = (grid[idx] & 0x0F) | (uint8_t)((i <= (size_t)OCC_NO_BOX_INDEX ? i : OCC_NO_BOX_INDEX) << OCC_BOX_INDEX_SHIFT);
    }
  }
}

#define OCC_IS_OBSTACLE(grid, cols, row, col) \
  (((grid)[(size_t)((row) * (cols) + (col))] & (OCC_OBSTACLE | OCC_BOMB | OCC_BLOCKED_BOMB)) != 0)
#define OCC_IS_OBSTACLE_NO_BOMB(grid, cols, row, col) \
  (((grid)[(size_t)((row) * (cols) + (col))] & (OCC_OBSTACLE | OCC_BLOCKED_BOMB)) != 0)
#define OCC_HAS_BOX(grid, cols, row, col) \
  (((grid)[(size_t)((row) * (cols) + (col))] & OCC_BOX) != 0)
#define OCC_HAS_BOX_EXCLUDING(grid, cols, row, col, skip_idx) \
  (OCC_HAS_BOX_EXCLUDING_IMPL((grid), (cols), (row), (col), (skip_idx)))
static inline int OCC_HAS_BOX_EXCLUDING_IMPL(const uint8_t *g, int cols, int row, int col, size_t skip_idx) {
  size_t idx = (size_t)(row * cols + col);
  uint8_t cell = g[idx];
  if (!(cell & OCC_BOX)) return 0;
  if (skip_idx == SIZE_MAX) return 1;
  return (size_t)(cell >> OCC_BOX_INDEX_SHIFT) != skip_idx;
}
#define PLANNER_V3_BFS_MAX_PATH_LEN 400
#define PLANNER_V3_BFS_STRATEGY_PATH 1
#define PLANNER_V3_BFS_STRATEGY_SCORE 2
#define PUSH_BOMB_SAVE_PATH_MAX 400
#define PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS 9

// 特殊路径失败时的细分错误码（当 last_err_detail == LAST_ERR_DETAIL_V3_SPECIAL_PATH_FAILED 时使用）

#define SPECIAL_PATH_FAIL_NONE 400
#define SPECIAL_PATH_FAIL_REQ1 401
#define SPECIAL_PATH_FAIL_REQ2 402
#define SPECIAL_PATH_FAIL_REQ3 403
#define SPECIAL_PATH_FAIL_SP_INVALID_GRID 411
#define SPECIAL_PATH_FAIL_SP_START_OR_TARGET_OOB 412
#define SPECIAL_PATH_FAIL_SP_INITIAL_BFS_FAILED 413
#define SPECIAL_PATH_FAIL_SP_INITIAL_ASTAR_FAILED 414
#define SPECIAL_PATH_FAIL_SP_INITIAL_PATH_EMPTY 415
#define SPECIAL_PATH_FAIL_SP_MULTI_REPLAN_NON_DESTROYABLE 421
#define SPECIAL_PATH_FAIL_SP_MULTI_REPLAN_PUSH_INVALID 422
#define SPECIAL_PATH_FAIL_SP_MULTI_REPLAN_BOMB_UNREACHABLE 423
#define SPECIAL_PATH_FAIL_SP_MULTI_INVALID_FORBIDDEN_EDGE 424
#define SPECIAL_PATH_FAIL_SP_MULTI_ADD_FORBIDDEN_FAILED 425
#define SPECIAL_PATH_FAIL_SP_MULTI_REPLAN_REACH_FAILED 426
#define SPECIAL_PATH_FAIL_SP_SINGLE_REPLAN_PUSH_INVALID 431
#define SPECIAL_PATH_FAIL_SP_ZERO_NO_FAIL_STEP 432
#define SPECIAL_PATH_FAIL_SP_ZERO_ADD_FORBIDDEN_FAILED 433
#define SPECIAL_PATH_FAIL_SP_ZERO_REPLAN_FORBIDDEN_FAILED 434
#define SPECIAL_PATH_FAIL_SP_SINGLE_REPLAN_NO_BOMB 435
#define SPECIAL_PATH_FAIL_SP_FINAL_NO_FAIL_POINT 436
#define SPECIAL_PATH_FAIL_SP_FINAL_INVALID_FORBIDDEN_EDGE 437
#define SPECIAL_PATH_FAIL_SP_FINAL_ADD_FORBIDDEN_FAILED 438
#define SPECIAL_PATH_FAIL_SP_FINAL_REPLAN_FORBIDDEN_FAILED 439
#define SPECIAL_PATH_FAIL_SP_MAX_ATTEMPTS_EXCEEDED 440
#define SPECIAL_PATH_FAIL_SP_REQ_MASK_EXHAUSTED 451
#define SPECIAL_PATH_FAIL_SP_NO_REQ_RELAX_MATCH 452
#define SPECIAL_PATH_FAIL_SP_REPLAN_AFTER_BLOCKED_BOMB_FAILED 453

/* planner_v3_bfs_resolve_first_push_by_bomb_special_path() 的子失败原因枚举 */
#define FIRST_PUSH_BOMB_FAIL_NONE 0
#define FIRST_PUSH_BOMB_FAIL_NO_BOMB_CANDIDATE 501
#define FIRST_PUSH_BOMB_FAIL_SAVE_PATH_OVERFLOW 502
#define FIRST_PUSH_BOMB_FAIL_SPECIAL_PATH_PLAN_FAILED 503
#define FIRST_PUSH_BOMB_FAIL_PUSH_OBS_NOT_ALLOWED 504
#define FIRST_PUSH_BOMB_FAIL_CLEAR_PUSH_OBS_FAILED 505
#define FIRST_PUSH_BOMB_FAIL_PUSH_BOMB_FAILED 506
#define FIRST_PUSH_BOMB_FAIL_PUSH_STEP_FAILED 507
#define FIRST_PUSH_BOMB_FAIL_ALL_ATTEMPTS_FAILED 508
#define FIRST_PUSH_BOMB_FAIL_PATH_BUFFER_OVERFLOW 509

// 大代价值常量（用于不可达场景的兜底比较）
#define PLANNER_V3_LARGE_COST 1000000

/* Planned bomb usage purpose (smaller priority value means higher priority). */
#define PLANNER_V3_BOMB_PURPOSE_NONE            0
#define PLANNER_V3_BOMB_PURPOSE_PATH_OBSTACLE   1
#define PLANNER_V3_BOMB_PURPOSE_BOMB_A_PUSH_POS 2
#define PLANNER_V3_BOMB_PURPOSE_FIRST_PUSH_POS  3
#define PLANNER_V3_BOMB_PURPOSE_OPTIMIZE_PATH   4



PlannerAllBoxPaths special_paths = {0};

// 用于记录“特殊路径”相关的临时障碍（例如需要清除的障碍物位置）
#define PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES 200
static Point s_special_path_obstacles[PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES];
static size_t s_special_path_obstacle_count = 0;
static int s_last_failed_bomb_action_index = -1;
static size_t s_last_failed_bomb_action_bomb_idx = SIZE_MAX;
static Point s_last_failed_bomb_action_target = {-1, -1};

static inline void planner_v3_bfs_record_stage3_detail_if_needed(int stage, int detail) {
  if (stage == 3) {
    last_err_detail_v3_stage3 = detail;
  }
}

/* Used when wrapping detailed bomb-push failure into LAST_ERR_DETAIL_V3_BOMB_PUSH_FAILED. */
static inline int planner_v3_bfs_pick_bomb_push_subreason(void) {
  if (last_err_stage == 3 &&
      last_err_detail != LAST_ERR_DETAIL_V3_BOMB_PUSH_FAILED &&
      last_err_detail >= LAST_ERR_DETAIL_V3_BOMB_PUSH_SAVE_PATH_OVERFLOW) {
    return last_err_detail;
  }
  if (last_err_detail_v3_stage3 != LAST_ERR_DETAIL_NONE &&
      last_err_detail_v3_stage3 != LAST_ERR_DETAIL_V3_BOMB_PUSH_FAILED) {
    return last_err_detail_v3_stage3;
  }
  return LAST_ERR_DETAIL_NONE;
}

// 清空上一轮的特殊路径记录，并根据当前箱子数量初始化结构
static void planner_v3_bfs_clear_special_paths(const PlannerPointV3_BFS *boxes,
    const PlannerPointV3_BFS *targets, size_t box_count, size_t target_count) {
  (void)boxes;
  (void)targets;
  (void)target_count;
  s_special_path_obstacle_count = 0;
  special_paths.box_count = box_count;
  for (size_t i = 0; i < box_count && i < PLANNER_V3_BFS_MAX_BOXES; ++i) {
    memset(&special_paths.box_paths[i], 0, sizeof(PlannerBoxPathOutput));
    special_paths.box_paths[i].planned_order = -1;
  }
}

static void planner_v3_bfs_find_bombs_on_path(int rows, int cols,
                                               const Point *path, size_t path_len,
                                               const Point *bombs, size_t bomb_count,
                                               size_t *bombs_on_path, size_t *bombs_on_path_count);

typedef struct {
  Point box;
  Point target;
  size_t target_idx;
  size_t source_idx;
  Point obstacle_to_clear;
  int has_obstacle_to_clear;
} PlannerBoxGoalBFS;

typedef struct {
  size_t bomb_idx;
  Point bomb_target;
  uint8_t bomb_purpose;
  Point exploded_obstacles[PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES];
  size_t exploded_obstacle_count;
} PlannerV3BombAction;

typedef struct {
  int reachable;
  int special_path_valid;
  Point special_path[PLANNER_V3_BFS_MAX_PATH_LEN];
  size_t special_path_len;
  int has_obstacle_to_clear;
  Point obstacle_to_clear;
  size_t bomb_action_count;
  PlannerV3BombAction bomb_actions[MAX_BOMBS];
  size_t used_bomb_indices[MAX_BOMBS];
  uint8_t used_bomb_purposes[MAX_BOMBS];
  Point used_bomb_targets[MAX_BOMBS];
  size_t used_bomb_count;
  Point exploded_obstacles[PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES];
  size_t exploded_obstacle_count;
} PlannerV3PreBoxPlan;

typedef struct {
  int valid;
  size_t bomb_a_idx;
  Point bomb_a_target;
  size_t bomb_b_idx;
  Point bomb_b_target;
  Point bomb_b_exploded_obstacles[PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES];
  size_t bomb_b_exploded_obstacle_count;
} PlannerV3SpecialSingleBombHint;

static PlannerV3SpecialSingleBombHint s_last_special_single_bomb_hint = {0};

typedef struct {
  PlannerBoxGoalBFS goals[PLANNER_V3_BFS_MAX_BOXES];
  size_t goal_count;
  size_t ordered_goal_indices[PLANNER_V3_BFS_MAX_BOXES];
  size_t ordered_goal_count;
  int normal_reachable_by_box[PLANNER_V3_BFS_MAX_BOXES];
  PlannerV3PreBoxPlan preplans[PLANNER_V3_BFS_MAX_BOXES];
  uint8_t banned_bomb_by_box[PLANNER_V3_BFS_MAX_BOXES][MAX_BOMBS];
  uint8_t yielded_bomb_by_box[PLANNER_V3_BFS_MAX_BOXES][MAX_BOMBS];
  PlannerV3TwoPhasePreplan two_phase;
} PlannerV3TwoPhaseRuntime;

static int planner_v3_bfs_bomb_purpose_priority(uint8_t purpose);
static void planner_v3_bfs_add_or_update_bomb_use(PlannerV3PreBoxPlan *plan,
                                                  size_t bomb_idx,
                                                  uint8_t purpose,
                                                  Point target_pos);
static void planner_v3_bfs_clear_bomb_actions(PlannerV3PreBoxPlan *plan);
static void planner_v3_bfs_append_bomb_action(
    PlannerV3PreBoxPlan *plan,
    size_t bomb_idx,
    Point bomb_target,
    uint8_t purpose,
    const Point *exploded_obstacles,
    size_t exploded_obstacle_count);
static void planner_v3_bfs_sync_bomb_summary_from_actions(PlannerV3PreBoxPlan *plan);
static void planner_v3_bfs_refresh_plan_exploded_obstacles_from_actions(
    PlannerV3PreBoxPlan *plan,
    const Point *planning_obstacles, size_t planning_obstacle_count);
static int build_assigned_goals(
    int rows, int cols, Point car,
    const Point *boxes, size_t box_count,
    const Point *targets, size_t target_count,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const size_t *manual_box_target_indices,
    PlannerBoxGoalBFS *out_goals,
    size_t *out_goal_count,
    size_t *out_mapping);
static int build_push_order(
    int rows, int cols, Point car,
    const PlannerBoxGoalBFS *goals, size_t goal_count, size_t box_count,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    PlannerV3TwoPhaseRuntime *runtime);
static void build_box_reachability(
    int rows, int cols,
    const PlannerBoxGoalBFS *goals, size_t goal_count, size_t box_count,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    Point car,
    PlannerV3TwoPhaseRuntime *runtime);
static void build_special_preplans(
    int rows, int cols,
    const PlannerBoxGoalBFS *goals, size_t goal_count, size_t box_count,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    Point car,
    size_t start_order,
    PlannerV3TwoPhaseRuntime *runtime);
static void resolve_bomb_conflicts_and_replan(
    int rows, int cols,
    const PlannerBoxGoalBFS *goals, size_t goal_count, size_t box_count,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    Point car,
    PlannerV3TwoPhaseRuntime *runtime);
static int execute_final_plan(
    int rows, int cols, Point car,
    const PlannerBoxGoalBFS *goals, size_t goal_count, size_t box_count,
    const Point *obstacles, size_t obstacle_count,
    Point *bombs_mutable, size_t bomb_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps,
    int include_return_to_start_path,
    PlannerV3TwoPhaseRuntime *runtime,
    PlannerAllBoxPaths *out_final_paths);
static int planner_v3_bfs_execute_single_bomb_action(
    int rows, int cols,
    Point *car_pos,
    Point *current_obstacles, size_t *current_obstacle_count,
    Point *bombs_mutable, size_t bomb_count,
    const Point *current_boxes, size_t box_count,
    const PlannerV3BombAction *action,
    size_t action_index,
    Point *path_buffer, size_t path_capacity, size_t *out_steps);
static int planner_v3_bfs_execute_planned_bomb_actions(
    int rows, int cols,
    Point *car_pos,
    Point *current_obstacles, size_t *current_obstacle_count,
    Point *bombs_mutable, size_t bomb_count,
    const Point *current_boxes, size_t box_count,
    const PlannerV3BombAction *actions, size_t action_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps);
static void planner_v3_bfs_sync_two_phase_from_preplans(
    const PlannerBoxGoalBFS *goals, size_t goal_count,
    const Point *bombs, size_t bomb_count,
    PlannerV3TwoPhaseRuntime *runtime);
static void planner_v3_bfs_publish_two_phase_to_special_paths(
    int rows, int cols,
    const Point *bombs, size_t bomb_count,
    PlannerV3TwoPhaseRuntime *runtime);
static int planner_v3_bfs_apply_bomb_conflict_yield_with_earliest(
    int rows, int cols,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const PlannerBoxGoalBFS *goals,
    const size_t *ordered_goal_indices, size_t ordered_count,
    const PlannerV3PreBoxPlan *plans,
    uint8_t banned_bomb_by_box[PLANNER_V3_BFS_MAX_BOXES][MAX_BOMBS],
    uint8_t yielded_bomb_by_box[PLANNER_V3_BFS_MAX_BOXES][MAX_BOMBS],
    size_t *out_earliest_order);
static int planner_v3_bfs_execute_box_path_fixed(
    int rows, int cols,
    Point *car_pos,
    Point *current_boxes, size_t box_count, size_t box_idx,
    Point target,
    const Point *current_obstacles, size_t current_obstacle_count,
    Point *bombs_mutable, size_t bomb_count,
    const Point *box_path, size_t box_path_len,
    Point *path_buffer, size_t path_capacity, size_t *out_steps,
    Point *box_path_trace, size_t *box_path_trace_len);
static int planner_v3_bfs_find_ordered_goals(int rows, int cols, Point car,
    const PlannerBoxGoalBFS *goals, size_t goal_count, size_t box_count,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    size_t *ordered_goal_indices, size_t *ordered_count);
static int planner_v3_bfs_special_path_planning(
    int rows, int cols, Point box_start, Point target,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count,
    size_t moving_idx,
    Point car_pos,
    Point *obstacle_passed, int *has_obstacle,
    Point *out_path, size_t out_path_cap,
    size_t *out_path_len,
    int apply_req1, int apply_req3);

typedef struct {
  int dr;
  int dc;
  int dist;
  int adj_pen;
  int score;
  int feasible;
  Point push_from;
  Point next_pos;
} PlannerV3PushScoreCandidate;

// 使用炸弹推动并清理障碍的核心 BFS/A* 逻辑
static int planner_v3_bfs_push_bomb(int rows, int cols, Point *car_pos,
                                    Point *bombs, size_t bomb_count, size_t bomb_idx,
                                    Point target_pos,
                                    Point target_obstacle,
                                    const Point *obstacles, size_t obstacle_count,
                                    const Point *boxes, size_t box_count,
                                    Point *path_buffer, size_t path_capacity,
                                    size_t *out_steps,
                                    const Point *path_obstacles_to_exclude, size_t path_obstacle_exclude_count);

static size_t planner_v3_bfs_bomb_explode(const Point *obstacles, size_t obstacle_count,
                                          Point bomb_pos, Point *out_obstacles, size_t out_cap);

static void planner_v3_bfs_bombs_sorted_by_dist_to_obstacle(int rows, int cols,
    Point obstacle_to_clear,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count,
    size_t *bomb_order, size_t *bomb_order_count,
    const Point *path_obstacles, size_t path_obstacle_count);

static void planner_v3_bfs_targets_sorted_by_reachable_dist(int rows, int cols, Point car,
    size_t bomb_idx, const Point *bombs, size_t bomb_count,
    Point obstacle_to_clear,
    const Point *obstacles, size_t obstacle_count,
    const Point *boxes, size_t box_count,
    Point *target_positions, size_t *target_count,
    const Point *path_obstacles, size_t path_obstacle_count,
    int apply_req1);

static int planner_v3_bfs_evaluate_push_candidates_score(
    int rows, int cols,
    Point moving_pos, Point target_pos,
    size_t moving_skip_idx,
    const uint8_t *occ,
    const int dist_to_target[PLANNER_V3_BFS_MAX_CELLS],
    const int dist_from_car[PLANNER_V3_BFS_MAX_CELLS],
    int allow_first_push_car_unreachable,
    int last_dr, int last_dc, int reverse_count,
    PlannerV3PushScoreCandidate candidates[4],
    int sorted_dirs[4], int *sorted_count);

// 一些轻量级的数学/工具函数（避免依赖 <math.h>）
static int planner_v3_bfs_abs(int v) { return v >= 0 ? v : -v; }

static int planner_v3_bfs_in_bounds(int rows, int cols, int row, int col) {
  return row >= 0 && row < rows && col >= 0 && col < cols;
}

static int planner_v3_bfs_point_in_list(Point p, const Point *list, size_t count) {
  if (!list || count == 0) {
    return 0;
  }
  for (size_t i = 0; i < count; ++i) {
    if (list[i].row == p.row && list[i].col == p.col) {
      return 1;
    }
  }
  return 0;
}

static int planner_v3_bfs_is_in_bomb_range_3x3(Point center, Point p) {
  return planner_v3_bfs_abs(p.row - center.row) <= 1 &&
         planner_v3_bfs_abs(p.col - center.col) <= 1;
}

static size_t planner_v3_bfs_collect_bomb_centers_covering_obstacle(
    int rows, int cols, Point obstacle_to_clear,
    const Point *obstacles, size_t obstacle_count,
    Point *out_centers, size_t out_cap) {
  if (!out_centers || out_cap == 0 || !obstacles || obstacle_count == 0) {
    return 0;
  }
  size_t center_count = 0;
  for (size_t i = 0; i < obstacle_count && center_count < out_cap; ++i) {
    Point center = obstacles[i];
    if (!planner_v3_bfs_in_bounds(rows, cols, center.row, center.col)) {
      continue;
    }
    if (!planner_v3_bfs_is_in_bomb_range_3x3(center, obstacle_to_clear)) {
      continue;
    }
    if (planner_v3_bfs_point_in_list(center, out_centers, center_count)) {
      continue;
    }
    out_centers[center_count++] = center;
  }
  return center_count;
}

// 路径合法性相关工具
// 1）检查整条路径是否都在地图范围内
static int planner_v3_bfs_path_in_bounds(int rows, int cols, const Point *path, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    if (!planner_v3_bfs_in_bounds(rows, cols, path[i].row, path[i].col)) {
      return 0;
    }
  }
  return 1;
}

// 仅在“路径方向发生变化”的节点上做推箱可行性检查，减少重复计算
static int planner_v3_bfs_should_check_push_step(const Point *path, size_t step_idx) {
  if (!path || step_idx <= 1) {
    return 1;
  }
  int prev_dr = path[step_idx - 1].row - path[step_idx - 2].row;
  int prev_dc = path[step_idx - 1].col - path[step_idx - 2].col;
  int curr_dr = path[step_idx].row - path[step_idx - 1].row;
  int curr_dc = path[step_idx].col - path[step_idx - 1].col;
  return (prev_dr != curr_dr) || (prev_dc != curr_dc);
}

// 占用网格上的障碍判定（含炸弹 / 已阻塞炸弹）
static int planner_v3_bfs_is_obstacle(const uint8_t *occ_grid, int cols, int row, int col) {
  if (row < 0 || col < 0) return 0;
  return OCC_IS_OBSTACLE(occ_grid, cols, row, col);
}

static int planner_v3_bfs_is_obstacle_no_bomb(const uint8_t *occ_grid, int cols, int row, int col) {
  if (row < 0 || col < 0) return 0;
  return OCC_IS_OBSTACLE_NO_BOMB(occ_grid, cols, row, col);
}

static int planner_v3_bfs_is_box_at(const uint8_t *occ_grid, int cols, int row, int col, size_t skip_idx) {
  if (row < 0 || col < 0) return 0;
  return OCC_HAS_BOX_EXCLUDING(occ_grid, cols, row, col, skip_idx);
}

// 统计给定格子在 4 邻接方向上，被障碍/箱子占用的相邻格数量
static int planner_v3_bfs_adjacent_blockers_occ(const uint8_t *occ, int rows, int cols,
                                                 size_t skip_idx, int row, int col) {
  int count = 0;
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (int i = 0; i < 4; ++i) {
    int nr = row + dirs[i][0];
    int nc = col + dirs[i][1];
    if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc)) {
      count++;
      continue;
    }
    if (planner_v3_bfs_is_obstacle(occ, cols, nr, nc) ||
        planner_v3_bfs_is_box_at(occ, cols, nr, nc, skip_idx)) {
      count++;
    }
  }
  return count;
}

// 计算任意两点之间的最短距离（基于当前占用状态做一次 BFS）
static int planner_v3_bfs_distance_between(int rows, int cols, Point start, Point target,
                                           const Point *obstacles, size_t obstacle_count,
                                           const Point *bombs, size_t bomb_count,
                                           const Point *boxes, size_t box_count,
                                           int include_bombs,
                                           const uint8_t *occ_prebuilt);
static Point planner_v3_bfs_check_bomb_path_push_blocker(int rows, int cols,
                                                          Point bomb_start, Point target_pos,
                                                          const Point *obstacles, size_t obstacle_count,
                                                          const Point *bombs, size_t bomb_count, size_t bomb_idx,
                                                          const Point *boxes, size_t box_count,
                                                          int *out_blocker_is_box,
                                                          int *out_blocker_is_bomb);
static int planner_v3_bfs_clear_push_pos_obstacle_recursive(
    int rows, int cols, Point *car_pos,
    Point *bombs, size_t bomb_count, size_t bomb_idx,
    Point obstacle_on_push_pos,
    Point *obstacles, size_t *obstacle_count,
    const Point *boxes, size_t box_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps,
    int recursion_depth,
    Point *extra_exploded_out, size_t extra_cap, size_t *extra_count_out);

// 死锁判定：判断某个位置在占用网格中是否构成“角落等不可推进”状态
static int planner_v3_bfs_is_deadlock_occ(const uint8_t *occ, int rows, int cols,
                                             size_t skip_idx, int row, int col) {
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  int blocked[4] = {0, 0, 0, 0};

  for (int i = 0; i < 4; ++i) {
    int nr = row + dirs[i][0];
    int nc = col + dirs[i][1];
    if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc)) {
      blocked[i] = 1;
      continue;
    }
    if (planner_v3_bfs_is_obstacle(occ, cols, nr, nc) ||
        planner_v3_bfs_is_box_at(occ, cols, nr, nc, skip_idx)) {
      blocked[i] = 1;
    }
  }

  int blocked_count = blocked[0] + blocked[1] + blocked[2] + blocked[3];
  if (blocked_count < 2) return 0;
  if (blocked_count >= 3) return 1;
  if (blocked_count == 2) {
    if (blocked[0] && blocked[1]) return 0;
    if (blocked[2] && blocked[3]) return 0;
    return 1;
  }
  return 0;
}

// 判定 moving_idx 对应的箱子，从 start 位置出发是否可以在给定占用状态下到达 target
static int planner_v3_bfs_can_reach_goal(int rows, int cols, const Point *obstacles,
                                size_t obstacle_count, const Point *bombs, size_t bomb_count,
                                const Point *boxes,
                                size_t box_count, size_t moving_idx,
                                Point start, Point target, int include_bombs,
                                const uint8_t *occ_prebuilt) {
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_BFS_MAX_CELLS || total_cells <= 0) {
    return 0;
  }
  uint8_t *occ_buf = s_bfs_scratch.occ_buf;
  const uint8_t *occ = occ_prebuilt;
  if (!occ) {
    planner_v3_bfs_build_occupancy_grid(rows, cols, obstacles, obstacle_count, bombs, bomb_count, boxes, box_count, 1, occ_buf);
    occ = occ_buf;
  }
  if (!planner_v3_bfs_in_bounds(rows, cols, start.row, start.col) ||
      !planner_v3_bfs_in_bounds(rows, cols, target.row, target.col)) {
    return 0;
  }
  

  if (include_bombs) {
    if (planner_v3_bfs_is_obstacle(occ, cols, start.row, start.col) ||
        planner_v3_bfs_is_obstacle(occ, cols, target.row, target.col)) {
      return 0;
    }
  } else {
    if (planner_v3_bfs_is_obstacle_no_bomb(occ, cols, start.row, start.col) ||
        planner_v3_bfs_is_obstacle_no_bomb(occ, cols, target.row, target.col)) {
      return 0;
    }
  }
  
  if (planner_v3_bfs_is_box_at(occ, cols, start.row, start.col, moving_idx) ||
      planner_v3_bfs_is_box_at(occ, cols, target.row, target.col, moving_idx)) {
    return 0;
  }

  if (start.row == target.row && start.col == target.col) {
    return 1;
  }

  int *queue = s_bfs_scratch.queue;
  int head = 0;
  int tail = 0;

  s_bfs_visit_stamp++;
  uint32_t stamp = s_bfs_visit_stamp;

  int start_idx = start.row * cols + start.col;
  queue[tail++] = start_idx;
  s_bfs_visit_mark[start_idx] = stamp;

  while (head < tail) {
    if (head >= PLANNER_V3_BFS_MAX_CELLS) {
      break;
    }
    int curr_idx = queue[head++];
    int row = curr_idx / cols;
    int col = curr_idx % cols;

    if (row == target.row && col == target.col) {
      return 1;
    }

    const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    for (int i = 0; i < 4; ++i) {
      int dr = dirs[i][0];
      int dc = dirs[i][1];
      int new_row = row + dr;
      int new_col = col + dc;

      int push_row = row - (new_row - row);
      int push_col = col - (new_col - col);

      if (!planner_v3_bfs_in_bounds(rows, cols, new_row, new_col) ||
          !planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
        continue;
      }


      if (include_bombs) {
        if (planner_v3_bfs_is_obstacle(occ, cols, new_row, new_col) ||
            planner_v3_bfs_is_obstacle(occ, cols, push_row, push_col)) {
          continue;
        }
      } else {
        if (planner_v3_bfs_is_obstacle_no_bomb(occ, cols, new_row, new_col) ||
            planner_v3_bfs_is_obstacle_no_bomb(occ, cols, push_row, push_col)) {
          continue;
        }
      }

      if (planner_v3_bfs_is_box_at(occ, cols, new_row, new_col, moving_idx) ||
          planner_v3_bfs_is_box_at(occ, cols, push_row, push_col, moving_idx)) {
        continue;
      }

      int idx = new_row * cols + new_col;
      if (s_bfs_visit_mark[idx] != stamp) {
        s_bfs_visit_mark[idx] = stamp;
        queue[tail++] = idx;
      }
    }
  }

  return 0;
}

// 从目标点出发做一次全局 BFS，生成供 A* 使用的距离图（到目标的距离）
static int planner_v3_bfs_global_bfs_from_target(int rows, int cols, Point target,
                                             const Point *obstacles, size_t obstacle_count,
                                             const Point *bombs, size_t bomb_count,
                                             const Point *boxes, size_t box_count,
                                             int dist[PLANNER_V3_BFS_MAX_CELLS], int include_bombs,
                                             const uint8_t *occ_prebuilt) {
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_BFS_MAX_CELLS || total_cells <= 0) {
    return 0;
  }
  uint8_t *occ_buf = s_bfs_scratch.occ_buf;
  const uint8_t *occ_buf_p = occ_prebuilt;
  if (!occ_buf_p) {
    planner_v3_bfs_build_occupancy_grid(rows, cols, obstacles, obstacle_count, bombs, bomb_count, boxes, box_count, 1, occ_buf);
    occ_buf_p = occ_buf;
  }

  for (int i = 0; i < total_cells; ++i) {
    dist[i] = INT_MAX;
  }
  
  int *queue = s_bfs_scratch.queue;
  s_bfs_visit_stamp++;
  uint32_t stamp = s_bfs_visit_stamp;

  int head = 0;
  int tail = 0;


  int target_idx = target.row * cols + target.col;
  queue[tail++] = target_idx;
  s_bfs_visit_mark[target_idx] = stamp;
  dist[target_idx] = 0;
  
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  
  while (head < tail) {
    if (head >= PLANNER_V3_BFS_MAX_CELLS) break;
    
    int curr_idx = queue[head++];
    int curr_row = curr_idx / cols;
    int curr_col = curr_idx % cols;
    int curr_dist = dist[curr_idx];
    
    for (int d = 0; d < 4; ++d) {
      int nr = curr_row + dirs[d][0];
      int nc = curr_col + dirs[d][1];
      
      if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc)) {
        continue;
      }
      

      if (include_bombs) {
        if (planner_v3_bfs_is_obstacle(occ_buf_p, cols, nr, nc)) {
          continue;
        }
      } else {
        if (planner_v3_bfs_is_obstacle_no_bomb(occ_buf_p, cols, nr, nc)) {
          continue;
        }
      }
      if (planner_v3_bfs_is_box_at(occ_buf_p, cols, nr, nc, SIZE_MAX)) {
        continue;
      }
      
      int next_idx = nr * cols + nc;
      if (next_idx < 0 || next_idx >= total_cells) {
        continue;
      }
      
      if (s_bfs_visit_mark[next_idx] != stamp) {
        s_bfs_visit_mark[next_idx] = stamp;
        dist[next_idx] = curr_dist + 1;
        if (tail < PLANNER_V3_BFS_MAX_CELLS) {
          queue[tail++] = next_idx;
        }
      }
    }
  }
  
  return 1;
}

// 在给定占用网格（occ_buf）上，从某个起点做一次全局 BFS，生成距离图
static int planner_v3_bfs_global_bfs_from_point_use_occ(int rows, int cols, Point start,
    const uint8_t *occ_buf, int dist_map[PLANNER_V3_BFS_MAX_CELLS], int include_bombs) {
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_BFS_MAX_CELLS || total_cells <= 0 || !occ_buf) {
    return 0;
  }
  for (int i = 0; i < total_cells; ++i) {
    dist_map[i] = INT_MAX;
  }
  int *queue = s_bfs_scratch.queue;
  s_bfs_visit_stamp++;
  uint32_t stamp = s_bfs_visit_stamp;
  int head = 0;
  int tail = 0;
  int start_idx = start.row * cols + start.col;
  if (start_idx < 0 || start_idx >= total_cells) {
    return 0;
  }
  if (include_bombs) {
    if (planner_v3_bfs_is_obstacle(occ_buf, cols, start.row, start.col)) {
      return 0;
    }
  } else {
    if (planner_v3_bfs_is_obstacle_no_bomb(occ_buf, cols, start.row, start.col)) {
      return 0;
    }
  }
  if (planner_v3_bfs_is_box_at(occ_buf, cols, start.row, start.col, SIZE_MAX)) {
    return 0;
  }
  queue[tail++] = start_idx;
  s_bfs_visit_mark[start_idx] = stamp;
  dist_map[start_idx] = 0;
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  while (head < tail) {
    if (head >= PLANNER_V3_BFS_MAX_CELLS) break;
    int curr_idx = queue[head++];
    int curr_row = curr_idx / cols;
    int curr_col = curr_idx % cols;
    int curr_dist = dist_map[curr_idx];
    for (int d = 0; d < 4; ++d) {
      int nr = curr_row + dirs[d][0];
      int nc = curr_col + dirs[d][1];
      if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc)) continue;
      if (include_bombs) {
        if (planner_v3_bfs_is_obstacle(occ_buf, cols, nr, nc)) continue;
      } else {
        if (planner_v3_bfs_is_obstacle_no_bomb(occ_buf, cols, nr, nc)) continue;
      }
      if (planner_v3_bfs_is_box_at(occ_buf, cols, nr, nc, SIZE_MAX)) continue;
      int next_idx = nr * cols + nc;
      if (next_idx < 0 || next_idx >= total_cells) continue;
      if (s_bfs_visit_mark[next_idx] != stamp) {
        s_bfs_visit_mark[next_idx] = stamp;
        dist_map[next_idx] = curr_dist + 1;
        if (tail < PLANNER_V3_BFS_MAX_CELLS) {
          queue[tail++] = next_idx;
        }
      }
    }
  }
  return 1;
}

// 从起点 source 出发做一次全局 BFS，生成到各点的距离图
static int planner_v3_bfs_global_bfs_from_source(int rows, int cols, Point source,
                                             const Point *obstacles, size_t obstacle_count,
                                             const Point *bombs, size_t bomb_count,
                                             const Point *boxes, size_t box_count,
                                             int dist[PLANNER_V3_BFS_MAX_CELLS], int include_bombs,
                                             const uint8_t *occ_prebuilt) {
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_BFS_MAX_CELLS || total_cells <= 0) {
    return 0;
  }
  uint8_t *occ_buf = s_bfs_scratch.occ_buf;
  const uint8_t *occ_buf_p = occ_prebuilt;
  if (!occ_buf_p) {
    planner_v3_bfs_build_occupancy_grid(rows, cols, obstacles, obstacle_count, bombs, bomb_count, boxes, box_count, 1, occ_buf);
    occ_buf_p = occ_buf;
  }
  for (int i = 0; i < total_cells; ++i) {
    dist[i] = INT_MAX;
  }
  int *queue = s_bfs_scratch.queue;
  s_bfs_visit_stamp++;
  uint32_t stamp = s_bfs_visit_stamp;
  int head = 0;
  int tail = 0;
  int source_idx = source.row * cols + source.col;
  queue[tail++] = source_idx;
  s_bfs_visit_mark[source_idx] = stamp;
  dist[source_idx] = 0;
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  while (head < tail) {
    if (head >= PLANNER_V3_BFS_MAX_CELLS) break;
    int curr_idx = queue[head++];
    int curr_row = curr_idx / cols;
    int curr_col = curr_idx % cols;
    int curr_dist = dist[curr_idx];
    for (int d = 0; d < 4; ++d) {
      int nr = curr_row + dirs[d][0];
      int nc = curr_col + dirs[d][1];
      if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc)) continue;
      if (include_bombs) {
        if (planner_v3_bfs_is_obstacle(occ_buf_p, cols, nr, nc)) continue;
      } else {
        if (planner_v3_bfs_is_obstacle_no_bomb(occ_buf_p, cols, nr, nc)) continue;
      }
      if (planner_v3_bfs_is_box_at(occ_buf_p, cols, nr, nc, box_count)) continue;
      int next_idx = nr * cols + nc;
      if (next_idx < 0 || next_idx >= total_cells) continue;
      if (s_bfs_visit_mark[next_idx] != stamp) {
        s_bfs_visit_mark[next_idx] = stamp;
        dist[next_idx] = curr_dist + 1;
        if (tail < PLANNER_V3_BFS_MAX_CELLS) queue[tail++] = next_idx;
      }
    }
  }
  return 1;
}

// 基于预先计算好的 dist（到目标的距离图）执行 A*，生成具体路径
static int planner_v3_bfs_astar_with_dist(int rows, int cols, Point start, Point target,
                                      const Point *obstacles, size_t obstacle_count,
                                      const Point *bombs, size_t bomb_count,
                                      const Point *boxes, size_t box_count,
                                      const int dist[PLANNER_V3_BFS_MAX_CELLS],
                                      Point *path, size_t path_cap, size_t *path_len,
                                      int include_bombs,
                                      int check_push, int push_mode, Point car_start, int strict_first_push,
                                      const Point *forbidden_from, const Point *forbidden_to, size_t forbidden_count,
                                      const uint8_t *occ_prebuilt) {
  (void)push_mode;
  (void)car_start;
    int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_BFS_MAX_CELLS || total_cells <= 0) {
    return 0;
  }
  uint8_t *occ_buf = s_astar_scratch.occ_buf;
  const uint8_t *occ_buf_p = occ_prebuilt;
  if (!occ_buf_p) {
    planner_v3_bfs_build_occupancy_grid(rows, cols, obstacles, obstacle_count, bombs, bomb_count, boxes, box_count, 1, occ_buf);
    occ_buf_p = occ_buf;
  }
  
  if (start.row == target.row && start.col == target.col) {
    if (path_cap < 1) return 0;
    path[0] = start;
    *path_len = 1;
    return 1;
  }
  
  // A* 开放表节点定义与辅助信息（包含 f 值和车到推点距离）
  typedef struct {
    int idx;
    int f_score;  // f = g + h
    int car_to_push_dist;
  } AStarNode;
  
  int *g_score = s_astar_scratch.g_score;
  int *parent = s_astar_scratch.parent;
  s_astar_run_stamp += 2;
  uint32_t open_mark = s_astar_run_stamp - 1;
  uint32_t closed_mark = s_astar_run_stamp;

  for (int i = 0; i < total_cells; ++i) {
    g_score[i] = INT_MAX;
    parent[i] = -1;
  }

  int *f_score = s_astar_scratch.f_score;
  int *car_to_push_best = s_astar_scratch.car_to_push_best;
  int *bucket_head = s_astar_scratch.bucket_head;
  int *bucket_next = s_astar_scratch.bucket_next;
  for (int i = 0; i < PLANNER_V3_ASTAR_F_BUCKET_MAX; ++i) {
    bucket_head[i] = -1;
  }
  for (int i = 0; i < total_cells; ++i) {
    bucket_next[i] = -1;
    f_score[i] = INT_MAX;
    car_to_push_best[i] = INT_MAX;
  }
  int open_size = 0;
  int current_min_f = PLANNER_V3_ASTAR_F_BUCKET_MAX - 1;
  
  int start_idx = start.row * cols + start.col;
  int target_idx = target.row * cols + target.col;
  

  if (dist[start_idx] == INT_MAX) {
    return 0;  // 起点不可达目标，提前失败
  }
  
  g_score[start_idx] = 0;
  int start_f = dist[start_idx];  // 此处 g=0，h=dist[start]，所以 f = g + h = h
  int start_bucket = start_f;
  if (start_bucket < 0) start_bucket = 0;
  if (start_bucket >= PLANNER_V3_ASTAR_F_BUCKET_MAX) start_bucket = PLANNER_V3_ASTAR_F_BUCKET_MAX - 1;
  bucket_head[start_bucket] = start_idx;
  bucket_next[start_idx] = -1;
  f_score[start_idx] = start_f;
  car_to_push_best[start_idx] = check_push ? 0 : dist[start_idx];
  open_size = 1;
  current_min_f = start_bucket;
  s_astar_mark[start_idx] = open_mark;
  
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  int last_expansion_dir = -1;

  while (open_size > 0) {

    int bucket_idx = current_min_f;
    while (bucket_idx < PLANNER_V3_ASTAR_F_BUCKET_MAX && bucket_head[bucket_idx] == -1) {
      ++bucket_idx;
    }
    if (bucket_idx >= PLANNER_V3_ASTAR_F_BUCKET_MAX) {
      break;
    }
    current_min_f = bucket_idx;
    int head = bucket_head[bucket_idx];
    int best = head;
    int best_prev = -1;
    int prev = head;
    int node = (head == -1) ? -1 : bucket_next[head];
    while (node != -1) {
      int prefer_i = 0;
      if (check_push) {

        if (car_to_push_best[node] < car_to_push_best[best]) {
          prefer_i = 1;
        }
      } else {
        /* 非 check_push 情况：仅保留方向延续 tiebreaker，取消二级 car_to_push tiebreaker */
        int ri = node / cols, ci = node % cols;
        int rbest = best / cols, cbest = best % cols;
        int pi = parent[node], pbest = parent[best];
        int in_dir_i = -1, in_dir_best = -1;
        if (pi >= 0) {
          int pr = pi / cols, pc = pi % cols;
          int dr = ri - pr, dc = ci - pc;
          for (int d = 0; d < 4; d++)
            if (dirs[d][0] == dr && dirs[d][1] == dc) { in_dir_i = d; break; }
        }
        if (pbest >= 0) {
          int pr = pbest / cols, pc = pbest % cols;
          int dr = rbest - pr, dc = cbest - pc;
          for (int d = 0; d < 4; d++)
            if (dirs[d][0] == dr && dirs[d][1] == dc) { in_dir_best = d; break; }
        }
        int match_i = (last_expansion_dir >= 0 && in_dir_i == last_expansion_dir);
        int match_best = (last_expansion_dir >= 0 && in_dir_best == last_expansion_dir);
        if (match_i && !match_best)
          prefer_i = 1;
        else if (!match_i && match_best)
          prefer_i = 0;
      }
      if (prefer_i) {
        best = node;
        best_prev = prev;
      }
      prev = node;
      node = bucket_next[node];
    }
    int curr_idx = best;
    if (best_prev == -1) {
      bucket_head[bucket_idx] = bucket_next[best];
    } else {
      bucket_next[best_prev] = bucket_next[best];
    }
    bucket_next[best] = -1;
    --open_size;
    s_astar_mark[curr_idx] = closed_mark;

    if (parent[curr_idx] >= 0) {
      int pr = parent[curr_idx] / cols, pc = parent[curr_idx] % cols;
      int curr_row_tmp = curr_idx / cols, curr_col_tmp = curr_idx % cols;
      int dr = curr_row_tmp - pr, dc = curr_col_tmp - pc;
      last_expansion_dir = -1;
      for (int d = 0; d < 4; d++)
        if (dirs[d][0] == dr && dirs[d][1] == dc) { last_expansion_dir = d; break; }
    } else {
      last_expansion_dir = -1;
    }
    

    if (curr_idx == target_idx) {

      int *path_indices = s_astar_scratch.path_indices;
      int path_count = 0;
      int idx = target_idx;
      
      while (idx != -1 && path_count < PLANNER_V3_BFS_MAX_CELLS) {
        path_indices[path_count++] = idx;
        idx = parent[idx];
      }
      

      *path_len = 0;
      for (int i = path_count - 1; i >= 0 && *path_len < path_cap; --i) {
        int pidx = path_indices[i];
        path[*path_len].row = pidx / cols;
        path[*path_len].col = pidx % cols;
        (*path_len)++;
      }
      
      return (*path_len > 0) ? 1 : 0;
    }
    
    int curr_row = curr_idx / cols;
    int curr_col = curr_idx % cols;
    

    for (int d = 0; d < 4; ++d) {
      int nr = curr_row + dirs[d][0];
      int nc = curr_col + dirs[d][1];
      
      if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc)) {
        continue;
      }
      

      int is_obs = 0;
      if (include_bombs) {
        is_obs = planner_v3_bfs_is_obstacle(occ_buf_p, cols, nr, nc);
      } else {
        is_obs = planner_v3_bfs_is_obstacle_no_bomb(occ_buf_p, cols, nr, nc);
      }
      if (is_obs) {
        continue;
      }
      
      if (planner_v3_bfs_is_box_at(occ_buf_p, cols, nr, nc, box_count)) {
        continue;
      }
      
      int next_idx = nr * cols + nc;
      if (next_idx < 0 || next_idx >= total_cells) {
        continue;
      }

      if (s_astar_forbidden_edge_bits) {
        int edge_idx = curr_idx * 4 + d;
        if (planner_v3_bfs_forbidden_edge_test(s_astar_forbidden_edge_bits, edge_idx)) {
          continue;
        }
      } else if (forbidden_count > 0 && forbidden_from && forbidden_to) {
        int skip_edge = 0;
        for (size_t fb = 0; fb < forbidden_count && !skip_edge; ++fb) {
          if (forbidden_from[fb].row == curr_row && forbidden_from[fb].col == curr_col &&
              forbidden_to[fb].row == nr && forbidden_to[fb].col == nc) {
            skip_edge = 1;
          }
        }
        if (skip_edge) {
          continue;
        }
      }
      
      int car_to_push_dist = dist[next_idx];
      

      if (check_push) {
        int should_check_push = 1;
        if (parent[curr_idx] == -1 && !strict_first_push) {
          should_check_push = 0;
        } else if (parent[curr_idx] != -1) {
          int prev_row = parent[curr_idx] / cols;
          int prev_col = parent[curr_idx] % cols;
          int prev_dr = curr_row - prev_row;
          int prev_dc = curr_col - prev_col;
          int curr_dr = nr - curr_row;
          int curr_dc = nc - curr_col;
          should_check_push = (prev_dr != curr_dr) || (prev_dc != curr_dc);
        }
        if (should_check_push) {
          int push_row = curr_row - (nr - curr_row);
          int push_col = curr_col - (nc - curr_col);
          if (!planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
            continue;
          }

          if (planner_v3_bfs_is_obstacle(occ_buf_p, cols, push_row, push_col)) {
            continue;
          }
          if (planner_v3_bfs_is_box_at(occ_buf_p, cols, push_row, push_col, SIZE_MAX)) {
            continue;
          }
        }
        car_to_push_dist = 0;
      }
      
      if (s_astar_mark[next_idx] == closed_mark) {
        continue;
      }

      int tentative_g = g_score[curr_idx] + 1;

      if (s_astar_mark[next_idx] != open_mark) {

        g_score[next_idx] = tentative_g;
        parent[next_idx] = curr_idx;
        

        int h = dist[next_idx];
        if (h == INT_MAX) {
          continue;  // 该节点到目标不可达，跳过
        }
        
        int f = tentative_g + h;
        
        int fb = f;
        if (fb < 0) fb = 0;
        if (fb >= PLANNER_V3_ASTAR_F_BUCKET_MAX) fb = PLANNER_V3_ASTAR_F_BUCKET_MAX - 1;
        bucket_next[next_idx] = bucket_head[fb];
        bucket_head[fb] = next_idx;
        f_score[next_idx] = f;
        car_to_push_best[next_idx] = car_to_push_dist;
        if (fb < current_min_f) current_min_f = fb;
        s_astar_mark[next_idx] = open_mark;
        ++open_size;
      } else if (tentative_g < g_score[next_idx]) {

        g_score[next_idx] = tentative_g;
        parent[next_idx] = curr_idx;
        
        int h = dist[next_idx];
        int f = tentative_g + h;
        int old_f = f_score[next_idx];
        if (old_f >= 0 && old_f < PLANNER_V3_ASTAR_F_BUCKET_MAX) {
          int prev = -1;
          int cur = bucket_head[old_f];
          while (cur != -1) {
            if (cur == next_idx) {
              if (prev == -1) {
                bucket_head[old_f] = bucket_next[cur];
              } else {
                bucket_next[prev] = bucket_next[cur];
              }
              break;
            }
            prev = cur;
            cur = bucket_next[cur];
          }
        }
        int fb = f;
        if (fb < 0) fb = 0;
        if (fb >= PLANNER_V3_ASTAR_F_BUCKET_MAX) fb = PLANNER_V3_ASTAR_F_BUCKET_MAX - 1;
        bucket_next[next_idx] = bucket_head[fb];
        bucket_head[fb] = next_idx;
        f_score[next_idx] = f;
        car_to_push_best[next_idx] = car_to_push_dist;
        if (fb < current_min_f) current_min_f = fb;
      }
    }
  }
  

  *path_len = 0;
  return 0;
}

// 计算任意两点最短距离：从 start 做 BFS，命中 target 立即返回（early-exit）
static int planner_v3_bfs_distance_between(int rows, int cols, Point start, Point target,
                                           const Point *obstacles, size_t obstacle_count,
                                           const Point *bombs, size_t bomb_count,
                                           const Point *boxes, size_t box_count,
                                           int include_bombs,
                                           const uint8_t *occ_prebuilt) {
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_BFS_MAX_CELLS || total_cells <= 0) {
    return INT_MAX;
  }
  if (!planner_v3_bfs_in_bounds(rows, cols, start.row, start.col) ||
      !planner_v3_bfs_in_bounds(rows, cols, target.row, target.col)) {
    return INT_MAX;
  }
  if (start.row == target.row && start.col == target.col) {
    return 0;
  }

  uint8_t *occ_buf = s_bfs_scratch.occ_buf;
  const uint8_t *occ = occ_prebuilt;
  if (!occ) {
    planner_v3_bfs_build_occupancy_grid(rows, cols, obstacles, obstacle_count, bombs, bomb_count, boxes, box_count, 1, occ_buf);
    occ = occ_buf;
  }

  if (include_bombs) {
    if (planner_v3_bfs_is_obstacle(occ, cols, start.row, start.col)) {
      return INT_MAX;
    }
  } else {
    if (planner_v3_bfs_is_obstacle_no_bomb(occ, cols, start.row, start.col)) {
      return INT_MAX;
    }
  }
  if (planner_v3_bfs_is_box_at(occ, cols, start.row, start.col, SIZE_MAX)) {
    return INT_MAX;
  }

  int start_idx = start.row * cols + start.col;
  int target_idx = target.row * cols + target.col;
  if (start_idx < 0 || start_idx >= total_cells || target_idx < 0 || target_idx >= total_cells) {
    return INT_MAX;
  }

  int *queue = s_bfs_scratch.queue;
  s_bfs_visit_stamp++;
  uint32_t stamp = s_bfs_visit_stamp;
  int head = 0;
  int tail = 0;
  int depth = 0;

  queue[tail++] = start_idx;
  s_bfs_visit_mark[start_idx] = stamp;

  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  while (head < tail) {
    int level_end = tail;
    while (head < level_end) {
      int curr_idx = queue[head++];
      int curr_row = curr_idx / cols;
      int curr_col = curr_idx % cols;

      for (int d = 0; d < 4; ++d) {
        int nr = curr_row + dirs[d][0];
        int nc = curr_col + dirs[d][1];
        if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc)) {
          continue;
        }
        int next_idx = nr * cols + nc;
        if (next_idx < 0 || next_idx >= total_cells) {
          continue;
        }
        if (s_bfs_visit_mark[next_idx] == stamp) {
          continue;
        }

        int blocked = 0;
        if (include_bombs) {
          blocked = planner_v3_bfs_is_obstacle(occ, cols, nr, nc);
        } else {
          blocked = planner_v3_bfs_is_obstacle_no_bomb(occ, cols, nr, nc);
        }
        if (!blocked && planner_v3_bfs_is_box_at(occ, cols, nr, nc, SIZE_MAX)) {
          blocked = 1;
        }

        if (next_idx == target_idx) {
          return depth + 1;
        }
        if (blocked) {
          continue;
        }

        s_bfs_visit_mark[next_idx] = stamp;
        if (tail < PLANNER_V3_BFS_MAX_CELLS) {
          queue[tail++] = next_idx;
        }
      }
    }
    depth++;
  }

  return INT_MAX;
}

// ?��????????????????????? 1 ?????????
static int planner_v3_bfs_check_adjacent(Point prev, Point curr) {
  int dr = planner_v3_bfs_abs(curr.row - prev.row);
  int dc = planner_v3_bfs_abs(curr.col - prev.col);
  return (dr + dc == 1);
}

// ??��??????????????????????��???? A* ????��??????????��??
static int planner_v3_bfs_car_move_with_global_astar(int rows, int cols, Point *car_pos, 
                                                  Point target,
                                                  const Point *obstacles, size_t obstacle_count,
                                                  const Point *bombs, size_t bomb_count,
                                                  const Point *boxes, size_t box_count,
                                                  Point *path_buffer, size_t path_capacity,
                                                  size_t *out_steps, int include_bombs,
                                                  const uint8_t *occ_prebuilt) {
  if (car_pos->row == target.row && car_pos->col == target.col) {
    return 1;  // ????????
  }
  

  int dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                         bombs, bomb_count, boxes, box_count, dist, include_bombs, occ_prebuilt)) {
    return 0;  // ?????????????? target ????????
  }
  

  Point temp_path[256];
  size_t temp_len = 0;
  
  if (!planner_v3_bfs_astar_with_dist(rows, cols, *car_pos, target,
                                  obstacles, obstacle_count,
                                  bombs, bomb_count,
                                  boxes, box_count, dist,
                                  temp_path, 256, &temp_len, include_bombs,
                                  0, 0, (Point){0, 0}, 0, NULL, NULL, 0, occ_prebuilt)) {
    return 0;  // A* ???????????????????
  }
  
  if (temp_len == 0) {
    return 0;
  }
  

  for (size_t i = 1; i < temp_len; ++i) {
    if (*out_steps >= path_capacity) {
      return -7;  // ???��????????????
    }
    

    if (*out_steps > 0 && 
        !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], temp_path[i])) {
      return 0;  // ?????????��???????????????��????? path_buffer ???????
    }
    
    path_buffer[(*out_steps)++] = temp_path[i];
  }
  

  if (temp_len > 0) {
    *car_pos = temp_path[temp_len - 1];
  }
  
  return 1;
}

static size_t planner_v3_bfs_collect_remaining_entities(
    int rows, int cols,
    const Point *src, size_t src_count,
    Point *dst, size_t dst_cap) {
  size_t out_count = 0;
  if (!src || !dst || dst_cap == 0) {
    return 0;
  }
  for (size_t i = 0; i < src_count && out_count < dst_cap; ++i) {
    if (!planner_v3_bfs_in_bounds(rows, cols, src[i].row, src[i].col)) {
      continue;
    }
    dst[out_count++] = src[i];
  }
  return out_count;
}

static int planner_v3_bfs_move_car_to_push_with_remaining_state(
    int rows, int cols,
    Point *car_pos, Point push_from,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps,
    const uint8_t *occ_prebuilt,
    int err_stage, int err_detail_unreachable) {
  if (!car_pos || !path_buffer || !out_steps) {
    return -6;
  }

  Point remaining_bombs[MAX_BOMBS];
  Point remaining_boxes[PLANNER_V3_BFS_MAX_BOXES];
  size_t remaining_bomb_count = planner_v3_bfs_collect_remaining_entities(
      rows, cols, bombs, bomb_count, remaining_bombs, MAX_BOMBS);
  size_t remaining_box_count = planner_v3_bfs_collect_remaining_entities(
      rows, cols, boxes, box_count, remaining_boxes, PLANNER_V3_BFS_MAX_BOXES);

  int car_result = planner_v3_bfs_car_move_with_global_astar(
      rows, cols, car_pos, push_from,
      obstacles, obstacle_count,
      remaining_bombs, remaining_bomb_count,
      remaining_boxes, remaining_box_count,
      path_buffer, path_capacity, out_steps,
      1, occ_prebuilt);
  if (car_result == 0) {
    last_err_stage = err_stage;
    last_err_detail = err_detail_unreachable;
    planner_v3_bfs_record_stage3_detail_if_needed(err_stage, err_detail_unreachable);
    return -6;
  }
  return car_result;
}

// ��??��???????????????????????????????????? 1??
static int planner_v3_bfs_validate_continuous_path(const Point *path, size_t len) {
  if (len < 2) {
    return 1;  // ??/????��?????????
  }
  for (size_t i = 1; i < len; ++i) {
    int dr = planner_v3_bfs_abs(path[i].row - path[i-1].row);
    int dc = planner_v3_bfs_abs(path[i].col - path[i-1].col);
    if (dr + dc != 1) {
      return 0;
    }
  }
  return 1;
}

// ??��??��??��?�??????/????????????????????????????????????????????
static int planner_v3_bfs_validate_boundary_path(int rows, int cols, Point boundary_pos, Point target,
                                                 Point car_pos,
                                                 const Point *obstacles, size_t obstacle_count,
                                                 const Point *bombs, size_t bomb_count,
                                                 const Point *boxes, size_t box_count,
                                                 size_t moving_idx, int include_bombs,
                                                 const uint8_t *occ_prebuilt) {

  if (boundary_pos.row == target.row && boundary_pos.col == target.col) {
    return 1;
  }


  if (!planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count, bombs, bomb_count,
                                    boxes, box_count, moving_idx, boundary_pos, target, include_bombs, occ_prebuilt)) {
    return 0;
  }

  // 仅做“推位合理性”检测：至少存在一个可行首推方向，且车可到该推位。
  uint8_t occ_local[PLANNER_V3_BFS_MAX_CELLS];
  const uint8_t *occ = occ_prebuilt;
  if (!occ) {
    planner_v3_bfs_build_occupancy_grid(rows, cols, obstacles, obstacle_count, bombs, bomb_count, boxes, box_count, 1, occ_local);
    occ = occ_local;
  }

  Point boxes_for_car[PLANNER_V3_BFS_MAX_BOXES];
  size_t bc = (box_count < PLANNER_V3_BFS_MAX_BOXES) ? box_count : PLANNER_V3_BFS_MAX_BOXES;
  for (size_t i = 0; i < bc; ++i) {
    boxes_for_car[i] = boxes[i];
  }
  if (moving_idx < bc) {
    boxes_for_car[moving_idx] = boundary_pos;
  }

  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (int d = 0; d < 4; ++d) {
    int nr = boundary_pos.row + dirs[d][0];
    int nc = boundary_pos.col + dirs[d][1];
    int pr = boundary_pos.row - dirs[d][0];
    int pc = boundary_pos.col - dirs[d][1];
    if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc) ||
        !planner_v3_bfs_in_bounds(rows, cols, pr, pc)) {
      continue;
    }

    if (include_bombs) {
      if (planner_v3_bfs_is_obstacle(occ, cols, nr, nc) ||
          planner_v3_bfs_is_obstacle(occ, cols, pr, pc)) {
        continue;
      }
    } else {
      if (planner_v3_bfs_is_obstacle_no_bomb(occ, cols, nr, nc) ||
          planner_v3_bfs_is_obstacle_no_bomb(occ, cols, pr, pc)) {
        continue;
      }
    }
    if (planner_v3_bfs_is_box_at(occ, cols, nr, nc, moving_idx) ||
        planner_v3_bfs_is_box_at(occ, cols, pr, pc, moving_idx)) {
      continue;
    }

    Point next_pos = {nr, nc};
    if (!planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count, bombs, bomb_count,
                                       boxes, box_count, moving_idx, next_pos, target, include_bombs, occ_prebuilt)) {
      continue;
    }

    int car_dist = planner_v3_bfs_distance_between(rows, cols, car_pos, (Point){pr, pc},
                                                    obstacles, obstacle_count, bombs, bomb_count,
                                                    boxes_for_car, bc, include_bombs, NULL);
    if (car_dist != INT_MAX) {
      return 1;
    }
  }

  return 0;
}

// ��?�A????��????????????????????��??push_from?????????????????????
static int planner_v3_bfs_validate_box_path_push_positions(int rows, int cols,
                                                           const Point *box_path, size_t path_len,
                                                           Point car_pos,
                                                           const Point *obstacles, size_t obstacle_count,
                                                           const Point *bombs, size_t bomb_count,
                                                           const Point *boxes, size_t box_count,
                                                           size_t moving_idx, int include_bombs, int strict_first_push,
                                                           size_t *out_fail_step,
                                                           const uint8_t *occ_prebuilt) {
  if (path_len < 2) return 1;
  (void)occ_prebuilt;
  Point current_pos = box_path[0];
  Point current_car = car_pos;
  for (size_t i = 1; i < path_len; ++i) {
    Point next_pos = box_path[i];
    if (!planner_v3_bfs_should_check_push_step(box_path, i)) {
      current_car = current_pos;
      current_pos = next_pos;
      continue;
    }
    if (i == 1 && !strict_first_push) {
      current_car = current_pos;
      current_pos = next_pos;
      continue;
    }

    int push_row = current_pos.row - (next_pos.row - current_pos.row);
    int push_col = current_pos.col - (next_pos.col - current_pos.col);
    if (!planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
      if (out_fail_step) *out_fail_step = i;
      return 0;
    }

    Point current_boxes_state[PLANNER_V3_BFS_MAX_BOXES];
    size_t bc = box_count <= PLANNER_V3_BFS_MAX_BOXES ? box_count : PLANNER_V3_BFS_MAX_BOXES;
    for (size_t k = 0; k < bc; ++k) {
      current_boxes_state[k] = (k == moving_idx) ? current_pos : boxes[k];
    }

    int car_dist = planner_v3_bfs_distance_between(rows, cols, current_car, (Point){push_row, push_col},
                                                    obstacles, obstacle_count, bombs, bomb_count,
                                                    current_boxes_state, bc, include_bombs, NULL);
    if (car_dist == INT_MAX) {
      if (i == 1 && !strict_first_push) {
        current_car = current_pos;
        current_pos = next_pos;
        continue;  // ?????????????????????
      }
      if (out_fail_step) *out_fail_step = i;
      return 0;
    }
    current_car = current_pos;
    current_pos = next_pos;
  }
  return 1;
}

// ?????????????
// ?????????????
// ?????????????
// ?????????????
// ?????????????
// ?????????????
static int planner_v3_bfs_validate_bomb_path_push_positions(int rows, int cols,
                                                            const Point *bomb_path, size_t path_len,
                                                            Point car_pos,
                                                            const Point *obstacles, size_t obstacle_count,
                                                            const Point *bombs_excluding_current, size_t bomb_count_excl,
                                                            const Point *boxes, size_t box_count) {
  if (path_len < 2) return 1;
  Point current_pos = bomb_path[0];
  Point current_car = car_pos;
  for (size_t i = 1; i < path_len; ++i) {
    Point next_pos = bomb_path[i];
    if (!planner_v3_bfs_should_check_push_step(bomb_path, i)) {
      current_car = current_pos;
      current_pos = next_pos;
      continue;
    }

    int push_row = current_pos.row - (next_pos.row - current_pos.row);
    int push_col = current_pos.col - (next_pos.col - current_pos.col);
    if (!planner_v3_bfs_in_bounds(rows, cols, push_row, push_col))
      return 0;

    Point temp_bombs[MAX_BOMBS];
    size_t tb_count = 0;
    for (size_t k = 0; k < bomb_count_excl && tb_count < MAX_BOMBS; ++k)
      temp_bombs[tb_count++] = bombs_excluding_current[k];
    if (tb_count < MAX_BOMBS)
      temp_bombs[tb_count++] = current_pos;
    uint8_t occ_buf[PLANNER_V3_BFS_MAX_CELLS];
    planner_v3_bfs_build_occupancy_grid(rows, cols, obstacles, obstacle_count, temp_bombs, tb_count, boxes, box_count, 1, occ_buf);
    if (planner_v3_bfs_is_obstacle(occ_buf, cols, push_row, push_col))
      return 0;
    if (planner_v3_bfs_is_box_at(occ_buf, cols, push_row, push_col, SIZE_MAX))
      return 0;
    int car_dist = planner_v3_bfs_distance_between(rows, cols, current_car, (Point){push_row, push_col},
                                                    obstacles, obstacle_count, temp_bombs, tb_count,
                                                    boxes, box_count, 1, occ_buf);
    if (car_dist == INT_MAX)
      return 0;
    current_car = current_pos;
    current_pos = next_pos;
  }
  return 1;
}

// ?????????????
// ?????????????
static size_t planner_v3_bfs_obstacles_destroyable_by_one_bomb(const Point *obstacles_to_clear,
                                                                size_t clear_count,
                                                                const Point *candidate_centers,
                                                                size_t candidate_center_count,
                                                                Point *out_centers,
                                                                size_t out_center_cap) {
  if (clear_count == 0 || !obstacles_to_clear || !out_centers || out_center_cap == 0) {
    return 0;
  }
  if (clear_count == 1) {
    out_centers[0] = obstacles_to_clear[0];
    return 1;
  }

  const Point *centers = candidate_centers;
  size_t center_source_count = candidate_center_count;

  size_t center_count = 0;

  // 规则B：某个路径障碍自身的 3x3 覆盖了其余全部路径障碍。
  for (size_t ci = 0; ci < clear_count && center_count < out_center_cap; ++ci) {
    Point path_center = obstacles_to_clear[ci];
    if (path_center.row < 0 || path_center.col < 0) {
      continue;
    }
    int cover_all_path = 1;
    for (size_t j = 0; j < clear_count && cover_all_path; ++j) {
      if (!planner_v3_bfs_is_in_bomb_range_3x3(path_center, obstacles_to_clear[j])) {
        cover_all_path = 0;
      }
    }
    if (cover_all_path &&
        !planner_v3_bfs_point_in_list(path_center, out_centers, center_count)) {
      out_centers[center_count++] = path_center;
    }
  }

  // 规则A：存在“非路径障碍”（仅要求不在 obstacles_on_path）作为公共覆盖点。
  if (centers && center_source_count > 0) {
    for (size_t ci = 0; ci < center_source_count && center_count < out_center_cap; ++ci) {
      Point center = centers[ci];
      if (center.row < 0 || center.col < 0) {
        continue;
      }
      if (planner_v3_bfs_point_in_list(center, obstacles_to_clear, clear_count)) {
        continue;
      }
      int in_all_path_ranges = 1;
      for (size_t j = 0; j < clear_count && in_all_path_ranges; ++j) {
        if (!planner_v3_bfs_is_in_bomb_range_3x3(obstacles_to_clear[j], center)) {
          in_all_path_ranges = 0;
        }
      }
      if (in_all_path_ranges &&
          !planner_v3_bfs_point_in_list(center, out_centers, center_count)) {
        out_centers[center_count++] = center;
      }
    }
  }

  return center_count;
}

// ?????????????
static Point planner_v3_bfs_nearest_unexcluded_obstacle_from_box(
    int rows, int cols, Point box_start,
    const Point *obstacles_on_path, size_t obstacles_on_path_count,
    const int16_t *obstacle_index_by_cell,
    const uint8_t *excluded_obstacle_mark) {
  Point nearest = {-1, -1};
  int min_dist = 0x7FFFFFFF;
  if (!obstacles_on_path || obstacles_on_path_count == 0 ||
      !obstacle_index_by_cell || !excluded_obstacle_mark) {
    return nearest;
  }
  for (size_t i = 0; i < obstacles_on_path_count; ++i) {
    Point p = obstacles_on_path[i];
    if (!planner_v3_bfs_in_bounds(rows, cols, p.row, p.col)) {
      continue;
    }
    int idx = p.row * cols + p.col;
    if (obstacle_index_by_cell[idx] < 0) {
      continue;
    }
    if (excluded_obstacle_mark[idx]) {
      continue;
    }
    {
      int d = planner_v3_bfs_abs(p.row - box_start.row) +
              planner_v3_bfs_abs(p.col - box_start.col);
      if (d < min_dist) {
        min_dist = d;
        nearest = p;
      }
    }
  }
  return nearest;
}

#define PLANNER_V3_BFS_SPECIAL_MAX_EXCLUDED_PATHS 50

static void planner_v3_bfs_special_build_obstacle_index_by_cell(
    int rows, int cols,
    const Point *obstacles, size_t obstacle_count,
    int16_t *obstacle_index_by_cell) {
  int total_cells = rows * cols;
  if (!obstacle_index_by_cell || total_cells <= 0 || total_cells > PLANNER_V3_BFS_MAX_CELLS) {
    return;
  }
  for (int i = 0; i < total_cells; ++i) {
    obstacle_index_by_cell[i] = -1;
  }
  for (size_t i = 0; i < obstacle_count; ++i) {
    if (!planner_v3_bfs_in_bounds(rows, cols, obstacles[i].row, obstacles[i].col)) {
      continue;
    }
    int idx = obstacles[i].row * cols + obstacles[i].col;
    obstacle_index_by_cell[idx] = (int16_t)i;
  }
}

static int planner_v3_bfs_special_mark_excluded_obstacle(
    int rows, int cols, Point obstacle_pt,
    const int16_t *obstacle_index_by_cell,
    uint8_t *excluded_obstacle_mark) {
  if (!obstacle_index_by_cell || !excluded_obstacle_mark) {
    return 0;
  }
  if (!planner_v3_bfs_in_bounds(rows, cols, obstacle_pt.row, obstacle_pt.col)) {
    return 0;
  }
  int idx = obstacle_pt.row * cols + obstacle_pt.col;
  if (obstacle_index_by_cell[idx] < 0) {
    return 0;
  }
  if (excluded_obstacle_mark[idx]) {
    return 0;
  }
  excluded_obstacle_mark[idx] = 1;
  return 1;
}

static size_t planner_v3_bfs_special_build_filtered_obstacles_from_marks(
    int rows, int cols,
    const Point *obstacles, size_t obstacle_count,
    const uint8_t *excluded_obstacle_mark,
    Point *filtered_obstacles, size_t filtered_cap) {
  size_t filtered_count = 0;
  if (!obstacles || !excluded_obstacle_mark || !filtered_obstacles || filtered_cap == 0) {
    return 0;
  }
  for (size_t i = 0; i < obstacle_count && filtered_count < filtered_cap; ++i) {
    if (!planner_v3_bfs_in_bounds(rows, cols, obstacles[i].row, obstacles[i].col)) {
      continue;
    }
    int idx = obstacles[i].row * cols + obstacles[i].col;
    if (excluded_obstacle_mark[idx]) {
      filtered_obstacles[filtered_count++] = obstacles[i];
    }
  }
  return filtered_count;
}

static int planner_v3_bfs_special_dir_to_index(int dr, int dc) {
  if (dr == -1 && dc == 0) return 0;
  if (dr == 1 && dc == 0) return 1;
  if (dr == 0 && dc == -1) return 2;
  if (dr == 0 && dc == 1) return 3;
  return -1;
}

static int planner_v3_bfs_special_add_forbidden_edge(
    int rows, int cols,
    Point from, Point to,
    uint32_t *forbidden_edge_bits,
    size_t *forbidden_count, size_t forbidden_cap) {
  if (!forbidden_edge_bits || !forbidden_count) {
    return 0;
  }
  if (!planner_v3_bfs_in_bounds(rows, cols, from.row, from.col) ||
      !planner_v3_bfs_in_bounds(rows, cols, to.row, to.col)) {
    return 0;
  }
  int dir = planner_v3_bfs_special_dir_to_index(to.row - from.row, to.col - from.col);
  if (dir < 0) {
    return 0;
  }
  int from_idx = from.row * cols + from.col;
  int mark_idx = from_idx * 4 + dir;
  if (planner_v3_bfs_forbidden_edge_test(forbidden_edge_bits, mark_idx)) {
    return 0;
  }
  if (*forbidden_count >= forbidden_cap) {
    return 0;
  }
  planner_v3_bfs_forbidden_edge_set(forbidden_edge_bits, mark_idx);
  (*forbidden_count)++;
  return 1;
}

static int planner_v3_bfs_special_replan_with_marks(
    int rows, int cols, Point box_start, Point target,
    const Point *obstacles, size_t obstacle_count,
    const uint8_t *excluded_obstacle_mark,
    const Point *temp_boxes, size_t temp_count,
    const uint32_t *forbidden_edge_bits, size_t forbidden_edge_count,
    int dist_ignore_obs_bomb[PLANNER_V3_BFS_MAX_CELLS],
    Point *initial_path, size_t *initial_path_len) {
  Point filtered_obstacles[200];
  size_t filtered_obstacle_count =
      planner_v3_bfs_special_build_filtered_obstacles_from_marks(
          rows, cols, obstacles, obstacle_count,
          excluded_obstacle_mark, filtered_obstacles, 200);
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target,
                                             filtered_obstacles, filtered_obstacle_count,
                                             NULL, 0, temp_boxes, temp_count,
                                             dist_ignore_obs_bomb, 0, NULL)) {
    return 0;
  }
  const uint32_t *saved_forbidden_bits = s_astar_forbidden_edge_bits;
  s_astar_forbidden_edge_bits = (forbidden_edge_bits && forbidden_edge_count > 0) ? forbidden_edge_bits : NULL;
  int astar_ok = planner_v3_bfs_astar_with_dist(rows, cols, box_start, target,
                                      filtered_obstacles, filtered_obstacle_count,
                                      NULL, 0, temp_boxes, temp_count,
                                      dist_ignore_obs_bomb,
                                      initial_path, PLANNER_V3_BFS_MAX_PATH_LEN, initial_path_len,
                                      0, 0, 0, (Point){0, 0}, 0,
                                      NULL, NULL, 0, NULL);
  s_astar_forbidden_edge_bits = saved_forbidden_bits;
  if (!astar_ok) {
    return 0;
  }
  return (*initial_path_len > 0) ? 1 : 0;
}

// ?????????????
static int planner_v3_bfs_special_add_next_forbidden_edge_from_path(
    int rows, int cols,
    const Point *path, size_t path_len,
    size_t preferred_step,
    uint32_t *forbidden_edge_bits,
    size_t *forbidden_count, size_t forbidden_cap) {
  if (!path || path_len < 2) {
    return 0;
  }
  if (preferred_step > 0 && preferred_step < path_len &&
      planner_v3_bfs_should_check_push_step(path, preferred_step)) {
    if (planner_v3_bfs_special_add_forbidden_edge(
            rows, cols, path[preferred_step - 1], path[preferred_step],
            forbidden_edge_bits,
            forbidden_count, forbidden_cap)) {
      return 1;
    }
  }
  for (size_t i = 1; i < path_len; ++i) {
    if (i == preferred_step) {
      continue;
    }
    if (!planner_v3_bfs_should_check_push_step(path, i)) {
      continue;
    }
    if (planner_v3_bfs_special_add_forbidden_edge(
            rows, cols, path[i - 1], path[i],
            forbidden_edge_bits,
            forbidden_count, forbidden_cap)) {
      return 1;
    }
  }
  for (size_t i = 1; i < path_len; ++i) {
    if (i == preferred_step) {
      continue;
    }
    if (planner_v3_bfs_special_add_forbidden_edge(
            rows, cols, path[i - 1], path[i],
            forbidden_edge_bits,
            forbidden_count, forbidden_cap)) {
      return 1;
    }
  }
  return 0;
}

/* Validate a full single-obstacle chain in planning phase only:
 *   (optional) bomb B projection explode -> bomb A projection explode -> box path push feasibility.
 * No execution-stage push routine is allowed here.
 */
static int planner_v3_bfs_special_validate_single_chain(
    int rows, int cols,
    const Point *initial_path, size_t initial_path_len,
    Point car_pos,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count,
    size_t moving_idx,
    size_t bomb_a_idx, Point bomb_a_target,
    size_t bomb_b_idx, Point bomb_b_target,
    int apply_req3) {
  if (!obstacles || !bombs || !boxes) {
    return 0;
  }
  if (bomb_a_idx == SIZE_MAX || bomb_a_idx >= bomb_count || bombs[bomb_a_idx].row < 0 || bombs[bomb_a_idx].col < 0) {
    return 0;
  }
  if (!planner_v3_bfs_in_bounds(rows, cols, bomb_a_target.row, bomb_a_target.col)) {
    return 0;
  }
  if (bomb_b_idx != SIZE_MAX) {
    if (bomb_b_idx >= bomb_count || bombs[bomb_b_idx].row < 0 || bombs[bomb_b_idx].col < 0) {
      return 0;
    }
    if (!planner_v3_bfs_in_bounds(rows, cols, bomb_b_target.row, bomb_b_target.col)) {
      return 0;
    }
  }

  int saved_last_err_stage = last_err_stage;
  int saved_last_err_detail = last_err_detail;
  int saved_last_err_detail_v3_stage3_local = last_err_detail_v3_stage3;

  Point temp_obstacles[200];
  size_t temp_obstacle_count = (obstacle_count < 200) ? obstacle_count : 200;
  memcpy(temp_obstacles, obstacles, temp_obstacle_count * sizeof(Point));

  Point temp_bombs[MAX_BOMBS];
  if (bomb_count > MAX_BOMBS) {
    last_err_stage = saved_last_err_stage;
    last_err_detail = saved_last_err_detail;
    last_err_detail_v3_stage3 = saved_last_err_detail_v3_stage3_local;
    return 0;
  }
  memcpy(temp_bombs, bombs, bomb_count * sizeof(Point));

  Point exploded_tmp[200];

  if (bomb_b_idx != SIZE_MAX) {
    Point bombs_excl_b[MAX_BOMBS];
    size_t bombs_excl_b_count = 0;
    for (size_t bi = 0; bi < bomb_count && bombs_excl_b_count < MAX_BOMBS; ++bi) {
      if (bi == bomb_b_idx) {
        continue;
      }
      if (temp_bombs[bi].row < 0 || temp_bombs[bi].col < 0) {
        continue;
      }
      bombs_excl_b[bombs_excl_b_count++] = temp_bombs[bi];
    }
    Point obs_for_b[200];
    size_t obs_for_b_count = 0;
    for (size_t oi = 0; oi < temp_obstacle_count && obs_for_b_count < 200; ++oi) {
      if (temp_obstacles[oi].row == bomb_b_target.row &&
          temp_obstacles[oi].col == bomb_b_target.col) {
        continue;
      }
      obs_for_b[obs_for_b_count++] = temp_obstacles[oi];
    }
    int d_b = planner_v3_bfs_distance_between(
        rows, cols,
        temp_bombs[bomb_b_idx], bomb_b_target,
        obs_for_b, obs_for_b_count,
        bombs_excl_b, bombs_excl_b_count,
        boxes, box_count,
        1, NULL);
    if (d_b == INT_MAX) {
      last_err_stage = saved_last_err_stage;
      last_err_detail = saved_last_err_detail;
      last_err_detail_v3_stage3 = saved_last_err_detail_v3_stage3_local;
      return 0;
    }
    size_t after_b_count = planner_v3_bfs_bomb_explode(
        temp_obstacles, temp_obstacle_count,
        bomb_b_target, exploded_tmp, 200);
    memcpy(temp_obstacles, exploded_tmp, after_b_count * sizeof(Point));
    temp_obstacle_count = after_b_count;
    temp_bombs[bomb_b_idx].row = -1;
    temp_bombs[bomb_b_idx].col = -1;
  }

  Point bombs_excl_a[MAX_BOMBS];
  size_t bombs_excl_a_count = 0;
  for (size_t bi = 0; bi < bomb_count && bombs_excl_a_count < MAX_BOMBS; ++bi) {
    if (bi == bomb_a_idx) {
      continue;
    }
    if (temp_bombs[bi].row < 0 || temp_bombs[bi].col < 0) {
      continue;
    }
    bombs_excl_a[bombs_excl_a_count++] = temp_bombs[bi];
  }
  Point obs_for_a[200];
  size_t obs_for_a_count = 0;
  for (size_t oi = 0; oi < temp_obstacle_count && obs_for_a_count < 200; ++oi) {
    if (temp_obstacles[oi].row == bomb_a_target.row &&
        temp_obstacles[oi].col == bomb_a_target.col) {
      continue;
    }
    obs_for_a[obs_for_a_count++] = temp_obstacles[oi];
  }
  int d_a = planner_v3_bfs_distance_between(
      rows, cols,
      temp_bombs[bomb_a_idx], bomb_a_target,
      obs_for_a, obs_for_a_count,
      bombs_excl_a, bombs_excl_a_count,
      boxes, box_count,
      1, NULL);
  if (d_a == INT_MAX) {
    last_err_stage = saved_last_err_stage;
    last_err_detail = saved_last_err_detail;
    last_err_detail_v3_stage3 = saved_last_err_detail_v3_stage3_local;
    return 0;
  }

  size_t after_a_count = planner_v3_bfs_bomb_explode(
      temp_obstacles, temp_obstacle_count,
      bomb_a_target, exploded_tmp, 200);
  memcpy(temp_obstacles, exploded_tmp, after_a_count * sizeof(Point));
  temp_obstacle_count = after_a_count;
  temp_bombs[bomb_a_idx].row = -1;
  temp_bombs[bomb_a_idx].col = -1;

  if (car_pos.row >= 0 && car_pos.col >= 0 && initial_path && initial_path_len >= 2) {
    size_t fail_step = 0;
    if (!planner_v3_bfs_validate_box_path_push_positions(
            rows, cols, initial_path, initial_path_len,
            car_pos,
            temp_obstacles, temp_obstacle_count,
            temp_bombs, bomb_count,
            boxes, box_count,
            moving_idx, 1, apply_req3,
            &fail_step, NULL)) {
      last_err_stage = saved_last_err_stage;
      last_err_detail = saved_last_err_detail;
      last_err_detail_v3_stage3 = saved_last_err_detail_v3_stage3_local;
      return 0;
    }
  }

  last_err_stage = saved_last_err_stage;
  last_err_detail = saved_last_err_detail;
  last_err_detail_v3_stage3 = saved_last_err_detail_v3_stage3_local;
  return 1;
}

static int planner_v3_bfs_special_pick_aux_bomb_for_obstacle(
    int rows, int cols,
    Point obstacle_to_clear,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const uint8_t *reserved_bomb_mark,
    size_t main_bomb_idx,
    const Point *boxes, size_t box_count,
    size_t *out_bomb_idx,
    Point *out_target) {
  if (!obstacles || !bombs || !out_bomb_idx || !out_target) {
    return 0;
  }
  size_t bomb_limit = (bomb_count < MAX_BOMBS) ? bomb_count : MAX_BOMBS;
  Point aux_targets[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
  size_t aux_target_count = planner_v3_bfs_collect_bomb_centers_covering_obstacle(
      rows, cols, obstacle_to_clear,
      obstacles, obstacle_count,
      aux_targets, PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS);
  if (aux_target_count == 0 &&
      planner_v3_bfs_in_bounds(rows, cols, obstacle_to_clear.row, obstacle_to_clear.col)) {
    aux_targets[aux_target_count++] = obstacle_to_clear;
  }
  if (aux_target_count == 0) {
    return 0;
  }

  size_t best_bomb_idx = SIZE_MAX;
  Point best_target = {-1, -1};
  int best_dist = INT_MAX;
  for (size_t bi = 0; bi < bomb_limit; ++bi) {
    if (bi == main_bomb_idx) {
      continue;
    }
    if (reserved_bomb_mark && reserved_bomb_mark[bi]) {
      continue;
    }
    if (bombs[bi].row < 0 || bombs[bi].col < 0) {
      continue;
    }
    for (size_t ti = 0; ti < aux_target_count; ++ti) {
      Point target = aux_targets[ti];
      Point filtered_obstacles[200];
      size_t filtered_count = 0;
      for (size_t oi = 0; oi < obstacle_count && filtered_count < 200; ++oi) {
        if (obstacles[oi].row == target.row &&
            obstacles[oi].col == target.col) {
          continue;
        }
        filtered_obstacles[filtered_count++] = obstacles[oi];
      }
      Point other_bombs[MAX_BOMBS];
      size_t other_count = 0;
      for (size_t bj = 0; bj < bomb_limit && other_count < MAX_BOMBS; ++bj) {
        if (bj == bi) {
          continue;
        }
        if (bombs[bj].row < 0 || bombs[bj].col < 0) {
          continue;
        }
        other_bombs[other_count++] = bombs[bj];
      }
      int d = planner_v3_bfs_distance_between(
          rows, cols,
          bombs[bi], target,
          filtered_obstacles, filtered_count,
          other_bombs, other_count,
          boxes, box_count,
          1, NULL);
      if (d != INT_MAX &&
          (best_bomb_idx == SIZE_MAX || d < best_dist)) {
        best_bomb_idx = bi;
        best_target = target;
        best_dist = d;
      }
    }
  }
  if (best_bomb_idx == SIZE_MAX) {
    return 0;
  }
  *out_bomb_idx = best_bomb_idx;
  *out_target = best_target;
  return 1;
}

static int planner_v3_bfs_plan_first_push_bomb_special_path(
    int rows, int cols,
    Point car_pos, Point first_push_pos,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count,
    size_t *out_bomb_idx,
    Point *out_bomb_target,
    Point *out_exploded_obstacles,
    size_t exploded_obstacle_cap,
    size_t *out_exploded_obstacle_count) {
  if (!out_bomb_idx || !out_bomb_target || !out_exploded_obstacle_count) {
    return 0;
  }
  *out_bomb_idx = SIZE_MAX;
  out_bomb_target->row = -1;
  out_bomb_target->col = -1;
  *out_exploded_obstacle_count = 0;

  if (!bombs || bomb_count == 0 ||
      !planner_v3_bfs_in_bounds(rows, cols, first_push_pos.row, first_push_pos.col)) {
    return 0;
  }

  int first_push_dist = planner_v3_bfs_distance_between(
      rows, cols,
      car_pos, first_push_pos,
      obstacles, obstacle_count,
      bombs, bomb_count,
      boxes, box_count,
      1, NULL);
  if (first_push_dist != INT_MAX) {
    return 0;
  }

  size_t bomb_limit = (bomb_count < MAX_BOMBS) ? bomb_count : MAX_BOMBS;
  size_t bomb_order[MAX_BOMBS];
  int bomb_score[MAX_BOMBS];
  size_t bomb_order_count = 0;

  for (size_t bi = 0; bi < bomb_limit && bomb_order_count < MAX_BOMBS; ++bi) {
    if (bombs[bi].row < 0 || bombs[bi].col < 0) {
      continue;
    }
    Point temp_bombs[MAX_BOMBS];
    size_t temp_bomb_count = 0;
    for (size_t bj = 0; bj < bomb_limit && temp_bomb_count < MAX_BOMBS; ++bj) {
      if (bj == bi) {
        continue;
      }
      if (bombs[bj].row < 0 || bombs[bj].col < 0) {
        continue;
      }
      temp_bombs[temp_bomb_count++] = bombs[bj];
    }
    int car_to_bomb = planner_v3_bfs_distance_between(
        rows, cols,
        car_pos, bombs[bi],
        obstacles, obstacle_count,
        temp_bombs, temp_bomb_count,
        boxes, box_count,
        1, NULL);
    if (car_to_bomb == INT_MAX) {
      continue;
    }
    int md = planner_v3_bfs_abs(bombs[bi].row - first_push_pos.row) +
             planner_v3_bfs_abs(bombs[bi].col - first_push_pos.col);
    bomb_order[bomb_order_count] = bi;
    bomb_score[bomb_order_count] = md + car_to_bomb;
    bomb_order_count++;
  }
  if (bomb_order_count == 0) {
    return 0;
  }

  for (size_t i = 0; i < bomb_order_count; ++i) {
    for (size_t j = i + 1; j < bomb_order_count; ++j) {
      if (bomb_score[j] < bomb_score[i]) {
        int td = bomb_score[i];
        bomb_score[i] = bomb_score[j];
        bomb_score[j] = td;
        size_t ti = bomb_order[i];
        bomb_order[i] = bomb_order[j];
        bomb_order[j] = ti;
      }
    }
  }

  for (size_t oi = 0; oi < bomb_order_count; ++oi) {
    size_t bomb_idx = bomb_order[oi];
    Point planning_obstacles[200];
    size_t planning_obstacle_count = obstacle_count;
    if (planning_obstacle_count > 200) {
      planning_obstacle_count = 200;
    }
    if (planning_obstacle_count > 0 && obstacles) {
      memcpy(planning_obstacles, obstacles, planning_obstacle_count * sizeof(Point));
    } else {
      planning_obstacle_count = 0;
    }

    Point sp_obstacle = {-1, -1};
    int sp_has_obstacle = 0;
    Point sp_path[PLANNER_V3_BFS_MAX_PATH_LEN];
    size_t sp_len = 0;
    int plan_ok = 0;

    for (int repl = 0; repl < 200; ++repl) {
      if (!planner_v3_bfs_special_path_planning(
              rows, cols,
              bombs[bomb_idx], first_push_pos,
              planning_obstacles, planning_obstacle_count,
              bombs, bomb_limit,
              boxes, box_count, SIZE_MAX,
              car_pos,
              &sp_obstacle, &sp_has_obstacle,
              sp_path, PLANNER_V3_BFS_MAX_PATH_LEN, &sp_len,
              1, 0) || sp_len == 0) {
        break;
      }

      Point legal_sp_centers[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
      size_t legal_sp_center_count = planner_v3_bfs_obstacles_destroyable_by_one_bomb(
          s_special_path_obstacles, s_special_path_obstacle_count,
          planning_obstacles, planning_obstacle_count,
          legal_sp_centers, PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS);
      if (legal_sp_center_count > 0) {
        plan_ok = 1;
        break;
      }

      Point nearest_obs = {-1, -1};
      int nearest_md = INT_MAX;
      for (size_t si = 0; si < s_special_path_obstacle_count; ++si) {
        int md = planner_v3_bfs_abs(bombs[bomb_idx].row - s_special_path_obstacles[si].row) +
                 planner_v3_bfs_abs(bombs[bomb_idx].col - s_special_path_obstacles[si].col);
        if (md < nearest_md) {
          nearest_md = md;
          nearest_obs = s_special_path_obstacles[si];
        }
      }
      if (nearest_obs.row < 0 || nearest_obs.col < 0) {
        break;
      }

      size_t new_plan_count = 0;
      for (size_t pi = 0; pi < planning_obstacle_count && new_plan_count < 200; ++pi) {
        if (planning_obstacles[pi].row == nearest_obs.row &&
            planning_obstacles[pi].col == nearest_obs.col) {
          continue;
        }
        planning_obstacles[new_plan_count++] = planning_obstacles[pi];
      }
      if (new_plan_count == planning_obstacle_count) {
        break;
      }
      planning_obstacle_count = new_plan_count;
    }

    if (!plan_ok) {
      continue;
    }

    Point chosen_target = first_push_pos;
    size_t chosen_exploded_count = 0;
    if (sp_has_obstacle && s_special_path_obstacle_count > 0) {
      Point explode_candidates[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
      size_t explode_candidate_count = planner_v3_bfs_obstacles_destroyable_by_one_bomb(
          s_special_path_obstacles, s_special_path_obstacle_count,
          obstacles, obstacle_count,
          explode_candidates, PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS);
      Point explode_target = {-1, -1};
      int explode_md = INT_MAX;
      for (size_t si = 0; si < explode_candidate_count; ++si) {
        int md = planner_v3_bfs_abs(bombs[bomb_idx].row - explode_candidates[si].row) +
                 planner_v3_bfs_abs(bombs[bomb_idx].col - explode_candidates[si].col);
        if (md < explode_md) {
          explode_md = md;
          explode_target = explode_candidates[si];
        }
      }
      if (explode_target.row < 0 || explode_target.col < 0) {
        continue;
      }
      chosen_target = explode_target;
      if (out_exploded_obstacles && exploded_obstacle_cap > 0) {
        chosen_exploded_count =
            (s_special_path_obstacle_count < exploded_obstacle_cap)
                ? s_special_path_obstacle_count
                : exploded_obstacle_cap;
        for (size_t si = 0; si < chosen_exploded_count; ++si) {
          out_exploded_obstacles[si] = s_special_path_obstacles[si];
        }
      }
    }

    *out_bomb_idx = bomb_idx;
    *out_bomb_target = chosen_target;
    *out_exploded_obstacle_count = chosen_exploded_count;
    return 1;
  }

  return 0;
}






// ?????????????
// ?????????????
// ?????????????
// ?????????????
// ?????????????
// ?????????????
// ?????????????
static int planner_v3_bfs_special_path_planning(int rows, int cols, Point box_start, Point target,
                                                const Point *obstacles, size_t obstacle_count,
                                                const Point *bombs, size_t bomb_count,
                                                const Point *boxes, size_t box_count,
                                                size_t moving_idx,
                                                Point car_pos,
                                                Point *obstacle_passed, int *has_obstacle,
                                                Point *out_path, size_t out_path_cap,
                                                size_t *out_path_len,
                                                int apply_req1, int apply_req3) {
#define SP_FAIL(code) do { last_special_path_fail_reason = (code); return 0; } while (0)
  /* Two-phase preplan mode: special path planning no longer keeps req-based behavior. */
  (void)apply_req1;
  (void)apply_req3;

  last_special_path_fail_reason = SPECIAL_PATH_FAIL_NONE;
  memset(&s_last_special_single_bomb_hint, 0, sizeof(s_last_special_single_bomb_hint));
  s_last_special_single_bomb_hint.valid = 0;
  s_last_special_single_bomb_hint.bomb_a_idx = SIZE_MAX;
  s_last_special_single_bomb_hint.bomb_b_idx = SIZE_MAX;
  s_last_special_single_bomb_hint.bomb_a_target.row = -1;
  s_last_special_single_bomb_hint.bomb_a_target.col = -1;
  s_last_special_single_bomb_hint.bomb_b_target.row = -1;
  s_last_special_single_bomb_hint.bomb_b_target.col = -1;
  if (out_path_len) {
    *out_path_len = 0;
  }

  int total_cells = rows * cols;
  if (total_cells <= 0 || total_cells > PLANNER_V3_BFS_MAX_CELLS) {
    SP_FAIL(SPECIAL_PATH_FAIL_SP_INVALID_GRID);
  }


  if (!planner_v3_bfs_in_bounds(rows, cols, box_start.row, box_start.col) ||
      !planner_v3_bfs_in_bounds(rows, cols, target.row, target.col)) {
    SP_FAIL(SPECIAL_PATH_FAIL_SP_START_OR_TARGET_OOB);
  }


  Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
  size_t temp_count = 0;
  for (size_t i = 0; i < box_count; ++i) {
    if (i == moving_idx) {
      continue;  // ?????????????????????
    }
    if (boxes[i].row < 0 || boxes[i].col < 0) {
      continue;  // ?????????????????????
    }
    temp_boxes[temp_count++] = boxes[i];
  }

  Point empty_obstacles[1] = {{-1, -1}};
  Point empty_bombs[1] = {{-1, -1}};


  int dist_ignore_obs_bomb[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, 
                                           empty_obstacles, 0,
                                           empty_bombs, 0,
                                           temp_boxes, temp_count,
                                           dist_ignore_obs_bomb, 0, NULL)) {
    SP_FAIL(SPECIAL_PATH_FAIL_SP_INITIAL_BFS_FAILED);
  }


  Point initial_path[PLANNER_V3_BFS_MAX_PATH_LEN];
  size_t initial_path_len = 0;
  if (!planner_v3_bfs_astar_with_dist(rows, cols, box_start, target,
                                      empty_obstacles, 0,
                                      empty_bombs, 0,
                                      temp_boxes, temp_count,
                                      dist_ignore_obs_bomb,
                                      initial_path, PLANNER_V3_BFS_MAX_PATH_LEN, &initial_path_len,
                                      0,
                                      0, 0, (Point){0, 0}, 0, NULL, NULL, 0, NULL)) {
    SP_FAIL(SPECIAL_PATH_FAIL_SP_INITIAL_ASTAR_FAILED);
  }

  if (initial_path_len == 0) {
    SP_FAIL(SPECIAL_PATH_FAIL_SP_INITIAL_PATH_EMPTY);
  }

  int16_t obstacle_index_by_cell[PLANNER_V3_BFS_MAX_CELLS];
  planner_v3_bfs_special_build_obstacle_index_by_cell(
      rows, cols, obstacles, obstacle_count, obstacle_index_by_cell);
  uint8_t excluded_obstacle_mark[PLANNER_V3_BFS_MAX_CELLS] = {0};


  uint32_t forbidden_edge_bits[PLANNER_V3_BFS_FORBIDDEN_EDGE_WORDS] = {0};
  size_t excluded_dir_count = 0;
  
  const int max_attempts = 22;
  int attempt = 0;

  while (attempt < max_attempts) {
    attempt++;
    if (attempt > 1) {

      if (!planner_v3_bfs_special_replan_with_marks(
              rows, cols, box_start, target,
              obstacles, obstacle_count, excluded_obstacle_mark,
              temp_boxes, temp_count,
              forbidden_edge_bits, excluded_dir_count,
              dist_ignore_obs_bomb, initial_path, &initial_path_len)) {
        continue;
      }
    }
    int reach_check_failed = 0;
    size_t reach_fail_step = 0;
    uint8_t occ_buf[PLANNER_V3_BFS_MAX_CELLS];
    planner_v3_bfs_build_occupancy_grid(rows, cols, obstacles, obstacle_count, empty_bombs, 0, boxes, box_count, 1, occ_buf);


    Point obstacles_on_path[200];
    size_t obstacles_on_path_count = 0;
    uint8_t obstacles_on_path_mark[PLANNER_V3_BFS_MAX_CELLS] = {0};
    uint8_t bombs_on_path_mark[PLANNER_V3_BFS_MAX_CELLS] = {0};
    for (size_t i = 0; i < initial_path_len; ++i) {
      int path_idx = initial_path[i].row * cols + initial_path[i].col;
      if (path_idx >= 0 && path_idx < total_cells) {
        bombs_on_path_mark[path_idx] = 1;
      }
      if (planner_v3_bfs_is_obstacle_no_bomb(occ_buf, cols,
                                             initial_path[i].row, initial_path[i].col)) {
        int cell_idx = initial_path[i].row * cols + initial_path[i].col;
        if (obstacles_on_path_count < 200 && !obstacles_on_path_mark[cell_idx]) {
          obstacles_on_path_mark[cell_idx] = 1;
          obstacles_on_path[obstacles_on_path_count++] = initial_path[i];
        }
      }
    }


    if (obstacles_on_path_count >= 1) {
      Point legal_centers[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
      size_t legal_center_count = planner_v3_bfs_obstacles_destroyable_by_one_bomb(
          obstacles_on_path, obstacles_on_path_count,
          obstacles, obstacle_count,
          legal_centers, PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS);
      if (obstacles_on_path_count == 1) {
        Point single_centers[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
        size_t single_center_count = planner_v3_bfs_collect_bomb_centers_covering_obstacle(
            rows, cols, obstacles_on_path[0],
            obstacles, obstacle_count,
            single_centers, PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS);
        if (single_center_count == 0) {
          single_centers[single_center_count++] = obstacles_on_path[0];
        }
        legal_center_count = (single_center_count < PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS)
                                 ? single_center_count
                                 : PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS;
        memcpy(legal_centers, single_centers, legal_center_count * sizeof(Point));
      }
      if (legal_center_count == 0) {

        Point nearest = planner_v3_bfs_nearest_unexcluded_obstacle_from_box(
            rows, cols, box_start, obstacles_on_path, obstacles_on_path_count,
            obstacle_index_by_cell, excluded_obstacle_mark);
        if (nearest.row >= 0 && nearest.col >= 0) {
          planner_v3_bfs_special_mark_excluded_obstacle(
              rows, cols, nearest, obstacle_index_by_cell, excluded_obstacle_mark);
        }
        continue;
      }

      int bomb_reachable = 0;
      size_t selected_bomb_idx_multi = SIZE_MAX;
      Point selected_target_multi = legal_centers[0];
      uint8_t selected_aux_bomb_mark_multi[MAX_BOMBS] = {0};
      size_t bomb_order_multi[MAX_BOMBS];
      int bomb_md_to_centers[MAX_BOMBS];
      size_t bomb_order_count_multi = 0;
      for (size_t bi = 0; bi < bomb_count && bomb_order_count_multi < MAX_BOMBS; ++bi) {
        if (bombs[bi].row < 0 || bombs[bi].col < 0) {
          continue;
        }
        bomb_order_multi[bomb_order_count_multi] = bi;
        int best_md = INT_MAX;
        for (size_t ci = 0; ci < legal_center_count; ++ci) {
          int md = planner_v3_bfs_abs(bombs[bi].row - legal_centers[ci].row) +
                   planner_v3_bfs_abs(bombs[bi].col - legal_centers[ci].col);
          if (md < best_md) {
            best_md = md;
          }
        }
        bomb_md_to_centers[bomb_order_count_multi] = best_md;
        bomb_order_count_multi++;
      }
      for (size_t i = 0; i < bomb_order_count_multi; ++i) {
        for (size_t j = i + 1; j < bomb_order_count_multi; ++j) {
          if (bomb_md_to_centers[j] < bomb_md_to_centers[i]) {
            int td = bomb_md_to_centers[i];
            bomb_md_to_centers[i] = bomb_md_to_centers[j];
            bomb_md_to_centers[j] = td;
            size_t ti = bomb_order_multi[i];
            bomb_order_multi[i] = bomb_order_multi[j];
            bomb_order_multi[j] = ti;
          }
        }
      }

      for (size_t bo = 0; bo < bomb_order_count_multi && !bomb_reachable; ++bo) {
        size_t bi = bomb_order_multi[bo];
        Point temp_bombs_other[MAX_BOMBS];
        size_t tb_count = 0;
        for (size_t bj = 0; bj < bomb_count && tb_count < MAX_BOMBS; ++bj) {
          if (bj == bi) continue;
          if (bombs[bj].row < 0 || bombs[bj].col < 0) continue;
          temp_bombs_other[tb_count++] = bombs[bj];
        }

        size_t center_order[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
        int center_dist[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
        size_t center_count = 0;
        for (size_t ti = 0; ti < legal_center_count && center_count < PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS; ++ti) {
          Point target_obs_multi = legal_centers[ti];
          Point filtered_for_bomb[200];
          size_t nf = 0;
          for (size_t i = 0; i < obstacle_count && nf < 200; ++i) {
            if (obstacles[i].row == target_obs_multi.row && obstacles[i].col == target_obs_multi.col) {
              continue;
            }
            filtered_for_bomb[nf++] = obstacles[i];
          }
          int dist = planner_v3_bfs_distance_between(rows, cols, bombs[bi], target_obs_multi,
                                                     filtered_for_bomb, nf, temp_bombs_other, tb_count,
                                                     boxes, box_count, 1, NULL);
          center_order[center_count] = ti;
          center_dist[center_count] = dist;
          center_count++;
        }
        for (size_t i = 0; i < center_count; ++i) {
          for (size_t j = i + 1; j < center_count; ++j) {
            int di = center_dist[i];
            int dj = center_dist[j];
            int j_better = 0;
            if (di == INT_MAX && dj != INT_MAX) {
              j_better = 1;
            } else if (dj != INT_MAX && di != INT_MAX && dj < di) {
              j_better = 1;
            }
            if (j_better) {
              int td = center_dist[i];
              center_dist[i] = center_dist[j];
              center_dist[j] = td;
              size_t ti = center_order[i];
              center_order[i] = center_order[j];
              center_order[j] = ti;
            }
          }
        }

        for (size_t ci = 0; ci < center_count && !bomb_reachable; ++ci) {
          if (center_dist[ci] == INT_MAX) {
            continue;
          }
          size_t ti = center_order[ci];
          Point target_obs_multi = legal_centers[ti];
          uint8_t candidate_aux_bomb_mark[MAX_BOMBS] = {0};
          int candidate_ok = 1;
          if (car_pos.row < 0 || car_pos.col < 0) {
            candidate_ok = 0;
          }

          Point temp_obstacles[200];
          size_t temp_obstacle_count = obstacle_count;
          if (temp_obstacle_count > 200) temp_obstacle_count = 200;
          memcpy(temp_obstacles, obstacles, temp_obstacle_count * sizeof(Point));
          Point temp_bombs[MAX_BOMBS];
          memcpy(temp_bombs, bombs, bomb_count * sizeof(Point));

          /* Planning phase only: resolve push blockers by projected auxiliary bomb usage. */
          int chain_guard = 0;
          while (candidate_ok && chain_guard < 16) {
            int push_blocker_is_box_multi = 0;
            int push_blocker_is_bomb_multi = 0;
            Point push_obs = planner_v3_bfs_check_bomb_path_push_blocker(
                rows, cols, temp_bombs[bi], target_obs_multi,
                temp_obstacles, temp_obstacle_count, temp_bombs, bomb_count, bi,
                boxes, box_count,
                &push_blocker_is_box_multi, &push_blocker_is_bomb_multi);
            if (push_obs.row < 0) {
              break;
            }
            if (push_blocker_is_box_multi || push_blocker_is_bomb_multi) {
              candidate_ok = 0;
              break;
            }

            size_t aux_bomb_idx = SIZE_MAX;
            Point aux_target = {-1, -1};
            if (!planner_v3_bfs_special_pick_aux_bomb_for_obstacle(
                    rows, cols,
                    push_obs,
                    temp_obstacles, temp_obstacle_count,
                    temp_bombs, bomb_count,
                    candidate_aux_bomb_mark,
                    bi,
                    boxes, box_count,
                    &aux_bomb_idx, &aux_target)) {
              candidate_ok = 0;
              break;
            }
            candidate_aux_bomb_mark[aux_bomb_idx] = 1;
            {
              Point projected_obstacles[200];
              size_t projected_count = planner_v3_bfs_bomb_explode(
                  temp_obstacles, temp_obstacle_count,
                  aux_target, projected_obstacles, 200);
              memcpy(temp_obstacles, projected_obstacles, projected_count * sizeof(Point));
              temp_obstacle_count = projected_count;
              temp_bombs[aux_bomb_idx].row = -1;
              temp_bombs[aux_bomb_idx].col = -1;
            }
            chain_guard++;
          }
          if (candidate_ok && chain_guard >= 16) {
            candidate_ok = 0;
          }
          if (candidate_ok) {
            Point filtered_for_a[200];
            size_t filtered_for_a_count = 0;
            for (size_t oi = 0; oi < temp_obstacle_count && filtered_for_a_count < 200; ++oi) {
              if (temp_obstacles[oi].row == target_obs_multi.row &&
                  temp_obstacles[oi].col == target_obs_multi.col) {
                continue;
              }
              filtered_for_a[filtered_for_a_count++] = temp_obstacles[oi];
            }

            Point other_bombs_for_a[MAX_BOMBS];
            size_t other_bombs_for_a_count = 0;
            for (size_t bj = 0; bj < bomb_count && other_bombs_for_a_count < MAX_BOMBS; ++bj) {
              if (bj == bi) continue;
              if (temp_bombs[bj].row < 0 || temp_bombs[bj].col < 0) continue;
              other_bombs_for_a[other_bombs_for_a_count++] = temp_bombs[bj];
            }

            int dist_map_a[PLANNER_V3_BFS_MAX_CELLS];
            if (!planner_v3_bfs_global_bfs_from_target(
                    rows, cols, target_obs_multi,
                    filtered_for_a, filtered_for_a_count,
                    other_bombs_for_a, other_bombs_for_a_count,
                    boxes, box_count,
                    dist_map_a, 1, NULL)) {
              candidate_ok = 0;
            } else {
              Point bomb_path_a[PLANNER_V3_BFS_MAX_PATH_LEN];
              size_t bomb_path_a_len = 0;
              if (!planner_v3_bfs_astar_with_dist(
                      rows, cols, temp_bombs[bi], target_obs_multi,
                      filtered_for_a, filtered_for_a_count,
                      other_bombs_for_a, other_bombs_for_a_count,
                      boxes, box_count,
                      dist_map_a,
                      bomb_path_a, PLANNER_V3_BFS_MAX_PATH_LEN, &bomb_path_a_len,
                      1,
                      0, 0, (Point){0, 0}, 0, NULL, NULL, 0, NULL)) {
                candidate_ok = 0;
              } else if (!planner_v3_bfs_validate_bomb_path_push_positions(
                             rows, cols,
                             bomb_path_a, bomb_path_a_len,
                             car_pos,
                             filtered_for_a, filtered_for_a_count,
                             other_bombs_for_a, other_bombs_for_a_count,
                             boxes, box_count)) {
                candidate_ok = 0;
              }
            }
          }
          if (!candidate_ok) {
            continue;
          }

          bomb_reachable = 1;
          selected_bomb_idx_multi = bi;
          selected_target_multi = target_obs_multi;
          memcpy(selected_aux_bomb_mark_multi, candidate_aux_bomb_mark, sizeof(selected_aux_bomb_mark_multi));
        }
      }
      if (!bomb_reachable) {

        Point nearest = planner_v3_bfs_nearest_unexcluded_obstacle_from_box(
            rows, cols, box_start, obstacles_on_path, obstacles_on_path_count,
            obstacle_index_by_cell, excluded_obstacle_mark);
        if (nearest.row >= 0 && nearest.col >= 0) {
          planner_v3_bfs_special_mark_excluded_obstacle(
              rows, cols, nearest, obstacle_index_by_cell, excluded_obstacle_mark);
        }
        continue;
      }


      Point bombs_excl_path[MAX_BOMBS];
      size_t bombs_excl_count = 0;
      for (size_t k = 0; k < bomb_count && bombs_excl_count < MAX_BOMBS; ++k) {
        if (bombs[k].row < 0 || bombs[k].col < 0) continue;
        if (k == selected_bomb_idx_multi) continue;
        if (selected_aux_bomb_mark_multi[k]) continue;  // ?????????????????????
        int bomb_idx = bombs[k].row * cols + bombs[k].col;
        if (bomb_idx >= 0 && bomb_idx < total_cells && bombs_on_path_mark[bomb_idx]) {
          continue;
        }
        bombs_excl_path[bombs_excl_count++] = bombs[k];
      }



      Point obs_excl_path[200];
      size_t obs_excl_count = 0;
      for (size_t k = 0; k < obstacle_count && obs_excl_count < 200; ++k) {
        int obs_idx = obstacles[k].row * cols + obstacles[k].col;
        if (!(obs_idx >= 0 && obs_idx < total_cells && obstacles_on_path_mark[obs_idx])) {
          obs_excl_path[obs_excl_count++] = obstacles[k];
        }
      }

      int push_check_failed = 0;
      size_t push_fail_step_multi = 0;
      uint8_t occ_multi_push_buf[PLANNER_V3_BFS_MAX_CELLS];
      planner_v3_bfs_build_occupancy_grid(
          rows, cols,
          obs_excl_path, obs_excl_count,
          bombs_excl_path, bombs_excl_count,
          boxes, box_count, 1, occ_multi_push_buf);
      for (size_t i = 1; i < initial_path_len && !push_check_failed; ++i) {
        if (!planner_v3_bfs_should_check_push_step(initial_path, i)) {
          continue;
        }
        Point from = initial_path[i - 1];
        Point to   = initial_path[i];
        Point push_pos = (Point){ from.row - (to.row - from.row), from.col - (to.col - from.col) };
        if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col) ||
            planner_v3_bfs_is_box_at(occ_multi_push_buf, cols, push_pos.row, push_pos.col, moving_idx)) {
          push_check_failed = 1;
          push_fail_step_multi = i;
          continue;
        }
        if (planner_v3_bfs_is_obstacle(occ_multi_push_buf, cols, push_pos.row, push_pos.col)) {
          push_check_failed = 1;
          push_fail_step_multi = i;
        }
      }
      if (push_check_failed) {
        // ?????????
        planner_v3_bfs_special_add_next_forbidden_edge_from_path(
            rows, cols, initial_path, initial_path_len, push_fail_step_multi,
            forbidden_edge_bits,
            &excluded_dir_count, PLANNER_V3_BFS_SPECIAL_MAX_EXCLUDED_PATHS);
        continue;
      }



      reach_check_failed = 0;
      reach_fail_step = 0;
      if (car_pos.row >= 0 && car_pos.col >= 0 && initial_path_len >= 2) {
        if (!planner_v3_bfs_validate_box_path_push_positions(rows, cols, initial_path, initial_path_len,
            car_pos, obs_excl_path, obs_excl_count, bombs_excl_path, bombs_excl_count,
            boxes, box_count, moving_idx, 1, 0, &reach_fail_step, NULL)) {
          reach_check_failed = 1;
        }
      }
      if (reach_check_failed) {

        if (planner_v3_bfs_special_add_next_forbidden_edge_from_path(
                rows, cols, initial_path, initial_path_len, reach_fail_step,
                forbidden_edge_bits,
                &excluded_dir_count, PLANNER_V3_BFS_SPECIAL_MAX_EXCLUDED_PATHS)) {
          continue;
        } else {
          Point nearest = planner_v3_bfs_nearest_unexcluded_obstacle_from_box(
              rows, cols, box_start, obstacles_on_path, obstacles_on_path_count,
              obstacle_index_by_cell, excluded_obstacle_mark);
          if (nearest.row >= 0 && nearest.col >= 0) {
            planner_v3_bfs_special_mark_excluded_obstacle(
                rows, cols, nearest, obstacle_index_by_cell, excluded_obstacle_mark);
          }
          continue;
        }
      }

      s_special_path_obstacle_count = obstacles_on_path_count;
      if (s_special_path_obstacle_count > PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES) {
        s_special_path_obstacle_count = PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES;
      }
      for (size_t i = 0; i < s_special_path_obstacle_count; ++i) {
        s_special_path_obstacles[i] = obstacles_on_path[i];
      }
      if (out_path_len) *out_path_len = initial_path_len;
      if (out_path && out_path_cap > 0) {
        size_t copy_len = (initial_path_len < out_path_cap) ? initial_path_len : out_path_cap;
        memcpy(out_path, initial_path, copy_len * sizeof(Point));
      }
      if (has_obstacle) *has_obstacle = 1;
      if (obstacle_passed) *obstacle_passed = selected_target_multi;
      return 1;
    } else {


      int push_check_failed = 0;
      size_t push_fail_step_0 = 0;
      if (obstacles_on_path_count == 0) {

        for (size_t i = 1; i < initial_path_len && !push_check_failed; ++i) {
          if (!planner_v3_bfs_should_check_push_step(initial_path, i)) {
            continue;
          }
          if (i == 1) {
            continue;
          }
          Point from = initial_path[i - 1];
          Point to   = initial_path[i];
          Point push_pos = (Point){ from.row - (to.row - from.row), from.col - (to.col - from.col) };
          if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col)) {
            push_check_failed = 1;
            push_fail_step_0 = i;
            break;
          }
          if (planner_v3_bfs_is_box_at(occ_buf, cols, push_pos.row, push_pos.col, moving_idx)) {
            push_check_failed = 1;
            push_fail_step_0 = i;
            break;
          }
          if (planner_v3_bfs_is_obstacle_no_bomb(occ_buf, cols, push_pos.row, push_pos.col)) {
            push_check_failed = 1;
            push_fail_step_0 = i;
            break;
          }
        }
      } else {

      }

      if (push_check_failed && obstacles_on_path_count == 0) {
        // ?????????
        planner_v3_bfs_special_add_next_forbidden_edge_from_path(
            rows, cols, initial_path, initial_path_len, push_fail_step_0,
            forbidden_edge_bits,
            &excluded_dir_count, PLANNER_V3_BFS_SPECIAL_MAX_EXCLUDED_PATHS);
        continue;
      }



      size_t selected_bomb_idx_01 = SIZE_MAX;
      Point selected_target_01 = {-1, -1};
      size_t bomb_b_idx_01 = SIZE_MAX;
      Point bomb_b_target_01 = {-1, -1};
      int saw_a_push_box_blocker_01 = 0;
      if (obstacles_on_path_count == 1) {
        Point single_obs = obstacles_on_path[0];
        size_t bomb_order_01[MAX_BOMBS];
        size_t bomb_order_count_01 = 0;
        planner_v3_bfs_bombs_sorted_by_dist_to_obstacle(rows, cols, single_obs,
            obstacles, obstacle_count, bombs, bomb_count, boxes, box_count,
            bomb_order_01, &bomb_order_count_01,
            obstacles_on_path, obstacles_on_path_count);
        for (size_t boi = 0; boi < bomb_order_count_01 && selected_bomb_idx_01 == SIZE_MAX; ++boi) {
          size_t bi = bomb_order_01[boi];
          Point target_positions_01[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
          size_t target_count_01 = 0;
          planner_v3_bfs_targets_sorted_by_reachable_dist(rows, cols, car_pos,
              bi, bombs, bomb_count, single_obs, obstacles, obstacle_count,
              boxes, box_count,
              target_positions_01, &target_count_01,
              obstacles_on_path, obstacles_on_path_count,
              0);
          for (size_t ti = 0; ti < target_count_01 && selected_bomb_idx_01 == SIZE_MAX; ++ti) {
            Point a_target = target_positions_01[ti];
            int blocker_is_box_a = 0;
            int blocker_is_bomb_a = 0;
            Point push_blocker_a = planner_v3_bfs_check_bomb_path_push_blocker(
                rows, cols, bombs[bi], a_target,
                obstacles, obstacle_count, bombs, bomb_count, bi,
                boxes, box_count, &blocker_is_box_a, &blocker_is_bomb_a);

            if (push_blocker_a.row >= 0) {
              if (blocker_is_box_a || blocker_is_bomb_a) {
                saw_a_push_box_blocker_01 = 1;
                continue;
              }

              Point cand_targets_b[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
              size_t cand_count_b = planner_v3_bfs_collect_bomb_centers_covering_obstacle(
                  rows, cols, push_blocker_a,
                  obstacles, obstacle_count,
                  cand_targets_b, PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS);
              if (cand_count_b == 0 &&
                  planner_v3_bfs_in_bounds(rows, cols, push_blocker_a.row, push_blocker_a.col)) {
                cand_targets_b[cand_count_b++] = push_blocker_a;
              }
              if (cand_count_b == 0) {
                continue;
              }

              size_t bomb_order_b[MAX_BOMBS];
              int bomb_dist_b[MAX_BOMBS];
              size_t bomb_order_b_count = 0;
              for (size_t bj = 0; bj < bomb_count && bomb_order_b_count < MAX_BOMBS; ++bj) {
                if (bj == bi || bombs[bj].row < 0 || bombs[bj].col < 0) {
                  continue;
                }
                Point temp_bombs_for_d[MAX_BOMBS];
                size_t tb_count = 0;
                for (size_t bk = 0; bk < bomb_count && tb_count < MAX_BOMBS; ++bk) {
                  if (bk == bi || bk == bj || bombs[bk].row < 0 || bombs[bk].col < 0) {
                    continue;
                  }
                  temp_bombs_for_d[tb_count++] = bombs[bk];
                }
                int d = planner_v3_bfs_distance_between(
                    rows, cols, bombs[bj], bombs[bi],
                    obstacles, obstacle_count,
                    temp_bombs_for_d, tb_count,
                    boxes, box_count, 1, NULL);
                bomb_order_b[bomb_order_b_count] = bj;
                bomb_dist_b[bomb_order_b_count] = d;
                bomb_order_b_count++;
              }
              for (size_t i = 0; i < bomb_order_b_count; ++i) {
                for (size_t j = i + 1; j < bomb_order_b_count; ++j) {
                  int di = bomb_dist_b[i];
                  int dj = bomb_dist_b[j];
                  int j_better = 0;
                  if (di == INT_MAX && dj != INT_MAX) {
                    j_better = 1;
                  } else if (di != INT_MAX && dj != INT_MAX && dj < di) {
                    j_better = 1;
                  }
                  if (j_better) {
                    int td = bomb_dist_b[i];
                    bomb_dist_b[i] = bomb_dist_b[j];
                    bomb_dist_b[j] = td;
                    size_t tb = bomb_order_b[i];
                    bomb_order_b[i] = bomb_order_b[j];
                    bomb_order_b[j] = tb;
                  }
                }
              }

              for (size_t bo = 0; bo < bomb_order_b_count && selected_bomb_idx_01 == SIZE_MAX; ++bo) {
                size_t bj = bomb_order_b[bo];
                for (size_t ci = 0; ci < cand_count_b && selected_bomb_idx_01 == SIZE_MAX; ++ci) {
                  Point b_target = cand_targets_b[ci];
                  int blocker_is_box_b = 0;
                  int blocker_is_bomb_b = 0;
                  Point push_blocker_b = planner_v3_bfs_check_bomb_path_push_blocker(
                      rows, cols, bombs[bj], b_target,
                      obstacles, obstacle_count, bombs, bomb_count, bj,
                      boxes, box_count, &blocker_is_box_b, &blocker_is_bomb_b);
                  (void)blocker_is_box_b;
                  (void)blocker_is_bomb_b;
                  if (push_blocker_b.row >= 0) {
                    continue;
                  }

                  if (!planner_v3_bfs_special_validate_single_chain(
                          rows, cols,
                          initial_path, initial_path_len,
                          car_pos,
                          obstacles, obstacle_count,
                          bombs, bomb_count,
                          boxes, box_count,
                          moving_idx,
                          bi, a_target,
                          bj, b_target,
                          0)) {
                    continue;
                  }

                  selected_bomb_idx_01 = bi;
                  selected_target_01 = a_target;
                  bomb_b_idx_01 = bj;
                  bomb_b_target_01 = b_target;
                }
              }
              continue;
            }

            if (!planner_v3_bfs_special_validate_single_chain(
                    rows, cols,
                    initial_path, initial_path_len,
                    car_pos,
                    obstacles, obstacle_count,
                    bombs, bomb_count,
                    boxes, box_count,
                    moving_idx,
                    bi, a_target,
                    SIZE_MAX, (Point){-1, -1},
                    0)) {
              continue;
            }
            selected_bomb_idx_01 = bi;
            selected_target_01 = a_target;
          }
        }
        if (selected_bomb_idx_01 == SIZE_MAX) {

          Point nearest = planner_v3_bfs_nearest_unexcluded_obstacle_from_box(
              rows, cols, box_start, obstacles_on_path, obstacles_on_path_count,
              obstacle_index_by_cell, excluded_obstacle_mark);
          if (saw_a_push_box_blocker_01) {
            /* A 推位被箱子阻挡：按规则排除离当前箱子最近障碍后重算。 */
          }
          if (nearest.row >= 0 && nearest.col >= 0) {
            planner_v3_bfs_special_mark_excluded_obstacle(
                rows, cols, nearest, obstacle_index_by_cell, excluded_obstacle_mark);
          }
          continue;
        }
      }

      /* ???????????????? */
      Point obs_for_push[200];
      size_t obs_for_push_count = 0;
      Point bombs_for_push[MAX_BOMBS];
      size_t bombs_for_push_count = 0;
      if (obstacles_on_path_count == 1 && selected_bomb_idx_01 != SIZE_MAX) {
        Point projected_obstacles[200];
        size_t projected_count = (obstacle_count < 200) ? obstacle_count : 200;
        memcpy(projected_obstacles, obstacles, projected_count * sizeof(Point));
        Point projected_tmp[200];
        if (bomb_b_idx_01 != SIZE_MAX) {
          size_t after_b_count = planner_v3_bfs_bomb_explode(
              projected_obstacles, projected_count,
              bomb_b_target_01, projected_tmp, 200);
          memcpy(projected_obstacles, projected_tmp, after_b_count * sizeof(Point));
          projected_count = after_b_count;
        }
        size_t after_a_count = planner_v3_bfs_bomb_explode(
            projected_obstacles, projected_count,
            selected_target_01, projected_tmp, 200);
        memcpy(projected_obstacles, projected_tmp, after_a_count * sizeof(Point));
        projected_count = after_a_count;

        for (size_t k = 0; k < projected_count && obs_for_push_count < 200; ++k) {
          obs_for_push[obs_for_push_count++] = projected_obstacles[k];
        }
        for (size_t k = 0; k < bomb_count && bombs_for_push_count < MAX_BOMBS; ++k) {
          if (bombs[k].row < 0 || bombs[k].col < 0) continue;
          if (k == selected_bomb_idx_01) continue;
          if (k == bomb_b_idx_01) continue;
          bombs_for_push[bombs_for_push_count++] = bombs[k];
        }
      } else if (obstacles_on_path_count >= 1 && selected_bomb_idx_01 != SIZE_MAX) {

        for (size_t k = 0; k < obstacle_count && obs_for_push_count < 200; ++k) {
          int obs_idx = obstacles[k].row * cols + obstacles[k].col;
          if (obs_idx >= 0 && obs_idx < total_cells && obstacles_on_path_mark[obs_idx]) {
            continue;
          }
          obs_for_push[obs_for_push_count++] = obstacles[k];
        }
        for (size_t k = 0; k < bomb_count && bombs_for_push_count < MAX_BOMBS; ++k) {
          if (bombs[k].row < 0 || bombs[k].col < 0) continue;
          if (k == selected_bomb_idx_01) continue;
          if (k == bomb_b_idx_01) continue;
          bombs_for_push[bombs_for_push_count++] = bombs[k];
        }
      } else {

        for (size_t k = 0; k < obstacle_count && obs_for_push_count < 200; ++k) {
          int obs_idx = obstacles[k].row * cols + obstacles[k].col;
          if (!(obs_idx >= 0 && obs_idx < total_cells && obstacles_on_path_mark[obs_idx])) {
            obs_for_push[obs_for_push_count++] = obstacles[k];
          }
        }
        for (size_t k = 0; k < bomb_count && bombs_for_push_count < MAX_BOMBS; ++k) {
          if (bombs[k].row < 0 || bombs[k].col < 0) continue;
          int bomb_idx = bombs[k].row * cols + bombs[k].col;
          if (!(bomb_idx >= 0 && bomb_idx < total_cells && bombs_on_path_mark[bomb_idx])) {
            bombs_for_push[bombs_for_push_count++] = bombs[k];
          }
        }
      }


      if (obstacles_on_path_count == 1 && selected_bomb_idx_01 != SIZE_MAX) {
        int push_pos_invalid = 0;
        size_t push_fail_step_01 = 0;
        uint8_t occ_01_push_buf[PLANNER_V3_BFS_MAX_CELLS];
        planner_v3_bfs_build_occupancy_grid(
            rows, cols,
            obs_for_push, obs_for_push_count,
            bombs_for_push, bombs_for_push_count,
            boxes, box_count, 1, occ_01_push_buf);
        for (size_t i = 1; i < initial_path_len && !push_pos_invalid; ++i) {
          if (!planner_v3_bfs_should_check_push_step(initial_path, i)) {
            continue;
          }
          Point from = initial_path[i - 1];
          Point to   = initial_path[i];
          Point push_pos = (Point){ from.row - (to.row - from.row), from.col - (to.col - from.col) };
          if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col) ||
              planner_v3_bfs_is_box_at(occ_01_push_buf, cols, push_pos.row, push_pos.col, moving_idx) ||
              planner_v3_bfs_is_obstacle(occ_01_push_buf, cols, push_pos.row, push_pos.col)) {
            push_pos_invalid = 1;
            push_fail_step_01 = i;
          }
        }
        if (push_pos_invalid) {
          planner_v3_bfs_special_add_next_forbidden_edge_from_path(
              rows, cols, initial_path, initial_path_len, push_fail_step_01,
              forbidden_edge_bits,
              &excluded_dir_count, PLANNER_V3_BFS_SPECIAL_MAX_EXCLUDED_PATHS);
          continue;
        }
      }


      reach_check_failed = 0;
      reach_fail_step = 0;
      if (car_pos.row >= 0 && car_pos.col >= 0 && initial_path_len >= 2) {
        if (!planner_v3_bfs_validate_box_path_push_positions(rows, cols, initial_path, initial_path_len,
            car_pos, obs_for_push, obs_for_push_count, bombs_for_push, bombs_for_push_count,
            boxes, box_count, moving_idx, 1, 0, &reach_fail_step, NULL)) {
          reach_check_failed = 1;
        }
      }
      if (reach_check_failed) {

        if (planner_v3_bfs_special_add_next_forbidden_edge_from_path(
                rows, cols, initial_path, initial_path_len, reach_fail_step,
                forbidden_edge_bits,
                &excluded_dir_count, PLANNER_V3_BFS_SPECIAL_MAX_EXCLUDED_PATHS)) {
          continue;
        } else if (obstacles_on_path_count > 0) {
          Point nearest = planner_v3_bfs_nearest_unexcluded_obstacle_from_box(
              rows, cols, box_start, obstacles_on_path, obstacles_on_path_count,
              obstacle_index_by_cell, excluded_obstacle_mark);
          if (nearest.row >= 0 && nearest.col >= 0) {
            planner_v3_bfs_special_mark_excluded_obstacle(
                rows, cols, nearest, obstacle_index_by_cell, excluded_obstacle_mark);
          }
          continue;
        }
      }

      if (obstacles_on_path_count == 1 && selected_bomb_idx_01 != SIZE_MAX) {
        s_last_special_single_bomb_hint.valid = 1;
        s_last_special_single_bomb_hint.bomb_a_idx = selected_bomb_idx_01;
        s_last_special_single_bomb_hint.bomb_a_target = selected_target_01;
        s_last_special_single_bomb_hint.bomb_b_idx = bomb_b_idx_01;
        s_last_special_single_bomb_hint.bomb_b_target = bomb_b_target_01;
        s_last_special_single_bomb_hint.bomb_b_exploded_obstacle_count = 0;
        if (bomb_b_idx_01 != SIZE_MAX &&
            planner_v3_bfs_in_bounds(rows, cols, bomb_b_target_01.row, bomb_b_target_01.col) &&
            obstacles && obstacle_count > 0) {
          Point remaining_obstacles[200];
          size_t remaining_count = planner_v3_bfs_bomb_explode(
              obstacles, obstacle_count,
              bomb_b_target_01, remaining_obstacles, 200);
          for (size_t oi = 0; oi < obstacle_count; ++oi) {
            if (s_last_special_single_bomb_hint.bomb_b_exploded_obstacle_count >=
                PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES) {
              break;
            }
            if (!planner_v3_bfs_point_in_list(
                    obstacles[oi], remaining_obstacles, remaining_count)) {
              s_last_special_single_bomb_hint.bomb_b_exploded_obstacles[
                  s_last_special_single_bomb_hint.bomb_b_exploded_obstacle_count++] = obstacles[oi];
            }
          }
        }
      }


      s_special_path_obstacle_count = obstacles_on_path_count;
      if (s_special_path_obstacle_count > PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES) {
        s_special_path_obstacle_count = PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES;
      }
      for (size_t i = 0; i < s_special_path_obstacle_count; ++i) {
        s_special_path_obstacles[i] = obstacles_on_path[i];
      }
      if (out_path_len) {
        *out_path_len = initial_path_len;
      }
      if (out_path && out_path_cap > 0) {
        size_t copy_len = (initial_path_len < out_path_cap) ? initial_path_len : out_path_cap;
        memcpy(out_path, initial_path, copy_len * sizeof(Point));
      }
      if (has_obstacle) {
        *has_obstacle = (obstacles_on_path_count > 0) ? 1 : 0;
      }
      if (obstacle_passed) {
        if (obstacles_on_path_count > 0) {
          *obstacle_passed = obstacles_on_path[0];
        } else {
          obstacle_passed->row = -1;
          obstacle_passed->col = -1;
        }
      }
      return 1;
    }
  }


  SP_FAIL(SPECIAL_PATH_FAIL_SP_MAX_ATTEMPTS_EXCEEDED);
#undef SP_FAIL
}

// ?????????????
static void planner_v3_bfs_find_bombs_on_path(int rows, int cols,
                                               const Point *path, size_t path_len,
                                               const Point *bombs, size_t bomb_count,
                                               size_t *bombs_on_path, size_t *bombs_on_path_count) {


  if (!bombs_on_path_count) {
    return;
  }
  *bombs_on_path_count = 0;
  if (!path || path_len == 0 || !bombs || bomb_count == 0) {
    return;
  }

  int total_cells = rows * cols;
  if (total_cells <= 0 || total_cells > PLANNER_V3_BFS_MAX_CELLS) {
    return;
  }
  uint8_t path_mark[PLANNER_V3_BFS_MAX_CELLS] = {0};
  for (size_t pi = 0; pi < path_len; ++pi) {
    if (!planner_v3_bfs_in_bounds(rows, cols, path[pi].row, path[pi].col)) {
      continue;
    }
    path_mark[path[pi].row * cols + path[pi].col] = 1;
  }

  for (size_t bi = 0; bi < bomb_count && *bombs_on_path_count < MAX_BOMBS; ++bi) {
    if (bombs[bi].row < 0 || bombs[bi].col < 0 ||
        !planner_v3_bfs_in_bounds(rows, cols, bombs[bi].row, bombs[bi].col)) {
      continue;  // ?????????????????????
    }
    int bomb_idx = bombs[bi].row * cols + bombs[bi].col;
    if (path_mark[bomb_idx]) {
      bombs_on_path[(*bombs_on_path_count)++] = bi;
    }
  }
}

/* Check first invalid bomb push position on bomb path.
 * Return {-1,-1} if no blocking push position is found.
 * out_blocker_is_box:
 *   0 => blocking by obstacle
 *   1 => blocking by box
 * out_blocker_is_bomb:
 *   1 => blocking by another bomb
 */
static Point planner_v3_bfs_check_bomb_path_push_blocker(int rows, int cols,
                                                          Point bomb_start, Point target_pos,
                                                          const Point *obstacles, size_t obstacle_count,
                                                          const Point *bombs, size_t bomb_count, size_t bomb_idx,
                                                          const Point *boxes, size_t box_count,
                                                          int *out_blocker_is_box,
                                                          int *out_blocker_is_bomb) {
  Point result = {-1, -1};
  if (out_blocker_is_box) {
    *out_blocker_is_box = 0;
  }
  if (out_blocker_is_bomb) {
    *out_blocker_is_bomb = 0;
  }

  PlannerV3ProbeScratch *probe_scratch = &s_probe_scratch;

  Point *temp_bombs = probe_scratch->temp_bombs;
  size_t temp_bomb_count = 0;
  for (size_t i = 0; i < bomb_count && temp_bomb_count < MAX_BOMBS; ++i) {
    if (i == bomb_idx) continue;
    if (bombs[i].row < 0 || bombs[i].col < 0) continue;
    temp_bombs[temp_bomb_count++] = bombs[i];
  }

  Point *filtered_obstacles = probe_scratch->temp_obstacles;
  size_t filtered_count = 0;
  for (size_t i = 0; i < obstacle_count && filtered_count < 200; ++i) {
    if (obstacles[i].row == target_pos.row && obstacles[i].col == target_pos.col) {
      continue;
    }
    filtered_obstacles[filtered_count++] = obstacles[i];
  }

  int dist_map[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target_pos,
                                             filtered_obstacles, filtered_count,
                                             temp_bombs, temp_bomb_count,
                                             boxes, box_count,
                                             dist_map, 1, NULL)) {
    return result;
  }

  Point *bomb_path = probe_scratch->temp_path;
  size_t bomb_path_len = 0;
  if (!planner_v3_bfs_astar_with_dist(rows, cols, bomb_start, target_pos,
                                      filtered_obstacles, filtered_count,
                                      temp_bombs, temp_bomb_count,
                                      boxes, box_count,
                                      dist_map,
                                      bomb_path, PLANNER_V3_BFS_MAX_PATH_LEN, &bomb_path_len,
                                      1,
                                      0, 0, (Point){0, 0}, 0, NULL, NULL, 0, NULL)) {
    return result;
  }

  for (size_t i = 1; i < bomb_path_len; ++i) {
    if (!planner_v3_bfs_should_check_push_step(bomb_path, i)) {
      continue;
    }
    Point from = bomb_path[i - 1];
    Point to = bomb_path[i];
    Point push_pos = (Point){from.row - (to.row - from.row), from.col - (to.col - from.col)};

    if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col)) {
      continue;
    }
    if (push_pos.row == target_pos.row && push_pos.col == target_pos.col) {
      continue;
    }

    for (size_t oi = 0; oi < obstacle_count; ++oi) {
      if (obstacles[oi].row == push_pos.row && obstacles[oi].col == push_pos.col) {
        if (out_blocker_is_box) {
          *out_blocker_is_box = 0;
        }
        if (out_blocker_is_bomb) {
          *out_blocker_is_bomb = 0;
        }
        return push_pos;
      }
    }
    for (size_t bi = 0; bi < bomb_count; ++bi) {
      if (bi == bomb_idx) {
        continue;
      }
      if (bombs[bi].row < 0 || bombs[bi].col < 0) {
        continue;
      }
      if (bombs[bi].row == push_pos.row && bombs[bi].col == push_pos.col) {
        if (out_blocker_is_box) {
          *out_blocker_is_box = 0;
        }
        if (out_blocker_is_bomb) {
          *out_blocker_is_bomb = 1;
        }
        return push_pos;
      }
    }
    for (size_t bi = 0; bi < box_count; ++bi) {
      if (boxes[bi].row < 0 || boxes[bi].col < 0) {
        continue;
      }
      if (boxes[bi].row == push_pos.row && boxes[bi].col == push_pos.col) {
        if (out_blocker_is_box) {
          *out_blocker_is_box = 1;
        }
        if (out_blocker_is_bomb) {
          *out_blocker_is_bomb = 0;
        }
        return push_pos;
      }
    }
  }

  return result;
}

// ??????MAX_RECURSION_DEPTH.
// ??????MAX_RECURSION_DEPTH.
// ??????MAX_RECURSION_DEPTH.
// ??????MAX_RECURSION_DEPTH.
// ??????MAX_RECURSION_DEPTH.
// ??????MAX_RECURSION_DEPTH.
// ??????MAX_RECURSION_DEPTH.
#define MAX_RECURSION_DEPTH 5
#define PLANNER_V3_BFS_MAX_EXTRA_BOMB_DISPLAY 5
static int planner_v3_bfs_clear_push_pos_obstacle_recursive(
    int rows, int cols, Point *car_pos,
    Point *bombs, size_t bomb_count, size_t bomb_idx,
    Point obstacle_on_push_pos,
    Point *obstacles, size_t *obstacle_count,
    const Point *boxes, size_t box_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps,
    int recursion_depth,
    Point *extra_exploded_out, size_t extra_cap, size_t *extra_count_out) {
  
  if (recursion_depth >= MAX_RECURSION_DEPTH) {
    return 0;  // ???????????
  }

  Point bomb_a = bombs[bomb_idx];
  size_t bomb_b_order[MAX_BOMBS];
  size_t bomb_b_count = 0;
  int bomb_b_dist_to_a[MAX_BOMBS];

  for (size_t bi = 0; bi < bomb_count && bomb_b_count < MAX_BOMBS; ++bi) {
    if (bi == bomb_idx) continue;
    if (bombs[bi].row < 0 || bombs[bi].col < 0) continue;
    bomb_b_order[bomb_b_count] = bi;

    Point temp_bombs[MAX_BOMBS];
    size_t tb = 0;
    for (size_t bj = 0; bj < bomb_count && tb < MAX_BOMBS; ++bj) {
      if (bj == bi || bj == bomb_idx) continue;
      if (bombs[bj].row < 0 || bombs[bj].col < 0) continue;
      temp_bombs[tb++] = bombs[bj];
    }
    Point filtered_obstacles[200];
    size_t nf = 0;
    for (size_t i = 0; i < *obstacle_count && nf < 200; ++i) {
      filtered_obstacles[nf++] = obstacles[i];
    }
    int d = planner_v3_bfs_distance_between(rows, cols, bombs[bi], bomb_a,
        filtered_obstacles, nf, temp_bombs, tb, boxes, box_count, 1, NULL);
    bomb_b_dist_to_a[bomb_b_count] = (d == INT_MAX) ? INT_MAX : d;
    bomb_b_count++;
  }


  for (size_t i = 0; i < bomb_b_count; ++i) {
    for (size_t j = i + 1; j < bomb_b_count; ++j) {
      if (bomb_b_dist_to_a[j] < bomb_b_dist_to_a[i]) {
        size_t ti = bomb_b_order[i];
        bomb_b_order[i] = bomb_b_order[j];
        bomb_b_order[j] = ti;
        int td = bomb_b_dist_to_a[i];
        bomb_b_dist_to_a[i] = bomb_b_dist_to_a[j];
        bomb_b_dist_to_a[j] = td;
      }
    }
  }

  for (size_t bo = 0; bo < bomb_b_count; ++bo) {
    size_t bi = bomb_b_order[bo];
    Point candidate_targets[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
    int candidate_dist[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
    size_t candidate_count = planner_v3_bfs_collect_bomb_centers_covering_obstacle(
        rows, cols, obstacle_on_push_pos,
        obstacles, *obstacle_count,
        candidate_targets, PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS);
    if (candidate_count == 0) {
      continue;
    }


    Point temp_bombs[MAX_BOMBS];
    size_t temp_bomb_count = 0;
    for (size_t bj = 0; bj < bomb_count && temp_bomb_count < MAX_BOMBS; ++bj) {
      if (bj == bi) continue;
      if (bombs[bj].row < 0 || bombs[bj].col < 0) continue;
      temp_bombs[temp_bomb_count++] = bombs[bj];
    }
    for (size_t ct = 0; ct < candidate_count; ++ct) {
      Point cand = candidate_targets[ct];
      Point filtered_obstacles[200];
      size_t nf = 0;
      for (size_t i = 0; i < *obstacle_count && nf < 200; ++i) {
        if (obstacles[i].row == cand.row && obstacles[i].col == cand.col) continue;
        filtered_obstacles[nf++] = obstacles[i];
      }
      int d = planner_v3_bfs_distance_between(rows, cols, bombs[bi], cand,
          filtered_obstacles, nf, temp_bombs, temp_bomb_count, boxes, box_count, 1, NULL);
      candidate_dist[ct] = (d == INT_MAX) ? INT_MAX : d;
    }
    for (size_t i = 0; i < candidate_count; ++i) {
      for (size_t j = i + 1; j < candidate_count; ++j) {
        if (candidate_dist[j] < candidate_dist[i]) {
          int td = candidate_dist[i];
          candidate_dist[i] = candidate_dist[j];
          candidate_dist[j] = td;
          Point tp = candidate_targets[i];
          candidate_targets[i] = candidate_targets[j];
          candidate_targets[j] = tp;
        }
      }
    }

    for (size_t ct = 0; ct < candidate_count; ++ct) {
      Point cand_target = candidate_targets[ct];

      int bomb_b_blocker_is_box = 0;
      int bomb_b_blocker_is_bomb = 0;
      Point bomb_b_push_obs = planner_v3_bfs_check_bomb_path_push_blocker(
          rows, cols, bombs[bi], cand_target,
          obstacles, *obstacle_count, bombs, bomb_count, bi, boxes, box_count,
          &bomb_b_blocker_is_box, &bomb_b_blocker_is_bomb);
      if (bomb_b_push_obs.row >= 0 && (bomb_b_blocker_is_box || bomb_b_blocker_is_bomb)) {
        continue;
      }

      if (bomb_b_push_obs.row < 0) {

        Point saved_obstacles[200];
        size_t saved_obstacle_count = *obstacle_count;
        memcpy(saved_obstacles, obstacles, saved_obstacle_count * sizeof(Point));
        Point saved_bombs[MAX_BOMBS];
        memcpy(saved_bombs, bombs, bomb_count * sizeof(Point));
        Point saved_car = *car_pos;
        size_t saved_steps = *out_steps;

        int push_result = planner_v3_bfs_push_bomb(rows, cols, car_pos,
                                                    bombs, bomb_count, bi, cand_target,
                                                    obstacle_on_push_pos,
                                                    obstacles, *obstacle_count,
                                                    boxes, box_count,
                                                    path_buffer, path_capacity, out_steps,
                                                    NULL, 0);
        if (push_result == 1) {
          if (extra_exploded_out && extra_count_out && *extra_count_out < extra_cap) {
            extra_exploded_out[*extra_count_out] = cand_target;
            (*extra_count_out)++;
          }
          Point new_obstacles[200];
          size_t new_obstacle_count = planner_v3_bfs_bomb_explode(obstacles, *obstacle_count,
                                                                   cand_target, new_obstacles, 200);
          memcpy(obstacles, new_obstacles, new_obstacle_count * sizeof(Point));
          *obstacle_count = new_obstacle_count;
          bombs[bi].row = -1;
          bombs[bi].col = -1;
          return 1;
        }

        memcpy(obstacles, saved_obstacles, saved_obstacle_count * sizeof(Point));
        *obstacle_count = saved_obstacle_count;
        memcpy(bombs, saved_bombs, bomb_count * sizeof(Point));
        *car_pos = saved_car;
        *out_steps = saved_steps;
      } else {

        Point saved_obstacles[200];
        size_t saved_obstacle_count = *obstacle_count;
        memcpy(saved_obstacles, obstacles, saved_obstacle_count * sizeof(Point));
        Point saved_bombs[MAX_BOMBS];
        memcpy(saved_bombs, bombs, bomb_count * sizeof(Point));
        Point saved_car = *car_pos;
        size_t saved_steps = *out_steps;

        if (planner_v3_bfs_clear_push_pos_obstacle_recursive(
                rows, cols, car_pos, bombs, bomb_count, bi,
                bomb_b_push_obs,
                obstacles, obstacle_count, boxes, box_count,
                path_buffer, path_capacity, out_steps,
                recursion_depth + 1,
                extra_exploded_out, extra_cap, extra_count_out)) {
          int push_result = planner_v3_bfs_push_bomb(rows, cols, car_pos,
                                                      bombs, bomb_count, bi, cand_target,
                                                      obstacle_on_push_pos,
                                                      obstacles, *obstacle_count,
                                                      boxes, box_count,
                                                      path_buffer, path_capacity, out_steps,
                                                      NULL, 0);
          if (push_result == 1) {
            if (extra_exploded_out && extra_count_out && *extra_count_out < extra_cap) {
              extra_exploded_out[*extra_count_out] = cand_target;
              (*extra_count_out)++;
            }
            Point new_obstacles[200];
            size_t new_obstacle_count = planner_v3_bfs_bomb_explode(obstacles, *obstacle_count,
                                                                     cand_target, new_obstacles, 200);
            memcpy(obstacles, new_obstacles, new_obstacle_count * sizeof(Point));
            *obstacle_count = new_obstacle_count;
            bombs[bi].row = -1;
            bombs[bi].col = -1;
            return 1;
          }

          memcpy(obstacles, saved_obstacles, saved_obstacle_count * sizeof(Point));
          *obstacle_count = saved_obstacle_count;
          memcpy(bombs, saved_bombs, bomb_count * sizeof(Point));
          *car_pos = saved_car;
          *out_steps = saved_steps;
        }
      }
    }
  }

  return 0;  // ???????????
}

// ?????????????
// ?????????????
static void planner_v3_bfs_bombs_sorted_by_dist_to_obstacle(int rows, int cols,
    Point obstacle_to_clear,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count,
    size_t *bomb_order, size_t *bomb_order_count,
    const Point *path_obstacles, size_t path_obstacle_count) {
  (void)boxes;
  (void)box_count;

  *bomb_order_count = 0;
  if (bomb_count == 0 || !bombs) {
    return;
  }

  Point legal_centers[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
  size_t legal_center_count = 0;
  if (path_obstacles && path_obstacle_count > 1) {
    legal_center_count = planner_v3_bfs_obstacles_destroyable_by_one_bomb(
        path_obstacles, path_obstacle_count,
        obstacles, obstacle_count,
        legal_centers, PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS);
  } else {
    legal_center_count = planner_v3_bfs_collect_bomb_centers_covering_obstacle(
        rows, cols, obstacle_to_clear,
        obstacles, obstacle_count,
        legal_centers, PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS);
  }
  if (legal_center_count == 0) {
    legal_centers[0] = obstacle_to_clear;
    legal_center_count = 1;
  }

  int dist_to_obs[MAX_BOMBS];
  for (size_t i = 0; i < bomb_count && i < MAX_BOMBS; ++i) {
    dist_to_obs[i] = INT_MAX;
    if (bombs[i].row < 0 || bombs[i].col < 0) {
      continue;
    }
    int best_center_d = INT_MAX;
    for (size_t ci = 0; ci < legal_center_count; ++ci) {
      int d = planner_v3_bfs_abs(bombs[i].row - legal_centers[ci].row) +
              planner_v3_bfs_abs(bombs[i].col - legal_centers[ci].col);
      if (d < best_center_d) {
        best_center_d = d;
      }
    }
    dist_to_obs[i] = best_center_d;
  }

  for (size_t n = 0; n < bomb_count && *bomb_order_count < MAX_BOMBS; ++n) {
    int best_d = INT_MAX;
    size_t best_i = SIZE_MAX;
    for (size_t i = 0; i < bomb_count; ++i) {
      int already = 0;
      for (size_t k = 0; k < *bomb_order_count; ++k) {
        if (bomb_order[k] == i) { already = 1; break; }
      }
      if (already) continue;
      if (dist_to_obs[i] < best_d) {
        best_d = dist_to_obs[i];
        best_i = i;
      }
    }
    if (best_i == SIZE_MAX) break;
    bomb_order[(*bomb_order_count)++] = best_i;
  }
}

// ?????????????
// ?????????????
// ?????????????
// ?????????????
static void planner_v3_bfs_targets_sorted_by_reachable_dist(int rows, int cols, Point car,
    size_t bomb_idx, const Point *bombs, size_t bomb_count,
    Point obstacle_to_clear,
    const Point *obstacles, size_t obstacle_count,
    const Point *boxes, size_t box_count,
    Point *target_positions, size_t *target_count,
    const Point *path_obstacles, size_t path_obstacle_count,
    int apply_req1) {
  *target_count = 0;
  uint8_t path_obstacle_mark[PLANNER_V3_BFS_MAX_CELLS] = {0};
  {
    int total_cells = rows * cols;
    if (total_cells <= 0 || total_cells > PLANNER_V3_BFS_MAX_CELLS) {
      return;
    }
    if (path_obstacles && path_obstacle_count > 0) {
      for (size_t i = 0; i < path_obstacle_count; ++i) {
        if (!planner_v3_bfs_in_bounds(rows, cols, path_obstacles[i].row, path_obstacles[i].col)) {
          continue;
        }
        path_obstacle_mark[path_obstacles[i].row * cols + path_obstacles[i].col] = 1;
      }
    }
  }
  Point temp_bombs[MAX_BOMBS];
  size_t temp_bomb_count = 0;
  for (size_t i = 0; i < bomb_count && temp_bomb_count < MAX_BOMBS; ++i) {
    if (i == bomb_idx) continue;
    if (bombs[i].row < 0 || bombs[i].col < 0) continue;
    temp_bombs[temp_bomb_count++] = bombs[i];
  }

  Point candidates[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
  int cand_dist[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
  size_t cand_count = 0;



  Point candidate_points[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
  size_t num_candidate_points = 0;
  if (path_obstacles && path_obstacle_count == 1) {
    // 单障碍分支：候选点=特殊路径经过的障碍 + 其 3x3 范围内所有障碍。
    Point anchor = path_obstacles[0];
    if (planner_v3_bfs_in_bounds(rows, cols, anchor.row, anchor.col) &&
        planner_v3_bfs_point_in_list(anchor, obstacles, obstacle_count)) {
      candidate_points[num_candidate_points++] = anchor;
    }
    for (int dr = -1; dr <= 1 && num_candidate_points < PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS; ++dr) {
      for (int dc = -1; dc <= 1 && num_candidate_points < PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS; ++dc) {
        int nr = anchor.row + dr;
        int nc = anchor.col + dc;
        Point c = (Point){nr, nc};
        if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc)) {
          continue;
        }
        if (!planner_v3_bfs_point_in_list(c, obstacles, obstacle_count)) {
          continue;
        }
        if (planner_v3_bfs_point_in_list(c, candidate_points, num_candidate_points)) {
          continue;
        }
        candidate_points[num_candidate_points++] = c;
      }
    }
  } else {
    num_candidate_points = planner_v3_bfs_collect_bomb_centers_covering_obstacle(
        rows, cols, obstacle_to_clear,
        obstacles, obstacle_count,
        candidate_points, PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS);
  }

  for (size_t ci = 0; ci < num_candidate_points && cand_count < PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS; ++ci) {
    Point cand = candidate_points[ci];
    Point filtered_for_cand[200];
    size_t nf = 0;
    for (size_t i = 0; i < obstacle_count && nf < 200; ++i) {
      int exclude = 0;
      if (obstacles[i].row == cand.row && obstacles[i].col == cand.col) {
        exclude = 1;
      } else if (path_obstacles && path_obstacle_count > 0 &&
                 planner_v3_bfs_in_bounds(rows, cols, obstacles[i].row, obstacles[i].col) &&
                 path_obstacle_mark[obstacles[i].row * cols + obstacles[i].col]) {
        exclude = 1;
      }
      if (!exclude) {
        filtered_for_cand[nf++] = obstacles[i];
      }
    }

    int dist_map[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_target(rows, cols, cand,
            filtered_for_cand, nf, temp_bombs, temp_bomb_count,
            boxes, box_count, dist_map, 1, NULL)) {
      continue;
    }
    int bomb_start_idx = bombs[bomb_idx].row * cols + bombs[bomb_idx].col;
    int d0 = dist_map[bomb_start_idx];
    Point bomb_path[PLANNER_V3_BFS_MAX_PATH_LEN];
    size_t bomb_path_len = 0;

    int astar_ok = planner_v3_bfs_astar_with_dist(rows, cols, bombs[bomb_idx], cand,
            filtered_for_cand, nf, temp_bombs, temp_bomb_count,
            boxes, box_count, dist_map,
            bomb_path, PLANNER_V3_BFS_MAX_PATH_LEN, &bomb_path_len,
            1,
            1, 1, car, 0, NULL, NULL, 0, NULL);
    int push_blocker_is_box = 0;
    int push_blocker_is_bomb = 0;
    Point push_blocker = planner_v3_bfs_check_bomb_path_push_blocker(
        rows, cols, bombs[bomb_idx], cand,
        obstacles, obstacle_count, bombs, bomb_count, bomb_idx,
        boxes, box_count,
        &push_blocker_is_box, &push_blocker_is_bomb);
    if (push_blocker.row >= 0 && (push_blocker_is_box || push_blocker_is_bomb)) {
      continue;  // 推位被箱子/炸弹阻挡：放弃该炸弹目标候选
    }
    if (!astar_ok) {

      int d = d0;
      if (d == INT_MAX) continue;
      if (push_blocker.row < 0) continue;  // 非推位障碍导致的失败，放弃该目标
      if (apply_req1) {
        continue;  // ??????1??????��????????????????
      }

    }

    int d = d0;
    candidates[cand_count] = cand;
    cand_dist[cand_count] = (d == INT_MAX) ? INT_MAX : d;
    cand_count++;
  }

  uint8_t used[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS] = {0};
  for (size_t n = 0; n < cand_count; ++n) {
    int best_d = INT_MAX;
    size_t best_j = SIZE_MAX;
    for (size_t j = 0; j < cand_count; ++j) {
      if (used[j]) continue;
      if (cand_dist[j] < best_d) {
        best_d = cand_dist[j];
        best_j = j;
      }
    }
    if (best_j == SIZE_MAX) break;
    used[best_j] = 1;
    target_positions[(*target_count)++] = candidates[best_j];
  }
}

// ?????????????
// ?????????????
// ?????????????
static int planner_v3_bfs_simulate_push_bomb(int rows, int cols, Point car_start,
                                             Point bomb_start, Point target_pos,
                                             Point target_obstacle,
                                             const Point *filtered_obstacles, size_t filtered_obstacle_count,
                                             const Point *obstacles_for_car, size_t obstacle_count_for_car,
                                             const Point *temp_bombs, size_t temp_bomb_count,
                                             const Point *temp_boxes, size_t temp_box_count,
                                             const Point *bomb_path, size_t bomb_path_len,
                                             int has_bomb_path, int strategy) {
  (void)target_obstacle;
  Point car = car_start;
  Point bomb = bomb_start;
  size_t steps = 0;
  size_t bomb_path_idx = 0;
  int last_dr = 0;
  int last_dc = 0;
  int reverse_count = 0;
  uint8_t occ_buf[PLANNER_V3_BFS_MAX_CELLS];
  planner_v3_bfs_build_occupancy_grid(rows, cols, filtered_obstacles, filtered_obstacle_count, temp_bombs, temp_bomb_count, temp_boxes, temp_box_count, 1, occ_buf);

  int dist_to_target[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target_pos,
                                             filtered_obstacles, filtered_obstacle_count,
                                             temp_bombs, temp_bomb_count,
                                             temp_boxes, temp_box_count,
                                             dist_to_target, 1, occ_buf)) {
    return INT_MAX;
  }
  
  while (bomb.row != target_pos.row || bomb.col != target_pos.col) {
    if (steps >= PLANNER_V3_BFS_MAX_GREEDY_STEPS) {
      return INT_MAX;
    }
    
    if (strategy == PLANNER_V3_BFS_STRATEGY_PATH && has_bomb_path) {

      if (bomb_path_idx + 1 >= bomb_path_len) {
        return INT_MAX;
      }
      Point next_in_path = bomb_path[bomb_path_idx + 1];
      int dr = next_in_path.row - bomb.row;
      int dc = next_in_path.col - bomb.col;
      
      if (planner_v3_bfs_abs(dr) + planner_v3_bfs_abs(dc) != 1) {
        return INT_MAX;
      }

      int push_row = bomb.row - (next_in_path.row - bomb.row);
      int push_col = bomb.col - (next_in_path.col - bomb.col);
      
      if (!planner_v3_bfs_in_bounds(rows, cols, next_in_path.row, next_in_path.col) ||
          !planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
        return INT_MAX;
      }
      
      if (planner_v3_bfs_is_obstacle(occ_buf, cols, next_in_path.row, next_in_path.col) ||
          planner_v3_bfs_is_obstacle(occ_buf, cols, push_row, push_col)) {
        return INT_MAX;
      }
      
      if (planner_v3_bfs_is_box_at(occ_buf, cols, next_in_path.row, next_in_path.col, SIZE_MAX) ||
          planner_v3_bfs_is_box_at(occ_buf, cols, push_row, push_col, SIZE_MAX)) {
        return INT_MAX;
      }
      
      Point push_from = {push_row, push_col};

      Point bombs_with_current[MAX_BOMBS];
      size_t bwc = 0;
      for (size_t ii = 0; ii < temp_bomb_count && bwc < MAX_BOMBS; ii++) bombs_with_current[bwc++] = temp_bombs[ii];
      if (bwc < MAX_BOMBS) bombs_with_current[bwc++] = bomb;
      int car_dist = planner_v3_bfs_distance_between(rows, cols, car, push_from,
                                                      obstacles_for_car, obstacle_count_for_car,
                                                      bombs_with_current, bwc,
                                                      temp_boxes, temp_box_count, 1, NULL);
      if (car_dist == INT_MAX) {
        return INT_MAX;
      }
      
      steps += (size_t)car_dist + 1;
      car = bomb;
      bomb = next_in_path;
      bomb_path_idx++;
      last_dr = dr;
      last_dc = dc;
      continue;
    }
    

    Point bombs_with_current[MAX_BOMBS];
    size_t bwc = 0;
    for (size_t ii = 0; ii < temp_bomb_count && bwc < MAX_BOMBS; ii++) bombs_with_current[bwc++] = temp_bombs[ii];
    if (bwc < MAX_BOMBS) bombs_with_current[bwc++] = bomb;
    int dist_from_car[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_source(rows, cols, car,
                                               obstacles_for_car, obstacle_count_for_car,
                                               bombs_with_current, bwc,
                                               temp_boxes, temp_box_count,
                                               dist_from_car, 1, NULL)) {
      return INT_MAX;
    }

    int sorted_dirs[4];
    int sorted_count = 0;
    PlannerV3PushScoreCandidate candidates[4];
    if (!planner_v3_bfs_evaluate_push_candidates_score(
            rows, cols, bomb, target_pos, SIZE_MAX, occ_buf,
            dist_to_target, dist_from_car,
            0, last_dr, last_dc, reverse_count,
            candidates, sorted_dirs, &sorted_count)) {
      return INT_MAX;
    }
    

    PlannerV3PushScoreCandidate chosen;
    int car_dist = INT_MAX;
    for (int k = 0; k < sorted_count; ++k) {
      chosen = candidates[sorted_dirs[k]];
      int next_on_boundary = (chosen.next_pos.row == 0 || chosen.next_pos.row == rows - 1 ||
                              chosen.next_pos.col == 0 || chosen.next_pos.col == cols - 1);
      if (next_on_boundary) {
        if (!planner_v3_bfs_validate_boundary_path(rows, cols, chosen.next_pos, target_pos,
                                                   car, filtered_obstacles, filtered_obstacle_count,
                                                   temp_bombs, temp_bomb_count, temp_boxes, temp_box_count,
                                                   SIZE_MAX, 1, occ_buf)) {
          continue;  // ?????????????????????
        }
      }
      if (chosen.next_pos.row != target_pos.row || chosen.next_pos.col != target_pos.col) {
        if (!planner_v3_bfs_can_reach_goal(rows, cols,
                                           filtered_obstacles, filtered_obstacle_count,
                                           temp_bombs, temp_bomb_count,
                                           temp_boxes, temp_box_count,
                                           SIZE_MAX, chosen.next_pos, target_pos, 1, occ_buf)) {
          continue;
        }
      }
      car_dist = dist_from_car[chosen.push_from.row * cols + chosen.push_from.col];
      if (car_dist == INT_MAX) {
        continue;
      }
      break;
    }
    if (car_dist == INT_MAX) {
      return INT_MAX;
    }
    
    steps += (size_t)car_dist + 1;
    car = chosen.push_from;
    bomb = chosen.next_pos;
    
    if (last_dr == -chosen.dr && last_dc == -chosen.dc &&
        (last_dr != 0 || last_dc != 0)) {
      reverse_count++;
    } else {
      reverse_count = 0;
    }
    
    last_dr = chosen.dr;
    last_dc = chosen.dc;
  }
  
  return (int)steps;
}

// ?????????????
// ?????????????
static int planner_v3_bfs_simulate_push_box_score(
    int rows, int cols, Point car_start, Point box_start, Point target,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count, size_t box_idx,
    int apply_req3) {
  Point car = car_start;
  Point box = box_start;
  size_t steps = 0;
  int last_dr = 0, last_dc = 0;
  int reverse_count = 0;
  
  Point boxes_local[PLANNER_V3_BFS_MAX_BOXES];
  size_t n = box_count < PLANNER_V3_BFS_MAX_BOXES ? box_count : PLANNER_V3_BFS_MAX_BOXES;
  for (size_t i = 0; i < n; ++i) {
    boxes_local[i] = boxes[i];
  }
  
  Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
  size_t temp_count = 0;
  for (size_t i = 0; i < n; ++i) {
    if (i == box_idx) continue;
    if (boxes[i].row < 0 || boxes[i].col < 0) continue;
    temp_boxes[temp_count++] = boxes[i];
  }

  int dist_to_target[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
        bombs, bomb_count, temp_boxes, temp_count, dist_to_target, 1, NULL)) {
    return INT_MAX;
  }
  
  while (box.row != target.row || box.col != target.col) {
    if (steps >= PLANNER_V3_BFS_MAX_GREEDY_STEPS) return INT_MAX;
    int is_first_push_step = (steps == 0);
    uint8_t occ_buf[PLANNER_V3_BFS_MAX_CELLS];
    planner_v3_bfs_build_occupancy_grid(rows, cols, obstacles, obstacle_count, bombs, bomb_count, boxes_local, n, 1, occ_buf);

    int dist_from_car[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_source(rows, cols, car, obstacles, obstacle_count,
          bombs, bomb_count, boxes_local, n, dist_from_car, 1, occ_buf)) {
      return INT_MAX;
    }
    PlannerV3PushScoreCandidate candidates[4];
    int order[4], cnt = 0;
    if (!planner_v3_bfs_evaluate_push_candidates_score(
            rows, cols, box, target, box_idx, occ_buf,
            dist_to_target, dist_from_car,
            0, last_dr, last_dc, reverse_count,
            candidates, order, &cnt)) {
      return INT_MAX;
    }

    PlannerV3PushScoreCandidate chosen;
    int car_dist = INT_MAX;
    for (int k = 0; k < cnt; ++k) {
      chosen = candidates[order[k]];
      int enforce_req3 = (apply_req3 || !is_first_push_step);
      int next_on_boundary = (chosen.next_pos.row == 0 || chosen.next_pos.row == rows - 1 ||
                              chosen.next_pos.col == 0 || chosen.next_pos.col == cols - 1);
      if (next_on_boundary && enforce_req3) {
        if (!planner_v3_bfs_validate_boundary_path(rows, cols, chosen.next_pos, target,
            car, obstacles, obstacle_count, bombs, bomb_count, boxes_local, n, box_idx, 1, occ_buf)) continue;
      }
      if (enforce_req3 &&
          (chosen.next_pos.row != target.row || chosen.next_pos.col != target.col)) {
        if (!planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count, bombs, bomb_count,
            boxes_local, n, box_idx, chosen.next_pos, target, 1, occ_buf)) continue;
      }
      car_dist = dist_from_car[chosen.push_from.row * cols + chosen.push_from.col];
      if (car_dist == INT_MAX) {
        if (enforce_req3) continue;
        car_dist = 0;
      }
      break;
    }
    if (car_dist == INT_MAX) return INT_MAX;
    
    steps += (size_t)car_dist + 1;
    car = box;
    box = chosen.next_pos;
    boxes_local[box_idx] = box;
    if (last_dr == -chosen.dr && last_dc == -chosen.dc && (last_dr != 0 || last_dc != 0))
      reverse_count++;
    else reverse_count = 0;
    last_dr = chosen.dr; last_dc = chosen.dc;
  }
  return (int)steps;
}

// ?????????????
// ?????????????
static int planner_v3_bfs_simulate_push_box_path(
    int rows, int cols, Point car_start, Point box_start, Point target,
    const Point *box_path, size_t box_path_len,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count, size_t box_idx,
    int apply_req3) {
  if (box_path_len < 2) return INT_MAX;
  Point car = car_start;
  Point box = box_start;
  size_t steps = 0;
  size_t path_idx = 0;
  
  Point boxes_local[PLANNER_V3_BFS_MAX_BOXES];
  size_t n = box_count < PLANNER_V3_BFS_MAX_BOXES ? box_count : PLANNER_V3_BFS_MAX_BOXES;
  for (size_t i = 0; i < n; ++i) boxes_local[i] = boxes[i];
  
  Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
  size_t temp_count = 0;
  for (size_t i = 0; i < n; ++i) {
    if (i == box_idx) continue;
    if (boxes[i].row < 0 || boxes[i].col < 0) continue;
    temp_boxes[temp_count++] = boxes[i];
  }
  
  while (box.row != target.row || box.col != target.col) {
    if (steps >= PLANNER_V3_BFS_MAX_GREEDY_STEPS) return INT_MAX;
    if (path_idx + 1 >= box_path_len) return INT_MAX;
    int is_first_push_step = (path_idx == 0);
    int enforce_req3 = (apply_req3 || !is_first_push_step);
    uint8_t occ_buf[PLANNER_V3_BFS_MAX_CELLS];
    planner_v3_bfs_build_occupancy_grid(rows, cols, obstacles, obstacle_count, bombs, bomb_count, boxes_local, n, 1, occ_buf);
    Point next = box_path[path_idx + 1];
    int dr = next.row - box.row, dc = next.col - box.col;
    if (planner_v3_bfs_abs(dr) + planner_v3_bfs_abs(dc) != 1) return INT_MAX;
    int pr = box.row - (next.row - box.row), pc = box.col - (next.col - box.col);
    if (!planner_v3_bfs_in_bounds(rows, cols, next.row, next.col) ||
        !planner_v3_bfs_in_bounds(rows, cols, pr, pc)) return INT_MAX;
    if (planner_v3_bfs_is_obstacle(occ_buf, cols, next.row, next.col)) return INT_MAX;
    if (enforce_req3 && planner_v3_bfs_is_obstacle(occ_buf, cols, pr, pc)) return INT_MAX;
    if (planner_v3_bfs_is_box_at(occ_buf, cols, next.row, next.col, box_idx)) return INT_MAX;
    if (enforce_req3 && planner_v3_bfs_is_box_at(occ_buf, cols, pr, pc, box_idx)) return INT_MAX;
    
    int next_on_boundary = (next.row == 0 || next.row == rows - 1 || next.col == 0 || next.col == cols - 1);
    if (next_on_boundary && enforce_req3) {
      if (!planner_v3_bfs_validate_boundary_path(rows, cols, next, target,
          car, obstacles, obstacle_count, bombs, bomb_count, boxes_local, n, box_idx, 1, occ_buf)) return INT_MAX;
    }
    int car_dist = planner_v3_bfs_distance_between(rows, cols, car, (Point){pr, pc},
        obstacles, obstacle_count, bombs, bomb_count, boxes_local, n, 1, occ_buf);
    if (car_dist == INT_MAX) {
      if (enforce_req3) return INT_MAX;
      car_dist = 0;
    }
    steps += (size_t)car_dist + 1;
    car = box;
    box = next;
    boxes_local[box_idx] = box;
    path_idx++;
  }
  return (int)steps;
}

// ?????????????
static int planner_v3_bfs_evaluate_push_strategy(
    int has_path, int path_steps, int score_steps,
    int only_path_mode, int strict_selected_check, int prefer_path_on_tie,
    int err_stage,
    int err_detail_only_path_invalid,
    int err_detail_both_failed,
    int err_detail_path_invalid,
    int err_detail_score_invalid,
    int *out_strategy) {
  int valid_path = has_path && (path_steps != INT_MAX);
  int strategy = PLANNER_V3_BFS_STRATEGY_SCORE;
  if (only_path_mode) {
    if (!valid_path) {
      last_err_stage = err_stage;
      last_err_detail = err_detail_only_path_invalid;
      planner_v3_bfs_record_stage3_detail_if_needed(err_stage, err_detail_only_path_invalid);
      return -6;
    }
    strategy = PLANNER_V3_BFS_STRATEGY_PATH;
  } else if (valid_path &&
             (prefer_path_on_tie ? (path_steps <= score_steps) : (path_steps < score_steps))) {
    strategy = PLANNER_V3_BFS_STRATEGY_PATH;
  }

  if (path_steps == INT_MAX && score_steps == INT_MAX) {
    last_err_stage = err_stage;
    last_err_detail = err_detail_both_failed;
    planner_v3_bfs_record_stage3_detail_if_needed(err_stage, err_detail_both_failed);
    return -6;
  }

  if (strict_selected_check) {
    if (strategy == PLANNER_V3_BFS_STRATEGY_PATH && path_steps == INT_MAX) {
      last_err_stage = err_stage;
      last_err_detail = err_detail_path_invalid;
      planner_v3_bfs_record_stage3_detail_if_needed(err_stage, err_detail_path_invalid);
      return -6;
    }
    if (strategy == PLANNER_V3_BFS_STRATEGY_SCORE && score_steps == INT_MAX) {
      last_err_stage = err_stage;
      last_err_detail = err_detail_score_invalid;
      planner_v3_bfs_record_stage3_detail_if_needed(err_stage, err_detail_score_invalid);
      return -6;
    }
  }

  *out_strategy = strategy;
  return 1;
}

typedef struct {
  Point expected_push_from;
  Point next_pos;
  Point *car_pos;
  Point *moving_pos;
  Point *moving_array;
  size_t moving_idx;
  Point *path_buffer;
  size_t path_capacity;
  size_t *out_steps;
  int err_stage;
  int err_detail_car_mismatch;
  int err_detail_not_adjacent;
  int err_detail_path_not_continuous;
} PlannerV3PushExecuteCtx;

// ?????????????
static int planner_v3_bfs_execute_push_strategy(PlannerV3PushExecuteCtx *ctx) {
  if (!ctx || !ctx->car_pos || !ctx->moving_pos || !ctx->out_steps || !ctx->path_buffer) {
    return -6;
  }
  if (ctx->car_pos->row != ctx->expected_push_from.row ||
      ctx->car_pos->col != ctx->expected_push_from.col) {
    last_err_stage = ctx->err_stage;
    last_err_detail = ctx->err_detail_car_mismatch;
    planner_v3_bfs_record_stage3_detail_if_needed(ctx->err_stage, ctx->err_detail_car_mismatch);
    return -6;
  }
  if (!planner_v3_bfs_check_adjacent(*ctx->car_pos, *ctx->moving_pos)) {
    last_err_stage = ctx->err_stage;
    last_err_detail = ctx->err_detail_not_adjacent;
    planner_v3_bfs_record_stage3_detail_if_needed(ctx->err_stage, ctx->err_detail_not_adjacent);
    return -6;
  }

  Point moving_old_pos = *ctx->moving_pos;
  *ctx->moving_pos = ctx->next_pos;
  if (ctx->moving_array) {
    ctx->moving_array[ctx->moving_idx] = ctx->next_pos;
  }

  *ctx->car_pos = moving_old_pos;
  if (*ctx->out_steps >= ctx->path_capacity) {
    return -7;
  }
  if (*ctx->out_steps > 0 &&
      !planner_v3_bfs_check_adjacent(ctx->path_buffer[*ctx->out_steps - 1], *ctx->car_pos)) {
    last_err_stage = ctx->err_stage;
    last_err_detail = ctx->err_detail_path_not_continuous;
    planner_v3_bfs_record_stage3_detail_if_needed(ctx->err_stage, ctx->err_detail_path_not_continuous);
    return -6;
  }
  ctx->path_buffer[(*ctx->out_steps)++] = *ctx->car_pos;
  return 1;
}

// ?????????????
// ?????????????
static int planner_v3_bfs_evaluate_push_candidates_score(
    int rows, int cols,
    Point moving_pos, Point target_pos,
    size_t moving_skip_idx,
    const uint8_t *occ,
    const int dist_to_target[PLANNER_V3_BFS_MAX_CELLS],
    const int dist_from_car[PLANNER_V3_BFS_MAX_CELLS],
    int allow_first_push_car_unreachable,
    int last_dr, int last_dc, int reverse_count,
    PlannerV3PushScoreCandidate candidates[4],
    int sorted_dirs[4], int *sorted_count) {
  (void)target_pos;
  if (!occ || !dist_to_target || !dist_from_car || !candidates || !sorted_dirs || !sorted_count) {
    return 0;
  }
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  *sorted_count = 0;
  for (int i = 0; i < 4; ++i) {
    candidates[i].dr = dirs[i][0];
    candidates[i].dc = dirs[i][1];
    candidates[i].feasible = 0;
    candidates[i].score = INT_MAX;

    int nr = moving_pos.row + dirs[i][0];
    int nc = moving_pos.col + dirs[i][1];
    int pr = moving_pos.row - dirs[i][0];
    int pc = moving_pos.col - dirs[i][1];
    if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc) ||
        !planner_v3_bfs_in_bounds(rows, cols, pr, pc)) {
      continue;
    }
    if (planner_v3_bfs_is_obstacle(occ, cols, nr, nc) ||
        planner_v3_bfs_is_obstacle(occ, cols, pr, pc)) {
      continue;
    }
    if (planner_v3_bfs_is_box_at(occ, cols, nr, nc, moving_skip_idx) ||
        planner_v3_bfs_is_box_at(occ, cols, pr, pc, moving_skip_idx)) {
      continue;
    }

    int dist_after = dist_to_target[nr * cols + nc];
    if (dist_after == INT_MAX) {
      continue;
    }
    int adj_pen = planner_v3_bfs_adjacent_blockers_occ(occ, rows, cols, moving_skip_idx, nr, nc);
    int reverse_pen = 0;
    if (last_dr == -dirs[i][0] && last_dc == -dirs[i][1]) {
      int power = 5;
      for (int p = 0; p < reverse_count; ++p) {
        power *= 5;
      }
      reverse_pen = power;
    }

    int car_to_push = dist_from_car[pr * cols + pc];
    if (car_to_push == INT_MAX) {
      if (!allow_first_push_car_unreachable) {
        continue;
      }
      car_to_push = 0;
    }

    candidates[i].dist = dist_after;
    candidates[i].adj_pen = adj_pen;
    candidates[i].score = dist_after * 13 + adj_pen * 2 + reverse_pen + car_to_push * 5;
    candidates[i].feasible = 1;
    candidates[i].push_from = (Point){pr, pc};
    candidates[i].next_pos = (Point){nr, nc};
    sorted_dirs[(*sorted_count)++] = i;
  }

  for (int i = 0; i < *sorted_count - 1; ++i) {
    for (int j = 0; j < *sorted_count - 1 - i; ++j) {
      if (candidates[sorted_dirs[j]].score > candidates[sorted_dirs[j + 1]].score) {
        int tmp = sorted_dirs[j];
        sorted_dirs[j] = sorted_dirs[j + 1];
        sorted_dirs[j + 1] = tmp;
      }
    }
  }
  return (*sorted_count > 0) ? 1 : 0;
}


// ?????????????
// ?????????????
// ?????????????
// ?????????????
// ?????????????
// ?????????????
// ?????????????
static int planner_v3_bfs_push_bomb(int rows, int cols, Point *car_pos,
                                    Point *bombs, size_t bomb_count, size_t bomb_idx,
                                    Point target_pos,
                                    Point target_obstacle,
                                    const Point *obstacles, size_t obstacle_count,
                                    const Point *boxes, size_t box_count,
                                    Point *path_buffer, size_t path_capacity,
                                    size_t *out_steps,
                                    const Point *path_obstacles_to_exclude, size_t path_obstacle_exclude_count) {
  (void)target_obstacle;
  Point bomb = bombs[bomb_idx];
  

  if (bomb.row == target_pos.row && bomb.col == target_pos.col) {
    return 1;
  }
  


  uint8_t path_obstacle_exclude_mark[PLANNER_V3_BFS_MAX_CELLS] = {0};
  {
    int total_cells = rows * cols;
    if (path_obstacles_to_exclude && path_obstacle_exclude_count > 0 &&
        total_cells > 0 && total_cells <= PLANNER_V3_BFS_MAX_CELLS) {
      for (size_t j = 0; j < path_obstacle_exclude_count; ++j) {
        if (!planner_v3_bfs_in_bounds(rows, cols,
                                      path_obstacles_to_exclude[j].row,
                                      path_obstacles_to_exclude[j].col)) {
          continue;
        }
        path_obstacle_exclude_mark[path_obstacles_to_exclude[j].row * cols +
                                   path_obstacles_to_exclude[j].col] = 1;
      }
    }
  }
  Point filtered_obstacles[200];
  size_t filtered_obstacle_count = 0;
  for (size_t i = 0; i < obstacle_count && filtered_obstacle_count < 200; ++i) {
    int exclude = 0;
    if (obstacles[i].row == target_pos.row && obstacles[i].col == target_pos.col) {
      exclude = 1;
    } else if (path_obstacles_to_exclude && path_obstacle_exclude_count > 0 &&
               planner_v3_bfs_in_bounds(rows, cols, obstacles[i].row, obstacles[i].col) &&
               path_obstacle_exclude_mark[obstacles[i].row * cols + obstacles[i].col]) {
      exclude = 1;
    }
    if (!exclude) {
      filtered_obstacles[filtered_obstacle_count++] = obstacles[i];
    }
  }
  

  Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
  size_t temp_box_count = 0;
  for (size_t i = 0; i < box_count && temp_box_count < PLANNER_V3_BFS_MAX_BOXES; ++i) {
    if (boxes[i].row < 0 || boxes[i].col < 0) {
      continue;  // ?????????????????????
    }
    temp_boxes[temp_box_count++] = boxes[i];
  }
  

  Point temp_bombs[MAX_BOMBS];
  size_t temp_bomb_count = 0;
  for (size_t i = 0; i < bomb_count && temp_bomb_count < MAX_BOMBS; ++i) {
    if (i == bomb_idx) {
      continue;  // ?????????????????????
    }
    if (bombs[i].row < 0 || bombs[i].col < 0) {
      continue;  // ?????????????????????
    }
    temp_bombs[temp_bomb_count++] = bombs[i];
  }
  

  int bomb_path_dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target_pos,
                                             filtered_obstacles, filtered_obstacle_count,
                                             temp_bombs, temp_bomb_count,
                                             temp_boxes, temp_box_count,
                                             bomb_path_dist, 1, NULL)) {
    last_err_stage = 3;  // ????????????????????
    last_err_detail = LAST_ERR_DETAIL_V3_BOMB_PUSH_TARGET_BFS_FAILED;  // ?????????????????????
    last_err_detail_v3_stage3 = LAST_ERR_DETAIL_V3_BOMB_PUSH_TARGET_BFS_FAILED;
    return -6;
  }
  

  Point bomb_path[PLANNER_V3_BFS_MAX_CELLS];
  size_t bomb_path_len = 0;
  int has_bomb_path = 0;

  if (planner_v3_bfs_astar_with_dist(rows, cols, bomb, target_pos,
                                     filtered_obstacles, filtered_obstacle_count,
                                     temp_bombs, temp_bomb_count,
                                     temp_boxes, temp_box_count,
                                     bomb_path_dist,
                                     bomb_path, PLANNER_V3_BFS_MAX_CELLS, &bomb_path_len,
                                     1,
                                     1, 1, *car_pos, 0, NULL, NULL, 0, NULL)) {
    if (bomb_path_len > 1) {
      has_bomb_path = 1;
    }
  }
  

  int path_steps = INT_MAX;
  int score_steps = INT_MAX;
  
  if (has_bomb_path) {
    path_steps = planner_v3_bfs_simulate_push_bomb(rows, cols, *car_pos, bomb, target_pos,
                                                   target_obstacle,
                                                   filtered_obstacles, filtered_obstacle_count,
                                                   obstacles, obstacle_count,
                                                   temp_bombs, temp_bomb_count,
                                                   temp_boxes, temp_box_count,
                                                   bomb_path, bomb_path_len, 1,
                                                   PLANNER_V3_BFS_STRATEGY_PATH);
  }
  
  score_steps = planner_v3_bfs_simulate_push_bomb(rows, cols, *car_pos, bomb, target_pos,
                                                  target_obstacle,
                                                  filtered_obstacles, filtered_obstacle_count,
                                                  obstacles, obstacle_count,
                                                  temp_bombs, temp_bomb_count,
                                                  temp_boxes, temp_box_count,
                                                  bomb_path, bomb_path_len, has_bomb_path,
                                                  PLANNER_V3_BFS_STRATEGY_SCORE);
  

  int chosen_strategy = PLANNER_V3_BFS_STRATEGY_SCORE;
  if (planner_v3_bfs_evaluate_push_strategy(
          has_bomb_path, path_steps, score_steps,
          0, 0, 0,
          3,
          LAST_ERR_DETAIL_V3_BOMB_PATH_ONLY_INVALID,
          LAST_ERR_DETAIL_V3_BOMB_PATH_AND_SCORE_BOTH_FAILED,
          0, 0,
          &chosen_strategy) < 0) {
    return -6;
  }
  

  size_t bomb_path_idx = 0;
  int last_dr = 0;
  int last_dc = 0;
  int reverse_count = 0;
  
  uint8_t occ_buf[PLANNER_V3_BFS_MAX_CELLS];
  planner_v3_bfs_build_occupancy_grid(rows, cols, filtered_obstacles, filtered_obstacle_count, temp_bombs, temp_bomb_count, boxes, box_count, 1, occ_buf);
  while (bomb.row != target_pos.row || bomb.col != target_pos.col) {
    if (chosen_strategy == PLANNER_V3_BFS_STRATEGY_PATH && has_bomb_path) {

      if (bomb_path_idx + 1 >= bomb_path_len) {
        last_err_stage = 3;  // ????????????????????
        last_err_detail = LAST_ERR_DETAIL_V3_BOMB_PATH_EXHAUSTED;  // ?????????????????????
        last_err_detail_v3_stage3 = LAST_ERR_DETAIL_V3_BOMB_PATH_EXHAUSTED;
        return -6;
      }
      
      Point next_in_path = bomb_path[bomb_path_idx + 1];
      int dr = next_in_path.row - bomb.row;
      int dc = next_in_path.col - bomb.col;
      
      if (planner_v3_bfs_abs(dr) + planner_v3_bfs_abs(dc) != 1) {
        last_err_stage = 3;  // ????????????????????
        last_err_detail = LAST_ERR_DETAIL_V3_BOMB_PATH_STEP_NOT_ADJACENT;  // ?????????????????????
        last_err_detail_v3_stage3 = LAST_ERR_DETAIL_V3_BOMB_PATH_STEP_NOT_ADJACENT;
        return -6;
      }

      int push_row = bomb.row - (next_in_path.row - bomb.row);
      int push_col = bomb.col - (next_in_path.col - bomb.col);
      

      if (!planner_v3_bfs_in_bounds(rows, cols, next_in_path.row, next_in_path.col) ||
          !planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
        last_err_stage = 3;  // ????????????????????
        last_err_detail = LAST_ERR_DETAIL_V3_BOMB_PATH_STEP_OUT_OF_BOUNDS;  // ?????????????????????
        last_err_detail_v3_stage3 = LAST_ERR_DETAIL_V3_BOMB_PATH_STEP_OUT_OF_BOUNDS;
        return -6;
      }
      

      int next_on_boundary = (next_in_path.row == 0 || next_in_path.row == rows - 1 ||
                              next_in_path.col == 0 || next_in_path.col == cols - 1);
      
      if (next_on_boundary) {

        if (!planner_v3_bfs_validate_boundary_path(rows, cols, next_in_path, target_pos,
                                                   *car_pos, filtered_obstacles, filtered_obstacle_count,
                                                   temp_bombs, temp_bomb_count, temp_boxes, temp_box_count,
                                                   SIZE_MAX, 1, occ_buf)) {
          last_err_stage = 3;  // ????????????????????
          last_err_detail = LAST_ERR_DETAIL_V3_BOMB_BOUNDARY_PATH_INVALID;  // ?????????????????????
          last_err_detail_v3_stage3 = LAST_ERR_DETAIL_V3_BOMB_BOUNDARY_PATH_INVALID;
          return -6;
        }
      }
      
      if (planner_v3_bfs_is_obstacle(occ_buf, cols, next_in_path.row, next_in_path.col) ||
          planner_v3_bfs_is_obstacle(occ_buf, cols, push_row, push_col)) {
        last_err_stage = 3;  // ????????????????????
        last_err_detail = LAST_ERR_DETAIL_V3_BOMB_PATH_OBSTACLE_CONFLICT;  // ?????????????????????
        last_err_detail_v3_stage3 = LAST_ERR_DETAIL_V3_BOMB_PATH_OBSTACLE_CONFLICT;
        return -6;
      }
      
      if (planner_v3_bfs_is_box_at(occ_buf, cols, next_in_path.row, next_in_path.col, SIZE_MAX) ||
          planner_v3_bfs_is_box_at(occ_buf, cols, push_row, push_col, SIZE_MAX)) {
        last_err_stage = 3;  // ????????????????????
        last_err_detail = LAST_ERR_DETAIL_V3_BOMB_PATH_BOX_CONFLICT;  // ?????????????????????
        last_err_detail_v3_stage3 = LAST_ERR_DETAIL_V3_BOMB_PATH_BOX_CONFLICT;
        return -6;
      }
      
      Point push_from = {push_row, push_col};
      if (car_pos->row != push_from.row || car_pos->col != push_from.col) {
        int car_result = planner_v3_bfs_move_car_to_push_with_remaining_state(
            rows, cols,
            car_pos, push_from,
            obstacles, obstacle_count,
            bombs, bomb_count,
            temp_boxes, temp_box_count,
            path_buffer, path_capacity, out_steps,
            NULL,
            3, LAST_ERR_DETAIL_V3_BOMB_PATH_CAR_MOVE_FAILED);
        if (car_result != 1) {
          return car_result;
        }
      }
      PlannerV3PushExecuteCtx exec_ctx = {
        push_from, next_in_path,
        car_pos, &bomb, bombs, bomb_idx,
        path_buffer, path_capacity, out_steps,
        3, LAST_ERR_DETAIL_V3_BOMB_PATH_EXEC_CAR_MISMATCH,
        LAST_ERR_DETAIL_V3_BOMB_PATH_EXEC_NOT_ADJACENT,
        LAST_ERR_DETAIL_V3_BOMB_PATH_EXEC_NOT_CONTINUOUS
      };
      int exec_ret = planner_v3_bfs_execute_push_strategy(&exec_ctx);
      if (exec_ret < 0) {
        return exec_ret;
      }
      bomb_path_idx++;
      
      last_dr = dr;
      last_dc = dc;
      continue;
    }
    

    

    uint8_t occ_car[PLANNER_V3_BFS_MAX_CELLS];
    int dist_from_car[PLANNER_V3_BFS_MAX_CELLS];
    planner_v3_bfs_build_occupancy_grid(rows, cols,
                                       obstacles, obstacle_count,
                                        bombs, bomb_count,
                                        temp_boxes, temp_box_count,
                                        1, occ_car);
    planner_v3_bfs_global_bfs_from_point_use_occ(rows, cols, *car_pos, occ_car, dist_from_car, 1);
    int sorted_dirs[4];
    int sorted_count = 0;
    PlannerV3PushScoreCandidate candidates[4];
    if (!planner_v3_bfs_evaluate_push_candidates_score(
            rows, cols, bomb, target_pos, SIZE_MAX, occ_buf,
            bomb_path_dist, dist_from_car,
            0, last_dr, last_dc, reverse_count,
            candidates, sorted_dirs, &sorted_count)) {
      last_err_stage = 3;  // ????????????????????
      last_err_detail = LAST_ERR_DETAIL_V3_BOMB_SCORE_NO_FEASIBLE_DIRECTION;  // ?????????????????????
      last_err_detail_v3_stage3 = LAST_ERR_DETAIL_V3_BOMB_SCORE_NO_FEASIBLE_DIRECTION;
      return -6;
    }
    

    PlannerV3PushScoreCandidate chosen;
    int dir_ok = 0;
    for (int k = 0; k < sorted_count; ++k) {
      chosen = candidates[sorted_dirs[k]];
      int next_on_boundary = (chosen.next_pos.row == 0 || chosen.next_pos.row == rows - 1 ||
                              chosen.next_pos.col == 0 || chosen.next_pos.col == cols - 1);
      if (next_on_boundary) {
        if (!planner_v3_bfs_validate_boundary_path(rows, cols, chosen.next_pos, target_pos,
                                                   *car_pos, filtered_obstacles, filtered_obstacle_count,
                                                   temp_bombs, temp_bomb_count, temp_boxes, temp_box_count,
                                                   SIZE_MAX, 1, occ_buf)) {
          continue;  // ?????????????????????
        }
      }
      if (chosen.next_pos.row != target_pos.row || chosen.next_pos.col != target_pos.col) {
        if (!planner_v3_bfs_can_reach_goal(rows, cols,
                                           filtered_obstacles, filtered_obstacle_count,
                                           temp_bombs, temp_bomb_count,
                                           temp_boxes, temp_box_count,
                                           SIZE_MAX, chosen.next_pos, target_pos, 1, occ_buf)) {
          continue;
        }
      }
      if (dist_from_car[chosen.push_from.row * cols + chosen.push_from.col] == INT_MAX) {
        continue;
      }
      dir_ok = 1;
      break;
    }
    if (!dir_ok) {
      last_err_stage = 3;
      last_err_detail = LAST_ERR_DETAIL_V3_BOMB_SCORE_DIR_SELECTION_FAILED;  // ?????????????????????
      last_err_detail_v3_stage3 = LAST_ERR_DETAIL_V3_BOMB_SCORE_DIR_SELECTION_FAILED;
      return -6;
    }
    
    Point push_from = chosen.push_from;
    if (car_pos->row != push_from.row || car_pos->col != push_from.col) {
      int car_result = planner_v3_bfs_move_car_to_push_with_remaining_state(
          rows, cols,
          car_pos, push_from,
          obstacles, obstacle_count,
          bombs, bomb_count,
          temp_boxes, temp_box_count,
          path_buffer, path_capacity, out_steps,
          NULL,
          3, LAST_ERR_DETAIL_V3_BOMB_SCORE_CAR_MOVE_FAILED);
      if (car_result != 1) {
        return car_result;
      }
    }
    PlannerV3PushExecuteCtx exec_ctx = {
      push_from, chosen.next_pos,
      car_pos, &bomb, bombs, bomb_idx,
      path_buffer, path_capacity, out_steps,
      3, LAST_ERR_DETAIL_V3_BOMB_SCORE_EXEC_CAR_MISMATCH,
      LAST_ERR_DETAIL_V3_BOMB_SCORE_EXEC_NOT_ADJACENT,
      LAST_ERR_DETAIL_V3_BOMB_SCORE_EXEC_NOT_CONTINUOUS
    };
    int exec_ret = planner_v3_bfs_execute_push_strategy(&exec_ctx);
    if (exec_ret < 0) {
      return exec_ret;
    }
    
    if (last_dr == -chosen.dr && last_dc == -chosen.dc &&
        (last_dr != 0 || last_dc != 0)) {
      reverse_count++;
    } else {
      reverse_count = 0;
    }
    
    last_dr = chosen.dr;
    last_dc = chosen.dc;
  }
  

  if (bomb.row == target_pos.row && bomb.col == target_pos.col) {
    return 1;
  }
  
  last_err_stage = 3;  // ????????????????????
  last_err_detail = LAST_ERR_DETAIL_V3_BOMB_FINAL_NOT_AT_TARGET;  // ?????????????????????
  last_err_detail_v3_stage3 = LAST_ERR_DETAIL_V3_BOMB_FINAL_NOT_AT_TARGET;
  return -6;
}

// ?????????????
static size_t planner_v3_bfs_bomb_explode(const Point *obstacles, size_t obstacle_count,
                                          Point bomb_pos, Point *new_obstacles,
                                          size_t new_capacity) {
  if (!obstacles || !new_obstacles || new_capacity == 0) {
    return 0;
  }
  

  int exploded_idx = -1;
  for (size_t i = 0; i < obstacle_count; ++i) {
    if (obstacles[i].row == bomb_pos.row && obstacles[i].col == bomb_pos.col) {
      exploded_idx = (int)i;
      break;
    }
  }
  
  if (exploded_idx < 0) {

    size_t copy_count = obstacle_count < new_capacity ? obstacle_count : new_capacity;
    memcpy(new_obstacles, obstacles, copy_count * sizeof(Point));
    return copy_count;
  }
  
  Point center = obstacles[exploded_idx];
  

  uint8_t remove[200] = {0};
  remove[exploded_idx] = 1;
  
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      int adj_r = center.row + dr;
      int adj_c = center.col + dc;
      
      for (size_t i = 0; i < obstacle_count; ++i) {
        if (obstacles[i].row == adj_r && obstacles[i].col == adj_c) {
          remove[i] = 1;
        }
      }
    }
  }
  

  size_t new_count = 0;
  for (size_t i = 0; i < obstacle_count && new_count < new_capacity; ++i) {
    if (!remove[i]) {
      new_obstacles[new_count++] = obstacles[i];
    }
  }
  
  return new_count;
}

// ?????????????
static size_t planner_v3_bfs_collect_boxes(int rows, int cols, const Point *boxes,
                                       size_t box_count, Point *filtered,
                                       size_t *source_indices,
                                       size_t *mapping_out) {
  size_t kept = 0;
  for (size_t i = 0; i < box_count && kept < PLANNER_V3_BFS_MAX_BOXES; ++i) {
    if (mapping_out) {
      mapping_out[i] = SIZE_MAX;
    }
    Point p = boxes[i];
    if (p.row < 0 || p.col < 0) {
      continue;
    }
    if (!planner_v3_bfs_in_bounds(rows, cols, p.row, p.col)) {
      continue;
    }
    filtered[kept] = p;
    source_indices[kept] = i;
    kept++;
  }
  return kept;
}

/* ????????????? */
static int planner_v3_bfs_score_car_to_box(int rows, int cols, Point car, size_t box_idx,
                                       const Point *current_boxes, size_t box_count,
                                       const Point *obstacles, size_t obstacle_count,
                                       const Point *bombs, size_t bomb_count) {
  if (!current_boxes || box_idx >= box_count) {
    return INT_MAX;
  }
  Point box = current_boxes[box_idx];
  if (box.row < 0 || box.col < 0) {
    return INT_MAX;
  }
  if (!planner_v3_bfs_in_bounds(rows, cols, car.row, car.col) ||
      !planner_v3_bfs_in_bounds(rows, cols, box.row, box.col)) {
    return INT_MAX;
  }


  int dist_map[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, car, obstacles, obstacle_count,
                                             bombs, bomb_count, current_boxes, box_count,
                                             dist_map, 1, NULL)) {
    return INT_MAX;
  }

  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  uint8_t occ_buf[PLANNER_V3_BFS_MAX_CELLS];
  planner_v3_bfs_build_occupancy_grid(rows, cols, obstacles, obstacle_count, bombs, bomb_count, current_boxes, box_count, 1, occ_buf);
  int best = INT_MAX;
  for (int d = 0; d < 4; ++d) {
    int dr = dirs[d][0];
    int dc = dirs[d][1];
    int to_r = box.row + dr, to_c = box.col + dc;
    Point push_pos = (Point){ box.row - (to_r - box.row), box.col - (to_c - box.col) };
    if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col)) {
      continue;
    }
    int new_box_row = box.row + dirs[d][0];
    int new_box_col = box.col + dirs[d][1];
    if (!planner_v3_bfs_in_bounds(rows, cols, new_box_row, new_box_col)) {
      continue;
    }
    if (planner_v3_bfs_is_obstacle_no_bomb(occ_buf, cols, new_box_row, new_box_col)) {
      continue;
    }
    if (planner_v3_bfs_is_box_at(occ_buf, cols, new_box_row, new_box_col, box_idx)) {
      continue;
    }
    if (planner_v3_bfs_is_deadlock_occ(occ_buf, rows, cols, box_idx, new_box_row, new_box_col)) {
      continue;
    }


    int push_idx = push_pos.row * cols + push_pos.col;
    if (push_idx >= 0 && push_idx < rows * cols &&
        dist_map[push_idx] != INT_MAX && dist_map[push_idx] < best) {
      best = dist_map[push_idx];
    }
  }
  return best;
}

/**
 */
static int planner_v3_bfs_assign_targets(int rows, int cols, Point car,
                                     const Point *boxes, size_t box_count,
                                     const Point *targets, size_t target_count,
                                     const Point *obstacles, size_t obstacle_count,
                                     const Point *bombs, size_t bomb_count,
                                     PlannerBoxGoalBFS *out_goals,
                                     size_t *out_goal_count,
                                     size_t *out_mapping) {
  if (!boxes || !targets || !out_goals || !out_goal_count) {
    return -1;
  }
  if (target_count == 0) {
    return -2;
  }
  if (target_count > PLANNER_V3_BFS_MAX_CELLS) {
    return -1;
  }

  Point valid_boxes[PLANNER_V3_BFS_MAX_BOXES];
  size_t source_indices[PLANNER_V3_BFS_MAX_BOXES];
  size_t usable = planner_v3_bfs_collect_boxes(rows, cols, boxes, box_count,
                                           valid_boxes, source_indices,
                                           out_mapping);
  if (usable == 0) {
    return -2;
  }
  if (target_count < usable) {
    return -8;
  }

  uint8_t target_used[PLANNER_V3_BFS_MAX_CELLS] = {0};
  size_t order[PLANNER_V3_BFS_MAX_BOXES];
  int scores[PLANNER_V3_BFS_MAX_BOXES];
  size_t assigned = 0;

  /* 1) 先按“车到箱子最近可行推位”的距离升序排箱子。 */
  for (size_t i = 0; i < usable; ++i) {
    order[i] = i;
    scores[i] = planner_v3_bfs_score_car_to_box(rows, cols, car, source_indices[i], boxes,
                                                box_count, obstacles, obstacle_count, bombs, bomb_count);
  }
  for (size_t i = 0; i + 1 < usable; ++i) {
    for (size_t j = 0; j + 1 < usable - i; ++j) {
      if (scores[j] > scores[j + 1] ||
          (scores[j] == scores[j + 1] && source_indices[order[j]] > source_indices[order[j + 1]])) {
        int ts = scores[j];
        scores[j] = scores[j + 1];
        scores[j + 1] = ts;
        size_t ti = order[j];
        order[j] = order[j + 1];
        order[j + 1] = ti;
      }
    }
  }

  /* 2) 按上述顺序，给每个箱子分配“曼哈顿距离最近”的未使用目标点。 */
  for (size_t oi = 0; oi < usable; ++oi) {
    size_t bi = order[oi];
    size_t best_t = SIZE_MAX;
    int best_md = INT_MAX;
    for (size_t t = 0; t < target_count; ++t) {
      if (target_used[t]) continue;
      int md = planner_v3_bfs_abs(valid_boxes[bi].row - targets[t].row) +
               planner_v3_bfs_abs(valid_boxes[bi].col - targets[t].col);
      if (md < best_md || (md == best_md && t < best_t)) {
        best_md = md;
        best_t = t;
      }
    }

    if (best_t == SIZE_MAX) {
      last_err_stage = 2;
      last_err_detail = LAST_ERR_DETAIL_V3_BOX_TO_TARGET_INF;
      return -6;
    }

    out_goals[assigned].box = valid_boxes[bi];
    out_goals[assigned].target = targets[best_t];
    out_goals[assigned].target_idx = best_t;
    out_goals[assigned].source_idx = source_indices[bi];
    out_goals[assigned].has_obstacle_to_clear = 0;
    out_goals[assigned].obstacle_to_clear.row = -1;
    out_goals[assigned].obstacle_to_clear.col = -1;
    if (out_mapping) {
      out_mapping[source_indices[bi]] = best_t;
    }
    target_used[best_t] = 1;
    assigned++;
  }

  *out_goal_count = assigned;
  return 0;
}

// ???????? box_target_indices ???? goals??????????/?????
static int planner_v3_bfs_build_goals_from_manual(int rows, int cols,
    const Point *boxes, size_t box_count,
    const Point *targets, size_t target_count,
    const size_t *box_target_indices,
    PlannerBoxGoalBFS *out_goals, size_t *out_goal_count,
    size_t *out_mapping) {
  if (!boxes || !targets || !box_target_indices || !out_goals || !out_goal_count) {
    return -1;
  }
  if (target_count == 0) {
    return -2;
  }
  uint8_t target_used[PLANNER_V3_BFS_MAX_CELLS];
  memset(target_used, 0, sizeof(target_used));
  size_t assigned = 0;
  for (size_t i = 0; i < box_count && assigned < PLANNER_V3_BFS_MAX_BOXES; ++i) {
    Point p = boxes[i];
    if (p.row < 0 || p.col < 0) {
      continue;
    }
    if (!planner_v3_bfs_in_bounds(rows, cols, p.row, p.col)) {
      continue;
    }
    size_t ti = box_target_indices[i];
    if (ti >= target_count) {
      last_err_stage = 2;
      last_err_detail = LAST_ERR_DETAIL_V3_MANUAL_TARGET_INDEX_INVALID;  // ????????????????
      return -11;  // ?????
    }
    if (target_used[ti]) {
      last_err_stage = 2;
      last_err_detail = LAST_ERR_DETAIL_V3_MANUAL_TARGET_DUPLICATED;  // ????????????????
      return -11;  // ?????
    }
    target_used[ti] = 1;
    out_goals[assigned].box = p;
    out_goals[assigned].target = targets[ti];
    out_goals[assigned].target_idx = ti;
    out_goals[assigned].source_idx = i;
    out_goals[assigned].has_obstacle_to_clear = 0;
    out_goals[assigned].obstacle_to_clear.row = -1;
    out_goals[assigned].obstacle_to_clear.col = -1;
    if (out_mapping) {
      out_mapping[i] = ti;
    }
    assigned++;
  }
  if (assigned == 0) {
    return -2;  // ??????????????
  }
  *out_goal_count = assigned;
  return 0;
}

static int build_assigned_goals(
    int rows, int cols, Point car,
    const Point *boxes, size_t box_count,
    const Point *targets, size_t target_count,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const size_t *manual_box_target_indices,
    PlannerBoxGoalBFS *out_goals,
    size_t *out_goal_count,
    size_t *out_mapping) {
  if (manual_box_target_indices) {
    return planner_v3_bfs_build_goals_from_manual(
        rows, cols,
        boxes, box_count,
        targets, target_count,
        manual_box_target_indices,
        out_goals, out_goal_count,
        out_mapping);
  }
  return planner_v3_bfs_assign_targets(
      rows, cols, car,
      boxes, box_count,
      targets, target_count,
      obstacles, obstacle_count,
      bombs, bomb_count,
      out_goals, out_goal_count,
      out_mapping);
}

static int build_push_order(
    int rows, int cols, Point car,
    const PlannerBoxGoalBFS *goals, size_t goal_count, size_t box_count,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    PlannerV3TwoPhaseRuntime *runtime) {
  if (!runtime || !goals) {
    return 0;
  }
  runtime->ordered_goal_count = 0;
  if (!planner_v3_bfs_find_ordered_goals(
          rows, cols, car,
          goals, goal_count, box_count,
          obstacles, obstacle_count,
          bombs, bomb_count,
          runtime->ordered_goal_indices, &runtime->ordered_goal_count) ||
      runtime->ordered_goal_count == 0) {
    return 0;
  }

  runtime->two_phase.box_count = box_count;
  for (size_t bi = 0; bi < PLANNER_V3_BFS_MAX_BOXES; ++bi) {
    runtime->two_phase.box_plan[bi].planned_order = SIZE_MAX;
    runtime->two_phase.box_plan[bi].box_index = bi;
  }
  for (size_t oi = 0; oi < runtime->ordered_goal_count; ++oi) {
    size_t gi = runtime->ordered_goal_indices[oi];
    runtime->two_phase.ordered_goal_indices[oi] = gi;
    if (gi >= goal_count) {
      continue;
    }
    size_t bi = goals[gi].source_idx;
    if (bi >= PLANNER_V3_BFS_MAX_BOXES) {
      continue;
    }
    runtime->two_phase.box_plan[bi].planned_order = oi;
    runtime->two_phase.box_plan[bi].box_index = bi;
    runtime->two_phase.box_plan[bi].box_start = goals[gi].box;
    runtime->two_phase.box_plan[bi].target = goals[gi].target;
  }
  return 1;
}

static int planner_v3_bfs_bomb_purpose_priority(uint8_t purpose) {
  switch (purpose) {
    case PLANNER_V3_BOMB_PURPOSE_PATH_OBSTACLE:
      return 0;
    case PLANNER_V3_BOMB_PURPOSE_BOMB_A_PUSH_POS:
      return 1;
    case PLANNER_V3_BOMB_PURPOSE_FIRST_PUSH_POS:
      return 2;
    case PLANNER_V3_BOMB_PURPOSE_OPTIMIZE_PATH:
      return 3;
    default:
      return 4;
  }
}

static void planner_v3_bfs_add_or_update_bomb_use(PlannerV3PreBoxPlan *plan,
                                                  size_t bomb_idx,
                                                  uint8_t purpose,
                                                  Point target_pos) {
  if (!plan || bomb_idx >= MAX_BOMBS) {
    return;
  }
  for (size_t i = 0; i < plan->used_bomb_count; ++i) {
    if (plan->used_bomb_indices[i] != bomb_idx) {
      continue;
    }
    if (planner_v3_bfs_bomb_purpose_priority(purpose) <
        planner_v3_bfs_bomb_purpose_priority(plan->used_bomb_purposes[i])) {
      plan->used_bomb_purposes[i] = purpose;
      plan->used_bomb_targets[i] = target_pos;
    }
    return;
  }
  if (plan->used_bomb_count >= MAX_BOMBS) {
    return;
  }
  {
    size_t idx = plan->used_bomb_count++;
    plan->used_bomb_indices[idx] = bomb_idx;
    plan->used_bomb_purposes[idx] = purpose;
    plan->used_bomb_targets[idx] = target_pos;
  }
}

static void planner_v3_bfs_clear_bomb_actions(PlannerV3PreBoxPlan *plan) {
  if (!plan) {
    return;
  }
  plan->bomb_action_count = 0;
  plan->used_bomb_count = 0;
}

static void planner_v3_bfs_append_bomb_action(
    PlannerV3PreBoxPlan *plan,
    size_t bomb_idx,
    Point bomb_target,
    uint8_t purpose,
    const Point *exploded_obstacles,
    size_t exploded_obstacle_count) {
  if (!plan || bomb_idx >= MAX_BOMBS) {
    return;
  }
  for (size_t i = 0; i < plan->bomb_action_count; ++i) {
    if (plan->bomb_actions[i].bomb_idx == bomb_idx) {
      return;
    }
  }
  if (plan->bomb_action_count >= MAX_BOMBS) {
    return;
  }
  PlannerV3BombAction *action = &plan->bomb_actions[plan->bomb_action_count++];
  memset(action, 0, sizeof(*action));
  action->bomb_idx = bomb_idx;
  action->bomb_target = bomb_target;
  action->bomb_purpose = purpose;
  if (exploded_obstacles && exploded_obstacle_count > 0) {
    action->exploded_obstacle_count =
        (exploded_obstacle_count < PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES)
            ? exploded_obstacle_count
            : PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES;
    for (size_t i = 0; i < action->exploded_obstacle_count; ++i) {
      action->exploded_obstacles[i] = exploded_obstacles[i];
    }
  }
}

static void planner_v3_bfs_sync_bomb_summary_from_actions(PlannerV3PreBoxPlan *plan) {
  if (!plan) {
    return;
  }
  plan->used_bomb_count = 0;
  for (size_t i = 0; i < plan->bomb_action_count; ++i) {
    planner_v3_bfs_add_or_update_bomb_use(
        plan,
        plan->bomb_actions[i].bomb_idx,
        plan->bomb_actions[i].bomb_purpose,
        plan->bomb_actions[i].bomb_target);
  }
}

static int planner_v3_bfs_find_ordered_goals(int rows, int cols, Point car,
    const PlannerBoxGoalBFS *goals, size_t goal_count, size_t box_count,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    size_t *ordered_goal_indices, size_t *ordered_count) {
  if (!goals || !ordered_goal_indices || !ordered_count) {
    return 0;
  }
  *ordered_count = 0;
  if (goal_count == 0 || goal_count > PLANNER_V3_BFS_MAX_BOXES) {
    return 0;
  }

  Point active_boxes[PLANNER_V3_BFS_MAX_BOXES];
  uint8_t selected_goal[PLANNER_V3_BFS_MAX_BOXES] = {0};
  for (size_t i = 0; i < PLANNER_V3_BFS_MAX_BOXES; ++i) {
    active_boxes[i].row = -1;
    active_boxes[i].col = -1;
  }
  for (size_t gi = 0; gi < goal_count; ++gi) {
    size_t bi = goals[gi].source_idx;
    if (bi >= box_count || bi >= PLANNER_V3_BFS_MAX_BOXES) {
      continue;
    }
    active_boxes[bi] = goals[gi].box;
  }

  Point ref = car;
  for (size_t oi = 0; oi < goal_count; ++oi) {
    size_t best_goal = SIZE_MAX;
    int best_dist = INT_MAX;
    int has_bfs = 0;

    for (size_t gi = 0; gi < goal_count; ++gi) {
      if (selected_goal[gi]) {
        continue;
      }
      size_t bi = goals[gi].source_idx;
      if (bi >= box_count || bi >= PLANNER_V3_BFS_MAX_BOXES) {
        continue;
      }
      if (active_boxes[bi].row < 0 || active_boxes[bi].col < 0) {
        continue;
      }

      int dist = planner_v3_bfs_distance_between(
          rows, cols, ref, active_boxes[bi],
          obstacles, obstacle_count,
          bombs, bomb_count,
          active_boxes, box_count,
          1, NULL);
      if (dist == INT_MAX) {
        continue;
      }
      if (!has_bfs || dist < best_dist ||
          (dist == best_dist &&
           (best_goal == SIZE_MAX || bi < goals[best_goal].source_idx))) {
        has_bfs = 1;
        best_dist = dist;
        best_goal = gi;
      }
    }

    if (!has_bfs) {
      for (size_t gi = 0; gi < goal_count; ++gi) {
        if (selected_goal[gi]) {
          continue;
        }
        size_t bi = goals[gi].source_idx;
        if (bi >= box_count || bi >= PLANNER_V3_BFS_MAX_BOXES) {
          continue;
        }
        if (active_boxes[bi].row < 0 || active_boxes[bi].col < 0) {
          continue;
        }
        int md = planner_v3_bfs_abs(ref.row - active_boxes[bi].row) +
                 planner_v3_bfs_abs(ref.col - active_boxes[bi].col);
        if (best_goal == SIZE_MAX || md < best_dist ||
            (md == best_dist && bi < goals[best_goal].source_idx)) {
          best_dist = md;
          best_goal = gi;
        }
      }
    }

    if (best_goal == SIZE_MAX) {
      return 0;
    }
    ordered_goal_indices[*ordered_count] = best_goal;
    (*ordered_count)++;
    selected_goal[best_goal] = 1;
    {
      size_t sel_bi = goals[best_goal].source_idx;
      if (sel_bi < box_count && sel_bi < PLANNER_V3_BFS_MAX_BOXES) {
        active_boxes[sel_bi].row = -1;
        active_boxes[sel_bi].col = -1;
      }
      ref = goals[best_goal].target;
    }
  }
  return (*ordered_count > 0) ? 1 : 0;
}

static void build_box_reachability(
    int rows, int cols,
    const PlannerBoxGoalBFS *goals, size_t goal_count, size_t box_count,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    Point car,
    PlannerV3TwoPhaseRuntime *runtime) {
  (void)car;
  if (!runtime || !goals) {
    return;
  }

  for (size_t bi = 0; bi < PLANNER_V3_BFS_MAX_BOXES; ++bi) {
    runtime->normal_reachable_by_box[bi] = 0;
    runtime->two_phase.box_plan[bi].normal_reachable = 0;
  }

  Point active_boxes[PLANNER_V3_BFS_MAX_BOXES];
  for (size_t i = 0; i < PLANNER_V3_BFS_MAX_BOXES; ++i) {
    active_boxes[i].row = -1;
    active_boxes[i].col = -1;
  }
  for (size_t gi = 0; gi < goal_count; ++gi) {
    size_t bi = goals[gi].source_idx;
    if (bi >= box_count || bi >= PLANNER_V3_BFS_MAX_BOXES) {
      continue;
    }
    active_boxes[bi] = goals[gi].box;
  }

  for (size_t oi = 0; oi < runtime->ordered_goal_count; ++oi) {
    size_t gi = runtime->ordered_goal_indices[oi];
    if (gi >= goal_count) {
      continue;
    }
    size_t bi = goals[gi].source_idx;
    if (bi >= box_count || bi >= PLANNER_V3_BFS_MAX_BOXES) {
      continue;
    }
    if (active_boxes[bi].row < 0 || active_boxes[bi].col < 0) {
      continue;
    }

    PlannerV3BoxTwoPhasePlan *entry = &runtime->two_phase.box_plan[bi];
    entry->planned_order = oi;
    entry->box_index = bi;
    entry->box_start = goals[gi].box;
    entry->target = goals[gi].target;

    entry->normal_reachable = planner_v3_bfs_can_reach_goal(
        rows, cols,
        obstacles, obstacle_count,
        bombs, bomb_count,
        active_boxes, box_count,
        bi, active_boxes[bi], goals[gi].target, 1, NULL) ? 1 : 0;
    runtime->normal_reachable_by_box[bi] = entry->normal_reachable;

    active_boxes[bi].row = -1;
    active_boxes[bi].col = -1;
  }
}

static int planner_v3_bfs_plan_has_any_path_obstacle_bomb(
    const PlannerV3PreBoxPlan *plan) {
  if (!plan) {
    return 0;
  }
  for (size_t u = 0; u < plan->bomb_action_count; ++u) {
    if (plan->bomb_actions[u].bomb_purpose == PLANNER_V3_BOMB_PURPOSE_PATH_OBSTACLE) {
      return 1;
    }
  }
  return 0;
}

static int planner_v3_bfs_plan_has_available_path_obstacle_bomb_after_forced_ban(
    const PlannerV3PreBoxPlan *plan,
    const uint8_t banned_bombs[MAX_BOMBS],
    size_t forced_ban_bomb_idx) {
  if (!plan) {
    return 0;
  }
  for (size_t u = 0; u < plan->bomb_action_count; ++u) {
    if (plan->bomb_actions[u].bomb_purpose != PLANNER_V3_BOMB_PURPOSE_PATH_OBSTACLE) {
      continue;
    }
    size_t bomb_idx = plan->bomb_actions[u].bomb_idx;
    if (bomb_idx >= MAX_BOMBS || bomb_idx == forced_ban_bomb_idx) {
      continue;
    }
    if (banned_bombs && banned_bombs[bomb_idx]) {
      continue;
    }
    return 1;
  }
  return 0;
}

static int planner_v3_bfs_plan_get_bomb_priority_and_target(
    const PlannerV3PreBoxPlan *plan,
    size_t bomb_idx,
    int *out_best_priority,
    Point *out_target) {
  if (!plan || bomb_idx >= MAX_BOMBS) {
    return 0;
  }
  int best = INT_MAX;
  int used = 0;
  Point target = {-1, -1};
  for (size_t u = 0; u < plan->bomb_action_count; ++u) {
    if (plan->bomb_actions[u].bomb_idx != bomb_idx) {
      continue;
    }
    int p = planner_v3_bfs_bomb_purpose_priority(plan->bomb_actions[u].bomb_purpose);
    if (plan->reachable) {
      p = planner_v3_bfs_bomb_purpose_priority(PLANNER_V3_BOMB_PURPOSE_OPTIMIZE_PATH);
    }
    if (!used || p < best) {
      best = p;
      target = plan->bomb_actions[u].bomb_target;
    }
    used = 1;
  }
  if (!used) {
    return 0;
  }
  if (out_best_priority) {
    *out_best_priority = best;
  }
  if (out_target) {
    *out_target = target;
  }
  return 1;
}

static int planner_v3_bfs_bomb_tie_break_score(
    int rows, int cols,
    Point bomb_pos, Point box_pos, Point bomb_target,
    const Point *obstacles, size_t obstacle_count) {
  if (!planner_v3_bfs_in_bounds(rows, cols, bomb_pos.row, bomb_pos.col) ||
      !planner_v3_bfs_in_bounds(rows, cols, box_pos.row, box_pos.col) ||
      !planner_v3_bfs_in_bounds(rows, cols, bomb_target.row, bomb_target.col)) {
    return INT_MAX;
  }
  size_t obs_count = obstacle_count;
  if (!obstacles) {
    obs_count = 0;
  }
  if (obs_count > 200) {
    obs_count = 200;
  }
  int d_bomb_to_box = planner_v3_bfs_distance_between(
      rows, cols,
      bomb_pos, box_pos,
      obstacles, obs_count,
      NULL, 0,
      NULL, 0,
      0, NULL);
  if (d_bomb_to_box == INT_MAX) {
    return INT_MAX;
  }
  Point filtered_obstacles[200];
  size_t filtered_count = 0;
  for (size_t i = 0; i < obs_count && filtered_count < 200; ++i) {
    if (obstacles[i].row == bomb_target.row &&
        obstacles[i].col == bomb_target.col) {
      continue;
    }
    filtered_obstacles[filtered_count++] = obstacles[i];
  }
  int d_bomb_to_target = planner_v3_bfs_distance_between(
      rows, cols,
      bomb_pos, bomb_target,
      filtered_obstacles, filtered_count,
      NULL, 0,
      NULL, 0,
      0, NULL);
  if (d_bomb_to_target == INT_MAX) {
    return INT_MAX;
  }
  if (d_bomb_to_box > INT_MAX - d_bomb_to_target) {
    return INT_MAX;
  }
  return d_bomb_to_box + d_bomb_to_target;
}

static size_t planner_v3_bfs_collect_obstacles_in_centers_3x3(
    const Point *obstacles, size_t obstacle_count,
    const Point *centers, size_t center_count,
    Point *out_obstacles, size_t out_cap) {
  if (!obstacles || !centers || !out_obstacles || out_cap == 0) {
    return 0;
  }
  size_t out_count = 0;
  for (size_t oi = 0; oi < obstacle_count && out_count < out_cap; ++oi) {
    int covered = 0;
    for (size_t ci = 0; ci < center_count; ++ci) {
      if (planner_v3_bfs_is_in_bomb_range_3x3(centers[ci], obstacles[oi])) {
        covered = 1;
        break;
      }
    }
    if (!covered) {
      continue;
    }
    if (planner_v3_bfs_point_in_list(obstacles[oi], out_obstacles, out_count)) {
      continue;
    }
    out_obstacles[out_count++] = obstacles[oi];
  }
  return out_count;
}

static void planner_v3_bfs_refresh_plan_exploded_obstacles_from_actions(
    PlannerV3PreBoxPlan *plan,
    const Point *planning_obstacles, size_t planning_obstacle_count) {
  if (!plan) {
    return;
  }
  plan->exploded_obstacle_count = 0;
  if (!planning_obstacles || planning_obstacle_count == 0 || plan->bomb_action_count == 0) {
    return;
  }

  Point selected_centers[MAX_BOMBS];
  size_t selected_center_count = 0;
  for (size_t ai = 0; ai < plan->bomb_action_count && selected_center_count < MAX_BOMBS; ++ai) {
    const PlannerV3BombAction *action = &plan->bomb_actions[ai];
    int use_center = 0;
    if (action->bomb_purpose == PLANNER_V3_BOMB_PURPOSE_PATH_OBSTACLE) {
      use_center = 1;  // bomb A center
    } else if (action->bomb_purpose == PLANNER_V3_BOMB_PURPOSE_BOMB_A_PUSH_POS &&
               action->exploded_obstacle_count > 0) {
      use_center = 1;  // bomb B center; bombs-on-path entries carry no exploded hint
    }
    if (!use_center) {
      continue;
    }
    if (action->bomb_target.row < 0 || action->bomb_target.col < 0) {
      continue;
    }
    if (planner_v3_bfs_point_in_list(action->bomb_target, selected_centers, selected_center_count)) {
      continue;
    }
    selected_centers[selected_center_count++] = action->bomb_target;
  }
  if (selected_center_count == 0) {
    return;
  }

  plan->exploded_obstacle_count = planner_v3_bfs_collect_obstacles_in_centers_3x3(
      planning_obstacles, planning_obstacle_count,
      selected_centers, selected_center_count,
      plan->exploded_obstacles, PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES);
}

static void build_special_preplans(
    int rows, int cols,
    const PlannerBoxGoalBFS *goals, size_t goal_count, size_t box_count,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    Point car,
    size_t start_order,
    PlannerV3TwoPhaseRuntime *runtime) {
  if (!runtime || !goals) {
    return;
  }

  if (start_order == 0) {
    memset(runtime->preplans, 0, sizeof(runtime->preplans));
  }

  Point active_boxes[PLANNER_V3_BFS_MAX_BOXES];
  for (size_t i = 0; i < PLANNER_V3_BFS_MAX_BOXES; ++i) {
    active_boxes[i].row = -1;
    active_boxes[i].col = -1;
  }
  for (size_t gi = 0; gi < goal_count; ++gi) {
    size_t bi = goals[gi].source_idx;
    if (bi >= box_count || bi >= PLANNER_V3_BFS_MAX_BOXES) {
      continue;
    }
    active_boxes[bi] = goals[gi].box;
  }

  Point planned_obstacles[200];
  size_t planned_obstacle_count = obstacle_count;
  if (planned_obstacle_count > 200) {
    planned_obstacle_count = 200;
  }
  if (planned_obstacle_count > 0 && obstacles) {
    memcpy(planned_obstacles, obstacles, planned_obstacle_count * sizeof(Point));
  } else {
    planned_obstacle_count = 0;
  }

  for (size_t oi = 0; oi < start_order && oi < runtime->ordered_goal_count; ++oi) {
    size_t gi = runtime->ordered_goal_indices[oi];
    if (gi >= goal_count) {
      continue;
    }
    size_t bi = goals[gi].source_idx;
    if (bi >= box_count || bi >= PLANNER_V3_BFS_MAX_BOXES) {
      continue;
    }
    active_boxes[bi].row = -1;
    active_boxes[bi].col = -1;
  }

  Point plan_car = car;
  if (start_order > 0 && start_order <= runtime->ordered_goal_count) {
    size_t prev_gi = runtime->ordered_goal_indices[start_order - 1];
    if (prev_gi < goal_count) {
      plan_car = goals[prev_gi].target;
    }
  }

  for (size_t oi = start_order; oi < runtime->ordered_goal_count; ++oi) {
    size_t gi = runtime->ordered_goal_indices[oi];
    if (gi >= goal_count) {
      continue;
    }
    size_t bi = goals[gi].source_idx;
    if (bi >= box_count || bi >= PLANNER_V3_BFS_MAX_BOXES) {
      continue;
    }

    PlannerV3PreBoxPlan *plan = &runtime->preplans[bi];
    memset(plan, 0, sizeof(*plan));
    plan->reachable = 0;
    planner_v3_bfs_clear_bomb_actions(plan);
    if (active_boxes[bi].row < 0 || active_boxes[bi].col < 0) {
      continue;
    }

    Point planner_obstacles[200];
    size_t planner_obstacle_count = planned_obstacle_count;
    memcpy(planner_obstacles, planned_obstacles, planner_obstacle_count * sizeof(Point));

    Point planner_bombs[MAX_BOMBS];
    size_t planner_bomb_count = bomb_count;
    if (planner_bomb_count > MAX_BOMBS) {
      planner_bomb_count = MAX_BOMBS;
    }
    memset(planner_bombs, 0, sizeof(planner_bombs));
    if (planner_bomb_count > 0 && bombs) {
      memcpy(planner_bombs, bombs, planner_bomb_count * sizeof(Point));
    }

    for (size_t bb = 0; bb < planner_bomb_count; ++bb) {
      if (!runtime->banned_bomb_by_box[bi][bb]) {
        continue;
      }
      if (planner_bombs[bb].row < 0 || planner_bombs[bb].col < 0) {
        continue;
      }
      if (planner_obstacle_count < 200 &&
          !planner_v3_bfs_point_in_list(planner_bombs[bb], planner_obstacles, planner_obstacle_count)) {
        planner_obstacles[planner_obstacle_count++] = planner_bombs[bb];
      }
      planner_bombs[bb].row = -1;
      planner_bombs[bb].col = -1;
    }

    plan->reachable = planner_v3_bfs_can_reach_goal(
        rows, cols,
        planner_obstacles, planner_obstacle_count,
        planner_bombs, planner_bomb_count,
        active_boxes, box_count,
        bi, active_boxes[bi], goals[gi].target, 1, NULL) ? 1 : 0;
    runtime->normal_reachable_by_box[bi] = plan->reachable;
    runtime->two_phase.box_plan[bi].normal_reachable = plan->reachable;

    Point sp_obstacle = {-1, -1};
    int sp_has_obs = 0;
    Point sp_path[PLANNER_V3_BFS_MAX_PATH_LEN];
    size_t sp_len = 0;
    if (planner_v3_bfs_special_path_planning(
            rows, cols,
            active_boxes[bi], goals[gi].target,
            planner_obstacles, planner_obstacle_count,
            planner_bombs, planner_bomb_count,
            active_boxes, box_count, bi,
            plan_car,
            &sp_obstacle, &sp_has_obs,
            sp_path, PLANNER_V3_BFS_MAX_PATH_LEN, &sp_len,
            0, 0) &&
        sp_len > 1) {
      plan->special_path_valid = 1;
      plan->special_path_len = sp_len;
      memcpy(plan->special_path, sp_path, sp_len * sizeof(Point));
      plan->has_obstacle_to_clear = sp_has_obs ? 1 : 0;
      plan->obstacle_to_clear = sp_obstacle;

      plan->exploded_obstacle_count = s_special_path_obstacle_count;
      if (plan->exploded_obstacle_count > PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES) {
        plan->exploded_obstacle_count = PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES;
      }
      for (size_t eo = 0; eo < plan->exploded_obstacle_count; ++eo) {
        plan->exploded_obstacles[eo] = s_special_path_obstacles[eo];
      }

      int actions_from_single_hint = 0;
      if (s_last_special_single_bomb_hint.valid &&
          s_last_special_single_bomb_hint.bomb_a_idx < planner_bomb_count &&
          s_last_special_single_bomb_hint.bomb_a_idx < MAX_BOMBS &&
          planner_bombs[s_last_special_single_bomb_hint.bomb_a_idx].row >= 0 &&
          planner_bombs[s_last_special_single_bomb_hint.bomb_a_idx].col >= 0 &&
          planner_v3_bfs_in_bounds(rows, cols,
              s_last_special_single_bomb_hint.bomb_a_target.row,
              s_last_special_single_bomb_hint.bomb_a_target.col)) {
        if (s_last_special_single_bomb_hint.bomb_b_idx < planner_bomb_count &&
            s_last_special_single_bomb_hint.bomb_b_idx < MAX_BOMBS &&
            planner_bombs[s_last_special_single_bomb_hint.bomb_b_idx].row >= 0 &&
            planner_bombs[s_last_special_single_bomb_hint.bomb_b_idx].col >= 0 &&
            planner_v3_bfs_in_bounds(rows, cols,
                s_last_special_single_bomb_hint.bomb_b_target.row,
                s_last_special_single_bomb_hint.bomb_b_target.col)) {
          planner_v3_bfs_append_bomb_action(
              plan,
              s_last_special_single_bomb_hint.bomb_b_idx,
              s_last_special_single_bomb_hint.bomb_b_target,
              PLANNER_V3_BOMB_PURPOSE_BOMB_A_PUSH_POS,
              s_last_special_single_bomb_hint.bomb_b_exploded_obstacles,
              s_last_special_single_bomb_hint.bomb_b_exploded_obstacle_count);
        }
        planner_v3_bfs_append_bomb_action(
            plan,
            s_last_special_single_bomb_hint.bomb_a_idx,
            s_last_special_single_bomb_hint.bomb_a_target,
            plan->reachable ? PLANNER_V3_BOMB_PURPOSE_OPTIMIZE_PATH
                            : PLANNER_V3_BOMB_PURPOSE_PATH_OBSTACLE,
            plan->exploded_obstacles,
            plan->exploded_obstacle_count);
        actions_from_single_hint = 1;
      }

      if (!actions_from_single_hint) {
        if (plan->has_obstacle_to_clear &&
            plan->obstacle_to_clear.row >= 0 && plan->obstacle_to_clear.col >= 0) {
          size_t chosen_bomb_idx = SIZE_MAX;
          size_t chosen_bomb_b_idx = SIZE_MAX;
          Point chosen_bomb_b_target = {-1, -1};
          Point chosen_bomb_b_exploded[PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES];
          size_t chosen_bomb_b_exploded_count = 0;
          int best_dist = INT_MAX;
          for (size_t b = 0; b < planner_bomb_count; ++b) {
            if (planner_bombs[b].row < 0 || planner_bombs[b].col < 0) {
              continue;
            }
            Point other_bombs[MAX_BOMBS];
            size_t other_bomb_count = 0;
            for (size_t ob = 0; ob < planner_bomb_count && other_bomb_count < MAX_BOMBS; ++ob) {
              if (ob == b) {
                continue;
              }
              if (planner_bombs[ob].row < 0 || planner_bombs[ob].col < 0) {
                continue;
              }
              other_bombs[other_bomb_count++] = planner_bombs[ob];
            }
            int dist = planner_v3_bfs_distance_between(
                rows, cols,
                planner_bombs[b], plan->obstacle_to_clear,
                planner_obstacles, planner_obstacle_count,
                other_bombs, other_bomb_count,
                active_boxes, box_count,
                1, NULL);
            if (dist == INT_MAX) {
              continue;
            }

            size_t candidate_b_idx = SIZE_MAX;
            Point candidate_b_target = {-1, -1};
            Point candidate_b_exploded[PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES];
            size_t candidate_b_exploded_count = 0;

            const Point *obs_for_push_check = planner_obstacles;
            size_t obs_for_push_check_count = planner_obstacle_count;

            int push_obs_a_is_box = 0;
            int push_obs_a_is_bomb = 0;
            Point push_obs_a = planner_v3_bfs_check_bomb_path_push_blocker(
                rows, cols, planner_bombs[b], plan->obstacle_to_clear,
                obs_for_push_check, obs_for_push_check_count,
                planner_bombs, planner_bomb_count, b,
                active_boxes, box_count,
                &push_obs_a_is_box, &push_obs_a_is_bomb);

            int candidate_feasible = 1;
            if (push_obs_a.row >= 0 && (push_obs_a_is_box || push_obs_a_is_bomb)) {
              candidate_feasible = 0;
            } else if (push_obs_a.row >= 0) {
              candidate_feasible = 0;
              Point cand_targets[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
              size_t cand_count = planner_v3_bfs_collect_bomb_centers_covering_obstacle(
                  rows, cols, push_obs_a,
                  obs_for_push_check, obs_for_push_check_count,
                  cand_targets, PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS);
              if (cand_count == 0 &&
                  planner_v3_bfs_in_bounds(rows, cols, push_obs_a.row, push_obs_a.col)) {
                cand_targets[cand_count++] = push_obs_a;
              }

              int bomb_b_dist_to_a[MAX_BOMBS];
              for (size_t bi2 = 0; bi2 < planner_bomb_count && bi2 < MAX_BOMBS; ++bi2) {
                bomb_b_dist_to_a[bi2] = INT_MAX;
              }
              for (size_t bi2 = 0; bi2 < planner_bomb_count && bi2 < MAX_BOMBS; ++bi2) {
                if (bi2 == b || planner_bombs[bi2].row < 0 || planner_bombs[bi2].col < 0) {
                  continue;
                }
                Point temp_bombs_for_b[MAX_BOMBS];
                size_t tb = 0;
                for (size_t bj2 = 0; bj2 < planner_bomb_count && tb < MAX_BOMBS; ++bj2) {
                  if (bj2 == bi2 || bj2 == b) {
                    continue;
                  }
                  if (planner_bombs[bj2].row < 0 || planner_bombs[bj2].col < 0) {
                    continue;
                  }
                  temp_bombs_for_b[tb++] = planner_bombs[bj2];
                }
                int d_b_to_a = planner_v3_bfs_distance_between(
                    rows, cols,
                    planner_bombs[bi2], planner_bombs[b],
                    obs_for_push_check, obs_for_push_check_count,
                    temp_bombs_for_b, tb,
                    active_boxes, box_count, 1, NULL);
                bomb_b_dist_to_a[bi2] = (d_b_to_a == INT_MAX) ? INT_MAX : d_b_to_a;
              }

              for (size_t n = 0; n < planner_bomb_count; ++n) {
                size_t best_bi2 = SIZE_MAX;
                int best_d_b = INT_MAX;
                for (size_t bi2 = 0; bi2 < planner_bomb_count && bi2 < MAX_BOMBS; ++bi2) {
                  if (bi2 == b || planner_bombs[bi2].row < 0 || planner_bombs[bi2].col < 0) {
                    continue;
                  }
                  if (bomb_b_dist_to_a[bi2] < best_d_b) {
                    best_d_b = bomb_b_dist_to_a[bi2];
                    best_bi2 = bi2;
                  }
                }
                if (best_bi2 == SIZE_MAX) {
                  break;
                }
                bomb_b_dist_to_a[best_bi2] = INT_MAX;

                for (size_t ci = 0; ci < cand_count; ++ci) {
                  Point push_obs_b = planner_v3_bfs_check_bomb_path_push_blocker(
                      rows, cols, planner_bombs[best_bi2], cand_targets[ci],
                      obs_for_push_check, obs_for_push_check_count,
                      planner_bombs, planner_bomb_count, best_bi2,
                      active_boxes, box_count,
                      NULL, NULL);
                  if (push_obs_b.row < 0) {
                    candidate_b_idx = best_bi2;
                    candidate_b_target = cand_targets[ci];
                    candidate_feasible = 1;

                    Point remaining_obstacles[200];
                    size_t remaining_count = planner_v3_bfs_bomb_explode(
                        obs_for_push_check, obs_for_push_check_count,
                        candidate_b_target, remaining_obstacles, 200);
                    for (size_t oi2 = 0; oi2 < obs_for_push_check_count; ++oi2) {
                      if (candidate_b_exploded_count >=
                          PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES) {
                        break;
                      }
                      if (!planner_v3_bfs_point_in_list(
                              obs_for_push_check[oi2], remaining_obstacles, remaining_count)) {
                        candidate_b_exploded[candidate_b_exploded_count++] = obs_for_push_check[oi2];
                      }
                    }
                    break;
                  }
                }
                if (candidate_feasible) {
                  break;
                }
              }
            }

            if (!candidate_feasible) {
              continue;
            }

            if (chosen_bomb_idx == SIZE_MAX || dist < best_dist) {
              chosen_bomb_idx = b;
              chosen_bomb_b_idx = candidate_b_idx;
              chosen_bomb_b_target = candidate_b_target;
              chosen_bomb_b_exploded_count = candidate_b_exploded_count;
              for (size_t eo2 = 0; eo2 < chosen_bomb_b_exploded_count; ++eo2) {
                chosen_bomb_b_exploded[eo2] = candidate_b_exploded[eo2];
              }
              best_dist = dist;
            }
          }
          if (chosen_bomb_idx != SIZE_MAX) {
            if (chosen_bomb_b_idx != SIZE_MAX) {
              planner_v3_bfs_append_bomb_action(
                  plan,
                  chosen_bomb_b_idx, chosen_bomb_b_target,
                  PLANNER_V3_BOMB_PURPOSE_BOMB_A_PUSH_POS,
                  chosen_bomb_b_exploded, chosen_bomb_b_exploded_count);
            }
            uint8_t purpose = plan->reachable
                                  ? PLANNER_V3_BOMB_PURPOSE_OPTIMIZE_PATH
                                  : PLANNER_V3_BOMB_PURPOSE_PATH_OBSTACLE;
            planner_v3_bfs_append_bomb_action(
                plan, chosen_bomb_idx, plan->obstacle_to_clear, purpose,
                plan->exploded_obstacles, plan->exploded_obstacle_count);
          }
        }

        if (!plan->reachable && sp_len > 1) {
          Point first_push_pos = {
              active_boxes[bi].row - (sp_path[1].row - sp_path[0].row),
              active_boxes[bi].col - (sp_path[1].col - sp_path[0].col)};
          size_t first_push_bomb_idx = SIZE_MAX;
          Point first_push_bomb_target = {-1, -1};
          Point first_push_exploded[PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES];
          size_t first_push_exploded_count = 0;
          if (planner_v3_bfs_plan_first_push_bomb_special_path(
                  rows, cols,
                  plan_car, first_push_pos,
                  planner_obstacles, planner_obstacle_count,
                  planner_bombs, planner_bomb_count,
                  active_boxes, box_count,
                  &first_push_bomb_idx,
                  &first_push_bomb_target,
                  first_push_exploded, PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES,
                  &first_push_exploded_count)) {
            planner_v3_bfs_append_bomb_action(
                plan,
                first_push_bomb_idx,
                first_push_bomb_target,
                PLANNER_V3_BOMB_PURPOSE_FIRST_PUSH_POS,
                first_push_exploded_count > 0 ? first_push_exploded : NULL,
                first_push_exploded_count);
          }
        }

        {
          size_t bombs_on_path[MAX_BOMBS];
          size_t bombs_on_path_count = 0;
          planner_v3_bfs_find_bombs_on_path(
              rows, cols,
              sp_path, sp_len,
              planner_bombs, planner_bomb_count,
              bombs_on_path, &bombs_on_path_count);
          for (size_t bp = 0; bp < bombs_on_path_count; ++bp) {
            size_t bidx = bombs_on_path[bp];
            if (bidx >= planner_bomb_count) {
              continue;
            }
            if (planner_bombs[bidx].row < 0 || planner_bombs[bidx].col < 0) {
              continue;
            }
            uint8_t purpose = plan->reachable
                                  ? PLANNER_V3_BOMB_PURPOSE_OPTIMIZE_PATH
                                  : PLANNER_V3_BOMB_PURPOSE_BOMB_A_PUSH_POS;
            planner_v3_bfs_append_bomb_action(plan, bidx, planner_bombs[bidx], purpose, NULL, 0);
          }
        }
      }

      planner_v3_bfs_refresh_plan_exploded_obstacles_from_actions(
          plan, planner_obstacles, planner_obstacle_count);
      planner_v3_bfs_sync_bomb_summary_from_actions(plan);
    }

    active_boxes[bi].row = -1;
    active_boxes[bi].col = -1;
    plan_car = goals[gi].target;
  }
}

static int planner_v3_bfs_apply_bomb_conflict_yield_with_earliest(
    int rows, int cols,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const PlannerBoxGoalBFS *goals,
    const size_t *ordered_goal_indices, size_t ordered_count,
    const PlannerV3PreBoxPlan *plans,
    uint8_t banned_bomb_by_box[PLANNER_V3_BFS_MAX_BOXES][MAX_BOMBS],
    uint8_t yielded_bomb_by_box[PLANNER_V3_BFS_MAX_BOXES][MAX_BOMBS],
    size_t *out_earliest_order) {
  if (!goals || !ordered_goal_indices || !plans || !banned_bomb_by_box) {
    return 0;
  }
  if (out_earliest_order) {
    *out_earliest_order = SIZE_MAX;
  }
  int changed = 0;

  for (size_t bomb_idx = 0; bomb_idx < MAX_BOMBS; ++bomb_idx) {
    size_t winner_box = SIZE_MAX;
    int winner_priority = INT_MAX;
    int winner_score = INT_MAX;

    for (size_t oi = 0; oi < ordered_count; ++oi) {
      size_t gi = ordered_goal_indices[oi];
      size_t bi = goals[gi].source_idx;
      if (bi >= PLANNER_V3_BFS_MAX_BOXES) {
        continue;
      }
      if (banned_bomb_by_box[bi][bomb_idx]) {
        continue;
      }
      const PlannerV3PreBoxPlan *plan = &plans[bi];
      int priority = INT_MAX;
      Point bomb_target = {-1, -1};
      if (!planner_v3_bfs_plan_get_bomb_priority_and_target(
              plan, bomb_idx, &priority, &bomb_target)) {
        continue;
      }

      Point bomb_pos = {-1, -1};
      if (bombs && bomb_idx < bomb_count) {
        bomb_pos = bombs[bomb_idx];
      }
      int tie_score = planner_v3_bfs_bomb_tie_break_score(
          rows, cols,
          bomb_pos, goals[gi].box, bomb_target,
          obstacles, obstacle_count);

      if (winner_box == SIZE_MAX ||
          priority < winner_priority ||
          (priority == winner_priority && tie_score < winner_score)) {
        winner_box = bi;
        winner_priority = priority;
        winner_score = tie_score;
      }
    }

    if (winner_box == SIZE_MAX) {
      continue;
    }

    {
      // 如果某个原本要退让的箱子会因此失去全部 PATH_OBSTACLE 炸弹，
      // 则该箱子保留本炸弹，改为让原赢家退让。
      size_t protected_box = SIZE_MAX;
      for (size_t oi = 0; oi < ordered_count; ++oi) {
        size_t gi = ordered_goal_indices[oi];
        size_t bi = goals[gi].source_idx;
        if (bi >= PLANNER_V3_BFS_MAX_BOXES || bi == winner_box) {
          continue;
        }
        if (banned_bomb_by_box[bi][bomb_idx]) {
          continue;
        }
        const PlannerV3PreBoxPlan *plan = &plans[bi];
        if (!planner_v3_bfs_plan_get_bomb_priority_and_target(
                plan, bomb_idx, NULL, NULL)) {
          continue;
        }
        if (!planner_v3_bfs_plan_has_any_path_obstacle_bomb(plan)) {
          continue;
        }
        if (planner_v3_bfs_plan_has_available_path_obstacle_bomb_after_forced_ban(
                plan, banned_bomb_by_box[bi], bomb_idx)) {
          continue;
        }
        protected_box = bi;
        break;
      }
      if (protected_box != SIZE_MAX) {
        winner_box = protected_box;
      }
    }

    for (size_t oi = 0; oi < ordered_count; ++oi) {
      size_t gi = ordered_goal_indices[oi];
      size_t bi = goals[gi].source_idx;
      if (bi >= PLANNER_V3_BFS_MAX_BOXES || bi == winner_box) {
        continue;
      }
      if (banned_bomb_by_box[bi][bomb_idx]) {
        continue;
      }
      const PlannerV3PreBoxPlan *plan = &plans[bi];
      if (!planner_v3_bfs_plan_get_bomb_priority_and_target(
              plan, bomb_idx, NULL, NULL)) {
        continue;
      }
      banned_bomb_by_box[bi][bomb_idx] = 1;
      if (yielded_bomb_by_box) {
        yielded_bomb_by_box[bi][bomb_idx] = 1;
      }
      if (out_earliest_order && oi < *out_earliest_order) {
        *out_earliest_order = oi;
      }
      changed = 1;
    }
  }

  return changed;
}

static void planner_v3_bfs_sync_two_phase_from_preplans(
    const PlannerBoxGoalBFS *goals, size_t goal_count,
    const Point *bombs, size_t bomb_count,
    PlannerV3TwoPhaseRuntime *runtime) {
  if (!runtime || !goals) {
    return;
  }

  for (size_t bi = 0; bi < PLANNER_V3_BFS_MAX_BOXES; ++bi) {
    PlannerV3BoxTwoPhasePlan *entry = &runtime->two_phase.box_plan[bi];
    entry->special_path_valid = 0;
    entry->special_path_len = 0;
    entry->has_obstacle_to_clear = 0;
    entry->obstacle_to_clear.row = -1;
    entry->obstacle_to_clear.col = -1;
    entry->bomb_plan_count = 0;
    entry->predicted_exploded_obstacle_count = 0;
  }

  for (size_t oi = 0; oi < runtime->ordered_goal_count; ++oi) {
    size_t gi = runtime->ordered_goal_indices[oi];
    if (gi >= goal_count) {
      continue;
    }
    size_t bi = goals[gi].source_idx;
    if (bi >= PLANNER_V3_BFS_MAX_BOXES) {
      continue;
    }
    PlannerV3BoxTwoPhasePlan *entry = &runtime->two_phase.box_plan[bi];
    const PlannerV3PreBoxPlan *plan = &runtime->preplans[bi];

    entry->planned_order = oi;
    entry->box_index = bi;
    entry->box_start = goals[gi].box;
    entry->target = goals[gi].target;
    entry->special_path_valid = plan->special_path_valid ? 1 : 0;
    entry->special_path_len = 0;
    if (entry->special_path_valid && plan->special_path_len > 0) {
      entry->special_path_len =
          (plan->special_path_len < PLANNER_V3_TWO_PHASE_MAX_PATH_LEN)
              ? plan->special_path_len
              : PLANNER_V3_TWO_PHASE_MAX_PATH_LEN;
      for (size_t pi = 0; pi < entry->special_path_len; ++pi) {
        entry->special_path[pi] = plan->special_path[pi];
      }
    }

    entry->has_obstacle_to_clear = plan->has_obstacle_to_clear ? 1 : 0;
    entry->obstacle_to_clear = plan->obstacle_to_clear;

    entry->predicted_exploded_obstacle_count =
        (plan->exploded_obstacle_count < PLANNER_V3_TWO_PHASE_MAX_EXPLODED_OBSTACLES)
            ? plan->exploded_obstacle_count
            : PLANNER_V3_TWO_PHASE_MAX_EXPLODED_OBSTACLES;
    for (size_t eo = 0; eo < entry->predicted_exploded_obstacle_count; ++eo) {
      entry->predicted_exploded_obstacles[eo] = plan->exploded_obstacles[eo];
    }

    entry->bomb_plan_count =
        (plan->bomb_action_count < PLANNER_V3_TWO_PHASE_MAX_BOMBS)
            ? plan->bomb_action_count
            : PLANNER_V3_TWO_PHASE_MAX_BOMBS;
    for (size_t u = 0; u < entry->bomb_plan_count; ++u) {
      size_t bomb_idx = plan->bomb_actions[u].bomb_idx;
      PlannerV3BombPreplan *bomb_plan = &entry->bomb_plan[u];
      memset(bomb_plan, 0, sizeof(*bomb_plan));
      bomb_plan->bomb_index = bomb_idx;
      if (bombs && bomb_idx < bomb_count) {
        bomb_plan->bomb_position = bombs[bomb_idx];
      } else {
        bomb_plan->bomb_position.row = -1;
        bomb_plan->bomb_position.col = -1;
      }
      bomb_plan->bomb_target = plan->bomb_actions[u].bomb_target;
      bomb_plan->bomb_purpose = plan->bomb_actions[u].bomb_purpose;
      bomb_plan->yielded_by_conflict =
          (bomb_idx < MAX_BOMBS && runtime->yielded_bomb_by_box[bi][bomb_idx]) ? 1 : 0;
      bomb_plan->exploded_obstacle_count =
          (plan->bomb_actions[u].exploded_obstacle_count < PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES)
              ? plan->bomb_actions[u].exploded_obstacle_count
              : PLANNER_V3_BFS_MAX_SPECIAL_PATH_OBSTACLES;
      for (size_t eo = 0; eo < bomb_plan->exploded_obstacle_count; ++eo) {
        bomb_plan->exploded_obstacles[eo] = plan->bomb_actions[u].exploded_obstacles[eo];
      }
    }
  }
}

static void planner_v3_bfs_publish_two_phase_to_special_paths(
    int rows, int cols,
    const Point *bombs, size_t bomb_count,
    PlannerV3TwoPhaseRuntime *runtime) {
  if (!runtime) {
    return;
  }
  special_paths.box_count = runtime->two_phase.box_count;
  for (size_t bi = 0; bi < special_paths.box_count && bi < PLANNER_V3_BFS_MAX_BOXES; ++bi) {
    PlannerBoxPathOutput *out = &special_paths.box_paths[bi];
    PlannerV3BoxTwoPhasePlan *entry = &runtime->two_phase.box_plan[bi];
    memset(out, 0, sizeof(*out));
    out->planned_order = -1;
    out->pre_reachable = entry->normal_reachable ? 1 : 0;
    if (entry->planned_order == SIZE_MAX) {
      continue;
    }

    out->planned_order = (int)entry->planned_order;
    if (entry->special_path_valid && entry->special_path_len > 0) {
      out->valid = 1;
      out->path_len = (entry->special_path_len < 400) ? entry->special_path_len : 400;
      for (size_t pi = 0; pi < out->path_len; ++pi) {
        out->path[pi] = entry->special_path[pi];
      }
    }

    out->has_obstacle_to_clear = entry->has_obstacle_to_clear ? 1 : 0;
    out->obstacle_to_clear = entry->obstacle_to_clear;
    out->dropped_req1 = 0;
    out->dropped_req2 = 0;
    out->dropped_req3 = 0;

    out->planned_bomb_count = (int)((entry->bomb_plan_count <= 5) ? entry->bomb_plan_count : 5);
    for (size_t u = 0; u < (size_t)out->planned_bomb_count; ++u) {
      size_t bomb_idx = entry->bomb_plan[u].bomb_index;
      out->planned_bomb_target[u] = entry->bomb_plan[u].bomb_target;
      out->planned_bomb_purpose[u] = (int)entry->bomb_plan[u].bomb_purpose;
      if (bombs && bomb_idx < bomb_count) {
        out->planned_bomb_pos[u] = bombs[bomb_idx];
      } else {
        out->planned_bomb_pos[u].row = -1;
        out->planned_bomb_pos[u].col = -1;
      }
    }

    out->planned_exploded_obstacle_count =
        (int)((entry->predicted_exploded_obstacle_count <= 20)
                  ? entry->predicted_exploded_obstacle_count
                  : 20);
    for (size_t eo = 0; eo < (size_t)out->planned_exploded_obstacle_count; ++eo) {
      out->planned_exploded_obstacles[eo] = entry->predicted_exploded_obstacles[eo];
    }

    {
      size_t bombs_on_path_idx[MAX_BOMBS];
      size_t bombs_on_path_cnt = 0;
      planner_v3_bfs_find_bombs_on_path(
          rows, cols, out->path, out->path_len,
          bombs, bomb_count,
          bombs_on_path_idx, &bombs_on_path_cnt);
      out->bomb_on_path_count = (int)((bombs_on_path_cnt <= 5) ? bombs_on_path_cnt : 5);
      for (size_t bop = 0; bop < (size_t)out->bomb_on_path_count; ++bop) {
        out->bomb_on_path[bop] = bombs[bombs_on_path_idx[bop]];
      }
    }
  }
}

static void resolve_bomb_conflicts_and_replan(
    int rows, int cols,
    const PlannerBoxGoalBFS *goals, size_t goal_count, size_t box_count,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    Point car,
    PlannerV3TwoPhaseRuntime *runtime) {
  if (!runtime || !goals) {
    return;
  }

  memset(runtime->banned_bomb_by_box, 0, sizeof(runtime->banned_bomb_by_box));
  memset(runtime->yielded_bomb_by_box, 0, sizeof(runtime->yielded_bomb_by_box));

  size_t replan_start = 0;
  const size_t max_iter = PLANNER_V3_BFS_MAX_BOXES * MAX_BOMBS + 1;
  for (size_t iter = 0; iter < max_iter; ++iter) {
    build_special_preplans(
        rows, cols,
        goals, goal_count, box_count,
        obstacles, obstacle_count,
        bombs, bomb_count,
        car,
        replan_start,
        runtime);

    size_t earliest = SIZE_MAX;
    if (!planner_v3_bfs_apply_bomb_conflict_yield_with_earliest(
            rows, cols,
            obstacles, obstacle_count,
            bombs, bomb_count,
            goals,
            runtime->ordered_goal_indices, runtime->ordered_goal_count,
            runtime->preplans,
            runtime->banned_bomb_by_box,
            runtime->yielded_bomb_by_box,
            &earliest)) {
      break;
    }
    if (earliest == SIZE_MAX) {
      break;
    }
    replan_start = earliest;
  }

  planner_v3_bfs_sync_two_phase_from_preplans(
      goals, goal_count,
      bombs, bomb_count,
      runtime);
  planner_v3_bfs_publish_two_phase_to_special_paths(rows, cols, bombs, bomb_count, runtime);
}

// ?????????????
// ?????????????
static int planner_v3_bfs_resolve_first_push_by_bomb_special_path(
    int rows, int cols,
    Point *car_pos, Point first_push_pos,
    Point *obstacles, size_t *obstacle_count,
    Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps) {
  last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_NONE;
  PlannerV3ProbeScratch *probe_scratch = &s_probe_scratch;
  size_t bomb_order[MAX_BOMBS];
  int bomb_score[MAX_BOMBS];
  size_t bomb_order_count = 0;
  for (size_t bi = 0; bi < bomb_count && bomb_order_count < MAX_BOMBS; ++bi) {
    if (bombs[bi].row < 0 || bombs[bi].col < 0) {
      continue;
    }
    Point *temp_bombs = probe_scratch->temp_bombs;
    size_t temp_bomb_count = 0;
    for (size_t bj = 0; bj < bomb_count && temp_bomb_count < MAX_BOMBS; ++bj) {
      if (bj == bi) continue;
      if (bombs[bj].row < 0 || bombs[bj].col < 0) continue;
      temp_bombs[temp_bomb_count++] = bombs[bj];
    }
    {
      int md = planner_v3_bfs_abs(bombs[bi].row - first_push_pos.row) +
               planner_v3_bfs_abs(bombs[bi].col - first_push_pos.col);
      int car_to_bomb = planner_v3_bfs_distance_between(
          rows, cols, *car_pos, bombs[bi],
          obstacles, *obstacle_count, temp_bombs, temp_bomb_count,
          boxes, box_count, 1, NULL);
      int score = INT_MAX;
      if (car_to_bomb != INT_MAX) {
        score = md + car_to_bomb;
      }
      bomb_order[bomb_order_count] = bi;
      bomb_score[bomb_order_count] = score;
      bomb_order_count++;
    }
  }
  if (bomb_order_count == 0) {
    last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_NO_BOMB_CANDIDATE;
    return 0;
  }
  for (size_t i = 0; i < bomb_order_count; ++i) {
    for (size_t j = i + 1; j < bomb_order_count; ++j) {
      if (bomb_score[j] < bomb_score[i]) {
        int td = bomb_score[i];
        bomb_score[i] = bomb_score[j];
        bomb_score[j] = td;
        size_t ti = bomb_order[i];
        bomb_order[i] = bomb_order[j];
        bomb_order[j] = ti;
      }
    }
  }

  for (size_t oi = 0; oi < bomb_order_count; ++oi) {
    size_t bomb_idx = bomb_order[oi];

    Point saved_car = *car_pos;
    size_t saved_steps = *out_steps;
    Point saved_bombs[MAX_BOMBS];
    memcpy(saved_bombs, bombs, bomb_count * sizeof(Point));
    Point saved_obstacles[200];
    size_t saved_obstacle_count = *obstacle_count;
    if (saved_obstacle_count > 200) {
      saved_obstacle_count = 200;
    }
    memcpy(saved_obstacles, obstacles, saved_obstacle_count * sizeof(Point));

    Point sp_obstacle = {-1, -1};
    int sp_has_obstacle = 0;
    Point sp_path[PLANNER_V3_BFS_MAX_PATH_LEN];
    size_t sp_len = 0;

    Point planning_obstacles[200];
    size_t planning_obstacle_count = *obstacle_count;
    if (planning_obstacle_count > 200) {
      planning_obstacle_count = 200;
    }
    memcpy(planning_obstacles, obstacles, planning_obstacle_count * sizeof(Point));

    int plan_ok = 0;
    {
      const int max_limit_replans = 200;
    for (int repl = 0; repl < max_limit_replans; ++repl) {
      if (!planner_v3_bfs_special_path_planning(
              rows, cols, bombs[bomb_idx], first_push_pos,
              planning_obstacles, planning_obstacle_count,
              bombs, bomb_count,
              boxes, box_count, SIZE_MAX,
              *car_pos,
              &sp_obstacle, &sp_has_obstacle,
              sp_path, PLANNER_V3_BFS_MAX_PATH_LEN, &sp_len,
              1, 0) || sp_len == 0) {
        last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_SPECIAL_PATH_PLAN_FAILED;
        break;
      }

      Point legal_sp_centers[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
      size_t legal_sp_center_count = planner_v3_bfs_obstacles_destroyable_by_one_bomb(
          s_special_path_obstacles, s_special_path_obstacle_count,
          planning_obstacles, planning_obstacle_count,
          legal_sp_centers, PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS);
      if (legal_sp_center_count > 0) {
        plan_ok = 1;
        break;
      }

      Point nearest_obs = {-1, -1};
      int nearest_md = INT_MAX;
      for (size_t si = 0; si < s_special_path_obstacle_count; ++si) {
        int md = planner_v3_bfs_abs(bombs[bomb_idx].row - s_special_path_obstacles[si].row) +
                 planner_v3_bfs_abs(bombs[bomb_idx].col - s_special_path_obstacles[si].col);
        if (md < nearest_md) {
          nearest_md = md;
          nearest_obs = s_special_path_obstacles[si];
        }
      }
      if (nearest_obs.row < 0 || nearest_obs.col < 0) {
        last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_SPECIAL_PATH_PLAN_FAILED;
        break;
      }

      size_t new_plan_count = 0;
      for (size_t pi = 0; pi < planning_obstacle_count && new_plan_count < 200; ++pi) {
        if (planning_obstacles[pi].row == nearest_obs.row &&
            planning_obstacles[pi].col == nearest_obs.col) {
          continue;
        }
        planning_obstacles[new_plan_count++] = planning_obstacles[pi];
      }
      if (new_plan_count == planning_obstacle_count) {
        last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_SPECIAL_PATH_PLAN_FAILED;
        break;
      }
      planning_obstacle_count = new_plan_count;
      }
    }

    if (!plan_ok) {
      if (last_first_push_bomb_special_fail_reason == FIRST_PUSH_BOMB_FAIL_NONE) {
        last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_SPECIAL_PATH_PLAN_FAILED;
      }
      memcpy(obstacles, saved_obstacles, saved_obstacle_count * sizeof(Point));
      *obstacle_count = saved_obstacle_count;
      memcpy(bombs, saved_bombs, bomb_count * sizeof(Point));
      *car_pos = saved_car;
      *out_steps = saved_steps;
      continue;
    }

    int handled = 0;
    if (sp_has_obstacle && s_special_path_obstacle_count > 0) {
      Point explode_candidates[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
      size_t explode_candidate_count = planner_v3_bfs_obstacles_destroyable_by_one_bomb(
          s_special_path_obstacles, s_special_path_obstacle_count,
          obstacles, *obstacle_count,
          explode_candidates, PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS);
      Point explode_target = {-1, -1};
      int explode_md = INT_MAX;
      for (size_t si = 0; si < explode_candidate_count; ++si) {
        int md = planner_v3_bfs_abs(bombs[bomb_idx].row - explode_candidates[si].row) +
                 planner_v3_bfs_abs(bombs[bomb_idx].col - explode_candidates[si].col);
        if (md < explode_md) {
          explode_md = md;
          explode_target = explode_candidates[si];
        }
      }
      if (explode_target.row < 0 || explode_target.col < 0) {
        last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_SPECIAL_PATH_PLAN_FAILED;
        memcpy(obstacles, saved_obstacles, saved_obstacle_count * sizeof(Point));
        *obstacle_count = saved_obstacle_count;
        memcpy(bombs, saved_bombs, bomb_count * sizeof(Point));
        *car_pos = saved_car;
        *out_steps = saved_steps;
        continue;
      }

      int push_ret = planner_v3_bfs_push_bomb(
          rows, cols, car_pos,
          bombs, bomb_count, bomb_idx,
          explode_target, explode_target,
          obstacles, *obstacle_count,
          boxes, box_count,
          path_buffer, path_capacity, out_steps,
          s_special_path_obstacles, s_special_path_obstacle_count);
      if (push_ret == -7) {
        last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_PATH_BUFFER_OVERFLOW;
        return -7;
      }
      if (push_ret == 1) {
        Point exploded_at = bombs[bomb_idx];
        Point new_obstacles[200];
        size_t new_obstacle_count = planner_v3_bfs_bomb_explode(
            obstacles, *obstacle_count, exploded_at, new_obstacles, 200);
        memcpy(obstacles, new_obstacles, new_obstacle_count * sizeof(Point));
        *obstacle_count = new_obstacle_count;
        bombs[bomb_idx].row = -1;
        bombs[bomb_idx].col = -1;
        handled = 1;
      } else {
        last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_PUSH_BOMB_FAILED;
      }
    } else {
      for (size_t pi = 1; pi < sp_len; ++pi) {
        Point step_target = sp_path[pi];
        int push_ret = planner_v3_bfs_push_bomb(
            rows, cols, car_pos,
            bombs, bomb_count, bomb_idx,
            step_target, step_target,
            obstacles, *obstacle_count,
            boxes, box_count,
            path_buffer, path_capacity, out_steps,
            NULL, 0);
        if (push_ret == -7) {
          last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_PATH_BUFFER_OVERFLOW;
          return -7;
        }
        if (push_ret != 1) {
          last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_PUSH_STEP_FAILED;
          break;
        }
        {
          int d = planner_v3_bfs_distance_between(
              rows, cols, *car_pos, first_push_pos,
              obstacles, *obstacle_count, bombs, bomb_count,
              boxes, box_count, 1, NULL);
          if (d != INT_MAX) {
            handled = 1;
            break;
          }
        }
      }
    }

    if (handled) {
      last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_NONE;
      return 1;
    }

    memcpy(obstacles, saved_obstacles, saved_obstacle_count * sizeof(Point));
    *obstacle_count = saved_obstacle_count;
    memcpy(bombs, saved_bombs, bomb_count * sizeof(Point));
    *car_pos = saved_car;
    *out_steps = saved_steps;
  }

  if (last_first_push_bomb_special_fail_reason == FIRST_PUSH_BOMB_FAIL_NONE) {
    last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_ALL_ATTEMPTS_FAILED;
  }
  return 0;
}

// ????? goals ??????????????????????????????????????????��??/????????
static int planner_v3_bfs_execute_box_path_fixed(
    int rows, int cols,
    Point *car_pos,
    Point *current_boxes, size_t box_count, size_t box_idx,
    Point target,
    const Point *current_obstacles, size_t current_obstacle_count,
    Point *bombs_mutable, size_t bomb_count,
    const Point *box_path, size_t box_path_len,
    Point *path_buffer, size_t path_capacity, size_t *out_steps,
    Point *box_path_trace, size_t *box_path_trace_len) {
  if (!car_pos || !current_boxes ||
      !path_buffer || !out_steps || box_idx >= box_count) {
    return -6;
  }

  Point box = current_boxes[box_idx];
  if (box.row < 0 || box.col < 0) {
    last_err_stage = 2;
    last_err_detail = LAST_ERR_DETAIL_V3_SOURCE_BOX_NOT_FOUND;
    return -6;
  }

  int has_box_path = (box_path && box_path_len > 1) ? 1 : 0;
  int box_path_steps = INT_MAX;
  if (has_box_path) {
    box_path_steps = planner_v3_bfs_simulate_push_box_path(
        rows, cols, *car_pos, box, target,
        box_path, box_path_len,
        current_obstacles, current_obstacle_count,
        bombs_mutable, bomb_count,
        current_boxes, box_count, box_idx, 1);
  }
  int box_score_steps = planner_v3_bfs_simulate_push_box_score(
      rows, cols, *car_pos, box, target,
      current_obstacles, current_obstacle_count,
      bombs_mutable, bomb_count,
      current_boxes, box_count, box_idx, 1);

  int chosen_box_strategy = PLANNER_V3_BFS_STRATEGY_SCORE;
  if (planner_v3_bfs_evaluate_push_strategy(
          has_box_path, box_path_steps, box_score_steps,
          0, 1, 1,
          2,
          LAST_ERR_DETAIL_V3_PATH_ONLY_INVALID,
          LAST_ERR_DETAIL_V3_PATH_AND_SCORE_BOTH_FAILED,
          LAST_ERR_DETAIL_V3_SELECTED_PATH_INVALID,
          LAST_ERR_DETAIL_V3_SELECTED_SCORE_INVALID,
          &chosen_box_strategy) < 0) {
    return -6;
  }

  if (chosen_box_strategy == PLANNER_V3_BFS_STRATEGY_PATH) {
    for (size_t pi = 1; pi < box_path_len; ++pi) {
      Point next_in_path = box_path[pi];
      int dr = next_in_path.row - box.row;
      int dc = next_in_path.col - box.col;
      if (planner_v3_bfs_abs(dr) + planner_v3_bfs_abs(dc) != 1) {
        last_err_stage = 2;
        last_err_detail = LAST_ERR_DETAIL_V3_BOX_PATH_STEP_NOT_ADJACENT;
        return -6;
      }

      int push_row = box.row - dr;
      int push_col = box.col - dc;
      if (!planner_v3_bfs_in_bounds(rows, cols, next_in_path.row, next_in_path.col) ||
          !planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
        last_err_stage = 2;
        last_err_detail = LAST_ERR_DETAIL_V3_BOX_PATH_STEP_OUT_OF_BOUNDS;
        return -6;
      }

      uint8_t occ[PLANNER_V3_BFS_MAX_CELLS];
      planner_v3_bfs_build_occupancy_grid(
          rows, cols,
          current_obstacles, current_obstacle_count,
          bombs_mutable, bomb_count,
          current_boxes, box_count,
          1, occ);

      if (planner_v3_bfs_is_obstacle(occ, cols, next_in_path.row, next_in_path.col) ||
          planner_v3_bfs_is_obstacle(occ, cols, push_row, push_col)) {
        last_err_stage = 2;
        last_err_detail = LAST_ERR_DETAIL_V3_BOX_OR_PUSH_ON_OBSTACLE;
        return -6;
      }
      if (planner_v3_bfs_is_box_at(occ, cols, next_in_path.row, next_in_path.col, box_idx) ||
          planner_v3_bfs_is_box_at(occ, cols, push_row, push_col, box_idx)) {
        last_err_stage = 2;
        last_err_detail = LAST_ERR_DETAIL_V3_BOX_OR_PUSH_COLLIDE_BOX;
        return -6;
      }

      Point push_from = {push_row, push_col};
      if (car_pos->row != push_from.row || car_pos->col != push_from.col) {
        int car_result = planner_v3_bfs_move_car_to_push_with_remaining_state(
            rows, cols,
            car_pos, push_from,
            current_obstacles, current_obstacle_count,
            bombs_mutable, bomb_count,
            current_boxes, box_count,
            path_buffer, path_capacity, out_steps,
            occ,
            2, LAST_ERR_DETAIL_V3_CAR_CANNOT_REACH_PUSH_POS);
        if (car_result != 1) {
          return car_result;
        }
      }

      PlannerV3PushExecuteCtx exec_ctx = {
        push_from, next_in_path,
        car_pos, &box, current_boxes, box_idx,
        path_buffer, path_capacity, out_steps,
        2, LAST_ERR_DETAIL_V3_PUSH_EXEC_CAR_MISMATCH,
        LAST_ERR_DETAIL_V3_PUSH_EXEC_NOT_ADJACENT,
        LAST_ERR_DETAIL_V3_PUSH_EXEC_PATH_NOT_CONTINUOUS
      };
      int exec_ret = planner_v3_bfs_execute_push_strategy(&exec_ctx);
      if (exec_ret < 0) {
        return exec_ret;
      }

      if (box_path_trace && box_path_trace_len &&
          *box_path_trace_len < PLANNER_V3_BFS_MAX_CELLS) {
        box_path_trace[(*box_path_trace_len)++] = box;
      }
    }
  } else {
    int last_dr = 0;
    int last_dc = 0;
    int reverse_count = 0;
    size_t exec_steps = 0;
    Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
    size_t temp_count = 0;
    for (size_t i = 0; i < box_count && i < PLANNER_V3_BFS_MAX_BOXES; ++i) {
      if (i == box_idx) {
        continue;
      }
      if (current_boxes[i].row < 0 || current_boxes[i].col < 0) {
        continue;
      }
      temp_boxes[temp_count++] = current_boxes[i];
    }

    int dist_to_target[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_target(
            rows, cols, target,
            current_obstacles, current_obstacle_count,
            bombs_mutable, bomb_count,
            temp_boxes, temp_count,
            dist_to_target, 1, NULL)) {
      last_err_stage = 2;
      last_err_detail = LAST_ERR_DETAIL_V3_PATH_AND_SCORE_BOTH_FAILED;
      return -6;
    }

    while (box.row != target.row || box.col != target.col) {
      if (exec_steps >= (size_t)PLANNER_V3_BFS_MAX_GREEDY_STEPS) {
        last_err_stage = 2;
        last_err_detail = LAST_ERR_DETAIL_V3_SCORE_EXEC_STEPS_OVERFLOW;
        return -6;
      }

      uint8_t occ[PLANNER_V3_BFS_MAX_CELLS];
      planner_v3_bfs_build_occupancy_grid(
          rows, cols,
          current_obstacles, current_obstacle_count,
          bombs_mutable, bomb_count,
          current_boxes, box_count,
          1, occ);

      int dist_from_car[PLANNER_V3_BFS_MAX_CELLS];
      if (!planner_v3_bfs_global_bfs_from_point_use_occ(rows, cols, *car_pos, occ, dist_from_car, 1)) {
        last_err_stage = 2;
        last_err_detail = LAST_ERR_DETAIL_V3_SCORE_NO_FEASIBLE_DIRECTION;
        return -6;
      }

      PlannerV3PushScoreCandidate candidates[4];
      int sorted_dirs[4];
      int sorted_count = 0;
      if (!planner_v3_bfs_evaluate_push_candidates_score(
              rows, cols, box, target, box_idx, occ,
              dist_to_target, dist_from_car,
              0, last_dr, last_dc, reverse_count,
              candidates, sorted_dirs, &sorted_count)) {
        last_err_stage = 2;
        last_err_detail = LAST_ERR_DETAIL_V3_SCORE_NO_FEASIBLE_DIRECTION;
        return -6;
      }

      PlannerV3PushScoreCandidate chosen;
      int dir_ok = 0;
      for (int k = 0; k < sorted_count; ++k) {
        chosen = candidates[sorted_dirs[k]];
        int next_on_boundary = (chosen.next_pos.row == 0 || chosen.next_pos.row == rows - 1 ||
                                chosen.next_pos.col == 0 || chosen.next_pos.col == cols - 1);
        if (next_on_boundary) {
          if (!planner_v3_bfs_validate_boundary_path(
                  rows, cols, chosen.next_pos, target,
                  *car_pos,
                  current_obstacles, current_obstacle_count,
                  bombs_mutable, bomb_count,
                  current_boxes, box_count,
                  box_idx, 1, occ)) {
            continue;
          }
        }
        if (chosen.next_pos.row != target.row || chosen.next_pos.col != target.col) {
          if (!planner_v3_bfs_can_reach_goal(
                  rows, cols,
                  current_obstacles, current_obstacle_count,
                  bombs_mutable, bomb_count,
                  current_boxes, box_count,
                  box_idx, chosen.next_pos, target, 1, occ)) {
            continue;
          }
        }
        if (dist_from_car[chosen.push_from.row * cols + chosen.push_from.col] == INT_MAX) {
          continue;
        }
        dir_ok = 1;
        break;
      }
      if (!dir_ok) {
        last_err_stage = 2;
        last_err_detail = LAST_ERR_DETAIL_V3_SCORE_DIR_SELECTION_FAILED;
        return -6;
      }

      Point push_from = chosen.push_from;
      if (car_pos->row != push_from.row || car_pos->col != push_from.col) {
        int car_result = planner_v3_bfs_move_car_to_push_with_remaining_state(
            rows, cols,
            car_pos, push_from,
            current_obstacles, current_obstacle_count,
            bombs_mutable, bomb_count,
            current_boxes, box_count,
            path_buffer, path_capacity, out_steps,
            occ,
            2, LAST_ERR_DETAIL_V3_SCORE_CAR_CANNOT_REACH_PUSH_POS);
        if (car_result != 1) {
          return car_result;
        }
      }

      PlannerV3PushExecuteCtx exec_ctx = {
        push_from, chosen.next_pos,
        car_pos, &box, current_boxes, box_idx,
        path_buffer, path_capacity, out_steps,
        2, LAST_ERR_DETAIL_V3_SCORE_EXEC_CAR_MISMATCH,
        LAST_ERR_DETAIL_V3_SCORE_EXEC_NOT_ADJACENT,
        LAST_ERR_DETAIL_V3_SCORE_EXEC_PATH_NOT_CONTINUOUS
      };
      int exec_ret = planner_v3_bfs_execute_push_strategy(&exec_ctx);
      if (exec_ret < 0) {
        return exec_ret;
      }

      if (box_path_trace && box_path_trace_len &&
          *box_path_trace_len < PLANNER_V3_BFS_MAX_CELLS) {
        box_path_trace[(*box_path_trace_len)++] = box;
      }

      exec_steps++;
      if (last_dr == -chosen.dr && last_dc == -chosen.dc &&
          (last_dr != 0 || last_dc != 0)) {
        reverse_count++;
      } else {
        reverse_count = 0;
      }
      last_dr = chosen.dr;
      last_dc = chosen.dc;
    }
  }

  if (box.row != target.row || box.col != target.col) {
    last_err_stage = 2;
    last_err_detail = LAST_ERR_DETAIL_V3_BOX_FINAL_NOT_AT_TARGET;
    return -6;
  }
  return 1;
}

static int planner_v3_bfs_execute_single_bomb_action(
    int rows, int cols,
    Point *car_pos,
    Point *current_obstacles, size_t *current_obstacle_count,
    Point *bombs_mutable, size_t bomb_count,
    const Point *current_boxes, size_t box_count,
    const PlannerV3BombAction *action,
    size_t action_index,
    Point *path_buffer, size_t path_capacity, size_t *out_steps) {
  if (!car_pos || !current_obstacles || !current_obstacle_count ||
      !bombs_mutable || !action || !path_buffer || !out_steps) {
    return -1;
  }

  s_last_failed_bomb_action_index = -1;
  s_last_failed_bomb_action_bomb_idx = SIZE_MAX;
  s_last_failed_bomb_action_target.row = -1;
  s_last_failed_bomb_action_target.col = -1;

  if (action->bomb_idx >= bomb_count || action->bomb_idx >= MAX_BOMBS) {
    return 1;
  }
  if (!planner_v3_bfs_in_bounds(rows, cols, action->bomb_target.row, action->bomb_target.col)) {
    return 1;
  }
  if (bombs_mutable[action->bomb_idx].row < 0 || bombs_mutable[action->bomb_idx].col < 0) {
    return 1;
  }

  int bomb_ret = planner_v3_bfs_push_bomb(
      rows, cols, car_pos,
      bombs_mutable, bomb_count, action->bomb_idx,
      action->bomb_target, action->bomb_target,
      current_obstacles, *current_obstacle_count,
      current_boxes, box_count,
      path_buffer, path_capacity, out_steps,
      action->exploded_obstacles, action->exploded_obstacle_count);
  if (bomb_ret == -7) {
    s_last_failed_bomb_action_index = (int)action_index;
    s_last_failed_bomb_action_bomb_idx = action->bomb_idx;
    s_last_failed_bomb_action_target = action->bomb_target;
    return -7;
  }
  if (bomb_ret != 1) {
    s_last_failed_bomb_action_index = (int)action_index;
    s_last_failed_bomb_action_bomb_idx = action->bomb_idx;
    s_last_failed_bomb_action_target = action->bomb_target;
    last_err_detail_v3_stage3 = planner_v3_bfs_pick_bomb_push_subreason();
    last_err_stage = 3;
    last_err_detail = LAST_ERR_DETAIL_V3_BOMB_PUSH_FAILED;
    return -6;
  }

  Point exploded_at = bombs_mutable[action->bomb_idx];
  Point next_obstacles[200];
  size_t next_count = planner_v3_bfs_bomb_explode(
      current_obstacles, *current_obstacle_count,
      exploded_at, next_obstacles, 200);
  memcpy(current_obstacles, next_obstacles, next_count * sizeof(Point));
  *current_obstacle_count = next_count;
  bombs_mutable[action->bomb_idx].row = -1;
  bombs_mutable[action->bomb_idx].col = -1;
  return 1;
}

static int planner_v3_bfs_execute_planned_bomb_actions(
    int rows, int cols,
    Point *car_pos,
    Point *current_obstacles, size_t *current_obstacle_count,
    Point *bombs_mutable, size_t bomb_count,
    const Point *current_boxes, size_t box_count,
    const PlannerV3BombAction *actions, size_t action_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps) {
  if (!actions || action_count == 0) {
    return 1;
  }
  for (size_t ai = 0; ai < action_count; ++ai) {
    int ret = planner_v3_bfs_execute_single_bomb_action(
        rows, cols,
        car_pos,
        current_obstacles, current_obstacle_count,
        bombs_mutable, bomb_count,
        current_boxes, box_count,
        &actions[ai], ai,
        path_buffer, path_capacity, out_steps);
    if (ret != 1) {
      return ret;
    }
  }
  return 1;
}

static int execute_final_plan(
    int rows, int cols, Point car,
    const PlannerBoxGoalBFS *goals, size_t goal_count, size_t box_count,
    const Point *obstacles, size_t obstacle_count,
    Point *bombs_mutable, size_t bomb_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps,
    int include_return_to_start_path,
    PlannerV3TwoPhaseRuntime *runtime,
    PlannerAllBoxPaths *out_final_paths) {
  if (!runtime || !goals || !path_buffer || !out_steps || !bombs_mutable) {
    return -1;
  }

  Point current_car = car;
  Point car_start = car;
  Point current_boxes[PLANNER_V3_BFS_MAX_BOXES];
  for (size_t i = 0; i < PLANNER_V3_BFS_MAX_BOXES; ++i) {
    current_boxes[i].row = -1;
    current_boxes[i].col = -1;
  }
  for (size_t gi = 0; gi < goal_count; ++gi) {
    size_t bi = goals[gi].source_idx;
    if (bi >= box_count || bi >= PLANNER_V3_BFS_MAX_BOXES) {
      continue;
    }
    current_boxes[bi] = goals[gi].box;
  }

  Point current_obstacles[200];
  size_t current_obstacle_count = obstacle_count;
  if (current_obstacle_count > 200) {
    current_obstacle_count = 200;
  }
  if (current_obstacle_count > 0 && obstacles) {
    memcpy(current_obstacles, obstacles, current_obstacle_count * sizeof(Point));
  } else {
    current_obstacle_count = 0;
  }

  Point box_paths[PLANNER_V3_BFS_MAX_BOXES][PLANNER_V3_BFS_MAX_CELLS];
  size_t box_path_lens[PLANNER_V3_BFS_MAX_BOXES] = {0};

  for (size_t oi = 0; oi < runtime->ordered_goal_count; ++oi) {
    size_t gi = runtime->ordered_goal_indices[oi];
    if (gi >= goal_count) {
      continue;
    }
    size_t bi = goals[gi].source_idx;
    if (bi >= box_count || bi >= PLANNER_V3_BFS_MAX_BOXES) {
      continue;
    }
    if (current_boxes[bi].row < 0 || current_boxes[bi].col < 0) {
      continue;
    }

    PlannerV3BoxTwoPhasePlan *entry = &runtime->two_phase.box_plan[bi];
    PlannerV3PreBoxPlan *current_plan = &runtime->preplans[bi];
    if (box_path_lens[bi] < PLANNER_V3_BFS_MAX_CELLS) {
      box_paths[bi][box_path_lens[bi]++] = current_boxes[bi];
    }

    if (current_plan->bomb_action_count > 0) {
      int action_ret = planner_v3_bfs_execute_planned_bomb_actions(
          rows, cols,
          &current_car,
          current_obstacles, &current_obstacle_count,
          bombs_mutable, bomb_count,
          current_boxes, box_count,
          current_plan->bomb_actions, current_plan->bomb_action_count,
          path_buffer, path_capacity, out_steps);
      if (action_ret != 1) {
        return action_ret;
      }
    }

    const Point *exec_path = NULL;
    size_t exec_path_len = 0;
    if (entry->special_path_valid && entry->special_path_len > 1) {
      exec_path = entry->special_path;
      exec_path_len = entry->special_path_len;
    }

    int box_exec_ret = planner_v3_bfs_execute_box_path_fixed(
        rows, cols,
        &current_car,
        current_boxes, box_count, bi,
        goals[gi].target,
        current_obstacles, current_obstacle_count,
        bombs_mutable, bomb_count,
        exec_path, exec_path_len,
        path_buffer, path_capacity, out_steps,
        box_paths[bi], &box_path_lens[bi]);
    if (box_exec_ret < 0) {
      return box_exec_ret;
    }

    if (box_path_lens[bi] < PLANNER_V3_BFS_MAX_CELLS) {
      if (box_path_lens[bi] == 0 ||
          box_paths[bi][box_path_lens[bi] - 1].row != goals[gi].target.row ||
          box_paths[bi][box_path_lens[bi] - 1].col != goals[gi].target.col) {
        box_paths[bi][box_path_lens[bi]++] = goals[gi].target;
      }
    }
    current_boxes[bi].row = -1;
    current_boxes[bi].col = -1;
  }

  if (*out_steps > 1 && !planner_v3_bfs_validate_continuous_path(path_buffer, *out_steps)) {
    last_err_stage = 2;
    last_err_detail = LAST_ERR_DETAIL_V3_OVERALL_PATH_NOT_CONTINUOUS;
    return -6;
  }

  if (include_return_to_start_path &&
      (current_car.row != car_start.row || current_car.col != car_start.col)) {
    Point empty_boxes[PLANNER_V3_BFS_MAX_BOXES];
    size_t empty_box_count = 0;
    int dist[PLANNER_V3_BFS_MAX_CELLS];
    if (planner_v3_bfs_global_bfs_from_target(
            rows, cols, car_start,
            current_obstacles, current_obstacle_count,
            bombs_mutable, bomb_count,
            empty_boxes, empty_box_count,
            dist, 1, NULL)) {
      Point return_path[256];
      size_t return_len = 0;
      if (planner_v3_bfs_astar_with_dist(
              rows, cols, current_car, car_start,
              current_obstacles, current_obstacle_count,
              bombs_mutable, bomb_count,
              empty_boxes, empty_box_count, dist,
              return_path, 256, &return_len,
              1,
              0, 0, (Point){0, 0}, 0, NULL, NULL, 0, NULL)) {
        for (size_t i = 1; i < return_len; ++i) {
          if (*out_steps >= path_capacity) {
            return -7;
          }
          if (*out_steps > 0 &&
              !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], return_path[i])) {
            last_err_stage = 2;
            last_err_detail = LAST_ERR_DETAIL_V3_RETURN_PATH_NOT_CONTINUOUS;
            return -6;
          }
          path_buffer[(*out_steps)++] = return_path[i];
        }
      }
    }
  }

  if (out_final_paths) {
    out_final_paths->box_count = box_count;
    for (size_t k = 0; k < box_count && k < PLANNER_V3_BFS_MAX_BOXES; ++k) {
      memset(&out_final_paths->box_paths[k], 0, sizeof(PlannerBoxPathOutput));
      out_final_paths->box_paths[k].planned_order = -1;
    }
    for (size_t bi = 0; bi < box_count && bi < PLANNER_V3_BFS_MAX_BOXES; ++bi) {
      out_final_paths->box_paths[bi] = special_paths.box_paths[bi];
      out_final_paths->box_paths[bi].valid = (box_path_lens[bi] > 0) ? 1 : 0;
      out_final_paths->box_paths[bi].path_len = box_path_lens[bi];
      size_t copy_len = (box_path_lens[bi] < 400) ? box_path_lens[bi] : 400;
      for (size_t j = 0; j < copy_len; ++j) {
        out_final_paths->box_paths[bi].path[j] = box_paths[bi][j];
      }
    }
  }

  return 0;
}

static int planner_v3_bfs_run_assigned_two_phase(
    int rows, int cols, Point car,
    const PlannerBoxGoalBFS *goals,
    size_t goal_count, size_t box_count,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps,
    Point *bombs_mutable,
    int include_return_to_start_path,
    PlannerAllBoxPaths *out_final_paths) {
  if (!goals || !path_buffer || !out_steps || !bombs_mutable) {
    return -1;
  }
  if (goal_count == 0) {
    return -2;
  }
  if (goal_count > PLANNER_V3_BFS_MAX_BOXES) {
    return -3;
  }
  if (path_capacity == 0) {
    return -4;
  }

  *out_steps = 0;
  s_blocked_bomb_count = 0;
  PlannerV3TwoPhaseRuntime runtime;
  memset(&runtime, 0, sizeof(runtime));
  runtime.goal_count = goal_count;
  for (size_t i = 0; i < goal_count && i < PLANNER_V3_BFS_MAX_BOXES; ++i) {
    runtime.goals[i] = goals[i];
  }

  if (!build_push_order(
          rows, cols, car,
          goals, goal_count, box_count,
          obstacles, obstacle_count,
          bombs, bomb_count,
          &runtime)) {
    last_err_stage = 2;
    last_err_detail = LAST_ERR_DETAIL_V3_SOURCE_BOX_NOT_FOUND;
    return -6;
  }

  build_box_reachability(
      rows, cols,
      goals, goal_count, box_count,
      obstacles, obstacle_count,
      bombs, bomb_count,
      car,
      &runtime);

  resolve_bomb_conflicts_and_replan(
      rows, cols,
      goals, goal_count, box_count,
      obstacles, obstacle_count,
      bombs, bomb_count,
      car,
      &runtime);

  return execute_final_plan(
      rows, cols, car,
      goals, goal_count, box_count,
      obstacles, obstacle_count,
      bombs_mutable, bomb_count,
      path_buffer, path_capacity, out_steps,
      include_return_to_start_path,
      &runtime,
      out_final_paths);
}


// ????????? 1????��????????????? -> ????????????��????????��??
// ?????????
//   rows, cols             : ?????????
//   car                    : ��?????????
//   boxes, box_count       : ???????????????????
//   targets, target_count  : ????????????????
//   bombs, bomb_count      : ???????????
//   obstacles, obstacle_count: ????????????????
//   path_buffer            : ??????????????��?????????????? NULL?????????t?????��
//   path_capacity          : path_buffer ??????????��?????????
//   out_steps              : ????��???????
//   out_box_target_indices : ??????????????????????????? NULL??
//   out_final_paths        : ???????????????��????????��?????????? NULL??
// ???????
//   0        : ?��???
//   ???     : ??????? / ??????????? / ????????????????
int plan_boxes_greedy_v3_with_return_control(int rows, int cols, PlannerPointV3_BFS car,
                         const PlannerPointV3_BFS *boxes, size_t box_count,
                         const PlannerPointV3_BFS *targets, size_t target_count,
                         const PlannerPointV3_BFS *bombs, size_t bomb_count,
                         const PlannerPointV3_BFS *obstacles,
                         size_t obstacle_count, PlannerPointV3_BFS *path_buffer,
                         size_t path_capacity, size_t *out_steps,
                          int include_return_to_start_path,
                          size_t *out_box_target_indices,
                          PlannerAllBoxPaths *out_final_paths) {
  last_err_detail_v3_stage3 = LAST_ERR_DETAIL_NONE;
  if (!boxes || !targets || !out_steps) {
    return -1;
  }
  *out_steps = 0;
  last_special_path_fail_reason = SPECIAL_PATH_FAIL_NONE;
  last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_NONE;
  
  // ???????�ɜ�??????��??/????��???????????
  planner_v3_bfs_clear_special_paths(boxes, targets, box_count, target_count);
  if (box_count == 0 || target_count == 0) {
    return -2;
  }
  if (box_count > PLANNER_V3_BFS_MAX_BOXES) {
    return -3;
  }
  PlannerPointV3_BFS local_path_buffer[PLANNER_V3_BFS_MAX_CELLS];
  PlannerPointV3_BFS *effective_path_buffer = path_buffer;
  size_t effective_path_capacity = path_capacity;
  if (!effective_path_buffer) {

    effective_path_buffer = local_path_buffer;
    effective_path_capacity = PLANNER_V3_BFS_MAX_CELLS;
  } else if (effective_path_capacity == 0) {
    return -4;
  }
  if (bomb_count > MAX_BOMBS) {
    return -9;
  }

  if (out_box_target_indices) {
    for (size_t i = 0; i < box_count; ++i) {
      out_box_target_indices[i] = SIZE_MAX;
    }
  }


  Point bombs_mutable[MAX_BOMBS];
  if (bomb_count > 0 && bombs) {
    memcpy(bombs_mutable, bombs, bomb_count * sizeof(Point));
  }

  PlannerBoxGoalBFS assigned[PLANNER_V3_BFS_MAX_BOXES];
  size_t assigned_count = 0;
  int assign_res = build_assigned_goals(
      rows, cols, car,
      boxes, box_count,
      targets, target_count,
      obstacles, obstacle_count,
      bombs, bomb_count,
      NULL,
      assigned, &assigned_count,
      out_box_target_indices);
  if (assign_res != 0) {
    *out_steps = 0;
    return assign_res;
  }

  int res = planner_v3_bfs_run_assigned_two_phase(
      rows, cols, car,
      assigned, assigned_count, box_count,
      obstacles, obstacle_count,
      bombs, bomb_count,
      (Point *)effective_path_buffer,
      effective_path_capacity, out_steps,
      bombs_mutable,
      include_return_to_start_path,
      out_final_paths);
  if (res != 0) {
    return res;
  }


  if (*out_steps > 0 && !planner_v3_bfs_path_in_bounds(rows, cols, (Point *)effective_path_buffer, *out_steps)) {
    *out_steps = 0;
    if (out_final_paths) {
      out_final_paths->box_count = 0;
      for (size_t k = 0; k < PLANNER_V3_BFS_MAX_BOXES; ++k) {
        out_final_paths->box_paths[k].valid = 0;
        out_final_paths->box_paths[k].path_len = 0;
      }
    }
    return -10;  // ???��?????
  }

  if (out_final_paths && out_final_paths->box_count > 0) {
    for (size_t k = 0; k < out_final_paths->box_count && k < PLANNER_V3_BFS_MAX_BOXES; ++k) {
      if (!out_final_paths->box_paths[k].valid || out_final_paths->box_paths[k].path_len == 0) {
        continue;
      }
      size_t plen = out_final_paths->box_paths[k].path_len;
      if (plen > 400) {
        plen = 400;
      }
      if (!planner_v3_bfs_path_in_bounds(rows, cols, out_final_paths->box_paths[k].path, plen)) {
        *out_steps = 0;
        out_final_paths->box_count = 0;
        for (size_t i = 0; i < PLANNER_V3_BFS_MAX_BOXES; ++i) {
          out_final_paths->box_paths[i].valid = 0;
          out_final_paths->box_paths[i].path_len = 0;
        }
        return -10;  // ???��?????
      }
    }
  }

  return 0;
}

int plan_boxes_greedy_v3(int rows, int cols, PlannerPointV3_BFS car,
                         const PlannerPointV3_BFS *boxes, size_t box_count,
                         const PlannerPointV3_BFS *targets, size_t target_count,
                         const PlannerPointV3_BFS *bombs, size_t bomb_count,
                         const PlannerPointV3_BFS *obstacles,
                         size_t obstacle_count, PlannerPointV3_BFS *path_buffer,
                         size_t path_capacity, size_t *out_steps,
                         size_t *out_box_target_indices,
                         PlannerAllBoxPaths *out_final_paths) {
  return plan_boxes_greedy_v3_with_return_control(
      rows, cols, car,
      boxes, box_count,
      targets, target_count,
      bombs, bomb_count,
      obstacles, obstacle_count,
      path_buffer, path_capacity, out_steps,
      1,
      out_box_target_indices,
      out_final_paths);
}




// ????????? 2??????????????? -> ???????????????????????����??
// ????????????????????????????????????��????????????
// ????????????? plan_boxes_greedy_v3 ???
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
                          PlannerAllBoxPaths *out_final_paths) {
  last_err_detail_v3_stage3 = LAST_ERR_DETAIL_NONE;
  if (!boxes || !targets || !out_steps || !box_target_indices) {
    return -1;
  }
  *out_steps = 0;
  last_special_path_fail_reason = SPECIAL_PATH_FAIL_NONE;
  last_first_push_bomb_special_fail_reason = FIRST_PUSH_BOMB_FAIL_NONE;
  planner_v3_bfs_clear_special_paths(boxes, targets, box_count, target_count);
  if (box_count == 0 || target_count == 0) {
    return -2;
  }
  if (box_count > PLANNER_V3_BFS_MAX_BOXES) {
    return -3;
  }
  if (bomb_count > MAX_BOMBS) {
    return -9;
  }
  PlannerPointV3_BFS local_path_buffer[PLANNER_V3_BFS_MAX_CELLS];
  PlannerPointV3_BFS *effective_path_buffer = path_buffer;
  size_t effective_path_capacity = path_capacity;
  if (!effective_path_buffer) {

    effective_path_buffer = local_path_buffer;
    effective_path_capacity = PLANNER_V3_BFS_MAX_CELLS;
  } else if (effective_path_capacity == 0) {
    return -4;
  }
  if (out_box_target_indices) {
    for (size_t i = 0; i < box_count; ++i) {
      out_box_target_indices[i] = SIZE_MAX;
    }
  }
  (void)box_req_masks;
  Point bombs_mutable[MAX_BOMBS];
  if (bomb_count > 0 && bombs) {
    memcpy(bombs_mutable, bombs, bomb_count * sizeof(Point));
  }
  PlannerBoxGoalBFS assigned[PLANNER_V3_BFS_MAX_BOXES];
  size_t assigned_count = 0;
  int build_res = build_assigned_goals(
      rows, cols, car,
      boxes, box_count,
      targets, target_count,
      obstacles, obstacle_count,
      bombs, bomb_count,
      box_target_indices,
      assigned, &assigned_count,
      out_box_target_indices);
  if (build_res != 0) {
    *out_steps = 0;
    return build_res;
  }
  int res = planner_v3_bfs_run_assigned_two_phase(
      rows, cols, car,
      assigned, assigned_count, box_count,
      obstacles, obstacle_count,
      bombs, bomb_count,
      (Point *)effective_path_buffer,
      effective_path_capacity, out_steps,
      bombs_mutable,
      include_return_to_start_path,
      out_final_paths);
  if (res != 0) {
    return res;
  }
  if (*out_steps > 0 && !planner_v3_bfs_path_in_bounds(rows, cols, (Point *)effective_path_buffer, *out_steps)) {
    *out_steps = 0;
    if (out_final_paths) {
      out_final_paths->box_count = 0;
      for (size_t k = 0; k < PLANNER_V3_BFS_MAX_BOXES; ++k) {
        out_final_paths->box_paths[k].valid = 0;
        out_final_paths->box_paths[k].path_len = 0;
      }
    }
    return -10;
  }
  if (out_final_paths && out_final_paths->box_count > 0) {
    for (size_t k = 0; k < out_final_paths->box_count && k < PLANNER_V3_BFS_MAX_BOXES; ++k) {
      if (!out_final_paths->box_paths[k].valid || out_final_paths->box_paths[k].path_len == 0) {
        continue;
      }
      size_t plen = out_final_paths->box_paths[k].path_len;
      if (plen > 400) {
        plen = 400;
      }
      if (!planner_v3_bfs_path_in_bounds(rows, cols, out_final_paths->box_paths[k].path, plen)) {
        *out_steps = 0;
        out_final_paths->box_count = 0;
        for (size_t i = 0; i < PLANNER_V3_BFS_MAX_BOXES; ++i) {
          out_final_paths->box_paths[i].valid = 0;
          out_final_paths->box_paths[i].path_len = 0;
        }
        return -10;
      }
    }
  }
  return 0;
}

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
                         PlannerAllBoxPaths *out_final_paths) {
  return plan_boxes_greedy_v3_manual_assignment_with_return_control(
      rows, cols, car,
      boxes, box_count,
      targets, target_count,
      bombs, bomb_count,
      obstacles, obstacle_count,
      box_target_indices,
      box_req_masks,
      path_buffer, path_capacity, out_steps,
      1,
      out_box_target_indices,
      out_final_paths);
}

