#include "assigned_box_planner_greedy_3.h"
#include "assigned_box_planner_greedy_2.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

extern int last_err_stage;   // 错误阶段
extern int last_err_detail;  // 错误详情

int planner_v3_push_only_bfs_astar_path = 0;  /* 0=路径与评分取优，1=仅用BFS+A*路径 */

#define PLANNER_V3_BFS_MAX_BOXES 10
#define PLANNER_V3_BFS_MAX_CELLS 400
#define PLANNER_V3_BFS_MAX_GREEDY_STEPS 5000
#define PLANNER_V3_BFS_MAX_PATH_LEN 5000
#define PLANNER_V3_BFS_STRATEGY_PATH 1
#define PLANNER_V3_BFS_STRATEGY_SCORE 2
#define MAX_BOMBS 5
#define PUSH_BOMB_SAVE_PATH_MAX 4096

// 特殊路径（不把炸弹当作障碍、允许经过多个障碍，需可被一弹炸掉且炸弹可达）的显示数据
// 用于在菜单中单独查看“特殊路径”，并保持每次规划后的最新结果
PlannerAllBoxPaths special_paths = {0};

// 特殊路径经过的多个障碍（用于推炸弹时排除，炸弹可达性校验）
static Point s_special_path_obstacles[200];
static size_t s_special_path_obstacle_count = 0;

// 清空 special_paths：除“标志位/计数”外也清“内容字段”；已完成的箱子不清理；box_count 与 boxes 一致
static void planner_v3_bfs_clear_special_paths(const PlannerPointV3_BFS *boxes,
    const PlannerPointV3_BFS *targets, size_t box_count, size_t target_count) {
  s_special_path_obstacle_count = 0;
  special_paths.box_count = box_count;  /* 与 PlannerPointV3_BFS boxes 数量一致，显示时索引对应 */
  for (size_t i = 0; i < box_count && i < PLANNER_V3_BFS_MAX_BOXES; ++i) {
    int completed = 0;
    if (boxes && targets && target_count > 0) {
      for (size_t t = 0; t < target_count; ++t) {
        if (boxes[i].row == targets[t].row && boxes[i].col == targets[t].col) {
          completed = 1;
          break;
        }
      }
    }
    if (completed)
      continue;  /* 已完成的箱子不清理 */
    memset(&special_paths.box_paths[i], 0, sizeof(PlannerBoxPathOutput));
  }
}

typedef struct {
  Point box;
  Point target;
  size_t target_idx;
  size_t source_idx;
  Point obstacle_to_clear;  // 需要炸掉的障碍（如果路径经过障碍）
  int has_obstacle_to_clear;  // 是否有需要炸掉的障碍
} PlannerBoxGoalBFS;

// 前向声明
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

// 工具函数
static int planner_v3_bfs_abs(int v) { return v >= 0 ? v : -v; }

static int planner_v3_bfs_in_bounds(int rows, int cols, int row, int col) {
  return row >= 0 && row < rows && col >= 0 && col < cols;
}

// 检查是否是障碍（包括炸弹）
static int planner_v3_bfs_is_obstacle(const Point *obstacles, size_t obstacle_count,
                                      const Point *bombs, size_t bomb_count,
                                      int row, int col) {
  // 检查普通障碍
  for (size_t i = 0; i < obstacle_count; ++i) {
    if (obstacles[i].row == row && obstacles[i].col == col) {
      return 1;
    }
  }
  // 检查炸弹（炸弹也视为障碍）
  for (size_t i = 0; i < bomb_count; ++i) {
    if (bombs[i].row == row && bombs[i].col == col) {
      return 1;
    }
  }
  return 0;
}

// 检查是否是障碍（不包括炸弹，用于特殊路径规划）
static int planner_v3_bfs_is_obstacle_no_bomb(const Point *obstacles, size_t obstacle_count,
                                               int row, int col) {
  for (size_t i = 0; i < obstacle_count; ++i) {
    if (obstacles[i].row == row && obstacles[i].col == col) {
      return 1;
    }
  }
  return 0;
}

static int planner_v3_bfs_is_box_at(const Point *boxes, size_t count, int row, int col,
                     size_t skip_idx) {
  for (size_t i = 0; i < count; ++i) {
    if (i == skip_idx) {
      continue;
    }
    // 忽略已到达目标的箱子（标记为负坐标，表示"消失"）
    if (boxes[i].row < 0 || boxes[i].col < 0) {
      continue;
    }
    if (boxes[i].row == row && boxes[i].col == col) {
      return 1;
    }
  }
  return 0;
}

// 计算某个位置周围4个方向的障碍物/箱子数量
static int planner_v3_bfs_adjacent_blockers(int rows, int cols, const Point *obstacles,
                                   size_t obstacle_count, const Point *bombs, size_t bomb_count,
                                   const Point *boxes,
                                   size_t box_count, size_t skip_idx, int row,
                                   int col) {
  int count = 0;
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (int i = 0; i < 4; ++i) {
    int nr = row + dirs[i][0];
    int nc = col + dirs[i][1];
    if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc)) {
      count++;
      continue;
    }
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, nr, nc) ||
        planner_v3_bfs_is_box_at(boxes, box_count, nr, nc, skip_idx)) {
      count++;
    }
  }
  return count;
}

/* 前向声明：供 A* 内推位检测使用（定义在文件后部） */
static int planner_v3_bfs_distance_between(int rows, int cols, Point start, Point target,
                                           const Point *obstacles, size_t obstacle_count,
                                           const Point *bombs, size_t bomb_count,
                                           const Point *boxes, size_t box_count,
                                           int include_bombs);
static int planner_v3_bfs_is_push_pos_reachable(int rows, int cols, Point car, Point push_pos,
                                                 const Point *obstacles, size_t obstacle_count,
                                                 const Point *bombs, size_t bomb_count,
                                                 const Point *boxes, size_t box_count,
                                                 int include_bombs);
static Point planner_v3_bfs_check_bomb_path_push_obstacle(int rows, int cols,
                                                          Point bomb_start, Point target_pos,
                                                          const Point *obstacles, size_t obstacle_count,
                                                          const Point *bombs, size_t bomb_count, size_t bomb_idx,
                                                          const Point *boxes, size_t box_count);
static int planner_v3_bfs_point_on_path(Point pt, const Point *path, size_t path_len);

// 判断某个位置是否是死点
static int planner_v3_bfs_is_deadlock(int rows, int cols, const Point *obstacles,
                                     size_t obstacle_count, const Point *bombs, size_t bomb_count,
                                     const Point *boxes,
                                     size_t box_count, size_t skip_idx, int row,
                                     int col) {
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};  // 上、下、左、右
  int blocked[4] = {0, 0, 0, 0};  // 记录每个方向是否被阻挡
  
  // 检查每个方向是否被阻挡
  for (int i = 0; i < 4; ++i) {
    int nr = row + dirs[i][0];
    int nc = col + dirs[i][1];
    if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc)) {
      blocked[i] = 1;
      continue;
    }
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, nr, nc) ||
        planner_v3_bfs_is_box_at(boxes, box_count, nr, nc, skip_idx)) {
      blocked[i] = 1;
    }
  }
  
  // 统计被阻挡的方向数量
  int blocked_count = blocked[0] + blocked[1] + blocked[2] + blocked[3];
  
  // 如果被阻挡的方向少于2个，不是死点
  if (blocked_count < 2) {
    return 0;
  }
  
  // 如果被阻挡的方向有3个或4个，是死点
  if (blocked_count >= 3) {
    return 1;
  }
  
  // 如果只有两个被阻挡的方向
  if (blocked_count == 2) {
    // 检查是否是"上、下"组合（dirs[0]和dirs[1]）
    if (blocked[0] && blocked[1]) {
      return 0;  // 不是死点
    }
    // 检查是否是"左、右"组合（dirs[2]和dirs[3]）
    if (blocked[2] && blocked[3]) {
      return 0;  // 不是死点
    }
    // 其他两个方向的组合（如"上、左"、"上、右"、"下、左"、"下、右"），是死点
    return 1;
  }
  
  return 0;
}

// 检查箱子从start是否能被推到target（不考虑死点，只检查物理可达性）
// include_bombs: 1=把炸弹当作障碍，0=不把炸弹当作障碍
static int planner_v3_bfs_can_reach_goal(int rows, int cols, const Point *obstacles,
                                size_t obstacle_count, const Point *bombs, size_t bomb_count,
                                const Point *boxes,
                                size_t box_count, size_t moving_idx,
                                Point start, Point target, int include_bombs) {
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_BFS_MAX_CELLS || total_cells <= 0) {
    return 0;
  }
  if (!planner_v3_bfs_in_bounds(rows, cols, start.row, start.col) ||
      !planner_v3_bfs_in_bounds(rows, cols, target.row, target.col)) {
    return 0;
  }
  
  // 检查起点和终点
  if (include_bombs) {
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, start.row, start.col) ||
        planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, target.row, target.col)) {
      return 0;
    }
  } else {
    if (planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, start.row, start.col) ||
        planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, target.row, target.col)) {
      return 0;
    }
  }
  
  if (planner_v3_bfs_is_box_at(boxes, box_count, start.row, start.col, SIZE_MAX) ||
      planner_v3_bfs_is_box_at(boxes, box_count, target.row, target.col, SIZE_MAX)) {
    return 0;
  }

  if (start.row == target.row && start.col == target.col) {
    return 1;
  }

  uint8_t visited[PLANNER_V3_BFS_MAX_CELLS];
  int queue[PLANNER_V3_BFS_MAX_CELLS];
  int head = 0;
  int tail = 0;

  memset(visited, 0, sizeof(visited));

  int start_idx = start.row * cols + start.col;
  queue[tail++] = start_idx;
  visited[start_idx] = 1;

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
      int new_row = row + dirs[i][0];
      int new_col = col + dirs[i][1];
      int push_row = row - dirs[i][0];
      int push_col = col - dirs[i][1];

      if (!planner_v3_bfs_in_bounds(rows, cols, new_row, new_col) ||
          !planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
        continue;
      }

      // 检查障碍（根据include_bombs决定是否包含炸弹）
      if (include_bombs) {
        if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, new_row, new_col) ||
            planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, push_row, push_col)) {
          continue;
        }
      } else {
        if (planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, new_row, new_col) ||
            planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, push_row, push_col)) {
          continue;
        }
      }

      if (planner_v3_bfs_is_box_at(boxes, box_count, new_row, new_col, moving_idx) ||
          planner_v3_bfs_is_box_at(boxes, box_count, push_row, push_col, moving_idx)) {
        continue;
      }

      int idx = new_row * cols + new_col;
      if (!visited[idx]) {
        visited[idx] = 1;
        queue[tail++] = idx;
      }
    }
  }

  return 0;
}

// 全局BFS：从终点开始，计算每个可达格子到终点的真实距离
// include_bombs: 1=把炸弹当作障碍，0=不把炸弹当作障碍
static int planner_v3_bfs_global_bfs_from_target(int rows, int cols, Point target,
                                             const Point *obstacles, size_t obstacle_count,
                                             const Point *bombs, size_t bomb_count,
                                             const Point *boxes, size_t box_count,
                                             int dist[PLANNER_V3_BFS_MAX_CELLS], int include_bombs) {
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_BFS_MAX_CELLS || total_cells <= 0) {
    return 0;
  }
  
  // 初始化距离为无穷大
  for (int i = 0; i < total_cells; ++i) {
    dist[i] = INT_MAX;
  }
  
  uint8_t visited[PLANNER_V3_BFS_MAX_CELLS];
  int queue[PLANNER_V3_BFS_MAX_CELLS];
  memset(visited, 0, sizeof(visited));
  
  int head = 0;
  int tail = 0;
  
  // 从终点开始BFS
  int target_idx = target.row * cols + target.col;
  queue[tail++] = target_idx;
  visited[target_idx] = 1;
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
      
      // 检查障碍物和箱子
      if (include_bombs) {
        if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, nr, nc)) {
          continue;
        }
      } else {
        if (planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, nr, nc)) {
          continue;
        }
      }
      if (planner_v3_bfs_is_box_at(boxes, box_count, nr, nc, box_count)) {
        continue;
      }
      
      int next_idx = nr * cols + nc;
      if (next_idx < 0 || next_idx >= total_cells) {
        continue;
      }
      
      if (!visited[next_idx]) {
        visited[next_idx] = 1;
        dist[next_idx] = curr_dist + 1;
        if (tail < PLANNER_V3_BFS_MAX_CELLS) {
          queue[tail++] = next_idx;
        }
      }
    }
  }
  
  return 1;
}

// A*搜索：使用预计算的距离作为启发函数
// include_bombs: 1=把炸弹当作障碍，0=不把炸弹当作障碍
// check_push: 1=在扩展时检测推位，若节点A到节点B的推位不合理则排除节点B
//   推位不合理包括：推位为障碍、箱子、炸弹或越界；箱子路径与炸弹路径均需车能到达推位格本身（非四邻即可）
// push_mode: 仅当check_push=1时有效；0=箱子路径（推位需可站立且车可达推位格），1=炸弹路径（推位需可站立且车可达推位格）
// car_start: 仅当check_push=1时有效；开始推时车的位置
static int planner_v3_bfs_astar_with_dist(int rows, int cols, Point start, Point target,
                                      const Point *obstacles, size_t obstacle_count,
                                      const Point *bombs, size_t bomb_count,
                                      const Point *boxes, size_t box_count,
                                      const int dist[PLANNER_V3_BFS_MAX_CELLS],
                                      Point *path, size_t path_cap, size_t *path_len,
                                      int include_bombs, int allow_one_obstacle, Point *obstacle_passed,
                                      int check_push, int push_mode, Point car_start) {
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_BFS_MAX_CELLS || total_cells <= 0) {
    return 0;
  }
  
  if (start.row == target.row && start.col == target.col) {
    if (path_cap < 1) return 0;
    path[0] = start;
    *path_len = 1;
    return 1;
  }
  
  // A*数据结构
  typedef struct {
    int idx;
    int f_score;  // f = g + h
    int car_to_push_dist;  // 车到推位的BFS距离；check_push=0时用dist[idx]，用于tiebreaker
  } AStarNode;
  
  int g_score[PLANNER_V3_BFS_MAX_CELLS];
  int parent[PLANNER_V3_BFS_MAX_CELLS];
  uint8_t in_open[PLANNER_V3_BFS_MAX_CELLS];
  uint8_t in_closed[PLANNER_V3_BFS_MAX_CELLS];
  
  for (int i = 0; i < total_cells; ++i) {
    g_score[i] = INT_MAX;
    parent[i] = -1;
    in_open[i] = 0;
    in_closed[i] = 0;
  }
  
  // 简单的优先队列（数组实现）
  AStarNode open_set[PLANNER_V3_BFS_MAX_CELLS];
  int open_count = 0;
  
  int start_idx = start.row * cols + start.col;
  int target_idx = target.row * cols + target.col;
  
  // 检查起点到终点的距离是否有效
  if (dist[start_idx] == INT_MAX) {
    return 0;  // 起点无法到达终点
  }
  
  g_score[start_idx] = 0;
  open_set[open_count].idx = start_idx;
  open_set[open_count].f_score = dist[start_idx];  // f = g + h = 0 + h
  open_set[open_count].car_to_push_dist = check_push ? 0 : dist[start_idx];  // 起点无推位，用0或dist
  open_count++;
  in_open[start_idx] = 1;
  
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  int obstacle_used = 0;  // 是否已经使用了一次经过障碍的机会
  Point obstacle_used_pos = {-1, -1};  // 使用的障碍位置
  int last_expansion_dir = -1;  // 上一次扩展方向（父->当前）；check_push=0 时用于 tiebreaker

  while (open_count > 0) {
    // 找到 f_score 最小的节点；f_score 相同时：check_push=1 用 car_to_push_dist，check_push=0 用“与上一扩展方向一致”再 car_to_push_dist
    int best_idx = 0;
    for (int i = 1; i < open_count; ++i) {
      if (open_set[i].f_score < open_set[best_idx].f_score) {
        best_idx = i;
      } else if (open_set[i].f_score == open_set[best_idx].f_score) {
        int prefer_i = 0;
        if (check_push) {
          prefer_i = (open_set[i].car_to_push_dist < open_set[best_idx].car_to_push_dist);
        } else {
          int ri = open_set[i].idx / cols, ci = open_set[i].idx % cols;
          int rbest = open_set[best_idx].idx / cols, cbest = open_set[best_idx].idx % cols;
          int pi = parent[open_set[i].idx], pbest = parent[open_set[best_idx].idx];
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
          else
            prefer_i = (open_set[i].car_to_push_dist < open_set[best_idx].car_to_push_dist);
        }
        if (prefer_i)
          best_idx = i;
      }
    }

    AStarNode current = open_set[best_idx];
    int curr_idx = current.idx;

    // 从 open_set 中移除
    for (int i = best_idx; i < open_count - 1; ++i) {
      open_set[i] = open_set[i + 1];
    }
    open_count--;
    in_open[curr_idx] = 0;
    in_closed[curr_idx] = 1;

    // 记录本步扩展方向，供下次 tiebreaker 使用（check_push=0 时）
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
    
    // 到达目标
    if (curr_idx == target_idx) {
      // 回溯路径
      int path_indices[PLANNER_V3_BFS_MAX_CELLS];
      int path_count = 0;
      int idx = target_idx;
      
      while (idx != -1 && path_count < PLANNER_V3_BFS_MAX_CELLS) {
        path_indices[path_count++] = idx;
        idx = parent[idx];
      }
      
      // 反转路径
      *path_len = 0;
      for (int i = path_count - 1; i >= 0 && *path_len < path_cap; --i) {
        int pidx = path_indices[i];
        path[*path_len].row = pidx / cols;
        path[*path_len].col = pidx % cols;
        (*path_len)++;
      }
      
      // 如果使用了障碍，记录障碍位置
      if (allow_one_obstacle && obstacle_used && obstacle_passed) {
        *obstacle_passed = obstacle_used_pos;
      }
      
      return (*path_len > 0) ? 1 : 0;
    }
    
    int curr_row = curr_idx / cols;
    int curr_col = curr_idx % cols;
    
    // 扩展邻居
    for (int d = 0; d < 4; ++d) {
      int nr = curr_row + dirs[d][0];
      int nc = curr_col + dirs[d][1];
      
      if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc)) {
        continue;
      }
      
      // 检查是否是障碍
      int is_obs = 0;
      if (include_bombs) {
        is_obs = planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, nr, nc);
      } else {
        is_obs = planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, nr, nc);
      }
      
      // 如果允许经过一次障碍，且还没有使用过，且当前是障碍
      if (allow_one_obstacle && !obstacle_used && is_obs) {
        // 可以使用这个障碍
        obstacle_used = 1;
        obstacle_used_pos.row = nr;
        obstacle_used_pos.col = nc;
      } else if (is_obs) {
        // 不允许经过障碍，跳过
        continue;
      }
      
      if (planner_v3_bfs_is_box_at(boxes, box_count, nr, nc, box_count)) {
        continue;
      }
      
      int next_idx = nr * cols + nc;
      if (next_idx < 0 || next_idx >= total_cells) {
        continue;
      }
      
      int car_to_push_dist = dist[next_idx];  /* check_push=0时用目标距离作为tiebreaker */
      
      /* 推位检测：若从节点A(curr)到节点B(next)的推位不合理，则排除节点B；推位 = from - (to-from)，dr/dc = to-from */
      if (check_push) {
        int dr = nr - curr_row;
        int dc = nc - curr_col;
        int push_row = curr_row - dr;
        int push_col = curr_col - dc;
        if (!planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
          continue;
        }
        /* 推位不能是障碍、箱子、炸弹，否则排除节点B（适用于箱子路径与炸弹路径） */
        if (include_bombs) {
          if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, push_row, push_col)) {
            continue;
          }
        } else {
          if (planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, push_row, push_col)) {
            continue;
          }
          {
            int is_bomb = 0;
            for (size_t bi = 0; bi < bomb_count && !is_bomb; ++bi) {
              if (bombs[bi].row == push_row && bombs[bi].col == push_col) {
                is_bomb = 1;
              }
            }
            if (is_bomb) {
              continue;
            }
          }
        }
        if (planner_v3_bfs_is_box_at(boxes, box_count, push_row, push_col, SIZE_MAX)) {
          continue;
        }
        Point car_pt;
        if (parent[curr_idx] == -1) {
          car_pt = car_start;
        } else {
          int pr = parent[curr_idx] / cols;
          int pc = parent[curr_idx] % cols;
          car_pt.row = pr;
          car_pt.col = pc;
        }
        if (push_mode == 0) {
          /* 箱子路径：车必须能到达推位格本身，当前节点为被推箱位置 */
          Point boxes_with_curr[PLANNER_V3_BFS_MAX_BOXES];
          size_t bc = 0;
          for (size_t i = 0; i < box_count && bc < PLANNER_V3_BFS_MAX_BOXES; ++i) {
            boxes_with_curr[bc++] = boxes[i];
          }
          if (bc < PLANNER_V3_BFS_MAX_BOXES) {
            boxes_with_curr[bc++] = (Point){curr_row, curr_col};
          }
          car_to_push_dist = planner_v3_bfs_distance_between(rows, cols, car_pt, (Point){push_row, push_col},
                obstacles, obstacle_count, bombs, bomb_count,
                boxes_with_curr, bc, include_bombs);
          if (car_to_push_dist == INT_MAX) {
            continue;
          }
        } else {
          /* 炸弹路径：推位需可站立（已在上面排除障碍/箱子/炸弹），车需能到达推位格，当前炸弹格视为障碍 */
          {
            Point bombs_with_curr[MAX_BOMBS + 1];
            size_t tb = 0;
            for (size_t k = 0; k < bomb_count && tb < MAX_BOMBS + 1; ++k) {
              bombs_with_curr[tb++] = bombs[k];
            }
            if (tb < MAX_BOMBS + 1) {
              bombs_with_curr[tb++] = (Point){curr_row, curr_col};
            }
            car_to_push_dist = planner_v3_bfs_distance_between(rows, cols, car_pt, (Point){push_row, push_col},
                  obstacles, obstacle_count, bombs_with_curr, tb, boxes, box_count, 1);
            if (car_to_push_dist == INT_MAX) {
              continue;
            }
          }
        }
      }
      
      if (in_closed[next_idx]) {
        continue;
      }
      
      int tentative_g = g_score[curr_idx] + 1;
      
      if (!in_open[next_idx]) {
        // 新节点
        g_score[next_idx] = tentative_g;
        parent[next_idx] = curr_idx;
        
        // h = dist[next_idx]（预计算的真实距离）
        int h = dist[next_idx];
        if (h == INT_MAX) {
          continue;  // 该点无法到达终点，跳过
        }
        
        int f = tentative_g + h;
        
        if (open_count < PLANNER_V3_BFS_MAX_CELLS) {
          open_set[open_count].idx = next_idx;
          open_set[open_count].f_score = f;
          open_set[open_count].car_to_push_dist = car_to_push_dist;
          open_count++;
          in_open[next_idx] = 1;
        }
      } else if (tentative_g < g_score[next_idx]) {
        // 找到更好的路径
        g_score[next_idx] = tentative_g;
        parent[next_idx] = curr_idx;
        
        int h = dist[next_idx];
        int f = tentative_g + h;
        
        // 更新open_set中的f_score和car_to_push_dist
        for (int i = 0; i < open_count; ++i) {
          if (open_set[i].idx == next_idx) {
            open_set[i].f_score = f;
            open_set[i].car_to_push_dist = car_to_push_dist;
            break;
          }
        }
      }
    }
  }
  
  // 没有找到路径
  *path_len = 0;
  return 0;
}

// 计算两点之间的BFS真实距离（不可达返回INT_MAX）
static int planner_v3_bfs_distance_between(int rows, int cols, Point start, Point target,
                                           const Point *obstacles, size_t obstacle_count,
                                           const Point *bombs, size_t bomb_count,
                                           const Point *boxes, size_t box_count,
                                           int include_bombs) {
  if (!planner_v3_bfs_in_bounds(rows, cols, start.row, start.col) ||
      !planner_v3_bfs_in_bounds(rows, cols, target.row, target.col)) {
    return INT_MAX;
  }
  int dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                             bombs, bomb_count, boxes, box_count, dist, include_bombs)) {
    return INT_MAX;
  }
  int idx = start.row * cols + start.col;
  if (idx < 0 || idx >= rows * cols) {
    return INT_MAX;
  }
  return dist[idx];
}

// 验证新添加的点与前一个点是否相邻
static int planner_v3_bfs_check_adjacent(Point prev, Point curr) {
  int dr = planner_v3_bfs_abs(curr.row - prev.row);
  int dc = planner_v3_bfs_abs(curr.col - prev.col);
  return (dr + dc == 1);
}

// 使用全局BFS+A*移动车到目标位置
// include_bombs: 1=把炸弹当作障碍，0=不把炸弹当作障碍
static int planner_v3_bfs_car_move_with_global_astar(int rows, int cols, Point *car_pos, 
                                                  Point target,
                                                  const Point *obstacles, size_t obstacle_count,
                                                  const Point *bombs, size_t bomb_count,
                                                  const Point *boxes, size_t box_count,
                                                  Point *path_buffer, size_t path_capacity,
                                                  size_t *out_steps, int include_bombs) {
  if (car_pos->row == target.row && car_pos->col == target.col) {
    return 1;  // 已在目标位置
  }
  
  // 第1步：从目标做全局BFS
  int dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                         bombs, bomb_count, boxes, box_count, dist, include_bombs)) {
    return 0;  // BFS失败
  }
  
  // 第2步：使用A*搜索路径
  Point temp_path[256];
  size_t temp_len = 0;
  
  if (!planner_v3_bfs_astar_with_dist(rows, cols, *car_pos, target,
                                  obstacles, obstacle_count,
                                  bombs, bomb_count,
                                  boxes, box_count, dist,
                                  temp_path, 256, &temp_len, include_bombs, 0, NULL,
                                  0, 0, (Point){0, 0})) {
    return 0;  // A*失败
  }
  
  if (temp_len == 0) {
    return 0;
  }
  
  // 第3步：将路径添加到输出缓冲区（跳过起点）
  for (size_t i = 1; i < temp_len; ++i) {
    if (*out_steps >= path_capacity) {
      return -7;  // 路径缓存不足
    }
    
    // 验证与前一个点相邻
    if (*out_steps > 0 && 
        !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], temp_path[i])) {
      return 0;  // 路径不连续
    }
    
    path_buffer[(*out_steps)++] = temp_path[i];
  }
  
  // 更新车位置
  if (temp_len > 0) {
    *car_pos = temp_path[temp_len - 1];
  }
  
  return 1;
}

// 仅用于仿真估算：移动车到目标位置，使用BFS距离累加步数
static int planner_v3_bfs_simulate_car_move_steps(int rows, int cols, Point *car_pos, Point target,
                                                 const Point *obstacles, size_t obstacle_count,
                                                 const Point *bombs, size_t bomb_count,
                                                 const Point *boxes, size_t box_count,
                                                 size_t *out_steps, int include_bombs) {
  if (car_pos->row == target.row && car_pos->col == target.col) {
    return 1;
  }

  int dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                             bombs, bomb_count, boxes, box_count, dist, include_bombs)) {
    return 0;
  }

  int car_idx = car_pos->row * cols + car_pos->col;
  if (car_idx < 0 || car_idx >= rows * cols || dist[car_idx] == INT_MAX) {
    return 0;
  }

  *out_steps += (size_t)dist[car_idx];
  *car_pos = target;
  return 1;
}

// 验证路径连续性（每步曼哈顿距离为1）
static int planner_v3_bfs_validate_continuous_path(const Point *path, size_t len) {
  if (len < 2) {
    return 1;  // 0或1步总是有效
  }
  for (size_t i = 1; i < len; ++i) {
    int dr = planner_v3_bfs_abs(path[i].row - path[i-1].row);
    int dc = planner_v3_bfs_abs(path[i].col - path[i-1].col);
    if (dr + dc != 1) {
      return 0;  // 跳跃或对角线移动
    }
  }
  return 1;
}

// 验证从边界位置到目标点的路径，并检查每一步的推位是否合理
// 返回值：1=路径有效且推位合理，0=路径无效或推位不合理
// include_bombs: 1=把炸弹当作障碍，0=不把炸弹当作障碍
static int planner_v3_bfs_validate_boundary_path(int rows, int cols, Point boundary_pos, Point target,
                                                 Point car_pos,
                                                 const Point *obstacles, size_t obstacle_count,
                                                 const Point *bombs, size_t bomb_count,
                                                 const Point *boxes, size_t box_count,
                                                 size_t moving_idx, int include_bombs) {
  // 如果边界位置就是目标点，直接返回成功
  if (boundary_pos.row == target.row && boundary_pos.col == target.col) {
    return 1;
  }
  
  // 检查从边界位置到目标点是否可达
  if (!planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count, bombs, bomb_count,
                                    boxes, box_count, moving_idx, boundary_pos, target, include_bombs)) {
    return 0;  // 不可达目标点
  }
  
  // 计算从边界位置到目标点的路径
  Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
  size_t temp_count = 0;
  for (size_t i = 0; i < box_count; ++i) {
    if (i == moving_idx) {
      continue;
    }
    if (boxes[i].row < 0 || boxes[i].col < 0) {
      continue;
    }
    temp_boxes[temp_count++] = boxes[i];
  }
  
  // 从目标点做全局BFS
  int dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                             bombs, bomb_count, temp_boxes, temp_count, dist, include_bombs)) {
    return 0;
  }
  
  // 使用A*计算路径
  Point path[PLANNER_V3_BFS_MAX_PATH_LEN];
  size_t path_len = 0;
  /* A* 内已做推位检测：若边界到目标每一步推位不合理则不会扩展该节点 */
  if (!planner_v3_bfs_astar_with_dist(rows, cols, boundary_pos, target,
                                      obstacles, obstacle_count,
                                      bombs, bomb_count,
                                      temp_boxes, temp_count, dist,
                                      path, PLANNER_V3_BFS_MAX_PATH_LEN, &path_len,
                                      include_bombs, 0, NULL,
                                      1, 0, car_pos)) {
    return 0;
  }
  return 1;  /* 路径有效且推位已在 A* 内校验 */
}

// 检验给定箱子路径上每一步的推位是否合理（车能否到达推位）
// 检测可达性时考虑：除推动的箱子外的其他箱子、所有炸弹和障碍；每步使用当前步的箱子状态（被推箱在 current_pos）
// 要求：车必须能到达推位格本身，而非推位四邻之一即可
// box_path: 箱子从起点到目标的路径；path_len: 路径长度；car_pos: 开始推该箱时车的位置
// out_fail_step: 可选，失败时输出不可达的步索引(1..path_len-1)，对应路径点 box_path[*out_fail_step]
// 返回值：1=所有推位合理，0=存在推位不可达
static int planner_v3_bfs_validate_box_path_push_positions(int rows, int cols,
                                                           const Point *box_path, size_t path_len,
                                                           Point car_pos,
                                                           const Point *obstacles, size_t obstacle_count,
                                                           const Point *bombs, size_t bomb_count,
                                                           const Point *boxes, size_t box_count,
                                                           size_t moving_idx, int include_bombs,
                                                           size_t *out_fail_step) {
  if (path_len < 2) return 1;
  Point current_pos = box_path[0];
  Point current_car = car_pos;
  for (size_t i = 1; i < path_len; ++i) {
    Point next_pos = box_path[i];
    int dr = next_pos.row - current_pos.row;
    int dc = next_pos.col - current_pos.col;
    /* 推位 = from - (to-from)，from=current_pos, to=next_pos */
    int push_row = current_pos.row - dr;
    int push_col = current_pos.col - dc;
    if (!planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
      if (out_fail_step) *out_fail_step = i;
      return 0;
    }
    // 当前步的箱子状态：其他箱子保持原位，被推箱子在 current_pos（用于严格可达性检测）
    Point current_boxes_state[PLANNER_V3_BFS_MAX_BOXES];
    size_t bc = box_count <= PLANNER_V3_BFS_MAX_BOXES ? box_count : PLANNER_V3_BFS_MAX_BOXES;
    for (size_t k = 0; k < bc; ++k) {
      current_boxes_state[k] = (k == moving_idx) ? current_pos : boxes[k];
    }
    // 严格检查推位可达性：车必须能到达推位格本身（非四邻即可）
    int car_dist = planner_v3_bfs_distance_between(rows, cols, current_car, (Point){push_row, push_col},
                                                    obstacles, obstacle_count, bombs, bomb_count,
                                                    current_boxes_state, bc, include_bombs);
    if (car_dist == INT_MAX) {
      if (out_fail_step) *out_fail_step = i;
      return 0;
    }
    current_car = current_pos;
    current_pos = next_pos;
  }
  return 1;
}

// 检验给定炸弹路径上每一步的推位是否合理（车能否到达推位）
// 对每一步推位进行严格可达性检测：车必须能到达推位格 (push_row,push_col) 本身，
// 且计算路径时当前炸弹所在格视为障碍，推位格必须可站立（非障碍/非炸弹/非箱子）。
// bomb_path: 炸弹从起点到目标的路径；path_len: 路径长度；car_pos: 开始推该炸弹时车的位置
// bombs_excluding_current: 除当前炸弹外的其他炸弹（用于障碍判断）
// 返回值：1=所有推位合理，0=存在推位不可达
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
    int dr = next_pos.row - current_pos.row;
    int dc = next_pos.col - current_pos.col;
    /* 推位 = from - (to-from)，from=current_pos, to=next_pos */
    int push_row = current_pos.row - dr;
    int push_col = current_pos.col - dc;
    if (!planner_v3_bfs_in_bounds(rows, cols, push_row, push_col))
      return 0;
    /* 推位格必须可站立：非障碍、非（其他）炸弹、非箱子 */
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs_excluding_current, bomb_count_excl, push_row, push_col))
      return 0;
    if (planner_v3_bfs_is_box_at(boxes, box_count, push_row, push_col, SIZE_MAX))
      return 0;
    /* 严格可达性：车必须能到达推位格本身；路径计算时当前炸弹所在格视为障碍 */
    Point temp_bombs[MAX_BOMBS];
    size_t tb_count = 0;
    for (size_t k = 0; k < bomb_count_excl && tb_count < MAX_BOMBS; ++k)
      temp_bombs[tb_count++] = bombs_excluding_current[k];
    if (tb_count < MAX_BOMBS)
      temp_bombs[tb_count++] = current_pos;
    int car_dist = planner_v3_bfs_distance_between(rows, cols, current_car, (Point){push_row, push_col},
                                                    obstacles, obstacle_count, temp_bombs, tb_count,
                                                    boxes, box_count, 1);
    if (car_dist == INT_MAX)
      return 0;
    current_car = current_pos;  /* 推完后车位于炸弹原位置 */
    current_pos = next_pos;
  }
  return 1;
}

// 检查推位是否可达：车必须能到达推位格本身（非四邻即可）
// 若推位格为障碍/炸弹/箱子则不可达；include_bombs: 1=把炸弹当作障碍，0=不把炸弹当作障碍
static int planner_v3_bfs_is_push_pos_reachable(int rows, int cols, Point car, Point push_pos,
                                                 const Point *obstacles, size_t obstacle_count,
                                                 const Point *bombs, size_t bomb_count,
                                                 const Point *boxes, size_t box_count,
                                                 int include_bombs) {
  if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col)) {
    return 0;
  }
  // 推位格必须可站立（非障碍、非炸弹、非箱子）
  if (include_bombs) {
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, push_pos.row, push_pos.col)) {
      return 0;
    }
  } else {
    if (planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, push_pos.row, push_pos.col)) {
      return 0;
    }
    for (size_t i = 0; i < bomb_count; ++i) {
      if (bombs[i].row == push_pos.row && bombs[i].col == push_pos.col) {
        return 0;
      }
    }
  }
  if (planner_v3_bfs_is_box_at(boxes, box_count, push_pos.row, push_pos.col, SIZE_MAX)) {
    return 0;
  }
  // 检查车是否能到达推位格本身
  return planner_v3_bfs_distance_between(rows, cols, car, push_pos,
                                          obstacles, obstacle_count, bombs, bomb_count,
                                          boxes, box_count, include_bombs) != INT_MAX;
}

// 检查点 p 是否在 obstacles 列表中
static int planner_v3_bfs_point_in_list(const Point *obstacles, size_t count, int r, int c) {
  for (size_t i = 0; i < count; ++i) {
    if (obstacles[i].row == r && obstacles[i].col == c) {
      return 1;
    }
  }
  return 0;
}

// 检查多个障碍是否可被一颗炸弹炸掉：存在中心障碍 C，使得所有障碍都在 C 的爆炸范围内（C 本身 + C 的上下左右）
// 若存在，out_center 设为该中心；返回值 1=可被一弹炸掉，0=否
static int planner_v3_bfs_obstacles_destroyable_by_one_bomb(const Point *obstacles_on_path,
                                                            size_t count,
                                                            Point *out_center) {
  if (count == 0 || !obstacles_on_path || !out_center) {
    return 0;
  }
  if (count == 1) {
    *out_center = obstacles_on_path[0];
    return 1;
  }
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (size_t ci = 0; ci < count; ++ci) {
    Point center = obstacles_on_path[ci];
    int all_in_range = 1;
    for (size_t j = 0; j < count && all_in_range; ++j) {
      Point p = obstacles_on_path[j];
      if (p.row == center.row && p.col == center.col) {
        continue;
      }
      int adj = 0;
      for (int d = 0; d < 4; ++d) {
        if (p.row == center.row + dirs[d][0] && p.col == center.col + dirs[d][1]) {
          adj = 1;
          break;
        }
      }
      if (!adj) {
        all_in_range = 0;
      }
    }
    if (all_in_range) {
      *out_center = center;
      return 1;
    }
  }
  return 0;
}

// 检查推位是否合法：推位可以是该障碍及其上下左右周围的障碍，不能是其他障碍、边界之外、其他箱子
static int planner_v3_bfs_is_valid_push_pos_for_obstacle(int rows, int cols, Point push_pos,
                                                          Point obstacle,
                                                          const Point *obstacles, size_t obstacle_count,
                                                          const Point *boxes, size_t box_count,
                                                          size_t moving_idx) {
  // 检查是否在边界内
  if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col)) {
    return 0;
  }

  // 检查是否是其他箱子
  if (planner_v3_bfs_is_box_at(boxes, box_count, push_pos.row, push_pos.col, moving_idx)) {
    return 0;
  }

  // 检查推位是否是该障碍本身
  if (push_pos.row == obstacle.row && push_pos.col == obstacle.col) {
    return 1;  // 推位可以是障碍本身
  }

  // 检查推位是否是该障碍的上下左右周围的障碍
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (int d = 0; d < 4; ++d) {
    int adj_r = obstacle.row + dirs[d][0];
    int adj_c = obstacle.col + dirs[d][1];
    if (push_pos.row == adj_r && push_pos.col == adj_c) {
      // 检查这个位置是否是障碍
      if (planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, adj_r, adj_c)) {
        return 1;  // 推位可以是障碍周围的障碍
      }
    }
  }

  // 推位不能是其他障碍
  if (planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, push_pos.row, push_pos.col)) {
    return 0;
  }

  // 推位可以是空地
  return 1;
}

// 从路径障碍列表中选出距离箱子 box_start 曼哈顿距离最远的障碍（用于重算时只排除一个障碍并累计）
static Point planner_v3_bfs_farthest_obstacle_from_box(Point box_start,
    const Point *obstacles_on_path, size_t obstacles_on_path_count) {
  if (obstacles_on_path_count == 0) return (Point){-1, -1};
  int max_dist = -1;
  Point farthest = obstacles_on_path[0];
  for (size_t i = 0; i < obstacles_on_path_count; ++i) {
    int d = planner_v3_bfs_abs(obstacles_on_path[i].row - box_start.row) +
            planner_v3_bfs_abs(obstacles_on_path[i].col - box_start.col);
    if (d > max_dist) {
      max_dist = d;
      farthest = obstacles_on_path[i];
    }
  }
  return farthest;
}

// 特殊路径规划：忽略障碍和炸弹（不忽略箱子）进行一次BFS+A*路径计算，
// 判断该路径所经过的障碍。允许多个障碍，但需满足：
// 1) 多个障碍可被一颗炸弹炸掉 2) 炸弹可达（考虑除目标障碍外的所有障碍、所有箱子、除推动炸弹外的其他炸弹）
// 3) 推动炸弹到目标的每一步推位都合理
// 并且箱子的每一步推位合理（推位可为路径障碍本身或其相邻障碍，不能是其他障碍、边界外、其他箱子）
// 4) 若 car_pos 有效(row>=0,col>=0)，检验每一步推位的可达性（车能否到达推位），失败则排除障碍重算
// 返回值：1=成功找到路径，0=失败
// 如果成功且路径经过障碍，obstacle_passed会被设置为该障碍位置
static int planner_v3_bfs_special_path_planning(int rows, int cols, Point box_start, Point target,
                                                const Point *obstacles, size_t obstacle_count,
                                                const Point *bombs, size_t bomb_count,
                                                const Point *boxes, size_t box_count,
                                                size_t moving_idx,
                                                Point car_pos,
                                                Point *obstacle_passed, int *has_obstacle,
                                                Point *out_path, size_t out_path_cap,
                                                size_t *out_path_len) {
  if (out_path_len) {
    *out_path_len = 0;
  }

  int total_cells = rows * cols;
  if (total_cells <= 0 || total_cells > PLANNER_V3_BFS_MAX_CELLS) {
    return 0;
  }

  // 起点越界则失败
  if (!planner_v3_bfs_in_bounds(rows, cols, box_start.row, box_start.col) ||
      !planner_v3_bfs_in_bounds(rows, cols, target.row, target.col)) {
    return 0;
  }

  // 构建临时箱子数组（忽略正在移动的箱子）
  Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
  size_t temp_count = 0;
  for (size_t i = 0; i < box_count; ++i) {
    if (i == moving_idx) {
      continue;  // 忽略正在移动的箱子
    }
    if (boxes[i].row < 0 || boxes[i].col < 0) {
      continue;  // 忽略已完成的箱子
    }
    temp_boxes[temp_count++] = boxes[i];
  }

  Point empty_obstacles[1] = {{-1, -1}};
  Point empty_bombs[1] = {{-1, -1}};

  // 忽略障碍和炸弹（不忽略箱子），进行一次BFS+A*路径计算
  int dist_ignore_obs_bomb[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, 
                                           empty_obstacles, 0,  // 忽略障碍
                                           empty_bombs, 0,      // 忽略炸弹
                                           temp_boxes, temp_count,  // 考虑箱子
                                           dist_ignore_obs_bomb, 0)) {
    return 0;
  }

  // 使用A*计算路径（忽略障碍和炸弹，考虑箱子）
  Point initial_path[PLANNER_V3_BFS_MAX_PATH_LEN];
  size_t initial_path_len = 0;
  if (!planner_v3_bfs_astar_with_dist(rows, cols, box_start, target,
                                      empty_obstacles, 0,  // 忽略障碍
                                      empty_bombs, 0,      // 忽略炸弹
                                      temp_boxes, temp_count,  // 考虑箱子
                                      dist_ignore_obs_bomb,
                                      initial_path, PLANNER_V3_BFS_MAX_PATH_LEN, &initial_path_len,
                                      0, 0, NULL,
                                      0, 0, (Point){0, 0})) {
    return 0;
  }

  if (initial_path_len == 0) {
    return 0;
  }

  // 记录已排除的路径（通过记录路径经过的障碍集合）
  // 使用简单的哈希集合来记录已排除的障碍组合
  #define MAX_EXCLUDED_PATHS 50
  Point excluded_obstacle_sets[MAX_EXCLUDED_PATHS][200];
  size_t excluded_obstacle_counts[MAX_EXCLUDED_PATHS];
  size_t excluded_count = 0;
  
  const int max_attempts = 100;  // 防止死循环的最大尝试次数
  int attempt = 0;

  while (attempt < max_attempts) {
    attempt++;
    int reach_check_failed = 0;
    size_t reach_fail_step = 0;

    // 步骤1：判断该路径所经过的障碍
    Point obstacles_on_path[200];
    size_t obstacles_on_path_count = 0;
    for (size_t i = 0; i < initial_path_len && obstacles_on_path_count < 200; ++i) {
      if (planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count,
                                             initial_path[i].row, initial_path[i].col)) {
        // 检查是否已经记录过这个障碍
        int already_recorded = 0;
        for (size_t j = 0; j < obstacles_on_path_count; ++j) {
          if (obstacles_on_path[j].row == initial_path[i].row &&
              obstacles_on_path[j].col == initial_path[i].col) {
            already_recorded = 1;
            break;
          }
        }
        if (!already_recorded) {
          obstacles_on_path[obstacles_on_path_count++] = initial_path[i];
        }
      }
    }

    // 步骤2：若经过一个以上的障碍，检查是否可被一颗炸弹炸掉、炸弹可达、每步推位合理
    if (obstacles_on_path_count > 1) {
      Point center_obs = {-1, -1};
      if (!planner_v3_bfs_obstacles_destroyable_by_one_bomb(obstacles_on_path,
                                                           obstacles_on_path_count,
                                                           &center_obs)) {
        // 多个障碍不能被一颗炸弹炸掉，只排除距箱子曼哈顿距离最远的障碍并重新算路（仍累计）
        Point farthest = planner_v3_bfs_farthest_obstacle_from_box(box_start,
            obstacles_on_path, obstacles_on_path_count);
        int already_excluded = 0;
        for (size_t i = 0; i < excluded_count; ++i) {
          if (excluded_obstacle_counts[i] == 1 &&
              excluded_obstacle_sets[i][0].row == farthest.row &&
              excluded_obstacle_sets[i][0].col == farthest.col) {
            already_excluded = 1;
            break;
          }
        }
        if (!already_excluded && excluded_count < MAX_EXCLUDED_PATHS) {
          excluded_obstacle_counts[excluded_count] = 1;
          excluded_obstacle_sets[excluded_count][0] = farthest;
          excluded_count++;
        }
        // 重算时累计所有已排除的障碍（每次只加最远一个，仍累计）
        uint8_t exclude_obstacle[200] = {0};
        for (size_t ex = 0; ex < excluded_count; ++ex) {
          for (size_t j = 0; j < excluded_obstacle_counts[ex]; ++j) {
            Point excl_obs = excluded_obstacle_sets[ex][j];
            for (size_t k = 0; k < obstacle_count; ++k) {
              if (obstacles[k].row == excl_obs.row && obstacles[k].col == excl_obs.col) {
                exclude_obstacle[k] = 1;
              }
            }
          }
        }
        Point filtered_obstacles[200];
        size_t filtered_obstacle_count = 0;
        for (size_t i = 0; i < obstacle_count && filtered_obstacle_count < 200; ++i) {
          if (exclude_obstacle[i]) {
            filtered_obstacles[filtered_obstacle_count++] = obstacles[i];
          }
        }
        if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, 
                                                   filtered_obstacles, filtered_obstacle_count,
                                                   empty_bombs, 0, temp_boxes, temp_count,
                                                   dist_ignore_obs_bomb, 0)) {
          return 0;
        }
        if (!planner_v3_bfs_astar_with_dist(rows, cols, box_start, target,
                                            filtered_obstacles, filtered_obstacle_count,
                                            empty_bombs, 0, temp_boxes, temp_count,
                                            dist_ignore_obs_bomb,
                                            initial_path, PLANNER_V3_BFS_MAX_PATH_LEN, &initial_path_len,
                                            0, 0, NULL, 0, 0, (Point){0, 0})) {
          return 0;
        }
        if (initial_path_len == 0) return 0;
        continue;
      }

      // 可被一弹炸掉，检查每步推位合法性：推位可为“本步涉及障碍”本身或其相邻障碍
      // 多障碍时每步用 from 或 to（若在路径障碍中）作为参考，确保推位合理
      int push_check_failed = 0;
      for (size_t i = 1; i < initial_path_len && !push_check_failed; ++i) {
        Point from = initial_path[i - 1];
        Point to   = initial_path[i];
        int dr = to.row - from.row;
        int dc = to.col - from.col;
        Point push_pos = (Point){ from.row - dr, from.col - dc };
        Point ref_obs = {-1, -1};
        if (planner_v3_bfs_point_in_list(obstacles_on_path, obstacles_on_path_count,
                                         from.row, from.col)) {
          ref_obs = from;
        } else if (planner_v3_bfs_point_in_list(obstacles_on_path, obstacles_on_path_count,
                                                to.row, to.col)) {
          ref_obs = to;
        }
        if (ref_obs.row >= 0) {
          if (!planner_v3_bfs_is_valid_push_pos_for_obstacle(rows, cols, push_pos, ref_obs,
                                                            obstacles, obstacle_count,
                                                            boxes, box_count, moving_idx)) {
            push_check_failed = 1;
          }
        } else {
          /* from/to 均非路径障碍，推位不能是任何障碍、箱子或越界 */
          if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col) ||
              planner_v3_bfs_is_box_at(boxes, box_count, push_pos.row, push_pos.col, moving_idx) ||
              planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, push_pos.row, push_pos.col)) {
            push_check_failed = 1;
          }
        }
      }

      if (push_check_failed) {
        // 推位不合法，只排除距箱子曼哈顿距离最远的障碍并重新算路（仍累计）
        Point farthest = planner_v3_bfs_farthest_obstacle_from_box(box_start,
            obstacles_on_path, obstacles_on_path_count);
        int already_excluded = 0;
        for (size_t i = 0; i < excluded_count; ++i) {
          if (excluded_obstacle_counts[i] == 1 &&
              excluded_obstacle_sets[i][0].row == farthest.row &&
              excluded_obstacle_sets[i][0].col == farthest.col) {
            already_excluded = 1;
            break;
          }
        }
        if (!already_excluded && excluded_count < MAX_EXCLUDED_PATHS) {
          excluded_obstacle_counts[excluded_count] = 1;
          excluded_obstacle_sets[excluded_count][0] = farthest;
          excluded_count++;
        }
        // 重算时累计所有已排除的障碍（每次只加最远一个，仍累计）
        uint8_t exclude_obstacle[200] = {0};
        for (size_t ex = 0; ex < excluded_count; ++ex) {
          for (size_t j = 0; j < excluded_obstacle_counts[ex]; ++j) {
            Point excl_obs = excluded_obstacle_sets[ex][j];
            for (size_t k = 0; k < obstacle_count; ++k) {
              if (obstacles[k].row == excl_obs.row && obstacles[k].col == excl_obs.col) {
                exclude_obstacle[k] = 1;
              }
            }
          }
        }
        Point filtered_obstacles[200];
        size_t filtered_obstacle_count = 0;
        for (size_t i = 0; i < obstacle_count && filtered_obstacle_count < 200; ++i) {
          if (exclude_obstacle[i]) {
            filtered_obstacles[filtered_obstacle_count++] = obstacles[i];
          }
        }
        if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, 
                                                   filtered_obstacles, filtered_obstacle_count,
                                                   empty_bombs, 0, temp_boxes, temp_count,
                                                   dist_ignore_obs_bomb, 0)) {
          return 0;
        }
        if (!planner_v3_bfs_astar_with_dist(rows, cols, box_start, target,
                                            filtered_obstacles, filtered_obstacle_count,
                                            empty_bombs, 0, temp_boxes, temp_count,
                                            dist_ignore_obs_bomb,
                                            initial_path, PLANNER_V3_BFS_MAX_PATH_LEN, &initial_path_len,
                                            0, 0, NULL, 0, 0, (Point){0, 0})) {
          return 0;
        }
        if (initial_path_len == 0) return 0;
        continue;
      }

      // 炸弹可达性检查：只能忽略目标障碍 C（center_obs），其他障碍均需考虑；
      // 炸弹目标只能是 center_obs；且需检验炸弹路径上每一步的推位是否合理
      Point filtered_for_bomb[200];  /* 仅排除 center_obs，包含路径上其他障碍 */
      size_t nf = 0;
      for (size_t i = 0; i < obstacle_count && nf < 200; ++i) {
        if (obstacles[i].row == center_obs.row && obstacles[i].col == center_obs.col) {
          continue;  /* 只忽略目标障碍 C */
        }
        filtered_for_bomb[nf++] = obstacles[i];
      }

      int bomb_reachable = 0;
      for (size_t bi = 0; bi < bomb_count && !bomb_reachable; ++bi) {
        if (bombs[bi].row < 0 || bombs[bi].col < 0) continue;
        Point temp_bombs[MAX_BOMBS];
        size_t tb_count = 0;
        for (size_t bj = 0; bj < bomb_count && tb_count < MAX_BOMBS; ++bj) {
          if (bj == bi) continue;
          if (bombs[bj].row < 0 || bombs[bj].col < 0) continue;
          temp_bombs[tb_count++] = bombs[bj];
        }

        /* 可达性：只忽略 center_obs，炸弹需能到达 center_obs */
        int dist = planner_v3_bfs_distance_between(rows, cols, bombs[bi], center_obs,
                                                   filtered_for_bomb, nf, temp_bombs, tb_count,
                                                   boxes, box_count, 1);
        if (dist == INT_MAX) continue;

        /* 推位检验：炸弹路径上每一步推位需合理（推位不能被不可接受的障碍占据） */
        Point push_obs = planner_v3_bfs_check_bomb_path_push_obstacle(
            rows, cols, bombs[bi], center_obs,
            obstacles, obstacle_count, bombs, bomb_count, bi,
            boxes, box_count);
        if (push_obs.row >= 0) continue;  /* 推位被障碍占据，该炸弹不可用 */

        bomb_reachable = 1;
        break;
      }
      if (!bomb_reachable) {
        // 炸弹不可达，只排除距箱子曼哈顿距离最远的障碍并重新算路（仍累计）
        Point farthest = planner_v3_bfs_farthest_obstacle_from_box(box_start,
            obstacles_on_path, obstacles_on_path_count);
        int already_excluded = 0;
        for (size_t i = 0; i < excluded_count; ++i) {
          if (excluded_obstacle_counts[i] == 1 &&
              excluded_obstacle_sets[i][0].row == farthest.row &&
              excluded_obstacle_sets[i][0].col == farthest.col) {
            already_excluded = 1;
            break;
          }
        }
        if (!already_excluded && excluded_count < MAX_EXCLUDED_PATHS) {
          excluded_obstacle_counts[excluded_count] = 1;
          excluded_obstacle_sets[excluded_count][0] = farthest;
          excluded_count++;
        }
        // 重算时累计所有已排除的障碍（每次只加最远一个，仍累计）
        uint8_t exclude_obstacle[200] = {0};
        for (size_t ex = 0; ex < excluded_count; ++ex) {
          for (size_t j = 0; j < excluded_obstacle_counts[ex]; ++j) {
            Point excl_obs = excluded_obstacle_sets[ex][j];
            for (size_t k = 0; k < obstacle_count; ++k) {
              if (obstacles[k].row == excl_obs.row && obstacles[k].col == excl_obs.col) {
                exclude_obstacle[k] = 1;
              }
            }
          }
        }
        Point filtered_obstacles[200];
        size_t filtered_obstacle_count = 0;
        for (size_t i = 0; i < obstacle_count && filtered_obstacle_count < 200; ++i) {
          if (exclude_obstacle[i]) {
            filtered_obstacles[filtered_obstacle_count++] = obstacles[i];
          }
        }
        if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, 
                                                   filtered_obstacles, filtered_obstacle_count,
                                                   empty_bombs, 0, temp_boxes, temp_count,
                                                   dist_ignore_obs_bomb, 0)) {
          return 0;
        }
        if (!planner_v3_bfs_astar_with_dist(rows, cols, box_start, target,
                                            filtered_obstacles, filtered_obstacle_count,
                                            empty_bombs, 0, temp_boxes, temp_count,
                                            dist_ignore_obs_bomb,
                                            initial_path, PLANNER_V3_BFS_MAX_PATH_LEN, &initial_path_len,
                                            0, 0, NULL, 0, 0, (Point){0, 0})) {
          return 0;
        }
        if (initial_path_len == 0) return 0;
        continue;
      }

      // 推位可达性检验：若提供了车位置，检验每一步推位车能否到达，失败则排除该失败点重算
      // 检验时忽略特殊路径上的障碍和炸弹（路径障碍将被炸掉，路径炸弹将被推走）
      reach_check_failed = 0;
      reach_fail_step = 0;
      if (car_pos.row >= 0 && car_pos.col >= 0 && initial_path_len >= 2) {
        Point obs_excl_path[200];
        size_t obs_excl_count = 0;
        for (size_t k = 0; k < obstacle_count && obs_excl_count < 200; ++k) {
          if (!planner_v3_bfs_point_in_list(obstacles_on_path, obstacles_on_path_count,
                                            obstacles[k].row, obstacles[k].col)) {
            obs_excl_path[obs_excl_count++] = obstacles[k];
          }
        }
        Point bombs_excl_path[MAX_BOMBS];
        size_t bombs_excl_count = 0;
        for (size_t k = 0; k < bomb_count && bombs_excl_count < MAX_BOMBS; ++k) {
          if (bombs[k].row < 0 || bombs[k].col < 0) continue;
          if (!planner_v3_bfs_point_on_path(bombs[k], initial_path, initial_path_len)) {
            bombs_excl_path[bombs_excl_count++] = bombs[k];
          }
        }
        if (!planner_v3_bfs_validate_box_path_push_positions(rows, cols, initial_path, initial_path_len,
            car_pos, obs_excl_path, obs_excl_count, bombs_excl_path, bombs_excl_count,
            boxes, box_count, moving_idx, 1, &reach_fail_step)) {
          reach_check_failed = 1;
        }
      }
      if (reach_check_failed) {
        /* 推位不可达时排除点A（从A推到B不可达，A=箱子推前位置=initial_path[reach_fail_step-1]）*/
        Point fail_point;
        if (reach_fail_step > 0 && reach_fail_step < initial_path_len) {
          fail_point = initial_path[reach_fail_step - 1];  /* 点A：推前箱子位置 */
        } else {
          fail_point = planner_v3_bfs_farthest_obstacle_from_box(box_start,
              obstacles_on_path, obstacles_on_path_count);
        }
        int already_excluded = 0;
        for (size_t i = 0; i < excluded_count; ++i) {
          if (excluded_obstacle_counts[i] == 1 &&
              excluded_obstacle_sets[i][0].row == fail_point.row &&
              excluded_obstacle_sets[i][0].col == fail_point.col) {
            already_excluded = 1;
            break;
          }
        }
        if (!already_excluded && excluded_count < MAX_EXCLUDED_PATHS) {
          excluded_obstacle_counts[excluded_count] = 1;
          excluded_obstacle_sets[excluded_count][0] = fail_point;
          excluded_count++;
        }
        uint8_t exclude_obstacle[200] = {0};
        for (size_t ex = 0; ex < excluded_count; ++ex) {
          for (size_t j = 0; j < excluded_obstacle_counts[ex]; ++j) {
            Point excl_obs = excluded_obstacle_sets[ex][j];
            for (size_t k = 0; k < obstacle_count; ++k) {
              if (obstacles[k].row == excl_obs.row && obstacles[k].col == excl_obs.col) {
                exclude_obstacle[k] = 1;
              }
            }
          }
        }
        Point filtered_obstacles[200];
        size_t filtered_obstacle_count = 0;
        for (size_t i = 0; i < obstacle_count && filtered_obstacle_count < 200; ++i) {
          if (exclude_obstacle[i]) {
            filtered_obstacles[filtered_obstacle_count++] = obstacles[i];
          }
        }
        for (size_t ex = 0; ex < excluded_count && filtered_obstacle_count < 200; ++ex) {
          for (size_t j = 0; j < excluded_obstacle_counts[ex]; ++j) {
            Point excl_obs = excluded_obstacle_sets[ex][j];
            int in_filtered = 0;
            for (size_t k = 0; k < filtered_obstacle_count; ++k) {
              if (filtered_obstacles[k].row == excl_obs.row && filtered_obstacles[k].col == excl_obs.col) {
                in_filtered = 1;
                break;
              }
            }
            if (!in_filtered) {
              filtered_obstacles[filtered_obstacle_count++] = excl_obs;
            }
          }
        }
        if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, 
                                                   filtered_obstacles, filtered_obstacle_count,
                                                   empty_bombs, 0, temp_boxes, temp_count,
                                                   dist_ignore_obs_bomb, 0)) {
          return 0;
        }
        if (!planner_v3_bfs_astar_with_dist(rows, cols, box_start, target,
                                            filtered_obstacles, filtered_obstacle_count,
                                            empty_bombs, 0, temp_boxes, temp_count,
                                            dist_ignore_obs_bomb,
                                            initial_path, PLANNER_V3_BFS_MAX_PATH_LEN, &initial_path_len,
                                            0, 0, NULL,
                                            0, 0, (Point){0, 0})) {
          return 0;
        }
        if (initial_path_len == 0) return 0;
        continue;
      }

      // 多障碍通过检查：可被一弹炸掉、炸弹可达、推位合法、推位可达；保存并返回
      s_special_path_obstacle_count = obstacles_on_path_count;
      if (s_special_path_obstacle_count > 200) s_special_path_obstacle_count = 200;
      for (size_t i = 0; i < s_special_path_obstacle_count; ++i) {
        s_special_path_obstacles[i] = obstacles_on_path[i];
      }
      if (out_path_len) *out_path_len = initial_path_len;
      if (out_path && out_path_cap > 0) {
        size_t copy_len = (initial_path_len < out_path_cap) ? initial_path_len : out_path_cap;
        memcpy(out_path, initial_path, copy_len * sizeof(Point));
      }
      if (has_obstacle) *has_obstacle = 1;
      if (obstacle_passed) *obstacle_passed = center_obs;
      return 1;
    } else {
      // 经过0个或1个障碍，检查步骤里每一步的推位是否合法
      // 推位必须在边界内、不能是其他箱子；若推位是障碍则只能是“该障碍本身”或“该障碍上下左右相邻的障碍”
      int push_check_failed = 0;
      size_t push_fail_step_0 = 0;  /* 0 障碍时推位失败所在步，用于排除 A 点重算 */
      if (obstacles_on_path_count == 0) {
        // 0 个障碍：每一步推位 = from - (to-from)，必须在边界内、不能是其他箱子、不能是任何障碍
        for (size_t i = 1; i < initial_path_len && !push_check_failed; ++i) {
          Point from = initial_path[i - 1];
          Point to   = initial_path[i];
          int dr = to.row - from.row;
          int dc = to.col - from.col;
          Point push_pos = (Point){ from.row - dr, from.col - dc };
          if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col)) {
            push_check_failed = 1;
            push_fail_step_0 = i;
            break;
          }
          if (planner_v3_bfs_is_box_at(boxes, box_count, push_pos.row, push_pos.col, moving_idx)) {
            push_check_failed = 1;
            push_fail_step_0 = i;
            break;
          }
          if (planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, push_pos.row, push_pos.col)) {
            push_check_failed = 1;
            push_fail_step_0 = i;
            break;
          }
        }
      } else {
        // 1 个障碍：每一步推位用“该障碍”做合法性检验（推位可为该障碍本身或其上下左右相邻障碍）
        for (size_t i = 1; i < initial_path_len && !push_check_failed; ++i) {
          Point from = initial_path[i - 1];
          Point to   = initial_path[i];
          int dr = to.row - from.row;
          int dc = to.col - from.col;
          Point push_pos = (Point){ from.row - dr, from.col - dc };
          if (!planner_v3_bfs_is_valid_push_pos_for_obstacle(rows, cols, push_pos, obstacles_on_path[0],
                                                              obstacles, obstacle_count,
                                                              boxes, box_count, moving_idx)) {
            push_check_failed = 1;
            break;
          }
        }
      }

      if (push_check_failed && obstacles_on_path_count == 1) {
        // 推位不合法，排除该障碍（单障碍即“最远”），用已排除障碍累计后重新算路
        int already_excluded = 0;
        for (size_t i = 0; i < excluded_count; ++i) {
          if (excluded_obstacle_counts[i] == 1 &&
              excluded_obstacle_sets[i][0].row == obstacles_on_path[0].row &&
              excluded_obstacle_sets[i][0].col == obstacles_on_path[0].col) {
            already_excluded = 1;
            break;
          }
        }

        if (!already_excluded && excluded_count < MAX_EXCLUDED_PATHS) {
          excluded_obstacle_counts[excluded_count] = 1;
          excluded_obstacle_sets[excluded_count][0] = obstacles_on_path[0];
          excluded_count++;
        }

        // 重算时累计所有已排除的障碍（仍累计）
        uint8_t exclude_obstacle[200] = {0};
        for (size_t ex = 0; ex < excluded_count; ++ex) {
          for (size_t j = 0; j < excluded_obstacle_counts[ex]; ++j) {
            Point excl_obs = excluded_obstacle_sets[ex][j];
            for (size_t k = 0; k < obstacle_count; ++k) {
              if (obstacles[k].row == excl_obs.row && obstacles[k].col == excl_obs.col) {
                exclude_obstacle[k] = 1;
              }
            }
          }
        }

        Point filtered_obstacles[200];
        size_t filtered_obstacle_count = 0;
        for (size_t i = 0; i < obstacle_count && filtered_obstacle_count < 200; ++i) {
          if (exclude_obstacle[i]) {
            filtered_obstacles[filtered_obstacle_count++] = obstacles[i];
          }
        }

        if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, 
                                                   filtered_obstacles, filtered_obstacle_count,
                                                   empty_bombs, 0,
                                                   temp_boxes, temp_count,
                                                   dist_ignore_obs_bomb, 0)) {
          return 0;
        }

        if (!planner_v3_bfs_astar_with_dist(rows, cols, box_start, target,
                                            filtered_obstacles, filtered_obstacle_count,
                                            empty_bombs, 0,
                                            temp_boxes, temp_count,
                                            dist_ignore_obs_bomb,
                                            initial_path, PLANNER_V3_BFS_MAX_PATH_LEN, &initial_path_len,
                                            0, 0, NULL,
                                            0, 0, (Point){0, 0})) {
          return 0;
        }

        if (initial_path_len == 0) {
          return 0;
        }

        continue;
      }

      if (push_check_failed && obstacles_on_path_count == 0) {
        // 0 障碍时推位不合法，排除 A 点（A = 箱子推前位置 = initial_path[push_fail_step_0-1]）并重算
        Point fail_point;
        if (push_fail_step_0 > 0 && push_fail_step_0 < initial_path_len) {
          fail_point = initial_path[push_fail_step_0 - 1];
        } else {
          return 0;
        }
        int already_excluded = 0;
        for (size_t i = 0; i < excluded_count; ++i) {
          if (excluded_obstacle_counts[i] == 1 &&
              excluded_obstacle_sets[i][0].row == fail_point.row &&
              excluded_obstacle_sets[i][0].col == fail_point.col) {
            already_excluded = 1;
            break;
          }
        }
        if (already_excluded || excluded_count >= MAX_EXCLUDED_PATHS) {
          return 0;  /* 已排除过 A 点仍失败或排除数达上限，无法再通过排除重算 */
        }
        excluded_obstacle_counts[excluded_count] = 1;
        excluded_obstacle_sets[excluded_count][0] = fail_point;
        excluded_count++;
        uint8_t exclude_obstacle[200] = {0};
        for (size_t ex = 0; ex < excluded_count; ++ex) {
          for (size_t j = 0; j < excluded_obstacle_counts[ex]; ++j) {
            Point excl_obs = excluded_obstacle_sets[ex][j];
            for (size_t k = 0; k < obstacle_count; ++k) {
              if (obstacles[k].row == excl_obs.row && obstacles[k].col == excl_obs.col) {
                exclude_obstacle[k] = 1;
              }
            }
          }
        }
        Point filtered_obstacles[200];
        size_t filtered_obstacle_count = 0;
        for (size_t i = 0; i < obstacle_count && filtered_obstacle_count < 200; ++i) {
          if (exclude_obstacle[i]) {
            filtered_obstacles[filtered_obstacle_count++] = obstacles[i];
          }
        }
        for (size_t ex = 0; ex < excluded_count && filtered_obstacle_count < 200; ++ex) {
          for (size_t j = 0; j < excluded_obstacle_counts[ex]; ++j) {
            Point excl_obs = excluded_obstacle_sets[ex][j];
            int in_filtered = 0;
            for (size_t k = 0; k < filtered_obstacle_count; ++k) {
              if (filtered_obstacles[k].row == excl_obs.row && filtered_obstacles[k].col == excl_obs.col) {
                in_filtered = 1;
                break;
              }
            }
            if (!in_filtered) {
              filtered_obstacles[filtered_obstacle_count++] = excl_obs;
            }
          }
        }
        if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target,
                                                   filtered_obstacles, filtered_obstacle_count,
                                                   empty_bombs, 0,
                                                   temp_boxes, temp_count,
                                                   dist_ignore_obs_bomb, 0)) {
          return 0;
        }
        if (!planner_v3_bfs_astar_with_dist(rows, cols, box_start, target,
                                            filtered_obstacles, filtered_obstacle_count,
                                            empty_bombs, 0,
                                            temp_boxes, temp_count,
                                            dist_ignore_obs_bomb,
                                            initial_path, PLANNER_V3_BFS_MAX_PATH_LEN, &initial_path_len,
                                            0, 0, NULL,
                                            0, 0, (Point){0, 0})) {
          return 0;
        }
        if (initial_path_len == 0) {
          return 0;
        }
        continue;
      }

      // 推位可达性检验：若提供了车位置，检验每一步推位车能否到达，失败则排除该失败点重算
      // 检验时忽略特殊路径上的障碍和炸弹（路径障碍将被炸掉，路径炸弹将被推走）
      reach_check_failed = 0;
      reach_fail_step = 0;
      if (car_pos.row >= 0 && car_pos.col >= 0 && initial_path_len >= 2) {
        Point obs_excl_path[200];
        size_t obs_excl_count = 0;
        for (size_t k = 0; k < obstacle_count && obs_excl_count < 200; ++k) {
          if (!planner_v3_bfs_point_in_list(obstacles_on_path, obstacles_on_path_count,
                                            obstacles[k].row, obstacles[k].col)) {
            obs_excl_path[obs_excl_count++] = obstacles[k];
          }
        }
        Point bombs_excl_path[MAX_BOMBS];
        size_t bombs_excl_count = 0;
        for (size_t k = 0; k < bomb_count && bombs_excl_count < MAX_BOMBS; ++k) {
          if (bombs[k].row < 0 || bombs[k].col < 0) continue;
          if (!planner_v3_bfs_point_on_path(bombs[k], initial_path, initial_path_len)) {
            bombs_excl_path[bombs_excl_count++] = bombs[k];
          }
        }
        if (!planner_v3_bfs_validate_box_path_push_positions(rows, cols, initial_path, initial_path_len,
            car_pos, obs_excl_path, obs_excl_count, bombs_excl_path, bombs_excl_count,
            boxes, box_count, moving_idx, 1, &reach_fail_step)) {
          reach_check_failed = 1;
        }
      }
      if (reach_check_failed) {
        /* 推位不可达时无论是否经过障碍都重算，排除点A（A=箱子推前位置=initial_path[reach_fail_step-1]）并累计 */
        Point fail_point;
        if (reach_fail_step > 0 && reach_fail_step < initial_path_len) {
          fail_point = initial_path[reach_fail_step - 1];  /* 点A：推前箱子位置 */
        } else if (obstacles_on_path_count > 0) {
          fail_point = obstacles_on_path[0];
        } else {
          return 0;  /* 无法获取失败点，无法重算 */
        }
        int already_excluded = 0;
        for (size_t i = 0; i < excluded_count; ++i) {
          if (excluded_obstacle_counts[i] == 1 &&
              excluded_obstacle_sets[i][0].row == fail_point.row &&
              excluded_obstacle_sets[i][0].col == fail_point.col) {
            already_excluded = 1;
            break;
          }
        }
        if (!already_excluded && excluded_count < MAX_EXCLUDED_PATHS) {
          excluded_obstacle_counts[excluded_count] = 1;
          excluded_obstacle_sets[excluded_count][0] = fail_point;
          excluded_count++;
        }
        uint8_t exclude_obstacle[200] = {0};
        for (size_t ex = 0; ex < excluded_count; ++ex) {
          for (size_t j = 0; j < excluded_obstacle_counts[ex]; ++j) {
            Point excl_obs = excluded_obstacle_sets[ex][j];
            for (size_t k = 0; k < obstacle_count; ++k) {
              if (obstacles[k].row == excl_obs.row && obstacles[k].col == excl_obs.col) {
                exclude_obstacle[k] = 1;
              }
            }
          }
        }
        Point filtered_obstacles[200];
        size_t filtered_obstacle_count = 0;
        for (size_t i = 0; i < obstacle_count && filtered_obstacle_count < 200; ++i) {
          if (exclude_obstacle[i]) {
            filtered_obstacles[filtered_obstacle_count++] = obstacles[i];
          }
        }
        for (size_t ex = 0; ex < excluded_count && filtered_obstacle_count < 200; ++ex) {
          for (size_t j = 0; j < excluded_obstacle_counts[ex]; ++j) {
            Point excl_obs = excluded_obstacle_sets[ex][j];
            int in_filtered = 0;
            for (size_t k = 0; k < filtered_obstacle_count; ++k) {
              if (filtered_obstacles[k].row == excl_obs.row && filtered_obstacles[k].col == excl_obs.col) {
                in_filtered = 1;
                break;
              }
            }
            if (!in_filtered) {
              filtered_obstacles[filtered_obstacle_count++] = excl_obs;
            }
          }
        }
        if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, 
                                                   filtered_obstacles, filtered_obstacle_count,
                                                   empty_bombs, 0,
                                                   temp_boxes, temp_count,
                                                   dist_ignore_obs_bomb, 0)) {
          return 0;
        }
        if (!planner_v3_bfs_astar_with_dist(rows, cols, box_start, target,
                                            filtered_obstacles, filtered_obstacle_count,
                                            empty_bombs, 0,
                                            temp_boxes, temp_count,
                                            dist_ignore_obs_bomb,
                                            initial_path, PLANNER_V3_BFS_MAX_PATH_LEN, &initial_path_len,
                                            0, 0, NULL,
                                            0, 0, (Point){0, 0})) {
          return 0;
        }
        if (initial_path_len == 0) {
          return 0;
        }
        continue;
      }

      // 找到合法路径，返回结果
      s_special_path_obstacle_count = obstacles_on_path_count;
      if (s_special_path_obstacle_count > 200) s_special_path_obstacle_count = 200;
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

  // 超过最大尝试次数，返回失败
  return 0;
}

// 找到距离目标障碍最近的炸弹，并返回炸弹索引和炸弹要被推到的位置
// bomb_target_pos 允许是"目标障碍本身"或"目标障碍上下左右的障碍格"
// 优先选取BFS真实距离（考虑除此障碍以外的所有障碍，箱子以及炸弹）最近的
// 返回值：1=成功找到，0=失败
static int planner_v3_bfs_find_nearest_bomb(int rows, int cols, Point car,
                                             const Point *bombs, size_t bomb_count,
                                             Point target_obstacle,
                                             const Point *obstacles, size_t obstacle_count,
                                             const Point *boxes, size_t box_count,
                                             size_t *out_bomb_idx, Point *out_bomb_target_pos) {
  if (bomb_count == 0 || !bombs) {
    return 0;
  }
  
  size_t best_bomb_idx = SIZE_MAX;
  Point best_bomb_target_pos = {-1, -1};
  int best_dist = INT_MAX;
  
  // 收集候选目标位置：该障碍本身 + 该障碍上下左右周围的障碍（如果这些位置是障碍）
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  Point candidate_positions[5];  // 最多5个候选：障碍本身 + 4个相邻位置
  int candidate_count = 0;
  
  // 添加目标障碍本身
  candidate_positions[candidate_count++] = target_obstacle;
  
  // 添加目标障碍上下左右周围的障碍（如果这些位置是障碍）
  for (int d = 0; d < 4; ++d) {
    int nr = target_obstacle.row + dirs[d][0];
    int nc = target_obstacle.col + dirs[d][1];
    if (planner_v3_bfs_in_bounds(rows, cols, nr, nc)) {
      // 检查这个位置是否是障碍
      if (planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, nr, nc)) {
        candidate_positions[candidate_count].row = nr;
        candidate_positions[candidate_count].col = nc;
        candidate_count++;
      }
    }
  }
  
  // 遍历所有炸弹
  for (size_t bi = 0; bi < bomb_count; ++bi) {
    Point bomb = bombs[bi];
    
    // 检查炸弹是否已经被使用（标记为负坐标表示已使用）
    if (bomb.row < 0 || bomb.col < 0) {
      continue;
    }
    
    // 遍历所有候选目标位置
    for (int ci = 0; ci < candidate_count; ++ci) {
      Point candidate = candidate_positions[ci];
      // 推炸弹时只排除 target_pos（当前候选即炸弹要推到的格）
      // candidate == target_obstacle 时排除该障碍；candidate 为相邻障碍时排除该相邻障碍
      Point filtered_obstacles[200];
      size_t filtered_obstacle_count = 0;
      for (size_t i = 0; i < obstacle_count && filtered_obstacle_count < 200; ++i) {
        if (obstacles[i].row != candidate.row || obstacles[i].col != candidate.col) {
          filtered_obstacles[filtered_obstacle_count++] = obstacles[i];
        }
      }
      
      // 检查炸弹是否能被推到候选位置；推位 = from - (to-from)，to=candidate，from=candidate-dir，故 push = candidate - 2*dir
      int push_pos_reachable = 0;
      for (int d = 0; d < 4; ++d) {
        int dr = dirs[d][0];
        int dc = dirs[d][1];
        int push_r = candidate.row - dr - dr;  /* from = candidate-dir, push = from - (to-from) = candidate - 2*dir */
        int push_c = candidate.col - dc - dc;
        if (planner_v3_bfs_in_bounds(rows, cols, push_r, push_c)) {
          if (planner_v3_bfs_is_push_pos_reachable(rows, cols, car, (Point){push_r, push_c},
                                                   filtered_obstacles, filtered_obstacle_count,
                                                   bombs, bomb_count,
                                                   boxes, box_count, 1)) {  // 1=把炸弹当作障碍
            push_pos_reachable = 1;
            break;
          }
        }
      }
      
      if (!push_pos_reachable) {
        continue;
      }
      
      // 计算炸弹到候选目标位置的BFS真实距离
      // 只排除当前候选（target_pos），不把当前炸弹当作障碍
      Point temp_bombs_for_dist[MAX_BOMBS];
      size_t temp_bombs_for_dist_count = 0;
      for (size_t i = 0; i < bomb_count && temp_bombs_for_dist_count < MAX_BOMBS; ++i) {
        if (i == bi) {
          continue;  // 不把当前炸弹当作障碍
        }
        if (bombs[i].row < 0 || bombs[i].col < 0) {
          continue;
        }
        temp_bombs_for_dist[temp_bombs_for_dist_count++] = bombs[i];
      }
      
      int dist = planner_v3_bfs_distance_between(rows, cols, bomb, candidate,
                                                filtered_obstacles, filtered_obstacle_count,
                                                temp_bombs_for_dist, temp_bombs_for_dist_count,
                                                boxes, box_count, 1);  // 1=把炸弹当作障碍
      
      if (dist != INT_MAX && dist < best_dist) {
        best_dist = dist;
        best_bomb_idx = bi;
        best_bomb_target_pos = candidate;
      }
    }
  }
  
  if (best_bomb_idx == SIZE_MAX) {
    return 0;  // 没有找到可达的炸弹
  }
  
  *out_bomb_idx = best_bomb_idx;
  *out_bomb_target_pos = best_bomb_target_pos;
  return 1;
}

// 按“炸弹到特殊路径经过的障碍”的BFS距离升序排列炸弹索引，供推炸弹环节依次尝试
static void planner_v3_bfs_find_bombs_on_path(const Point *path, size_t path_len,
                                               const Point *bombs, size_t bomb_count,
                                               size_t *bombs_on_path, size_t *bombs_on_path_count) {
  // 检查特殊路径上是否有炸弹，返回路径上的炸弹索引列表
  // 输出：bombs_on_path[0..*bombs_on_path_count-1] 为路径上炸弹的索引
  *bombs_on_path_count = 0;
  if (!path || path_len == 0 || !bombs || bomb_count == 0) {
    return;
  }
  
  for (size_t bi = 0; bi < bomb_count && *bombs_on_path_count < MAX_BOMBS; ++bi) {
    if (bombs[bi].row < 0 || bombs[bi].col < 0) {
      continue;  // 跳过已使用的炸弹
    }
    for (size_t pi = 0; pi < path_len; ++pi) {
      if (path[pi].row == bombs[bi].row && path[pi].col == bombs[bi].col) {
        bombs_on_path[(*bombs_on_path_count)++] = bi;
        break;
      }
    }
  }
}

// 判断点是否在路径上
static int planner_v3_bfs_point_on_path(Point pt, const Point *path, size_t path_len) {
  if (!path || path_len == 0) return 0;
  for (size_t i = 0; i < path_len; ++i) {
    if (path[i].row == pt.row && path[i].col == pt.col) return 1;
  }
  return 0;
}

// 找到距离炸弹最近的障碍列表（按BFS距离升序排列）
// 输出：nearest_obstacles[0..*nearest_count-1] 为障碍点，距离最近的在前
#define MAX_NEAREST_OBSTACLES 20
static void planner_v3_bfs_find_nearest_obstacles_for_bomb(int rows, int cols,
                                                           Point bomb_pos,
                                                           const Point *obstacles, size_t obstacle_count,
                                                           const Point *bombs, size_t bomb_count, size_t bomb_idx,
                                                           const Point *boxes, size_t box_count,
                                                           Point *nearest_obstacles, size_t *nearest_count) {
  *nearest_count = 0;
  if (obstacle_count == 0 || !obstacles) {
    return;
  }
  
  // 构建临时炸弹数组（排除当前炸弹）
  Point temp_bombs[MAX_BOMBS];
  size_t temp_bomb_count = 0;
  for (size_t i = 0; i < bomb_count && temp_bomb_count < MAX_BOMBS; ++i) {
    if (i == bomb_idx) continue;
    if (bombs[i].row < 0 || bombs[i].col < 0) continue;
    temp_bombs[temp_bomb_count++] = bombs[i];
  }
  
  // 计算炸弹到每个障碍的距离
  int dist_to_obs[200];
  size_t valid_obs_count = 0;
  Point valid_obstacles[200];
  
  for (size_t i = 0; i < obstacle_count && valid_obs_count < 200; ++i) {
    Point obs = obstacles[i];
    // 排除该障碍计算距离
    Point filtered_obstacles[200];
    size_t filtered_count = 0;
    for (size_t j = 0; j < obstacle_count && filtered_count < 200; ++j) {
      if (obstacles[j].row != obs.row || obstacles[j].col != obs.col) {
        filtered_obstacles[filtered_count++] = obstacles[j];
      }
    }
    
    int d = planner_v3_bfs_distance_between(rows, cols, bomb_pos, obs,
                                            filtered_obstacles, filtered_count,
                                            temp_bombs, temp_bomb_count,
                                            boxes, box_count, 1);
    if (d != INT_MAX) {
      dist_to_obs[valid_obs_count] = d;
      valid_obstacles[valid_obs_count] = obs;
      valid_obs_count++;
    }
  }
  
  // 按距离升序排列
  for (size_t n = 0; n < valid_obs_count && *nearest_count < MAX_NEAREST_OBSTACLES; ++n) {
    int best_d = INT_MAX;
    size_t best_i = SIZE_MAX;
    for (size_t i = 0; i < valid_obs_count; ++i) {
      int already = 0;
      for (size_t k = 0; k < *nearest_count; ++k) {
        if (nearest_obstacles[k].row == valid_obstacles[i].row &&
            nearest_obstacles[k].col == valid_obstacles[i].col) {
          already = 1;
          break;
        }
      }
      if (already) continue;
      if (dist_to_obs[i] < best_d) {
        best_d = dist_to_obs[i];
        best_i = i;
      }
    }
    if (best_i == SIZE_MAX) break;
    nearest_obstacles[(*nearest_count)++] = valid_obstacles[best_i];
  }
}

// 检查炸弹推到目标位置时每一步的推位是否被障碍占据
// 返回值：如果某步推位被障碍占据，返回该障碍位置；否则返回 {-1, -1}
static Point planner_v3_bfs_check_bomb_path_push_obstacle(int rows, int cols,
                                                           Point bomb_start, Point target_pos,
                                                           const Point *obstacles, size_t obstacle_count,
                                                           const Point *bombs, size_t bomb_count, size_t bomb_idx,
                                                           const Point *boxes, size_t box_count) {
  Point result = {-1, -1};
  
  // 构建临时炸弹数组（排除当前炸弹）
  Point temp_bombs[MAX_BOMBS];
  size_t temp_bomb_count = 0;
  for (size_t i = 0; i < bomb_count && temp_bomb_count < MAX_BOMBS; ++i) {
    if (i == bomb_idx) continue;
    if (bombs[i].row < 0 || bombs[i].col < 0) continue;
    temp_bombs[temp_bomb_count++] = bombs[i];
  }
  
  // 排除目标障碍计算路径
  Point filtered_obstacles[200];
  size_t filtered_count = 0;
  for (size_t i = 0; i < obstacle_count && filtered_count < 200; ++i) {
    if (obstacles[i].row != target_pos.row || obstacles[i].col != target_pos.col) {
      filtered_obstacles[filtered_count++] = obstacles[i];
    }
  }
  
  // 计算炸弹路径
  int dist_map[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target_pos,
                                             filtered_obstacles, filtered_count,
                                             temp_bombs, temp_bomb_count,
                                             boxes, box_count,
                                             dist_map, 1)) {
    return result;
  }
  
  Point bomb_path[PLANNER_V3_BFS_MAX_PATH_LEN];
  size_t bomb_path_len = 0;
  if (!planner_v3_bfs_astar_with_dist(rows, cols, bomb_start, target_pos,
                                      filtered_obstacles, filtered_count,
                                      temp_bombs, temp_bomb_count,
                                      boxes, box_count,
                                      dist_map,
                                      bomb_path, PLANNER_V3_BFS_MAX_PATH_LEN, &bomb_path_len,
                                      1, 0, NULL,
                                      0, 0, (Point){0, 0})) {
    return result;
  }
  
  // 检查每一步的推位
  for (size_t i = 1; i < bomb_path_len; ++i) {
    Point from = bomb_path[i - 1];
    Point to = bomb_path[i];
    int dr = to.row - from.row;
    int dc = to.col - from.col;
    Point push_pos = {from.row - dr, from.col - dc};
    
    if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col)) {
      continue;  // 推位越界，但这不是障碍问题
    }
    
    // 检查推位是否是障碍（不排除目标障碍）
    for (size_t oi = 0; oi < obstacle_count; ++oi) {
      if (obstacles[oi].row == push_pos.row && obstacles[oi].col == push_pos.col) {
        // 如果推位障碍就是目标障碍本身或其相邻障碍，则可以接受
        if (push_pos.row == target_pos.row && push_pos.col == target_pos.col) {
          continue;
        }
        // 检查是否是目标障碍的相邻障碍
        int is_adjacent = 0;
        const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        for (int d = 0; d < 4; ++d) {
          if (push_pos.row == target_pos.row + dirs[d][0] &&
              push_pos.col == target_pos.col + dirs[d][1]) {
            is_adjacent = 1;
            break;
          }
        }
        if (is_adjacent) {
          continue;
        }
        // 推位被其他障碍占据
        result = push_pos;
        return result;
      }
    }
  }
  
  return result;
}

// 递归处理炸弹推位被障碍占据的情况
// 返回值：1=成功清除障碍，0=失败
// 逻辑：
// - 优先选取距离炸弹A（bomb_idx）最近的炸弹B，尝试用炸弹B去炸掉推位上的障碍
// - 每次 planner_v3_bfs_push_bomb 前保存状态，失败则回滚并尝试下一个候选
// - 如果炸弹B的推位也被障碍占据，递归处理
// - extra_exploded_out/extra_cap/extra_count_out：可选，用于主菜单显示炸弹B等爆炸位置
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
    return 0;  // 防止无限递归
  }

  Point bomb_a = bombs[bomb_idx];
  size_t bomb_b_order[MAX_BOMBS];
  size_t bomb_b_count = 0;
  int bomb_b_dist_to_a[MAX_BOMBS];  // 炸弹B到炸弹A的BFS距离

  for (size_t bi = 0; bi < bomb_count && bomb_b_count < MAX_BOMBS; ++bi) {
    if (bi == bomb_idx) continue;
    if (bombs[bi].row < 0 || bombs[bi].col < 0) continue;
    bomb_b_order[bomb_b_count] = bi;
    /* 计算炸弹B到炸弹A的距离时，排除炸弹A（目标点）和炸弹B（起点） */
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
        filtered_obstacles, nf, temp_bombs, tb, boxes, box_count, 1);
    bomb_b_dist_to_a[bomb_b_count] = (d == INT_MAX) ? INT_MAX : d;
    bomb_b_count++;
  }

  /* 按炸弹B到炸弹A的距离升序排列，优先尝试距A最近的炸弹B */
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

    // 构建目标列表：推位障碍本身 + 其上下左右的障碍
    const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    Point candidate_targets[5];
    int candidate_dist[5];
    size_t candidate_count = 0;

    candidate_targets[candidate_count++] = obstacle_on_push_pos;
    for (int d = 0; d < 4; ++d) {
      int nr = obstacle_on_push_pos.row + dirs[d][0];
      int nc = obstacle_on_push_pos.col + dirs[d][1];
      if (planner_v3_bfs_in_bounds(rows, cols, nr, nc)) {
        if (planner_v3_bfs_is_obstacle_no_bomb(obstacles, *obstacle_count, nr, nc)) {
          candidate_targets[candidate_count].row = nr;
          candidate_targets[candidate_count].col = nc;
          candidate_count++;
          if (candidate_count >= 5) break;
        }
      }
    }

    // 计算炸弹B到每个候选障碍的BFS距离，并按距离升序排序
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
          filtered_obstacles, nf, temp_bombs, temp_bomb_count, boxes, box_count, 1);
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

      Point bomb_b_push_obs = planner_v3_bfs_check_bomb_path_push_obstacle(
          rows, cols, bombs[bi], cand_target,
          obstacles, *obstacle_count, bombs, bomb_count, bi, boxes, box_count);

      if (bomb_b_push_obs.row < 0) {
        // 炸弹B的推位没有障碍，可以直接推；每次 push_bomb 前保存状态
        Point saved_obstacles[200];
        size_t saved_obstacle_count = *obstacle_count;
        memcpy(saved_obstacles, obstacles, saved_obstacle_count * sizeof(Point));
        Point saved_bombs[MAX_BOMBS];
        memcpy(saved_bombs, bombs, bomb_count * sizeof(Point));
        Point saved_car = *car_pos;
        size_t saved_steps = *out_steps;
        size_t save_len = saved_steps < PUSH_BOMB_SAVE_PATH_MAX ? saved_steps : PUSH_BOMB_SAVE_PATH_MAX;
        Point saved_path[PUSH_BOMB_SAVE_PATH_MAX];
        memcpy(saved_path, path_buffer, save_len * sizeof(Point));

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
        /* 失败回滚 */
        memcpy(obstacles, saved_obstacles, saved_obstacle_count * sizeof(Point));
        *obstacle_count = saved_obstacle_count;
        memcpy(bombs, saved_bombs, bomb_count * sizeof(Point));
        *car_pos = saved_car;
        *out_steps = saved_steps;
        memcpy(path_buffer, saved_path, save_len * sizeof(Point));
      } else {
        // 炸弹B的推位也被障碍占据，递归前保存状态（递归成功但后续 push_bomb 失败时需回滚）
        Point saved_obstacles[200];
        size_t saved_obstacle_count = *obstacle_count;
        memcpy(saved_obstacles, obstacles, saved_obstacle_count * sizeof(Point));
        Point saved_bombs[MAX_BOMBS];
        memcpy(saved_bombs, bombs, bomb_count * sizeof(Point));
        Point saved_car = *car_pos;
        size_t saved_steps = *out_steps;
        size_t save_len = saved_steps < PUSH_BOMB_SAVE_PATH_MAX ? saved_steps : PUSH_BOMB_SAVE_PATH_MAX;
        Point saved_path[PUSH_BOMB_SAVE_PATH_MAX];
        memcpy(saved_path, path_buffer, save_len * sizeof(Point));

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
          /* push 失败，回滚到递归前，尝试下一个候选 */
          memcpy(obstacles, saved_obstacles, saved_obstacle_count * sizeof(Point));
          *obstacle_count = saved_obstacle_count;
          memcpy(bombs, saved_bombs, bomb_count * sizeof(Point));
          *car_pos = saved_car;
          *out_steps = saved_steps;
          memcpy(path_buffer, saved_path, save_len * sizeof(Point));
        }
      }
    }
  }

  return 0;  // 没有其他炸弹可以清除推位障碍
}

// 处理特殊路径上的炸弹（无障碍但有炸弹的情况）
// 目标：将炸弹A推离路径即可（可推到障碍或空地）；推离方向优先级：
// 1) 炸弹A最终位置不是箱子、不是其他炸弹；2) 推位合理（无障碍或可清除）；3) 推位被挡时优先炸弹B到该障碍路径最短的
// special_path/special_path_len：特殊路径，用于判断“推离路径”（最终位置不在路径上或推到障碍）
// extra_exploded_out/extra_cap/extra_count_out：可选，用于主菜单显示炸弹B等清除推位时的爆炸位置
// 返回值：1=成功处理，0=失败
#define PLANNER_V3_BFS_MAX_OFF_PATH_CANDIDATES 4
static int planner_v3_bfs_handle_bomb_on_path(
    int rows, int cols, Point *car_pos,
    Point *bombs, size_t bomb_count, size_t bomb_on_path_idx,
    Point *obstacles, size_t *obstacle_count,
    const Point *boxes, size_t box_count,
    const Point *special_path, size_t special_path_len,
    Point *path_buffer, size_t path_capacity, size_t *out_steps,
    Point *bomb_target_out,
    Point *extra_exploded_out, size_t extra_cap, size_t *extra_count_out) {
  
  Point bomb_pos = bombs[bomb_on_path_idx];
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

  typedef struct {
    Point target;
    int is_obstacle;   /* 1=推到障碍会爆炸，0=推到空地仅移动 */
    Point push_obs;    /* 推位被挡时的障碍，(-1,-1)表示推位无障碍 */
    int bomb_b_dist;   /* 推位被挡时，其他炸弹到该障碍的最短BFS距离，用于排序 */
  } OffPathCandidate;
  OffPathCandidate cands[PLANNER_V3_BFS_MAX_OFF_PATH_CANDIDATES];
  size_t cand_count = 0;

  for (int di = 0; di < 4 && cand_count < PLANNER_V3_BFS_MAX_OFF_PATH_CANDIDATES; ++di) {
    int nr = bomb_pos.row + dirs[di][0];
    int nc = bomb_pos.col + dirs[di][1];
    if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc)) continue;

    Point next = {nr, nc};
    if (planner_v3_bfs_is_box_at(boxes, box_count, nr, nc, SIZE_MAX)) continue;
    for (size_t bi = 0; bi < bomb_count; ++bi) {
      if (bi == bomb_on_path_idx) continue;
      if (bombs[bi].row < 0 || bombs[bi].col < 0) continue;
      if (bombs[bi].row == nr && bombs[bi].col == nc) goto skip_dir;
    }
    if (planner_v3_bfs_is_obstacle_no_bomb(obstacles, *obstacle_count, nr, nc)) {
      cands[cand_count].target = next;
      cands[cand_count].is_obstacle = 1;
      cand_count++;
      continue;
    }
    if (planner_v3_bfs_point_on_path(next, special_path, special_path_len)) continue;
    cands[cand_count].target = next;
    cands[cand_count].is_obstacle = 0;
    cand_count++;
skip_dir:;
  }

  if (cand_count == 0) {
    return 0;
  }

  Point bomb_a = bombs[bomb_on_path_idx];
  for (size_t ci = 0; ci < cand_count; ++ci) {
    cands[ci].push_obs = planner_v3_bfs_check_bomb_path_push_obstacle(
        rows, cols, bomb_pos, cands[ci].target,
        obstacles, *obstacle_count, bombs, bomb_count, bomb_on_path_idx, boxes, box_count);
    cands[ci].bomb_b_dist = INT_MAX;
    if (cands[ci].push_obs.row >= 0) {
      /* 优先选取距离炸弹A最近的炸弹B，再根据炸弹B到推位障碍的距离排序推离方向 */
      Point filtered_obstacles[200];
      size_t nf = 0;
      for (size_t i = 0; i < *obstacle_count && nf < 200; ++i) {
        if (obstacles[i].row == cands[ci].push_obs.row && obstacles[i].col == cands[ci].push_obs.col) continue;
        filtered_obstacles[nf++] = obstacles[i];
      }
      int best_b_dist_to_obs = INT_MAX;
      int best_b_dist_to_a = INT_MAX;
      for (size_t bi = 0; bi < bomb_count; ++bi) {
        if (bi == bomb_on_path_idx) continue;
        if (bombs[bi].row < 0 || bombs[bi].col < 0) continue;
        Point temp_bombs[MAX_BOMBS];
        size_t tb = 0;
        for (size_t bj = 0; bj < bomb_count && tb < MAX_BOMBS; ++bj) {
          if (bj == bi || bj == bomb_on_path_idx) continue;
          if (bombs[bj].row < 0 || bombs[bj].col < 0) continue;
          temp_bombs[tb++] = bombs[bj];
        }
        int d_to_obs = planner_v3_bfs_distance_between(rows, cols, bombs[bi], cands[ci].push_obs,
            filtered_obstacles, nf, temp_bombs, tb, boxes, box_count, 1);
        if (d_to_obs == INT_MAX) continue;
        int d_to_a = planner_v3_bfs_distance_between(rows, cols, bombs[bi], bomb_a,
            filtered_obstacles, nf, temp_bombs, tb, boxes, box_count, 1);
        /* 优先距A近的炸弹B，其次该B到推位障碍更近的 */
        if (d_to_a < best_b_dist_to_a || (d_to_a == best_b_dist_to_a && d_to_obs < best_b_dist_to_obs)) {
          best_b_dist_to_a = d_to_a;
          best_b_dist_to_obs = d_to_obs;
        }
      }
      cands[ci].bomb_b_dist = best_b_dist_to_obs;
    }
  }

  /* 排序：优先推位无障碍，其次按「距A最近的炸弹B」到推位障碍的距离升序 */
  for (size_t i = 0; i < cand_count; ++i) {
    for (size_t j = i + 1; j < cand_count; ++j) {
      int j_better = 0;
      if (cands[j].push_obs.row < 0 && cands[i].push_obs.row >= 0) j_better = 1;
      else if (cands[i].push_obs.row < 0 && cands[j].push_obs.row >= 0) j_better = 0;
      else if (cands[j].bomb_b_dist < cands[i].bomb_b_dist) j_better = 1;
      if (j_better) {
        OffPathCandidate t = cands[i];
        cands[i] = cands[j];
        cands[j] = t;
      }
    }
  }

  for (size_t ti = 0; ti < cand_count; ++ti) {
    Point target_obs = cands[ti].target;
    Point push_obs = cands[ti].push_obs;
    int is_obstacle = cands[ti].is_obstacle;
    /* 每次尝试前保存状态，失败则回滚 */
    Point saved_obstacles[200];
    size_t saved_obstacle_count = *obstacle_count;
    memcpy(saved_obstacles, obstacles, saved_obstacle_count * sizeof(Point));
    Point saved_bombs[MAX_BOMBS];
    memcpy(saved_bombs, bombs, bomb_count * sizeof(Point));
    Point saved_car = *car_pos;
    size_t saved_steps = *out_steps;
    size_t save_len = saved_steps < PUSH_BOMB_SAVE_PATH_MAX ? saved_steps : PUSH_BOMB_SAVE_PATH_MAX;
    Point saved_path[PUSH_BOMB_SAVE_PATH_MAX];
    memcpy(saved_path, path_buffer, save_len * sizeof(Point));

    if (push_obs.row >= 0) {
      if (!planner_v3_bfs_clear_push_pos_obstacle_recursive(
              rows, cols, car_pos, bombs, bomb_count, bomb_on_path_idx,
              push_obs, obstacles, obstacle_count, boxes, box_count,
              path_buffer, path_capacity, out_steps, 0,
              extra_exploded_out, extra_cap, extra_count_out)) {
        memcpy(obstacles, saved_obstacles, saved_obstacle_count * sizeof(Point));
        *obstacle_count = saved_obstacle_count;
        memcpy(bombs, saved_bombs, bomb_count * sizeof(Point));
        *car_pos = saved_car;
        *out_steps = saved_steps;
        memcpy(path_buffer, saved_path, save_len * sizeof(Point));
        continue;
      }
    }

    /* push 炸弹A 前已保存，失败则回滚 */
    int push_result = planner_v3_bfs_push_bomb(rows, cols, car_pos,
        bombs, bomb_count, bomb_on_path_idx, target_obs,
        target_obs,
        obstacles, *obstacle_count,
        boxes, box_count,
        path_buffer, path_capacity, out_steps,
        NULL, 0);
    if (push_result == 1) {
      if (is_obstacle) {
        Point new_obstacles[200];
        size_t new_obstacle_count = planner_v3_bfs_bomb_explode(obstacles, *obstacle_count,
                                                                 target_obs, new_obstacles, 200);
        memcpy(obstacles, new_obstacles, new_obstacle_count * sizeof(Point));
        *obstacle_count = new_obstacle_count;
        bombs[bomb_on_path_idx].row = -1;
        bombs[bomb_on_path_idx].col = -1;
      }
      if (bomb_target_out) *bomb_target_out = target_obs;
      return 1;
    }

    /* 失败回滚，尝试下一个候选 */
    memcpy(obstacles, saved_obstacles, saved_obstacle_count * sizeof(Point));
    *obstacle_count = saved_obstacle_count;
    memcpy(bombs, saved_bombs, bomb_count * sizeof(Point));
    *car_pos = saved_car;
    *out_steps = saved_steps;
    memcpy(path_buffer, saved_path, save_len * sizeof(Point));
  }
  return 0;
}

// 按"炸弹到特殊路径经过的障碍"的BFS距离升序排列炸弹索引，供推炸弹环节依次尝试
// 多障碍时排除 path_obstacles 中全部障碍；否则仅排除 obstacle_to_clear
// 输出：bomb_order[0..*bomb_order_count-1] 为炸弹索引，最近障碍的炸弹在前
static void planner_v3_bfs_bombs_sorted_by_dist_to_obstacle(int rows, int cols,
    Point obstacle_to_clear,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count,
    size_t *bomb_order, size_t *bomb_order_count,
    const Point *path_obstacles, size_t path_obstacle_count) {
  *bomb_order_count = 0;
  if (bomb_count == 0 || !bombs) {
    return;
  }

  Point filtered_obstacles[200];
  size_t filtered_obstacle_count = 0;
  for (size_t i = 0; i < obstacle_count && filtered_obstacle_count < 200; ++i) {
    int exclude = 0;
    if (path_obstacles && path_obstacle_count > 0) {
      for (size_t j = 0; j < path_obstacle_count; ++j) {
        if (obstacles[i].row == path_obstacles[j].row &&
            obstacles[i].col == path_obstacles[j].col) {
          exclude = 1;
          break;
        }
      }
    } else if (obstacles[i].row == obstacle_to_clear.row &&
               obstacles[i].col == obstacle_to_clear.col) {
      exclude = 1;
    }
    if (!exclude) {
      filtered_obstacles[filtered_obstacle_count++] = obstacles[i];
    }
  }

  int dist_to_obs[MAX_BOMBS];
  for (size_t i = 0; i < bomb_count && i < MAX_BOMBS; ++i) {
    dist_to_obs[i] = INT_MAX;
    if (bombs[i].row < 0 || bombs[i].col < 0) {
      continue;
    }
    Point temp_bombs[MAX_BOMBS];
    size_t temp_bomb_count = 0;
    for (size_t j = 0; j < bomb_count && temp_bomb_count < MAX_BOMBS; ++j) {
      if (j == i) continue;
      if (bombs[j].row < 0 || bombs[j].col < 0) continue;
      temp_bombs[temp_bomb_count++] = bombs[j];
    }
    int d = planner_v3_bfs_distance_between(rows, cols, bombs[i], obstacle_to_clear,
        filtered_obstacles, filtered_obstacle_count,
        temp_bombs, temp_bomb_count,
        boxes, box_count, 1);
    if (d != INT_MAX) {
      dist_to_obs[i] = d;
    }
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

// 备选目标 = 特殊路径经过的障碍 + 其上下左右的障碍；按该炸弹到目标的BFS可达距离升序排列
// 多障碍时排除 path_obstacles 中全部障碍，否则仅排除候选点
// 预筛选逻辑：BFS+A*能得到路径，且每一步推位都可达；输出 target_positions[0..*target_count-1]
#define PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS 5
static void planner_v3_bfs_targets_sorted_by_reachable_dist(int rows, int cols, Point car,
    size_t bomb_idx, const Point *bombs, size_t bomb_count,
    Point obstacle_to_clear,
    const Point *obstacles, size_t obstacle_count,
    const Point *boxes, size_t box_count,
    Point *target_positions, size_t *target_count,
    const Point *path_obstacles, size_t path_obstacle_count) {
  *target_count = 0;
  Point temp_bombs[MAX_BOMBS];
  size_t temp_bomb_count = 0;
  for (size_t i = 0; i < bomb_count && temp_bomb_count < MAX_BOMBS; ++i) {
    if (i == bomb_idx) continue;
    if (bombs[i].row < 0 || bombs[i].col < 0) continue;
    temp_bombs[temp_bomb_count++] = bombs[i];
  }

  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  Point candidates[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
  int cand_dist[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
  size_t cand_count = 0;

  // 对每个候选目标：排除该候选点做 BFS+A* 得到路径，并校验每一步推位可达
  Point candidate_points[5];
  size_t num_candidate_points = 0;
  candidate_points[num_candidate_points++] = obstacle_to_clear;
  for (int di = 0; di < 4; ++di) {
    int nr = obstacle_to_clear.row + dirs[di][0];
    int nc = obstacle_to_clear.col + dirs[di][1];
    if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc)) continue;
    if (!planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, nr, nc)) continue;
    candidate_points[num_candidate_points++] = (Point){nr, nc};
    if (num_candidate_points >= 5) break;
  }

  for (size_t ci = 0; ci < num_candidate_points && cand_count < PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS; ++ci) {
    Point cand = candidate_points[ci];
    Point filtered_for_cand[200];
    size_t nf = 0;
    for (size_t i = 0; i < obstacle_count && nf < 200; ++i) {
      int exclude = 0;
      if (obstacles[i].row == cand.row && obstacles[i].col == cand.col) {
        exclude = 1;  /* 排除候选点本身，炸弹才能到达 */
      } else if (path_obstacles && path_obstacle_count > 0) {
        for (size_t j = 0; j < path_obstacle_count; ++j) {
          if (obstacles[i].row == path_obstacles[j].row &&
              obstacles[i].col == path_obstacles[j].col) {
            exclude = 1;
            break;
          }
        }
      }
      if (!exclude) {
        filtered_for_cand[nf++] = obstacles[i];
      }
    }

    int dist_map[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_target(rows, cols, cand,
            filtered_for_cand, nf, temp_bombs, temp_bomb_count,
            boxes, box_count, dist_map, 1)) {
      continue;
    }
    Point bomb_path[PLANNER_V3_BFS_MAX_PATH_LEN];
    size_t bomb_path_len = 0;
    /* A* 内已做推位检测，推位不合理的节点会被排除 */
    if (!planner_v3_bfs_astar_with_dist(rows, cols, bombs[bomb_idx], cand,
            filtered_for_cand, nf, temp_bombs, temp_bomb_count,
            boxes, box_count, dist_map,
            bomb_path, PLANNER_V3_BFS_MAX_PATH_LEN, &bomb_path_len,
            1, 0, NULL,
            1, 1, car)) {
      continue;
    }

    int d = planner_v3_bfs_distance_between(rows, cols, bombs[bomb_idx], cand,
        filtered_for_cand, nf, temp_bombs, temp_bomb_count, boxes, box_count, 1);
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

// 模拟推动炸弹（用于计算步数，不实际修改状态）
// obstacles_for_car: 车到推位时的障碍（炸弹未到目标时应含目标点障碍，传完整 obstacles）
// 返回值：成功返回步数，失败返回INT_MAX
static int planner_v3_bfs_simulate_push_bomb(int rows, int cols, Point car_start,
                                             Point bomb_start, Point target_pos,
                                             Point target_obstacle,
                                             const Point *filtered_obstacles, size_t filtered_obstacle_count,
                                             const Point *obstacles_for_car, size_t obstacle_count_for_car,
                                             const Point *temp_bombs, size_t temp_bomb_count,
                                             const Point *temp_boxes, size_t temp_box_count,
                                             const Point *bomb_path, size_t bomb_path_len,
                                             int has_bomb_path, int strategy) {
  Point car = car_start;
  Point bomb = bomb_start;
  size_t steps = 0;
  size_t bomb_path_idx = 0;
  int last_dr = 0;
  int last_dc = 0;
  int reverse_count = 0;
  
  typedef struct {
    int dr;
    int dc;
    int dist;
    int adj_pen;
    int score;
    int feasible;
    Point push_from;
    Point bomb_next;
  } DirCandidate;
  
  while (bomb.row != target_pos.row || bomb.col != target_pos.col) {
    if (steps++ >= PLANNER_V3_BFS_MAX_GREEDY_STEPS) {
      return INT_MAX;
    }
    
    if (strategy == PLANNER_V3_BFS_STRATEGY_PATH && has_bomb_path) {
      // 路径策略
      if (bomb_path_idx + 1 >= bomb_path_len) {
        return INT_MAX;
      }
      
      Point next_in_path = bomb_path[bomb_path_idx + 1];
      int dr = next_in_path.row - bomb.row;
      int dc = next_in_path.col - bomb.col;
      
      if (planner_v3_bfs_abs(dr) + planner_v3_bfs_abs(dc) != 1) {
        return INT_MAX;
      }
      
      int push_row = bomb.row - dr;
      int push_col = bomb.col - dc;
      
      if (!planner_v3_bfs_in_bounds(rows, cols, next_in_path.row, next_in_path.col) ||
          !planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
        return INT_MAX;
      }
      
      // 检查障碍（只排除 target_pos，炸弹和箱子仍当作障碍）
      if (planner_v3_bfs_is_obstacle(filtered_obstacles, filtered_obstacle_count, temp_bombs, temp_bomb_count, 
                                    next_in_path.row, next_in_path.col) ||
          planner_v3_bfs_is_obstacle(filtered_obstacles, filtered_obstacle_count, temp_bombs, temp_bomb_count, 
                                    push_row, push_col)) {
        return INT_MAX;
      }
      
      if (planner_v3_bfs_is_box_at(temp_boxes, temp_box_count, next_in_path.row, next_in_path.col, SIZE_MAX) ||
          planner_v3_bfs_is_box_at(temp_boxes, temp_box_count, push_row, push_col, SIZE_MAX)) {
        return INT_MAX;
      }
      
      Point push_from = {push_row, push_col};
      // 车到推位距离需考虑当前炸弹（否则会误判为可穿过炸弹格）；炸弹未到目标时考虑目标点障碍
      Point bombs_with_current[MAX_BOMBS];
      size_t bwc = 0;
      for (size_t ii = 0; ii < temp_bomb_count && bwc < MAX_BOMBS; ii++) bombs_with_current[bwc++] = temp_bombs[ii];
      if (bwc < MAX_BOMBS) bombs_with_current[bwc++] = bomb;
      int car_dist = planner_v3_bfs_distance_between(rows, cols, car, push_from,
                                                      obstacles_for_car, obstacle_count_for_car,
                                                      bombs_with_current, bwc,
                                                      temp_boxes, temp_box_count, 1);
      if (car_dist == INT_MAX) {
        return INT_MAX;
      }
      
      steps += (size_t)car_dist;
      car = push_from;
      bomb = next_in_path;
      bomb_path_idx++;
      last_dr = dr;
      last_dc = dc;
      continue;
    }
    
    // 评分策略
    DirCandidate candidates[4];
    const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    
    for (int i = 0; i < 4; ++i) {
      candidates[i].dr = dirs[i][0];
      candidates[i].dc = dirs[i][1];
      candidates[i].feasible = 0;
      candidates[i].score = INT_MAX;
      
      int new_bomb_row = bomb.row + dirs[i][0];
      int new_bomb_col = bomb.col + dirs[i][1];
      int push_row = bomb.row - dirs[i][0];
      int push_col = bomb.col - dirs[i][1];
      
      if (!planner_v3_bfs_in_bounds(rows, cols, new_bomb_row, new_bomb_col) ||
          !planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
        continue;
      }
      
      if (planner_v3_bfs_is_obstacle(filtered_obstacles, filtered_obstacle_count, temp_bombs, temp_bomb_count, 
                                     new_bomb_row, new_bomb_col) ||
          planner_v3_bfs_is_obstacle(filtered_obstacles, filtered_obstacle_count, temp_bombs, temp_bomb_count, 
                                     push_row, push_col)) {
        continue;
      }
      
      if (planner_v3_bfs_is_box_at(temp_boxes, temp_box_count, new_bomb_row, new_bomb_col, SIZE_MAX) ||
          planner_v3_bfs_is_box_at(temp_boxes, temp_box_count, push_row, push_col, SIZE_MAX)) {
        continue;
      }
      
      Point push_from = {push_row, push_col};
      
      // 计算炸弹到目标的距离（不把当前炸弹当作障碍）
      int dist_after = planner_v3_bfs_distance_between(rows, cols, (Point){new_bomb_row, new_bomb_col}, target_pos,
                                                       filtered_obstacles, filtered_obstacle_count,
                                                       temp_bombs, temp_bomb_count,
                                                       temp_boxes, temp_box_count, 1);
      if (dist_after == INT_MAX) {
        continue;
      }
      
      int adj_pen = planner_v3_bfs_adjacent_blockers(rows, cols, filtered_obstacles, filtered_obstacle_count,
                                                      temp_bombs, temp_bomb_count, temp_boxes, temp_box_count,
                                                      SIZE_MAX, new_bomb_row, new_bomb_col);
      
      int reverse_pen = 0;
      if (last_dr == -dirs[i][0] && last_dc == -dirs[i][1]) {
        int power = 5;
        for (int p = 0; p < reverse_count; ++p) {
          power *= 5;
        }
        reverse_pen = power;
      }
      
      // 车到推位距离需考虑当前炸弹；炸弹未到目标时考虑目标点障碍
      Point bombs_with_current[MAX_BOMBS];
      size_t bwc = 0;
      for (size_t ii = 0; ii < temp_bomb_count && bwc < MAX_BOMBS; ii++) bombs_with_current[bwc++] = temp_bombs[ii];
      if (bwc < MAX_BOMBS) bombs_with_current[bwc++] = bomb;
      int car_to_push = planner_v3_bfs_distance_between(rows, cols, car, push_from,
                                                        obstacles_for_car, obstacle_count_for_car,
                                                        bombs_with_current, bwc,
                                                        temp_boxes, temp_box_count, 1);
      if (car_to_push == INT_MAX) {
        continue;
      }
      
      int score = dist_after * 13 + adj_pen * 2 + reverse_pen + car_to_push * 5;
      candidates[i].dist = dist_after;
      candidates[i].adj_pen = adj_pen;
      candidates[i].score = score;
      candidates[i].feasible = 1;
      candidates[i].push_from = push_from;
      candidates[i].bomb_next.row = new_bomb_row;
      candidates[i].bomb_next.col = new_bomb_col;
    }
    
    int sorted_dirs[4];
    int sorted_count = 0;
    for (int i = 0; i < 4; ++i) {
      if (candidates[i].feasible) {
        sorted_dirs[sorted_count++] = i;
      }
    }
    
    if (sorted_count == 0) {
      return INT_MAX;
    }
    
    // 排序（按评分从优到劣）
    for (int i = 0; i < sorted_count - 1; ++i) {
      for (int j = 0; j < sorted_count - 1 - i; ++j) {
        if (candidates[sorted_dirs[j]].score > candidates[sorted_dirs[j + 1]].score) {
          int tmp = sorted_dirs[j];
          sorted_dirs[j] = sorted_dirs[j + 1];
          sorted_dirs[j + 1] = tmp;
        }
      }
    }
    
    // 按优劣依次尝试各方向，直到有一个可行
    DirCandidate chosen;
    int car_dist = INT_MAX;
    for (int k = 0; k < sorted_count; ++k) {
      chosen = candidates[sorted_dirs[k]];
      int next_on_boundary = (chosen.bomb_next.row == 0 || chosen.bomb_next.row == rows - 1 ||
                              chosen.bomb_next.col == 0 || chosen.bomb_next.col == cols - 1);
      if (next_on_boundary) {
        if (!planner_v3_bfs_validate_boundary_path(rows, cols, chosen.bomb_next, target_pos,
                                                   car, filtered_obstacles, filtered_obstacle_count,
                                                   temp_bombs, temp_bomb_count, temp_boxes, temp_box_count,
                                                   SIZE_MAX, 1)) {
          continue;  /* 推到边界后不可达目标，尝试次优方向 */
        }
      }
      // 车到推位距离需考虑当前炸弹；炸弹未到目标时考虑目标点障碍
      {
        Point bombs_with_current[MAX_BOMBS];
        size_t bwc = 0;
        for (size_t ii = 0; ii < temp_bomb_count && bwc < MAX_BOMBS; ii++) bombs_with_current[bwc++] = temp_bombs[ii];
        if (bwc < MAX_BOMBS) bombs_with_current[bwc++] = bomb;
        car_dist = planner_v3_bfs_distance_between(rows, cols, car, chosen.push_from,
                                                   obstacles_for_car, obstacle_count_for_car,
                                                   bombs_with_current, bwc,
                                                   temp_boxes, temp_box_count, 1);
      }
      if (car_dist == INT_MAX) {
        continue;
      }
      break;
    }
    if (car_dist == INT_MAX) {
      return INT_MAX;
    }
    
    steps += (size_t)car_dist;
    car = chosen.push_from;
    bomb = chosen.bomb_next;
    
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

// 模拟按评分策略推箱子到目标（仅计算步数，不修改状态）
// 返回值：成功返回步数，失败返回 INT_MAX
static int planner_v3_bfs_simulate_push_box_score(
    int rows, int cols, Point car_start, Point box_start, Point target,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count, size_t box_idx) {
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
  
  typedef struct { int dr, dc; int feasible; int score; Point push_from; Point box_next; } DirCand;
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  
  while (box.row != target.row || box.col != target.col) {
    if (steps >= PLANNER_V3_BFS_MAX_GREEDY_STEPS) return INT_MAX;
    
    DirCand candidates[4];
    for (int i = 0; i < 4; ++i) {
      candidates[i].feasible = 0;
      candidates[i].score = INT_MAX;
      int nr = box.row + dirs[i][0], nc = box.col + dirs[i][1];
      int pr = box.row - dirs[i][0], pc = box.col - dirs[i][1];
      if (!planner_v3_bfs_in_bounds(rows, cols, nr, nc) ||
          !planner_v3_bfs_in_bounds(rows, cols, pr, pc)) continue;
      if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, nr, nc) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, pr, pc)) continue;
      if (planner_v3_bfs_is_box_at(boxes_local, n, nr, nc, box_idx) ||
          planner_v3_bfs_is_box_at(boxes_local, n, pr, pc, box_idx)) continue;
      int next_on_boundary = (nr == 0 || nr == rows - 1 || nc == 0 || nc == cols - 1);
      if (next_on_boundary) {
        if (!planner_v3_bfs_validate_boundary_path(rows, cols, (Point){nr, nc}, target,
            car, obstacles, obstacle_count, bombs, bomb_count, boxes_local, n, box_idx, 1)) continue;
      }
      if (nr != target.row || nc != target.col) {
        if (!planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count, bombs, bomb_count,
            boxes_local, n, box_idx, (Point){nr, nc}, target, 1)) continue;
      }
      int dist = planner_v3_bfs_distance_between(rows, cols, (Point){nr, nc}, target,
          obstacles, obstacle_count, bombs, bomb_count, temp_boxes, temp_count, 1);
      if (dist == INT_MAX) continue;
      int adj = planner_v3_bfs_adjacent_blockers(rows, cols, obstacles, obstacle_count, bombs, bomb_count,
          boxes_local, n, box_idx, nr, nc);
      int rev = 0;
      if (last_dr == -dirs[i][0] && last_dc == -dirs[i][1]) {
        int p = 5; for (int k = 0; k < reverse_count; ++k) p *= 5; rev = p;
      }
      int c2p = planner_v3_bfs_distance_between(rows, cols, car, (Point){pr, pc},
          obstacles, obstacle_count, bombs, bomb_count, boxes_local, n, 1);
      if (c2p == INT_MAX) continue;
      candidates[i].dr = dirs[i][0]; candidates[i].dc = dirs[i][1];
      candidates[i].score = dist * 13 + adj * 2 + rev + c2p * 5;
      candidates[i].feasible = 1;
      candidates[i].push_from = (Point){pr, pc};
      candidates[i].box_next = (Point){nr, nc};
    }
    
    int order[4], cnt = 0;
    for (int i = 0; i < 4; ++i) if (candidates[i].feasible) order[cnt++] = i;
    if (cnt == 0) return INT_MAX;
    for (int i = 0; i < cnt - 1; ++i)
      for (int j = 0; j < cnt - 1 - i; ++j)
        if (candidates[order[j]].score > candidates[order[j + 1]].score) {
          int t = order[j]; order[j] = order[j + 1]; order[j + 1] = t;
        }
    
    DirCand chosen;
    int car_dist = INT_MAX;
    for (int k = 0; k < cnt; ++k) {
      chosen = candidates[order[k]];
      int next_on_boundary = (chosen.box_next.row == 0 || chosen.box_next.row == rows - 1 ||
                              chosen.box_next.col == 0 || chosen.box_next.col == cols - 1);
      if (next_on_boundary) {
        if (!planner_v3_bfs_validate_boundary_path(rows, cols, chosen.box_next, target,
            car, obstacles, obstacle_count, bombs, bomb_count, boxes_local, n, box_idx, 1)) continue;
      }
      car_dist = planner_v3_bfs_distance_between(rows, cols, car, chosen.push_from,
          obstacles, obstacle_count, bombs, bomb_count, boxes_local, n, 1);
      if (car_dist == INT_MAX) continue;
      break;
    }
    if (car_dist == INT_MAX) return INT_MAX;
    
    steps += (size_t)car_dist + 1;
    car = box;
    box = chosen.box_next;
    boxes_local[box_idx] = box;
    if (last_dr == -chosen.dr && last_dc == -chosen.dc && (last_dr != 0 || last_dc != 0))
      reverse_count++;
    else reverse_count = 0;
    last_dr = chosen.dr; last_dc = chosen.dc;
  }
  return (int)steps;
}

// 模拟按路径策略推箱子到目标（仅计算步数；若路径中途需推炸弹则本模拟返回 INT_MAX）
// 返回值：成功返回步数，失败返回 INT_MAX
static int planner_v3_bfs_simulate_push_box_path(
    int rows, int cols, Point car_start, Point box_start, Point target,
    const Point *box_path, size_t box_path_len,
    const Point *obstacles, size_t obstacle_count,
    const Point *bombs, size_t bomb_count,
    const Point *boxes, size_t box_count, size_t box_idx) {
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
    
    Point next = box_path[path_idx + 1];
    int dr = next.row - box.row, dc = next.col - box.col;
    if (planner_v3_bfs_abs(dr) + planner_v3_bfs_abs(dc) != 1) return INT_MAX;
    int pr = box.row - dr, pc = box.col - dc;
    if (!planner_v3_bfs_in_bounds(rows, cols, next.row, next.col) ||
        !planner_v3_bfs_in_bounds(rows, cols, pr, pc)) return INT_MAX;
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, next.row, next.col) ||
        planner_v3_bfs_is_obstacle(obstacles, obstacle_count, bombs, bomb_count, pr, pc)) return INT_MAX;
    if (planner_v3_bfs_is_box_at(boxes_local, n, next.row, next.col, box_idx) ||
        planner_v3_bfs_is_box_at(boxes_local, n, pr, pc, box_idx)) return INT_MAX;
    
    int next_on_boundary = (next.row == 0 || next.row == rows - 1 || next.col == 0 || next.col == cols - 1);
    if (next_on_boundary) {
      if (!planner_v3_bfs_validate_boundary_path(rows, cols, next, target,
          car, obstacles, obstacle_count, bombs, bomb_count, boxes_local, n, box_idx, 1)) return INT_MAX;
    }
    if (next.row != target.row || next.col != target.col) {
      if (!planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count, bombs, bomb_count,
          boxes_local, n, box_idx, next, target, 1)) return INT_MAX; /* 路径需炸弹，本模拟视为失败 */
    }
    
    int car_dist = planner_v3_bfs_distance_between(rows, cols, car, (Point){pr, pc},
        obstacles, obstacle_count, bombs, bomb_count, boxes_local, n, 1);
    if (car_dist == INT_MAX) return INT_MAX;
    steps += (size_t)car_dist + 1;
    car = box;
    box = next;
    boxes_local[box_idx] = box;
    path_idx++;
  }
  return (int)steps;
}

// 推动炸弹到目标位置
// 推炸弹的逻辑基于推箱子的逻辑：
// - 先模拟按照计算的路径推，再模拟采用评分的方式进行推动，选取步数低的实现
// - 目标点是箱子的特殊路径里的障碍，或者其上下左右周围障碍的其中一个
// - 炸弹路径规划时：在推到过程中不把目标点的障碍当作障碍（以便炸弹能走到目标格）
// - 车到推位时：若炸弹尚未到达目标点，则车前往推位需考虑目标点障碍（不能穿目标格）
// - 要把炸弹和箱子当作障碍，但不把要推动的炸弹当作障碍
// - 其他推动的逻辑和方式都和推箱子一样
static int planner_v3_bfs_push_bomb(int rows, int cols, Point *car_pos,
                                    Point *bombs, size_t bomb_count, size_t bomb_idx,
                                    Point target_pos,
                                    Point target_obstacle,  // 目标障碍（需要排除）
                                    const Point *obstacles, size_t obstacle_count,
                                    const Point *boxes, size_t box_count,
                                    Point *path_buffer, size_t path_capacity,
                                    size_t *out_steps,
                                    const Point *path_obstacles_to_exclude, size_t path_obstacle_exclude_count) {
  Point bomb = bombs[bomb_idx];
  
  // 如果炸弹已经在目标位置，直接返回
  if (bomb.row == target_pos.row && bomb.col == target_pos.col) {
    return 1;
  }
  
  // 构建障碍列表：排除 target_pos 以及（若提供）所有 path_obstacles_to_exclude
  // 多障碍时需排除路径上全部障碍，炸弹才能通过到达目标
  Point filtered_obstacles[200];
  size_t filtered_obstacle_count = 0;
  for (size_t i = 0; i < obstacle_count && filtered_obstacle_count < 200; ++i) {
    int exclude = 0;
    if (obstacles[i].row == target_pos.row && obstacles[i].col == target_pos.col) {
      exclude = 1;
    } else if (path_obstacles_to_exclude && path_obstacle_exclude_count > 0) {
      for (size_t j = 0; j < path_obstacle_exclude_count; ++j) {
        if (obstacles[i].row == path_obstacles_to_exclude[j].row &&
            obstacles[i].col == path_obstacles_to_exclude[j].col) {
          exclude = 1;
          break;
        }
      }
    }
    if (!exclude) {
      filtered_obstacles[filtered_obstacle_count++] = obstacles[i];
    }
  }
  
  // 构建临时箱子数组（用于路径规划，排除已完成的箱子）
  Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
  size_t temp_box_count = 0;
  for (size_t i = 0; i < box_count && temp_box_count < PLANNER_V3_BFS_MAX_BOXES; ++i) {
    if (boxes[i].row < 0 || boxes[i].col < 0) {
      continue;  // 忽略已完成的箱子
    }
    temp_boxes[temp_box_count++] = boxes[i];
  }
  
  // 构建临时炸弹数组（用于路径规划，排除当前正在推动的炸弹，不把要推动的炸弹当作障碍）
  Point temp_bombs[MAX_BOMBS];
  size_t temp_bomb_count = 0;
  for (size_t i = 0; i < bomb_count && temp_bomb_count < MAX_BOMBS; ++i) {
    if (i == bomb_idx) {
      continue;  // 忽略当前正在推动的炸弹（不把它当作障碍）
    }
    if (bombs[i].row < 0 || bombs[i].col < 0) {
      continue;  // 忽略已使用的炸弹
    }
    temp_bombs[temp_bomb_count++] = bombs[i];
  }
  
  // 从目标点做全局BFS（只排除 target_pos，其他炸弹和箱子仍当作障碍）
  int bomb_path_dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target_pos,
                                             filtered_obstacles, filtered_obstacle_count,
                                             temp_bombs, temp_bomb_count,
                                             temp_boxes, temp_box_count,
                                             bomb_path_dist, 1)) {  // 1=把炸弹当作障碍
    last_err_stage = 3;    // 炸弹相关错误
    last_err_detail = 310; // v3: 炸弹路径规划全局BFS失败
    return -6;
  }
  
  // 使用A*计算炸弹路径（只排除 target_pos，其他炸弹和箱子仍当作障碍）
  Point bomb_path[PLANNER_V3_BFS_MAX_CELLS];
  size_t bomb_path_len = 0;
  int has_bomb_path = 0;
  /* A* 内已做推位检测，推位不合理的节点会被排除 */
  if (planner_v3_bfs_astar_with_dist(rows, cols, bomb, target_pos,
                                     filtered_obstacles, filtered_obstacle_count,
                                     temp_bombs, temp_bomb_count,
                                     temp_boxes, temp_box_count,
                                     bomb_path_dist,
                                     bomb_path, PLANNER_V3_BFS_MAX_CELLS, &bomb_path_len,
                                     1, 0, NULL,
                                     1, 1, *car_pos)) {  // 1=把炸弹当作障碍，0=不允许经过障碍
    if (bomb_path_len > 1) {
      has_bomb_path = 1;
    }
  }
  
  // 模拟两种策略，选择步数更低的
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
  
  // 选择步数更低的策略（或仅路径模式则强制用 BFS+A* 路径）
  int chosen_strategy = PLANNER_V3_BFS_STRATEGY_SCORE;
  if (planner_v3_push_only_bfs_astar_path) {
    if (!has_bomb_path || path_steps == INT_MAX) {
      last_err_stage = 3;
      last_err_detail = 328; // v3: 仅路径模式但炸弹无有效BFS+A*路径
      return -6;
    }
    chosen_strategy = PLANNER_V3_BFS_STRATEGY_PATH;
  } else {
    if (path_steps < score_steps) {
      chosen_strategy = PLANNER_V3_BFS_STRATEGY_PATH;
    }
    if (path_steps == INT_MAX && score_steps == INT_MAX) {
      last_err_stage = 3;    // 炸弹相关错误
      last_err_detail = 311; // v3: 炸弹推动两种策略都失败
      return -6;
    }
  }
  
  // 使用选定的策略实际推动炸弹
  size_t bomb_path_idx = 0;
  int last_dr = 0;
  int last_dc = 0;
  int reverse_count = 0;
  
  typedef struct {
    int dr;
    int dc;
    int dist;
    int adj_pen;
    int score;
    int feasible;
    Point push_from;
    Point bomb_next;
  } DirCandidate;
  
  while (bomb.row != target_pos.row || bomb.col != target_pos.col) {
    if (chosen_strategy == PLANNER_V3_BFS_STRATEGY_PATH && has_bomb_path) {
      // 路径策略
      if (bomb_path_idx + 1 >= bomb_path_len) {
        last_err_stage = 3;    // 炸弹相关错误
        last_err_detail = 312; // v3: 炸弹路径索引越界
        return -6;
      }
      
      Point next_in_path = bomb_path[bomb_path_idx + 1];
      int dr = next_in_path.row - bomb.row;
      int dc = next_in_path.col - bomb.col;
      
      if (planner_v3_bfs_abs(dr) + planner_v3_bfs_abs(dc) != 1) {
        last_err_stage = 3;    // 炸弹相关错误
        last_err_detail = 313; // v3: 炸弹路径出现非相邻跳跃
        return -6;
      }
      
      int push_row = bomb.row - dr;
      int push_col = bomb.col - dc;
      
      // 检查是否越界（边界外）
      if (!planner_v3_bfs_in_bounds(rows, cols, next_in_path.row, next_in_path.col) ||
          !planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
        last_err_stage = 3;    // 炸弹相关错误
        last_err_detail = 314; // v3: 炸弹或推位越界
        return -6;
      }
      
      // 检查下一步是否会把炸弹推到边界上（row=0, row=rows-1, col=0, col=cols-1）
      int next_on_boundary = (next_in_path.row == 0 || next_in_path.row == rows - 1 ||
                              next_in_path.col == 0 || next_in_path.col == cols - 1);
      
      if (next_on_boundary) {
        // 如果下一步会把炸弹推到边界上，需要检测是否可达目标点并且检验到达目标点的每一步的推位是否合理
        if (!planner_v3_bfs_validate_boundary_path(rows, cols, next_in_path, target_pos,
                                                   *car_pos, filtered_obstacles, filtered_obstacle_count,
                                                   temp_bombs, temp_bomb_count, temp_boxes, temp_box_count,
                                                   SIZE_MAX, 1)) {  // SIZE_MAX表示不是箱子，1=把炸弹当作障碍
          last_err_stage = 3;    // 炸弹相关错误
          last_err_detail = 327; // v3: 炸弹边界位置无法到达目标或推位不合理
          return -6;
        }
      }
      
      // 检查障碍（只排除 target_pos，其他炸弹和箱子仍当作障碍）
      if (planner_v3_bfs_is_obstacle(filtered_obstacles, filtered_obstacle_count, temp_bombs, temp_bomb_count, 
                                     next_in_path.row, next_in_path.col) ||
          planner_v3_bfs_is_obstacle(filtered_obstacles, filtered_obstacle_count, temp_bombs, temp_bomb_count, 
                                     push_row, push_col)) {
        last_err_stage = 3;    // 炸弹相关错误
        last_err_detail = 315; // v3: 炸弹或推位落在障碍/炸弹
        return -6;
      }
      
      if (planner_v3_bfs_is_box_at(boxes, box_count, next_in_path.row, next_in_path.col, SIZE_MAX) ||
          planner_v3_bfs_is_box_at(boxes, box_count, push_row, push_col, SIZE_MAX)) {
        last_err_stage = 3;    // 炸弹相关错误
        last_err_detail = 316; // v3: 炸弹或推位与其他箱子冲突
        return -6;
      }
      
      Point push_from = {push_row, push_col};
      if (car_pos->row != push_from.row || car_pos->col != push_from.col) {
        // 车到推位时需考虑所有障碍（含目标点障碍，因炸弹尚未到达）、箱子及所有炸弹（含当前炸弹）
        const Point *car_obs = obstacles;
        size_t car_obs_n = obstacle_count;
        int car_result = planner_v3_bfs_car_move_with_global_astar(rows, cols, car_pos, push_from,
                                                                   car_obs, car_obs_n,
                                                                   bombs, bomb_count,
                                                                   temp_boxes, temp_box_count,
                                                                   path_buffer, path_capacity, out_steps, 1);
        if (car_result == -7) {
          return -7;
        }
        if (car_result == 0) {
          last_err_stage = 3;    // 炸弹相关错误
          last_err_detail = 317; // v3: 车无法到达推位（路径策略）
          return -6;
        }
      }
      
      if (car_pos->row != push_from.row || car_pos->col != push_from.col) {
        last_err_stage = 3;    // 炸弹相关错误
        last_err_detail = 318; // v3: 车到推位后位置不一致（路径策略）
        return -6;
      }
      
      if (!planner_v3_bfs_check_adjacent(*car_pos, bomb)) {
        last_err_stage = 3;    // 炸弹相关错误
        last_err_detail = 319; // v3: 车与炸弹不相邻（路径策略）
        return -6;
      }
      
      Point bomb_old_pos = bomb;
      bomb = next_in_path;
      bombs[bomb_idx] = bomb;
      bomb_path_idx++;
      
      *car_pos = bomb_old_pos;
      if (*out_steps >= path_capacity) {
        return -7;
      }
      if (*out_steps > 0 &&
          !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], *car_pos)) {
        last_err_stage = 3;    // 炸弹相关错误
        last_err_detail = 320; // v3: 车路径不连续（路径策略）
        return -6;
      }
      path_buffer[(*out_steps)++] = *car_pos;
      
      last_dr = dr;
      last_dc = dc;
      continue;
    }
    
    // 评分策略
    DirCandidate candidates[4];
    const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    
    for (int i = 0; i < 4; ++i) {
      candidates[i].dr = dirs[i][0];
      candidates[i].dc = dirs[i][1];
      candidates[i].feasible = 0;
      candidates[i].score = INT_MAX;
      
      int new_bomb_row = bomb.row + dirs[i][0];
      int new_bomb_col = bomb.col + dirs[i][1];
      int push_row = bomb.row - dirs[i][0];
      int push_col = bomb.col - dirs[i][1];
      
      if (!planner_v3_bfs_in_bounds(rows, cols, new_bomb_row, new_bomb_col) ||
          !planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
        continue;
      }
      
      if (planner_v3_bfs_is_obstacle(filtered_obstacles, filtered_obstacle_count, temp_bombs, temp_bomb_count, 
                                     new_bomb_row, new_bomb_col) ||
          planner_v3_bfs_is_obstacle(filtered_obstacles, filtered_obstacle_count, temp_bombs, temp_bomb_count, 
                                     push_row, push_col)) {
        continue;
      }
      
      if (planner_v3_bfs_is_box_at(boxes, box_count, new_bomb_row, new_bomb_col, SIZE_MAX) ||
          planner_v3_bfs_is_box_at(boxes, box_count, push_row, push_col, SIZE_MAX)) {
        continue;
      }
      
      Point push_from = {push_row, push_col};
      
      // 计算炸弹到目标的距离（不把当前炸弹当作障碍）
      int dist_after = planner_v3_bfs_distance_between(rows, cols, (Point){new_bomb_row, new_bomb_col}, target_pos,
                                                       filtered_obstacles, filtered_obstacle_count,
                                                       temp_bombs, temp_bomb_count,
                                                       temp_boxes, temp_box_count, 1);
      if (dist_after == INT_MAX) {
        continue;
      }
      
      int adj_pen = planner_v3_bfs_adjacent_blockers(rows, cols, filtered_obstacles, filtered_obstacle_count,
                                                     temp_bombs, temp_bomb_count, temp_boxes, temp_box_count,
                                                     SIZE_MAX, new_bomb_row, new_bomb_col);
      
      int reverse_pen = 0;
      if (last_dr == -dirs[i][0] && last_dc == -dirs[i][1]) {
        int power = 5;
        for (int p = 0; p < reverse_count; ++p) {
          power *= 5;
        }
        reverse_pen = power;
      }
      
      // 车到推位距离需考虑当前炸弹及目标点障碍（炸弹未到目标时）
      int car_to_push = planner_v3_bfs_distance_between(rows, cols, *car_pos, push_from,
                                                       obstacles, obstacle_count,
                                                       bombs, bomb_count,
                                                       temp_boxes, temp_box_count, 1);
      if (car_to_push == INT_MAX) {
        continue;
      }
      
      int score = dist_after * 13 + adj_pen * 2 + reverse_pen + car_to_push * 5;
      candidates[i].dist = dist_after;
      candidates[i].adj_pen = adj_pen;
      candidates[i].score = score;
      candidates[i].feasible = 1;
      candidates[i].push_from = push_from;
      candidates[i].bomb_next.row = new_bomb_row;
      candidates[i].bomb_next.col = new_bomb_col;
    }
    
    int sorted_dirs[4];
    int sorted_count = 0;
    for (int i = 0; i < 4; ++i) {
      if (candidates[i].feasible) {
        sorted_dirs[sorted_count++] = i;
      }
    }
    
    if (sorted_count == 0) {
      last_err_stage = 3;    // 炸弹相关错误
      last_err_detail = 321; // v3: 评分策略无可行方向
      return -6;
    }
    
    // 排序（按评分从优到劣）
    for (int i = 0; i < sorted_count - 1; ++i) {
      for (int j = 0; j < sorted_count - 1 - i; ++j) {
        if (candidates[sorted_dirs[j]].score > candidates[sorted_dirs[j + 1]].score) {
          int tmp = sorted_dirs[j];
          sorted_dirs[j] = sorted_dirs[j + 1];
          sorted_dirs[j + 1] = tmp;
        }
      }
    }
    
    // 按优劣依次尝试各方向，直到有一个可行（仅做可达性检查，不写入 path_buffer）
    DirCandidate chosen;
    int dir_ok = 0;
    for (int k = 0; k < sorted_count; ++k) {
      chosen = candidates[sorted_dirs[k]];
      int next_on_boundary = (chosen.bomb_next.row == 0 || chosen.bomb_next.row == rows - 1 ||
                              chosen.bomb_next.col == 0 || chosen.bomb_next.col == cols - 1);
      if (next_on_boundary) {
        if (!planner_v3_bfs_validate_boundary_path(rows, cols, chosen.bomb_next, target_pos,
                                                   *car_pos, filtered_obstacles, filtered_obstacle_count,
                                                   temp_bombs, temp_bomb_count, temp_boxes, temp_box_count,
                                                   SIZE_MAX, 1)) {
          continue;  /* 推到边界后不可达目标，尝试次优方向 */
        }
      }
      // 车到推位需考虑所有障碍（含目标点）、箱子、炸弹（含当前炸弹）
      if (planner_v3_bfs_distance_between(rows, cols, *car_pos, chosen.push_from,
                                          obstacles, obstacle_count,
                                          bombs, bomb_count,
                                          temp_boxes, temp_box_count, 1) == INT_MAX) {
        continue;  /* 车无法到达推位，尝试次优方向 */
      }
      dir_ok = 1;
      break;
    }
    if (!dir_ok) {
      last_err_stage = 3;
      last_err_detail = 329; /* v3: 炸弹评分策略无可行方向（dir_ok 分支） */
      return -6;
    }
    
    Point push_from = chosen.push_from;
    if (car_pos->row != push_from.row || car_pos->col != push_from.col) {
      // 车到推位时需考虑所有障碍（含目标点，因炸弹尚未到达）、箱子及所有炸弹（含当前炸弹）
      int car_result = planner_v3_bfs_car_move_with_global_astar(rows, cols, car_pos, push_from,
                                                                 obstacles, obstacle_count,
                                                                 bombs, bomb_count,
                                                                 temp_boxes, temp_box_count,
                                                                 path_buffer, path_capacity, out_steps, 1);
      if (car_result == -7) {
        return -7;
      }
      if (car_result == 0) {
        last_err_stage = 3;
        last_err_detail = 322; /* v3: 车无法到达推位（评分策略） */
        return -6;
      }
    }
    if (car_pos->row != push_from.row || car_pos->col != push_from.col) {
      last_err_stage = 3;
      last_err_detail = 323; /* v3: 车到推位后位置不一致（评分策略） */
      return -6;
    }
    if (!planner_v3_bfs_check_adjacent(*car_pos, bomb)) {
      last_err_stage = 3;
      last_err_detail = 324; /* v3: 车与炸弹不相邻（评分策略） */
      return -6;
    }
    
    Point bomb_old_pos = bomb;
    bomb = chosen.bomb_next;
    bombs[bomb_idx] = bomb;
    
    *car_pos = bomb_old_pos;
    if (*out_steps >= path_capacity) {
      return -7;
    }
    if (*out_steps > 0 &&
        !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], *car_pos)) {
      last_err_stage = 3;    // 炸弹相关错误
      last_err_detail = 325; // v3: 车路径不连续（评分策略）
      return -6;
    }
    path_buffer[(*out_steps)++] = *car_pos;
    
    if (last_dr == -chosen.dr && last_dc == -chosen.dc &&
        (last_dr != 0 || last_dc != 0)) {
      reverse_count++;
    } else {
      reverse_count = 0;
    }
    
    last_dr = chosen.dr;
    last_dc = chosen.dc;
  }
  
  // 炸弹到达目标位置
  if (bomb.row == target_pos.row && bomb.col == target_pos.col) {
    return 1;
  }
  
  last_err_stage = 3;    // 炸弹相关错误
  last_err_detail = 326; // v3: 炸弹最终未到达目标
  return -6;
}

// 炸弹爆炸：移除障碍及其相邻障碍
static size_t planner_v3_bfs_bomb_explode(const Point *obstacles, size_t obstacle_count,
                                          Point bomb_pos, Point *new_obstacles,
                                          size_t new_capacity) {
  if (!obstacles || !new_obstacles || new_capacity == 0) {
    return 0;
  }
  
  // 找到被炸的障碍
  int exploded_idx = -1;
  for (size_t i = 0; i < obstacle_count; ++i) {
    if (obstacles[i].row == bomb_pos.row && obstacles[i].col == bomb_pos.col) {
      exploded_idx = (int)i;
      break;
    }
  }
  
  if (exploded_idx < 0) {
    // 炸弹没有击中障碍，保持障碍不变
    size_t copy_count = obstacle_count < new_capacity ? obstacle_count : new_capacity;
    memcpy(new_obstacles, obstacles, copy_count * sizeof(Point));
    return copy_count;
  }
  
  Point center = obstacles[exploded_idx];
  
  // 标记要移除的障碍：中心障碍 + 上下左右相邻障碍
  uint8_t remove[200] = {0};
  remove[exploded_idx] = 1;
  
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (int d = 0; d < 4; ++d) {
    int adj_r = center.row + dirs[d][0];
    int adj_c = center.col + dirs[d][1];
    
    for (size_t i = 0; i < obstacle_count; ++i) {
      if (obstacles[i].row == adj_r && obstacles[i].col == adj_c) {
        remove[i] = 1;
      }
    }
  }
  
  // 复制未被移除的障碍
  size_t new_count = 0;
  for (size_t i = 0; i < obstacle_count && new_count < new_capacity; ++i) {
    if (!remove[i]) {
      new_obstacles[new_count++] = obstacles[i];
    }
  }
  
  return new_count;
}

// 收集有效的箱子
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

// 评分：车到箱子的 BFS 真实距离（按"到达该箱子任意推箱位"的最短距离计算）
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

  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  int best = INT_MAX;
  for (int d = 0; d < 4; ++d) {
    // 推箱位：车站在 box 的反方向
    Point push_pos = {box.row - dirs[d][0], box.col - dirs[d][1]};
    if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col)) {
      continue;
    }
    // 推位可以是炸弹或障碍位置
    // 这里先检查是否是障碍/炸弹（推位可以是这些位置）
    // 但需要检查车是否能到达推位
    
    // 检查推过去后箱子的新位置
    int new_box_row = box.row + dirs[d][0];
    int new_box_col = box.col + dirs[d][1];
    
    // 检查新位置是否在边界内
    if (!planner_v3_bfs_in_bounds(rows, cols, new_box_row, new_box_col)) {
      continue;
    }
    
    // 检查新位置是否在障碍上（不包括炸弹，因为炸弹可以被推走）
    if (planner_v3_bfs_is_obstacle_no_bomb(obstacles, obstacle_count, new_box_row, new_box_col)) {
      continue;
    }
    
    // 检查新位置是否在其他箱子上
    if (planner_v3_bfs_is_box_at(current_boxes, box_count, new_box_row, new_box_col, box_idx)) {
      continue;
    }
    
    // 检查新位置是否是死点（把炸弹当作障碍）
    if (planner_v3_bfs_is_deadlock(rows, cols, obstacles, obstacle_count, bombs, bomb_count,
                                   current_boxes, box_count, box_idx, new_box_row, new_box_col)) {
      continue;
    }

    // 计算车到推位的距离（把炸弹当作障碍）
    int dist_map[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_target(rows, cols, push_pos, obstacles, obstacle_count,
                                               bombs, bomb_count, current_boxes, box_count, 
                                               dist_map, 1)) {  // 1=把炸弹当作障碍
      continue;
    }
    int car_idx = car.row * cols + car.col;
    if (car_idx < 0 || car_idx >= rows * cols) {
      continue;
    }
    if (dist_map[car_idx] != INT_MAX && dist_map[car_idx] < best) {
      best = dist_map[car_idx];
    }
  }
  return best;
}

// 计算箱子到目标的BFS路径长度（考虑除推动箱子外的其他箱子、所有炸弹和障碍）
static int planner_v3_bfs_compute_box_path_length(int rows, int cols, Point box_start,
                                                   Point target, const Point *obstacles,
                                                   size_t obstacle_count, const Point *bombs, size_t bomb_count,
                                                   const Point *boxes,
                                                   size_t box_count, size_t moving_idx,
                                                   int include_bombs) {
  // 构建临时箱子数组：除正在推动的箱子外，其他箱子、所有炸弹和障碍均参与BFS/A*
  Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
  size_t temp_count = 0;
  for (size_t i = 0; i < box_count; ++i) {
    if (i == moving_idx) {
      continue;  // 仅排除正在移动的箱子
    }
    if (boxes[i].row < 0 || boxes[i].col < 0) {
      continue;  // 忽略已完成的箱子
    }
    temp_boxes[temp_count++] = boxes[i];
  }

  // 从目标点做全局BFS（障碍、炸弹、其他箱子均视为不可通行）
  int dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                             bombs, bomb_count, temp_boxes, temp_count, dist, include_bombs)) {
    return INT_MAX;
  }

  // 使用A*计算路径长度（同样考虑障碍、炸弹、其他箱子）
  Point temp_path[PLANNER_V3_BFS_MAX_PATH_LEN];
  size_t path_len = 0;
  if (!planner_v3_bfs_astar_with_dist(rows, cols, box_start, target, obstacles, obstacle_count,
                                      bombs, bomb_count, temp_boxes, temp_count, dist,
                                      temp_path, PLANNER_V3_BFS_MAX_PATH_LEN, &path_len,
                                      include_bombs, 0, NULL,
                                      0, 0, (Point){0, 0})) {
    return INT_MAX;
  }

  if (path_len == 0) {
    return INT_MAX;
  }

  // 返回路径步数（路径长度-1，因为起点也算一个点）
  return (int)(path_len > 0 ? path_len - 1 : 0);
}

// 为箱子分配目标点（按照文档逻辑：先按车到箱子BFS距离排序，再依次为箱子选最近目标）
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

  // 构建箱子排序（按车到箱子的BFS真实距离）
  size_t order[PLANNER_V3_BFS_MAX_BOXES];
  int scores[PLANNER_V3_BFS_MAX_BOXES];
  size_t n = 0;
  for (size_t i = 0; i < usable; ++i) {
    order[n] = i;
    // 计算车到箱子的BFS真实距离（需要考虑原始索引）
    scores[n] = planner_v3_bfs_score_car_to_box(rows, cols, car, source_indices[i], boxes,
                                                 box_count, obstacles, obstacle_count, bombs, bomb_count);
    n++;
  }

  // 简单冒泡排序（n<=10）
  for (size_t i = 0; i + 1 < n; ++i) {
    for (size_t j = 0; j + 1 < n - i; ++j) {
      if (scores[j] > scores[j + 1] ||
          (scores[j] == scores[j + 1] && order[j] > order[j + 1])) {
        int ts = scores[j];
        scores[j] = scores[j + 1];
        scores[j + 1] = ts;
        size_t ti = order[j];
        order[j] = order[j + 1];
        order[j + 1] = ti;
      }
    }
  }

  uint8_t target_used[PLANNER_V3_BFS_MAX_CELLS] = {0};

  // 依序分配目标：依次为箱子从空闲目标里选"箱子到目标 BFS 真实距离最近且可达"的目标
  size_t assigned = 0;
  for (size_t oi = 0; oi < n; ++oi) {
    size_t bi = order[oi];
    size_t best_t = SIZE_MAX;
    int best_steps = INT_MAX;
    int best_is_special = 0;
    Point best_obstacle_passed = {-1, -1};
    int best_has_obstacle = 0;

    for (size_t t = 0; t < target_count; ++t) {
      if (target_used[t]) {
        continue;
      }
      // 直接尝试特殊路径规划（允许经过一次障碍，不把炸弹当作障碍）
      Point obstacle_passed = {-1, -1};
      int has_obstacle = 0;
      size_t sp_len = 0;
      int steps = INT_MAX;
      int cur_is_special = 0;
      
      if (planner_v3_bfs_special_path_planning(rows, cols, valid_boxes[bi], targets[t],
                                               obstacles, obstacle_count, bombs, bomb_count,
                                               boxes, box_count, source_indices[bi],
                                               car,
                                               &obstacle_passed, &has_obstacle,
                                               NULL, 0, &sp_len)) {
        // 特殊路径规划成功，使用真实步数估算（路径长度-1）
        if (sp_len > 0) {
          steps = (int)(sp_len - 1);
        } else {
          steps = INT_MAX;
        }
        cur_is_special = 1;
      } else {
        // 特殊路径不可达，尝试正常路径规划（把炸弹当作障碍）
        steps = planner_v3_bfs_compute_box_path_length(rows, cols, valid_boxes[bi],
                                                        targets[t], obstacles, obstacle_count,
                                                        bombs, bomb_count, boxes, box_count, 
                                                        source_indices[bi], 1);  // 1=把炸弹当作障碍
        if (steps == INT_MAX) {
          continue;  // 正常路径也不可达
        }
        cur_is_special = 0;
        obstacle_passed.row = -1;
        obstacle_passed.col = -1;
        has_obstacle = 0;
      }
      if (steps < best_steps || (steps == best_steps && t < best_t)) {
        best_steps = steps;
        best_t = t;
        best_is_special = cur_is_special;
        best_obstacle_passed = obstacle_passed;
        best_has_obstacle = has_obstacle;
      }
    }

    if (best_t == SIZE_MAX) {
      last_err_stage = 2;   // 动态选目标失败
      last_err_detail = 101; // v3: 目标分配失败：无可行目标
      return -6;   // 找不到可行目标
    }

    out_goals[assigned].box = valid_boxes[bi];
    out_goals[assigned].target = targets[best_t];
    out_goals[assigned].target_idx = best_t;
    out_goals[assigned].source_idx = source_indices[bi];
    out_goals[assigned].has_obstacle_to_clear = 0;
    out_goals[assigned].obstacle_to_clear.row = -1;
    out_goals[assigned].obstacle_to_clear.col = -1;

    // 如果使用了特殊路径规划，记录需要炸掉的障碍
    if (best_is_special && best_has_obstacle) {
      out_goals[assigned].has_obstacle_to_clear = 1;
      out_goals[assigned].obstacle_to_clear = best_obstacle_passed;
    }

    if (out_mapping) {
      out_mapping[source_indices[bi]] = best_t;
    }

    target_used[best_t] = 1;
    assigned++;
  }

  *out_goal_count = assigned;
  return 0;
}

// 主推箱规划函数（纯BFS+A*版本，支持炸弹）
static int planner_v3_bfs_run_assigned(int rows, int cols, Point car,
                                   const PlannerBoxGoalBFS *goals,
                                   size_t goal_count, size_t box_count,
                                   const Point *obstacles,
                                   size_t obstacle_count, const Point *bombs, size_t bomb_count,
                                   Point *path_buffer,
                                   size_t path_capacity, size_t *out_steps,
                                   PlannerAllBoxPaths *out_final_paths,
                                   Point *bombs_mutable) {
  if (!goals || !path_buffer || !out_steps) {
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

  Point current_car = car;
  Point car_start = car;  // 保存起点，用于最后返回
  Point current_boxes[PLANNER_V3_BFS_MAX_BOXES];
  Point current_obstacles[200];  // 动态障碍列表
  size_t current_obstacle_count = obstacle_count;
  if (obstacle_count > 200) {
    current_obstacle_count = 200;
  }
  memcpy(current_obstacles, obstacles, current_obstacle_count * sizeof(Point));

  for (size_t i = 0; i < goal_count; ++i) {
    current_boxes[i] = goals[i].box;
  }

  // 初始化箱子路径记录
  Point box_paths[PLANNER_V3_BFS_MAX_BOXES][PLANNER_V3_BFS_MAX_CELLS];
  size_t box_path_lens[PLANNER_V3_BFS_MAX_BOXES] = {0};
  size_t box_path_starts[PLANNER_V3_BFS_MAX_BOXES] = {0};

  // 循环处理：每推完一个箱子后重新分配目标
  size_t completed_count = 0;
  while (completed_count < goal_count) {
    // 重新分配目标（使用当前车位置和当前箱子状态）
    PlannerBoxGoalBFS new_goals[PLANNER_V3_BFS_MAX_BOXES];
    size_t new_goal_count = 0;
    
    // 构建当前有效的箱子数组（排除已完成的）
    Point active_boxes[PLANNER_V3_BFS_MAX_BOXES];
    size_t active_box_count = 0;
    size_t active_to_original[PLANNER_V3_BFS_MAX_BOXES];
    
    for (size_t i = 0; i < goal_count; ++i) {
      if (current_boxes[i].row >= 0 && current_boxes[i].col >= 0) {
        active_boxes[active_box_count] = current_boxes[i];
        active_to_original[active_box_count] = i;
        active_box_count++;
      }
    }
    
    if (active_box_count == 0) {
      break;  // 所有箱子都已完成
    }
    
    // 构建当前有效的目标数组（排除已被占用的目标）
    Point active_targets[PLANNER_V3_BFS_MAX_BOXES];
    size_t active_target_count = 0;
    uint8_t target_used[PLANNER_V3_BFS_MAX_BOXES] = {0};
    
    // 找出已完成的箱子占用的目标
    for (size_t i = 0; i < goal_count; ++i) {
      if (current_boxes[i].row < 0 || current_boxes[i].col < 0) {
        // 箱子已完成，标记其目标为已使用
        for (size_t j = 0; j < goal_count; ++j) {
          if (goals[j].target.row == goals[i].target.row &&
              goals[j].target.col == goals[i].target.col) {
            target_used[j] = 1;
            break;
          }
        }
      }
    }
    
    // 收集未使用的目标（去重）
    for (size_t i = 0; i < goal_count; ++i) {
      if (target_used[i]) {
        continue;
      }
      int already_added = 0;
      for (size_t j = 0; j < active_target_count; ++j) {
        if (active_targets[j].row == goals[i].target.row &&
            active_targets[j].col == goals[i].target.col) {
          already_added = 1;
          break;
        }
      }
      if (!already_added) {
        active_targets[active_target_count++] = goals[i].target;
      }
    }
    
    // 重新分配目标
    size_t temp_mapping[PLANNER_V3_BFS_MAX_BOXES];
    int assign_res = planner_v3_bfs_assign_targets(
        rows, cols, current_car, active_boxes, active_box_count,
        active_targets, active_target_count, current_obstacles, current_obstacle_count,
        bombs_mutable, bomb_count,
        new_goals, &new_goal_count, temp_mapping);
    
    if (assign_res != 0 || new_goal_count == 0) {
      last_err_stage = 2;    // 动态选目标失败
      last_err_detail = 102; // v3: 动态重分配失败
      return -6;   // 重新分配失败
    }
    
    // 选择下一个要推的箱子：只依据车到箱子的BFS真实距离
    size_t selected_box_idx = SIZE_MAX;
    int best_car_dist = INT_MAX;
    
    for (size_t i = 0; i < new_goal_count; ++i) {
      // 找到对应的原始箱子索引
      size_t orig_idx = SIZE_MAX;
      for (size_t j = 0; j < active_box_count; ++j) {
        if (active_boxes[j].row == new_goals[i].box.row &&
            active_boxes[j].col == new_goals[i].box.col) {
          orig_idx = active_to_original[j];
          break;
        }
      }
      
      if (orig_idx == SIZE_MAX) {
        continue;
      }
      
      // 计算车到箱子的BFS真实距离
      int car_dist = planner_v3_bfs_score_car_to_box(
          rows, cols, current_car, orig_idx, current_boxes, goal_count,
          current_obstacles, current_obstacle_count, bombs_mutable, bomb_count);
      
      if (car_dist < best_car_dist) {
        best_car_dist = car_dist;
        selected_box_idx = i;
      }
    }
    
    if (selected_box_idx == SIZE_MAX) {
      last_err_stage = 2;    // 动态选目标失败
      last_err_detail = 103; // v3: 无法找到可推箱子
      return -6;   // 无法选择箱子
    }
    
    // 找到对应的原始箱子索引
    size_t box_idx = SIZE_MAX;
    for (size_t j = 0; j < active_box_count; ++j) {
      if (active_boxes[j].row == new_goals[selected_box_idx].box.row &&
          active_boxes[j].col == new_goals[selected_box_idx].box.col) {
        box_idx = active_to_original[j];
        break;
      }
    }
    
    if (box_idx == SIZE_MAX) {
      last_err_stage = 2;    // 动态选目标失败
      last_err_detail = 104; // v3: 箱子索引映射失败
      return -6;
    }
    /* 与 PlannerPointV3_BFS boxes 索引一致，用于 special_paths / 显示 */
    size_t orig_box_idx = goals[box_idx].source_idx;

    Point box = current_boxes[box_idx];
    Point target = new_goals[selected_box_idx].target;
    int has_obstacle_to_clear = new_goals[selected_box_idx].has_obstacle_to_clear;
    Point obstacle_to_clear = new_goals[selected_box_idx].obstacle_to_clear;

    // 记录当前箱子路径的起始位置
    box_path_starts[box_idx] = *out_steps;
    // 记录箱子起始位置
    if (box_path_lens[box_idx] < PLANNER_V3_BFS_MAX_CELLS) {
      box_paths[box_idx][box_path_lens[box_idx]++] = box;
    }

    // 箱子已在目标点，标记为"消失"
    if (box.row == target.row && box.col == target.col) {
      current_boxes[box_idx].row = -1;
      current_boxes[box_idx].col = -1;
      if (box_path_lens[box_idx] < PLANNER_V3_BFS_MAX_CELLS) {
        box_paths[box_idx][box_path_lens[box_idx]++] = target;
      }
      completed_count++;
      continue;
    }

    // 分配目标完后：先使用 BFS+A* 给该箱子规划路径；规划过程中任何错误导致不成功则直接进行特殊路径计算
    int use_special_path = 0;
    int target_dist[PLANNER_V3_BFS_MAX_CELLS];
    Point box_path[PLANNER_V3_BFS_MAX_CELLS];
    size_t box_path_len = 0;
    int has_box_path = 0;

    // 构建临时箱子数组：除当前推动的箱子外，其他箱子、所有炸弹和障碍均参与路径计算
    Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
    size_t temp_count = 0;
    for (size_t i = 0; i < goal_count; ++i) {
      if (i == box_idx) continue;
      if (current_boxes[i].row < 0 || current_boxes[i].col < 0) continue;
      temp_boxes[temp_count++] = current_boxes[i];
    }

    // 1) 先尝试 BFS+A* 正常路径规划（障碍、炸弹、除当前箱外的其他箱子均考虑）
    int target_bfs_ok = planner_v3_bfs_global_bfs_from_target(rows, cols, target, current_obstacles, current_obstacle_count,
                                                               bombs_mutable, bomb_count, current_boxes, goal_count,
                                                               target_dist, 1);  // 1=把炸弹当作障碍
    if (!target_bfs_ok) {
      last_err_stage = 2;
      last_err_detail = 128; // v3: 目标BFS不可达，启用特殊路径
      use_special_path = 1;
    }
    if (!use_special_path) {
      int box_path_dist[PLANNER_V3_BFS_MAX_CELLS];
      if (planner_v3_bfs_global_bfs_from_target(rows, cols, target, current_obstacles, current_obstacle_count,
                                                 bombs_mutable, bomb_count, temp_boxes, temp_count,
                                                 box_path_dist, 1)) {
        /* A* 内已做推位检测，推位不合理的节点会被排除 */
        if (planner_v3_bfs_astar_with_dist(rows, cols, box, target, current_obstacles, current_obstacle_count,
                                           bombs_mutable, bomb_count, temp_boxes, temp_count, box_path_dist,
                                           box_path, PLANNER_V3_BFS_MAX_CELLS, &box_path_len,
                                           1, 0, NULL,
                                           1, 0, current_car)) {
          if (box_path_len > 1) {
            has_box_path = 1;
          }
        }
      }
      if (!has_box_path) {
        last_err_stage = 2;
        last_err_detail = 129; // v3: 正常箱子路径不可行，启用特殊路径
        use_special_path = 1;
      }
    }

    // 2) 规划不成功则直接进行特殊路径计算；特殊路径出来后先推炸弹再推箱子
    if (use_special_path) {
      Point sp_obstacle = {-1, -1};
      int sp_has_obs = 0;
      Point sp_path[PLANNER_V3_BFS_MAX_PATH_LEN];
      size_t sp_len = 0;
      if (!planner_v3_bfs_special_path_planning(rows, cols, box, target,
                                                current_obstacles, current_obstacle_count,
                                                bombs_mutable, bomb_count,
                                                current_boxes, goal_count, box_idx,
                                                current_car,
                                                &sp_obstacle, &sp_has_obs,
                                                sp_path, PLANNER_V3_BFS_MAX_PATH_LEN, &sp_len)) {
        last_err_stage = 2;
        last_err_detail = 105; // v3: 正常路径规划失败且特殊路径也不可达
        return -6;
      }
      has_obstacle_to_clear = sp_has_obs;
      obstacle_to_clear = sp_obstacle;
      // 记录特殊路径用于菜单显示（索引与 boxes 一致）
      if (orig_box_idx < PLANNER_V3_BFS_MAX_BOXES) {
        special_paths.box_paths[orig_box_idx].valid = 1;
        special_paths.box_paths[orig_box_idx].path_len = (sp_len < 400) ? (size_t)sp_len : 400;
        for (size_t pi = 0; pi < special_paths.box_paths[orig_box_idx].path_len; ++pi) {
          special_paths.box_paths[orig_box_idx].path[pi] = sp_path[pi];
        }
        special_paths.box_paths[orig_box_idx].has_obstacle_to_clear = has_obstacle_to_clear ? 1 : 0;
        special_paths.box_paths[orig_box_idx].obstacle_to_clear = obstacle_to_clear;
        {
          size_t bombs_on_path_idx[MAX_BOMBS];
          size_t bombs_on_path_cnt = 0;
          planner_v3_bfs_find_bombs_on_path(sp_path, sp_len, bombs_mutable, bomb_count,
                                            bombs_on_path_idx, &bombs_on_path_cnt);
          special_paths.box_paths[orig_box_idx].bomb_on_path_count = (int)(bombs_on_path_cnt <= 5 ? bombs_on_path_cnt : 5);
          for (size_t bop = 0; bop < (size_t)special_paths.box_paths[orig_box_idx].bomb_on_path_count; ++bop) {
            special_paths.box_paths[orig_box_idx].bomb_on_path[bop] = bombs_mutable[bombs_on_path_idx[bop]];
          }
        }
      }

      // 推炸弹：先选炸弹（距特殊路径经过的障碍最近的优先），再选目标（备选=该障碍及其上下左右障碍，按可达距离升序），
      // 先试最优目标，不行则试次优目标，直至所有目标；该炸弹都推不到则试次优炸弹，依次尝试所有炸弹
      if (has_obstacle_to_clear && obstacle_to_clear.row >= 0 && obstacle_to_clear.col >= 0) {
        Point saved_car = current_car;
        Point saved_bombs[MAX_BOMBS];
        memcpy(saved_bombs, bombs_mutable, bomb_count * sizeof(Point));
        size_t saved_steps = *out_steps;
        Point saved_path[PUSH_BOMB_SAVE_PATH_MAX];
        if (saved_steps > PUSH_BOMB_SAVE_PATH_MAX) {
          last_err_stage = 3;
          last_err_detail = 300; // v3: 推炸弹前路径过长无法保存状态
          return -6;
        }
        memcpy(saved_path, path_buffer, saved_steps * sizeof(Point));

        size_t bomb_order[MAX_BOMBS];
        size_t bomb_order_count = 0;
        planner_v3_bfs_bombs_sorted_by_dist_to_obstacle(rows, cols, obstacle_to_clear,
            current_obstacles, current_obstacle_count, bombs_mutable, bomb_count,
            current_boxes, goal_count, bomb_order, &bomb_order_count,
            s_special_path_obstacles, s_special_path_obstacle_count);

        // 若特殊路径经过某炸弹，消灭障碍时优先使用该炸弹（将路径上的炸弹排在前面）
        {
          size_t bomb_order_reordered[MAX_BOMBS];
          size_t reorder_count = 0;
          for (size_t bo = 0; bo < bomb_order_count; ++bo) {
            size_t bi = bomb_order[bo];
            int on_path = 0;
            for (size_t p = 0; p < sp_len; ++p) {
              if (sp_path[p].row == bombs_mutable[bi].row && sp_path[p].col == bombs_mutable[bi].col) {
                on_path = 1;
                break;
              }
            }
            if (on_path) bomb_order_reordered[reorder_count++] = bi;
          }
          for (size_t bo = 0; bo < bomb_order_count; ++bo) {
            size_t bi = bomb_order[bo];
            int on_path = 0;
            for (size_t p = 0; p < sp_len; ++p) {
              if (sp_path[p].row == bombs_mutable[bi].row && sp_path[p].col == bombs_mutable[bi].col) {
                on_path = 1;
                break;
              }
            }
            if (!on_path) bomb_order_reordered[reorder_count++] = bi;
          }
          if (reorder_count == bomb_order_count) {
            memcpy(bomb_order, bomb_order_reordered, reorder_count * sizeof(size_t));
          }
        }

        int push_success = 0;
        size_t chosen_bomb_idx = SIZE_MAX;

        for (size_t bo = 0; bo < bomb_order_count && !push_success; ++bo) {
          size_t bomb_idx = bomb_order[bo];
          Point bomb_targets[PLANNER_V3_BFS_MAX_CANDIDATE_TARGETS];
          size_t bomb_target_count = 0;
          planner_v3_bfs_targets_sorted_by_reachable_dist(rows, cols, saved_car,
              bomb_idx, saved_bombs, bomb_count, obstacle_to_clear,
              current_obstacles, current_obstacle_count, current_boxes, goal_count,
              bomb_targets, &bomb_target_count,
              s_special_path_obstacles, s_special_path_obstacle_count);

          for (size_t bt = 0; bt < bomb_target_count && !push_success; ++bt) {
            current_car = saved_car;
            memcpy(bombs_mutable, saved_bombs, bomb_count * sizeof(Point));
            *out_steps = saved_steps;
            memcpy(path_buffer, saved_path, saved_steps * sizeof(Point));

            int bomb_result = planner_v3_bfs_push_bomb(rows, cols, &current_car,
                bombs_mutable, bomb_count, bomb_idx, bomb_targets[bt],
                obstacle_to_clear, current_obstacles, current_obstacle_count,
                current_boxes, goal_count, path_buffer, path_capacity, out_steps,
                s_special_path_obstacles, s_special_path_obstacle_count);
            if (bomb_result == 1) {
              push_success = 1;
              chosen_bomb_idx = bomb_idx;
            }
          }
        }

        // 仅当尝试了所有炸弹及各自备选目标均失败后才返回
        if (!push_success) {
          last_err_stage = 3;
          last_err_detail = 301; // v3: 尝试了所有炸弹都不行
          return -6;
        }
        Point bomb_target_pos = bombs_mutable[chosen_bomb_idx];
        if (orig_box_idx < PLANNER_V3_BFS_MAX_BOXES) {
          special_paths.box_paths[orig_box_idx].has_bomb_exploded = 1;
          special_paths.box_paths[orig_box_idx].bomb_exploded_at = bomb_target_pos;
          special_paths.box_paths[orig_box_idx].bomb_extra_exploded_count = 0;
        }
        Point new_obstacles[200];
        size_t new_obstacle_count = planner_v3_bfs_bomb_explode(current_obstacles, current_obstacle_count,
                                                                 bomb_target_pos, new_obstacles, 200);
        memcpy(current_obstacles, new_obstacles, new_obstacle_count * sizeof(Point));
        current_obstacle_count = new_obstacle_count;
        bombs_mutable[chosen_bomb_idx].row = -1;
        bombs_mutable[chosen_bomb_idx].col = -1;
      } else {
        // 特殊路径没有障碍需要清除，检查路径上是否有炸弹
        // 如果有炸弹，需要把炸弹推到距离它最近的障碍里
        size_t bombs_on_path[MAX_BOMBS];
        size_t bombs_on_path_count = 0;
        planner_v3_bfs_find_bombs_on_path(sp_path, sp_len, bombs_mutable, bomb_count,
                                          bombs_on_path, &bombs_on_path_count);
        
        if (bombs_on_path_count > 0) {
          // 路径上有炸弹，逐个处理
          for (size_t bop_idx = 0; bop_idx < bombs_on_path_count; ++bop_idx) {
            size_t bomb_on_path_idx = bombs_on_path[bop_idx];
            
            // 检查炸弹是否已被使用
            if (bombs_mutable[bomb_on_path_idx].row < 0 || bombs_mutable[bomb_on_path_idx].col < 0) {
              continue;
            }
            
            Point bomb_target = {-1, -1};
            Point extra_bomb_pos[PLANNER_V3_BFS_MAX_EXTRA_BOMB_DISPLAY];
            size_t extra_bomb_count = 0;
            if (planner_v3_bfs_handle_bomb_on_path(rows, cols, &current_car,
                                                    bombs_mutable, bomb_count, bomb_on_path_idx,
                                                    current_obstacles, &current_obstacle_count,
                                                    current_boxes, goal_count,
                                                    sp_path, sp_len,
                                                    path_buffer, path_capacity, out_steps,
                                                    &bomb_target,
                                                    extra_bomb_pos, PLANNER_V3_BFS_MAX_EXTRA_BOMB_DISPLAY, &extra_bomb_count)) {
              // 炸弹处理成功，记录用于显示（炸弹A + 炸弹B等清除推位的爆炸位置）
              if (orig_box_idx < PLANNER_V3_BFS_MAX_BOXES && bomb_target.row >= 0) {
                special_paths.box_paths[orig_box_idx].has_bomb_exploded = 1;
                special_paths.box_paths[orig_box_idx].bomb_exploded_at = bomb_target;
                special_paths.box_paths[orig_box_idx].bomb_extra_exploded_count = 0;
              }
              if (orig_box_idx < PLANNER_V3_BFS_MAX_BOXES && extra_bomb_count > 0) {
                size_t n = extra_bomb_count;
                if (n > 5) n = 5;
                special_paths.box_paths[orig_box_idx].bomb_extra_exploded_count = (int)n;
                for (size_t ei = 0; ei < n; ++ei) {
                  special_paths.box_paths[orig_box_idx].bomb_extra_exploded_at[ei] = extra_bomb_pos[ei];
                }
              }
            } else {
              // 炸弹处理失败
              last_err_stage = 3;
              last_err_detail = 302; // v3: 特殊路径上的炸弹无法处理
              return -6;
            }
          }
        }
      }

      // 推箱子前重新计算路径（考虑所有障碍、箱子、炸弹）
      if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, current_obstacles, current_obstacle_count,
                                                 bombs_mutable, bomb_count, current_boxes, goal_count,
                                                 target_dist, 1)) {
        last_err_stage = 2;
        last_err_detail = 121; // v3: 特殊路径推炸弹后单箱推送全局BFS失败
        return -6;
      }
      temp_count = 0;
      for (size_t i = 0; i < goal_count; ++i) {
        if (i == box_idx) continue;
        if (current_boxes[i].row < 0 || current_boxes[i].col < 0) continue;
        temp_boxes[temp_count++] = current_boxes[i];
      }
      {
        int box_path_dist[PLANNER_V3_BFS_MAX_CELLS];
        if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, current_obstacles, current_obstacle_count,
                                                   bombs_mutable, bomb_count, temp_boxes, temp_count,
                                                   box_path_dist, 1) ||
            !planner_v3_bfs_astar_with_dist(rows, cols, box, target, current_obstacles, current_obstacle_count,
                                            bombs_mutable, bomb_count, temp_boxes, temp_count, box_path_dist,
                                            box_path, PLANNER_V3_BFS_MAX_CELLS, &box_path_len,
                                            1, 0, NULL,
                                            1, 0, current_car) || box_path_len <= 1) {
          last_err_stage = 2;
          last_err_detail = 106; // v3: 特殊路径推炸弹后箱子路径规划失败
          return -6;  // 推箱子环节失败不再计算特殊路径，直接返回
        }
        has_box_path = 1;
      }
    }
    
    // 先模拟路径策略与评分策略，选取步数少的再实际执行
    int box_path_steps = INT_MAX;
    int box_score_steps = INT_MAX;
    if (has_box_path && box_path_len > 1) {
      box_path_steps = planner_v3_bfs_simulate_push_box_path(rows, cols, current_car, box, target,
          box_path, box_path_len, current_obstacles, current_obstacle_count,
          bombs_mutable, bomb_count, current_boxes, goal_count, box_idx);
    }
    box_score_steps = planner_v3_bfs_simulate_push_box_score(rows, cols, current_car, box, target,
        current_obstacles, current_obstacle_count, bombs_mutable, bomb_count,
        current_boxes, goal_count, box_idx);
    /* 仅路径模式：有有效 BFS+A* 路径即用路径；否则与评分策略比较步数取少者 */
    int use_box_path = (has_box_path && box_path_len > 1 && box_path_steps != INT_MAX &&
        (planner_v3_push_only_bfs_astar_path ? 1 : (box_path_steps <= box_score_steps)));
    if (planner_v3_push_only_bfs_astar_path && !(has_box_path && box_path_len > 1 && box_path_steps != INT_MAX)) {
      last_err_stage = 2;
      last_err_detail = 124; // v3: 仅路径模式但箱子无有效BFS+A*路径
      return -6;
    }
    
    // 推箱子到目标（使用路径策略或评分策略，取步数少者）
    if (use_box_path && has_box_path && box_path_len > 1) {
      size_t box_path_idx = 0;
      while (box.row != target.row || box.col != target.col) {
        if (box_path_idx + 1 >= box_path_len) {
          last_err_stage = 2;
          last_err_detail = 122; // v3: 预计算箱子路径越界
          return -6;
        }
        
        Point next_in_path = box_path[box_path_idx + 1];
        int dr = next_in_path.row - box.row;
        int dc = next_in_path.col - box.col;
        
        if (planner_v3_bfs_abs(dr) + planner_v3_bfs_abs(dc) != 1) {
          last_err_stage = 2;
          last_err_detail = 107; // v3: 箱子路径出现非相邻跳跃
          return -6;
        }
        
        int push_row = box.row - dr;
        int push_col = box.col - dc;
        
        // 检查是否越界（边界外）
        if (!planner_v3_bfs_in_bounds(rows, cols, next_in_path.row, next_in_path.col) ||
            !planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
          last_err_stage = 2;
          last_err_detail = 108; // v3: 箱子或推位越界
          return -6;
        }
        
        // 检查下一步是否会把箱子推到边界上（row=0, row=rows-1, col=0, col=cols-1）
        int next_on_boundary = (next_in_path.row == 0 || next_in_path.row == rows - 1 ||
                                next_in_path.col == 0 || next_in_path.col == cols - 1);
        
        if (next_on_boundary) {
          // 如果下一步会把箱子推到边界上，需要检测是否可达目标点并且检验到达目标点的每一步的推位是否合理
          if (!planner_v3_bfs_validate_boundary_path(rows, cols, next_in_path, target,
                                                     current_car, current_obstacles, current_obstacle_count,
                                                     bombs_mutable, bomb_count, current_boxes, goal_count,
                                                     box_idx, 1)) {  // 1=把炸弹当作障碍
            last_err_stage = 2;
            last_err_detail = 120; // v3: 边界位置无法到达目标或推位不合理
            return -6;
          }
        }
        
        // 检查障碍（把炸弹当作障碍）
        if (planner_v3_bfs_is_obstacle(current_obstacles, current_obstacle_count, bombs_mutable, bomb_count, 
                                      next_in_path.row, next_in_path.col) ||
            planner_v3_bfs_is_obstacle(current_obstacles, current_obstacle_count, bombs_mutable, bomb_count, 
                                      push_row, push_col)) {
          last_err_stage = 2;
          last_err_detail = 109; // v3: 箱子或推位落在障碍/炸弹
          return -6;
        }
        
        if (planner_v3_bfs_is_box_at(current_boxes, goal_count, next_in_path.row, next_in_path.col, box_idx) ||
            planner_v3_bfs_is_box_at(current_boxes, goal_count, push_row, push_col, box_idx)) {
          last_err_stage = 2;
          last_err_detail = 110; // v3: 箱子或推位与其他箱子冲突
          return -6;
        }
        
        if (next_in_path.row != target.row || next_in_path.col != target.col) {
          if (!planner_v3_bfs_can_reach_goal(rows, cols, current_obstacles, current_obstacle_count,
                                            bombs_mutable, bomb_count, current_boxes, goal_count, 
                                            box_idx, next_in_path, target, 1)) {  // 1=把炸弹当作障碍
            // 推箱子环节失败不再计算特殊路径，直接返回
            last_err_stage = 2;
            last_err_detail = 111; // v3: 中间箱子位置无法再到目标
            return -6;
          }
        }
        
        Point push_from = {push_row, push_col};
        if (current_car.row != push_from.row || current_car.col != push_from.col) {
          int car_result = planner_v3_bfs_car_move_with_global_astar(rows, cols, &current_car, push_from,
                                                                     current_obstacles, current_obstacle_count,
                                                                     bombs_mutable, bomb_count,
                                                                     current_boxes, goal_count,
                                                                     path_buffer, path_capacity, out_steps, 1);
          if (car_result == -7) {
            return -7;
          }
          if (car_result == 0) {
            last_err_stage = 2;
            last_err_detail = 112; // v3: 车无法到达推位
            return -6;
          }
        }
        
        if (current_car.row != push_from.row || current_car.col != push_from.col) {
          last_err_stage = 2;
          last_err_detail = 113; // v3: 车到推位后位置不一致
          return -6;
        }
        
        if (!planner_v3_bfs_check_adjacent(current_car, box)) {
          last_err_stage = 2;
          last_err_detail = 114; // v3: 车与箱子不相邻
          return -6;
        }
        
        Point box_old_pos = box;
        box = next_in_path;
        current_boxes[box_idx] = box;
        box_path_idx++;
        
        if (box_path_lens[box_idx] < PLANNER_V3_BFS_MAX_CELLS) {
          box_paths[box_idx][box_path_lens[box_idx]++] = box;
        }
        
        current_car = box_old_pos;
        if (*out_steps >= path_capacity) {
          return -7;
        }
        if (*out_steps > 0 &&
            !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], current_car)) {
          last_err_stage = 2;
          last_err_detail = 115; // v3: 车路径不连续（推箱过程中）
          return -6;
        }
        path_buffer[(*out_steps)++] = current_car;
      }
    } else {
      // 路径策略失败，使用评分策略作为备选
      typedef struct {
        int dr;
        int dc;
        int dist;
        int adj_pen;
        int score;
        int feasible;
        Point push_from;
        Point box_next;
      } DirCandidate;
      
      DirCandidate candidates[4];
      const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
      int last_dr = 0;
      int last_dc = 0;
      int reverse_count = 0;
      
      while (box.row != target.row || box.col != target.col) {
        for (int i = 0; i < 4; ++i) {
          candidates[i].dr = dirs[i][0];
          candidates[i].dc = dirs[i][1];
          candidates[i].feasible = 0;
          candidates[i].score = INT_MAX;
          
          int new_box_row = box.row + dirs[i][0];
          int new_box_col = box.col + dirs[i][1];
          int push_row = box.row - dirs[i][0];
          int push_col = box.col - dirs[i][1];
          
          if (!planner_v3_bfs_in_bounds(rows, cols, new_box_row, new_box_col) ||
              !planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
            continue;
          }
          
          // 检查下一步是否会把箱子推到边界上
          int next_on_boundary = (new_box_row == 0 || new_box_row == rows - 1 ||
                                  new_box_col == 0 || new_box_col == cols - 1);
          
          if (next_on_boundary) {
            // 推到边界后检测是否可达目标点，并检验到达目标点的每一步推位是否合理
            if (!planner_v3_bfs_validate_boundary_path(rows, cols, (Point){new_box_row, new_box_col}, target,
                                                      current_car, current_obstacles, current_obstacle_count,
                                                      bombs_mutable, bomb_count, current_boxes, goal_count,
                                                      box_idx, 1)) {
              continue;
            }
          }
          
          if (planner_v3_bfs_is_obstacle(current_obstacles, current_obstacle_count, bombs_mutable, bomb_count,
                                        new_box_row, new_box_col) ||
              planner_v3_bfs_is_obstacle(current_obstacles, current_obstacle_count, bombs_mutable, bomb_count,
                                        push_row, push_col)) {
            continue;
          }
          
          if (planner_v3_bfs_is_box_at(current_boxes, goal_count, new_box_row, new_box_col, box_idx) ||
              planner_v3_bfs_is_box_at(current_boxes, goal_count, push_row, push_col, box_idx)) {
            continue;
          }
          
          // 如果下一步不是目标点，检查是否可达目标
          if (new_box_row != target.row || new_box_col != target.col) {
            if (!planner_v3_bfs_can_reach_goal(rows, cols, current_obstacles, current_obstacle_count,
                                              bombs_mutable, bomb_count, current_boxes, goal_count,
                                              box_idx, (Point){new_box_row, new_box_col}, target, 1)) {
              continue;
            }
          }
          
          Point push_from = {push_row, push_col};
          
          // 计算箱子到目标的距离
          int dist_after = planner_v3_bfs_distance_between(rows, cols, (Point){new_box_row, new_box_col}, target,
                                                          current_obstacles, current_obstacle_count,
                                                          bombs_mutable, bomb_count, temp_boxes, temp_count, 1);
          if (dist_after == INT_MAX) {
            continue;
          }
          
          int adj_pen = planner_v3_bfs_adjacent_blockers(rows, cols, current_obstacles, current_obstacle_count,
                                                         bombs_mutable, bomb_count, current_boxes, goal_count,
                                                         box_idx, new_box_row, new_box_col);
          
          int reverse_pen = 0;
          if (last_dr == -dirs[i][0] && last_dc == -dirs[i][1]) {
            int power = 5;
            for (int p = 0; p < reverse_count; ++p) {
              power *= 5;
            }
            reverse_pen = power;
          }
          
          int car_to_push = planner_v3_bfs_distance_between(rows, cols, current_car, push_from,
                                                            current_obstacles, current_obstacle_count,
                                                            bombs_mutable, bomb_count, current_boxes, goal_count, 1);
          if (car_to_push == INT_MAX) {
            continue;
          }
          
          int score = dist_after * 13 + adj_pen * 2 + reverse_pen + car_to_push * 5;
          candidates[i].dist = dist_after;
          candidates[i].adj_pen = adj_pen;
          candidates[i].score = score;
          candidates[i].feasible = 1;
          candidates[i].push_from = push_from;
          candidates[i].box_next.row = new_box_row;
          candidates[i].box_next.col = new_box_col;
        }
        
        int sorted_dirs[4];
        int sorted_count = 0;
        for (int i = 0; i < 4; ++i) {
          if (candidates[i].feasible) {
            sorted_dirs[sorted_count++] = i;
          }
        }
        
        if (sorted_count == 0) {
          last_err_stage = 2;
          last_err_detail = 116; // v3: 推箱子评分策略无可行方向，不再计算特殊路径
          return -6;
        }
        
        // 排序（按评分从优到劣）
        for (int i = 0; i < sorted_count - 1; ++i) {
          for (int j = 0; j < sorted_count - 1 - i; ++j) {
            if (candidates[sorted_dirs[j]].score > candidates[sorted_dirs[j + 1]].score) {
              int tmp = sorted_dirs[j];
              sorted_dirs[j] = sorted_dirs[j + 1];
              sorted_dirs[j + 1] = tmp;
            }
          }
        }
        
        // 按优劣依次尝试各方向，直到有一个可行（仅做可达性检查，不写入 path_buffer）
        DirCandidate chosen;
        int dir_ok = 0;
        for (int k = 0; k < sorted_count; ++k) {
          chosen = candidates[sorted_dirs[k]];
          int next_on_boundary = (chosen.box_next.row == 0 || chosen.box_next.row == rows - 1 ||
                                  chosen.box_next.col == 0 || chosen.box_next.col == cols - 1);
          if (next_on_boundary) {
            if (!planner_v3_bfs_validate_boundary_path(rows, cols, chosen.box_next, target,
                                                      current_car, current_obstacles, current_obstacle_count,
                                                      bombs_mutable, bomb_count, current_boxes, goal_count,
                                                      box_idx, 1)) {
              continue;
            }
          }
          if (planner_v3_bfs_distance_between(rows, cols, current_car, chosen.push_from,
                                              current_obstacles, current_obstacle_count,
                                              bombs_mutable, bomb_count, current_boxes, goal_count, 1) == INT_MAX) {
            continue;
          }
          dir_ok = 1;
          break;
        }
        if (!dir_ok) {
          last_err_stage = 2;
          last_err_detail = 127; // v3: 推箱子评分策略无可行方向（dir_ok 分支），不再计算特殊路径
          return -6;
        }
        
        Point push_from = chosen.push_from;
        if (current_car.row != push_from.row || current_car.col != push_from.col) {
          int car_result = planner_v3_bfs_car_move_with_global_astar(rows, cols, &current_car, push_from,
                                                                     current_obstacles, current_obstacle_count,
                                                                     bombs_mutable, bomb_count,
                                                                     current_boxes, goal_count,
                                                                     path_buffer, path_capacity, out_steps, 1);
          if (car_result == -7) {
            return -7;
          }
          if (car_result == 0) {
            last_err_stage = 2;
            last_err_detail = 123; /* v3: 车无法到达推位（评分策略） */
            return -6;
          }
        }
        if (current_car.row != push_from.row || current_car.col != push_from.col) {
          last_err_stage = 2;
          last_err_detail = 130; /* v3: 车到推位后位置不一致（评分策略） */
          return -6;
        }
        if (!planner_v3_bfs_check_adjacent(current_car, box)) {
          last_err_stage = 2;
          last_err_detail = 125; /* v3: 车与箱子不相邻（评分策略） */
          return -6;
        }
        
        Point box_old_pos = box;
        box = chosen.box_next;
        current_boxes[box_idx] = box;
        
        if (box_path_lens[box_idx] < PLANNER_V3_BFS_MAX_CELLS) {
          box_paths[box_idx][box_path_lens[box_idx]++] = box;
        }
        
        current_car = box_old_pos;
        if (*out_steps >= path_capacity) {
          return -7;
        }
        if (*out_steps > 0 &&
            !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], current_car)) {
          last_err_stage = 2;
          last_err_detail = 126; // v3: 车路径不连续（评分策略推箱过程中）
          return -6;
        }
        path_buffer[(*out_steps)++] = current_car;
        
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
    
    // 箱子到达目标，标记为"消失"
    if (box.row == target.row && box.col == target.col) {
      current_boxes[box_idx].row = -1;
      current_boxes[box_idx].col = -1;
      completed_count++;
      
      if (box_path_lens[box_idx] < PLANNER_V3_BFS_MAX_CELLS) {
        box_paths[box_idx][box_path_lens[box_idx]++] = target;
      }
    } else {
      // 箱子未到达目标，报告错误
      last_err_stage = 2;
      last_err_detail = 117; // v3: 箱子最终未到达目标
      return -6;
    }
  }

  // 验证生成的路径是否连续
  if (*out_steps > 1 && !planner_v3_bfs_validate_continuous_path(path_buffer, *out_steps)) {
    last_err_stage = 2;
    last_err_detail = 118; // v3: 整体路径不连续
    return -6;
  }

  // 推完所有箱子后，使用BFS+A*返回起点
  if (current_car.row != car_start.row || current_car.col != car_start.col) {
    Point empty_boxes[PLANNER_V3_BFS_MAX_BOXES];
    size_t empty_box_count = 0;
    
    int dist[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_target(rows, cols, car_start, current_obstacles, current_obstacle_count,
                                               bombs_mutable, bomb_count, empty_boxes, empty_box_count, 
                                               dist, 1)) {
      // BFS失败，但不算错误，继续
    } else {
      Point return_path[256];
      size_t return_len = 0;
      if (planner_v3_bfs_astar_with_dist(rows, cols, current_car, car_start,
                                        current_obstacles, current_obstacle_count,
                                        bombs_mutable, bomb_count,
                                        empty_boxes, empty_box_count, dist,
                                        return_path, 256, &return_len,
                                        1, 0, NULL,
                                        0, 0, (Point){0, 0})) {
        for (size_t i = 1; i < return_len; ++i) {
          if (*out_steps >= path_capacity) {
            return -7;
          }
          if (*out_steps > 0 && 
              !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], return_path[i])) {
            last_err_stage = 2;
            last_err_detail = 119; // v3: 返回起点路径不连续
            return -6;
          }
          path_buffer[(*out_steps)++] = return_path[i];
        }
      }
    }
  }

  // 填充out_final_paths（索引与 PlannerPointV3_BFS boxes 一致）
  if (out_final_paths) {
    out_final_paths->box_count = box_count;
    for (size_t k = 0; k < box_count && k < PLANNER_V3_BFS_MAX_BOXES; ++k) {
      out_final_paths->box_paths[k].valid = 0;
      out_final_paths->box_paths[k].path_len = 0;
    }
    for (size_t i = 0; i < goal_count; ++i) {
      size_t bi = goals[i].source_idx;
      if (bi >= PLANNER_V3_BFS_MAX_BOXES)
        continue;
      out_final_paths->box_paths[bi].valid = (box_path_lens[i] > 0) ? 1 : 0;
      out_final_paths->box_paths[bi].path_len = box_path_lens[i];
      size_t copy_len = (box_path_lens[i] < 400) ? box_path_lens[i] : 400;
      for (size_t j = 0; j < copy_len; ++j) {
        out_final_paths->box_paths[bi].path[j] = box_paths[i][j];
      }
    }
  }

  return 0;
}

// 主函数接口
int plan_boxes_greedy_v3(int rows, int cols, PlannerPointV3_BFS car,
                         const PlannerPointV3_BFS *boxes, size_t box_count,
                         const PlannerPointV3_BFS *targets, size_t target_count,
                         const PlannerPointV3_BFS *bombs, size_t bomb_count,
                         const PlannerPointV3_BFS *obstacles,
                         size_t obstacle_count, PlannerPointV3_BFS *path_buffer,
                         size_t path_capacity, size_t *out_steps,
                         size_t *out_box_target_indices,
                         PlannerAllBoxPaths *out_final_paths) {
  if (!boxes || !targets || !path_buffer || !out_steps) {
    return -1;
  }
  *out_steps = 0;
  
  // 清空特殊路径数据（每次规划开始时重置，保持最新；已完成的箱子不清理，索引与 boxes 一致）
  planner_v3_bfs_clear_special_paths(boxes, targets, box_count, target_count);
  if (box_count == 0 || target_count == 0) {
    return -2;
  }
  if (box_count > PLANNER_V3_BFS_MAX_BOXES) {
    return -3;
  }
  if (path_capacity == 0) {
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

  // 创建可变的炸弹数组
  Point bombs_mutable[MAX_BOMBS];
  if (bomb_count > 0 && bombs) {
    memcpy(bombs_mutable, bombs, bomb_count * sizeof(Point));
  }

  PlannerBoxGoalBFS assigned[PLANNER_V3_BFS_MAX_BOXES];
  size_t assigned_count = 0;
  int assign_res = planner_v3_bfs_assign_targets(
      rows, cols, car, boxes, box_count, targets, target_count, obstacles, obstacle_count,
      bombs, bomb_count,
      assigned, &assigned_count, out_box_target_indices);
  if (assign_res != 0) {
    *out_steps = 0;
    return assign_res;
  }

  return planner_v3_bfs_run_assigned(rows, cols, car, assigned, assigned_count, box_count,
                                 obstacles, obstacle_count, bombs, bomb_count, path_buffer,
                                 path_capacity, out_steps, out_final_paths, bombs_mutable);
}
