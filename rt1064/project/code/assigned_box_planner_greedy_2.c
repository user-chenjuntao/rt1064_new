#include "assigned_box_planner_greedy_2.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

#define PLANNER_V3_BFS_MAX_BOXES 10
#define PLANNER_V3_BFS_MAX_CELLS 400
#define PLANNER_V3_BFS_MAX_GREEDY_STEPS 5000
// 路径最长不超过可达格子数量，这里按最大格子数限制，避免大数组造成栈溢出
#define PLANNER_V3_BFS_MAX_PATH_LEN PLANNER_V3_BFS_MAX_CELLS
// 内部控制码：请求外层回到“第一次路径规划（忽略箱子阻挡）”阶段重新计算
#define PLANNER_V3_BFS_REPLAN 1
// 链式箱子“被非链内箱子阻挡”时，对链内所有箱子增加的惩罚分（分数越小越优先）
// 取值应明显大于棋盘内 BFS 距离上限（<=PLANNER_V3_BFS_MAX_CELLS），以确保优先清障
#define PLANNER_V3_BFS_CHAIN_BLOCK_PENALTY 1000

typedef struct {
  Point path[PLANNER_V3_BFS_MAX_PATH_LEN];
  Point dir[PLANNER_V3_BFS_MAX_PATH_LEN];
  size_t len;
  int valid;
} PlannerBoxPathInfo;

// PlannerBoxOverlap 结构体定义在头文件中

// 链式箱子结构（本地定义，与头文件一致）
typedef struct {
  size_t indices[PLANNER_V3_BFS_MAX_BOXES];       // 链中箱子的索引，从主箱开始
  size_t overlap_lens[PLANNER_V3_BFS_MAX_BOXES];  // 每个箱子的重叠路径长度
  Point overlap_end_positions[PLANNER_V3_BFS_MAX_BOXES];  // 每个箱子的重叠路径末端坐标
  size_t steps_walked[PLANNER_V3_BFS_MAX_BOXES];  // 每个箱子已走的步数
  size_t count;                                    // 链的长度（包含主箱）
  Point dir;                                       // 当前推动方向
} PlannerChainLocal;

static PlannerBoxPathInfo g_planner_free_paths[PLANNER_V3_BFS_MAX_BOXES];
static PlannerBoxPathInfo g_planner_planned_paths[PLANNER_V3_BFS_MAX_BOXES];
static PlannerBoxOverlap g_planner_overlaps[PLANNER_V3_BFS_MAX_BOXES];

// 全局变量：重构链状态信息（供菜单显示）
PlannerRebuildChainStatus g_rebuild_chain_status;

// 全局错误信息变量
int last_err_stage = 0;   // 错误阶段：1=初始分配失败, 2=动态选目标失败, 3=第二次规划失败, 4=车辆路径连续性失败, 5=链式推动内部失败
int last_err_detail = 0;  // 错误详情：对应阶段的子错误码

// 外部变量：最终路径规划结果（供链式箱子使用）
extern PlannerAllBoxPaths final_paths;

// 前向声明
static int planner_v3_bfs_execute_single_box_path(int rows, int cols, size_t box_idx,
                                              const PlannerBoxPathInfo *path_info,
                                              const Point *obstacles, size_t obstacle_count,
                                              Point *current_car, Point *current_boxes,
                                              size_t box_count, Point *path_buffer,
                                              size_t path_capacity, size_t *out_steps);

static int planner_v3_bfs_push_single_box_scored(int rows, int cols, size_t box_idx,
                                                  size_t target_idx, const Point *targets,
                                                  const Point *obstacles, size_t obstacle_count,
                                                  Point *current_car, Point *current_boxes,
                                                  size_t box_count, Point *path_buffer,
                                                  size_t path_capacity, size_t *out_steps);

static int planner_v3_bfs_abs(int v) { return v >= 0 ? v : -v; }

static int planner_v3_bfs_manhattan(Point a, Point b) {
  return planner_v3_bfs_abs(a.row - b.row) + planner_v3_bfs_abs(a.col - b.col);
}

static int planner_v3_bfs_in_bounds(int rows, int cols, int row, int col) {
  return row >= 0 && row < rows && col >= 0 && col < cols;
}

static int planner_v3_bfs_is_obstacle(const Point *obstacles, size_t count, int row, int col) {
  for (size_t i = 0; i < count; ++i) {
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

static int planner_v3_bfs_find_box_index_at(const Point *boxes, size_t count, int row, int col,
                                            size_t skip_idx) {
  if (!boxes) return -1;
  for (size_t i = 0; i < count; ++i) {
    if (i == skip_idx) {
      continue;
    }
    if (boxes[i].row < 0 || boxes[i].col < 0) {
      continue;
    }
    if (boxes[i].row == row && boxes[i].col == col) {
      return (int)i;
    }
  }
  return -1;
}

static int planner_v3_bfs_is_cardinal_dir(Point dir) {
  return (dir.row == -1 && dir.col == 0) || (dir.row == 1 && dir.col == 0) ||
         (dir.row == 0 && dir.col == -1) || (dir.row == 0 && dir.col == 1);
}

// 若 to 位于 from 沿 dir 方向的正向射线上，则返回步数 k(>0)，否则返回 0
static int planner_v3_bfs_steps_along_dir(Point from, Point to, Point dir) {
  if (!planner_v3_bfs_is_cardinal_dir(dir)) return 0;
  if (dir.row != 0) {
    if (to.col != from.col) return 0;
    int dr = to.row - from.row;
    if (dr == 0) return 0;
    if ((dr > 0 && dir.row < 0) || (dr < 0 && dir.row > 0)) return 0;
    return planner_v3_bfs_abs(dr);  // dir.row 为 ±1
  } else {
    if (to.row != from.row) return 0;
    int dc = to.col - from.col;
    if (dc == 0) return 0;
    if ((dc > 0 && dir.col < 0) || (dc < 0 && dir.col > 0)) return 0;
    return planner_v3_bfs_abs(dc);  // dir.col 为 ±1
  }
}

// 规则：在“形成路径并标记（重叠/链式）”阶段，如果存在潜在链式关系 primary->secondary，
// 且两者之间（沿推动方向）夹着其它箱子（非该链的相邻副箱），则对该链内所有箱子增加惩罚分，
// 让贪心优先去清除阻碍链形成的箱子。
static void planner_v3_bfs_mark_obstructed_chain_boxes(const Point *boxes, size_t box_count,
                                                       const PlannerBoxOverlap *overlaps,
                                                       uint8_t *out_penalize) {
  if (!boxes || !overlaps || !out_penalize) return;
  for (size_t i = 0; i < box_count; ++i) out_penalize[i] = 0;

  // 仅从“非副箱”作为链头开始构建链（避免重复）
  for (size_t head = 0; head < box_count; ++head) {
    if (boxes[head].row < 0 || boxes[head].col < 0) continue;
    if (overlaps[head].valid && overlaps[head].primary != SIZE_MAX) continue;  // head 是副箱

    // head 可能对应多个方向的潜在链，逐方向尝试
    Point tried_dirs[4];
    size_t tried_n = 0;
    for (size_t s = 0; s < box_count; ++s) {
      if (!overlaps[s].valid || overlaps[s].primary != head) continue;
      Point dir = overlaps[s].dir;
      if (!planner_v3_bfs_is_cardinal_dir(dir)) continue;
      int seen = 0;
      for (size_t k = 0; k < tried_n; ++k) {
        if (tried_dirs[k].row == dir.row && tried_dirs[k].col == dir.col) {
          seen = 1;
          break;
        }
      }
      if (seen) continue;
      tried_dirs[tried_n++] = dir;
      if (tried_n >= 4) break;
    }

    for (size_t di = 0; di < tried_n; ++di) {
      Point chain_dir = tried_dirs[di];
      size_t chain_nodes[PLANNER_V3_BFS_MAX_BOXES];
      size_t chain_n = 0;
      int blocked = 0;

      size_t curr = head;
      chain_nodes[chain_n++] = curr;

      // 沿 overlaps 的 primary 指针按 chain_dir 串起来（允许中间有空格，但若夹箱则视为阻挡）
      while (chain_n < box_count) {
        // 找到 curr 的“最近的”下一个副箱（同方向）
        size_t next = SIZE_MAX;
        int best_k = INT_MAX;
        for (size_t s = 0; s < box_count; ++s) {
          if (!overlaps[s].valid || overlaps[s].primary != curr) continue;
          if (overlaps[s].dir.row != chain_dir.row || overlaps[s].dir.col != chain_dir.col) {
            continue;
          }
          if (boxes[s].row < 0 || boxes[s].col < 0) continue;
          int k = planner_v3_bfs_steps_along_dir(boxes[curr], boxes[s], chain_dir);
          if (k > 0 && k < best_k) {
            best_k = k;
            next = s;
          }
        }
        if (next == SIZE_MAX) {
          break;
        }

        // 检查 curr 与 next 之间是否夹了其它箱子（非该相邻副箱）
        if (best_k > 1) {
          for (int step = 1; step < best_k; ++step) {
            int rr = boxes[curr].row + chain_dir.row * step;
            int cc = boxes[curr].col + chain_dir.col * step;
            int mid = planner_v3_bfs_find_box_index_at(boxes, box_count, rr, cc, SIZE_MAX);
            if (mid >= 0 && (size_t)mid != next) {
              blocked = 1;
              break;
            }
          }
        }

        chain_nodes[chain_n++] = next;
        curr = next;
      }

      if (blocked && chain_n >= 2) {
        for (size_t k = 0; k < chain_n; ++k) {
          out_penalize[chain_nodes[k]] = 1;
        }
      }
    }
  }
}

static int planner_v3_bfs_is_box_at_mask(const Point *boxes, size_t count, int row, int col,
                             size_t skip_idx, const uint8_t *ignore_mask) {
  for (size_t i = 0; i < count; ++i) {
    if (i == skip_idx) {
      continue;
    }
    if (ignore_mask && ignore_mask[i]) {
      continue;
    }
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
                                   size_t obstacle_count, const Point *boxes,
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
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, nr, nc) ||
        planner_v3_bfs_is_box_at(boxes, box_count, nr, nc, skip_idx)) {
      count++;
    }
  }
  return count;
}

// 检查箱子从start是否能被推到target（不考虑死点，只检查物理可达性）
static int planner_v3_bfs_can_reach_goal_mask(int rows, int cols, const Point *obstacles,
                                size_t obstacle_count, const Point *boxes,
                                size_t box_count, size_t moving_idx,
                                const uint8_t *ignore_mask, Point start, Point target) {
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_BFS_MAX_CELLS || total_cells <= 0) {
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

      if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, new_row, new_col) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_row, push_col)) {
        continue;
      }

      if (planner_v3_bfs_is_box_at_mask(boxes, box_count, new_row, new_col, moving_idx,
                         ignore_mask) ||
          planner_v3_bfs_is_box_at_mask(boxes, box_count, push_row, push_col, moving_idx,
                         ignore_mask)) {
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

static int planner_v3_bfs_can_reach_goal(int rows, int cols, const Point *obstacles,
                                size_t obstacle_count, const Point *boxes,
                                size_t box_count, size_t moving_idx,
                                Point start, Point target) {
  return planner_v3_bfs_can_reach_goal_mask(rows, cols, obstacles, obstacle_count, boxes,
                                 box_count, moving_idx, NULL, start, target);
}

// 全局BFS：从终点开始，计算每个可达格子到终点的真实距离
// 返回值：1=成功，0=失败
static int planner_v3_bfs_global_bfs_from_target(int rows, int cols, Point target,
                                             const Point *obstacles, size_t obstacle_count,
                                             const Point *boxes, size_t box_count,
                                             size_t skip_box_idx,
                                             int dist[PLANNER_V3_BFS_MAX_CELLS]) {
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_BFS_MAX_CELLS || total_cells <= 0) {
    return 0;
  }
  
  // 初始化距离为无穷大
  for (int i = 0; i < total_cells; ++i) {
    dist[i] = INT_MAX;
  }
  
  // 【入口判定】检查 target 是否可站立
  // 若 target 越界 / 是障碍 / 被任意箱子占用（skip_box_idx 之外）
  // → 直接返回 0（保持 dist 全 INT_MAX）
  if (!planner_v3_bfs_in_bounds(rows, cols, target.row, target.col)) {
    return 0;
  }
  if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, target.row, target.col)) {
    return 0;
  }
  if (planner_v3_bfs_is_box_at(boxes, box_count, target.row, target.col, skip_box_idx)) {
    return 0;
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
      if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, nr, nc)) {
        continue;
      }
      if (planner_v3_bfs_is_box_at(boxes, box_count, nr, nc, skip_box_idx)) {
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
// is_first_plan: 1=第一次路径规划（车辆移动），0=第二次路径规划（绕箱子）
// 返回值：1=成功找到路径，0=失败
static int planner_v3_bfs_astar_with_dist(int rows, int cols, Point start, Point target,
                                      const Point *obstacles, size_t obstacle_count,
                                      const Point *boxes, size_t box_count,
                                      const int dist[PLANNER_V3_BFS_MAX_CELLS],
                                      Point *path, size_t path_cap, size_t *path_len,
                                      int is_first_plan) {
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
    int box_count_on_path;  // 路径上的箱子数量（第一次路径规划用）
    int direction_changes;   // 方向变换数量（第二次路径规划用）
  } AStarNode;
  
  int g_score[PLANNER_V3_BFS_MAX_CELLS];
  int parent[PLANNER_V3_BFS_MAX_CELLS];
  int box_count_on_path[PLANNER_V3_BFS_MAX_CELLS];  // 到达该节点的路径上的箱子数量
  int direction_changes[PLANNER_V3_BFS_MAX_CELLS];  // 到达该节点的路径上的方向变换数量
  int last_dir[PLANNER_V3_BFS_MAX_CELLS];  // 到达该节点的最后一步方向（-1表示未设置）
  uint8_t in_open[PLANNER_V3_BFS_MAX_CELLS];
  uint8_t in_closed[PLANNER_V3_BFS_MAX_CELLS];
  
  for (int i = 0; i < total_cells; ++i) {
    g_score[i] = INT_MAX;
    parent[i] = -1;
    box_count_on_path[i] = 0;
    direction_changes[i] = 0;
    last_dir[i] = -1;
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
  box_count_on_path[start_idx] = 0;
  direction_changes[start_idx] = 0;
  last_dir[start_idx] = -1;
  open_set[open_count].idx = start_idx;
  open_set[open_count].f_score = dist[start_idx];  // f = g + h = 0 + h
  open_set[open_count].box_count_on_path = 0;
  open_set[open_count].direction_changes = 0;
  open_count++;
  in_open[start_idx] = 1;
  
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  
  while (open_count > 0) {
    // 找到f_score最小的节点，如果f_score相同则使用不同的比较逻辑
    int best_idx = 0;
    for (int i = 1; i < open_count; ++i) {
      int cmp = 0;
      if (open_set[i].f_score < open_set[best_idx].f_score) {
        cmp = -1;
      } else if (open_set[i].f_score > open_set[best_idx].f_score) {
        cmp = 1;
      } else {
        // f_score相同，根据规划类型使用不同的比较逻辑
        if (is_first_plan) {
          // 第一次路径规划：优先选择路径上箱子数量最多的
          if (open_set[i].box_count_on_path > open_set[best_idx].box_count_on_path) {
            cmp = -1;
          } else if (open_set[i].box_count_on_path < open_set[best_idx].box_count_on_path) {
            cmp = 1;
          }
        } else {
          // 第二次路径规划：优先选择方向变换数量最少的
          if (open_set[i].direction_changes < open_set[best_idx].direction_changes) {
            cmp = -1;
          } else if (open_set[i].direction_changes > open_set[best_idx].direction_changes) {
            cmp = 1;
          }
        }
      }
      if (cmp < 0) {
        best_idx = i;
      }
    }
    
    AStarNode current = open_set[best_idx];
    int curr_idx = current.idx;
    
    // 从open_set中移除
    for (int i = best_idx; i < open_count - 1; ++i) {
      open_set[i] = open_set[i + 1];
    }
    open_count--;
    in_open[curr_idx] = 0;
    in_closed[curr_idx] = 1;
    
    // 到达目标
    if (curr_idx == target_idx) {
      // 回溯路径
      int path_indices[PLANNER_V3_BFS_MAX_PATH_LEN];
      int path_count = 0;
      int idx = target_idx;
      
      while (idx != -1 && path_count < PLANNER_V3_BFS_MAX_PATH_LEN) {
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
      
      if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, nr, nc)) {
        continue;
      }
      if (planner_v3_bfs_is_box_at(boxes, box_count, nr, nc, box_count)) {
        continue;
      }
      
      int next_idx = nr * cols + nc;
      if (next_idx < 0 || next_idx >= total_cells) {
        continue;
      }
      
      if (in_closed[next_idx]) {
        continue;
      }
      
      int tentative_g = g_score[curr_idx] + 1;
      
      // 计算当前移动方向
      int dir_code = -1;
      if (dirs[d][0] == -1 && dirs[d][1] == 0) dir_code = 0;  // 上
      else if (dirs[d][0] == 1 && dirs[d][1] == 0) dir_code = 1;  // 下
      else if (dirs[d][0] == 0 && dirs[d][1] == -1) dir_code = 2;  // 左
      else if (dirs[d][0] == 0 && dirs[d][1] == 1) dir_code = 3;  // 右
      
      // 计算路径上的箱子数量（第一次路径规划）
      int new_box_count = box_count_on_path[curr_idx];
      if (is_first_plan && planner_v3_bfs_is_box_at(boxes, box_count, nr, nc, SIZE_MAX)) {
        new_box_count++;
      }
      
      // 计算方向变换数量（第二次路径规划）
      int new_dir_changes = direction_changes[curr_idx];
      if (!is_first_plan && last_dir[curr_idx] != -1 && last_dir[curr_idx] != dir_code) {
        new_dir_changes++;
      }
      
      if (!in_open[next_idx]) {
        // 新节点
        g_score[next_idx] = tentative_g;
        parent[next_idx] = curr_idx;
        box_count_on_path[next_idx] = new_box_count;
        direction_changes[next_idx] = new_dir_changes;
        last_dir[next_idx] = dir_code;
        
        // h = dist[next_idx]（预计算的真实距离）
        int h = dist[next_idx];
        if (h == INT_MAX) {
          continue;  // 该点无法到达终点，跳过
        }
        
        int f = tentative_g + h;
        
        if (open_count < PLANNER_V3_BFS_MAX_CELLS) {
          open_set[open_count].idx = next_idx;
          open_set[open_count].f_score = f;
          open_set[open_count].box_count_on_path = new_box_count;
          open_set[open_count].direction_changes = new_dir_changes;
          open_count++;
          in_open[next_idx] = 1;
        }
      } else if (tentative_g < g_score[next_idx]) {
        // 找到更好的路径（g_score更小）
        g_score[next_idx] = tentative_g;
        parent[next_idx] = curr_idx;
        box_count_on_path[next_idx] = new_box_count;
        direction_changes[next_idx] = new_dir_changes;
        last_dir[next_idx] = dir_code;
        
        int h = dist[next_idx];
        int f = tentative_g + h;
        
        // 更新open_set中的f_score和额外信息
        for (int i = 0; i < open_count; ++i) {
          if (open_set[i].idx == next_idx) {
            open_set[i].f_score = f;
            open_set[i].box_count_on_path = new_box_count;
            open_set[i].direction_changes = new_dir_changes;
            break;
          }
        }
      } else if (tentative_g == g_score[next_idx]) {
        // g_score相同，根据规划类型判断是否需要更新
        int should_update = 0;
        if (is_first_plan) {
          // 第一次路径规划：如果路径上箱子数量更多，则更新
          if (new_box_count > box_count_on_path[next_idx]) {
            should_update = 1;
          }
        } else {
          // 第二次路径规划：如果方向变换数量更少，则更新
          if (new_dir_changes < direction_changes[next_idx]) {
            should_update = 1;
          }
        }
        
        if (should_update) {
          parent[next_idx] = curr_idx;
          box_count_on_path[next_idx] = new_box_count;
          direction_changes[next_idx] = new_dir_changes;
          last_dir[next_idx] = dir_code;
          
          // 更新open_set中的额外信息（f_score不变）
          for (int i = 0; i < open_count; ++i) {
            if (open_set[i].idx == next_idx) {
              open_set[i].box_count_on_path = new_box_count;
              open_set[i].direction_changes = new_dir_changes;
              break;
            }
          }
        }
      }
    }
  }
  
  // 没有找到路径
  *path_len = 0;
  return 0;
}

// 验证新添加的点与前一个点是否相邻
static int planner_v3_bfs_check_adjacent(Point prev, Point curr) {
  int dr = planner_v3_bfs_abs(curr.row - prev.row);
  int dc = planner_v3_bfs_abs(curr.col - prev.col);
  return (dr + dc == 1);
}

// 使用全局BFS+A*绕箱子到另一个推箱位
// 返回值：1=成功，0=失败
static int planner_v3_bfs_follow_box_with_global_astar(int rows, int cols, Point car, Point box,
                                                     Point target, const Point *obstacles,
                                                     size_t obstacle_count, const Point *boxes,
                                                     size_t box_count, size_t moving_idx,
                                                     Point *path, size_t path_cap,
                                                     size_t *path_len, Point *out_push_pos) {
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  
  // 找到所有有效的推箱位
  Point valid_push_positions[4];
  int valid_count = 0;
  
  for (int d = 0; d < 4; ++d) {
    int push_r = box.row - dirs[d][0];
    int push_c = box.col - dirs[d][1];
    int new_box_r = box.row + dirs[d][0];
    int new_box_c = box.col + dirs[d][1];
    
    if (!planner_v3_bfs_in_bounds(rows, cols, push_r, push_c)) continue;
    if (!planner_v3_bfs_in_bounds(rows, cols, new_box_r, new_box_c)) continue;
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_r, push_c)) continue;
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, new_box_r, new_box_c)) continue;
    if (planner_v3_bfs_is_box_at(boxes, box_count, new_box_r, new_box_c, moving_idx)) continue;
    
    // 检查推过去后箱子能否继续到目标
    if (planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count, boxes,
                                   box_count, moving_idx,
                                   (Point){new_box_r, new_box_c}, target)) {
      valid_push_positions[valid_count].row = push_r;
      valid_push_positions[valid_count].col = push_c;
      valid_count++;
    }
  }
  
  if (valid_count == 0) {
    return 0;  // 没有有效的推箱位
  }
  
  // 按距离排序推箱位
  typedef struct {
    Point pos;
    int dist;
  } PushPosWithDist;
  
  PushPosWithDist sorted_positions[4];
  for (int i = 0; i < valid_count; ++i) {
    sorted_positions[i].pos = valid_push_positions[i];
    sorted_positions[i].dist = planner_v3_bfs_manhattan(car, valid_push_positions[i]);
  }
  
  for (int i = 0; i < valid_count - 1; ++i) {
    for (int j = 0; j < valid_count - 1 - i; ++j) {
      if (sorted_positions[j].dist > sorted_positions[j + 1].dist) {
        PushPosWithDist tmp = sorted_positions[j];
        sorted_positions[j] = sorted_positions[j + 1];
        sorted_positions[j + 1] = tmp;
      }
    }
  }
  
  // 尝试所有有效推箱位，使用全局BFS+A*
  for (int try_idx = 0; try_idx < valid_count; ++try_idx) {
    Point target_push_pos = sorted_positions[try_idx].pos;
    
    // 第1步：从目标推箱位做全局BFS
    int dist[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target_push_pos, obstacles, 
                                           obstacle_count, boxes, box_count, SIZE_MAX, dist)) {
      continue;  // BFS失败，尝试下一个推箱位
    }
    
    // 第2步：使用A*搜索路径（第二次路径规划：绕箱子）
    if (!planner_v3_bfs_astar_with_dist(rows, cols, car, target_push_pos,
                                    obstacles, obstacle_count,
                                    boxes, box_count, dist,
                                    path, path_cap, path_len, 0)) {
      continue;  // A*失败，尝试下一个推箱位
    }
    
    if (*path_len > 0) {
      Point last = path[*path_len - 1];
      if (last.row == target_push_pos.row && last.col == target_push_pos.col) {
        // 成功到达推箱位
        out_push_pos->row = target_push_pos.row;
        out_push_pos->col = target_push_pos.col;
        return 1;
      }
    }
  }
  
  // 所有推箱位都无法到达
  return 0;
}

// 使用全局BFS+A*移动车到目标位置
// 返回值：1=成功，0=失败，-7=路径缓存不足
static int planner_v3_bfs_car_move_with_global_astar(int rows, int cols, Point *car_pos, 
                                                  Point target,
                                                  const Point *obstacles, size_t obstacle_count,
                                                  const Point *boxes, size_t box_count,
                                                  Point *path_buffer, size_t path_capacity,
                                                  size_t *out_steps) {
  if (!planner_v3_bfs_in_bounds(rows, cols, target.row, target.col)) {
    return 0;
  }

  if (car_pos->row == target.row && car_pos->col == target.col) {
    return 1;  // 已在目标位置
  }
  
  // 第1步：从目标做全局BFS
  int dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                         boxes, box_count, SIZE_MAX, dist)) {
    return 0;  // BFS失败
  }
  
  // 第2步：使用A*搜索路径（第一次路径规划：车辆移动）
  Point temp_path[256];
  size_t temp_len = 0;
  
  if (!planner_v3_bfs_astar_with_dist(rows, cols, *car_pos, target,
                                  obstacles, obstacle_count,
                                  boxes, box_count, dist,
                                  temp_path, 256, &temp_len, 1)) {
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

static void planner_v3_bfs_fill_dirs(PlannerBoxPathInfo *info) {
  if (!info) {
    return;
  }
  for (size_t i = 1; i < info->len; ++i) {
    info->dir[i - 1].row = info->path[i].row - info->path[i - 1].row;
    info->dir[i - 1].col = info->path[i].col - info->path[i - 1].col;
  }
}

static int planner_v3_bfs_validate_push_positions(int rows, int cols,
                                              const PlannerBoxPathInfo *path_info,
                                              const Point *obstacles, size_t obstacle_count,
                                              const Point *boxes, size_t box_count,
                                              size_t moving_idx) {
  if (!path_info || path_info->len < 2) {
    return 1;
  }

  for (size_t step = 1; step < path_info->len; ++step) {
    Point curr = path_info->path[step - 1];
    Point next = path_info->path[step];
    Point push_pos = {curr.row - (next.row - curr.row), curr.col - (next.col - curr.col)};

    if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col)) {
      return 0;
    }
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_pos.row, push_pos.col)) {
      return 0;
    }
    if (planner_v3_bfs_is_box_at(boxes, box_count, push_pos.row, push_pos.col, moving_idx)) {
      return 0;
    }
  }

  return 1;
}

static void planner_v3_bfs_build_temp_boxes(const Point *boxes, size_t box_count,
                                        const uint8_t *ignore_mask, size_t moving_idx,
                                        Point *out) {
  for (size_t i = 0; i < box_count; ++i) {
    if ((ignore_mask && ignore_mask[i]) || i == moving_idx) {
      out[i].row = -1;
      out[i].col = -1;
    } else {
      // 如果箱子已完成（负坐标），也标记为忽略
      if (boxes[i].row < 0 || boxes[i].col < 0) {
        out[i].row = -1;
        out[i].col = -1;
      } else {
        out[i] = boxes[i];
      }
    }
  }
}

static int planner_v3_bfs_compute_box_path_with_mask(int rows, int cols, Point start,
                                                 Point target, const Point *obstacles,
                                                 size_t obstacle_count, const Point *boxes,
                                                 size_t box_count,
                                                 const uint8_t *ignore_mask, size_t moving_idx,
                                                 PlannerBoxPathInfo *out) {
  if (!out) {
    return 0;
  }
  out->len = 0;
  out->valid = 0;

  Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
  planner_v3_bfs_build_temp_boxes(boxes, box_count, ignore_mask, moving_idx, temp_boxes);

  int dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                             temp_boxes, box_count, moving_idx, dist)) {
    return 0;
  }

  size_t path_len = 0;
  if (!planner_v3_bfs_astar_with_dist(rows, cols, start, target, obstacles, obstacle_count,
                                      temp_boxes, box_count, dist, out->path,
                                      PLANNER_V3_BFS_MAX_PATH_LEN, &path_len, 1)) {
    return 0;
  }

  out->len = path_len;
  planner_v3_bfs_fill_dirs(out);
  out->valid = (path_len > 0) &&
               planner_v3_bfs_validate_push_positions(rows, cols, out, obstacles,
                                                       obstacle_count, temp_boxes, box_count,
                                                       moving_idx);
  return out->valid;
}

// 评分：车到箱子的 BFS 真实距离（按“到达该箱子任意推箱位”的最短距离计算）
static int planner_v3_bfs_score_car_to_box(int rows, int cols, Point car, size_t box_idx,
                                       const Point *current_boxes, size_t box_count,
                                       const Point *obstacles, size_t obstacle_count) {
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
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_pos.row, push_pos.col)) {
      continue;
    }
    if (planner_v3_bfs_is_box_at(current_boxes, box_count, push_pos.row, push_pos.col, SIZE_MAX)) {
      continue;
    }

    int dist_map[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_target(rows, cols, push_pos, obstacles, obstacle_count,
                                               current_boxes, box_count, SIZE_MAX, dist_map)) {
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

// 初始分配：
// 1) 按“车到箱子 BFS 真实距离”对箱子排序（小的优先）
// 2) 依次为箱子从空闲目标里选“箱子到目标 BFS 真实距离最近且可达”的目标
static int planner_v3_bfs_initial_assign_targets(int rows, int cols, Point car,
                                             const Point *current_boxes, size_t box_count,
                                             const Point *targets, size_t target_count,
                                             const Point *obstacles, size_t obstacle_count,
                                             size_t *box_targets) {
  if (!current_boxes || !targets || !box_targets) {
    return -1;
  }
  if (box_count == 0 || target_count == 0) {
    return -2;
  }
  if (box_count > PLANNER_V3_BFS_MAX_BOXES) {
    return -3;
  }
  if (target_count < box_count) {
    return -8;
  }
  if (target_count > PLANNER_V3_BFS_MAX_CELLS) {
    return -1;
  }

  // 初始化
  for (size_t i = 0; i < box_count; ++i) {
    box_targets[i] = SIZE_MAX;
  }

  // 构建箱子排序
  size_t order[PLANNER_V3_BFS_MAX_BOXES];
  int scores[PLANNER_V3_BFS_MAX_BOXES];
  size_t n = 0;
  for (size_t i = 0; i < box_count; ++i) {
    if (current_boxes[i].row < 0 || current_boxes[i].col < 0) {
      continue;
    }
    order[n] = i;
    scores[n] = planner_v3_bfs_score_car_to_box(rows, cols, car, i, current_boxes, box_count,
                                                obstacles, obstacle_count);
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
  uint8_t ignore_all[PLANNER_V3_BFS_MAX_BOXES];
  for (size_t k = 0; k < box_count; ++k) {
    ignore_all[k] = 1;
  }

  // 依序分配目标
  for (size_t oi = 0; oi < n; ++oi) {
    size_t bi = order[oi];
    PlannerBoxPathInfo best_path;
    best_path.valid = 0;
    best_path.len = 0;
    size_t best_t = SIZE_MAX;
    int best_steps = INT_MAX;

    for (size_t t = 0; t < target_count; ++t) {
      if (target_used[t]) {
        continue;
      }
      PlannerBoxPathInfo tmp;
      if (!planner_v3_bfs_compute_box_path_with_mask(
              rows, cols, current_boxes[bi], targets[t], obstacles, obstacle_count,
              current_boxes, box_count, ignore_all, bi, &tmp)) {
        continue;
      }
      int steps = (tmp.len > 0) ? (int)(tmp.len - 1) : INT_MAX;
      if (steps < best_steps || (steps == best_steps && t < best_t)) {
        best_steps = steps;
        best_t = t;
        best_path = tmp;
      }
    }

    if (best_t == SIZE_MAX) {
      last_err_stage = 1;  // 初始分配失败
      last_err_detail = 1; // 找不到可行目标
      return -6;
    }
    (void)best_path;  // 初始分配阶段只需要目标索引，具体路径后续会重算
    box_targets[bi] = best_t;
    target_used[best_t] = 1;
  }

  return 0;
}

// 对链内目标做一次再分配：从链尾开始依次选“距离自己最近的目标”（仅在链内目标集合里重分配）
// 成功重分配后返回 1，失败返回 0
static int planner_v3_bfs_chain_reassign_targets_tail_to_head(int rows, int cols,
                                                           const PlannerChainLocal *chain,
                                                           const Point *current_boxes,
                                                           size_t box_count,
                                                           size_t *box_targets,
                                                           const Point *targets,
                                                           const Point *obstacles,
                                                           size_t obstacle_count) {
  if (!chain || chain->count == 0 || !current_boxes || !box_targets || !targets) {
    return 0;
  }

  // 收集链内“当前占用的目标集合”（去重）
  size_t chain_targets[PLANNER_V3_BFS_MAX_BOXES];
  size_t chain_tn = 0;
  for (size_t ci = 0; ci < chain->count; ++ci) {
    size_t bi = chain->indices[ci];
    if (box_targets[bi] == SIZE_MAX) {
      return 0;
    }
    size_t t = box_targets[bi];
    int dup = 0;
    for (size_t k = 0; k < chain_tn; ++k) {
      if (chain_targets[k] == t) {
        dup = 1;
        break;
      }
    }
    if (!dup) {
      chain_targets[chain_tn++] = t;
    }
  }
  if (chain_tn < chain->count) {
    // 链内本身就存在重复目标，无法在“仅链内目标集合”里分配出一一对应
    return 0;
  }

  uint8_t used[PLANNER_V3_BFS_MAX_BOXES] = {0};
  uint8_t ignore_all[PLANNER_V3_BFS_MAX_BOXES];
  for (size_t i = 0; i < box_count; ++i) {
    ignore_all[i] = 1;
  }

  int prev_dist = -1;
  int changed = 0;

  for (int ci = (int)chain->count - 1; ci >= 0; --ci) {
    size_t bi = chain->indices[ci];
    Point bp = current_boxes[bi];
    if (bp.row < 0 || bp.col < 0) {
      continue;
    }

    size_t best_k = SIZE_MAX;
    int best_steps = INT_MAX;
    size_t best_target = SIZE_MAX;

    // 先尝试满足“单调递增约束”的最近目标
    for (size_t k = 0; k < chain_tn; ++k) {
      if (used[k]) continue;
      size_t t = chain_targets[k];
      PlannerBoxPathInfo tmp;
      if (!planner_v3_bfs_compute_box_path_with_mask(
              rows, cols, bp, targets[t], obstacles, obstacle_count,
              current_boxes, box_count, ignore_all, bi, &tmp)) {
        continue;
      }
      int steps = (tmp.len > 0) ? (int)(tmp.len - 1) : INT_MAX;
      if (prev_dist >= 0 && steps < prev_dist) {
        continue;
      }
      if (steps < best_steps || (steps == best_steps && t < best_target)) {
        best_steps = steps;
        best_k = k;
        best_target = t;
      }
    }

    // 如果不存在满足约束的目标，退化为“最近目标”（严格按需求描述）
    if (best_k == SIZE_MAX) {
      for (size_t k = 0; k < chain_tn; ++k) {
        if (used[k]) continue;
        size_t t = chain_targets[k];
        PlannerBoxPathInfo tmp;
        if (!planner_v3_bfs_compute_box_path_with_mask(
                rows, cols, bp, targets[t], obstacles, obstacle_count,
                current_boxes, box_count, ignore_all, bi, &tmp)) {
          continue;
        }
        int steps = (tmp.len > 0) ? (int)(tmp.len - 1) : INT_MAX;
        if (steps < best_steps || (steps == best_steps && t < best_target)) {
          best_steps = steps;
          best_k = k;
          best_target = t;
        }
      }
    }

    if (best_k == SIZE_MAX || best_target == SIZE_MAX) {
      return 0;
    }

    if (box_targets[bi] != best_target) {
      box_targets[bi] = best_target;
      changed = 1;
    }
    used[best_k] = 1;
    prev_dist = best_steps;
  }

  return changed ? 1 : 0;
}

// 前向声明
static void planner_v3_bfs_collect_all_secondaries_recursive(
    size_t primary_idx, const PlannerBoxOverlap *overlaps, size_t box_count,
    uint8_t *ignore_mask);

// 检查箱子是否已完成
static int planner_v3_bfs_is_box_done(size_t box_idx, const Point *current_boxes, 
                                      const size_t *box_targets, const Point *targets) {
  // 已完成的箱子通常被标记为负坐标
  if (current_boxes[box_idx].row < 0 || current_boxes[box_idx].col < 0) {
    return 1;
  }
  // 检查箱子是否已到达目标位置
  if (box_targets && targets && box_targets[box_idx] != SIZE_MAX) {
    Point box = current_boxes[box_idx];
    Point tgt = targets[box_targets[box_idx]];
    if (box.row == tgt.row && box.col == tgt.col) {
      return 1;
    }
  }
  return 0;
}

static void planner_v3_bfs_detect_overlaps(int rows, int cols, Point car,
                                       const PlannerBoxPathInfo *free_paths,
                                       const PlannerBoxPathInfo *planned_paths,
                                       const Point *current_boxes, size_t box_count,
                                       const Point *obstacles, size_t obstacle_count,
                                       const size_t *box_targets, const Point *targets,
                                       PlannerBoxOverlap *overlaps) {
  // 保存已完成箱子的 overlap_end_pos
  Point saved_overlap_end_pos[PLANNER_V3_BFS_MAX_BOXES];
  for (size_t i = 0; i < box_count && i < PLANNER_V3_BFS_MAX_BOXES; ++i) {
    if (planner_v3_bfs_is_box_done(i, current_boxes, box_targets, targets) && 
        overlaps[i].overlap_end_pos.row >= 0 && overlaps[i].overlap_end_pos.col >= 0) {
      saved_overlap_end_pos[i] = overlaps[i].overlap_end_pos;
    } else {
      saved_overlap_end_pos[i].row = -1;
      saved_overlap_end_pos[i].col = -1;
    }
  }
  
  for (size_t i = 0; i < box_count; ++i) {
    overlaps[i].valid = 0;
    overlaps[i].primary = SIZE_MAX;
    overlaps[i].primary_start = 0;
    overlaps[i].secondary_start = 0;
    overlaps[i].overlap_len = 0;
    overlaps[i].overlap_len_second = 0;
    overlaps[i].overlap_end_pos.row = -1;
    overlaps[i].overlap_end_pos.col = -1;
    overlaps[i].dir.row = 0;
    overlaps[i].dir.col = 0;
  }
  
  // 恢复已完成箱子的 overlap_end_pos
  for (size_t i = 0; i < box_count && i < PLANNER_V3_BFS_MAX_BOXES; ++i) {
    if (saved_overlap_end_pos[i].row >= 0 && saved_overlap_end_pos[i].col >= 0) {
      overlaps[i].overlap_end_pos = saved_overlap_end_pos[i];
    }
  }
  
  // 如果传入了planned_paths（第二次路径规划，与final_paths相同），则使用它来计算重叠路径末端坐标
  // 如果没有传入（为NULL），则使用第一次路径规划（free_paths）来检测基础重叠关系

  // 第一遍遍历：构建基础的主箱-副箱关系（直接关系，第一次路径规划只保留最基础的overlap）
  for (size_t primary = 0; primary < box_count; ++primary) {
    if (!free_paths[primary].valid) {
      continue;
    }
    for (size_t secondary = 0; secondary < box_count; ++secondary) {
      if (primary == secondary) {
        continue;
      }
      if (!free_paths[secondary].valid) {
        continue;
      }
      Point sec_start = current_boxes[secondary];
      if (sec_start.row < 0 || sec_start.col < 0) {
        continue;
      }

      for (size_t pi = 0; pi < free_paths[primary].len; ++pi) {
        if (free_paths[primary].path[pi].row != sec_start.row ||
            free_paths[primary].path[pi].col != sec_start.col) {
          continue;
        }
        size_t sec_idx = 0;
        size_t overlap_len = 0;
        while (pi + overlap_len + 1 < free_paths[primary].len &&
               sec_idx + overlap_len + 1 < free_paths[secondary].len) {
          Point dir_p = free_paths[primary].dir[pi + overlap_len];
          Point dir_s = free_paths[secondary].dir[sec_idx + overlap_len];
          Point next_p = free_paths[primary].path[pi + overlap_len + 1];
          Point next_s = free_paths[secondary].path[sec_idx + overlap_len + 1];
          if (dir_p.row != dir_s.row || dir_p.col != dir_s.col) {
            break;
          }
          if (next_p.row != next_s.row || next_p.col != next_s.col) {
            break;
          }
          overlap_len++;
        }

        if (overlap_len > 0) {
          PlannerBoxOverlap *ov = &overlaps[secondary];
          // 计算当前主箱到副箱的距离
          Point primary_pos = current_boxes[primary];
          Point secondary_pos = current_boxes[secondary];
          int current_dist = planner_v3_bfs_manhattan(primary_pos, secondary_pos);
          
          // 优先选择距离副箱更近的主箱（而不是重叠长度更长的）
          if (!ov->valid) {
            // 还没有主箱关系，直接设置
            ov->valid = 1;
            ov->primary = primary;
            ov->primary_start = pi;
            ov->secondary_start = 0;
            ov->overlap_len = overlap_len;
            ov->dir = free_paths[primary].dir[pi];
          } else {
            // 已有主箱关系，计算之前主箱到副箱的距离
            Point old_primary_pos = current_boxes[ov->primary];
            int old_dist = planner_v3_bfs_manhattan(old_primary_pos, secondary_pos);
            
            // 如果当前主箱距离更近，则更新；如果距离相同，则选择重叠更短的（重叠更短优先），重叠长度相同则路径起始位置更早的优先
            if (current_dist < old_dist ||
                (current_dist == old_dist && overlap_len < ov->overlap_len) ||
                (current_dist == old_dist && overlap_len == ov->overlap_len && pi < ov->primary_start)) {
              ov->valid = 1;
              ov->primary = primary;
              ov->primary_start = pi;
              ov->secondary_start = 0;
              ov->overlap_len = overlap_len;
              ov->dir = free_paths[primary].dir[pi];
            }
          }
        }
        break;  // 只考虑首次经过secondary起点的重叠
      }
    }
  }
  
  // 第二遍：计算第二次路径规划的重叠长度（使用箱子的路径，而不是车辆的路径）
  // 只有在传入了planned_paths时才计算重叠路径末端坐标
  if (planned_paths) {
    for (size_t secondary = 0; secondary < box_count; ++secondary) {
      PlannerBoxOverlap *ov = &overlaps[secondary];
      if (!ov->valid || ov->primary == SIZE_MAX) {
        continue;
      }
      
      // 对于已完成的箱子，跳过更新 overlap_end_pos，保留之前的值
      if (planner_v3_bfs_is_box_done(secondary, current_boxes, box_targets, targets)) {
        continue;
      }
      
      // 计算第二次路径规划的重叠长度（使用直接primary和副箱的箱子路径）
      // 使用传入的planned_paths（与final_paths相同）
      size_t overlap_len_second = 0;
      if (planned_paths[ov->primary].valid && planned_paths[secondary].valid) {
        // 找到两个箱子路径的起点（箱子当前位置）
        Point sec_start = current_boxes[secondary];
        size_t box_path_primary_start = SIZE_MAX;
        size_t box_path_secondary_start = 0;  // secondary路径从0开始就是箱子当前位置
        
        const PlannerBoxPathInfo *primary_box_path = &planned_paths[ov->primary];
        
        // 在primary路径中查找secondary的起始位置
        for (size_t i = 0; i < primary_box_path->len; ++i) {
          if (primary_box_path->path[i].row == sec_start.row &&
              primary_box_path->path[i].col == sec_start.col) {
            box_path_primary_start = i;
            break;
          }
        }
        
        // 如果找到了起点，计算重叠长度和末端坐标
        if (box_path_primary_start != SIZE_MAX) {
          size_t bp_idx = box_path_primary_start;
          size_t bs_idx = box_path_secondary_start;
          
          while (bp_idx + 1 < primary_box_path->len &&
                 bs_idx + 1 < planned_paths[secondary].len) {
            Point dir_bp = primary_box_path->dir[bp_idx];
            Point dir_bs = planned_paths[secondary].dir[bs_idx];
            Point next_bp = primary_box_path->path[bp_idx + 1];
            Point next_bs = planned_paths[secondary].path[bs_idx + 1];
            
            if (dir_bp.row != dir_bs.row || dir_bp.col != dir_bs.col) {
              break;
            }
            if (next_bp.row != next_bs.row || next_bp.col != next_bs.col) {
              break;
            }
            overlap_len_second++;
            bp_idx++;
            bs_idx++;
          }
          
          // 保存重叠路径的末端坐标
          // 循环结束后，bp_idx 指向重叠路径的最后一个位置（因为每次循环后都递增）
          // 所以末端坐标就是 bp_idx 对应的位置
          if (overlap_len_second > 0 && bp_idx < primary_box_path->len) {
            ov->overlap_end_pos = primary_box_path->path[bp_idx];
          } else {
            ov->overlap_end_pos.row = -1;
            ov->overlap_end_pos.col = -1;
          }
        } else {
          ov->overlap_end_pos.row = -1;
          ov->overlap_end_pos.col = -1;
        }
      } else {
        ov->overlap_end_pos.row = -1;
        ov->overlap_end_pos.col = -1;
      }
      ov->overlap_len_second = overlap_len_second;
    }
  }
}

// 检查链中是否所有箱子都已完成
static int planner_v3_bfs_chain_all_finished(
    const PlannerChainLocal *chain,
    const Point *current_boxes,
    const size_t *box_targets,
    const Point *targets) {
  if (!chain || chain->count == 0) return 1;
  for (size_t c = 0; c < chain->count; ++c) {
    size_t box_idx = chain->indices[c];
    Point box_pos = current_boxes[box_idx];
    if (box_pos.row >= 0 && box_pos.col >= 0) {
      if (box_targets[box_idx] != SIZE_MAX) {
        Point target = targets[box_targets[box_idx]];
        if (box_pos.row != target.row || box_pos.col != target.col) {
          return 0;  // 还有未完成的箱子
        }
      } else {
        return 0;  // 没有目标但位置有效，视为未完成
      }
    }
  }
  return 1;  // 所有箱子都已完成
}

// 从主箱开始，沿着推动方向构建完整的链
// 返回链的长度（包含主箱），0表示无链
static size_t planner_v3_bfs_build_chain_from_primary(
    size_t primary_idx, Point push_dir, const Point *current_boxes, size_t box_count,
    const PlannerBoxOverlap *overlaps, PlannerChainLocal *chain) {
  if (!chain) return 0;
  
  chain->count = 0;
  chain->dir = push_dir;
  
  // 主箱是链的第一个，重叠长度为0（它是主箱），已走步数为0
  chain->indices[chain->count] = primary_idx;
  chain->overlap_lens[chain->count] = 0;  // 主箱没有重叠长度限制
  chain->overlap_end_positions[chain->count].row = -1;  // 主箱没有重叠路径末端坐标
  chain->overlap_end_positions[chain->count].col = -1;
  chain->steps_walked[chain->count] = 0;
  chain->count++;
  
  // 沿着推动方向查找连续的副箱子
  Point expected_pos = current_boxes[primary_idx];
  expected_pos.row += push_dir.row;
  expected_pos.col += push_dir.col;
  
  size_t current_in_chain = primary_idx;
  
  while (chain->count < PLANNER_V3_BFS_MAX_BOXES) {
    size_t found = SIZE_MAX;
    size_t found_overlap_len = 0;
    
    // 查找在expected_pos位置的副箱子
    for (size_t s = 0; s < box_count; ++s) {
      if (s == current_in_chain) continue;
      
      // 检查是否已经在链中
      int already_in_chain = 0;
      for (size_t c = 0; c < chain->count; ++c) {
        if (chain->indices[c] == s) {
          already_in_chain = 1;
          break;
        }
      }
      if (already_in_chain) continue;
      
      Point box_pos = current_boxes[s];
      if (box_pos.row < 0 || box_pos.col < 0) continue;
      
      if (box_pos.row == expected_pos.row && box_pos.col == expected_pos.col) {
        // 检查这个箱子的重叠信息是否表明它是当前箱子的副箱（同方向）
        // 使用第一次路径规划的overlap_len判断是否构成链式
        if (overlaps && overlaps[s].valid && overlaps[s].primary == current_in_chain &&
            overlaps[s].dir.row == push_dir.row && overlaps[s].dir.col == push_dir.col) {
          found = s;
          // 存储第二次路径规划的overlap_len_second用于检测何时路径不再重叠
          found_overlap_len = overlaps[s].overlap_len_second;
          break;
        }
        // 即使没有重叠信息，只要位置对就认为是链中的一员（重叠长度设为1）
        found = s;
        found_overlap_len = 1;
        break;
      }
    }
    
    if (found == SIZE_MAX) {
      break;  // 没有更多副箱子了
    }
    
    chain->indices[chain->count] = found;
    chain->overlap_lens[chain->count] = found_overlap_len;
    // 保存重叠路径的末端坐标
    if (overlaps && overlaps[found].valid && overlaps[found].overlap_end_pos.row >= 0 && overlaps[found].overlap_end_pos.col >= 0) {
      chain->overlap_end_positions[chain->count] = overlaps[found].overlap_end_pos;
    } else {
      chain->overlap_end_positions[chain->count].row = -1;
      chain->overlap_end_positions[chain->count].col = -1;
    }
    chain->steps_walked[chain->count] = 0;
    chain->count++;
    current_in_chain = found;
    
    // 更新期望位置
    expected_pos.row += push_dir.row;
    expected_pos.col += push_dir.col;
  }
  
  return chain->count;
}

// 检查链中是否有箱子到达目标点并剔除
// 只有到达目标点的箱子才被剔除
// 返回被剔除的箱子数量
static size_t planner_v3_bfs_chain_remove_finished_ex(
    PlannerChainLocal *chain, Point *current_boxes, const size_t *box_targets,
    const Point *targets, size_t box_count, int check_overlap) {
  if (!chain || chain->count == 0) return 0;
  
  (void)check_overlap;  // 不再使用此参数，但保持接口兼容
  
  size_t removed = 0;
  size_t write_idx = 0;
  
  for (size_t i = 0; i < chain->count; ++i) {
    size_t box_idx = chain->indices[i];
    Point box_pos = current_boxes[box_idx];
    
    int should_remove = 0;
    
    // 只检查是否到达目标点
    if (box_targets[box_idx] != SIZE_MAX && box_pos.row >= 0 && box_pos.col >= 0) {
      Point target = targets[box_targets[box_idx]];
      if (box_pos.row == target.row && box_pos.col == target.col) {
        should_remove = 1;
        // 标记箱子为已完成
        current_boxes[box_idx].row = -1;
        current_boxes[box_idx].col = -1;
      }
    }
    
    if (should_remove) {
      removed++;
    } else {
      // 保留在链中，复制数据
      chain->indices[write_idx] = chain->indices[i];
      chain->overlap_lens[write_idx] = chain->overlap_lens[i];
      chain->steps_walked[write_idx] = chain->steps_walked[i];
      write_idx++;
    }
  }
  
  chain->count = write_idx;
  return removed;
}

// 兼容函数：检查走完重叠路径或到达目标（直行时使用）
static size_t planner_v3_bfs_chain_remove_finished(
    PlannerChainLocal *chain, Point *current_boxes, const size_t *box_targets,
    const Point *targets, size_t box_count) {
  return planner_v3_bfs_chain_remove_finished_ex(chain, current_boxes, box_targets, 
                                                  targets, box_count, 1);
}

// forward declarations: 供直行和拐弯推进函数提前调用（避免 C99 隐式声明）
static int planner_v3_bfs_chain_try_detach_any_box_by_overlap(
    int rows, int cols, PlannerChainLocal *chain,
    const size_t *box_targets, const Point *targets,
    const Point *obstacles, size_t obstacle_count,
    Point *current_car, Point *current_boxes, size_t box_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps,
    int *out_detached, int rollback_needed, const PlannerChainLocal *saved_chain_state,
    const Point *saved_car, const Point *saved_boxes, size_t saved_steps);

static int planner_v3_bfs_chain_detach_tail_if_overlap_done(
    int rows, int cols, PlannerChainLocal *chain,
    const size_t *box_targets, const Point *targets,
    const Point *obstacles, size_t obstacle_count,
    Point *current_car, Point *current_boxes, size_t box_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps);

// 链式直行推动：所有链中的箱子同时向push_dir方向移动一格
// 返回：0=成功, -6=失败, -7=缓存不足
static int planner_v3_bfs_chain_push_straight(
    int rows, int cols, PlannerChainLocal *chain, Point push_dir,
    const Point *obstacles, size_t obstacle_count,
    Point *current_car, Point *current_boxes, size_t box_count,
    const size_t *box_targets, const Point *targets,
    Point *path_buffer, size_t path_capacity, size_t *out_steps) {
  if (!chain) {
    last_err_stage = 5;  // 链式推动内部失败
    last_err_detail = 1; // 链指针为空
    return -6;
  }
  
  // 检查链中所有箱子是否都已完成
  if (chain->count == 0 || planner_v3_bfs_chain_all_finished(chain, current_boxes, box_targets, targets)) {
    // 所有箱子都已完成，标记为完成并返回
    for (size_t c = 0; c < chain->count; ++c) {
      size_t box_idx = chain->indices[c];
      current_boxes[box_idx].row = -1;
      current_boxes[box_idx].col = -1;
    }
    return 0;
  }
  
  // 主箱索引
  size_t primary_idx = chain->indices[0];
  Point primary_pos = current_boxes[primary_idx];
  
  // 计算推箱位（主箱的反方向）
  Point push_pos = {primary_pos.row - push_dir.row, primary_pos.col - push_dir.col};
  
  // 检查推箱位有效性
  if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col) ||
      planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_pos.row, push_pos.col)) {
    last_err_stage = 5;  // 链式推动内部失败
    last_err_detail = 2; // 推位非法（越界或障碍物）
    return -6;
  }
  
  // 检查推箱位是否被非链内箱子占用
  for (size_t bi = 0; bi < box_count; ++bi) {
    int in_chain = 0;
    for (size_t c = 0; c < chain->count; ++c) {
      if (chain->indices[c] == bi) {
        in_chain = 1;
        break;
      }
    }
    if (in_chain) continue;
    
    Point b = current_boxes[bi];
    if (b.row >= 0 && b.col >= 0 && b.row == push_pos.row && b.col == push_pos.col) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 3; // 推位被占（被非链内箱子占用）
      return -6;
    }
  }
  
  // 检查链尾移动后的位置是否有效
  size_t tail_idx = chain->indices[chain->count - 1];
  Point tail_pos = current_boxes[tail_idx];
  Point tail_next = {tail_pos.row + push_dir.row, tail_pos.col + push_dir.col};
  
  if (!planner_v3_bfs_in_bounds(rows, cols, tail_next.row, tail_next.col) ||
      planner_v3_bfs_is_obstacle(obstacles, obstacle_count, tail_next.row, tail_next.col)) {
    last_err_stage = 5;  // 链式推动内部失败
    last_err_detail = 4; // 链尾移动位置非法（越界或障碍物）
    return -6;
  }
  
  // 检查链尾移动位置是否被非链内箱子占用
  for (size_t bi = 0; bi < box_count; ++bi) {
    int in_chain = 0;
    for (size_t c = 0; c < chain->count; ++c) {
      if (chain->indices[c] == bi) {
        in_chain = 1;
        break;
      }
    }
    if (in_chain) continue;
    
    Point b = current_boxes[bi];
    if (b.row >= 0 && b.col >= 0 && b.row == tail_next.row && b.col == tail_next.col) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 5; // 链尾移动位置被占（被非链内箱子占用）
      return -6;
    }
  }
  
  // 【保存推动前的状态，用于可能的回溯】
  PlannerChainLocal saved_chain_state = *chain;
  Point saved_car = *current_car;
  Point saved_boxes[PLANNER_V3_BFS_MAX_BOXES];
  for (size_t i = 0; i < box_count; ++i) {
    saved_boxes[i] = current_boxes[i];
  }
  size_t saved_steps = *out_steps;
  
  // 车移动到推箱位
  if (current_car->row != push_pos.row || current_car->col != push_pos.col) {
    int car_res = planner_v3_bfs_car_move_with_global_astar(
        rows, cols, current_car, push_pos, obstacles, obstacle_count,
        current_boxes, box_count, path_buffer, path_capacity, out_steps);
    if (car_res == -7) return -7;
    if (car_res == 0) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 6; // 车到不了推位
      return -6;
    }
  }
  
  if (*out_steps >= path_capacity) return -7;
  
  // 从链尾开始，依次更新所有箱子的位置
  for (int i = (int)chain->count - 1; i >= 0; --i) {
    size_t idx = chain->indices[i];
    current_boxes[idx].row += push_dir.row;
    current_boxes[idx].col += push_dir.col;
    // 更新已走步数
    chain->steps_walked[i]++;
  }
  
  // 车移动到原主箱位置
  *current_car = primary_pos;
  path_buffer[(*out_steps)++] = *current_car;
  
  // 【新增】直行推进后检测链中任意箱子是否需要脱链（需要回溯时先回溯到上一步再脱链）
  int detached_any = 0;
  (void)planner_v3_bfs_chain_try_detach_any_box_by_overlap(
      rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
      current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps,
      &detached_any, 1, &saved_chain_state, &saved_car, saved_boxes, saved_steps);
  
  // 如果脱链成功，直接返回（不再执行后续的链尾检测）
  if (detached_any) {
    return 0;
  }
  
  // 继续执行原有的链尾脱链检测（按overlap_len精确脱链）
  (void)planner_v3_bfs_chain_detach_tail_if_overlap_done(
      rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
      current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
  
  if (chain->count == 0) return 0;
  
  // 每次直行推动后，检查链尾箱子是否到达目标，如果到达则自动缩小链
  while (chain->count > 0) {
    size_t tail_idx = chain->indices[chain->count - 1];
    Point tail_pos = current_boxes[tail_idx];
    
    // 检查链尾箱子是否到达目标
    int tail_finished = 0;
    if (box_targets[tail_idx] != SIZE_MAX && tail_pos.row >= 0 && tail_pos.col >= 0) {
      Point target = targets[box_targets[tail_idx]];
      if (tail_pos.row == target.row && tail_pos.col == target.col) {
        tail_finished = 1;
        // 标记链尾箱子为已完成
        current_boxes[tail_idx].row = -1;
        current_boxes[tail_idx].col = -1;
      }
    }
    
    if (tail_finished) {
      // 移除链尾（缩小链）
      chain->count--;
      
      // 如果链缩小到0，检查链内的所有箱子是否都已经到达目标点
      if (chain->count == 0) {
        // 链已经为空，说明链中的所有箱子都已经到达目标点并被移除
        // 此时需要检查所有箱子（不仅仅是链中的）是否都已完成
        // 但由于我们只处理链中的箱子，链为空时说明链中所有箱子都已完成
        return 0;
      }
    } else {
      // 链尾未到达目标，停止检查
      break;
    }
  }
  
  // 推动后检查链中剩余箱子是否都已完成（链可能已经缩小）
  if (chain->count > 0) {
    if (planner_v3_bfs_chain_all_finished(chain, current_boxes, box_targets, targets)) {
      // 链中所有剩余箱子都已完成，标记为完成并返回
      for (size_t c = 0; c < chain->count; ++c) {
        size_t box_idx = chain->indices[c];
        current_boxes[box_idx].row = -1;
        current_boxes[box_idx].col = -1;
      }
      return 0;
    }
  }
  
  return 0;
}

// 链式拐弯推动：处理链在拐弯点的复杂情况
// 拐弯逻辑（严格按用户要求）：
// 1. 车去推动拐弯点的副箱子朝拐弯方向移动一格
// 2. 车回到主箱推箱位推动主箱朝拐弯点移动一格（整个链朝原方向移动）
// 3. 此时又有一个副箱到达拐弯点
// 4. 重复1-3直到主箱到达拐弯点
// 在拐弯过程中只有箱子到达目标点的情况才可以被剔除
// 返回：0=成功, -6=失败, -7=缓存不足
static int planner_v3_bfs_chain_push_turn(
    int rows, int cols, PlannerChainLocal *chain, Point old_dir, Point new_dir,
    Point bend_point,
    const Point *obstacles, size_t obstacle_count, Point *current_car,
    Point *current_boxes, size_t box_count, const size_t *box_targets,
    const Point *targets, Point *path_buffer, size_t path_capacity,
    size_t *out_steps) {
  if (!chain) {
    last_err_stage = 5;  // 链式推动内部失败
    last_err_detail = 1; // 链指针为空
    return -6;
  }
  
  // 检查链中所有箱子是否都已完成
  if (chain->count == 0 || planner_v3_bfs_chain_all_finished(chain, current_boxes, box_targets, targets)) {
    // 所有箱子都已完成，标记为完成并返回
    for (size_t c = 0; c < chain->count; ++c) {
      size_t box_idx = chain->indices[c];
      current_boxes[box_idx].row = -1;
      current_boxes[box_idx].col = -1;
    }
    return 0;
  }

  while (chain->count > 0) {
    // 在每次循环开始时检查链中所有箱子是否都已完成
    if (planner_v3_bfs_chain_all_finished(chain, current_boxes, box_targets, targets)) {
      // 所有箱子都已完成，标记为完成并返回
      for (size_t c = 0; c < chain->count; ++c) {
        size_t box_idx = chain->indices[c];
        current_boxes[box_idx].row = -1;
        current_boxes[box_idx].col = -1;
      }
      return 0;
    }
    
    // 【保存循环开始时的状态，用于可能的回溯（包括提前分离判定）】
    PlannerChainLocal saved_loop_chain = *chain;
    Point saved_loop_car = *current_car;
    Point saved_loop_boxes[PLANNER_V3_BFS_MAX_BOXES];
    for (size_t i = 0; i < box_count; ++i) {
      saved_loop_boxes[i] = current_boxes[i];
    }
    size_t saved_loop_steps = *out_steps;
    
    size_t primary_idx = chain->indices[0];
    Point primary_pos = current_boxes[primary_idx];

    if (primary_pos.row == bend_point.row && primary_pos.col == bend_point.col) {
      chain->dir = new_dir;
      return 0;
    }

    size_t box_at_bend = SIZE_MAX;
    int box_at_bend_chain_idx = -1;
    for (size_t i = 0; i < chain->count; ++i) {
      size_t idx = chain->indices[i];
      Point pos = current_boxes[idx];
      if (pos.row == bend_point.row && pos.col == bend_point.col) {
        box_at_bend = idx;
        box_at_bend_chain_idx = (int)i;
        break;
      }
    }

    if (box_at_bend == SIZE_MAX) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 7; // 拐弯点没有箱子
      return -6;
    }

    // ===== 【提前分离判定】在步骤1之前，使用和拐弯时一样的脱离判定 =====
    // 检测是否有箱子路径分叉需要脱离
    // - 如果车无法到达推位，则回溯到循环开始时的状态再脱链
    if (chain->count > 0) {
      int detached_any = 0;
      (void)planner_v3_bfs_chain_try_detach_any_box_by_overlap(
          rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
          current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps,
          &detached_any, 1, &saved_loop_chain, &saved_loop_car, saved_loop_boxes, saved_loop_steps);
      
      // 如果检测到分叉且车无法到达推位导致脱链，直接返回（不继续执行步骤1）
      if (detached_any) {
        return 0;
      }
      
      if (chain->count == 0) return 0;
    }

    // ===== 步骤1：推动拐弯点的箱子朝新方向一格 =====
    // 【保存步骤1前的状态，用于可能的回溯】
    PlannerChainLocal saved_step1_chain = *chain;
    Point saved_step1_car = *current_car;
    Point saved_step1_boxes[PLANNER_V3_BFS_MAX_BOXES];
    for (size_t i = 0; i < box_count; ++i) {
      saved_step1_boxes[i] = current_boxes[i];
    }
    size_t saved_step1_steps = *out_steps;
    
    Point box_at_bend_pos = current_boxes[box_at_bend];
    Point box_next = {box_at_bend_pos.row + new_dir.row, box_at_bend_pos.col + new_dir.col};
    Point push_pos = {box_at_bend_pos.row - new_dir.row, box_at_bend_pos.col - new_dir.col};

    // 检查推箱位和目标位有效性
    if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col) ||
        planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_pos.row, push_pos.col)) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 8; // 拐弯推位非法
      return -6;
    }
    if (!planner_v3_bfs_in_bounds(rows, cols, box_next.row, box_next.col) ||
        planner_v3_bfs_is_obstacle(obstacles, obstacle_count, box_next.row, box_next.col)) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 9; // 拐弯目标位非法
      return -6;
    }

    // 检查是否被非链内箱子占用
    for (size_t bi = 0; bi < box_count; ++bi) {
      int in_chain = 0;
      for (size_t c = 0; c < chain->count; ++c) {
        if (chain->indices[c] == bi) {
          in_chain = 1;
          break;
        }
      }
      if (in_chain) continue;
      Point b = current_boxes[bi];
      if (b.row >= 0 && b.col >= 0) {
        if ((b.row == push_pos.row && b.col == push_pos.col) ||
            (b.row == box_next.row && b.col == box_next.col)) {
          last_err_stage = 5;  // 链式推动内部失败
          last_err_detail = 10; // 拐弯位置被占
          return -6;
        }
      }
    }

    if (current_car->row != push_pos.row || current_car->col != push_pos.col) {
      int car_res = planner_v3_bfs_car_move_with_global_astar(
          rows, cols, current_car, push_pos, obstacles, obstacle_count, current_boxes, box_count,
          path_buffer, path_capacity, out_steps);
      if (car_res == -7) return -7;
      if (car_res == 0) {
        last_err_stage = 5;  // 链式推动内部失败
        last_err_detail = 6; // 车到不了推位
        return -6;
      }
    }

    if (*out_steps >= path_capacity) return -7;

    Point old_box_pos = box_at_bend_pos;
    current_boxes[box_at_bend] = box_next;
    *current_car = old_box_pos;
    path_buffer[(*out_steps)++] = *current_car;
    if (box_at_bend_chain_idx >= 0 && box_at_bend_chain_idx < (int)chain->count) {
      chain->steps_walked[box_at_bend_chain_idx]++;
    }

    if (box_targets[box_at_bend] != SIZE_MAX) {
      Point target = targets[box_targets[box_at_bend]];
      if (box_next.row == target.row && box_next.col == target.col) {
        current_boxes[box_at_bend].row = -1;
        current_boxes[box_at_bend].col = -1;
        for (size_t j = box_at_bend_chain_idx; j < chain->count - 1; ++j) {
          chain->indices[j] = chain->indices[j + 1];
          chain->overlap_lens[j] = chain->overlap_lens[j + 1];
          chain->overlap_end_positions[j] = chain->overlap_end_positions[j + 1];
          chain->steps_walked[j] = chain->steps_walked[j + 1];
        }
        chain->count--;
      }
    }

    if (chain->count == 0) return 0;

    // 【新增】拐弯过程中检测是否有箱子路径分叉需要脱离
    // - 如果车无法到达推位，则回溯到步骤1前的状态再脱链
    int detached_any = 0;
    (void)planner_v3_bfs_chain_try_detach_any_box_by_overlap(
        rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
        current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps,
        &detached_any, 1, &saved_step1_chain, &saved_step1_car, saved_step1_boxes, saved_step1_steps);
    
    // 如果检测到分叉且车无法到达推位导致脱链，直接返回（不继续执行步骤2）
    if (detached_any) {
      return 0;
    }
    
    if (chain->count == 0) return 0;

    // 【新增】拐弯过程中也允许按 overlap_len 精确脱链：
    // - 走完共享段后让副箱按自身路径独立走
    // - 且动作序列上"先推此副箱，然后继续推链"
    (void)planner_v3_bfs_chain_detach_tail_if_overlap_done(
        rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
        current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
    if (chain->count == 0) return 0;

    primary_idx = chain->indices[0];
    primary_pos = current_boxes[primary_idx];

    if (primary_pos.row == bend_point.row && primary_pos.col == bend_point.col) {
      chain->dir = new_dir;
      return 0;
    }

    // ===== 步骤2：推主箱朝原方向（拐弯点方向）移动一格 =====
    // 【保存步骤2前的状态，用于可能的回溯】
    PlannerChainLocal saved_step2_chain = *chain;
    Point saved_step2_car = *current_car;
    Point saved_step2_boxes[PLANNER_V3_BFS_MAX_BOXES];
    for (size_t i = 0; i < box_count; ++i) {
      saved_step2_boxes[i] = current_boxes[i];
    }
    size_t saved_step2_steps = *out_steps;
    
    Point primary_next = {primary_pos.row + old_dir.row, primary_pos.col + old_dir.col};
    Point push_primary_pos = {primary_pos.row - old_dir.row, primary_pos.col - old_dir.col};

    if (!planner_v3_bfs_in_bounds(rows, cols, push_primary_pos.row, push_primary_pos.col) ||
        planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_primary_pos.row,
                                   push_primary_pos.col)) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 19; // 拐弯步骤2推位非法
      return -6;
    }
    if (!planner_v3_bfs_in_bounds(rows, cols, primary_next.row, primary_next.col) ||
        planner_v3_bfs_is_obstacle(obstacles, obstacle_count, primary_next.row,
                                   primary_next.col)) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 20; // 拐弯步骤2目标位非法
      return -6;
    }

    for (size_t bi = 0; bi < box_count; ++bi) {
      int in_chain = 0;
      for (size_t c = 0; c < chain->count; ++c) {
        if (chain->indices[c] == bi) {
          in_chain = 1;
          break;
        }
      }
      if (in_chain) continue;
      Point b = current_boxes[bi];
      if (b.row >= 0 && b.col >= 0 &&
          b.row == push_primary_pos.row && b.col == push_primary_pos.col) {
        last_err_stage = 5;  // 链式推动内部失败
        last_err_detail = 21; // 拐弯步骤2推位被占
        return -6;
      }
    }

    if (current_car->row != push_primary_pos.row || current_car->col != push_primary_pos.col) {
      int car_res = planner_v3_bfs_car_move_with_global_astar(
          rows, cols, current_car, push_primary_pos, obstacles, obstacle_count, current_boxes,
          box_count, path_buffer, path_capacity, out_steps);
      if (car_res == -7) return -7;
      if (car_res == 0) {
        last_err_stage = 5;  // 链式推动内部失败
        last_err_detail = 6; // 车到不了推位
        return -6;
      }
    }

    if (*out_steps >= path_capacity) return -7;

    // 基于 box_id 重新定位拐弯点箱子（消除对易失效索引的依赖）
    // 步骤1可能已删除 box_at_bend，需要检查它是否仍在链中
    int box_at_bend_still_in_chain = 0;
    int current_bend_chain_idx = -1;
    for (size_t ci = 0; ci < chain->count; ++ci) {
      if (chain->indices[ci] == box_at_bend) {
        box_at_bend_still_in_chain = 1;
        current_bend_chain_idx = (int)ci;
        break;
      }
    }

    uint8_t chain_mask[PLANNER_V3_BFS_MAX_BOXES] = {0};
    for (size_t ci = 0; ci < chain->count; ++ci) {
      chain_mask[chain->indices[ci]] = 1;
    }

    Point planned[PLANNER_V3_BFS_MAX_BOXES];
    for (size_t bi = 0; bi < box_count; ++bi) {
      planned[bi] = current_boxes[bi];
    }

    // 步骤2的位移逻辑：
    // - 如果 box_at_bend 在步骤1中被删除，则所有箱子都移动（无跳过元素）
    // - 如果 box_at_bend 仍在链中，则跳过它（通过 box_id 定位）
    if (box_at_bend_still_in_chain && current_bend_chain_idx >= 0) {
      // 拐弯点箱子仍在链中：跳过它，其他箱子按位置分段移动
      for (size_t ci = 0; ci < chain->count; ++ci) {
        size_t idx = chain->indices[ci];
        if (idx == box_at_bend) {
          // 跳过拐弯点箱子（已在步骤1移动）
          continue;
        }
        // 根据在链中的位置决定移动方向
        if ((int)ci > current_bend_chain_idx) {
          // 拐弯点之后的箱子：朝新方向移动
          planned[idx].row += new_dir.row;
          planned[idx].col += new_dir.col;
        } else {
          // 拐弯点之前的箱子：朝原方向移动
          planned[idx].row += old_dir.row;
          planned[idx].col += old_dir.col;
        }
      }
    } else {
      // box_at_bend 在步骤1中被删除：所有箱子都移动，无跳过元素
      // 需要确定分割点：删除前的 box_at_bend_chain_idx 位置
      // 由于已删除，分割点应该是删除前位置（或删除后第一个元素的位置）
      // 简化处理：如果删除的是链尾，则所有箱子朝 old_dir 移动
      // 如果删除的是中间元素，则删除位置之后的朝 new_dir，之前的朝 old_dir
      int removed_idx = box_at_bend_chain_idx;
      if (removed_idx < 0) removed_idx = 0;
      if (removed_idx >= (int)chain->count) {
        // 删除的是链尾或超出范围：所有剩余箱子朝 old_dir 移动
        for (size_t ci = 0; ci < chain->count; ++ci) {
          size_t idx = chain->indices[ci];
          planned[idx].row += old_dir.row;
          planned[idx].col += old_dir.col;
        }
      } else {
        // 删除的是中间元素：删除位置之后的朝 new_dir，之前的朝 old_dir
        for (size_t ci = 0; ci < chain->count; ++ci) {
          size_t idx = chain->indices[ci];
          if ((int)ci >= removed_idx) {
            planned[idx].row += new_dir.row;
            planned[idx].col += new_dir.col;
          } else {
            planned[idx].row += old_dir.row;
            planned[idx].col += old_dir.col;
          }
        }
      }
    }

    for (size_t ci = 0; ci < chain->count; ++ci) {
      size_t idx = chain->indices[ci];
      Point np = planned[idx];
      if (!planner_v3_bfs_in_bounds(rows, cols, np.row, np.col) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, np.row, np.col)) {
        last_err_stage = 5;  // 链式推动内部失败
        last_err_detail = 22; // 拐弯步骤2计划位置非法
        return -6;
      }
      for (size_t bi = 0; bi < box_count; ++bi) {
        if (chain_mask[bi]) {
          continue;
        }
        Point b = current_boxes[bi];
        if (b.row >= 0 && b.col >= 0 && b.row == np.row && b.col == np.col) {
          last_err_stage = 5;  // 链式推动内部失败
          last_err_detail = 23; // 拐弯步骤2计划位置被占
          return -6;
        }
      }
    }

    for (size_t a = 0; a < chain->count; ++a) {
      size_t idx_a = chain->indices[a];
      Point pa = planned[idx_a];
      for (size_t b = a + 1; b < chain->count; ++b) {
        size_t idx_b = chain->indices[b];
        Point pb = planned[idx_b];
        if (pa.row == pb.row && pa.col == pb.col) {
          last_err_stage = 5;  // 链式推动内部失败
          last_err_detail = 33; // 拐弯步骤2计划位置冲突
          return -6;
        }
      }
    }

    // 执行位移：基于 box_id 定位，而不是依赖易失效的索引
    if (box_at_bend_still_in_chain && current_bend_chain_idx >= 0) {
      // 拐弯点箱子仍在链中：跳过它，其他箱子按位置分段移动
      for (size_t ci = 0; ci < chain->count; ++ci) {
        size_t idx = chain->indices[ci];
        if (idx == box_at_bend) {
          // 跳过拐弯点箱子（已在步骤1移动）
          continue;
        }
        current_boxes[idx] = planned[idx];
        chain->steps_walked[ci]++;
      }
    } else {
      // box_at_bend 在步骤1中被删除：所有箱子都移动
      int removed_idx = box_at_bend_chain_idx;
      if (removed_idx < 0) removed_idx = 0;
      if (removed_idx >= (int)chain->count) {
        // 删除的是链尾或超出范围：所有剩余箱子朝 old_dir 移动
        for (size_t ci = 0; ci < chain->count; ++ci) {
          size_t idx = chain->indices[ci];
          current_boxes[idx] = planned[idx];
          chain->steps_walked[ci]++;
        }
      } else {
        // 删除的是中间元素：删除位置之后的朝 new_dir，之前的朝 old_dir
        for (size_t ci = 0; ci < chain->count; ++ci) {
          size_t idx = chain->indices[ci];
          current_boxes[idx] = planned[idx];
          chain->steps_walked[ci]++;
        }
      }
    }

    *current_car = primary_pos;
    path_buffer[(*out_steps)++] = *current_car;

    for (int i = (int)chain->count - 1; i >= 0; --i) {
      size_t idx = chain->indices[i];
      Point pos = current_boxes[idx];
      if (box_targets[idx] != SIZE_MAX && pos.row >= 0 && pos.col >= 0) {
        Point target = targets[box_targets[idx]];
        if (pos.row == target.row && pos.col == target.col) {
          current_boxes[idx].row = -1;
          current_boxes[idx].col = -1;
          for (size_t j = i; j < chain->count - 1; ++j) {
            chain->indices[j] = chain->indices[j + 1];
            chain->overlap_lens[j] = chain->overlap_lens[j + 1];
            chain->overlap_end_positions[j] = chain->overlap_end_positions[j + 1];
            chain->steps_walked[j] = chain->steps_walked[j + 1];
          }
          chain->count--;
        }
      }
    }

    if (chain->count == 0) return 0;

    // 【新增】步骤2结束后检测是否有箱子路径分叉需要脱离
    // - 如果车无法到达推位，则回溯到步骤2前的状态再脱链
    int detached_any_step2 = 0;
    (void)planner_v3_bfs_chain_try_detach_any_box_by_overlap(
        rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
        current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps,
        &detached_any_step2, 1, &saved_step2_chain, &saved_step2_car, saved_step2_boxes, saved_step2_steps);
    
    // 如果检测到脱链，直接返回
    if (detached_any_step2) {
      return 0;
    }

    // 【新增】步骤2结束后也允许按 overlap_len 精确脱链：先推脱链副箱，再继续拐弯推链
    (void)planner_v3_bfs_chain_detach_tail_if_overlap_done(
        rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
        current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
  }

  return 0;
}

static int planner_v3_bfs_execute_single_box_path(int rows, int cols, size_t box_idx,
                                              const PlannerBoxPathInfo *path_info,
                                              const Point *obstacles, size_t obstacle_count,
                                              Point *current_car, Point *current_boxes,
                                              size_t box_count, Point *path_buffer,
                                              size_t path_capacity, size_t *out_steps) {
  if (!path_info || !path_info->valid || path_info->len == 0) {
    last_err_stage = 5;  // 链式推动内部失败
    last_err_detail = 13; // 单箱路径无效或为空
    return -6;
  }

  for (size_t step = 1; step < path_info->len; ++step) {
    Point curr = current_boxes[box_idx];
    Point next = path_info->path[step];
    Point dir = {next.row - curr.row, next.col - curr.col};
    Point push_pos = {curr.row - dir.row, curr.col - dir.col};

    if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col) ||
        planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_pos.row, push_pos.col) ||
        planner_v3_bfs_is_box_at(current_boxes, box_count, push_pos.row, push_pos.col,
                                 box_idx)) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 14; // 单箱推位非法或被占
      return -6;
    }

    int car_result = planner_v3_bfs_car_move_with_global_astar(
        rows, cols, current_car, push_pos, obstacles, obstacle_count, current_boxes, box_count,
        path_buffer, path_capacity, out_steps);
    if (car_result == -7) {
      return -7;
    }
    if (car_result == 0) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 6; // 车到不了推位
      return -6;
    }

    if (current_car->row != push_pos.row || current_car->col != push_pos.col) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 15; // 车未到达推位
      return -6;
    }

    if (!planner_v3_bfs_in_bounds(rows, cols, next.row, next.col)) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 16; // 箱子目标位越界
      return -6;
    }
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, next.row, next.col)) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 17; // 箱子目标位是障碍物
      return -6;
    }
    for (size_t bi = 0; bi < box_count; ++bi) {
      if (bi == box_idx) {
        continue;
      }
      Point b = current_boxes[bi];
      if (b.row == next.row && b.col == next.col) {
        last_err_stage = 5;  // 链式推动内部失败
        last_err_detail = 18; // 箱子目标位被其他箱子占用
        return -6;
      }
    }

    if (*out_steps >= path_capacity) {
      return -7;
    }
    Point old_pos = curr;
    current_boxes[box_idx] = next;
    *current_car = old_pos;
    path_buffer[(*out_steps)++] = *current_car;
  }

  current_boxes[box_idx].row = -1;
  current_boxes[box_idx].col = -1;
  return 0;
}

// 同步链尾（驱动箱）对应的路径与路径索引：
// - 优先使用 g_planner_planned_paths[tail]
// - 如果当前 tail 位置不在路径上，则按“忽略链内前驱箱子”的规则重新计算 tail 路径
static int planner_v3_bfs_chain_sync_tail_path_state(
    int rows, int cols, PlannerChainLocal *chain,
    const size_t *box_targets, const Point *targets,
    const Point *obstacles, size_t obstacle_count,
    Point *current_boxes, size_t box_count,
    size_t *out_tail_idx, const PlannerBoxPathInfo **out_tail_path, size_t *out_path_idx) {
  if (!chain || chain->count == 0) return 0;

  size_t tail_idx = chain->indices[chain->count - 1];
  if (out_tail_idx) *out_tail_idx = tail_idx;

  if (!current_boxes || !box_targets || !targets) {
    last_err_stage = 5;  // 链式推动内部失败
    last_err_detail = 24; // 同步链尾路径状态参数为空
    return -6;
  }
  Point tail_pos = current_boxes[tail_idx];
  if (tail_pos.row < 0 || tail_pos.col < 0) {
    last_err_stage = 5;  // 链式推动内部失败
    last_err_detail = 25; // 链尾箱子已消失
    return -6;
  }
  if (box_targets[tail_idx] == SIZE_MAX) {
    last_err_stage = 5;  // 链式推动内部失败
    last_err_detail = 26; // 链尾箱子目标未分配
    return -6;
  }

  const PlannerBoxPathInfo *tail_path = &g_planner_planned_paths[tail_idx];
  size_t path_idx = SIZE_MAX;

  if (tail_path->valid && tail_path->len >= 2) {
    for (size_t i = 0; i < tail_path->len; ++i) {
      if (tail_path->path[i].row == tail_pos.row && tail_path->path[i].col == tail_pos.col) {
        path_idx = i;
        break;
      }
    }
  }

  if (path_idx == SIZE_MAX) {
    // 路径缺失或已失效：按链内“忽略前驱箱子”的规则重算 tail 路径
    PlannerBoxPathInfo new_tail_path;
    uint8_t ignore_mask[PLANNER_V3_BFS_MAX_BOXES] = {0};
    for (size_t c = 0; c + 1 < chain->count; ++c) {
      ignore_mask[chain->indices[c]] = 1;
    }
    if (!planner_v3_bfs_compute_box_path_with_mask(
            rows, cols, tail_pos, targets[box_targets[tail_idx]],
            obstacles, obstacle_count, current_boxes, box_count, ignore_mask, tail_idx,
            &new_tail_path)) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 27; // 链尾路径重算失败
      return -6;
    }
    g_planner_planned_paths[tail_idx] = new_tail_path;
    tail_path = &g_planner_planned_paths[tail_idx];

    // 重新定位 path_idx
    for (size_t i = 0; i < tail_path->len; ++i) {
      if (tail_path->path[i].row == tail_pos.row && tail_path->path[i].col == tail_pos.col) {
        path_idx = i;
        break;
      }
    }
    if (path_idx == SIZE_MAX) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 28; // 链尾路径索引未找到
      return -6;
    }
  }

  if (out_tail_path) *out_tail_path = tail_path;
  if (out_path_idx) *out_path_idx = path_idx;
  return 0;
}

// 辅助函数：回溯链的状态到上一步（直行推动的情况）
// 将所有链内箱子向push_dir的反方向移动一格，车位置也相应回溯
// 【改进版】检测链中任意箱子是否需要脱链（不只是链尾）
// - 检测当前坐标和重叠路径的末端坐标是否相同
// 尝试将脱离的箱子重新构成链式并检查是否可以推动
// 与一开始构建链式的流程一致：先第一次路径规划，然后标记，然后第二次路径规划
// 返回：1=可以链式推动, 0=不能链式推动
static int planner_v3_bfs_try_rebuild_chain_for_detached(
    int rows, int cols, const size_t *detached_indices, size_t detached_count,
    const Point *current_boxes, size_t box_count, const Point *obstacles, size_t obstacle_count,
    const size_t *box_targets, const Point *targets,
    PlannerChainLocal *out_chain, Point *out_push_dir,
    PlannerRebuildChainStatus *out_status) {
  // 初始化状态信息
  if (out_status) {
    out_status->success = 0;
    out_status->fail_step = 0;
    out_status->fail_box_idx = SIZE_MAX;
    out_status->fail_box_pos.row = -1;
    out_status->fail_box_pos.col = -1;
    out_status->detached_count = detached_count;
    for (size_t i = 0; i < detached_count && i < PLANNER_V3_BFS_MAX_BOXES; i++) {
      out_status->detached_indices[i] = detached_indices[i];
    }
    out_status->exec_success = -1;  // -1表示未尝试执行
    out_status->exec_res = 0;       // 初始化为0
    out_status->exec_fail_pos.row = -1;
    out_status->exec_fail_pos.col = -1;
    out_status->exec_fail_box_idx = SIZE_MAX;
  }
  
  if (!detached_indices || detached_count == 0 || !out_chain || !out_push_dir) {
    if (out_status) {
      out_status->fail_step = 0;  // 参数错误
    }
    return 0;
  }
  
  // 为脱离的箱子创建临时路径和重叠信息数组
  PlannerBoxPathInfo free_paths[PLANNER_V3_BFS_MAX_BOXES];
  PlannerBoxPathInfo planned_paths[PLANNER_V3_BFS_MAX_BOXES];
  PlannerBoxOverlap temp_overlaps[PLANNER_V3_BFS_MAX_BOXES];
  
  // 初始化
  for (size_t i = 0; i < detached_count; i++) {
    free_paths[i].valid = 0;
    free_paths[i].len = 0;
    planned_paths[i].valid = 0;
    planned_paths[i].len = 0;
    temp_overlaps[i].valid = 0;
    temp_overlaps[i].primary = SIZE_MAX;
  }
  
  // 第一步：第一次路径规划（忽略箱子阻挡）
  uint8_t ignore_all[PLANNER_V3_BFS_MAX_BOXES];
  for (size_t i = 0; i < box_count; ++i) {
    ignore_all[i] = 1;  // 忽略所有箱子
  }
  
  for (size_t i = 0; i < detached_count; i++) {
    size_t box_idx = detached_indices[i];
    if (box_targets[box_idx] == SIZE_MAX) {
      continue;
    }
    Point target = targets[box_targets[box_idx]];
    if (!planner_v3_bfs_compute_box_path_with_mask(
            rows, cols, current_boxes[box_idx], target, obstacles, obstacle_count,
            current_boxes, box_count, ignore_all, box_idx, &free_paths[i])) {
      // 第一次路径规划失败，无法构建链式
      if (out_status) {
        out_status->fail_step = 1;  // 第一次路径规划失败
        out_status->fail_box_idx = box_idx;
        out_status->fail_box_pos = current_boxes[box_idx];
      }
      return 0;
    }
  }
  
  // 第二步：第一次检测重叠（使用第一次路径规划的结果）
  // 创建映射：脱离箱子索引 -> 临时数组索引
  size_t detached_to_temp[PLANNER_V3_BFS_MAX_BOXES];
  for (size_t i = 0; i < box_count; i++) {
    detached_to_temp[i] = SIZE_MAX;
  }
  for (size_t i = 0; i < detached_count; i++) {
    detached_to_temp[detached_indices[i]] = i;
  }
  
  // 检测重叠（只检测脱离箱子之间的重叠）
  for (size_t primary_i = 0; primary_i < detached_count; primary_i++) {
    size_t primary_orig = detached_indices[primary_i];
    if (!free_paths[primary_i].valid) continue;
    
    for (size_t secondary_i = 0; secondary_i < detached_count; secondary_i++) {
      if (primary_i == secondary_i) continue;
      size_t secondary_orig = detached_indices[secondary_i];
      if (!free_paths[secondary_i].valid) continue;
      
      Point sec_start = current_boxes[secondary_orig];
      if (sec_start.row < 0 || sec_start.col < 0) continue;
      
      for (size_t pi = 0; pi < free_paths[primary_i].len; ++pi) {
        if (free_paths[primary_i].path[pi].row != sec_start.row ||
            free_paths[primary_i].path[pi].col != sec_start.col) {
          continue;
        }
        size_t sec_idx = 0;
        size_t overlap_len = 0;
        while (pi + overlap_len + 1 < free_paths[primary_i].len &&
               sec_idx + overlap_len + 1 < free_paths[secondary_i].len) {
          Point dir_p = free_paths[primary_i].dir[pi + overlap_len];
          Point dir_s = free_paths[secondary_i].dir[sec_idx + overlap_len];
          Point next_p = free_paths[primary_i].path[pi + overlap_len + 1];
          Point next_s = free_paths[secondary_i].path[sec_idx + overlap_len + 1];
          if (dir_p.row != dir_s.row || dir_p.col != dir_s.col) {
            break;
          }
          if (next_p.row != next_s.row || next_p.col != next_s.col) {
            break;
          }
          overlap_len++;
        }
        
        if (overlap_len > 0) {
          PlannerBoxOverlap *ov = &temp_overlaps[secondary_i];
          Point primary_pos = current_boxes[primary_orig];
          Point secondary_pos = current_boxes[secondary_orig];
          int current_dist = planner_v3_bfs_manhattan(primary_pos, secondary_pos);
          
          if (!ov->valid) {
            ov->valid = 1;
            ov->primary = primary_i;  // 使用临时索引
            ov->primary_start = pi;
            ov->secondary_start = 0;
            ov->overlap_len = overlap_len;
            ov->dir = free_paths[primary_i].dir[pi];
          } else {
            size_t old_primary_orig = detached_indices[ov->primary];
            Point old_primary_pos = current_boxes[old_primary_orig];
            int old_dist = planner_v3_bfs_manhattan(old_primary_pos, secondary_pos);
            
            // 如果当前主箱距离更近，则更新；如果距离相同，则选择重叠更短的（重叠更短优先），重叠长度相同则路径起始位置更早的优先
            if (current_dist < old_dist ||
                (current_dist == old_dist && overlap_len < ov->overlap_len) ||
                (current_dist == old_dist && overlap_len == ov->overlap_len && pi < ov->primary_start)) {
              ov->valid = 1;
              ov->primary = primary_i;
              ov->primary_start = pi;
              ov->secondary_start = 0;
              ov->overlap_len = overlap_len;
              ov->dir = free_paths[primary_i].dir[pi];
            }
          }
        }
        break;
      }
    }
  }
  
  // 第三步：第二次路径规划（考虑箱子阻挡，使用第一次检测的overlaps来构建忽略掩码）
  for (size_t i = 0; i < detached_count; i++) {
    size_t box_idx = detached_indices[i];
    if (box_targets[box_idx] == SIZE_MAX) {
      continue;
    }
    Point target = targets[box_targets[box_idx]];
    
    uint8_t ignore_mask[PLANNER_V3_BFS_MAX_BOXES] = {0};
    ignore_mask[box_idx] = 1;
    // 忽略已完成的箱子（负坐标）
    for (size_t j = 0; j < box_count; ++j) {
      if (current_boxes[j].row < 0 || current_boxes[j].col < 0) {
        ignore_mask[j] = 1;
      }
    }
    // 作为主箱，递归忽略自己对应的所有层级的副箱
    // 作为副箱，忽略自己的主箱
    // 使用第一次检测的temp_overlaps来构建忽略掩码
    // 注意：这里需要将临时索引转换为原始索引
    
    // 递归收集所有副箱（使用临时索引，内联实现）
    uint8_t visited_temp[PLANNER_V3_BFS_MAX_BOXES] = {0};
    typedef struct {
      size_t temp_idx;
    } StackItem;
    StackItem stack[PLANNER_V3_BFS_MAX_BOXES];
    size_t stack_top = 0;
    
    // 如果当前箱子是副箱，先处理主箱
    if (temp_overlaps[i].valid && temp_overlaps[i].primary != SIZE_MAX) {
      size_t primary_temp = temp_overlaps[i].primary;
      size_t primary_orig = detached_indices[primary_temp];
      ignore_mask[primary_orig] = 1;
      if (!visited_temp[primary_temp]) {
        stack[stack_top++].temp_idx = primary_temp;
        visited_temp[primary_temp] = 1;
      }
    }
    
    // 作为主箱，递归忽略所有副箱
    if (!visited_temp[i]) {
      stack[stack_top++].temp_idx = i;
      visited_temp[i] = 1;
    }
    
    // 使用栈进行递归处理
    while (stack_top > 0) {
      size_t current_temp = stack[--stack_top].temp_idx;
      for (size_t s = 0; s < detached_count; ++s) {
        if (temp_overlaps[s].valid && temp_overlaps[s].primary == current_temp) {
          size_t sec_orig = detached_indices[s];
          ignore_mask[sec_orig] = 1;
          if (!visited_temp[s]) {
            stack[stack_top++].temp_idx = s;
            visited_temp[s] = 1;
          }
        }
      }
    }
    
    if (!planner_v3_bfs_compute_box_path_with_mask(
            rows, cols, current_boxes[box_idx], target, obstacles, obstacle_count,
            current_boxes, box_count, ignore_mask, box_idx, &planned_paths[i])) {
      // 第二次路径规划失败，无法构建链式
      if (out_status) {
        out_status->fail_step = 2;  // 第二次路径规划失败
        out_status->fail_box_idx = box_idx;
        out_status->fail_box_pos = current_boxes[box_idx];
      }
      return 0;
    }
  }
  
  // 第四步：第二次检测重叠（计算重叠路径末端坐标）
  // 使用free_paths和planned_paths重新检测重叠
  for (size_t secondary_i = 0; secondary_i < detached_count; secondary_i++) {
    PlannerBoxOverlap *ov = &temp_overlaps[secondary_i];
    if (!ov->valid || ov->primary == SIZE_MAX) {
      continue;
    }
    
    size_t primary_i = ov->primary;
    size_t overlap_len_second = 0;
    if (planned_paths[primary_i].valid && planned_paths[secondary_i].valid) {
      Point sec_start = current_boxes[detached_indices[secondary_i]];
      size_t box_path_primary_start = SIZE_MAX;
      size_t box_path_secondary_start = 0;
      
      const PlannerBoxPathInfo *primary_box_path = &planned_paths[primary_i];
      const PlannerBoxPathInfo *secondary_box_path = &planned_paths[secondary_i];
      
      for (size_t p = 0; p < primary_box_path->len; ++p) {
        if (primary_box_path->path[p].row == sec_start.row &&
            primary_box_path->path[p].col == sec_start.col) {
          box_path_primary_start = p;
          break;
        }
      }
      
      if (box_path_primary_start != SIZE_MAX) {
        while (box_path_primary_start + overlap_len_second + 1 < primary_box_path->len &&
               box_path_secondary_start + overlap_len_second + 1 < secondary_box_path->len) {
          Point dir_p = primary_box_path->dir[box_path_primary_start + overlap_len_second];
          Point dir_s = secondary_box_path->dir[box_path_secondary_start + overlap_len_second];
          Point next_p = primary_box_path->path[box_path_primary_start + overlap_len_second + 1];
          Point next_s = secondary_box_path->path[box_path_secondary_start + overlap_len_second + 1];
          if (dir_p.row != dir_s.row || dir_p.col != dir_s.col) {
            break;
          }
          if (next_p.row != next_s.row || next_p.col != next_s.col) {
            break;
          }
          overlap_len_second++;
        }
      }
    }
    
    ov->overlap_len_second = overlap_len_second;
    
    // 计算重叠路径末端坐标
    if (planned_paths[secondary_i].valid && overlap_len_second > 0) {
      size_t end_idx = overlap_len_second;
      if (end_idx < planned_paths[secondary_i].len) {
        ov->overlap_end_pos = planned_paths[secondary_i].path[end_idx];
      } else {
        ov->overlap_end_pos.row = -1;
        ov->overlap_end_pos.col = -1;
      }
    } else {
      ov->overlap_end_pos.row = -1;
      ov->overlap_end_pos.col = -1;
    }
  }
  
  // 第五步：构建链式（基于temp_overlaps）
  // 尝试每个脱离的箱子作为主箱，每个方向作为推动方向
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  
  for (size_t primary_i = 0; primary_i < detached_count; primary_i++) {
    size_t primary_orig = detached_indices[primary_i];
    Point primary_pos = current_boxes[primary_orig];
    if (primary_pos.row < 0 || primary_pos.col < 0) continue;
    
    for (int d = 0; d < 4; d++) {
      Point push_dir = {dirs[d][0], dirs[d][1]};
      
      // 尝试构建链式（使用temp_overlaps，但需要转换为原始索引）
      PlannerChainLocal test_chain;
      test_chain.count = 0;
      test_chain.dir = push_dir;
      
      // 主箱
      test_chain.indices[test_chain.count] = primary_orig;
      test_chain.overlap_lens[test_chain.count] = 0;
      test_chain.overlap_end_positions[test_chain.count].row = -1;
      test_chain.overlap_end_positions[test_chain.count].col = -1;
      test_chain.steps_walked[test_chain.count] = 0;
      test_chain.count++;
      
      // 沿着推动方向查找连续的副箱子
      size_t current_temp = primary_i;
      Point expected_pos = primary_pos;
      expected_pos.row += push_dir.row;
      expected_pos.col += push_dir.col;
      
      while (test_chain.count < PLANNER_V3_BFS_MAX_BOXES) {
        size_t found_temp = SIZE_MAX;
        size_t found_overlap_len = 0;
        
        for (size_t s_i = 0; s_i < detached_count; ++s_i) {
          if (s_i == current_temp) continue;
          
          int already_in_chain = 0;
          for (size_t c = 0; c < test_chain.count; ++c) {
            if (test_chain.indices[c] == detached_indices[s_i]) {
              already_in_chain = 1;
              break;
            }
          }
          if (already_in_chain) continue;
          
          Point box_pos = current_boxes[detached_indices[s_i]];
          if (box_pos.row < 0 || box_pos.col < 0) continue;
          
          if (box_pos.row == expected_pos.row && box_pos.col == expected_pos.col) {
            // 检查这个箱子的重叠信息
            if (temp_overlaps[s_i].valid && temp_overlaps[s_i].primary == current_temp &&
                temp_overlaps[s_i].dir.row == push_dir.row && temp_overlaps[s_i].dir.col == push_dir.col) {
              found_temp = s_i;
              found_overlap_len = temp_overlaps[s_i].overlap_len_second;
              break;
            }
            // 即使没有重叠信息，只要位置对就认为是链中的一员
            found_temp = s_i;
            found_overlap_len = 1;
            break;
          }
        }
        
        if (found_temp == SIZE_MAX) {
          break;
        }
        
        size_t found_orig = detached_indices[found_temp];
        test_chain.indices[test_chain.count] = found_orig;
        test_chain.overlap_lens[test_chain.count] = found_overlap_len;
        if (temp_overlaps[found_temp].valid && 
            temp_overlaps[found_temp].overlap_end_pos.row >= 0 && 
            temp_overlaps[found_temp].overlap_end_pos.col >= 0) {
          test_chain.overlap_end_positions[test_chain.count] = temp_overlaps[found_temp].overlap_end_pos;
        } else {
          test_chain.overlap_end_positions[test_chain.count].row = -1;
          test_chain.overlap_end_positions[test_chain.count].col = -1;
        }
        test_chain.steps_walked[test_chain.count] = 0;
        test_chain.count++;
        current_temp = found_temp;
        
        expected_pos.row += push_dir.row;
        expected_pos.col += push_dir.col;
      }
      
      if (test_chain.count < 2) continue;  // 至少需要2个箱子才能构成链式
      
      // 检查构建的链是否包含所有脱离的箱子
      int all_detached_included = 1;
      for (size_t i = 0; i < detached_count; i++) {
        int found = 0;
        for (size_t j = 0; j < test_chain.count; j++) {
          if (test_chain.indices[j] == detached_indices[i]) {
            found = 1;
            break;
          }
        }
        if (!found) {
          all_detached_included = 0;
          break;
        }
      }
      
      if (!all_detached_included) continue;
      
      // 检查链式推动是否可行（检查推箱位是否可达）
      size_t test_primary_idx = test_chain.indices[0];
      Point test_primary_pos = current_boxes[test_primary_idx];
      Point test_push_pos = {test_primary_pos.row - push_dir.row, 
                              test_primary_pos.col - push_dir.col};
      
      // 检查推箱位有效性
      if (!planner_v3_bfs_in_bounds(rows, cols, test_push_pos.row, test_push_pos.col) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, test_push_pos.row, test_push_pos.col)) {
        continue;
      }
      
      // 检查推箱位是否被非链内箱子占用
      int push_pos_blocked = 0;
      for (size_t bi = 0; bi < box_count; ++bi) {
        int in_chain = 0;
        for (size_t c = 0; c < test_chain.count; ++c) {
          if (test_chain.indices[c] == bi) {
            in_chain = 1;
            break;
          }
        }
        if (in_chain) continue;
        
        Point b = current_boxes[bi];
        if (b.row >= 0 && b.col >= 0 && b.row == test_push_pos.row && b.col == test_push_pos.col) {
          push_pos_blocked = 1;
          break;
        }
      }
      
      if (push_pos_blocked) continue;
      
      // 检查链尾移动后的位置是否有效
      size_t tail_idx = test_chain.indices[test_chain.count - 1];
      Point tail_pos = current_boxes[tail_idx];
      Point tail_next = {tail_pos.row + push_dir.row, tail_pos.col + push_dir.col};
      
      if (!planner_v3_bfs_in_bounds(rows, cols, tail_next.row, tail_next.col) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, tail_next.row, tail_next.col)) {
        continue;
      }
      
      // 检查链尾移动位置是否被非链内箱子占用
      int tail_next_blocked = 0;
      for (size_t bi = 0; bi < box_count; ++bi) {
        int in_chain = 0;
        for (size_t c = 0; c < test_chain.count; ++c) {
          if (test_chain.indices[c] == bi) {
            in_chain = 1;
            break;
          }
        }
        if (in_chain) continue;
        
        Point b = current_boxes[bi];
        if (b.row >= 0 && b.col >= 0 && b.row == tail_next.row && b.col == tail_next.col) {
          tail_next_blocked = 1;
          break;
        }
      }
      
      if (tail_next_blocked) continue;
      
      // 链式推动可行，返回结果
      *out_chain = test_chain;
      *out_push_dir = push_dir;
      if (out_status) {
        out_status->success = 1;
        out_status->fail_step = 0;
      }
      return 1;
    }
  }
  
  // 无法构建链式（所有尝试都失败）
  if (out_status) {
    out_status->fail_step = 3;  // 无法构建链式
    if (detached_count > 0) {
      out_status->fail_box_idx = detached_indices[0];
      out_status->fail_box_pos = current_boxes[detached_indices[0]];
    }
  }
  return 0;
}

// - 如果箱子到达与overlap的末端位置就检测为需要脱离链式
// - 如果分叉后车辆无法到达推位，则需要在回溯到上一步后再脱链
// - 从链尾开始向前检测，确保按顺序脱链
// - rollback_needed: 如果检测到需要脱链，是否需要在脱链前先回溯（由调用者决定）
// - saved_state: 保存的上一步状态（仅在rollback_needed=1时使用）
// - 新增：脱离时先尝试把所有脱离的箱子重新构成一个链式，检查是否可以推动，不行则返回到原来的单箱评分推动机制
static int planner_v3_bfs_chain_try_detach_any_box_by_overlap(
    int rows, int cols, PlannerChainLocal *chain,
    const size_t *box_targets, const Point *targets,
    const Point *obstacles, size_t obstacle_count,
    Point *current_car, Point *current_boxes, size_t box_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps,
    int *out_detached, int rollback_needed, const PlannerChainLocal *saved_chain_state,
    const Point *saved_car, const Point *saved_boxes, size_t saved_steps) {
  if (out_detached) *out_detached = 0;
  if (!chain || chain->count <= 1) return 0;

  // 从链尾向前遍历（跳过主箱，主箱索引0）
  for (int chain_idx = (int)chain->count - 1; chain_idx >= 1; chain_idx--) {
    size_t box_idx = chain->indices[chain_idx];
    
    // 获取当前箱子的位置和重叠路径的末端坐标
    Point current_pos = current_boxes[box_idx];
    if (current_pos.row < 0 || current_pos.col < 0) continue;
    
    // 使用全局的overlaps数组，确保与菜单显示的值一致
    // 而不是使用chain->overlap_end_positions（可能在构建链后overlaps被更新）
    Point overlap_end_pos = {-1, -1};
    if (g_planner_overlaps[box_idx].valid && 
        g_planner_overlaps[box_idx].overlap_end_pos.row >= 0 && 
        g_planner_overlaps[box_idx].overlap_end_pos.col >= 0) {
      overlap_end_pos = g_planner_overlaps[box_idx].overlap_end_pos;
    }
    // 如果全局overlaps中没有有效值，回退到使用chain中的值
    if (overlap_end_pos.row < 0 || overlap_end_pos.col < 0) {
      overlap_end_pos = chain->overlap_end_positions[chain_idx];
    }
    // 检查是否有有效的重叠路径末端坐标
    if (overlap_end_pos.row < 0 || overlap_end_pos.col < 0) continue;
    
    // 检测当前坐标和重叠路径的末端坐标是否相同
    if (current_pos.row != overlap_end_pos.row || current_pos.col != overlap_end_pos.col) {
      continue;  // 还未到达重叠路径的末端位置
    }
    
    // 箱子已到达重叠路径的末端位置，需要检测是否需要脱离
    if (box_targets[box_idx] == SIZE_MAX) continue;
    
    // 使用final_paths中对应箱子的路径（与菜单显示一致）
    const PlannerBoxPathOutput *box_path = NULL;
    if (box_idx < final_paths.box_count) {
      box_path = &final_paths.box_paths[box_idx];
    }
    if (!box_path || !box_path->valid || box_path->path_len == 0) continue;
    
    // 查找当前箱子在路径上的位置
    size_t path_idx = SIZE_MAX;
    for (size_t p = 0; p < box_path->path_len; ++p) {
      if (box_path->path[p].row == current_pos.row && 
          box_path->path[p].col == current_pos.col) {
        path_idx = p;
        break;
      }
    }
    
    if (path_idx == SIZE_MAX || path_idx + 1 >= box_path->path_len) continue;
    
    // 检测下一步推动位置（分叉后的推位）
    if (path_idx + 1 >= box_path->path_len) continue;
    Point next_pos = box_path->path[path_idx + 1];
    Point dir = {next_pos.row - current_pos.row, next_pos.col - current_pos.col};
    Point push_pos = {current_pos.row - dir.row, current_pos.col - dir.col};
    
    // 检查车辆是否能到达推位（考虑所有箱子，包括链内箱子，作为障碍）
    // 【修改】BFS检测时应把"链里所有的箱子"作为障碍保留
    // 确保链内所有箱子都被明确作为障碍，即使它们可能已经完成（负坐标）
    // 创建临时箱子数组，明确包含链内所有箱子作为障碍
    Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
    for (size_t bi = 0; bi < box_count; ++bi) {
      temp_boxes[bi] = current_boxes[bi];
    }
    
    // 确保链内所有箱子都被作为障碍保留（即使它们可能已经完成）
    for (size_t ci = 0; ci < chain->count; ++ci) {
      size_t chain_box_idx = chain->indices[ci];
      if (chain_box_idx < box_count) {
        Point chain_box_pos = current_boxes[chain_box_idx];
        // 如果链内箱子已完成（负坐标），仍然作为障碍保留（使用其最后有效位置）
        // 但这里我们只处理未完成的箱子，已完成的箱子在planner_v3_bfs_is_box_at中会被忽略
        // 所以这里只需要确保链内所有箱子都被包含在temp_boxes中
        temp_boxes[chain_box_idx] = chain_box_pos;
      }
    }
    
    int car_dist[PLANNER_V3_BFS_MAX_CELLS];
    int can_reach = planner_v3_bfs_global_bfs_from_target(
        rows, cols, push_pos, obstacles, obstacle_count,
        temp_boxes, box_count, SIZE_MAX, car_dist);
    
    int car_can_reach_push = 0;
    if (can_reach) {
      int car_idx = current_car->row * cols + current_car->col;
      if (car_idx >= 0 && car_idx < rows * cols && car_dist[car_idx] != INT_MAX) {
        car_can_reach_push = 1;
      }
    }
    
    // 判断车辆是否能到达分叉后下一步的推位
    // 如果车辆能到达推位，则在当前状态脱离
    // 如果车辆无法到达推位，则需要回溯到上一状态再脱离
    if (car_can_reach_push) {
      // 车辆能到达推位，在当前状态脱离（不需要回溯）
      // 备份用于失败回滚
      PlannerChainLocal chain_bak = *chain;
      Point car_bak = *current_car;
      Point boxes_bak[PLANNER_V3_BFS_MAX_BOXES];
      for (size_t i = 0; i < box_count; ++i) boxes_bak[i] = current_boxes[i];
      size_t steps_bak = *out_steps;
      
      // 收集所有要脱离的箱子（从chain_idx到chain->count-1的所有箱子）
      size_t detached_indices[PLANNER_V3_BFS_MAX_BOXES];
      size_t detached_count = 0;
      for (size_t i = chain_idx; i < chain->count; i++) {
        detached_indices[detached_count++] = chain->indices[i];
      }
      
      // 从链中移除该箱子及其后所有箱子
      chain->count = chain_idx;
      
      // 每次进入脱链尝试前清状态
      g_rebuild_chain_status.exec_success = -1;
      g_rebuild_chain_status.exec_res = 0;
      
      // 【新增】先尝试把所有脱离的箱子重新构成一个链式，检查是否可以推动
      // 与一开始构建链式的流程一致：先第一次路径规划，然后标记，然后第二次路径规划
      PlannerChainLocal rebuilt_chain;
      Point rebuilt_push_dir;
      PlannerRebuildChainStatus rebuild_status;
      int can_rebuild_chain = planner_v3_bfs_try_rebuild_chain_for_detached(
          rows, cols, detached_indices, detached_count,
          current_boxes, box_count, obstacles, obstacle_count,
          box_targets, targets, &rebuilt_chain, &rebuilt_push_dir, &rebuild_status);
      
      // 保存重构链状态信息到全局变量（供菜单显示）
      g_rebuild_chain_status = rebuild_status;
      
      int exec_res = 0;
      if (can_rebuild_chain) {
        // 可以重新构成链式，循环推动直到所有箱子都到达目标点
        while (rebuilt_chain.count > 0 && 
               !planner_v3_bfs_chain_all_finished(&rebuilt_chain, current_boxes, box_targets, targets)) {
          // 检查车辆是否能到达推箱位
          size_t rebuilt_primary_idx = rebuilt_chain.indices[0];
          Point rebuilt_primary_pos = current_boxes[rebuilt_primary_idx];
          Point rebuilt_push_pos = {rebuilt_primary_pos.row - rebuilt_push_dir.row,
                                    rebuilt_primary_pos.col - rebuilt_push_dir.col};
          
          int car_can_reach_rebuilt_push = 0;
          int rebuilt_car_dist[PLANNER_V3_BFS_MAX_CELLS];
          if (planner_v3_bfs_global_bfs_from_target(
                  rows, cols, rebuilt_push_pos, obstacles, obstacle_count,
                  current_boxes, box_count, SIZE_MAX, rebuilt_car_dist)) {
            int car_idx = current_car->row * cols + current_car->col;
            if (car_idx >= 0 && car_idx < rows * cols && 
                rebuilt_car_dist[car_idx] != INT_MAX) {
              car_can_reach_rebuilt_push = 1;
            }
          }
          
          if (car_can_reach_rebuilt_push) {
            // 车辆移动到推箱位
            Point temp_path[256];
            size_t temp_len = 0;
            if (planner_v3_bfs_astar_with_dist(rows, cols, *current_car, rebuilt_push_pos,
                                                obstacles, obstacle_count,
                                                current_boxes, box_count, rebuilt_car_dist,
                                                temp_path, 256, &temp_len, 1)) {
              // 执行移动路径（跳过起点）
              for (size_t i = 1; i < temp_len; i++) {
                if (*out_steps >= path_capacity) {
                  exec_res = -7;
                  break;
                }
                path_buffer[(*out_steps)++] = temp_path[i];
              }
              if (exec_res == 0) {
                *current_car = rebuilt_push_pos;
                
                // 尝试链式推动
                exec_res = planner_v3_bfs_chain_push_straight(
                    rows, cols, &rebuilt_chain, rebuilt_push_dir,
                    obstacles, obstacle_count, current_car, current_boxes, box_count,
                    box_targets, targets, path_buffer, path_capacity, out_steps);
                
                if (exec_res != 0) {
                  break;  // 推动失败，退出循环
                }
              } else {
                break;  // 移动失败，退出循环
              }
            } else {
              exec_res = -6;
              break;
            }
          } else {
            exec_res = -6;
            break;
          }
        }
      } else {
        // 无法构建链式，循环推动每个箱子直到所有箱子都到达目标点
        exec_res = 0;
        for (size_t i = 0; i < detached_count; i++) {
          size_t detached_box_idx = detached_indices[i];
          if (box_targets[detached_box_idx] == SIZE_MAX) {
            exec_res = -6;
            break;
          }
          
          // 循环推动直到该箱子到达目标点
          while (current_boxes[detached_box_idx].row >= 0 && 
                 current_boxes[detached_box_idx].col >= 0) {
            Point box_pos = current_boxes[detached_box_idx];
            Point target = targets[box_targets[detached_box_idx]];
            if (box_pos.row == target.row && box_pos.col == target.col) {
              // 箱子已到达目标点
              current_boxes[detached_box_idx].row = -1;
              current_boxes[detached_box_idx].col = -1;
              break;
            }
            
            int single_res = planner_v3_bfs_push_single_box_scored(
                rows, cols, detached_box_idx, box_targets[detached_box_idx], targets,
                obstacles, obstacle_count, current_car, current_boxes, box_count,
                path_buffer, path_capacity, out_steps);
            if (single_res != 0) {
              exec_res = single_res;
              break;
            }
          }
          
          if (exec_res != 0) {
            break;
          }
        }
      }
      
      // 执行结束后（无论成功失败）都写一次
      // 但只有在 can_rebuild_chain == 1 时才更新执行状态
      if (can_rebuild_chain) {
        g_rebuild_chain_status.exec_success = (exec_res == 0 ? 1 : 0);
        g_rebuild_chain_status.exec_res = exec_res;
        
        // 如果执行失败，记录失败位置
        if (exec_res != 0 && rebuilt_chain.count > 0) {
          g_rebuild_chain_status.exec_fail_box_idx = rebuilt_chain.indices[0];
          g_rebuild_chain_status.exec_fail_pos = current_boxes[rebuilt_chain.indices[0]];
        } else {
          g_rebuild_chain_status.exec_fail_box_idx = SIZE_MAX;
          g_rebuild_chain_status.exec_fail_pos.row = -1;
          g_rebuild_chain_status.exec_fail_pos.col = -1;
        }
      } else {
        // 无法构建链式，保持未尝试状态
        // exec_success 已经在重建链函数中设置为 -1，这里不需要更新
        g_rebuild_chain_status.exec_res = 0;  // 未尝试执行，exec_res 为 0
        g_rebuild_chain_status.exec_fail_box_idx = SIZE_MAX;
        g_rebuild_chain_status.exec_fail_pos.row = -1;
        g_rebuild_chain_status.exec_fail_pos.col = -1;
      }
      
      if (exec_res != 0) {
        // 推失败：回滚，保持旧行为（不脱链）
        *chain = chain_bak;
        *current_car = car_bak;
        for (size_t i = 0; i < box_count; ++i) current_boxes[i] = boxes_bak[i];
        *out_steps = steps_bak;
        // 在触发回滚的分支里也覆盖 exec_success=0
        g_rebuild_chain_status.exec_success = 0;
        g_rebuild_chain_status.exec_res = exec_res;
        continue;  // 尝试下一个箱子
      }
      
      // 脱链成功
      if (out_detached) *out_detached = 1;
      return 0;
    } else {
      // 车辆无法到达推位，需要回溯到上一状态再脱离
      // 如果需要在脱链前回溯
      if (rollback_needed && saved_chain_state && saved_car && saved_boxes) {
        // 先回溯到保存的状态
        *chain = *saved_chain_state;
        *current_car = *saved_car;
        for (size_t i = 0; i < box_count; ++i) {
          current_boxes[i] = saved_boxes[i];
        }
        *out_steps = saved_steps;
        
        // 重新查找回溯后箱子在路径上的位置
        Point rollback_pos = current_boxes[box_idx];
        if (rollback_pos.row >= 0 && rollback_pos.col >= 0) {
          path_idx = SIZE_MAX;
          for (size_t p = 0; p < box_path->path_len; ++p) {
            if (box_path->path[p].row == rollback_pos.row && 
                box_path->path[p].col == rollback_pos.col) {
              path_idx = p;
              break;
            }
          }
        }
        
        // 如果回溯后找不到路径位置，尝试下一个箱子
        if (path_idx == SIZE_MAX || path_idx + 1 >= box_path->path_len) {
          continue;
        }
      }
      
      // 备份用于失败回滚
      PlannerChainLocal chain_bak = *chain;
      Point car_bak = *current_car;
      Point boxes_bak[PLANNER_V3_BFS_MAX_BOXES];
      for (size_t i = 0; i < box_count; ++i) boxes_bak[i] = current_boxes[i];
      size_t steps_bak = *out_steps;
      
      // 收集所有要脱离的箱子（从chain_idx到chain->count-1的所有箱子）
      size_t detached_indices[PLANNER_V3_BFS_MAX_BOXES];
      size_t detached_count = 0;
      for (size_t i = chain_idx; i < chain->count; i++) {
        detached_indices[detached_count++] = chain->indices[i];
      }
      
      // 从链中移除该箱子及其后所有箱子
      chain->count = chain_idx;
      
      // 每次进入脱链尝试前清状态
      g_rebuild_chain_status.exec_success = -1;
      g_rebuild_chain_status.exec_res = 0;
      
      // 【新增】先尝试把所有脱离的箱子重新构成一个链式，检查是否可以推动
      // 与一开始构建链式的流程一致：先第一次路径规划，然后标记，然后第二次路径规划
      PlannerChainLocal rebuilt_chain;
      Point rebuilt_push_dir;
      PlannerRebuildChainStatus rebuild_status;
      int can_rebuild_chain = planner_v3_bfs_try_rebuild_chain_for_detached(
          rows, cols, detached_indices, detached_count,
          current_boxes, box_count, obstacles, obstacle_count,
          box_targets, targets, &rebuilt_chain, &rebuilt_push_dir, &rebuild_status);
      
      // 保存重构链状态信息到全局变量（供菜单显示）
      g_rebuild_chain_status = rebuild_status;
      
      int exec_res = 0;
      if (can_rebuild_chain) {
        // 可以重新构成链式，循环推动直到所有箱子都到达目标点
        while (rebuilt_chain.count > 0 && 
               !planner_v3_bfs_chain_all_finished(&rebuilt_chain, current_boxes, box_targets, targets)) {
          // 检查车辆是否能到达推箱位
          size_t rebuilt_primary_idx = rebuilt_chain.indices[0];
          Point rebuilt_primary_pos = current_boxes[rebuilt_primary_idx];
          Point rebuilt_push_pos = {rebuilt_primary_pos.row - rebuilt_push_dir.row,
                                    rebuilt_primary_pos.col - rebuilt_push_dir.col};
          
          int car_can_reach_rebuilt_push = 0;
          int rebuilt_car_dist[PLANNER_V3_BFS_MAX_CELLS];
          if (planner_v3_bfs_global_bfs_from_target(
                  rows, cols, rebuilt_push_pos, obstacles, obstacle_count,
                  current_boxes, box_count, SIZE_MAX, rebuilt_car_dist)) {
            int car_idx = current_car->row * cols + current_car->col;
            if (car_idx >= 0 && car_idx < rows * cols && 
                rebuilt_car_dist[car_idx] != INT_MAX) {
              car_can_reach_rebuilt_push = 1;
            }
          }
          
          if (car_can_reach_rebuilt_push) {
            // 车辆移动到推箱位
            Point temp_path[256];
            size_t temp_len = 0;
            if (planner_v3_bfs_astar_with_dist(rows, cols, *current_car, rebuilt_push_pos,
                                                obstacles, obstacle_count,
                                                current_boxes, box_count, rebuilt_car_dist,
                                                temp_path, 256, &temp_len, 1)) {
              // 执行移动路径（跳过起点）
              for (size_t i = 1; i < temp_len; i++) {
                if (*out_steps >= path_capacity) {
                  exec_res = -7;
                  break;
                }
                path_buffer[(*out_steps)++] = temp_path[i];
              }
              if (exec_res == 0) {
                *current_car = rebuilt_push_pos;
                
                // 尝试链式推动
                exec_res = planner_v3_bfs_chain_push_straight(
                    rows, cols, &rebuilt_chain, rebuilt_push_dir,
                    obstacles, obstacle_count, current_car, current_boxes, box_count,
                    box_targets, targets, path_buffer, path_capacity, out_steps);
                
                if (exec_res != 0) {
                  break;  // 推动失败，退出循环
                }
              } else {
                break;  // 移动失败，退出循环
              }
            } else {
              exec_res = -6;
              break;
            }
          } else {
            exec_res = -6;
            break;
          }
        }
      } else {
        // 无法构建链式，循环推动每个箱子直到所有箱子都到达目标点
        exec_res = 0;
        for (size_t i = 0; i < detached_count; i++) {
          size_t detached_box_idx = detached_indices[i];
          if (box_targets[detached_box_idx] == SIZE_MAX) {
            exec_res = -6;
            break;
          }
          
          // 循环推动直到该箱子到达目标点
          while (current_boxes[detached_box_idx].row >= 0 && 
                 current_boxes[detached_box_idx].col >= 0) {
            Point box_pos = current_boxes[detached_box_idx];
            Point target = targets[box_targets[detached_box_idx]];
            if (box_pos.row == target.row && box_pos.col == target.col) {
              // 箱子已到达目标点
              current_boxes[detached_box_idx].row = -1;
              current_boxes[detached_box_idx].col = -1;
              break;
            }
            
            int single_res = planner_v3_bfs_push_single_box_scored(
                rows, cols, detached_box_idx, box_targets[detached_box_idx], targets,
                obstacles, obstacle_count, current_car, current_boxes, box_count,
                path_buffer, path_capacity, out_steps);
            if (single_res != 0) {
              exec_res = single_res;
              break;
            }
          }
          
          if (exec_res != 0) {
            break;
          }
        }
      }
      
      // 执行结束后（无论成功失败）都写一次
      // 但只有在 can_rebuild_chain == 1 时才更新执行状态
      if (can_rebuild_chain) {
        g_rebuild_chain_status.exec_success = (exec_res == 0 ? 1 : 0);
        g_rebuild_chain_status.exec_res = exec_res;
        
        // 如果执行失败，记录失败位置
        if (exec_res != 0 && rebuilt_chain.count > 0) {
          g_rebuild_chain_status.exec_fail_box_idx = rebuilt_chain.indices[0];
          g_rebuild_chain_status.exec_fail_pos = current_boxes[rebuilt_chain.indices[0]];
        } else {
          g_rebuild_chain_status.exec_fail_box_idx = SIZE_MAX;
          g_rebuild_chain_status.exec_fail_pos.row = -1;
          g_rebuild_chain_status.exec_fail_pos.col = -1;
        }
      } else {
        // 无法构建链式，保持未尝试状态
        // exec_success 已经在重建链函数中设置为 -1，这里不需要更新
        g_rebuild_chain_status.exec_res = 0;  // 未尝试执行，exec_res 为 0
        g_rebuild_chain_status.exec_fail_box_idx = SIZE_MAX;
        g_rebuild_chain_status.exec_fail_pos.row = -1;
        g_rebuild_chain_status.exec_fail_pos.col = -1;
      }
      
      if (exec_res != 0) {
        // 推失败：回滚，保持旧行为（不脱链）
        *chain = chain_bak;
        *current_car = car_bak;
        for (size_t i = 0; i < box_count; ++i) current_boxes[i] = boxes_bak[i];
        *out_steps = steps_bak;
        // 在触发回滚的分支里也覆盖 exec_success=0
        g_rebuild_chain_status.exec_success = 0;
        g_rebuild_chain_status.exec_res = exec_res;
        continue;  // 尝试下一个箱子
      }
      
      // 脱链成功
      if (out_detached) *out_detached = 1;
      return 0;
    }
  }
  
  return 0;
}


// 【改进版】在当前状态下尝试脱链任意满足条件的箱子（不只是链尾）
// 这是"先推脱链副箱，再继续推链"的实现：脱链发生时，会立刻独立推该箱到目标
// 新增：检测链中任意箱子是否即将分叉且车辆无法到达推位，在分叉前脱链
// 注意：该函数是 best-effort；若同步路径失败或独立推失败，会保持旧行为（不强制脱链）
static int planner_v3_bfs_chain_detach_tail_if_overlap_done(
    int rows, int cols, PlannerChainLocal *chain,
    const size_t *box_targets, const Point *targets,
    const Point *obstacles, size_t obstacle_count,
    Point *current_car, Point *current_boxes, size_t box_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps) {
  if (!chain || chain->count <= 1) return 0;

  while (chain->count > 1) {
    // 检测链中任意箱子是否需要在分叉前脱链
    // 注意：此函数在推动后调用，不需要回溯（传递rollback_needed=0）
    int detached_any = 0;
    (void)planner_v3_bfs_chain_try_detach_any_box_by_overlap(
        rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
        current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps,
        &detached_any, 0, NULL, NULL, NULL, 0);
    
    if (detached_any) {
      // 成功脱链一个箱子，继续检测其他箱子
      if (chain->count <= 1) return 0;
      continue;
    }
    
    // 没有箱子需要脱链，退出循环
    break;
  }
  return 0;
}

static int planner_v3_bfs_push_single_box_scored(
    int rows, int cols, size_t box_idx, size_t target_idx, const Point *targets,
    const Point *obstacles, size_t obstacle_count, Point *current_car, Point *current_boxes,
    size_t box_count, Point *path_buffer, size_t path_capacity, size_t *out_steps) {
  if (target_idx == SIZE_MAX) {
    last_err_stage = 3;  // 第二次规划失败
    last_err_detail = 5; // 单箱推送目标未分配
    return -6;
  }
  Point target = targets[target_idx];
  Point box = current_boxes[box_idx];
  if (box.row == target.row && box.col == target.col) {
    current_boxes[box_idx].row = -1;
    current_boxes[box_idx].col = -1;
    return 0;
  }

  int target_dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                             current_boxes, box_count, box_idx, target_dist)) {
    last_err_stage = 3;  // 第二次规划失败
    last_err_detail = 6; // 单箱推送全局BFS失败
    return -6;
  }

  int step_count = 0;
  int last_dr = 0;
  int last_dc = 0;
  int reverse_count = 0;

  while (box.row != target.row || box.col != target.col) {
    if (step_count++ >= PLANNER_V3_BFS_MAX_GREEDY_STEPS) {
      return -5;
    }

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

      if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, new_box_row, new_box_col) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_row, push_col)) {
        continue;
      }

      if (planner_v3_bfs_is_box_at(current_boxes, box_count, new_box_row, new_box_col, box_idx) ||
          planner_v3_bfs_is_box_at(current_boxes, box_count, push_row, push_col, box_idx)) {
        continue;
      }

      if (new_box_row == target.row && new_box_col == target.col) {
        // ok
      } else {
        if (!planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count, current_boxes,
                                           box_count, box_idx, (Point){new_box_row, new_box_col},
                                           target)) {
          continue;
        }
      }

      Point push_from = {push_row, push_col};

      int dist_after;
      int idx = new_box_row * cols + new_box_col;
      if (idx >= 0 && idx < rows * cols && target_dist[idx] != INT_MAX) {
        dist_after = target_dist[idx];
      } else {
        dist_after = INT_MAX / 100;
      }

      int adj_pen = planner_v3_bfs_adjacent_blockers(rows, cols, obstacles, obstacle_count,
                                                     current_boxes, box_count, box_idx,
                                                     new_box_row, new_box_col);

      int reverse_pen = 0;
      if (last_dr == -dirs[i][0] && last_dc == -dirs[i][1]) {
        int power = 5;
        for (int p = 0; p < reverse_count; ++p) {
          power *= 5;
        }
        reverse_pen = power;
      }

      int car_to_push;
      int car_to_push_dist[PLANNER_V3_BFS_MAX_CELLS];
      // 计算车到推位的BFS真实距离时，需要考虑所有箱子作为障碍
      if (planner_v3_bfs_global_bfs_from_target(rows, cols, push_from, obstacles, obstacle_count,
                                                current_boxes, box_count, SIZE_MAX, car_to_push_dist)) {
        int car_idx = current_car->row * cols + current_car->col;
        if (car_idx >= 0 && car_idx < rows * cols && car_to_push_dist[car_idx] != INT_MAX) {
          car_to_push = car_to_push_dist[car_idx];
        } else {
          car_to_push = INT_MAX / 100;
        }
      } else {
        car_to_push = planner_v3_bfs_manhattan(*current_car, push_from);
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
    for (int i = 0; i < sorted_count - 1; ++i) {
      for (int j = 0; j < sorted_count - 1 - i; ++j) {
        if (candidates[sorted_dirs[j]].score > candidates[sorted_dirs[j + 1]].score) {
          int tmp = sorted_dirs[j];
          sorted_dirs[j] = sorted_dirs[j + 1];
          sorted_dirs[j + 1] = tmp;
        }
      }
    }

    if (sorted_count == 0) {
      last_err_stage = 3;  // 第二次规划失败
      last_err_detail = 7; // 单箱推送无可行方向
      return -6;
    }

    int push_success = 0;
    DirCandidate chosen;
    for (int try_idx = 0; try_idx < sorted_count; ++try_idx) {
      int dir_idx = sorted_dirs[try_idx];
      DirCandidate candidate = candidates[dir_idx];

      Point car_before = *current_car;
      size_t steps_before = *out_steps;

      if (current_car->row == candidate.push_from.row &&
          current_car->col == candidate.push_from.col) {
        if (*out_steps == 0) {
          if (*out_steps >= path_capacity) {
            return -7;
          }
          path_buffer[(*out_steps)++] = *current_car;
        } else {
          Point last = path_buffer[*out_steps - 1];
          if (last.row != current_car->row || last.col != current_car->col) {
            if (!planner_v3_bfs_check_adjacent(last, *current_car)) {
              *current_car = car_before;
              *out_steps = steps_before;
              continue;
            }
            if (*out_steps >= path_capacity) {
              return -7;
            }
            path_buffer[(*out_steps)++] = *current_car;
          }
        }
        chosen = candidate;
        push_success = 1;
        break;
      }

      int car_result = planner_v3_bfs_car_move_with_global_astar(
          rows, cols, current_car, candidate.push_from, obstacles, obstacle_count, current_boxes,
          box_count, path_buffer, path_capacity, out_steps);
      if (car_result == -7) {
        return -7;
      }
      if (car_result == 0) {
        *current_car = car_before;
        *out_steps = steps_before;
        continue;
      }
      chosen = candidate;
      push_success = 1;
      break;
    }

    if (!push_success) {
      last_err_stage = 3;  // 第二次规划失败
      last_err_detail = 8; // 单箱推送无法到达推位
      return -6;
    }

    if (current_car->row != chosen.push_from.row || current_car->col != chosen.push_from.col) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 15; // 车未到达推位
      return -6;
    }

    int dist_to_box = planner_v3_bfs_manhattan(*current_car, box);
    if (dist_to_box != 1) {
      last_err_stage = 5;  // 链式推动内部失败
      last_err_detail = 29; // 车到箱子距离不为1
      return -6;
    }

    Point box_old_pos = box;
    box = chosen.box_next;
    current_boxes[box_idx] = box;

    *current_car = box_old_pos;
    if (*out_steps >= path_capacity) {
      return -7;
    }
    if (*out_steps > 0 &&
        !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], *current_car)) {
      last_err_stage = 4;  // 车辆路径连续性失败
      last_err_detail = 2; // 推箱后车位置不连续
      return -6;
    }
    path_buffer[(*out_steps)++] = *current_car;

    if (last_dr == -chosen.dr && last_dc == -chosen.dc && (last_dr != 0 || last_dc != 0)) {
      reverse_count++;
    } else {
      reverse_count = 0;
    }
    last_dr = chosen.dr;
    last_dc = chosen.dc;
  }

  current_boxes[box_idx].row = -1;
  current_boxes[box_idx].col = -1;
  return 0;
}

// 解散链并逐个推送链中所有箱子到各自目标
static int planner_v3_bfs_split_chain_and_finish(
    int rows, int cols, const PlannerChainLocal *chain,
    const size_t *box_targets, const Point *targets,
    const Point *obstacles, size_t obstacle_count,
    Point *current_car, Point *current_boxes, size_t box_count,
    Point *path_buffer, size_t path_capacity, size_t *out_steps) {
  if (!chain || chain->count == 0) return 0;
  
  // 先检查链中是否所有箱子都已完成
  if (planner_v3_bfs_chain_all_finished(chain, current_boxes, box_targets, targets)) {
    // 所有箱子都已完成，标记为完成并返回
    for (size_t c = 0; c < chain->count; ++c) {
      size_t box_idx = chain->indices[c];
      current_boxes[box_idx].row = -1;
      current_boxes[box_idx].col = -1;
    }
    return 0;
  }
  
  // 从链尾开始，逐个推送到目标
  for (int i = (int)chain->count - 1; i >= 0; --i) {
    size_t box_idx = chain->indices[i];
    Point box_pos = current_boxes[box_idx];
    
    // 跳过已完成的箱子
    if (box_pos.row < 0 || box_pos.col < 0) continue;
    if (box_targets[box_idx] == SIZE_MAX) continue;
    
    Point target = targets[box_targets[box_idx]];
    
    // 如果已在目标位置
    if (box_pos.row == target.row && box_pos.col == target.col) {
      current_boxes[box_idx].row = -1;
      current_boxes[box_idx].col = -1;
      continue;
    }
    
    PlannerBoxPathInfo path_info;
    if (!planner_v3_bfs_compute_box_path_with_mask(
            rows, cols, box_pos, target, obstacles, obstacle_count,
            current_boxes, box_count, NULL, box_idx, &path_info)) {
      // 如果计算路径失败，检查是否所有箱子都已完成
      if (planner_v3_bfs_chain_all_finished(chain, current_boxes, box_targets, targets)) {
        // 所有箱子都已完成，标记为完成并返回
        for (size_t c = 0; c < chain->count; ++c) {
          size_t c_idx = chain->indices[c];
          current_boxes[c_idx].row = -1;
          current_boxes[c_idx].col = -1;
        }
        return 0;
      }
      last_err_stage = 3;  // 第二次规划失败
      last_err_detail = 9; // 链解散后路径计算失败
      return -6;
    }
    
    int res = planner_v3_bfs_execute_single_box_path(
        rows, cols, box_idx, &path_info, obstacles, obstacle_count,
        current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
    if (res != 0) return res;
    
    current_boxes[box_idx].row = -1;
    current_boxes[box_idx].col = -1;
  }
  
  return 0;
}

// 新版多副箱子链式推动函数
// 支持主箱推副箱1，副箱1推副箱2...的链式推动
// 直行时：所有链中箱子同时移动一格
// 拐弯时：车推链尾副箱朝新方向，然后推主箱朝原方向，循环直到主箱到拐弯点
static int planner_v3_bfs_execute_multi_chain(
    int rows, int cols, PlannerChainLocal *chain,
    const PlannerBoxPathInfo *planned_paths, const PlannerBoxOverlap *overlaps,
    size_t *box_targets, const Point *targets, const Point *obstacles,
    size_t obstacle_count, Point *current_car, Point *current_boxes,
    size_t box_count, Point *path_buffer, size_t path_capacity,
    size_t *out_steps) {
  // 静态变量：保存每个主箱上一次链式箱子的索引集合（与planner_v3_bfs_push_primary_with_chain共享）
  static size_t exec_last_chain_indices[PLANNER_V3_BFS_MAX_BOXES][PLANNER_V3_BFS_MAX_BOXES];
  static size_t exec_last_chain_counts[PLANNER_V3_BFS_MAX_BOXES] = {0};
  if (!chain) {
    last_err_stage = 5;  // 链式推动内部失败
    last_err_detail = 1; // 链指针为空
    return -6;
  }
  
  // 修复1：空链检查 - 如果链为空，返回0（所有箱子都已完成）
  if (chain->count == 0) return 0;
  
  // 修复2：检查链中所有箱子是否都已完成
  if (planner_v3_bfs_chain_all_finished(chain, current_boxes, box_targets, targets)) {
    // 所有箱子都已完成，标记为完成并返回
    for (size_t c = 0; c < chain->count; ++c) {
      size_t box_idx = chain->indices[c];
      current_boxes[box_idx].row = -1;
      current_boxes[box_idx].col = -1;
    }
    return 0;
  }
  
  // 同步链尾（驱动箱）的路径状态（支持路径失效时自动重算）
  (void)planned_paths;  // 兼容参数：链尾实际使用 g_planner_planned_paths（会在运行中更新）
  size_t tail_idx = SIZE_MAX;
  const PlannerBoxPathInfo *tail_path = NULL;
  size_t path_idx = 0;
  if (planner_v3_bfs_chain_sync_tail_path_state(
          rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
          current_boxes, box_count, &tail_idx, &tail_path, &path_idx) != 0) {
    last_err_stage = 5;  // 链式推动内部失败
    last_err_detail = 30; // 多链执行同步链尾路径失败
    return -6;
  }
  Point last_dir = chain->dir;
  
  while (path_idx + 1 < tail_path->len && chain->count > 0) {
    Point tail_curr = current_boxes[tail_idx];
    Point tail_next = tail_path->path[path_idx + 1];
    Point dir = {tail_next.row - tail_curr.row, tail_next.col - tail_curr.col};
    
    // 检查是否有新的箱子要加入链尾（在tail_next位置）
    size_t new_tail = SIZE_MAX;
    for (size_t t = 0; t < box_count; ++t) {
      // 检查是否已在链中
      int in_chain = 0;
      for (size_t c = 0; c < chain->count; ++c) {
        if (chain->indices[c] == t) { in_chain = 1; break; }
      }
      if (in_chain) continue;
      
      Point tb = current_boxes[t];
      if (tb.row < 0 || tb.col < 0) continue;
      
      if (tb.row == tail_next.row && tb.col == tail_next.col) {
        // 检查重叠信息
        if (overlaps && overlaps[t].valid && overlaps[t].primary == tail_idx &&
            overlaps[t].dir.row == dir.row && overlaps[t].dir.col == dir.col) {
          new_tail = t;
          break;
        }
        // 即使没有明确的重叠信息，位置对也认为是新链尾
        new_tail = t;
        break;
      }
    }
    
    if (new_tail != SIZE_MAX && chain->count < PLANNER_V3_BFS_MAX_BOXES) {
      // 将新箱子加入链尾
      size_t prev_tail_idx = chain->indices[chain->count - 1];
      size_t new_overlap_len = 0;
      
      // 从overlaps获取重叠长度（使用第二次路径规划的overlap_len_second）
      if (overlaps && overlaps[new_tail].valid && overlaps[new_tail].primary == prev_tail_idx &&
          overlaps[new_tail].dir.row == dir.row && overlaps[new_tail].dir.col == dir.col) {
        new_overlap_len = overlaps[new_tail].overlap_len_second;
      } else {
        new_overlap_len = 1;  // 默认重叠长度为1
      }
      
      chain->indices[chain->count] = new_tail;
      chain->overlap_lens[chain->count] = new_overlap_len;
      // 保存重叠路径的末端坐标
      if (overlaps && overlaps[new_tail].valid && overlaps[new_tail].overlap_end_pos.row >= 0 && overlaps[new_tail].overlap_end_pos.col >= 0) {
        chain->overlap_end_positions[chain->count] = overlaps[new_tail].overlap_end_pos;
      } else {
        chain->overlap_end_positions[chain->count].row = -1;
        chain->overlap_end_positions[chain->count].col = -1;
      }
      chain->steps_walked[chain->count] = 0;
      chain->count++;
      tail_idx = new_tail;
      
      // 【新增】检测到新箱子动态加入，触发目标重分配
      if (chain->count > 1) {
        size_t primary_idx = chain->indices[0];
        // 检测是否有新箱子加入链式
        int has_new_box = 0;
        if (primary_idx < PLANNER_V3_BFS_MAX_BOXES) {
          // 如果上一次链式为空（第一次构建链），视为有新箱子加入
          if (exec_last_chain_counts[primary_idx] == 0) {
            has_new_box = 1;
          } else {
            // 检查当前链中的每个箱子是否在上一次的链中
            for (size_t c = 0; c < chain->count; ++c) {
              size_t box_idx = chain->indices[c];
              int found_in_last = 0;
              for (size_t l = 0; l < exec_last_chain_counts[primary_idx]; ++l) {
                if (exec_last_chain_indices[primary_idx][l] == box_idx) {
                  found_in_last = 1;
                  break;
                }
              }
              if (!found_in_last) {
                // 发现新箱子加入
                has_new_box = 1;
                break;
              }
            }
          }
        } else {
          // 如果主箱索引超出范围，第一次构建链时也视为有新箱子（链式首次形成）
          has_new_box = 1;
        }
        
        // 如果有新箱子加入，进行目标重分配
        if (has_new_box) {
          int changed = planner_v3_bfs_chain_reassign_targets_tail_to_head(
              rows, cols, chain, current_boxes, box_count, box_targets, targets,
              obstacles, obstacle_count);
          if (changed) {
            // 目标发生变化：返回重规划信号，回到第一次路径规划阶段重算
            // 更新上一次链式状态
            if (primary_idx < PLANNER_V3_BFS_MAX_BOXES) {
              exec_last_chain_counts[primary_idx] = chain->count;
              for (size_t c = 0; c < chain->count; ++c) {
                exec_last_chain_indices[primary_idx][c] = chain->indices[c];
              }
            }
            return PLANNER_V3_BFS_REPLAN;
          }
          // 即使目标没有变化，也更新上一次链式状态（因为链式已经变化）
          if (primary_idx < PLANNER_V3_BFS_MAX_BOXES) {
            exec_last_chain_counts[primary_idx] = chain->count;
            for (size_t c = 0; c < chain->count; ++c) {
              exec_last_chain_indices[primary_idx][c] = chain->indices[c];
            }
          }
        } else {
          // 没有新箱子加入（可能有箱子脱离），更新链式状态但不进行重分配
          if (primary_idx < PLANNER_V3_BFS_MAX_BOXES) {
            exec_last_chain_counts[primary_idx] = chain->count;
            for (size_t c = 0; c < chain->count; ++c) {
              exec_last_chain_indices[primary_idx][c] = chain->indices[c];
            }
          }
        }
      }
      
      // 获取新链尾的路径
      if (box_targets[tail_idx] == SIZE_MAX) {
        return planner_v3_bfs_split_chain_and_finish(
            rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
            current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
      }
      
      // 重新计算新链尾的路径
      PlannerBoxPathInfo new_tail_path;
      uint8_t ignore_mask[PLANNER_V3_BFS_MAX_BOXES] = {0};
      for (size_t c = 0; c < chain->count - 1; ++c) {
        ignore_mask[chain->indices[c]] = 1;
      }
      if (!planner_v3_bfs_compute_box_path_with_mask(
              rows, cols, current_boxes[tail_idx], targets[box_targets[tail_idx]],
              obstacles, obstacle_count, current_boxes, box_count, ignore_mask, tail_idx,
              &new_tail_path)) {
        return planner_v3_bfs_split_chain_and_finish(
            rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
            current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
      }
      g_planner_planned_paths[tail_idx] = new_tail_path;
      tail_path = &g_planner_planned_paths[tail_idx];
      
      // 重新查找路径索引
      path_idx = 0;
      for (; path_idx < tail_path->len; ++path_idx) {
        if (tail_path->path[path_idx].row == current_boxes[tail_idx].row &&
            tail_path->path[path_idx].col == current_boxes[tail_idx].col) {
          break;
        }
      }
      if (path_idx >= tail_path->len) {
        return planner_v3_bfs_split_chain_and_finish(
            rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
            current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
      }
      
      last_dir.row = 0;
      last_dir.col = 0;
      continue;
    }
    
    // 在判断是否拐弯之前，先检查链尾箱子是否需要脱离
    // 如果链尾箱子需要脱离，则链尾箱子的新推动方向不作为链式箱子拐弯的方向
    int tail_needs_detach = 0;
    if (chain->count > 1) {
      size_t tail_chain_idx = chain->count - 1;
      Point tail_current_pos = current_boxes[tail_idx];
      Point overlap_end_pos = chain->overlap_end_positions[tail_chain_idx];
      // 检测当前坐标和重叠路径的末端坐标是否相同
      if (tail_current_pos.row >= 0 && tail_current_pos.col >= 0 &&
          overlap_end_pos.row >= 0 && overlap_end_pos.col >= 0 &&
          tail_current_pos.row == overlap_end_pos.row &&
          tail_current_pos.col == overlap_end_pos.col) {
        // 链尾箱子已经到达overlap的末端位置，需要脱离
        tail_needs_detach = 1;
      }
    }
    
    // 如果链尾箱子需要脱离，先处理脱离，不进行拐弯判断
    // 使用planner_v3_bfs_chain_try_detach_any_box_by_overlap来检测并脱离
    if (tail_needs_detach) {
      int detached = 0;
      // 不需要回溯（传递rollback_needed=0），因为这是在推动前检测
      (void)planner_v3_bfs_chain_try_detach_any_box_by_overlap(
          rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
          current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps,
          &detached, 0, NULL, NULL, NULL, 0);
      if (detached) {
        // 脱离成功，重新同步链尾状态
        if (chain->count == 0) return 0;
        if (planner_v3_bfs_chain_sync_tail_path_state(
                rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
                current_boxes, box_count, &tail_idx, &tail_path, &path_idx) != 0) {
          return planner_v3_bfs_split_chain_and_finish(
              rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
              current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
        }
        // 脱离后更新last_dir，避免误判拐弯
        if (chain->count > 0) {
          last_dir = chain->dir;
        }
        continue;  // 继续下一轮循环
      }
    }
    
    // 判断是否拐弯
    // 如果链尾箱子需要脱离，则不使用链尾箱子的新推动方向来触发拐弯
    int turning = (last_dir.row != 0 || last_dir.col != 0) &&
                  (last_dir.row != dir.row || last_dir.col != dir.col);
    
    if (turning) {
      // ===== 拐弯处理 =====
      // 拐弯点就是链尾当前位置
      Point bend_point = tail_curr;
      
      // 使用链式拐弯推动
      int turn_res = planner_v3_bfs_chain_push_turn(
          rows, cols, chain, last_dir, dir, bend_point,
          obstacles, obstacle_count, current_car, current_boxes, box_count,
          box_targets, targets, path_buffer, path_capacity, out_steps);
      
      if (turn_res != 0) {
        if (turn_res == -7) return -7;
        // 拐弯失败，解散链
        return planner_v3_bfs_split_chain_and_finish(
            rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
            current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
      }
      
      // 拐弯结束，更新链尾和方向
      if (chain->count == 0) return 0;
      
      // 检查链中是否还有未完成的箱子
      int has_unfinished = 0;
      for (size_t c = 0; c < chain->count; ++c) {
        size_t box_idx = chain->indices[c];
        Point box_pos = current_boxes[box_idx];
        if (box_pos.row >= 0 && box_pos.col >= 0) {
          if (box_targets[box_idx] != SIZE_MAX) {
            Point target = targets[box_targets[box_idx]];
            if (box_pos.row != target.row || box_pos.col != target.col) {
              has_unfinished = 1;
              break;
            }
          }
        }
      }
      if (!has_unfinished) {
        // 所有箱子都已完成，标记为完成并返回
        for (size_t c = 0; c < chain->count; ++c) {
          size_t box_idx = chain->indices[c];
          current_boxes[box_idx].row = -1;
          current_boxes[box_idx].col = -1;
        }
        return 0;
      }
      
      // 更新链尾和路径索引
      tail_idx = chain->indices[chain->count - 1];
      tail_path = &g_planner_planned_paths[tail_idx];
      
      // 重新查找路径索引
      path_idx = 0;
      for (; path_idx < tail_path->len; ++path_idx) {
        if (tail_path->path[path_idx].row == current_boxes[tail_idx].row &&
            tail_path->path[path_idx].col == current_boxes[tail_idx].col) {
          break;
        }
      }
      if (path_idx >= tail_path->len) {
        // 如果找不到路径索引，检查是否所有箱子都已完成
        int all_finished = 1;
        for (size_t c = 0; c < chain->count; ++c) {
          size_t box_idx = chain->indices[c];
          Point box_pos = current_boxes[box_idx];
          if (box_pos.row >= 0 && box_pos.col >= 0) {
            if (box_targets[box_idx] != SIZE_MAX) {
              Point target = targets[box_targets[box_idx]];
              if (box_pos.row != target.row || box_pos.col != target.col) {
                all_finished = 0;
                break;
              }
            }
          }
        }
        if (all_finished) {
          // 所有箱子都已完成，标记为完成并返回
          for (size_t c = 0; c < chain->count; ++c) {
            size_t box_idx = chain->indices[c];
            current_boxes[box_idx].row = -1;
            current_boxes[box_idx].col = -1;
          }
          return 0;
        }
        return planner_v3_bfs_split_chain_and_finish(
            rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
            current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
      }
      
      last_dir = chain->dir;
      
    } else {
      // ===== 直行处理 =====
      // 所有链中箱子同时向dir方向移动一格
      chain->dir = dir;
      
      int push_res = planner_v3_bfs_chain_push_straight(
          rows, cols, chain, dir, obstacles, obstacle_count,
          current_car, current_boxes, box_count, box_targets, targets,
          path_buffer, path_capacity, out_steps);
      
      if (push_res != 0) {
        if (push_res == -7) return -7;
        // 直行失败，解散链
        return planner_v3_bfs_split_chain_and_finish(
            rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
            current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
      }
      
      // 直行后：同步链尾路径索引（链尾可能变化/路径可能失效）
      if (chain->count == 0) return 0;
      if (planner_v3_bfs_chain_sync_tail_path_state(
              rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
              current_boxes, box_count, &tail_idx, &tail_path, &path_idx) != 0) {
        return planner_v3_bfs_split_chain_and_finish(
            rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
            current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
      }

      // 检测链中任意箱子是否需要脱链
      int detached_any = 0;
      while (chain->count > 0) {
        int detached = 0;
        // 不需要回溯（传递rollback_needed=0），因为这是在推动后检测
        (void)planner_v3_bfs_chain_try_detach_any_box_by_overlap(
            rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
            current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps,
            &detached, 0, NULL, NULL, NULL, 0);
        if (!detached) break;
        detached_any = 1;
        if (chain->count == 0) return 0;
        if (planner_v3_bfs_chain_sync_tail_path_state(
                rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
                current_boxes, box_count, &tail_idx, &tail_path, &path_idx) != 0) {
          return planner_v3_bfs_split_chain_and_finish(
              rows, cols, chain, box_targets, targets, obstacles, obstacle_count,
              current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
        }
      }

      // 如果发生过脱链，重置 last_dir，避免把旧方向带入新链尾的拐弯判断
      if (detached_any) {
        last_dir.row = 0;
        last_dir.col = 0;
      } else {
        last_dir = dir;
      }
    }
  }
  
  // 检查链中是否所有箱子都已完成
  if (chain->count > 0) {
    int all_finished = 1;
    for (size_t c = 0; c < chain->count; ++c) {
      size_t box_idx = chain->indices[c];
      Point box_pos = current_boxes[box_idx];
      if (box_pos.row >= 0 && box_pos.col >= 0) {
        if (box_targets[box_idx] != SIZE_MAX) {
          Point target = targets[box_targets[box_idx]];
          if (box_pos.row != target.row || box_pos.col != target.col) {
            all_finished = 0;
            break;
          }
        } else {
          all_finished = 0;
          break;
        }
      }
    }
    if (all_finished) {
      // 所有箱子都已完成，标记为完成
      for (size_t c = 0; c < chain->count; ++c) {
        size_t box_idx = chain->indices[c];
        current_boxes[box_idx].row = -1;
        current_boxes[box_idx].col = -1;
      }
      return 0;
    }
    
    // 链尾到达目标，标记完成
    tail_idx = chain->indices[chain->count - 1];
    Point tail_pos = current_boxes[tail_idx];
    if (box_targets[tail_idx] != SIZE_MAX) {
      Point target = targets[box_targets[tail_idx]];
      if (tail_pos.row == target.row && tail_pos.col == target.col) {
        current_boxes[tail_idx].row = -1;
        current_boxes[tail_idx].col = -1;
        chain->count--;
      }
    }
  }
  
  return 0;
}

static int planner_v3_bfs_push_primary_with_chain(
    int rows, int cols, size_t primary_idx, const PlannerBoxPathInfo *planned_paths,
    const PlannerBoxOverlap *overlaps, size_t *box_targets, const Point *targets,
    const Point *obstacles, size_t obstacle_count, Point *current_car, Point *current_boxes,
    size_t box_count, Point *path_buffer, size_t path_capacity, size_t *out_steps,
    uint8_t *primary_chain_reassigned_once) {
  // 静态变量：保存每个主箱上一次链式箱子的索引集合
  static size_t last_chain_indices[PLANNER_V3_BFS_MAX_BOXES][PLANNER_V3_BFS_MAX_BOXES];
  static size_t last_chain_counts[PLANNER_V3_BFS_MAX_BOXES] = {0};
  
  while (1) {
    // 每轮动态确认是否仍有存活的副箱子；链条结束后自动退化为单箱推送
    int has_secondary = 0;
    for (size_t s = 0; s < box_count; ++s) {
      if (!overlaps[s].valid || overlaps[s].primary != primary_idx) {
        continue;
      }
      Point sec_pos = current_boxes[s];
      if (sec_pos.row < 0 || sec_pos.col < 0) {
        continue;  // 副箱已完成，忽略
      }
      has_secondary = 1;
      break;
    }
    if (!has_secondary) {
      return planner_v3_bfs_push_single_box_scored(
          rows, cols, primary_idx, box_targets[primary_idx], targets, obstacles, obstacle_count,
          current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
    }

    uint8_t deadlock_ignore_mask[PLANNER_V3_BFS_MAX_BOXES] = {0};
    for (size_t s = 0; s < box_count; ++s) {
      if (overlaps[s].valid && overlaps[s].primary == primary_idx) {
        // 仅忽略仍然存在的副箱子
        if (current_boxes[s].row >= 0 && current_boxes[s].col >= 0) {
          deadlock_ignore_mask[s] = 1;
        }
      }
    }

    // 检查主箱是否已经完成
    Point primary_pos = current_boxes[primary_idx];
    if (primary_pos.row >= 0 && primary_pos.col >= 0 && box_targets[primary_idx] != SIZE_MAX) {
      Point target = targets[box_targets[primary_idx]];
      if (primary_pos.row == target.row && primary_pos.col == target.col) {
        // 主箱已经完成
        current_boxes[primary_idx].row = -1;
        current_boxes[primary_idx].col = -1;
        return 0;
      }
    }
    
    if (box_targets[primary_idx] == SIZE_MAX) {
      last_err_stage = 3;  // 第二次规划失败
      last_err_detail = 2; // 主箱目标未分配
      return -6;
    }
    PlannerBoxPathInfo path_local;
    uint8_t ignore_mask[PLANNER_V3_BFS_MAX_BOXES] = {0};
    for (size_t s = 0; s < box_count; ++s) {
      if (overlaps[s].valid && overlaps[s].primary == primary_idx) {
        if (current_boxes[s].row >= 0 && current_boxes[s].col >= 0) {
          ignore_mask[s] = 1;
        }
      }
    }
    if (!planner_v3_bfs_compute_box_path_with_mask(
            rows, cols, current_boxes[primary_idx], targets[box_targets[primary_idx]], obstacles,
            obstacle_count, current_boxes, box_count, ignore_mask, primary_idx, &path_local)) {
      // 如果计算路径失败，检查主箱是否已经完成
      if (primary_pos.row >= 0 && primary_pos.col >= 0 && box_targets[primary_idx] != SIZE_MAX) {
        Point target = targets[box_targets[primary_idx]];
        if (primary_pos.row == target.row && primary_pos.col == target.col) {
          current_boxes[primary_idx].row = -1;
          current_boxes[primary_idx].col = -1;
          return 0;
        }
      }
      last_err_stage = 3;  // 第二次规划失败
      last_err_detail = 3; // 主箱路径计算失败
      return -6;
    }
    g_planner_planned_paths[primary_idx] = path_local;

    if (!path_local.valid) {
      // 如果路径无效，检查主箱是否已经完成
      if (primary_pos.row >= 0 && primary_pos.col >= 0 && box_targets[primary_idx] != SIZE_MAX) {
        Point target = targets[box_targets[primary_idx]];
        if (primary_pos.row == target.row && primary_pos.col == target.col) {
          current_boxes[primary_idx].row = -1;
          current_boxes[primary_idx].col = -1;
          return 0;
        }
      }
      last_err_stage = 3;  // 第二次规划失败
      last_err_detail = 4; // 主箱路径无效
      return -6;
    }
    if (path_local.len < 2) {
      current_boxes[primary_idx].row = -1;
      current_boxes[primary_idx].col = -1;
      return 0;
    }

    for (size_t step = 1; step < path_local.len; ++step) {
      Point curr = current_boxes[primary_idx];
      Point next = path_local.path[step];
      Point dir = {next.row - curr.row, next.col - curr.col};
      Point push_pos = {curr.row - dir.row, curr.col - dir.col};
      int retried = 0;

    retry_push_primary:
      if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_pos.row, push_pos.col) ||
          planner_v3_bfs_is_box_at(current_boxes, box_count, push_pos.row, push_pos.col,
                                   primary_idx)) {
        if (retried) {
          last_err_stage = 5;  // 链式推动内部失败
          last_err_detail = 11; // 重试后推位仍非法
          return -6;
        }
        Point detour_path[256];
        size_t detour_len = 0;
        Point new_push_pos;
        int detour_ok = planner_v3_bfs_follow_box_with_global_astar(
            rows, cols, *current_car, curr, next, obstacles, obstacle_count, current_boxes,
            box_count, primary_idx, detour_path, 256, &detour_len, &new_push_pos);
        if (!detour_ok || detour_len == 0) {
          last_err_stage = 5;  // 链式推动内部失败
          last_err_detail = 12; // 无法找到绕行路径
          return -6;
        }
        for (size_t i = 1; i < detour_len; ++i) {
          if (*out_steps >= path_capacity) {
            return -7;
          }
          if (*out_steps > 0 &&
              !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], detour_path[i])) {
            last_err_stage = 4;  // 车辆路径连续性失败
            last_err_detail = 1; // 绕行路径不连续
            return -6;
          }
          path_buffer[(*out_steps)++] = detour_path[i];
        }
        *current_car = detour_path[detour_len - 1];
        push_pos = *current_car;
        retried = 1;
        goto retry_push_primary;
      }

      int car_result = planner_v3_bfs_car_move_with_global_astar(
          rows, cols, current_car, push_pos, obstacles, obstacle_count, current_boxes, box_count,
          path_buffer, path_capacity, out_steps);
      if (car_result == -7) {
        return -7;
      }
      if (car_result == 0) {
        if (retried) {
          last_err_stage = 5;  // 链式推动内部失败
          last_err_detail = 11; // 重试后推位仍非法
          return -6;
        }
        Point detour_path[256];
        size_t detour_len = 0;
        Point new_push_pos;
        int detour_ok = planner_v3_bfs_follow_box_with_global_astar(
            rows, cols, *current_car, curr, next, obstacles, obstacle_count, current_boxes,
            box_count, primary_idx, detour_path, 256, &detour_len, &new_push_pos);
        if (!detour_ok || detour_len == 0) {
          last_err_stage = 5;  // 链式推动内部失败
          last_err_detail = 12; // 无法找到绕行路径
          return -6;
        }
        for (size_t i = 1; i < detour_len; ++i) {
          if (*out_steps >= path_capacity) {
            return -7;
          }
          if (*out_steps > 0 &&
              !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], detour_path[i])) {
            last_err_stage = 4;  // 车辆路径连续性失败
            last_err_detail = 1; // 绕行路径不连续
            return -6;
          }
          path_buffer[(*out_steps)++] = detour_path[i];
        }
        *current_car = detour_path[detour_len - 1];
        push_pos = *current_car;
        retried = 1;
        goto retry_push_primary;
      }

      if (current_car->row != push_pos.row || current_car->col != push_pos.col) {
        last_err_stage = 5;  // 链式推动内部失败
        last_err_detail = 15; // 车未到达推位
        return -6;
      }

      // 检测是否有副箱子在推动方向上
      size_t secondary = SIZE_MAX;
      for (size_t s = 0; s < box_count; ++s) {
        if (!overlaps[s].valid || overlaps[s].primary != primary_idx) {
          continue;
        }
        Point sec_pos = current_boxes[s];
        if (sec_pos.row == next.row && sec_pos.col == next.col &&
            overlaps[s].dir.row == dir.row && overlaps[s].dir.col == dir.col) {
          secondary = s;
          break;
        }
      }

      if (secondary != SIZE_MAX) {
        // 【新增】构建完整的链（可能包含多个副箱子）
        PlannerChainLocal chain;
        chain.count = 0;
        chain.dir = dir;
        
        // 添加主箱
        chain.indices[chain.count] = primary_idx;
        chain.overlap_lens[chain.count] = 0;  // 主箱没有重叠长度限制
        chain.overlap_end_positions[chain.count].row = -1;
        chain.overlap_end_positions[chain.count].col = -1;
        chain.steps_walked[chain.count] = 0;
        chain.count++;
        
        // 沿着推动方向构建链
        size_t current_in_chain = primary_idx;
        Point expected_pos = next;
        
        while (chain.count < PLANNER_V3_BFS_MAX_BOXES) {
          size_t found = SIZE_MAX;
          size_t found_overlap_len = 0;
          
          // 在expected_pos位置查找副箱子
          for (size_t s = 0; s < box_count; ++s) {
            // 检查是否已在链中
            int already_in = 0;
            for (size_t c = 0; c < chain.count; ++c) {
              if (chain.indices[c] == s) { already_in = 1; break; }
            }
            if (already_in) continue;
            
            Point box_pos = current_boxes[s];
            if (box_pos.row < 0 || box_pos.col < 0) continue;
            
            if (box_pos.row == expected_pos.row && box_pos.col == expected_pos.col) {
              // 检查重叠信息是否匹配（使用第一次路径规划的overlap_len判断是否构成链式）
              // 但存储第二次路径规划的overlap_len_second用于检测何时路径不再重叠
              if (overlaps[s].valid && overlaps[s].primary == current_in_chain &&
                  overlaps[s].dir.row == dir.row && overlaps[s].dir.col == dir.col) {
                found = s;
                found_overlap_len = overlaps[s].overlap_len_second;
                break;
              }
              // 即使没有明确的重叠信息，位置匹配也加入链（重叠长度设为1）
              found = s;
              found_overlap_len = 1;
              break;
            }
          }
          
          if (found == SIZE_MAX) break;
          
          chain.indices[chain.count] = found;
          chain.overlap_lens[chain.count] = found_overlap_len;
          // 保存重叠路径的末端坐标
          if (overlaps && overlaps[found].valid && overlaps[found].overlap_end_pos.row >= 0 && overlaps[found].overlap_end_pos.col >= 0) {
            chain.overlap_end_positions[chain.count] = overlaps[found].overlap_end_pos;
          } else {
            chain.overlap_end_positions[chain.count].row = -1;
            chain.overlap_end_positions[chain.count].col = -1;
          }
          chain.steps_walked[chain.count] = 0;
          chain.count++;
          current_in_chain = found;
          expected_pos.row += dir.row;
          expected_pos.col += dir.col;
        }

        // 【规则更新】链式内目标重分配规则：
        // 1. 如果链式内有新箱子加入，则需要进行一次目标重分配，重新规划路径等后续过程
        // 2. 如果有箱子脱离链式，则不需要进行目标重分配
        if (chain.count > 1) {
          // 检测是否有新箱子加入链式
          int has_new_box = 0;
          if (primary_idx < PLANNER_V3_BFS_MAX_BOXES) {
            // 如果上一次链式为空（第一次构建链），视为有新箱子加入
            if (last_chain_counts[primary_idx] == 0) {
              has_new_box = 1;
            } else {
              // 检查当前链中的每个箱子是否在上一次的链中
              for (size_t c = 0; c < chain.count; ++c) {
                size_t box_idx = chain.indices[c];
                int found_in_last = 0;
                for (size_t l = 0; l < last_chain_counts[primary_idx]; ++l) {
                  if (last_chain_indices[primary_idx][l] == box_idx) {
                    found_in_last = 1;
                    break;
                  }
                }
                if (!found_in_last) {
                  // 发现新箱子加入
                  has_new_box = 1;
                  break;
                }
              }
            }
          } else {
            // 如果主箱索引超出范围，第一次构建链时也视为有新箱子（链式首次形成）
            has_new_box = 1;
          }
          
          // 如果有新箱子加入，进行目标重分配
          if (has_new_box) {
            int changed = planner_v3_bfs_chain_reassign_targets_tail_to_head(
                rows, cols, &chain, current_boxes, box_count, box_targets, targets,
                obstacles, obstacle_count);
            if (changed) {
              // 目标发生变化：按原需求回到第一次路径规划（忽略箱子阻挡）阶段重算
              // 更新上一次链式状态
              if (primary_idx < PLANNER_V3_BFS_MAX_BOXES) {
                last_chain_counts[primary_idx] = chain.count;
                for (size_t c = 0; c < chain.count; ++c) {
                  last_chain_indices[primary_idx][c] = chain.indices[c];
                }
              }
              return PLANNER_V3_BFS_REPLAN;
            }
            // 即使目标没有变化，也更新上一次链式状态（因为链式已经变化）
            if (primary_idx < PLANNER_V3_BFS_MAX_BOXES) {
              last_chain_counts[primary_idx] = chain.count;
              for (size_t c = 0; c < chain.count; ++c) {
                last_chain_indices[primary_idx][c] = chain.indices[c];
              }
            }
          } else {
            // 没有新箱子加入（可能有箱子脱离），更新链式状态但不进行重分配
            if (primary_idx < PLANNER_V3_BFS_MAX_BOXES) {
              last_chain_counts[primary_idx] = chain.count;
              for (size_t c = 0; c < chain.count; ++c) {
                last_chain_indices[primary_idx][c] = chain.indices[c];
              }
            }
          }
        } else {
          // 链式只有1个箱子（只有主箱），清空上一次链式状态
          if (primary_idx < PLANNER_V3_BFS_MAX_BOXES) {
            last_chain_counts[primary_idx] = 0;
          }
        }
        
        // 为链中所有箱子计算路径
        for (size_t c = 1; c < chain.count; ++c) {
          size_t box_idx = chain.indices[c];
          if (box_targets[box_idx] == SIZE_MAX) continue;
          
          PlannerBoxPathInfo box_path;
          uint8_t box_ignore[PLANNER_V3_BFS_MAX_BOXES] = {0};
          // 忽略链中排在前面的箱子
          for (size_t p = 0; p < c; ++p) {
            box_ignore[chain.indices[p]] = 1;
          }
          // 忽略当前副箱作为主箱时的所有层级副箱
          planner_v3_bfs_collect_all_secondaries_recursive(
              box_idx, overlaps, box_count, box_ignore);
          if (!planner_v3_bfs_compute_box_path_with_mask(
                  rows, cols, current_boxes[box_idx], targets[box_targets[box_idx]], obstacles,
                  obstacle_count, current_boxes, box_count, box_ignore, box_idx, &box_path)) {
            // 计算失败，尝试解散链
            return planner_v3_bfs_split_chain_and_finish(
                rows, cols, &chain, box_targets, targets, obstacles, obstacle_count,
                current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
          }
          g_planner_planned_paths[box_idx] = box_path;
        }

        // 调用新的多副箱子链式推动函数
        int chain_res = planner_v3_bfs_execute_multi_chain(
            rows, cols, &chain, planned_paths, overlaps, box_targets, targets,
            obstacles, obstacle_count, current_car, current_boxes, box_count, path_buffer,
            path_capacity, out_steps);
        if (chain_res != 0) {
          return chain_res;
        }

        // 链式推箱成功，直接返回外层，由外层下一轮重新分配/重新选 candidate
        return 0;
      }

      if (!planner_v3_bfs_in_bounds(rows, cols, next.row, next.col) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, next.row, next.col)) {
        last_err_stage = 5;  // 链式推动内部失败
        last_err_detail = 31; // 主箱下一步位置非法
        return -6;
      }
      for (size_t bi = 0; bi < box_count; ++bi) {
        if (bi == primary_idx) {
          continue;
        }
        Point b = current_boxes[bi];
        if (b.row == next.row && b.col == next.col) {
          last_err_stage = 5;  // 链式推动内部失败
          last_err_detail = 32; // 主箱下一步位置被占
          return -6;
        }
      }

      if (box_targets[primary_idx] == SIZE_MAX) {
        last_err_stage = 3;  // 第二次规划失败
        last_err_detail = 2; // 主箱目标未分配
        return -6;
      }
      Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
      for (size_t bi = 0; bi < box_count; ++bi) {
        temp_boxes[bi] = current_boxes[bi];
      }
      temp_boxes[primary_idx] = next;
      if (!planner_v3_bfs_can_reach_goal_mask(
              rows, cols, obstacles, obstacle_count, temp_boxes, box_count, primary_idx,
              deadlock_ignore_mask, next, targets[box_targets[primary_idx]])) {
        last_err_stage = 3;  // 第二次规划失败
        last_err_detail = 10; // 主箱无法到达目标
        return -6;
      }

      if (*out_steps >= path_capacity) {
        return -7;
      }

      Point old_pos = curr;
      current_boxes[primary_idx] = next;
      *current_car = old_pos;
      path_buffer[(*out_steps)++] = *current_car;
    }

    current_boxes[primary_idx].row = -1;
    current_boxes[primary_idx].col = -1;
    return 0;
  }
}

// 验证最终状态：检查所有箱子是否都到达目标点
// 返回值：1=所有箱子都已完成，0=有箱子未完成
static int planner_v3_bfs_verify_final_state(
    const Point *boxes, size_t box_count,
    const size_t *box_targets, const Point *targets) {
  if (!boxes || !targets || !box_targets) {
    return 0;
  }
  
  for (size_t i = 0; i < box_count; ++i) {
    Point box_pos = boxes[i];
    
    // 如果箱子位置无效（已标记为完成），跳过
    if (box_pos.row < 0 || box_pos.col < 0) {
      continue;
    }
    
    // 如果箱子没有分配目标，视为未完成
    if (box_targets[i] == SIZE_MAX) {
      return 0;
    }
    
    // 检查箱子是否到达目标点
    Point target = targets[box_targets[i]];
    if (box_pos.row != target.row || box_pos.col != target.col) {
      return 0;  // 有箱子未到达目标
    }
  }
  
  return 1;  // 所有箱子都已完成
}

static void planner_v3_bfs_sync_out_box_target_indices(
    size_t *out_box_target_indices, size_t box_count, const size_t *box_targets) {
  if (!out_box_target_indices || !box_targets) {
    return;
  }
  for (size_t i = 0; i < box_count; ++i) {
    out_box_target_indices[i] = box_targets[i];
  }
}

// 在返回非0值前检查最终状态，如果所有箱子都已完成则返回0
static int planner_v3_bfs_check_and_return(
    int result, const Point *current_boxes, size_t box_count,
    const size_t *box_targets, const Point *targets,
    size_t *out_box_target_indices) {
  // 如果返回值为0，直接返回
  if (result == 0) {
    planner_v3_bfs_sync_out_box_target_indices(out_box_target_indices, box_count, box_targets);
    return result;
  }
  
  // 如果返回非0值，检查所有箱子是否都已完成
  if (planner_v3_bfs_verify_final_state(current_boxes, box_count, box_targets, targets)) {
    planner_v3_bfs_sync_out_box_target_indices(out_box_target_indices, box_count, box_targets);
    return 0;  // 所有箱子都已完成，返回0
  }
  
  planner_v3_bfs_sync_out_box_target_indices(out_box_target_indices, box_count, box_targets);
  return result;  // 有箱子未完成，返回原错误码
}

// 辅助函数：检查链是否已经存在于链式信息中（无序比较，只要箱子集合相同即认为是同一链）
static int planner_v3_bfs_chain_already_exists(const PlannerChainInfo *chain_info,
                                                const size_t *new_chain_indices,
                                                size_t new_chain_len) {
  if (!chain_info || !new_chain_indices || new_chain_len == 0) {
    return 0;
  }
  
  for (size_t ci = 0; ci < chain_info->chain_count; ++ci) {
    size_t existing_len = chain_info->chain_lengths[ci];
    
    // 长度不同，肯定不是同一链
    if (existing_len != new_chain_len) {
      continue;
    }
    
    // 检查箱子集合是否相同（无序比较）
    int all_match = 1;
    for (size_t i = 0; i < new_chain_len; ++i) {
      int found = 0;
      for (size_t j = 0; j < existing_len; ++j) {
        if (new_chain_indices[i] == chain_info->chain_indices[ci][j]) {
          found = 1;
          break;
        }
      }
      if (!found) {
        all_match = 0;
        break;
      }
    }
    
    if (all_match) {
      return 1;  // 找到相同的链
    }
  }
  
  return 0;  // 没有找到相同的链
}

// 辅助函数：从 overlaps 数组构建完整的链式信息（累积添加，不覆盖已有记录）
static void planner_v3_bfs_accumulate_chain_info(const PlannerBoxOverlap *overlaps, size_t box_count,
                                                  const uint8_t *box_done, PlannerChainInfo *out_chain_info) {
  if (!out_chain_info || !overlaps) {
    return;
  }
  
  // 注意：不清空 chain_count，而是累积添加
  
  // 标记哪些箱子已经被记录在当前轮次的某个链中
  uint8_t box_recorded[PLANNER_V3_BFS_MAX_BOXES] = {0};
  
  // 遍历所有箱子，找出所有主箱（非副箱）
  for (size_t i = 0; i < box_count; ++i) {
    // 不再跳过已完成的箱子，因为链可能在箱子完成前就形成了
    if (box_recorded[i]) {
      continue;  // 跳过已记录的箱子
    }
    
    // 检查箱子 i 是否是主箱（不是任何箱子的副箱）
    int is_primary = 1;
    if (overlaps[i].valid && overlaps[i].primary != SIZE_MAX) {
      is_primary = 0;  // 箱子 i 是某个链的副箱
    }
    
    if (!is_primary) {
      continue;  // 跳过副箱，只从主箱开始构建链
    }
    
    // 检查箱子 i 是否有副箱（形成链式）
    int has_secondary = 0;
    for (size_t j = 0; j < box_count; ++j) {
      if (j != i && overlaps[j].valid && overlaps[j].primary == i) {
        has_secondary = 1;
        break;
      }
    }
    
    if (!has_secondary) {
      continue;  // 箱子 i 是独立箱子，不形成链式
    }
    
    // 构建以箱子 i 为主箱的链式
    size_t temp_chain[PLANNER_V3_BFS_MAX_CHAIN_LEN];
    size_t chain_len = 0;
    
    // 添加主箱
    temp_chain[chain_len++] = i;
    box_recorded[i] = 1;
    
    // 递归添加所有副箱（构建完整链）
    size_t current_primary = i;
    while (chain_len < PLANNER_V3_BFS_MAX_CHAIN_LEN) {
      // 查找 current_primary 的直接副箱
      int found_secondary = 0;
      for (size_t j = 0; j < box_count; ++j) {
        if (box_recorded[j]) {
          continue;
        }
        if (overlaps[j].valid && overlaps[j].primary == current_primary) {
          temp_chain[chain_len++] = j;
          box_recorded[j] = 1;
          current_primary = j;  // 将副箱作为新的主箱，继续查找
          found_secondary = 1;
          break;
        }
      }
      if (!found_secondary) {
        break;  // 链的末尾
      }
    }
    
    // 检查这个链是否已经存在
    if (!planner_v3_bfs_chain_already_exists(out_chain_info, temp_chain, chain_len)) {
      // 新链，添加到输出结构中
      if (out_chain_info->chain_count < PLANNER_V3_BFS_MAX_CHAIN_COUNT) {
        size_t chain_idx = out_chain_info->chain_count;
        for (size_t k = 0; k < chain_len; ++k) {
          out_chain_info->chain_indices[chain_idx][k] = temp_chain[k];
          // 收集重叠末端坐标（主箱没有重叠末端坐标，设为无效）
          if (k == 0) {
            // 主箱没有重叠末端坐标
            out_chain_info->chain_overlap_end_pos[chain_idx][k].row = -1;
            out_chain_info->chain_overlap_end_pos[chain_idx][k].col = -1;
          } else {
            // 副箱从overlaps中获取重叠末端坐标
            // 注意：这个坐标与脱离检测时使用的坐标一致，都来自同一个overlaps数组
            // 脱离检测时，planner_v3_bfs_build_chain_from_primary也会从overlaps[found].overlap_end_pos
            // 复制到chain->overlap_end_positions，所以显示的值应该与实际脱离时使用的值一致
            size_t box_idx = temp_chain[k];
            if (overlaps[box_idx].valid && overlaps[box_idx].overlap_end_pos.row >= 0 && overlaps[box_idx].overlap_end_pos.col >= 0) {
              out_chain_info->chain_overlap_end_pos[chain_idx][k] = overlaps[box_idx].overlap_end_pos;
            } else {
              out_chain_info->chain_overlap_end_pos[chain_idx][k].row = -1;
              out_chain_info->chain_overlap_end_pos[chain_idx][k].col = -1;
            }
          }
        }
        out_chain_info->chain_lengths[chain_idx] = chain_len;
        out_chain_info->chain_count++;
      }
    }
  }
}

// 辅助函数：递归收集所有层级的副箱（包括副箱的副箱...）
static void planner_v3_bfs_collect_all_secondaries_recursive(
    size_t primary_idx, const PlannerBoxOverlap *overlaps, size_t box_count,
    uint8_t *ignore_mask) {
  if (!overlaps || !ignore_mask) return;
  
  // 遍历所有箱子，找出当前主箱的所有直接副箱
  for (size_t s = 0; s < box_count; ++s) {
    if (overlaps[s].valid && overlaps[s].primary == primary_idx) {
      // 标记这个副箱为忽略
      ignore_mask[s] = 1;
      // 递归处理这个副箱的所有副箱
      planner_v3_bfs_collect_all_secondaries_recursive(
          s, overlaps, box_count, ignore_mask);
    }
  }
}

// 辅助函数：将 PlannerBoxPathInfo 转换为 PlannerBoxPathOutput
static void planner_v3_bfs_copy_path_info_to_output(const PlannerBoxPathInfo *src,
                                                     PlannerBoxPathOutput *dst) {
  if (!src || !dst) return;
  dst->valid = src->valid;
  dst->path_len = src->len;
  if (src->len > 0 && src->len <= PLANNER_V3_BFS_MAX_PATH_LEN) {
    for (size_t i = 0; i < src->len; ++i) {
      dst->path[i] = src->path[i];
    }
  }
}

// 【新实现】动态推箱函数：每推完一个箱子后重新选择下一个最优箱子和目标
// 支持先忽略箱子阻挡寻找重叠路径，再以多箱联动方式规划
static int planner_v3_bfs_run_dynamic(int rows, int cols, Point car,
                                   const Point *boxes, size_t box_count,
                                   const Point *targets, size_t target_count,
                                   const Point *obstacles, size_t obstacle_count,
                                   Point *path_buffer, size_t path_capacity,
                                   size_t *out_steps, size_t *out_box_target_indices,
                                   PlannerChainInfo *out_chain_info,
                                   PlannerAllBoxPaths *out_first_paths,
                                   PlannerAllBoxPaths *out_final_paths,
                                   PlannerBoxOverlap *out_overlaps) {
  if (!boxes || !targets || !path_buffer || !out_steps) {
    return -1;
  }
  if (box_count == 0 || target_count == 0) {
    return -2;
  }
  if (box_count > PLANNER_V3_BFS_MAX_BOXES) {
    return -3;
  }
  if (path_capacity == 0) {
    return -4;
  }
  if (target_count < box_count) {
    return -8;
  }

  *out_steps = 0;
  if (out_box_target_indices) {
    for (size_t i = 0; i < box_count; ++i) {
      out_box_target_indices[i] = SIZE_MAX;
    }
  }

  Point current_car = car;
  Point current_boxes[PLANNER_V3_BFS_MAX_BOXES];
  size_t box_targets[PLANNER_V3_BFS_MAX_BOXES];
  uint8_t box_done[PLANNER_V3_BFS_MAX_BOXES] = {0};
  uint8_t target_taken[PLANNER_V3_BFS_MAX_CELLS] = {0};

  for (size_t i = 0; i < box_count; ++i) {
    current_boxes[i] = boxes[i];
    box_targets[i] = SIZE_MAX;
  }

  if (out_box_target_indices) {
    for (size_t i = 0; i < box_count; ++i) {
      out_box_target_indices[i] = SIZE_MAX;
    }
  }

  size_t remaining = 0;
  for (size_t i = 0; i < box_count; ++i) {
    if (current_boxes[i].row < 0 || current_boxes[i].col < 0) {
      box_done[i] = 1;
      continue;
    }
    // 箱子必须推送到分配的目标点才算完成
    remaining++;
  }

  // 【初始分配】按“车到箱 BFS 真实距离”排序，再让优先箱子从空闲目标中选最近目标
  // 初始目标会被后续动态阶段优先沿用；若后续检测到不可达/冲突仍会进入动态再分配分支
  {
    int init_ret = planner_v3_bfs_initial_assign_targets(
        rows, cols, current_car, current_boxes, box_count, targets, target_count,
        obstacles, obstacle_count, box_targets);
    if (init_ret != 0) {
      return planner_v3_bfs_check_and_return(init_ret, current_boxes, box_count, box_targets, targets,
                                             out_box_target_indices);
    }
    if (out_box_target_indices) {
      for (size_t i = 0; i < box_count; ++i) {
        out_box_target_indices[i] = box_targets[i];
      }
    }
  }

  // 链式重分配限制：每个主箱所在的链式“目标重分配”最多执行一次
  uint8_t chain_reassigned_once[PLANNER_V3_BFS_MAX_BOXES] = {0};

  int replan_guard = 0;
  while (remaining > 0) {
    PlannerBoxPathInfo *free_paths = g_planner_free_paths;
    PlannerBoxPathInfo *planned_paths = g_planner_planned_paths;
    PlannerBoxOverlap *overlaps = g_planner_overlaps;

    for (size_t i = 0; i < PLANNER_V3_BFS_MAX_BOXES; ++i) {
      free_paths[i].valid = 0;
      free_paths[i].len = 0;
      planned_paths[i].valid = 0;
      planned_paths[i].len = 0;
      overlaps[i].valid = 0;
      overlaps[i].primary = SIZE_MAX;
      overlaps[i].primary_start = 0;
      overlaps[i].secondary_start = 0;
      overlaps[i].overlap_len = 0;
      overlaps[i].dir.row = 0;
      overlaps[i].dir.col = 0;
    }

    // 更新已完成箱子与占用的目标
    remaining = 0;
    for (size_t i = 0; i < box_count; ++i) {
      if (box_done[i]) {
        continue;
      }
      if (current_boxes[i].row < 0 || current_boxes[i].col < 0) {
        box_done[i] = 1;
        if (box_targets[i] != SIZE_MAX && box_targets[i] < target_count) {
          target_taken[box_targets[i]] = 1;
          if (out_box_target_indices) {
            out_box_target_indices[i] = box_targets[i];
          }
        }
        continue;
      }
      // 箱子必须推送到分配的目标点才算完成
      remaining++;
    }
    if (remaining == 0) {
      break;
    }

    // 当前轮次中已经被占用/预占用的目标
    uint8_t target_claimed[PLANNER_V3_BFS_MAX_CELLS] = {0};
    for (size_t t = 0; t < target_count; ++t) {
      if (target_taken[t]) {
        target_claimed[t] = 1;
      }
    }

    // 为每个未完成的箱子“选择”选择可行目标并计算无阻挡路径
    for (size_t i = 0; i < box_count; ++i) {
      if (box_done[i]) {
        continue;
      }

      PlannerBoxPathInfo candidate_path;
      candidate_path.valid = 0;
      candidate_path.len = 0;
      size_t chosen_target = SIZE_MAX;

      // 优先尝试保留已有目标（若仍可用且可达）
      if (box_targets[i] != SIZE_MAX && box_targets[i] < target_count &&
          !target_claimed[box_targets[i]]) {
        uint8_t ignore_all[PLANNER_V3_BFS_MAX_BOXES];
        for (size_t j = 0; j < box_count; ++j) {
          ignore_all[j] = 1;
        }
        if (planner_v3_bfs_compute_box_path_with_mask(
                rows, cols, current_boxes[i], targets[box_targets[i]], obstacles, obstacle_count,
                current_boxes, box_count, ignore_all, i, &candidate_path)) {
          chosen_target = box_targets[i];
        }
      }

      // 否则在未被占用/预占用的目标中寻找可达目标
      if (chosen_target == SIZE_MAX) {
        PlannerBoxPathInfo best_path;
        best_path.valid = 0;
        best_path.len = 0;
        size_t best_target = SIZE_MAX;
        int best_steps = INT_MAX;

        for (size_t t = 0; t < target_count; ++t) {
          if (target_claimed[t]) {
            continue;
          }
          uint8_t ignore_all[PLANNER_V3_BFS_MAX_BOXES];
          for (size_t j = 0; j < box_count; ++j) {
            ignore_all[j] = 1;
          }
          PlannerBoxPathInfo tmp_path;
          if (planner_v3_bfs_compute_box_path_with_mask(
                  rows, cols, current_boxes[i], targets[t], obstacles, obstacle_count, current_boxes,
                  box_count, ignore_all, i, &tmp_path)) {
            int steps = (tmp_path.len > 0) ? (int)(tmp_path.len - 1) : INT_MAX;
            if (steps < best_steps || (steps == best_steps && t < best_target)) {
              best_steps = steps;
              best_target = t;
              best_path = tmp_path;
            }
          }
        }
        if (best_target != SIZE_MAX) {
          chosen_target = best_target;
          candidate_path = best_path;
        }
      }

      if (chosen_target == SIZE_MAX) {
        // 没有任何可达的空闲目标
        last_err_stage = 2;  // 动态选目标失败
        last_err_detail = 1; // 找不到可达的空闲目标
        return planner_v3_bfs_check_and_return(-6, current_boxes, box_count, box_targets, targets,
                                               out_box_target_indices);
      }

      box_targets[i] = chosen_target;
      target_claimed[chosen_target] = 1;
      free_paths[i] = candidate_path;
      if (out_box_target_indices) {
        out_box_target_indices[i] = chosen_target;
      }
    }

    // 输出第一次路径规划（忽略箱子阻挡）的所有箱子路径
    // 保留已完成的箱子的路径信息，只更新未完成的箱子
    if (out_first_paths) {
      out_first_paths->box_count = box_count;
      for (size_t i = 0; i < box_count; ++i) {
        // 如果箱子已完成且之前已经有有效的路径信息，则保留它
        if (box_done[i] && out_first_paths->box_paths[i].valid && out_first_paths->box_paths[i].path_len > 0) {
          // 保留已完成的箱子的路径信息
          continue;
        }
        // 否则更新路径信息（包括未完成的箱子和之前没有路径信息的已完成箱子）
        planner_v3_bfs_copy_path_info_to_output(&free_paths[i], &out_first_paths->box_paths[i]);
      }
    }

    // 先检测第一次路径规划的重叠（用于构建忽略掩码）
    planner_v3_bfs_detect_overlaps(rows, cols, current_car, free_paths, NULL, current_boxes, box_count, obstacles, obstacle_count, box_targets, targets, overlaps);
    
    // 然后计算第二次路径规划（考虑箱子阻挡），使用第一次检测的overlaps来构建忽略掩码
    // 这样重叠路径末端坐标使用的路径与final_paths相同
    for (size_t i = 0; i < box_count; ++i) {
      if (box_done[i]) {
        continue;
      }
      if (box_targets[i] == SIZE_MAX) {
        continue;
      }
      // 检查箱子是否已在目标位置
      Point tgt = targets[box_targets[i]];
      if (current_boxes[i].row == tgt.row && current_boxes[i].col == tgt.col) {
        // 箱子已在目标位置，标记为完成
        current_boxes[i].row = -1;
        current_boxes[i].col = -1;
        box_done[i] = 1;
        continue;
      }
      uint8_t ignore_mask[PLANNER_V3_BFS_MAX_BOXES] = {0};
      ignore_mask[i] = 1;
      // 忽略已完成的箱子（负坐标）
      for (size_t j = 0; j < box_count; ++j) {
        if (current_boxes[j].row < 0 || current_boxes[j].col < 0) {
          ignore_mask[j] = 1;
        }
      }
      // 作为主箱，递归忽略自己对应的所有层级的副箱（包括副箱的副箱...）
      // 作为副箱，忽略自己的主箱，避免互相占位导致死角误判
      // 使用第一次检测的overlaps来构建忽略掩码
      planner_v3_bfs_collect_all_secondaries_recursive(i, overlaps, box_count, ignore_mask);
      if (overlaps[i].valid && overlaps[i].primary != SIZE_MAX) {
        ignore_mask[overlaps[i].primary] = 1;
      }
      
      if (!planner_v3_bfs_compute_box_path_with_mask(
              rows, cols, current_boxes[i], targets[box_targets[i]], obstacles, obstacle_count,
              current_boxes, box_count, ignore_mask, i, &planned_paths[i])) {
        // 路径计算失败，检查箱子是否已在目标位置
        if (current_boxes[i].row == tgt.row && current_boxes[i].col == tgt.col) {
          current_boxes[i].row = -1;
          current_boxes[i].col = -1;
          box_done[i] = 1;
          continue;
        }
        // 路径计算失败，检查所有箱子是否都已完成
        return planner_v3_bfs_check_and_return(-6, current_boxes, box_count, box_targets, targets,
                                               out_box_target_indices);
      }
    }

    // 使用计算好的planned_paths来重新检测重叠（计算重叠路径末端坐标，与final_paths相同）
    planner_v3_bfs_detect_overlaps(rows, cols, current_car, free_paths, planned_paths, current_boxes, box_count, obstacles, obstacle_count, box_targets, targets, overlaps);
    
    // 输出重叠信息（最近一次检测的结果）
    if (out_overlaps) {
      for (size_t i = 0; i < box_count; ++i) {
        out_overlaps[i] = overlaps[i];
      }
    }
    
    // 累积收集链式信息（记录所有出现过的链）
    if (out_chain_info) {
      planner_v3_bfs_accumulate_chain_info(overlaps, box_count, box_done, out_chain_info);
    }

    // planned_paths已经在检测重叠之前计算好了，这里不需要重新计算

    // 输出最终路径规划（考虑箱子阻挡）的所有箱子路径
    // 保留已完成的箱子的路径信息，只更新未完成的箱子
    if (out_final_paths) {
      out_final_paths->box_count = box_count;
      for (size_t i = 0; i < box_count; ++i) {
        // 如果箱子已完成且之前已经有有效的路径信息，则保留它
        if (box_done[i] && out_final_paths->box_paths[i].valid && out_final_paths->box_paths[i].path_len > 0) {
          // 保留已完成的箱子的路径信息
          continue;
        }
        // 否则更新路径信息（包括未完成的箱子和之前没有路径信息的已完成箱子）
        planner_v3_bfs_copy_path_info_to_output(&planned_paths[i], &out_final_paths->box_paths[i]);
      }
    }

    // 记录在当前轮次中已经尝试过但失败的箱子（返回-6）
    uint8_t failed_boxes[PLANNER_V3_BFS_MAX_BOXES] = {0};
    int all_boxes_tried = 0;
    
    while (!all_boxes_tried) {
      size_t candidate = SIZE_MAX;
      int best_score = INT_MAX;
      // 链式阻挡惩罚：若某条潜在链两箱之间夹着其它箱子，则对该链内所有箱子加分（惩罚）
      uint8_t chain_penalize[PLANNER_V3_BFS_MAX_BOXES];
      planner_v3_bfs_mark_obstructed_chain_boxes(current_boxes, box_count, overlaps, chain_penalize);

      for (size_t i = 0; i < box_count; ++i) {
        if (box_done[i]) {
          continue;
        }
        // 跳过已经尝试过但失败的箱子
        if (failed_boxes[i]) {
          continue;
        }
        if (!planned_paths[i].valid || planned_paths[i].len == 0) {
          // 路径无效，检查所有箱子是否都已完成
          return planner_v3_bfs_check_and_return(-6, current_boxes, box_count, box_targets, targets,
                                                 out_box_target_indices);
        }
        if (planned_paths[i].len < 2) {
          current_boxes[i].row = -1;
          current_boxes[i].col = -1;
          box_done[i] = 1;
          if (remaining > 0) {
            remaining--;
          }
          continue;
        }

        int score;
        if (overlaps[i].valid && overlaps[i].primary != SIZE_MAX) {
          score = INT_MAX / 4;
        } else {
          // 检查箱子是否有副箱
          int has_secondary = 0;
          for (size_t j = 0; j < box_count; ++j) {
            if (j != i && overlaps[j].valid && overlaps[j].primary == i) {
              has_secondary = 1;
              break;
            }
          }
          
          // 构建临时箱子数组，忽略已完成的箱子（负坐标）
          Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
          for (size_t j = 0; j < box_count; ++j) {
            if (current_boxes[j].row < 0 || current_boxes[j].col < 0) {
              temp_boxes[j].row = -1;
              temp_boxes[j].col = -1;
            } else {
              temp_boxes[j] = current_boxes[j];
            }
          }
          
          // 如果箱子不为副箱且没有副箱，选取四个推箱位中距离车最近的推位作为评分项
          if (!has_secondary) {
            const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
            int best_dist = INT_MAX;
            // 获取箱子的目标点
            Point target = {0, 0};
            if (box_targets[i] != SIZE_MAX && box_targets[i] < target_count) {
              target = targets[box_targets[i]];
            } else {
              // 如果目标未分配，跳过这个箱子（不应该发生，但为了安全）
              score = INT_MAX / 2;
            }
            
            if (box_targets[i] != SIZE_MAX && box_targets[i] < target_count) {
              for (int d = 0; d < 4; ++d) {
                // 推箱位：车站在 box 的反方向
                Point push_pos = {current_boxes[i].row - dirs[d][0], current_boxes[i].col - dirs[d][1]};
                // 箱子被推后的位置（下一步）
                Point next_box_pos = {current_boxes[i].row + dirs[d][0], current_boxes[i].col + dirs[d][1]};
                
                // 检查推箱位是否在边界内
                if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col)) {
                  continue;
                }
                // 检查推箱位是否是障碍
                if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_pos.row, push_pos.col)) {
                  continue;
                }
                // 检查推箱位是否有箱子
                if (planner_v3_bfs_is_box_at(temp_boxes, box_count, push_pos.row, push_pos.col, SIZE_MAX)) {
                  continue;
                }
                
                // 检查下一步（箱子被推后的位置）是否在边界内
                if (!planner_v3_bfs_in_bounds(rows, cols, next_box_pos.row, next_box_pos.col)) {
                  continue;
                }
                // 检查下一步是否是障碍
                if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, next_box_pos.row, next_box_pos.col)) {
                  continue;
                }
                // 检查下一步是否有箱子
                if (planner_v3_bfs_is_box_at(temp_boxes, box_count, next_box_pos.row, next_box_pos.col, i)) {
                  continue;
                }
                
                // 检查下一步是否是死角（箱子从下一步能否到达目标）
                // 特殊处理：如果推到目标点，跳过死点检测
                if (next_box_pos.row != target.row || next_box_pos.col != target.col) {
                  // 构建临时箱子数组，假设箱子已经在next_box_pos位置
                  Point test_boxes[PLANNER_V3_BFS_MAX_BOXES];
                  for (size_t j = 0; j < box_count; ++j) {
                    if (j == i) {
                      test_boxes[j] = next_box_pos;  // 箱子i移动到下一步位置
                    } else {
                      test_boxes[j] = temp_boxes[j];
                    }
                  }
                  if (!planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count,
                                                     test_boxes, box_count, i, next_box_pos, target)) {
                    continue;  // 死角，跳过这个方向
                  }
                }
                
                // 所有检查通过，计算距离
                int dist_map[PLANNER_V3_BFS_MAX_CELLS];
                if (!planner_v3_bfs_global_bfs_from_target(rows, cols, push_pos, obstacles, obstacle_count,
                                                          temp_boxes, box_count, SIZE_MAX, dist_map)) {
                  continue;
                }
                int car_idx = current_car.row * cols + current_car.col;
                if (car_idx >= 0 && car_idx < rows * cols && dist_map[car_idx] != INT_MAX) {
                  if (dist_map[car_idx] < best_dist) {
                    best_dist = dist_map[car_idx];
                  }
                }
              }
              score = (best_dist != INT_MAX) ? best_dist : INT_MAX / 2;
            }
          } else {
            // 有副箱的情况，使用原来的逻辑（第一个方向的推箱位）
            Point first_dir = planned_paths[i].dir[0];
            Point push_pos = {current_boxes[i].row - first_dir.row,
                              current_boxes[i].col - first_dir.col};
            int dist_map[PLANNER_V3_BFS_MAX_CELLS];
            if (planner_v3_bfs_global_bfs_from_target(rows, cols, push_pos, obstacles, obstacle_count,
                                                      temp_boxes, box_count, SIZE_MAX, dist_map)) {
              int car_idx = current_car.row * cols + current_car.col;
              if (car_idx >= 0 && car_idx < rows * cols && dist_map[car_idx] != INT_MAX) {
                score = dist_map[car_idx];
              } else {
                score = INT_MAX / 2;
              }
            } else {
              score = INT_MAX / 2;
            }
          }
        }

        // 如果该箱子属于"潜在链式但中间夹箱阻挡"的链，则提高评分（惩罚），
        // 使算法优先处理阻挡箱，从而更早形成链式推箱。
        if (chain_penalize[i]) {
          if (score >= INT_MAX - PLANNER_V3_BFS_CHAIN_BLOCK_PENALTY) {
            score = INT_MAX;
          } else {
            score += PLANNER_V3_BFS_CHAIN_BLOCK_PENALTY;
          }
        }

        if (score < best_score) {
          best_score = score;
          candidate = i;
        }
      }

      if (candidate == SIZE_MAX) {
        // 无法找到候选箱子，检查所有箱子是否都已完成
        return planner_v3_bfs_check_and_return(-6, current_boxes, box_count, box_targets, targets,
                                               out_box_target_indices);
      }

      // 保存推动前的状态，用于判断是否需要重新分配目标
      // 判断candidate是否是单箱（无overlap关系）、链头箱子（有副箱的主箱）或副箱
      int is_single_box = 0;  // 单箱：无overlap关系
      int is_chain_head = 0;   // 链头箱子：是主箱且有副箱
      int is_secondary = 0;    // 副箱：有主箱
      
      // 保存推动前的目标分配，用于推动后判断是否完成
      size_t saved_target = box_targets[candidate];
      
      if (overlaps[candidate].valid && overlaps[candidate].primary != SIZE_MAX) {
        is_secondary = 1;  // candidate是副箱
      } else {
        // candidate不是副箱，检查是否是单箱或链头箱子
        int has_secondary_before = 0;
        // 检查推动前是否有副箱
        for (size_t s = 0; s < box_count; ++s) {
          if (s != candidate && overlaps[s].valid && overlaps[s].primary == candidate) {
            // 检查副箱是否还存在（推动前）
            Point sec_pos_before = current_boxes[s];
            if (sec_pos_before.row >= 0 && sec_pos_before.col >= 0) {
              has_secondary_before = 1;
              break;
            }
          }
        }
        if (has_secondary_before) {
          is_chain_head = 1;  // 链头箱子：有副箱的主箱
        } else {
          is_single_box = 1;  // 单箱：无overlap关系
        }
      }

      int push_res = planner_v3_bfs_push_primary_with_chain(
          rows, cols, candidate, planned_paths, overlaps, box_targets, targets, obstacles,
          obstacle_count, &current_car, current_boxes, box_count, path_buffer, path_capacity,
          out_steps, &chain_reassigned_once[candidate]);
      if (push_res == PLANNER_V3_BFS_REPLAN) {
        // 链式推箱触发"整链目标重分配"，需要回到第一次路径规划阶段重算
        replan_guard++;
        if (replan_guard > 20) {
          return planner_v3_bfs_check_and_return(-6, current_boxes, box_count, box_targets, targets,
                                                 out_box_target_indices);
        }
        // 重新规划时，重置失败记录，因为重新规划后之前的失败记录可能不再有效
        for (size_t i = 0; i < box_count; ++i) {
          failed_boxes[i] = 0;
        }
        all_boxes_tried = 0;
        break;  // 跳出内层循环，继续外层循环（重新规划）
      }
      if (push_res == -6) {
        // 首选箱子无法推动（返回-6），标记为失败并尝试其他箱子
        failed_boxes[candidate] = 1;
        // 检查是否所有箱子都已尝试过
        all_boxes_tried = 1;
        for (size_t i = 0; i < box_count; ++i) {
          if (!box_done[i] && !failed_boxes[i] && 
              planned_paths[i].valid && planned_paths[i].len >= 2) {
            all_boxes_tried = 0;
            break;
          }
        }
        if (all_boxes_tried) {
          // 所有箱子都尝试过了且都返回-6，检查所有箱子是否都已完成
          return planner_v3_bfs_check_and_return(-6, current_boxes, box_count, box_targets, targets,
                                                 out_box_target_indices);
        }
        // 继续尝试其他箱子
        continue;
      }
      if (push_res != 0) {
        // 推动失败（非-6错误），检查所有箱子是否都已完成
        return planner_v3_bfs_check_and_return(push_res, current_boxes, box_count, box_targets, targets,
                                               out_box_target_indices);
      }
      
      // 推动成功，检查是否需要重新分配目标
      // 如果推完的是单箱（无overlap关系）或链头箱子（有副箱的主箱），且箱子尚未完成，
      // 则清除其目标分配，强制外层循环重新分配目标
      if (is_single_box || is_chain_head) {
        // 检查箱子是否已完成（到达目标或已标记为完成）
        // 使用推动前保存的目标来判断，因为推动后可能已经清除了目标分配
        int box_finished = 0;
        if (current_boxes[candidate].row < 0 || current_boxes[candidate].col < 0) {
          box_finished = 1;  // 箱子已标记为完成
        } else if (saved_target != SIZE_MAX && saved_target < target_count) {
          Point target = targets[saved_target];
          if (current_boxes[candidate].row == target.row && 
              current_boxes[candidate].col == target.col) {
            box_finished = 1;  // 箱子已到达目标
          }
        }
        
        // 如果箱子未完成，且是单箱或链头箱子，清除目标分配以强制重新分配
        if (!box_finished) {
          // 推完单箱（无overlap关系）或链头箱子，清除目标分配
          box_targets[candidate] = SIZE_MAX;
          if (out_box_target_indices) {
            out_box_target_indices[candidate] = SIZE_MAX;
          }
        }
      }
      
      // 推动成功，跳出内层循环
      // 注意：外层循环会在每次循环时重新分配目标（5089行开始），
      // 推完单箱（无overlap关系）或链头箱子后，已清除目标分配，外层循环会重新分配目标；
      // 推完副箱子后，如果已有目标仍可用，则保留原有目标
      break;
    }
    replan_guard = 0;

    remaining = 0;
    for (size_t i = 0; i < box_count; ++i) {
      if (current_boxes[i].row < 0 || current_boxes[i].col < 0) {
        box_done[i] = 1;
        if (box_targets[i] != SIZE_MAX && box_targets[i] < target_count) {
          target_taken[box_targets[i]] = 1;
          if (out_box_target_indices) {
            out_box_target_indices[i] = box_targets[i];
          }
        }
        continue;
      }
      // 箱子必须推送到分配的目标点才算完成
      box_done[i] = 0;
      remaining++;
    }
  }

  if (*out_steps > 1 && !planner_v3_bfs_validate_continuous_path(path_buffer, *out_steps)) {
    // 路径验证失败，检查所有箱子是否都已完成
    int all_finished = 1;
    for (size_t i = 0; i < box_count; ++i) {
      if (box_targets[i] != SIZE_MAX) {
        Point box_pos = current_boxes[i];
        if (box_pos.row >= 0 && box_pos.col >= 0) {
          Point target = targets[box_targets[i]];
          if (box_pos.row != target.row || box_pos.col != target.col) {
            all_finished = 0;
            break;
          }
        }
      } else {
        // 没有目标但位置有效，视为未完成
        Point box_pos = current_boxes[i];
        if (box_pos.row >= 0 && box_pos.col >= 0) {
          all_finished = 0;
          break;
        }
      }
    }
    if (all_finished) {
      planner_v3_bfs_sync_out_box_target_indices(out_box_target_indices, box_count, box_targets);
      // 所有箱子已完成，让车子返回到起点
      Point empty_boxes[PLANNER_V3_BFS_MAX_BOXES];  // 所有箱子已完成，使用空数组
      for (size_t i = 0; i < box_count; ++i) {
        empty_boxes[i].row = -1;
        empty_boxes[i].col = -1;
      }
      int return_result = planner_v3_bfs_car_move_with_global_astar(
          rows, cols, &current_car, car, obstacles, obstacle_count,
          empty_boxes, box_count, path_buffer, path_capacity, out_steps);
      if (return_result == -7) {
        return -7;  // 路径缓存不足
      }
      // 即使返回起点失败，也返回0（因为所有箱子已完成）
      return 0;
    }
    planner_v3_bfs_sync_out_box_target_indices(out_box_target_indices, box_count, box_targets);
    last_err_stage = 2;  // 动态选目标失败
    last_err_detail = 2; // 所有箱子未完成但无法继续
    return -6;
  }

  planner_v3_bfs_sync_out_box_target_indices(out_box_target_indices, box_count, box_targets);
  // 所有箱子已完成，让车子返回到起点
  Point empty_boxes[PLANNER_V3_BFS_MAX_BOXES];  // 所有箱子已完成，使用空数组
  for (size_t i = 0; i < box_count; ++i) {
    empty_boxes[i].row = -1;
    empty_boxes[i].col = -1;
  }
  int return_result = planner_v3_bfs_car_move_with_global_astar(
      rows, cols, &current_car, car, obstacles, obstacle_count,
      empty_boxes, box_count, path_buffer, path_capacity, out_steps);
  if (return_result == -7) {
    return -7;  // 路径缓存不足
  }
  // 即使返回起点失败，也返回0（因为所有箱子已完成）
  return 0;
}

int plan_boxes_greedy_v3_bfs(int rows, int cols, PlannerPointV3_BFS car,
                         const PlannerPointV3_BFS *boxes, size_t box_count,
                         const PlannerPointV3_BFS *targets, size_t target_count,
                         const PlannerPointV3_BFS *obstacles,
                         size_t obstacle_count, PlannerPointV3_BFS *path_buffer,
                         size_t path_capacity, size_t *out_steps,
                         size_t *out_box_target_indices,
                         PlannerChainInfo *out_chain_info,
                         PlannerAllBoxPaths *out_first_paths,
                         PlannerAllBoxPaths *out_final_paths,
                         PlannerBoxOverlap *out_overlaps) {
  if (!boxes || !targets || !path_buffer || !out_steps) {
    return -1;
  }
  *out_steps = 0;
  if (out_chain_info) {
    out_chain_info->chain_count = 0;
    // 初始化重叠末端坐标数组
    for (size_t i = 0; i < PLANNER_V3_BFS_MAX_CHAIN_COUNT; ++i) {
      for (size_t j = 0; j < PLANNER_V3_BFS_MAX_CHAIN_LEN; ++j) {
        out_chain_info->chain_overlap_end_pos[i][j].row = -1;
        out_chain_info->chain_overlap_end_pos[i][j].col = -1;
      }
    }
  }
  if (box_count == 0 || target_count == 0) {
    return -2;
  }
  if (box_count > PLANNER_V3_BFS_MAX_BOXES) {
    return -3;
  }
  if (path_capacity == 0) {
    return -4;
  }

  // 【核心修改】直接调用新的动态选择函数
  // 注意：最终状态验证已在planner_v3_bfs_run_dynamic内部的关键返回点完成
  return planner_v3_bfs_run_dynamic(rows, cols, car, boxes, box_count,
                                    targets, target_count, obstacles, obstacle_count,
                                    path_buffer, path_capacity, out_steps,
                                    out_box_target_indices, out_chain_info,
                                    out_first_paths, out_final_paths, out_overlaps);
}
