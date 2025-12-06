#include "assigned_box_planner_greedy_2.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

#define PLANNER_V3_BFS_MAX_BOXES 10
#define PLANNER_V3_BFS_MAX_CELLS 400
#define PLANNER_V3_BFS_MAX_GREEDY_STEPS 5000
#define PLANNER_V3_BFS_MAX_PATH_LEN 5000

typedef struct {
  Point box;
  Point target;
  size_t target_idx;
  size_t source_idx;
} PlannerBoxGoalBFS;

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
static int planner_v3_bfs_can_reach_goal(int rows, int cols, const Point *obstacles,
                                size_t obstacle_count, const Point *boxes,
                                size_t box_count, size_t moving_idx,
                                Point start, Point target) {
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
// 返回值：1=成功，0=失败
static int planner_v3_bfs_global_bfs_from_target(int rows, int cols, Point target,
                                             const Point *obstacles, size_t obstacle_count,
                                             const Point *boxes, size_t box_count,
                                             int dist[PLANNER_V3_BFS_MAX_CELLS]) {
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
// 返回值：1=成功找到路径，0=失败
static int planner_v3_bfs_astar_with_dist(int rows, int cols, Point start, Point target,
                                      const Point *obstacles, size_t obstacle_count,
                                      const Point *boxes, size_t box_count,
                                      const int dist[PLANNER_V3_BFS_MAX_CELLS],
                                      Point *path, size_t path_cap, size_t *path_len) {
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
  open_count++;
  in_open[start_idx] = 1;
  
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  
  while (open_count > 0) {
    // 找到f_score最小的节点
    int best_idx = 0;
    for (int i = 1; i < open_count; ++i) {
      if (open_set[i].f_score < open_set[best_idx].f_score) {
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
          open_count++;
          in_open[next_idx] = 1;
        }
      } else if (tentative_g < g_score[next_idx]) {
        // 找到更好的路径
        g_score[next_idx] = tentative_g;
        parent[next_idx] = curr_idx;
        
        int h = dist[next_idx];
        int f = tentative_g + h;
        
        // 更新open_set中的f_score
        for (int i = 0; i < open_count; ++i) {
          if (open_set[i].idx == next_idx) {
            open_set[i].f_score = f;
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
                                           obstacle_count, boxes, box_count, dist)) {
      continue;  // BFS失败，尝试下一个推箱位
    }
    
    // 第2步：使用A*搜索路径
    if (!planner_v3_bfs_astar_with_dist(rows, cols, car, target_push_pos,
                                    obstacles, obstacle_count,
                                    boxes, box_count, dist,
                                    path, path_cap, path_len)) {
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
  if (car_pos->row == target.row && car_pos->col == target.col) {
    return 1;  // 已在目标位置
  }
  
  // 第1步：从目标做全局BFS
  int dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                         boxes, box_count, dist)) {
    return 0;  // BFS失败
  }
  
  // 第2步：使用A*搜索路径
  Point temp_path[256];
  size_t temp_len = 0;
  
  if (!planner_v3_bfs_astar_with_dist(rows, cols, *car_pos, target,
                                  obstacles, obstacle_count,
                                  boxes, box_count, dist,
                                  temp_path, 256, &temp_len)) {
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

// 为箱子分配目标点（贪心策略）
static int planner_v3_bfs_assign_targets(int rows, int cols, Point car,
                                     const Point *boxes, size_t box_count,
                                     const Point *targets, size_t target_count,
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
  uint8_t box_used[PLANNER_V3_BFS_MAX_BOXES] = {0};

  size_t assigned = 0;
  while (assigned < usable) {
    size_t best_box = SIZE_MAX;
    size_t best_target = SIZE_MAX;
    int best_score = INT_MAX;
    int best_box_goal = INT_MAX;

    for (size_t bi = 0; bi < usable; ++bi) {
      if (box_used[bi]) {
        continue;
      }
      int car_to_box = planner_v3_bfs_manhattan(car, valid_boxes[bi]);
      for (size_t ti = 0; ti < target_count; ++ti) {
        if (target_used[ti]) {
          continue;
        }
        int box_to_goal = planner_v3_bfs_manhattan(valid_boxes[bi], targets[ti]);
        int score = car_to_box + box_to_goal;
        if (score < best_score ||
            (score == best_score && box_to_goal < best_box_goal) ||
            (score == best_score && box_to_goal == best_box_goal &&
             ti < best_target)) {
          best_score = score;
          best_box_goal = box_to_goal;
          best_box = bi;
          best_target = ti;
        }
      }
    }

    if (best_box == SIZE_MAX || best_target == SIZE_MAX) {
      return -6;
    }

    out_goals[assigned].box = valid_boxes[best_box];
    out_goals[assigned].target = targets[best_target];
    out_goals[assigned].target_idx = best_target;
    out_goals[assigned].source_idx = source_indices[best_box];

    if (out_mapping) {
      out_mapping[source_indices[best_box]] = best_target;
    }

    box_used[best_box] = 1;
    target_used[best_target] = 1;
    assigned++;
  }

  *out_goal_count = assigned;
  return 0;
}

// 主推箱规划函数（纯BFS+A*版本）
static int planner_v3_bfs_run_assigned(int rows, int cols, Point car,
                                   const PlannerBoxGoalBFS *goals,
                                   size_t goal_count, const Point *obstacles,
                                   size_t obstacle_count, Point *path_buffer,
                                   size_t path_capacity, size_t *out_steps) {
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
  Point current_boxes[PLANNER_V3_BFS_MAX_BOXES];

  for (size_t i = 0; i < goal_count; ++i) {
    current_boxes[i] = goals[i].box;
  }

  int picked[PLANNER_V3_BFS_MAX_BOXES] = {0};

  // 按贪心顺序处理每个箱子
  for (size_t order_idx = 0; order_idx < goal_count; ++order_idx) {
    size_t box_idx = SIZE_MAX;
    int best_score = INT_MAX;
    for (size_t i = 0; i < goal_count; ++i) {
      if (picked[i]) {
        continue;
      }
      int dist_car = planner_v3_bfs_manhattan(current_car, current_boxes[i]);
      int dist_goal = planner_v3_bfs_manhattan(current_boxes[i], goals[i].target);
      int score = dist_car + dist_goal;
      if (score < best_score) {
        best_score = score;
        box_idx = i;
      }
    }
    if (box_idx == SIZE_MAX) {
      return -6;
    }
    picked[box_idx] = 1;
    
    Point box = current_boxes[box_idx];
    Point target = goals[box_idx].target;

    // 箱子已在目标点，标记为"消失"（后续路径规划中自动忽略）
    if (box.row == target.row && box.col == target.col) {
      current_boxes[box_idx].row = -1;
      current_boxes[box_idx].col = -1;
      continue;
    }

    // 【核心修改】始终使用全局BFS+A*策略
    // 从目标点做全局BFS，计算每个格子到目标的真实距离
    int target_dist[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                           current_boxes, goal_count, target_dist)) {
      return -6;  // BFS失败
    }

    // 推箱子到目标
    int step_count = 0;
    int last_dr = 0;
    int last_dc = 0;
    int reverse_count = 0;  // 连续反向移动次数

    while (box.row != target.row || box.col != target.col) {
      if (step_count++ >= PLANNER_V3_BFS_MAX_GREEDY_STEPS) {
        return -5;
      }

      // 贪心选择推箱方向
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

        if (planner_v3_bfs_is_box_at(current_boxes, goal_count, new_box_row, new_box_col,
                      box_idx) ||
            planner_v3_bfs_is_box_at(current_boxes, goal_count, push_row, push_col,
                      box_idx)) {
          continue;
        }

        // 特殊处理：如果推到目标点，跳过死点检测
        if (new_box_row == target.row && new_box_col == target.col) {
          // 推到目标点，不需要死点检测
        } else {
          // 非目标点，进行严格的死点检测
          if (!planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count, 
                                         current_boxes, goal_count, box_idx,
                                         (Point){new_box_row, new_box_col}, target)) {
            continue;
          }
        }

        Point push_from = {push_row, push_col};
        
        // 【核心修改】使用全局BFS计算的真实距离 - 箱子到目标
        int dist_after;
        int idx = new_box_row * cols + new_box_col;
        if (idx >= 0 && idx < rows * cols && target_dist[idx] != INT_MAX) {
          dist_after = target_dist[idx];
        } else {
          // 该位置无法到达目标，使用一个很大的值
          dist_after = INT_MAX / 100;
        }
        
        int adj_pen = planner_v3_bfs_adjacent_blockers(
            rows, cols, obstacles, obstacle_count, current_boxes, goal_count,
            box_idx, new_box_row, new_box_col);

        // 反向移动惩罚：5的n次方，n为连续反向次数
        int reverse_pen = 0;
        if (last_dr == -dirs[i][0] && last_dc == -dirs[i][1]) {
          // 计算5的(reverse_count+1)次方
          int power = 5;
          for (int p = 0; p < reverse_count; ++p) {
            power *= 5;
          }
          reverse_pen = power;
        }
        
        // 【核心修改】使用BFS真实距离 - 车到推箱位
        int car_to_push;
        int car_to_push_dist[PLANNER_V3_BFS_MAX_CELLS];
        if (planner_v3_bfs_global_bfs_from_target(rows, cols, push_from, obstacles, 
                                                   obstacle_count, current_boxes, 
                                                   goal_count, car_to_push_dist)) {
          int car_idx = current_car.row * cols + current_car.col;
          if (car_idx >= 0 && car_idx < rows * cols && car_to_push_dist[car_idx] != INT_MAX) {
            car_to_push = car_to_push_dist[car_idx];
          } else {
            // 车无法到达推箱位，使用极大值
            car_to_push = INT_MAX / 100;
          }
        } else {
          // BFS失败，回退到曼哈顿距离估算
          car_to_push = planner_v3_bfs_manhattan(current_car, push_from);
        }

        int score = dist_after * 10 + adj_pen * 5 + reverse_pen + car_to_push * 2;
        
        candidates[i].dist = dist_after;
        candidates[i].adj_pen = adj_pen;
        candidates[i].score = score;
        candidates[i].feasible = 1;
        candidates[i].push_from = push_from;
        candidates[i].box_next.row = new_box_row;
        candidates[i].box_next.col = new_box_col;
      }

      // 按评分排序所有可行方向（从优到劣）
      int sorted_dirs[4];
      int sorted_count = 0;
      
      for (int i = 0; i < 4; ++i) {
        if (candidates[i].feasible) {
          sorted_dirs[sorted_count++] = i;
        }
      }
      
      // 简单冒泡排序（最多4个方向）
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
        // 【核心修改】无可行方向，使用全局BFS+A*绕箱子
        Point detour_path[256];
        size_t detour_len = 0;
        Point new_push_pos;
        
        int detour_ok = planner_v3_bfs_follow_box_with_global_astar(
            rows, cols, current_car, box, target, obstacles, obstacle_count,
            current_boxes, goal_count, box_idx, detour_path, 256, 
            &detour_len, &new_push_pos);
        
        if (!detour_ok || detour_len == 0) {
          return -6;  // 无法找到推箱位
        }
        
        // 执行绕行路径（跳过起点，因为起点是当前车位置）
        for (size_t i = 1; i < detour_len; ++i) {
          if (*out_steps >= path_capacity) {
            return -7;
          }
          // 验证与前一个点相邻
          if (*out_steps > 0 && 
              !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], detour_path[i])) {
            return -6;  // 绕箱路径不连续
          }
          path_buffer[(*out_steps)++] = detour_path[i];
        }
        
        if (detour_len > 0) {
          current_car = detour_path[detour_len - 1];
        } else {
          current_car = new_push_pos;
        }
        
        // 不移动箱子，继续下一轮尝试推箱
        continue;
      }

      // 尝试所有可行方向，从最优到次优
      int push_success = 0;
      DirCandidate chosen;
      
      for (int try_idx = 0; try_idx < sorted_count; ++try_idx) {
        int dir_idx = sorted_dirs[try_idx];
        DirCandidate candidate = candidates[dir_idx];
        
        // 保存状态，以便失败时回退
        Point car_before = current_car;
        size_t steps_before = *out_steps;
        
        // 特殊情况：车已经在推箱位，无需移动
        if (current_car.row == candidate.push_from.row && 
            current_car.col == candidate.push_from.col) {
          // 车已在推箱位，但需确保路径连续性
          if (*out_steps == 0) {
            // 路径为空，添加车的当前位置作为起点
            if (*out_steps >= path_capacity) {
              return -7;
            }
            path_buffer[(*out_steps)++] = current_car;
          } else {
            Point last = path_buffer[*out_steps - 1];
            if (last.row != current_car.row || last.col != current_car.col) {
              // 最后一个点不是车当前位置，检查是否相邻
              if (!planner_v3_bfs_check_adjacent(last, current_car)) {
                // 不相邻，这个方向不可行
                current_car = car_before;
                *out_steps = steps_before;
                continue;
              }
              // 相邻但不同，添加车的当前位置
              if (*out_steps >= path_capacity) {
                return -7;
              }
              path_buffer[(*out_steps)++] = current_car;
            }
          }
          // 车已在推箱位，直接成功
          chosen = candidate;
          push_success = 1;
          break;
        }
        
        // 【核心修改】使用全局BFS+A*移动车辆
        int car_result = planner_v3_bfs_car_move_with_global_astar(
            rows, cols, &current_car, candidate.push_from, obstacles, obstacle_count,
            current_boxes, goal_count, path_buffer, path_capacity, out_steps);
        
        if (car_result == -7) {
          return -7;  // 路径缓存不足
        }
        
        if (car_result == 0) {
          // 全局BFS+A*失败，回退状态，尝试下一个方向
          current_car = car_before;
          *out_steps = steps_before;
          continue;
        }
        
        // 成功到达推箱位
        chosen = candidate;
        push_success = 1;
        break;
      }
      
      if (!push_success) {
        return -6;  // 所有方向都无法到达推箱位
      }
      
      // 验证车已到达推箱位
      if (current_car.row != chosen.push_from.row || 
          current_car.col != chosen.push_from.col) {
        return -6;  // 内部错误：车未到达推箱位
      }
      
      // 验证推箱位和箱子相邻
      int dist_to_box = planner_v3_bfs_manhattan(current_car, box);
      if (dist_to_box != 1) {
        return -6;  // 内部错误：推箱位和箱子不相邻
      }
      
      // 推箱子：车从 chosen.push_from 推动箱子，车移动到箱子旧位置
      Point box_old_pos = box;
      box = chosen.box_next;
      current_boxes[box_idx] = box;
      
      // 车推完箱子后占据箱子旧位置（这是推箱子的标准规则）
      current_car = box_old_pos;
      if (*out_steps >= path_capacity) {
        return -7;
      }
      // 验证与前一个点相邻
      if (*out_steps > 0 && 
          !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], current_car)) {
        return -6;  // 推箱后车位置不连续
      }
      path_buffer[(*out_steps)++] = current_car;

      // 更新反向移动计数
      if (last_dr == -chosen.dr && last_dc == -chosen.dc && (last_dr != 0 || last_dc != 0)) {
        // 本次移动是反向移动
        reverse_count++;
      } else {
        // 不是反向移动，重置计数器
        reverse_count = 0;
      }
      
      last_dr = chosen.dr;
      last_dc = chosen.dc;
    }
    
    // 箱子到达目标，标记为"消失"（-1坐标表示不再参与路径规划）
    current_boxes[box_idx].row = -1;
    current_boxes[box_idx].col = -1;
  }

  // 验证生成的路径是否连续
  if (*out_steps > 1 && !planner_v3_bfs_validate_continuous_path(path_buffer, *out_steps)) {
    return -6;  // 路径不连续，视为规划失败
  }

  return 0;
}

int plan_boxes_greedy_v3_bfs(int rows, int cols, PlannerPointV3_BFS car,
                         const PlannerPointV3_BFS *boxes, size_t box_count,
                         const PlannerPointV3_BFS *targets, size_t target_count,
                         const PlannerPointV3_BFS *obstacles,
                         size_t obstacle_count, PlannerPointV3_BFS *path_buffer,
                         size_t path_capacity, size_t *out_steps,
                         size_t *out_box_target_indices) {
  if (!boxes || !targets || !path_buffer || !out_steps) {
    return -1;
  }
  *out_steps = 0;
  if (box_count == 0 || target_count == 0) {
    return -2;
  }
  if (box_count > PLANNER_V3_BFS_MAX_BOXES) {
    return -3;
  }
  if (path_capacity == 0) {
    return -4;
  }

  if (out_box_target_indices) {
    for (size_t i = 0; i < box_count; ++i) {
      out_box_target_indices[i] = SIZE_MAX;
    }
  }

  PlannerBoxGoalBFS assigned[PLANNER_V3_BFS_MAX_BOXES];
  size_t assigned_count = 0;
  int assign_res = planner_v3_bfs_assign_targets(
      rows, cols, car, boxes, box_count, targets, target_count, assigned,
      &assigned_count, out_box_target_indices);
  if (assign_res != 0) {
    *out_steps = 0;
    return assign_res;
  }

  return planner_v3_bfs_run_assigned(rows, cols, car, assigned, assigned_count,
                                 obstacles, obstacle_count, path_buffer,
                                 path_capacity, out_steps);
}

