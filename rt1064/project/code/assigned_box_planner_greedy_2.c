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

// 【新实现】动态推箱函数：每推完一个箱子后重新选择下一个最优箱子和目标
static int planner_v3_bfs_run_dynamic(int rows, int cols, Point car,
                                   const Point *boxes, size_t box_count,
                                   const Point *targets, size_t target_count,
                                   const Point *obstacles, size_t obstacle_count,
                                   Point *path_buffer, size_t path_capacity,
                                   size_t *out_steps, size_t *out_box_target_indices) {
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
  
  // 初始化映射为未分配
  if (out_box_target_indices) {
    for (size_t i = 0; i < box_count; ++i) {
      out_box_target_indices[i] = SIZE_MAX;
    }
  }

  Point current_car = car;
  Point current_boxes[PLANNER_V3_BFS_MAX_BOXES];
  
  // 复制箱子位置
  for (size_t i = 0; i < box_count; ++i) {
    current_boxes[i] = boxes[i];
  }

  uint8_t box_done[PLANNER_V3_BFS_MAX_BOXES] = {0};
  uint8_t target_used[PLANNER_V3_BFS_MAX_CELLS] = {0};

  // 逐个处理箱子
  for (size_t round = 0; round < box_count; ++round) {
    // 【优化】每次从当前车位置，动态选择最优的箱子-目标组合
    // 考虑箱子的最佳推箱位，计算车到推箱位的距离
    size_t best_box = SIZE_MAX;
    size_t best_target = SIZE_MAX;
    int best_score = INT_MAX;

    for (size_t bi = 0; bi < box_count; ++bi) {
      if (box_done[bi]) {
        continue;
      }
      // 跳过已消失的箱子
      if (current_boxes[bi].row < 0 || current_boxes[bi].col < 0) {
        continue;
      }

      for (size_t ti = 0; ti < target_count; ++ti) {
        if (target_used[ti]) {
          continue;
        }

        Point box = current_boxes[bi];
        Point target_pos = targets[ti];
        
        // 计算箱子到目标的距离
        int dist_box_to_target = planner_v3_bfs_manhattan(box, target_pos);
        
        // 【关键优化】找到该箱子朝向目标的最佳推箱位
        // 尝试所有4个方向的推箱位，选择最优的
        const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        int best_push_dist = INT_MAX;
        
        for (int d = 0; d < 4; ++d) {
          int push_r = box.row - dirs[d][0];
          int push_c = box.col - dirs[d][1];
          int new_box_r = box.row + dirs[d][0];
          int new_box_c = box.col + dirs[d][1];
          
          // 检查推箱位和新箱子位置是否合法
          if (!planner_v3_bfs_in_bounds(rows, cols, push_r, push_c) ||
              !planner_v3_bfs_in_bounds(rows, cols, new_box_r, new_box_c)) {
            continue;
          }
          
          // 检查是否有障碍物
          if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_r, push_c) ||
              planner_v3_bfs_is_obstacle(obstacles, obstacle_count, new_box_r, new_box_c)) {
            continue;
          }
          
          // 检查是否有其他箱子
          if (planner_v3_bfs_is_box_at(current_boxes, box_count, push_r, push_c, bi) ||
              planner_v3_bfs_is_box_at(current_boxes, box_count, new_box_r, new_box_c, bi)) {
            continue;
          }
          
          // 检查推过去后箱子能否到达目标（死点检测）
          if (new_box_r != target_pos.row || new_box_c != target_pos.col) {
            if (!planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count,
                                               current_boxes, box_count, bi,
                                               (Point){new_box_r, new_box_c}, target_pos)) {
              continue;
            }
          }
          
          // 计算车到该推箱位的距离（使用BFS真实距离）
          Point push_pos = {push_r, push_c};
          int dist_car_to_push;
          
          // 使用全局BFS计算车到推箱位的真实距离
          int push_dist_map[PLANNER_V3_BFS_MAX_CELLS];
          if (planner_v3_bfs_global_bfs_from_target(rows, cols, push_pos, obstacles,
                                                     obstacle_count, current_boxes,
                                                     box_count, push_dist_map)) {
            int car_idx = current_car.row * cols + current_car.col;
            if (car_idx >= 0 && car_idx < rows * cols && push_dist_map[car_idx] != INT_MAX) {
              dist_car_to_push = push_dist_map[car_idx];
            } else {
              // 车无法到达该推箱位
              continue;
            }
          } else {
            // BFS失败，回退到曼哈顿距离估算
            dist_car_to_push = planner_v3_bfs_manhattan(current_car, push_pos);
          }
          
          // 选择车到推箱位距离最小的推箱位
          if (dist_car_to_push < best_push_dist) {
            best_push_dist = dist_car_to_push;
          }
        }
        
        // 如果没有找到任何可行的推箱位，跳过这个组合
        if (best_push_dist == INT_MAX) {
          continue;
        }
        
        // 评分 = 车到最佳推箱位的距离 + 箱子到目标的距离
        int score = best_push_dist + dist_box_to_target;

        if (score < best_score) {
          best_score = score;
          best_box = bi;
          best_target = ti;
        }
      }
    }

    if (best_box == SIZE_MAX || best_target == SIZE_MAX) {
      return -6;  // 无法找到可行的箱子-目标组合
    }

    // 标记该箱子和目标已被使用
    box_done[best_box] = 1;
    target_used[best_target] = 1;
    
    // 记录映射关系
    if (out_box_target_indices) {
      out_box_target_indices[best_box] = best_target;
    }

    Point box = current_boxes[best_box];
    Point target = targets[best_target];

    // 箱子已在目标点，跳过
    if (box.row == target.row && box.col == target.col) {
      current_boxes[best_box].row = -1;
      current_boxes[best_box].col = -1;
      continue;
    }

    // 【推箱逻辑】从这里开始与原来的逻辑类似
    // 从目标点做全局BFS
    int target_dist[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                           current_boxes, box_count, target_dist)) {
      return -6;
    }

    // 推箱子到目标
    int step_count = 0;
    int last_dr = 0;
    int last_dc = 0;
    int reverse_count = 0;

    while (box.row != target.row || box.col != target.col) {
      if (step_count++ >= PLANNER_V3_BFS_MAX_GREEDY_STEPS) {
        return -5;
      }

      // [继续使用原来的推箱逻辑 - 选择推箱方向]
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

        if (planner_v3_bfs_is_box_at(current_boxes, box_count, new_box_row, new_box_col,
                      best_box) ||
            planner_v3_bfs_is_box_at(current_boxes, box_count, push_row, push_col,
                      best_box)) {
          continue;
        }

        if (new_box_row == target.row && new_box_col == target.col) {
          // 推到目标点
        } else {
          if (!planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count, 
                                         current_boxes, box_count, best_box,
                                         (Point){new_box_row, new_box_col}, target)) {
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
        
        int adj_pen = planner_v3_bfs_adjacent_blockers(
            rows, cols, obstacles, obstacle_count, current_boxes, box_count,
            best_box, new_box_row, new_box_col);

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
        if (planner_v3_bfs_global_bfs_from_target(rows, cols, push_from, obstacles, 
                                                   obstacle_count, current_boxes, 
                                                   box_count, car_to_push_dist)) {
          int car_idx = current_car.row * cols + current_car.col;
          if (car_idx >= 0 && car_idx < rows * cols && car_to_push_dist[car_idx] != INT_MAX) {
            car_to_push = car_to_push_dist[car_idx];
          } else {
            car_to_push = INT_MAX / 100;
          }
        } else {
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

      // 排序方向
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
        // 无可行方向，尝试绕箱子
        Point detour_path[256];
        size_t detour_len = 0;
        Point new_push_pos;
        
        int detour_ok = planner_v3_bfs_follow_box_with_global_astar(
            rows, cols, current_car, box, target, obstacles, obstacle_count,
            current_boxes, box_count, best_box, detour_path, 256, 
            &detour_len, &new_push_pos);
        
        if (!detour_ok || detour_len == 0) {
          return -6;
        }
        
        for (size_t i = 1; i < detour_len; ++i) {
          if (*out_steps >= path_capacity) {
            return -7;
          }
          if (*out_steps > 0 && 
              !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], detour_path[i])) {
            return -6;
          }
          path_buffer[(*out_steps)++] = detour_path[i];
        }
        
        if (detour_len > 0) {
          current_car = detour_path[detour_len - 1];
        } else {
          current_car = new_push_pos;
        }
        
        continue;
      }

      // 尝试推箱
      int push_success = 0;
      DirCandidate chosen;
      
      for (int try_idx = 0; try_idx < sorted_count; ++try_idx) {
        int dir_idx = sorted_dirs[try_idx];
        DirCandidate candidate = candidates[dir_idx];
        
        Point car_before = current_car;
        size_t steps_before = *out_steps;
        
        if (current_car.row == candidate.push_from.row && 
            current_car.col == candidate.push_from.col) {
          if (*out_steps == 0) {
            if (*out_steps >= path_capacity) {
              return -7;
            }
            path_buffer[(*out_steps)++] = current_car;
          } else {
            Point last = path_buffer[*out_steps - 1];
            if (last.row != current_car.row || last.col != current_car.col) {
              if (!planner_v3_bfs_check_adjacent(last, current_car)) {
                current_car = car_before;
                *out_steps = steps_before;
                continue;
              }
              if (*out_steps >= path_capacity) {
                return -7;
              }
              path_buffer[(*out_steps)++] = current_car;
            }
          }
          chosen = candidate;
          push_success = 1;
          break;
        }
        
        int car_result = planner_v3_bfs_car_move_with_global_astar(
            rows, cols, &current_car, candidate.push_from, obstacles, obstacle_count,
            current_boxes, box_count, path_buffer, path_capacity, out_steps);
        
        if (car_result == -7) {
          return -7;
        }
        
        if (car_result == 0) {
          current_car = car_before;
          *out_steps = steps_before;
          continue;
        }
        
        chosen = candidate;
        push_success = 1;
        break;
      }
      
      if (!push_success) {
        return -6;
      }
      
      if (current_car.row != chosen.push_from.row || 
          current_car.col != chosen.push_from.col) {
        return -6;
      }
      
      int dist_to_box = planner_v3_bfs_manhattan(current_car, box);
      if (dist_to_box != 1) {
        return -6;
      }
      
      // 推箱子
      Point box_old_pos = box;
      box = chosen.box_next;
      current_boxes[best_box] = box;
      
      current_car = box_old_pos;
      if (*out_steps >= path_capacity) {
        return -7;
      }
      if (*out_steps > 0 && 
          !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], current_car)) {
        return -6;
      }
      path_buffer[(*out_steps)++] = current_car;

      // 更新反向移动计数
      if (last_dr == -chosen.dr && last_dc == -chosen.dc && (last_dr != 0 || last_dc != 0)) {
        reverse_count++;
      } else {
        reverse_count = 0;
      }
      
      last_dr = chosen.dr;
      last_dc = chosen.dc;
    }
    
    // 箱子到达目标，标记为消失
    current_boxes[best_box].row = -1;
    current_boxes[best_box].col = -1;
  }

  // 验证路径连续性
  if (*out_steps > 1 && !planner_v3_bfs_validate_continuous_path(path_buffer, *out_steps)) {
    return -6;
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

  // 【核心修改】直接调用新的动态选择函数
  return planner_v3_bfs_run_dynamic(rows, cols, car, boxes, box_count,
                                    targets, target_count, obstacles, obstacle_count,
                                    path_buffer, path_capacity, out_steps,
                                    out_box_target_indices);
}

