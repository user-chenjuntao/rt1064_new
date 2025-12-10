#include "assigned_box_planner_greedy_2.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

#define PLANNER_V3_BFS_MAX_BOXES 10
#define PLANNER_V3_BFS_MAX_CELLS 400
#define PLANNER_V3_BFS_MAX_GREEDY_STEPS 5000
// 路径最长不超过可达格子数量，这里按最大格子数限制，避免大数组造成栈溢出
#define PLANNER_V3_BFS_MAX_PATH_LEN PLANNER_V3_BFS_MAX_CELLS

typedef struct {
  Point box;
  Point target;
  size_t target_idx;
  size_t source_idx;
} PlannerBoxGoalBFS;

typedef struct {
  Point path[PLANNER_V3_BFS_MAX_PATH_LEN];
  Point dir[PLANNER_V3_BFS_MAX_PATH_LEN];
  size_t len;
  int valid;
} PlannerBoxPathInfo;

typedef struct {
  size_t primary;
  size_t primary_start;
  size_t secondary_start;
  size_t overlap_len;
  Point dir;
  int valid;
} PlannerBoxOverlap;

static PlannerBoxPathInfo g_planner_free_paths[PLANNER_V3_BFS_MAX_BOXES];
static PlannerBoxPathInfo g_planner_planned_paths[PLANNER_V3_BFS_MAX_BOXES];
static PlannerBoxOverlap g_planner_overlaps[PLANNER_V3_BFS_MAX_BOXES];

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
  if (!planner_v3_bfs_in_bounds(rows, cols, target.row, target.col)) {
    return 0;
  }

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
      out[i] = boxes[i];
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
                                             temp_boxes, box_count, dist)) {
    return 0;
  }

  size_t path_len = 0;
  if (!planner_v3_bfs_astar_with_dist(rows, cols, start, target, obstacles, obstacle_count,
                                      temp_boxes, box_count, dist, out->path,
                                      PLANNER_V3_BFS_MAX_PATH_LEN, &path_len)) {
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

static void planner_v3_bfs_detect_overlaps(const PlannerBoxPathInfo *free_paths,
                                       const Point *current_boxes, size_t box_count,
                                       PlannerBoxOverlap *overlaps) {
  for (size_t i = 0; i < box_count; ++i) {
    overlaps[i].valid = 0;
    overlaps[i].primary = SIZE_MAX;
    overlaps[i].primary_start = 0;
    overlaps[i].secondary_start = 0;
    overlaps[i].overlap_len = 0;
    overlaps[i].dir.row = 0;
    overlaps[i].dir.col = 0;
  }

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
          if (!ov->valid || overlap_len > ov->overlap_len ||
              (overlap_len == ov->overlap_len && pi < ov->primary_start)) {
            ov->valid = 1;
            ov->primary = primary;
            ov->primary_start = pi;
            ov->secondary_start = 0;
            ov->overlap_len = overlap_len;
            ov->dir = free_paths[primary].dir[pi];
          }
        }
        break;  // 只考虑首次经过secondary起点的重叠
      }
    }
  }
}

static int planner_v3_bfs_execute_single_box_path(int rows, int cols, size_t box_idx,
                                              const PlannerBoxPathInfo *path_info,
                                              const Point *obstacles, size_t obstacle_count,
                                              Point *current_car, Point *current_boxes,
                                              size_t box_count, Point *path_buffer,
                                              size_t path_capacity, size_t *out_steps) {
  if (!path_info || !path_info->valid || path_info->len == 0) {
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
      return -6;
    }

    int car_result = planner_v3_bfs_car_move_with_global_astar(
        rows, cols, current_car, push_pos, obstacles, obstacle_count, current_boxes, box_count,
        path_buffer, path_capacity, out_steps);
    if (car_result == -7) {
      return -7;
    }
    if (car_result == 0) {
      return -6;
    }

    if (current_car->row != push_pos.row || current_car->col != push_pos.col) {
      return -6;
    }

    if (!planner_v3_bfs_in_bounds(rows, cols, next.row, next.col)) {
      return -6;
    }
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, next.row, next.col)) {
      return -6;
    }
    for (size_t bi = 0; bi < box_count; ++bi) {
      if (bi == box_idx) {
        continue;
      }
      Point b = current_boxes[bi];
      if (b.row == next.row && b.col == next.col) {
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

static int planner_v3_bfs_push_single_box_scored(
    int rows, int cols, size_t box_idx, size_t target_idx, const Point *targets,
    const Point *obstacles, size_t obstacle_count, Point *current_car, Point *current_boxes,
    size_t box_count, Point *path_buffer, size_t path_capacity, size_t *out_steps) {
  if (target_idx == SIZE_MAX) {
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
                                             current_boxes, box_count, target_dist)) {
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
      if (planner_v3_bfs_global_bfs_from_target(rows, cols, push_from, obstacles, obstacle_count,
                                                current_boxes, box_count, car_to_push_dist)) {
        int car_idx = current_car->row * cols + current_car->col;
        if (car_idx >= 0 && car_idx < rows * cols && car_to_push_dist[car_idx] != INT_MAX) {
          car_to_push = car_to_push_dist[car_idx];
        } else {
          car_to_push = INT_MAX / 100;
        }
      } else {
        car_to_push = planner_v3_bfs_manhattan(*current_car, push_from);
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
      return -6;
    }

    if (current_car->row != chosen.push_from.row || current_car->col != chosen.push_from.col) {
      return -6;
    }

    int dist_to_box = planner_v3_bfs_manhattan(*current_car, box);
    if (dist_to_box != 1) {
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

static int planner_v3_bfs_split_then_finish(int rows, int cols, size_t primary_idx,
                                        size_t secondary_idx, const size_t *box_targets,
                                        const Point *targets, const Point *obstacles,
                                        size_t obstacle_count, Point *current_car,
                                        Point *current_boxes, size_t box_count,
                                        Point *path_buffer, size_t path_capacity,
                                        size_t *out_steps) {
  if (box_targets[primary_idx] == SIZE_MAX || box_targets[secondary_idx] == SIZE_MAX) {
    return -6;
  }

  PlannerBoxPathInfo sec_path;
  if (!planner_v3_bfs_compute_box_path_with_mask(
          rows, cols, current_boxes[secondary_idx], targets[box_targets[secondary_idx]],
          obstacles, obstacle_count, current_boxes, box_count, NULL, secondary_idx, &sec_path)) {
    return -6;
  }

  int res = planner_v3_bfs_execute_single_box_path(rows, cols, secondary_idx, &sec_path,
                                                   obstacles, obstacle_count, current_car,
                                                   current_boxes, box_count, path_buffer,
                                                   path_capacity, out_steps);
  if (res != 0) {
    return res;
  }

  current_boxes[secondary_idx].row = -1;
  current_boxes[secondary_idx].col = -1;

  PlannerBoxPathInfo pri_path;
  if (!planner_v3_bfs_compute_box_path_with_mask(
          rows, cols, current_boxes[primary_idx], targets[box_targets[primary_idx]], obstacles,
          obstacle_count, current_boxes, box_count, NULL, primary_idx, &pri_path)) {
    return -6;
  }

  return planner_v3_bfs_execute_single_box_path(rows, cols, primary_idx, &pri_path, obstacles,
                                                obstacle_count, current_car, current_boxes,
                                                box_count, path_buffer, path_capacity, out_steps);
}

static int planner_v3_bfs_execute_chain_then_targets(
    int rows, int cols, size_t primary_idx, size_t secondary_idx,
    const PlannerBoxPathInfo *planned_paths, const size_t *box_targets,
    const Point *targets, const Point *obstacles, size_t obstacle_count,
    Point *current_car, Point *current_boxes, size_t box_count, Point *path_buffer,
    size_t path_capacity, size_t *out_steps) {
  if (box_targets[primary_idx] == SIZE_MAX || box_targets[secondary_idx] == SIZE_MAX) {
    return -6;
  }

  const PlannerBoxPathInfo *secondary_path = &planned_paths[secondary_idx];
  if (!secondary_path->valid || secondary_path->len < 2) {
    return -6;
  }

  size_t sec_idx = 0;
  for (; sec_idx < secondary_path->len; ++sec_idx) {
    if (secondary_path->path[sec_idx].row == current_boxes[secondary_idx].row &&
        secondary_path->path[sec_idx].col == current_boxes[secondary_idx].col) {
      break;
    }
  }
  if (sec_idx >= secondary_path->len) {
    return -6;
  }

  Point last_dir = {0, 0};
  for (; sec_idx + 1 < secondary_path->len; ++sec_idx) {
    Point sec_curr = current_boxes[secondary_idx];
    Point sec_next = secondary_path->path[sec_idx + 1];
    Point dir = {sec_next.row - sec_curr.row, sec_next.col - sec_curr.col};

    int turning = (last_dir.row != 0 || last_dir.col != 0) &&
                  (last_dir.row != dir.row || last_dir.col != dir.col);

    if (turning) {
      // 拐弯：车先推副箱到新方向，再推主箱到拐弯点
      Point push_pos_sec = {sec_curr.row - dir.row, sec_curr.col - dir.col};
      if (!planner_v3_bfs_in_bounds(rows, cols, sec_next.row, sec_next.col) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, sec_next.row, sec_next.col) ||
          planner_v3_bfs_is_box_at(current_boxes, box_count, sec_next.row, sec_next.col,
                                   primary_idx) ||
          !planner_v3_bfs_in_bounds(rows, cols, push_pos_sec.row, push_pos_sec.col) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_pos_sec.row, push_pos_sec.col) ||
          planner_v3_bfs_is_box_at(current_boxes, box_count, push_pos_sec.row, push_pos_sec.col,
                                   secondary_idx)) {
        return planner_v3_bfs_split_then_finish(rows, cols, primary_idx, secondary_idx, box_targets,
                                                targets, obstacles, obstacle_count, current_car,
                                                current_boxes, box_count, path_buffer,
                                                path_capacity, out_steps);
      }
      if (current_car->row != push_pos_sec.row || current_car->col != push_pos_sec.col) {
        int car_res = planner_v3_bfs_car_move_with_global_astar(
            rows, cols, current_car, push_pos_sec, obstacles, obstacle_count, current_boxes,
            box_count, path_buffer, path_capacity, out_steps);
        if (car_res <= 0) {
          return planner_v3_bfs_split_then_finish(rows, cols, primary_idx, secondary_idx,
                                                  box_targets, targets, obstacles, obstacle_count,
                                                  current_car, current_boxes, box_count,
                                                  path_buffer, path_capacity, out_steps);
        }
      }
      if (*out_steps >= path_capacity) {
        return -7;
      }
      Point old_sec = sec_curr;
      current_boxes[secondary_idx] = sec_next;
      *current_car = old_sec;
      path_buffer[(*out_steps)++] = *current_car;

      if (last_dir.row == 0 && last_dir.col == 0) {
        return planner_v3_bfs_split_then_finish(rows, cols, primary_idx, secondary_idx, box_targets,
                                                targets, obstacles, obstacle_count, current_car,
                                                current_boxes, box_count, path_buffer,
                                                path_capacity, out_steps);
      }
      Point primary_curr = current_boxes[primary_idx];
      Point bend_target = sec_curr;
      Point push_pos_primary = {primary_curr.row - last_dir.row,
                                primary_curr.col - last_dir.col};
      if (!planner_v3_bfs_in_bounds(rows, cols, push_pos_primary.row, push_pos_primary.col) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_pos_primary.row,
                                     push_pos_primary.col) ||
          planner_v3_bfs_is_box_at(current_boxes, box_count, push_pos_primary.row,
                                   push_pos_primary.col, primary_idx)) {
        return planner_v3_bfs_split_then_finish(rows, cols, primary_idx, secondary_idx,
                                                box_targets, targets, obstacles, obstacle_count,
                                                current_car, current_boxes, box_count, path_buffer,
                                                path_capacity, out_steps);
      }
      if (current_car->row != push_pos_primary.row || current_car->col != push_pos_primary.col) {
        int car_res = planner_v3_bfs_car_move_with_global_astar(
            rows, cols, current_car, push_pos_primary, obstacles, obstacle_count, current_boxes,
            box_count, path_buffer, path_capacity, out_steps);
        if (car_res <= 0) {
          return planner_v3_bfs_split_then_finish(rows, cols, primary_idx, secondary_idx,
                                                  box_targets, targets, obstacles, obstacle_count,
                                                  current_car, current_boxes, box_count,
                                                  path_buffer, path_capacity, out_steps);
        }
      }
      if (*out_steps >= path_capacity) {
        return -7;
      }
      Point old_primary = primary_curr;
      current_boxes[primary_idx] = bend_target;
      *current_car = old_primary;
      path_buffer[(*out_steps)++] = *current_car;
    } else {
      // 直行：主箱推副箱
      Point primary_curr = current_boxes[primary_idx];
      Point expect_primary = {sec_curr.row - dir.row, sec_curr.col - dir.col};
      if (primary_curr.row != expect_primary.row || primary_curr.col != expect_primary.col) {
        int align_res = planner_v3_bfs_car_move_with_global_astar(
            rows, cols, current_car, expect_primary, obstacles, obstacle_count, current_boxes,
            box_count, path_buffer, path_capacity, out_steps);
        if (align_res <= 0) {
          return (align_res == -7) ? -7 : -6;
        }
        primary_curr = current_boxes[primary_idx];
        if (primary_curr.row != expect_primary.row || primary_curr.col != expect_primary.col) {
          return -6;
        }
      }

      Point push_pos = {primary_curr.row - dir.row, primary_curr.col - dir.col};
      if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_pos.row, push_pos.col) ||
          planner_v3_bfs_is_box_at(current_boxes, box_count, push_pos.row, push_pos.col,
                                   primary_idx)) {
        return -6;
      }
      if (current_car->row != push_pos.row || current_car->col != push_pos.col) {
        int car_res = planner_v3_bfs_car_move_with_global_astar(
            rows, cols, current_car, push_pos, obstacles, obstacle_count, current_boxes, box_count,
            path_buffer, path_capacity, out_steps);
        if (car_res <= 0) {
          return (car_res == -7) ? -7 : -6;
        }
      }

      if (!planner_v3_bfs_in_bounds(rows, cols, sec_next.row, sec_next.col) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, sec_next.row, sec_next.col)) {
        return planner_v3_bfs_split_then_finish(rows, cols, primary_idx, secondary_idx,
                                                box_targets, targets, obstacles, obstacle_count,
                                                current_car, current_boxes, box_count, path_buffer,
                                                path_capacity, out_steps);
      }
      for (size_t bi = 0; bi < box_count; ++bi) {
        if (bi == primary_idx || bi == secondary_idx) {
          continue;
        }
        Point b = current_boxes[bi];
        if (b.row == sec_next.row && b.col == sec_next.col) {
          return planner_v3_bfs_split_then_finish(rows, cols, primary_idx, secondary_idx,
                                                  box_targets, targets, obstacles, obstacle_count,
                                                  current_car, current_boxes, box_count,
                                                  path_buffer, path_capacity, out_steps);
        }
      }

      Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
      for (size_t bi = 0; bi < box_count; ++bi) {
        temp_boxes[bi] = current_boxes[bi];
      }
      temp_boxes[secondary_idx] = sec_next;
      temp_boxes[primary_idx] = sec_curr;
      uint8_t ignore_mask[PLANNER_V3_BFS_MAX_BOXES] = {0};
      ignore_mask[primary_idx] = 1;
      if (!planner_v3_bfs_can_reach_goal_mask(
              rows, cols, obstacles, obstacle_count, temp_boxes, box_count, secondary_idx,
              ignore_mask, sec_next, targets[box_targets[secondary_idx]])) {
        return planner_v3_bfs_split_then_finish(rows, cols, primary_idx, secondary_idx,
                                                box_targets, targets, obstacles, obstacle_count,
                                                current_car, current_boxes, box_count, path_buffer,
                                                path_capacity, out_steps);
      }

      if (*out_steps >= path_capacity) {
        return -7;
      }

      Point old_primary = primary_curr;
      Point old_secondary = sec_curr;
      current_boxes[secondary_idx] = sec_next;
      current_boxes[primary_idx] = old_secondary;
      *current_car = old_primary;
      path_buffer[(*out_steps)++] = *current_car;
    }

    last_dir = dir;
  }

  current_boxes[secondary_idx].row = -1;
  current_boxes[secondary_idx].col = -1;

  PlannerBoxPathInfo primary_follow;
  uint8_t ignore_mask[PLANNER_V3_BFS_MAX_BOXES] = {0};
  if (!planner_v3_bfs_compute_box_path_with_mask(
          rows, cols, current_boxes[primary_idx], targets[box_targets[primary_idx]], obstacles,
          obstacle_count, current_boxes, box_count, ignore_mask, primary_idx, &primary_follow)) {
    return -6;
  }

  return planner_v3_bfs_execute_single_box_path(rows, cols, primary_idx, &primary_follow,
                                                obstacles, obstacle_count, current_car,
                                                current_boxes, box_count, path_buffer,
                                                path_capacity, out_steps);
}

static int planner_v3_bfs_push_primary_with_chain(
    int rows, int cols, size_t primary_idx, const PlannerBoxPathInfo *planned_paths,
    const PlannerBoxOverlap *overlaps, const size_t *box_targets, const Point *targets,
    const Point *obstacles, size_t obstacle_count, Point *current_car, Point *current_boxes,
    size_t box_count, Point *path_buffer, size_t path_capacity, size_t *out_steps) {
  const PlannerBoxPathInfo *path = &planned_paths[primary_idx];
  if (!path->valid || path->len < 2) {
    return -6;
  }

  uint8_t deadlock_ignore_mask[PLANNER_V3_BFS_MAX_BOXES] = {0};
  for (size_t s = 0; s < box_count; ++s) {
    if (overlaps[s].valid && overlaps[s].primary == primary_idx) {
      deadlock_ignore_mask[s] = 1;
    }
  }

  int has_secondary = 0;
  for (size_t s = 0; s < box_count; ++s) {
    if (overlaps[s].valid && overlaps[s].primary == primary_idx) {
      has_secondary = 1;
      break;
    }
  }

  if (!has_secondary) {
    return planner_v3_bfs_push_single_box_scored(
        rows, cols, primary_idx, box_targets[primary_idx], targets, obstacles, obstacle_count,
        current_car, current_boxes, box_count, path_buffer, path_capacity, out_steps);
  }

  for (size_t step = 1; step < path->len; ++step) {
    Point curr = current_boxes[primary_idx];
    Point next = path->path[step];
    Point dir = {next.row - curr.row, next.col - curr.col};
    Point push_pos = {curr.row - dir.row, curr.col - dir.col};
    int retried = 0;

retry_push:
    if (!planner_v3_bfs_in_bounds(rows, cols, push_pos.row, push_pos.col) ||
        planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_pos.row, push_pos.col) ||
        planner_v3_bfs_is_box_at(current_boxes, box_count, push_pos.row, push_pos.col,
                                 primary_idx)) {
      if (retried) {
        return -6;
      }
      Point detour_path[256];
      size_t detour_len = 0;
      Point new_push_pos;
      int detour_ok = planner_v3_bfs_follow_box_with_global_astar(
          rows, cols, *current_car, curr, next, obstacles, obstacle_count, current_boxes,
          box_count, primary_idx, detour_path, 256, &detour_len, &new_push_pos);
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
      *current_car = detour_path[detour_len - 1];
      push_pos = *current_car;
      retried = 1;
      goto retry_push;
    }

    int car_result = planner_v3_bfs_car_move_with_global_astar(
        rows, cols, current_car, push_pos, obstacles, obstacle_count, current_boxes, box_count,
        path_buffer, path_capacity, out_steps);
    if (car_result == -7) {
      return -7;
    }
    if (car_result == 0) {
      if (retried) {
        return -6;
      }
      Point detour_path[256];
      size_t detour_len = 0;
      Point new_push_pos;
      int detour_ok = planner_v3_bfs_follow_box_with_global_astar(
          rows, cols, *current_car, curr, next, obstacles, obstacle_count, current_boxes,
          box_count, primary_idx, detour_path, 256, &detour_len, &new_push_pos);
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
      *current_car = detour_path[detour_len - 1];
      push_pos = *current_car;
      retried = 1;
      goto retry_push;
    }

    if (current_car->row != push_pos.row || current_car->col != push_pos.col) {
      return -6;
    }

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
      return planner_v3_bfs_execute_chain_then_targets(
          rows, cols, primary_idx, secondary, planned_paths, box_targets, targets, obstacles,
          obstacle_count, current_car, current_boxes, box_count, path_buffer, path_capacity,
          out_steps);
    }

    if (!planner_v3_bfs_in_bounds(rows, cols, next.row, next.col) ||
        planner_v3_bfs_is_obstacle(obstacles, obstacle_count, next.row, next.col)) {
      return -6;
    }
    for (size_t bi = 0; bi < box_count; ++bi) {
      if (bi == primary_idx) {
        continue;
      }
      Point b = current_boxes[bi];
      if (b.row == next.row && b.col == next.col) {
        return -6;
      }
    }

    if (box_targets[primary_idx] == SIZE_MAX) {
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
// 支持先忽略箱子阻挡寻找重叠路径，再以多箱联动方式规划
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
  if (out_box_target_indices) {
    for (size_t i = 0; i < box_count; ++i) {
      out_box_target_indices[i] = SIZE_MAX;
    }
  }

  Point current_car = car;
  Point current_boxes[PLANNER_V3_BFS_MAX_BOXES];
  size_t box_targets[PLANNER_V3_BFS_MAX_BOXES];
  uint8_t box_done[PLANNER_V3_BFS_MAX_BOXES] = {0};

  for (size_t i = 0; i < box_count; ++i) {
    current_boxes[i] = boxes[i];
    box_targets[i] = SIZE_MAX;
  }

  PlannerBoxGoalBFS goals[PLANNER_V3_BFS_MAX_BOXES];
  size_t goal_count = 0;
  size_t mapping[PLANNER_V3_BFS_MAX_BOXES];
  int assign_res = planner_v3_bfs_assign_targets(rows, cols, car, boxes, box_count, targets,
                                                 target_count, goals, &goal_count, mapping);
  if (assign_res != 0) {
    return assign_res;
  }
  (void)goal_count;

  for (size_t i = 0; i < box_count; ++i) {
    box_targets[i] = mapping[i];
    if (out_box_target_indices) {
      out_box_target_indices[i] = mapping[i];
    }
  }

  size_t remaining = 0;
  for (size_t i = 0; i < box_count; ++i) {
    if (current_boxes[i].row < 0 || current_boxes[i].col < 0) {
      box_done[i] = 1;
      continue;
    }
    if (box_targets[i] != SIZE_MAX) {
      Point tgt = targets[box_targets[i]];
      if (current_boxes[i].row == tgt.row && current_boxes[i].col == tgt.col) {
        current_boxes[i].row = -1;
        current_boxes[i].col = -1;
        box_done[i] = 1;
        continue;
      }
    }
    remaining++;
  }

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

    for (size_t i = 0; i < box_count; ++i) {
      if (box_done[i]) {
        continue;
      }
      if (box_targets[i] == SIZE_MAX) {
        return -6;
      }
      uint8_t ignore_all[PLANNER_V3_BFS_MAX_BOXES];
      for (size_t j = 0; j < box_count; ++j) {
        ignore_all[j] = 1;
      }
      if (!planner_v3_bfs_compute_box_path_with_mask(
              rows, cols, current_boxes[i], targets[box_targets[i]], obstacles, obstacle_count,
              current_boxes, box_count, ignore_all, i, &free_paths[i])) {
        return -6;
      }
    }

    planner_v3_bfs_detect_overlaps(free_paths, current_boxes, box_count, overlaps);

    for (size_t i = 0; i < box_count; ++i) {
      if (box_done[i]) {
        continue;
      }
      uint8_t ignore_mask[PLANNER_V3_BFS_MAX_BOXES] = {0};
      ignore_mask[i] = 1;
      // 作为主箱，忽略自己对应的副箱；作为副箱，忽略自己的主箱，避免互相占位导致死角误判
      for (size_t s = 0; s < box_count; ++s) {
        if (overlaps[s].valid && overlaps[s].primary == i) {
          ignore_mask[s] = 1;
        }
      }
      if (overlaps[i].valid && overlaps[i].primary != SIZE_MAX) {
        ignore_mask[overlaps[i].primary] = 1;
      }
      if (!planner_v3_bfs_compute_box_path_with_mask(
              rows, cols, current_boxes[i], targets[box_targets[i]], obstacles, obstacle_count,
              current_boxes, box_count, ignore_mask, i, &planned_paths[i])) {
        return -6;
      }
    }

    size_t candidate = SIZE_MAX;
    int best_score = INT_MAX;

    for (size_t i = 0; i < box_count; ++i) {
      if (box_done[i]) {
        continue;
      }
      if (!planned_paths[i].valid || planned_paths[i].len == 0) {
        return -6;
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
        Point first_dir = planned_paths[i].dir[0];
        Point push_pos = {current_boxes[i].row - first_dir.row,
                          current_boxes[i].col - first_dir.col};
        int dist_map[PLANNER_V3_BFS_MAX_CELLS];
        if (planner_v3_bfs_global_bfs_from_target(rows, cols, push_pos, obstacles, obstacle_count,
                                                  current_boxes, box_count, dist_map)) {
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

      if (score < best_score) {
        best_score = score;
        candidate = i;
      }
    }

    if (candidate == SIZE_MAX) {
      return -6;
    }

    int push_res = planner_v3_bfs_push_primary_with_chain(
        rows, cols, candidate, planned_paths, overlaps, box_targets, targets, obstacles,
        obstacle_count, &current_car, current_boxes, box_count, path_buffer, path_capacity,
        out_steps);
    if (push_res != 0) {
      return push_res;
    }

    remaining = 0;
    for (size_t i = 0; i < box_count; ++i) {
      if (current_boxes[i].row < 0 || current_boxes[i].col < 0) {
        box_done[i] = 1;
        continue;
      }
      box_done[i] = 0;
      remaining++;
    }
  }

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
