#include "assigned_box_planner_greedy_2.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

extern int last_err_stage;   // 错误阶段
extern int last_err_detail;  // 错误详情

int last_err_stage = 0;
int last_err_detail = 0;

#define PLANNER_V3_BFS_MAX_BOXES 10
#define PLANNER_V3_BFS_MAX_CELLS 400
#define PLANNER_V3_BFS_MAX_GREEDY_STEPS 5000
#define PLANNER_V3_BFS_MAX_PATH_LEN 5000
#define PLANNER_V3_BFS_STRATEGY_PATH 1
#define PLANNER_V3_BFS_STRATEGY_SCORE 2

typedef struct {
  Point box;
  Point target;
  size_t target_idx;
  size_t source_idx;
} PlannerBoxGoalBFS;

static int planner_v3_bfs_abs(int v) { return v >= 0 ? v : -v; }

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

// 判断某个位置是否是死点
// 死点判断规则：
// 1. 被挡住的方向至少有两个
// 2. 如果只有两个被挡住的方向，且这两个方向不同时为"上、下"或"左、右"时，才是死点
// 3. 其余情况不为死点
// 返回值：1=是死点，0=不是死点
static int planner_v3_bfs_is_deadlock(int rows, int cols, const Point *obstacles,
                                     size_t obstacle_count, const Point *boxes,
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
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, nr, nc) ||
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
static int planner_v3_bfs_can_reach_goal(int rows, int cols, const Point *obstacles,
                                size_t obstacle_count, const Point *boxes,
                                size_t box_count, size_t moving_idx,
                                Point start, Point target) {
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_BFS_MAX_CELLS || total_cells <= 0) {
    return 0;
  }
  if (!planner_v3_bfs_in_bounds(rows, cols, start.row, start.col) ||
      !planner_v3_bfs_in_bounds(rows, cols, target.row, target.col)) {
    return 0;
  }
  if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, start.row, start.col) ||
      planner_v3_bfs_is_obstacle(obstacles, obstacle_count, target.row, target.col)) {
    return 0;
  }
  if (planner_v3_bfs_is_box_at(boxes, box_count, start.row, start.col, SIZE_MAX) ||
      planner_v3_bfs_is_box_at(boxes, box_count, target.row, target.col, SIZE_MAX)) {
    return 0;
  }
  if (!planner_v3_bfs_in_bounds(rows, cols, target.row, target.col)) {
    return 0;
  }
  if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, target.row, target.col) ||
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
// check_push: 1=在扩展时检测推位，若节点A到节点B的推位不合理（障碍、箱、边界外）则排除节点B；0=不检测（车路径等）
// 返回值：1=成功找到路径，0=失败
static int planner_v3_bfs_astar_with_dist(int rows, int cols, Point start, Point target,
                                      const Point *obstacles, size_t obstacle_count,
                                      const Point *boxes, size_t box_count,
                                      const int dist[PLANNER_V3_BFS_MAX_CELLS],
                                      Point *path, size_t path_cap, size_t *path_len,
                                      int check_push) {
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
      
      /* 推位检测：若从节点A(curr)到节点B(next)的推位不合理（障碍、箱、边界外），则排除节点B */
      if (check_push) {
        int push_row = curr_row * 2 - nr;
        int push_col = curr_col * 2 - nc;
        if (!planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
          continue;
        }
        if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_row, push_col)) {
          continue;
        }
        if (planner_v3_bfs_is_box_at(boxes, box_count, push_row, push_col, SIZE_MAX)) {
          continue;
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

// 计算两点之间的BFS真实距离（不可达返回INT_MAX）
static int planner_v3_bfs_distance_between(int rows, int cols, Point start, Point target,
                                           const Point *obstacles, size_t obstacle_count,
                                           const Point *boxes, size_t box_count) {
  if (!planner_v3_bfs_in_bounds(rows, cols, start.row, start.col) ||
      !planner_v3_bfs_in_bounds(rows, cols, target.row, target.col)) {
    return INT_MAX;
  }
  int dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                             boxes, box_count, dist)) {
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
    int dist = planner_v3_bfs_distance_between(rows, cols, car, valid_push_positions[i],
                                               obstacles, obstacle_count, boxes, box_count);
    sorted_positions[i].dist = (dist == INT_MAX) ? INT_MAX / 100 : dist;
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
                                  temp_path, 256, &temp_len, 0)) {
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
// 返回值：1=成功，0=失败
static int planner_v3_bfs_simulate_car_move_steps(int rows, int cols, Point *car_pos, Point target,
                                                 const Point *obstacles, size_t obstacle_count,
                                                 const Point *boxes, size_t box_count,
                                                 size_t *out_steps) {
  if (car_pos->row == target.row && car_pos->col == target.col) {
    return 1;
  }

  int dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                             boxes, box_count, dist)) {
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

// 推箱子到目标：支持按路径或按评分策略，可用于仿真（simulate_only=1）
static int planner_v3_bfs_push_box_with_strategy(
    int rows, int cols, Point *car_pos, Point *boxes, size_t box_count, size_t box_idx,
    Point target, const Point *obstacles, size_t obstacle_count, const int *target_dist,
    const Point *box_path, size_t box_path_len, int has_box_path, int strategy,
    int simulate_only, Point *path_buffer, size_t path_capacity, size_t *out_steps,
    Point (*box_paths)[PLANNER_V3_BFS_MAX_CELLS], size_t *box_path_lens,
    int record_box_path) {
  if (!car_pos || !boxes || !out_steps) {
    return -1;
  }
  if (!simulate_only && !path_buffer) {
    return -1;
  }

  Point box = boxes[box_idx];
  int step_count = 0;
  int last_dr = 0;
  int last_dc = 0;
  int reverse_count = 0;
  size_t box_path_idx = 0;

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

#define SET_ERR(stage, detail)           \
  do {                                   \
    if (!simulate_only) {                \
      last_err_stage = (stage);          \
      last_err_detail = (detail);        \
    }                                    \
  } while (0)

  while (box.row != target.row || box.col != target.col) {
    if (step_count++ >= PLANNER_V3_BFS_MAX_GREEDY_STEPS) {
      return -5;
    }

    if (strategy == PLANNER_V3_BFS_STRATEGY_PATH) {
      if (!has_box_path || box_path_idx + 1 >= box_path_len) {
        SET_ERR(2, 7);
        return -6;
      }

      Point next_in_path = box_path[box_path_idx + 1];
      int dr = next_in_path.row - box.row;
      int dc = next_in_path.col - box.col;
      if (planner_v3_bfs_abs(dr) + planner_v3_bfs_abs(dc) != 1) {
        SET_ERR(2, 7);
        return -6;
      }

      int push_row = box.row - dr;
      int push_col = box.col - dc;

      if (!planner_v3_bfs_in_bounds(rows, cols, next_in_path.row, next_in_path.col) ||
          !planner_v3_bfs_in_bounds(rows, cols, push_row, push_col)) {
        SET_ERR(2, 7);
        return -6;
      }
      if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, next_in_path.row, next_in_path.col) ||
          planner_v3_bfs_is_obstacle(obstacles, obstacle_count, push_row, push_col)) {
        SET_ERR(2, 7);
        return -6;
      }
      if (planner_v3_bfs_is_box_at(boxes, box_count, next_in_path.row, next_in_path.col, box_idx) ||
          planner_v3_bfs_is_box_at(boxes, box_count, push_row, push_col, box_idx)) {
        SET_ERR(2, 7);
        return -6;
      }

      if (next_in_path.row != target.row || next_in_path.col != target.col) {
        if (!planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count, boxes,
                                           box_count, box_idx, next_in_path, target)) {
          SET_ERR(2, 7);
          return -6;
        }
      }

      Point push_from = {push_row, push_col};
      if (car_pos->row != push_from.row || car_pos->col != push_from.col) {
        int car_result;
        if (simulate_only) {
          car_result = planner_v3_bfs_simulate_car_move_steps(
              rows, cols, car_pos, push_from, obstacles, obstacle_count, boxes, box_count,
              out_steps);
        } else {
          car_result = planner_v3_bfs_car_move_with_global_astar(
              rows, cols, car_pos, push_from, obstacles, obstacle_count, boxes, box_count,
              path_buffer, path_capacity, out_steps);
        }
        if (car_result == -7) {
          return -7;
        }
        if (car_result == 0) {
          SET_ERR(2, 15);
          return -6;
        }
      }

      if (car_pos->row != push_from.row || car_pos->col != push_from.col) {
        SET_ERR(2, 15);
        return -6;
      }

      if (!planner_v3_bfs_check_adjacent(*car_pos, box)) {
        SET_ERR(2, 29);
        return -6;
      }

      Point box_old_pos = box;
      box = next_in_path;
      boxes[box_idx] = box;
      box_path_idx++;

      if (record_box_path && box_paths && box_path_lens &&
          box_path_lens[box_idx] < PLANNER_V3_BFS_MAX_CELLS) {
        box_paths[box_idx][box_path_lens[box_idx]++] = box;
      }

      *car_pos = box_old_pos;
      if (simulate_only) {
        (*out_steps)++;
      } else {
        if (*out_steps >= path_capacity) {
          return -7;
        }
        if (*out_steps > 0 &&
            !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], *car_pos)) {
          SET_ERR(2, 2);
          return -6;
        }
        path_buffer[(*out_steps)++] = *car_pos;
      }

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
      if (planner_v3_bfs_is_box_at(boxes, box_count, new_box_row, new_box_col, box_idx) ||
          planner_v3_bfs_is_box_at(boxes, box_count, push_row, push_col, box_idx)) {
        continue;
      }

      if (new_box_row != target.row || new_box_col != target.col) {
        if (!planner_v3_bfs_can_reach_goal(rows, cols, obstacles, obstacle_count,
                                           boxes, box_count, box_idx,
                                           (Point){new_box_row, new_box_col}, target)) {
          continue;
        }
      }

      Point push_from = {push_row, push_col};

      int dist_after = INT_MAX / 100;
      // 计算 dist_after 时，不把当前正在移动的箱子当作障碍
      // 构建临时箱子数组（排除当前正在移动的箱子）
      Point temp_boxes_for_dist[PLANNER_V3_BFS_MAX_BOXES];
      size_t temp_box_count = 0;
      for (size_t j = 0; j < box_count; ++j) {
        if (j == box_idx) {
          continue;  // 忽略当前正在移动的箱子
        }
        if (boxes[j].row < 0 || boxes[j].col < 0) {
          continue;  // 忽略已完成的箱子
        }
        temp_boxes_for_dist[temp_box_count++] = boxes[j];
      }
      
      // 重新计算不包含当前箱子的距离
      int temp_target_dist[PLANNER_V3_BFS_MAX_CELLS];
      if (planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                                temp_boxes_for_dist, temp_box_count, temp_target_dist)) {
        int idx = new_box_row * cols + new_box_col;
        if (idx >= 0 && idx < rows * cols && temp_target_dist[idx] != INT_MAX) {
          dist_after = temp_target_dist[idx];
        }
      }

      int adj_pen = planner_v3_bfs_adjacent_blockers(
          rows, cols, obstacles, obstacle_count, boxes, box_count,
          box_idx, new_box_row, new_box_col);

      int reverse_pen = 0;
      if (last_dr == -dirs[i][0] && last_dc == -dirs[i][1]) {
        int power = 5;
        for (int p = 0; p < reverse_count; ++p) {
          power *= 5;
        }
        reverse_pen = power;
      }

      int car_to_push = INT_MAX / 100;
      int car_to_push_dist[PLANNER_V3_BFS_MAX_CELLS];
      if (planner_v3_bfs_global_bfs_from_target(rows, cols, push_from, obstacles,
                                                obstacle_count, boxes,
                                                box_count, car_to_push_dist)) {
        int car_idx = car_pos->row * cols + car_pos->col;
        if (car_idx >= 0 && car_idx < rows * cols && car_to_push_dist[car_idx] != INT_MAX) {
          car_to_push = car_to_push_dist[car_idx];
        }
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

    int chosen_ok = 0;
    DirCandidate chosen;

    for (int try_idx = 0; try_idx < sorted_count; ++try_idx) {
      int dir_idx = sorted_dirs[try_idx];
      DirCandidate candidate = candidates[dir_idx];

      Point car_before = *car_pos;
      size_t steps_before = *out_steps;

      if (car_pos->row == candidate.push_from.row &&
          car_pos->col == candidate.push_from.col) {
        if (!simulate_only) {
          if (*out_steps == 0) {
            if (*out_steps >= path_capacity) {
              return -7;
            }
            path_buffer[(*out_steps)++] = *car_pos;
          } else {
            Point last = path_buffer[*out_steps - 1];
            if (last.row != car_pos->row || last.col != car_pos->col) {
              if (!planner_v3_bfs_check_adjacent(last, *car_pos)) {
                *car_pos = car_before;
                *out_steps = steps_before;
                continue;
              }
              if (*out_steps >= path_capacity) {
                return -7;
              }
              path_buffer[(*out_steps)++] = *car_pos;
            }
          }
        }
        chosen = candidate;
        chosen_ok = 1;
        break;
      }

      int car_result;
      if (simulate_only) {
        car_result = planner_v3_bfs_simulate_car_move_steps(
            rows, cols, car_pos, candidate.push_from, obstacles, obstacle_count, boxes,
            box_count, out_steps);
      } else {
        car_result = planner_v3_bfs_car_move_with_global_astar(
            rows, cols, car_pos, candidate.push_from, obstacles, obstacle_count, boxes,
            box_count, path_buffer, path_capacity, out_steps);
      }

      if (car_result == -7) {
        return -7;
      }
      if (car_result == 0) {
        *car_pos = car_before;
        *out_steps = steps_before;
        continue;
      }

      chosen = candidate;
      chosen_ok = 1;
      break;
    }

    if (!chosen_ok) {
      SET_ERR(2, 7);
      return -6;
    }

    if (car_pos->row != chosen.push_from.row || car_pos->col != chosen.push_from.col) {
      SET_ERR(2, 15);
      return -6;
    }

    if (!planner_v3_bfs_check_adjacent(*car_pos, box)) {
      SET_ERR(2, 29);
      return -6;
    }

    Point box_old_pos = box;
    box = chosen.box_next;
    boxes[box_idx] = box;

    if (record_box_path && box_paths && box_path_lens &&
        box_path_lens[box_idx] < PLANNER_V3_BFS_MAX_CELLS) {
      box_paths[box_idx][box_path_lens[box_idx]++] = box;
    }

    *car_pos = box_old_pos;
    if (simulate_only) {
      (*out_steps)++;
    } else {
      if (*out_steps >= path_capacity) {
        return -7;
      }
      if (*out_steps > 0 &&
          !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], *car_pos)) {
        SET_ERR(2, 2);
        return -6;
      }
      path_buffer[(*out_steps)++] = *car_pos;
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

#undef SET_ERR
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

// 评分：车到箱子的 BFS 真实距离（按"到达该箱子任意推箱位"的最短距离计算）
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
    
    // 计算推完这一步后箱子的新位置
    int new_box_row = box.row + dirs[d][0];
    int new_box_col = box.col + dirs[d][1];
    
    // 检查新位置是否在边界内
    if (!planner_v3_bfs_in_bounds(rows, cols, new_box_row, new_box_col)) {
      continue;
    }
    
    // 检查新位置是否在障碍上
    if (planner_v3_bfs_is_obstacle(obstacles, obstacle_count, new_box_row, new_box_col)) {
      continue;
    }
    
    // 检查新位置是否在其他箱子上
    if (planner_v3_bfs_is_box_at(current_boxes, box_count, new_box_row, new_box_col, box_idx)) {
      continue;
    }
    
    // 检查新位置是否是死点
    if (planner_v3_bfs_is_deadlock(rows, cols, obstacles, obstacle_count, current_boxes, box_count,
                                   box_idx, new_box_row, new_box_col)) {
      // 死点，跳过这个方向
      continue;
    }

    int dist_map[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_target(rows, cols, push_pos, obstacles, obstacle_count,
                                               current_boxes, box_count, dist_map)) {
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

// 计算箱子到目标的BFS路径长度（忽略其他箱子）
// 返回值：路径长度（步数），如果不可达返回INT_MAX
static int planner_v3_bfs_compute_box_path_length(int rows, int cols, Point box_start,
                                                   Point target, const Point *obstacles,
                                                   size_t obstacle_count, const Point *boxes,
                                                   size_t box_count, size_t moving_idx) {
  // 构建临时箱子数组（忽略其他箱子）
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

  // 从目标点做全局BFS
  int dist[PLANNER_V3_BFS_MAX_CELLS];
  if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                             temp_boxes, temp_count, dist)) {
    return INT_MAX;
  }

  // 使用A*计算路径长度
  Point temp_path[PLANNER_V3_BFS_MAX_PATH_LEN];
  size_t path_len = 0;
  if (!planner_v3_bfs_astar_with_dist(rows, cols, box_start, target, obstacles, obstacle_count,
                                      temp_boxes, temp_count, dist,
                                      temp_path, PLANNER_V3_BFS_MAX_PATH_LEN, &path_len, 1)) {
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
                                                 box_count, obstacles, obstacle_count);
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

    for (size_t t = 0; t < target_count; ++t) {
      if (target_used[t]) {
        continue;
      }
      // 计算箱子到目标的BFS路径长度（忽略其他箱子）
      int steps = planner_v3_bfs_compute_box_path_length(rows, cols, valid_boxes[bi],
                                                          targets[t], obstacles, obstacle_count,
                                                          boxes, box_count, source_indices[bi]);
      if (steps == INT_MAX) {
        continue;  // 不可达
      }
      if (steps < best_steps || (steps == best_steps && t < best_t)) {
        best_steps = steps;
        best_t = t;
      }
    }

    if (best_t == SIZE_MAX) {
      last_err_stage = 2;  // 动态选目标失败
      last_err_detail = 1; // 找不到可行目标
      return -6;  // 找不到可行目标
    }

    out_goals[assigned].box = valid_boxes[bi];
    out_goals[assigned].target = targets[best_t];
    out_goals[assigned].target_idx = best_t;
    out_goals[assigned].source_idx = source_indices[bi];

    if (out_mapping) {
      out_mapping[source_indices[bi]] = best_t;
    }

    target_used[best_t] = 1;
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
                                   size_t path_capacity, size_t *out_steps,
                                   PlannerAllBoxPaths *out_final_paths) {
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

  for (size_t i = 0; i < goal_count; ++i) {
    current_boxes[i] = goals[i].box;
  }

  // 初始化箱子路径记录
  Point box_paths[PLANNER_V3_BFS_MAX_BOXES][PLANNER_V3_BFS_MAX_CELLS];
  size_t box_path_lens[PLANNER_V3_BFS_MAX_BOXES] = {0};
  size_t box_path_starts[PLANNER_V3_BFS_MAX_BOXES] = {0};  // 记录每个箱子路径在path_buffer中的起始位置

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
        // 需要找到这个目标在goals数组中的位置
        for (size_t j = 0; j < goal_count; ++j) {
          if (goals[j].target.row == goals[i].target.row &&
              goals[j].target.col == goals[i].target.col) {
            target_used[j] = 1;
            break;
          }
        }
      }
    }
    
    // 收集未使用的目标（去重，因为可能有多个箱子指向同一个目标）
    for (size_t i = 0; i < goal_count; ++i) {
      if (target_used[i]) {
        continue;
      }
      // 检查是否已经添加到active_targets中（去重）
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
        active_targets, active_target_count, obstacles, obstacle_count,
        new_goals, &new_goal_count, temp_mapping);
    
    if (assign_res != 0 || new_goal_count == 0) {
      last_err_stage = 2;  // 动态选目标失败
      last_err_detail = 4; // 重规划次数过多（重新分配失败）
      return -6;  // 重新分配失败
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
          obstacles, obstacle_count);
      
      if (car_dist < best_car_dist) {
        best_car_dist = car_dist;
        selected_box_idx = i;
      }
    }
    
    if (selected_box_idx == SIZE_MAX) {
      last_err_stage = 2;  // 动态选目标失败
      last_err_detail = 3; // 无法找到候选箱子
      return -6;  // 无法选择箱子
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
      last_err_stage = 2;  // 动态选目标失败
      last_err_detail = 3; // 无法找到候选箱子（索引映射失败）
      return -6;
    }
    
    Point box = current_boxes[box_idx];
    Point target = new_goals[selected_box_idx].target;

    // 记录当前箱子路径的起始位置
    box_path_starts[box_idx] = *out_steps;
    // 记录箱子起始位置
    if (box_path_lens[box_idx] < PLANNER_V3_BFS_MAX_CELLS) {
      box_paths[box_idx][box_path_lens[box_idx]++] = box;
    }

    // 箱子已在目标点，标记为"消失"（后续路径规划中自动忽略）
    if (box.row == target.row && box.col == target.col) {
      current_boxes[box_idx].row = -1;
      current_boxes[box_idx].col = -1;
      // 记录箱子到达目标
      if (box_path_lens[box_idx] < PLANNER_V3_BFS_MAX_CELLS) {
        box_paths[box_idx][box_path_lens[box_idx]++] = target;
      }
      continue;
    }

    // 【核心修改】始终使用全局BFS+A*策略
    // 从目标点做全局BFS，计算每个格子到目标的真实距离
    int target_dist[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                           current_boxes, goal_count, target_dist)) {
      last_err_stage = 2;  // 动态选目标失败
      last_err_detail = 6; // 单箱推送全局BFS失败
      return -6;  // BFS失败
    }

    // 【新增】预先计算箱子的完整路径（从当前位置到目标）
    Point box_path[PLANNER_V3_BFS_MAX_CELLS];
    size_t box_path_len = 0;
    int has_box_path = 0;
    
    // 构建临时箱子数组（忽略其他箱子，因为我们要计算这个箱子的路径）
    Point temp_boxes[PLANNER_V3_BFS_MAX_BOXES];
    size_t temp_count = 0;
    for (size_t i = 0; i < goal_count; ++i) {
      if (i == box_idx) {
        continue;  // 忽略正在移动的箱子
      }
      if (current_boxes[i].row < 0 || current_boxes[i].col < 0) {
        continue;  // 忽略已完成的箱子
      }
      temp_boxes[temp_count++] = current_boxes[i];
    }
    
    // 从目标点做全局BFS（不考虑当前箱子）
    int box_path_dist[PLANNER_V3_BFS_MAX_CELLS];
    if (planner_v3_bfs_global_bfs_from_target(rows, cols, target, obstacles, obstacle_count,
                                             temp_boxes, temp_count, box_path_dist)) {
      // 使用A*计算箱子路径
      if (planner_v3_bfs_astar_with_dist(rows, cols, box, target, obstacles, obstacle_count,
                                         temp_boxes, temp_count, box_path_dist,
                                         box_path, PLANNER_V3_BFS_MAX_CELLS, &box_path_len, 1)) {
        if (box_path_len > 1) {
          has_box_path = 1;  // 有有效路径（至少包含起点和终点）
        }
      }
    }
    
    // 仿真两种策略：按路径/按评分推到终点，比较车步数
    size_t path_sim_steps = SIZE_MAX;
    int path_sim_ok = 0;
    if (has_box_path) {
      Point sim_car = current_car;
      Point sim_boxes[PLANNER_V3_BFS_MAX_BOXES];
      for (size_t i = 0; i < goal_count; ++i) {
        sim_boxes[i] = current_boxes[i];
      }
      size_t sim_steps = 0;
      int sim_res = planner_v3_bfs_push_box_with_strategy(
          rows, cols, &sim_car, sim_boxes, goal_count, box_idx, target, obstacles,
          obstacle_count, target_dist, box_path, box_path_len, has_box_path,
          PLANNER_V3_BFS_STRATEGY_PATH, 1, NULL, 0, &sim_steps, NULL, NULL, 0);
      if (sim_res == 0) {
        path_sim_ok = 1;
        path_sim_steps = sim_steps;
      }
    }

    size_t score_sim_steps = SIZE_MAX;
    int score_sim_ok = 0;
    {
      Point sim_car = current_car;
      Point sim_boxes[PLANNER_V3_BFS_MAX_BOXES];
      for (size_t i = 0; i < goal_count; ++i) {
        sim_boxes[i] = current_boxes[i];
      }
      size_t sim_steps = 0;
      int sim_res = planner_v3_bfs_push_box_with_strategy(
          rows, cols, &sim_car, sim_boxes, goal_count, box_idx, target, obstacles,
          obstacle_count, target_dist, box_path, box_path_len, has_box_path,
          PLANNER_V3_BFS_STRATEGY_SCORE, 1, NULL, 0, &sim_steps, NULL, NULL, 0);
      if (sim_res == 0) {
        score_sim_ok = 1;
        score_sim_steps = sim_steps;
      }
    }

    if (!path_sim_ok && !score_sim_ok) {
      last_err_stage = 2;
      last_err_detail = 7;
      return -6;
    }

    int force_score = 0;
    int use_path = path_sim_ok && (!score_sim_ok || path_sim_steps <= score_sim_steps);
    if (force_score && score_sim_ok) {
      use_path = 0;
    }
    int run_res = planner_v3_bfs_push_box_with_strategy(
        rows, cols, &current_car, current_boxes, goal_count, box_idx, target, obstacles,
        obstacle_count, target_dist, box_path, box_path_len, has_box_path,
        use_path ? PLANNER_V3_BFS_STRATEGY_PATH : PLANNER_V3_BFS_STRATEGY_SCORE,
        0, path_buffer, path_capacity, out_steps, box_paths, box_path_lens, 1);
    if (run_res != 0) {
      return run_res;
    }

    box = current_boxes[box_idx];
    
    // 箱子到达目标，标记为"消失"（-1坐标表示不再参与路径规划）
    current_boxes[box_idx].row = -1;
    current_boxes[box_idx].col = -1;
    completed_count++;
    
    // 记录箱子到达目标（用于路径输出）
    if (box_path_lens[box_idx] < PLANNER_V3_BFS_MAX_CELLS) {
      box_paths[box_idx][box_path_lens[box_idx]++] = target;
    }
  }

  // 验证生成的路径是否连续
  if (*out_steps > 1 && !planner_v3_bfs_validate_continuous_path(path_buffer, *out_steps)) {
    last_err_stage = 2;  // 动态选目标失败
    last_err_detail = 2; // 推箱后车位置不连续（最终路径验证失败）
    return -6;  // 路径不连续，视为规划失败
  }

  // 推完所有箱子后，使用BFS+A*返回起点
  // 此时所有箱子都已到达目标（标记为-1），不再作为障碍物
  if (current_car.row != car_start.row || current_car.col != car_start.col) {
    // 构建空的箱子数组（所有箱子已到达目标，不再作为障碍）
    Point empty_boxes[PLANNER_V3_BFS_MAX_BOXES];
    size_t empty_box_count = 0;
    
    // 从起点做全局BFS（不考虑箱子，因为都已到达目标）
    int dist[PLANNER_V3_BFS_MAX_CELLS];
    if (!planner_v3_bfs_global_bfs_from_target(rows, cols, car_start, obstacles, obstacle_count,
                                               empty_boxes, empty_box_count, dist)) {
      // BFS失败，但不算错误，继续
    } else {
      // 使用A*搜索返回路径（不考虑箱子）
      Point return_path[256];
      size_t return_len = 0;
      if (planner_v3_bfs_astar_with_dist(rows, cols, current_car, car_start,
                                        obstacles, obstacle_count,
                                        empty_boxes, empty_box_count, dist,
                                        return_path, 256, &return_len, 0)) {
        // 将返回路径添加到输出（跳过起点，因为起点是当前车位置）
        for (size_t i = 1; i < return_len; ++i) {
          if (*out_steps >= path_capacity) {
            return -7;  // 路径缓存不足
          }
          // 验证与前一个点相邻
          if (*out_steps > 0 && 
              !planner_v3_bfs_check_adjacent(path_buffer[*out_steps - 1], return_path[i])) {
            last_err_stage = 2;  // 动态选目标失败
            last_err_detail = 2; // 推箱后车位置不连续（返回路径不连续）
            return -6;  // 返回路径不连续
          }
          path_buffer[(*out_steps)++] = return_path[i];
        }
      }
    }
  }

  // 填充out_final_paths
  if (out_final_paths) {
    out_final_paths->box_count = goal_count;
    for (size_t i = 0; i < goal_count; ++i) {
      out_final_paths->box_paths[i].valid = (box_path_lens[i] > 0) ? 1 : 0;
      out_final_paths->box_paths[i].path_len = box_path_lens[i];
      size_t copy_len = (box_path_lens[i] < 400) ? box_path_lens[i] : 400;
      for (size_t j = 0; j < copy_len; ++j) {
        out_final_paths->box_paths[i].path[j] = box_paths[i][j];
      }
    }
  }

  return 0;
}

int plan_boxes_greedy_v3_bfs(int rows, int cols, PlannerPointV3_BFS car,
                         const PlannerPointV3_BFS *boxes, size_t box_count,
                         const PlannerPointV3_BFS *targets, size_t target_count,
                         const PlannerPointV3_BFS *obstacles,
                         size_t obstacle_count, PlannerPointV3_BFS *path_buffer,
                         size_t path_capacity, size_t *out_steps,
                         size_t *out_box_target_indices,
                         PlannerAllBoxPaths *out_final_paths) {
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
      rows, cols, car, boxes, box_count, targets, target_count, obstacles, obstacle_count,
      assigned, &assigned_count, out_box_target_indices);
  if (assign_res != 0) {
    *out_steps = 0;
    return assign_res;
  }

  return planner_v3_bfs_run_assigned(rows, cols, car, assigned, assigned_count,
                                 obstacles, obstacle_count, path_buffer,
                                 path_capacity, out_steps, out_final_paths);
}
