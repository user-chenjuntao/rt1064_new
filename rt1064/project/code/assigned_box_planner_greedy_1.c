#include "assigned_box_planner_greedy_1.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

#define PLANNER_V3_MAX_BOXES 10
#define PLANNER_V3_MAX_CELLS 400
#define PLANNER_V3_MAX_GREEDY_STEPS 5000
#define PLANNER_V3_MAX_PATH_LEN 5000
#define PLANNER_V3_LOOP_WINDOW 20
#define PLANNER_V3_REPEAT_THRESHOLD 3  // 触发绕行的重复次数阈值
#define PLANNER_V3_OBSTACLE_FOLLOW_STEPS 100  // 贴障碍绕行最大步数
#define PLANNER_V3_BOX_FOLLOW_STEPS 50  // 贴箱子绕行最大步数
#define PLANNER_V3_CAR_GREEDY_MAX_STEPS 200  // 车贪心移动最大步数

typedef struct {
  Point box;
  Point target;
  size_t target_idx;
  size_t source_idx;
} PlannerBoxGoal;

static int planner_v3_abs(int v) { return v >= 0 ? v : -v; }

static int planner_v3_manhattan(Point a, Point b) {
  return planner_v3_abs(a.row - b.row) + planner_v3_abs(a.col - b.col);
}

static int planner_v3_in_bounds(int rows, int cols, int row, int col) {
  return row >= 0 && row < rows && col >= 0 && col < cols;
}

static int planner_v3_is_obstacle(const Point *obstacles, size_t count, int row, int col) {
  for (size_t i = 0; i < count; ++i) {
    if (obstacles[i].row == row && obstacles[i].col == col) {
      return 1;
    }
  }
  return 0;
}

static int planner_v3_is_box_at(const Point *boxes, size_t count, int row, int col,
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
static int planner_v3_adjacent_blockers(int rows, int cols, const Point *obstacles,
                                   size_t obstacle_count, const Point *boxes,
                                   size_t box_count, size_t skip_idx, int row,
                                   int col) {
  int count = 0;
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (int i = 0; i < 4; ++i) {
    int nr = row + dirs[i][0];
    int nc = col + dirs[i][1];
    if (!planner_v3_in_bounds(rows, cols, nr, nc)) {
      count++;
      continue;
    }
    if (planner_v3_is_obstacle(obstacles, obstacle_count, nr, nc) ||
        planner_v3_is_box_at(boxes, box_count, nr, nc, skip_idx)) {
      count++;
    }
  }
  return count;
}

// 检查箱子从start是否能被推到target（不考虑死点，只检查物理可达性）
static int planner_v3_can_reach_goal(int rows, int cols, const Point *obstacles,
                                size_t obstacle_count, const Point *boxes,
                                size_t box_count, size_t moving_idx,
                                Point start, Point target) {
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_MAX_CELLS || total_cells <= 0) {
    return 0;
  }

  if (start.row == target.row && start.col == target.col) {
    return 1;
  }

  uint8_t visited[PLANNER_V3_MAX_CELLS];
  int queue[PLANNER_V3_MAX_CELLS];
  int head = 0;
  int tail = 0;

  memset(visited, 0, sizeof(visited));

  int start_idx = start.row * cols + start.col;
  queue[tail++] = start_idx;
  visited[start_idx] = 1;

  while (head < tail) {
    if (head >= PLANNER_V3_MAX_CELLS) {
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

      if (!planner_v3_in_bounds(rows, cols, new_row, new_col) ||
          !planner_v3_in_bounds(rows, cols, push_row, push_col)) {
        continue;
      }

      if (planner_v3_is_obstacle(obstacles, obstacle_count, new_row, new_col) ||
          planner_v3_is_obstacle(obstacles, obstacle_count, push_row, push_col)) {
        continue;
      }

      if (planner_v3_is_box_at(boxes, box_count, new_row, new_col, moving_idx) ||
          planner_v3_is_box_at(boxes, box_count, push_row, push_col, moving_idx)) {
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

// 贪心选择车的下一步移动方向（不推箱子）
static int planner_v3_greedy_car_move(int rows, int cols, Point car, Point target,
                                      const Point *obstacles, size_t obstacle_count,
                                      const Point *boxes, size_t box_count,
                                      Point *next_pos) {
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  int best_dist = INT_MAX;
  int best_dir = -1;
  
  for (int i = 0; i < 4; ++i) {
    int nr = car.row + dirs[i][0];
    int nc = car.col + dirs[i][1];
    
    if (!planner_v3_in_bounds(rows, cols, nr, nc)) {
      continue;
    }
    if (planner_v3_is_obstacle(obstacles, obstacle_count, nr, nc)) {
      continue;
    }
    if (planner_v3_is_box_at(boxes, box_count, nr, nc, box_count)) {
      continue;
    }
    
    int dist = planner_v3_manhattan((Point){nr, nc}, target);
    if (dist < best_dist) {
      best_dist = dist;
      best_dir = i;
    }
  }
  
  if (best_dir < 0) {
    return 0;
  }
  
  next_pos->row = car.row + dirs[best_dir][0];
  next_pos->col = car.col + dirs[best_dir][1];
  return 1;
}

// BFS绕行：在限定区域内使用BFS搜索路径
// allow_greedy_resume: 是否允许在绕行中途恢复贪心
//   = 1: 车辆绕行模式 - 允许中途检测并恢复贪心（通畅后立即恢复）
//   = 0: 箱子绕行模式 - 禁止中途打断，必须到达目标推箱位才停止
static int planner_v3_follow_obstacle_ex(int rows, int cols, Point start, Point target,
                                         const Point *obstacles, size_t obstacle_count,
                                         const Point *boxes, size_t box_count,
                                         int prefer_right, int allow_greedy_resume,
                                         Point *path, size_t path_cap,
                                         size_t *path_len) {
  (void)prefer_right;  // 新BFS实现不需要此参数
  
  if (start.row == target.row && start.col == target.col) {
    if (path_cap < 1) return 0;
    path[0] = start;
    *path_len = 1;
    return 1;
  }
  
  // 计算搜索区域：以起点到终点的行、列距离+1为基础
  int row_dist = planner_v3_abs(target.row - start.row);
  int col_dist = planner_v3_abs(target.col - start.col);
  
  // 基础距离 = 行距离+1 和 列距离+1
  int row_base = row_dist + 1;
  int col_base = col_dist + 1;
  
  // 如果其中一个为0，则以另一个距离来画区域
  if (col_dist == 0) {
    col_base = row_base;
  }
  if (row_dist == 0) {
    row_base = col_base;
  }
  
  // 计算起点到地图边界的距离，如果基础距离超出边界则以边界为准
  int dist_to_top = start.row;                    // 起点到上边界
  int dist_to_bottom = rows - 1 - start.row;      // 起点到下边界
  int dist_to_left = start.col;                   // 起点到左边界
  int dist_to_right = cols - 1 - start.col;       // 起点到右边界
  
  // 限制基础距离不超过边界
  if (row_base > dist_to_top) {
    row_base = dist_to_top;
  }
  if (row_base > dist_to_bottom) {
    row_base = dist_to_bottom;
  }
  if (col_base > dist_to_left) {
    col_base = dist_to_left;
  }
  if (col_base > dist_to_right) {
    col_base = dist_to_right;
  }
  
  // 确定矩形搜索区域的边界
  int min_row = start.row - row_base;
  int max_row = start.row + row_base;
  int min_col = start.col - col_base;
  int max_col = start.col + col_base;
  
  // 分区限制：以起点行坐标为中点
  // 确定终点在上半区还是下半区
  int target_in_upper = (target.row < start.row);
  
  // 另一半区只保留2格行距离
  if (target_in_upper) {
    // 终点在上半区，下半区限制到start.row + 2
    if (max_row > start.row + 2) {
      max_row = start.row + 2;
    }
  } else {
    // 终点在下半区或同行，上半区限制到start.row - 2
    if (min_row < start.row - 2) {
      min_row = start.row - 2;
    }
  }
  
  // 边界裁剪到网格范围内
  if (min_row < 0) min_row = 0;
  if (max_row >= rows) max_row = rows - 1;
  if (min_col < 0) min_col = 0;
  if (max_col >= cols) max_col = cols - 1;
  
  // BFS搜索
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_MAX_CELLS) {
    total_cells = PLANNER_V3_MAX_CELLS;
  }
  
  uint8_t visited[PLANNER_V3_MAX_CELLS];
  int parent[PLANNER_V3_MAX_CELLS];  // 记录父节点用于回溯路径
  int queue[PLANNER_V3_MAX_CELLS];
  
  memset(visited, 0, sizeof(visited));
  for (int i = 0; i < total_cells; ++i) {
    parent[i] = -1;
  }
  
  int head = 0;
  int tail = 0;
  
  int start_idx = start.row * cols + start.col;
  queue[tail++] = start_idx;
  visited[start_idx] = 1;
  
  int found_target = 0;
  int target_idx = target.row * cols + target.col;
  
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  
  while (head < tail && !found_target) {
    if (head >= PLANNER_V3_MAX_CELLS) break;
    
    int curr_idx = queue[head++];
    int curr_row = curr_idx / cols;
    int curr_col = curr_idx % cols;
    
    if (curr_idx == target_idx) {
      found_target = 1;
      break;
    }
    
    // 检查是否可以贪心恢复（仅当允许时）
    if (allow_greedy_resume) {
      Point curr_pos = {curr_row, curr_col};
      Point greedy_next;
      if (planner_v3_greedy_car_move(rows, cols, curr_pos, target, obstacles, obstacle_count,
                                     boxes, box_count, &greedy_next)) {
        int greedy_dist = planner_v3_manhattan(greedy_next, target);
        int curr_dist = planner_v3_manhattan(curr_pos, target);
        if (greedy_dist < curr_dist) {
          // 可以恢复贪心，回溯路径到当前位置，然后返回
          found_target = 1;
          target_idx = curr_idx;
          break;
        }
      }
    }
    
    for (int d = 0; d < 4; ++d) {
      int nr = curr_row + dirs[d][0];
      int nc = curr_col + dirs[d][1];
      
      // 检查是否在搜索区域内
      if (nr < min_row || nr > max_row || nc < min_col || nc > max_col) {
        continue;
      }
      
      // 检查是否在网格边界内
      if (!planner_v3_in_bounds(rows, cols, nr, nc)) {
        continue;
      }
      
      // 检查是否是障碍物或箱子
      if (planner_v3_is_obstacle(obstacles, obstacle_count, nr, nc)) {
        continue;
      }
      if (planner_v3_is_box_at(boxes, box_count, nr, nc, box_count)) {
        continue;
      }
      
      int next_idx = nr * cols + nc;
      if (next_idx < 0 || next_idx >= total_cells) {
        continue;
      }
      
      if (!visited[next_idx]) {
        visited[next_idx] = 1;
        parent[next_idx] = curr_idx;
        if (tail < PLANNER_V3_MAX_CELLS) {
          queue[tail++] = next_idx;
        }
      }
    }
  }
  
  if (!found_target) {
    *path_len = 0;
    return 0;
  }
  
  // 回溯路径
  int path_indices[PLANNER_V3_MAX_PATH_LEN];
  int path_count = 0;
  int idx = target_idx;
  
  while (idx != -1 && path_count < PLANNER_V3_MAX_PATH_LEN) {
    path_indices[path_count++] = idx;
    idx = parent[idx];
  }
  
  // 反转路径（从起点到终点）
  *path_len = 0;
  for (int i = path_count - 1; i >= 0 && *path_len < path_cap; --i) {
    int pidx = path_indices[i];
    path[*path_len].row = pidx / cols;
    path[*path_len].col = pidx % cols;
    (*path_len)++;
  }
  
  return (*path_len > 0) ? 1 : 0;
}

// 贴着障碍物绕行（包装函数，默认允许贪心恢复）
static int planner_v3_follow_obstacle(int rows, int cols, Point start, Point target,
                                      const Point *obstacles, size_t obstacle_count,
                                      const Point *boxes, size_t box_count,
                                      int prefer_right, Point *path, size_t path_cap,
                                      size_t *path_len) {
  return planner_v3_follow_obstacle_ex(rows, cols, start, target, obstacles, obstacle_count,
                                       boxes, box_count, prefer_right, 1,
                                       path, path_cap, path_len);
}

// 贴着箱子绕行到另一个推箱位
// 策略：先尝试车辆模式（允许贪心恢复），如果未到达推箱位则使用箱子模式（禁止贪心恢复）
static int planner_v3_follow_box_to_push_pos(int rows, int cols, Point car, Point box,
                                             Point target, const Point *obstacles,
                                             size_t obstacle_count, const Point *boxes,
                                             size_t box_count, size_t moving_idx,
                                             int prefer_right, Point *path,
                                             size_t path_cap, size_t *path_len,
                                             Point *out_push_pos) {
  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  
  // 先找到所有有效的推箱位，选择最近的一个作为目标
  Point valid_push_positions[4];
  int valid_count = 0;
  
  for (int d = 0; d < 4; ++d) {
    int push_r = box.row - dirs[d][0];
    int push_c = box.col - dirs[d][1];
    int new_box_r = box.row + dirs[d][0];
    int new_box_c = box.col + dirs[d][1];
    
    if (!planner_v3_in_bounds(rows, cols, push_r, push_c)) continue;
    if (!planner_v3_in_bounds(rows, cols, new_box_r, new_box_c)) continue;
    if (planner_v3_is_obstacle(obstacles, obstacle_count, push_r, push_c)) continue;
    if (planner_v3_is_obstacle(obstacles, obstacle_count, new_box_r, new_box_c)) continue;
    if (planner_v3_is_box_at(boxes, box_count, new_box_r, new_box_c, moving_idx)) continue;
    
    // 检查推过去后箱子能否继续到目标
    if (planner_v3_can_reach_goal(rows, cols, obstacles, obstacle_count, boxes,
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
  
  // 按距离排序推箱位（从近到远）
  typedef struct {
    Point pos;
    int dist;
  } PushPosWithDist;
  
  PushPosWithDist sorted_positions[4];
  for (int i = 0; i < valid_count; ++i) {
    sorted_positions[i].pos = valid_push_positions[i];
    sorted_positions[i].dist = planner_v3_manhattan(car, valid_push_positions[i]);
  }
  
  // 简单冒泡排序（最多4个元素）
  for (int i = 0; i < valid_count - 1; ++i) {
    for (int j = 0; j < valid_count - 1 - i; ++j) {
      if (sorted_positions[j].dist > sorted_positions[j + 1].dist) {
        PushPosWithDist tmp = sorted_positions[j];
        sorted_positions[j] = sorted_positions[j + 1];
        sorted_positions[j + 1] = tmp;
      }
    }
  }
  
  // 尝试所有有效推箱位，从最近的开始
  for (int try_idx = 0; try_idx < valid_count; ++try_idx) {
    Point target_push_pos = sorted_positions[try_idx].pos;
    
    // 先使用车辆模式：允许贪心恢复，提高效率
    int detour_ok = planner_v3_follow_obstacle_ex(rows, cols, car, target_push_pos,
                                                  obstacles, obstacle_count, boxes, box_count,
                                                  prefer_right, 1, path, path_cap, path_len);
    
    if (detour_ok && *path_len > 0) {
      Point last = path[*path_len - 1];
      if (last.row == target_push_pos.row && last.col == target_push_pos.col) {
        // 成功到达推箱位
        out_push_pos->row = target_push_pos.row;
        out_push_pos->col = target_push_pos.col;
        return 1;
      }
      // 车辆模式未到达推箱位，从起点重新尝试箱子模式
    }
    
    // 使用箱子模式：禁止贪心恢复，确保到达推箱位
    detour_ok = planner_v3_follow_obstacle_ex(rows, cols, car, target_push_pos,
                                              obstacles, obstacle_count, boxes, box_count,
                                              prefer_right, 0, path, path_cap, path_len);
    
    if (detour_ok && *path_len > 0) {
      Point last = path[*path_len - 1];
      if (last.row == target_push_pos.row && last.col == target_push_pos.col) {
        out_push_pos->row = target_push_pos.row;
        out_push_pos->col = target_push_pos.col;
        return 1;
      }
    }
    // 如果仍然失败，继续尝试下一个推箱位
  }
  
  // 所有有效推箱位都尝试过，仍然无法到达
  return 0;
}

// 验证路径连续性（每步曼哈顿距离为1）
static int planner_v3_validate_continuous_path(const Point *path, size_t len) {
  if (len < 2) {
    return 1;  // 0或1步总是有效
  }
  for (size_t i = 1; i < len; ++i) {
    int dr = planner_v3_abs(path[i].row - path[i-1].row);
    int dc = planner_v3_abs(path[i].col - path[i-1].col);
    if (dr + dc != 1) {
      return 0;  // 跳跃或对角线移动
    }
  }
  return 1;
}

// 验证新添加的点与前一个点是否相邻
static int planner_v3_check_adjacent(Point prev, Point curr) {
  int dr = planner_v3_abs(curr.row - prev.row);
  int dc = planner_v3_abs(curr.col - prev.col);
  return (dr + dc == 1);
}

// 车贪心移动到目标位置，检测路径重复
static int planner_v3_car_greedy_to_target(int rows, int cols, Point *car_pos, Point target,
                                           const Point *obstacles, size_t obstacle_count,
                                           const Point *boxes, size_t box_count,
                                           Point *path_buffer, size_t path_capacity,
                                           size_t *out_steps) {
  Point recent_positions[PLANNER_V3_LOOP_WINDOW];
  for (int i = 0; i < PLANNER_V3_LOOP_WINDOW; ++i) {
    recent_positions[i].row = -1;
    recent_positions[i].col = -1;
  }
  int pos_idx = 0;
  int steps = 0;
  
  // 验证起点：如果路径缓冲区已有内容，确保car_pos与最后一个点相邻
  if (*out_steps > 0) {
    Point last = path_buffer[*out_steps - 1];
    if (last.row != car_pos->row || last.col != car_pos->col) {
      if (!planner_v3_check_adjacent(last, *car_pos)) {
        return 0;  // 起点不连续
      }
    }
  }
  
  while (car_pos->row != target.row || car_pos->col != target.col) {
    if (steps++ > PLANNER_V3_CAR_GREEDY_MAX_STEPS) {
      return 0;  // 超时
    }
    
    // 记录位置
    recent_positions[pos_idx % PLANNER_V3_LOOP_WINDOW] = *car_pos;
    pos_idx++;
    
    // 检测重复
    int repeat_count = 0;
    for (int i = 0; i < PLANNER_V3_LOOP_WINDOW; ++i) {
      if (recent_positions[i].row == car_pos->row && 
          recent_positions[i].col == car_pos->col) {
        repeat_count++;
      }
    }
    
    if (repeat_count >= PLANNER_V3_REPEAT_THRESHOLD) {
      // 路径重复，触发绕行
      return 0;
    }
    
    Point next;
    if (!planner_v3_greedy_car_move(rows, cols, *car_pos, target, obstacles, 
                                    obstacle_count, boxes, box_count, &next)) {
      // 贪心失败，遇到障碍
      return 0;
    }
    
    if (*out_steps >= path_capacity) {
      return -7;  // 路径缓存不足
    }
    
    // 验证与前一个点相邻
    if (*out_steps > 0 && 
        !planner_v3_check_adjacent(path_buffer[*out_steps - 1], next)) {
      return 0;  // 贪心路径不连续，触发绕行
    }
    
    path_buffer[(*out_steps)++] = next;
    *car_pos = next;
  }
  
  return 1;  // 成功
}

static size_t planner_v3_collect_boxes(int rows, int cols, const Point *boxes,
                                       size_t box_count, Point *filtered,
                                       size_t *source_indices,
                                       size_t *mapping_out) {
  size_t kept = 0;
  for (size_t i = 0; i < box_count && kept < PLANNER_V3_MAX_BOXES; ++i) {
    if (mapping_out) {
      mapping_out[i] = SIZE_MAX;
    }
    Point p = boxes[i];
    if (p.row < 0 || p.col < 0) {
      continue;
    }
    if (!planner_v3_in_bounds(rows, cols, p.row, p.col)) {
      continue;
    }
    filtered[kept] = p;
    source_indices[kept] = i;
    kept++;
  }
  return kept;
}

// 为箱子分配目标点（贪心策略）
// 规则：
// 1. 每个箱子可以推向任意目标点（不是预先固定配对）
// 2. 使用贪心算法选择最优配对：车到箱子+箱子到目标的距离最小
// 3. 每个目标点只分配给一个箱子（通过target_used数组保证）
// 4. 箱子到达目标后消失，该目标点也不再被其他箱子使用
static int planner_v3_assign_targets(int rows, int cols, Point car,
                                     const Point *boxes, size_t box_count,
                                     const Point *targets, size_t target_count,
                                     PlannerBoxGoal *out_goals,
                                     size_t *out_goal_count,
                                     size_t *out_mapping) {
  if (!boxes || !targets || !out_goals || !out_goal_count) {
    return -1;
  }
  if (target_count == 0) {
    return -2;
  }
  if (target_count > PLANNER_V3_MAX_CELLS) {
    return -1;
  }

  Point valid_boxes[PLANNER_V3_MAX_BOXES];
  size_t source_indices[PLANNER_V3_MAX_BOXES];
  size_t usable = planner_v3_collect_boxes(rows, cols, boxes, box_count,
                                           valid_boxes, source_indices,
                                           out_mapping);
  if (usable == 0) {
    return -2;
  }
  if (target_count < usable) {
    return -8;
  }

  uint8_t target_used[PLANNER_V3_MAX_CELLS] = {0};
  uint8_t box_used[PLANNER_V3_MAX_BOXES] = {0};

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
      int car_to_box = planner_v3_manhattan(car, valid_boxes[bi]);
      for (size_t ti = 0; ti < target_count; ++ti) {
        if (target_used[ti]) {
          continue;
        }
        int box_to_goal = planner_v3_manhattan(valid_boxes[bi], targets[ti]);
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

// 主推箱规划函数（简化版：贪心 + 绕行）
static int planner_v3_run_assigned(int rows, int cols, Point car,
                                   const PlannerBoxGoal *goals,
                                   size_t goal_count, const Point *obstacles,
                                   size_t obstacle_count, Point *path_buffer,
                                   size_t path_capacity, size_t *out_steps) {
  if (!goals || !path_buffer || !out_steps) {
    return -1;
  }
  if (goal_count == 0) {
    return -2;
  }
  if (goal_count > PLANNER_V3_MAX_BOXES) {
    return -3;
  }
  if (path_capacity == 0) {
    return -4;
  }

  *out_steps = 0;

  Point current_car = car;
  Point current_boxes[PLANNER_V3_MAX_BOXES];

  for (size_t i = 0; i < goal_count; ++i) {
    current_boxes[i] = goals[i].box;
  }

  int picked[PLANNER_V3_MAX_BOXES] = {0};

  // 按贪心顺序处理每个箱子
  for (size_t order_idx = 0; order_idx < goal_count; ++order_idx) {
    size_t box_idx = SIZE_MAX;
    int best_score = INT_MAX;
    for (size_t i = 0; i < goal_count; ++i) {
      if (picked[i]) {
        continue;
      }
      int dist_car = planner_v3_manhattan(current_car, current_boxes[i]);
      int dist_goal = planner_v3_manhattan(current_boxes[i], goals[i].target);
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

    // 推箱子到目标
    int step_count = 0;
    int last_dr = 0;
    int last_dc = 0;
    int reverse_count = 0;  // 连续反向移动次数
    
    Point recent_positions[PLANNER_V3_LOOP_WINDOW];
    for (int i = 0; i < PLANNER_V3_LOOP_WINDOW; ++i) {
      recent_positions[i].row = -1;
      recent_positions[i].col = -1;
    }
    int pos_idx = 0;

    while (box.row != target.row || box.col != target.col) {
      if (step_count++ >= PLANNER_V3_MAX_GREEDY_STEPS) {
        return -5;
      }

      // 记录箱子位置
      recent_positions[pos_idx % PLANNER_V3_LOOP_WINDOW] = box;
      pos_idx++;

      // 检测箱子位置重复
      int box_repeat = 0;
      for (int rp = 0; rp < PLANNER_V3_LOOP_WINDOW; ++rp) {
        if (recent_positions[rp].row == box.row &&
            recent_positions[rp].col == box.col) {
          box_repeat++;
        }
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

        if (!planner_v3_in_bounds(rows, cols, new_box_row, new_box_col) ||
            !planner_v3_in_bounds(rows, cols, push_row, push_col)) {
          continue;
        }

        if (planner_v3_is_obstacle(obstacles, obstacle_count, new_box_row, new_box_col) ||
            planner_v3_is_obstacle(obstacles, obstacle_count, push_row, push_col)) {
          continue;
        }

        if (planner_v3_is_box_at(current_boxes, goal_count, new_box_row, new_box_col,
                      box_idx) ||
            planner_v3_is_box_at(current_boxes, goal_count, push_row, push_col,
                      box_idx)) {
          continue;
        }

        // 特殊处理：如果推到目标点，跳过死点检测
        if (new_box_row == target.row && new_box_col == target.col) {
          // 推到目标点，不需要死点检测
        } else {
          // 非目标点，进行严格的死点检测
          if (!planner_v3_can_reach_goal(rows, cols, obstacles, obstacle_count, 
                                         current_boxes, goal_count, box_idx,
                                         (Point){new_box_row, new_box_col}, target)) {
            continue;
          }
        }

        Point push_from = {push_row, push_col};
        
        int dist_after = planner_v3_manhattan((Point){new_box_row, new_box_col}, target);
        int adj_pen = planner_v3_adjacent_blockers(
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
        
        // 新增：考虑车到推箱位的距离（权重较小，避免过度影响箱子到目标的距离）
        int car_to_push = planner_v3_manhattan(current_car, push_from);

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
        // 无可行方向，需要绕箱子换推箱位
        Point detour_path[256];
        size_t detour_len = 0;
        Point new_push_pos;
        
        int detour_ok = planner_v3_follow_box_to_push_pos(
            rows, cols, current_car, box, target, obstacles, obstacle_count,
            current_boxes, goal_count, box_idx, 1, detour_path, 256, 
            &detour_len, &new_push_pos);
        
        if (!detour_ok || detour_len == 0) {
          detour_ok = planner_v3_follow_box_to_push_pos(
              rows, cols, current_car, box, target, obstacles, obstacle_count,
              current_boxes, goal_count, box_idx, 0, detour_path, 256, 
              &detour_len, &new_push_pos);
        }
        
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
              !planner_v3_check_adjacent(path_buffer[*out_steps - 1], detour_path[i])) {
            return -6;  // 绕箱路径不连续
          }
          path_buffer[(*out_steps)++] = detour_path[i];
        }
        
        // 验证路径最后一个点是否是目标推箱位
        if (detour_len > 0) {
          Point last = detour_path[detour_len - 1];
          if (last.row != new_push_pos.row || last.col != new_push_pos.col) {
            // 路径不一致，这是内部错误
            return -6;
          }
          current_car = new_push_pos;
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
          // 检查路径缓冲区的最后一个点是否是当前车位置
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
              if (!planner_v3_check_adjacent(last, current_car)) {
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
            // 如果last就是current_car，不需要添加
          }
          // 车已在推箱位，直接成功
          chosen = candidate;
          push_success = 1;
          break;
        }
        
        // 尝试贪心移动车到推箱位
        int car_result = planner_v3_car_greedy_to_target(
            rows, cols, &current_car, candidate.push_from, obstacles, obstacle_count,
            current_boxes, goal_count, path_buffer, path_capacity, out_steps);
        
        if (car_result == 0) {
          // 贪心失败（遇障碍或重复），使用绕行
          current_car = car_before;
          *out_steps = steps_before;
          
          Point detour_path[256];
          size_t detour_len = 0;
          int detour_ok = 0;
          
          // 第一次尝试：车辆模式（允许贪心恢复，提高效率）
          detour_ok = planner_v3_follow_obstacle_ex(
              rows, cols, current_car, candidate.push_from, obstacles, obstacle_count,
              current_boxes, goal_count, 1, 1, detour_path, 256, &detour_len);
          
          // 验证是否到达推箱位
          if (detour_ok && detour_len > 0) {
            Point last = detour_path[detour_len - 1];
            if (last.row != candidate.push_from.row || last.col != candidate.push_from.col) {
              // 车辆模式未到达推箱位，从起点重新尝试箱子模式
              detour_ok = 0;
            }
          }
          
          // 第二次尝试：箱子模式（禁止贪心恢复，确保到达推箱位）
          if (!detour_ok || detour_len == 0) {
            detour_ok = planner_v3_follow_obstacle_ex(
                rows, cols, current_car, candidate.push_from, obstacles, obstacle_count,
                current_boxes, goal_count, 1, 0, detour_path, 256, &detour_len);
          }
          
          // 第三次尝试：换个方向的箱子模式
          if (!detour_ok || detour_len == 0) {
            detour_ok = planner_v3_follow_obstacle_ex(
                rows, cols, current_car, candidate.push_from, obstacles, obstacle_count,
                current_boxes, goal_count, 0, 0, detour_path, 256, &detour_len);
          }
          
          if (!detour_ok || detour_len == 0) {
            // 这个方向失败，回退状态，尝试下一个方向
            current_car = car_before;
            *out_steps = steps_before;
            continue;
          }
          
          // 执行绕行路径（跳过起点，因为起点是当前车位置）
          for (size_t i = 1; i < detour_len; ++i) {
            if (*out_steps >= path_capacity) {
              return -7;
            }
            // 验证与前一个点相邻
            if (*out_steps > 0 && 
                !planner_v3_check_adjacent(path_buffer[*out_steps - 1], detour_path[i])) {
              // 路径不连续，回退状态，尝试下一个方向
              current_car = car_before;
              *out_steps = steps_before;
              continue;
            }
            path_buffer[(*out_steps)++] = detour_path[i];
          }
          if (detour_len > 0) {
            current_car = detour_path[detour_len - 1];
          }
          
          // 绕行可能在中途停止，继续贪心到达推箱位
          if (current_car.row != candidate.push_from.row || 
              current_car.col != candidate.push_from.col) {
            int resume_result = planner_v3_car_greedy_to_target(
                rows, cols, &current_car, candidate.push_from, obstacles, obstacle_count,
                current_boxes, goal_count, path_buffer, path_capacity, out_steps);
            if (resume_result == -7) {
              return -7;
            }
            if (resume_result == 0 || 
                (current_car.row != candidate.push_from.row || 
                 current_car.col != candidate.push_from.col)) {
              // 这个方向失败，回退状态，尝试下一个方向
              current_car = car_before;
              *out_steps = steps_before;
              continue;
            }
          }
        } else if (car_result == -7) {
          return -7;  // 路径缓存不足
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
      int dist_to_box = planner_v3_manhattan(current_car, box);
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
          !planner_v3_check_adjacent(path_buffer[*out_steps - 1], current_car)) {
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
  if (*out_steps > 1 && !planner_v3_validate_continuous_path(path_buffer, *out_steps)) {
    return -6;  // 路径不连续，视为规划失败
  }

  return 0;
}

int plan_boxes_greedy_v3(int rows, int cols, PlannerPointV3 car,
                         const PlannerPointV3 *boxes, size_t box_count,
                         const PlannerPointV3 *targets, size_t target_count,
                         const PlannerPointV3 *obstacles,
                         size_t obstacle_count, PlannerPointV3 *path_buffer,
                         size_t path_capacity, size_t *out_steps,
                         size_t *out_box_target_indices) {
  if (!boxes || !targets || !path_buffer || !out_steps) {
    return -1;
  }
  *out_steps = 0;
  if (box_count == 0 || target_count == 0) {
    return -2;
  }
  if (box_count > PLANNER_V3_MAX_BOXES) {
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

  PlannerBoxGoal assigned[PLANNER_V3_MAX_BOXES];
  size_t assigned_count = 0;
  int assign_res = planner_v3_assign_targets(
      rows, cols, car, boxes, box_count, targets, target_count, assigned,
      &assigned_count, out_box_target_indices);
  if (assign_res != 0) {
    *out_steps = 0;
    return assign_res;
  }

  return planner_v3_run_assigned(rows, cols, car, assigned, assigned_count,
                                 obstacles, obstacle_count, path_buffer,
                                 path_capacity, out_steps);
}
