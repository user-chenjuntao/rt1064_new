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

// 贴着障碍物绕行（右手法则或左手法则）
// allow_greedy_resume: 是否允许在绕行中途恢复贪心
//   = 1: 车辆绕行模式 - 允许中途检测并恢复贪心（通畅后立即恢复）
//   = 0: 箱子绕行模式 - 禁止中途打断，必须到达目标推箱位才停止
static int planner_v3_follow_obstacle_ex(int rows, int cols, Point start, Point target,
                                         const Point *obstacles, size_t obstacle_count,
                                         const Point *boxes, size_t box_count,
                                         int prefer_right, int allow_greedy_resume,
                                         Point *path, size_t path_cap,
                                         size_t *path_len) {
  if (start.row == target.row && start.col == target.col) {
    if (path_cap < 1) return 0;
    path[0] = start;
    *path_len = 1;
    return 1;
  }
  
  const int dirs[4][2] = {{-1, 0}, {0, 1}, {1, 0}, {0, -1}};  // 上右下左
  Point cur = start;
  size_t steps = 0;
  path[steps++] = cur;
  
  int current_dir = 0;  // 初始朝上
  uint8_t visited[PLANNER_V3_MAX_CELLS] = {0};
  int total_cells = rows * cols;
  if (total_cells > PLANNER_V3_MAX_CELLS) {
    total_cells = PLANNER_V3_MAX_CELLS;
  }
  
  for (int iter = 0; iter < PLANNER_V3_OBSTACLE_FOLLOW_STEPS && steps < path_cap; ++iter) {
    // 检查是否可以直接贪心到达目标（仅当允许时）
    if (allow_greedy_resume) {
      Point greedy_next;
      if (planner_v3_greedy_car_move(rows, cols, cur, target, obstacles, obstacle_count,
                                     boxes, box_count, &greedy_next)) {
        int greedy_dist = planner_v3_manhattan(greedy_next, target);
        int cur_dist = planner_v3_manhattan(cur, target);
        if (greedy_dist < cur_dist) {
          // 可以继续贪心，停止绕行（不添加贪心点，由调用方继续）
          *path_len = steps;
          return 1;
        }
      }
    }
    
    // 贴障碍绕行：尝试右手法则或左手法则
    int tried_dirs = 0;
    int found = 0;
    
    while (tried_dirs < 4) {
      int try_dir = prefer_right ? (current_dir + 4 - 1) % 4 : (current_dir + 1) % 4;
      
      for (int rot = 0; rot < 4; ++rot) {
        int test_dir = (try_dir + (prefer_right ? rot : (4 - rot))) % 4;
        int nr = cur.row + dirs[test_dir][0];
        int nc = cur.col + dirs[test_dir][1];
        
        if (planner_v3_in_bounds(rows, cols, nr, nc) &&
            !planner_v3_is_obstacle(obstacles, obstacle_count, nr, nc) &&
            !planner_v3_is_box_at(boxes, box_count, nr, nc, box_count)) {
          cur.row = nr;
          cur.col = nc;
          current_dir = test_dir;
          found = 1;
          break;
        }
      }
      
      if (found) break;
      tried_dirs++;
    }
    
    if (!found) {
      *path_len = steps;
      return 0;
    }
    
    if (steps < path_cap) {
      path[steps++] = cur;
    }
    
    if (cur.row == target.row && cur.col == target.col) {
      *path_len = steps;
      return 1;
    }
  }
  
  *path_len = steps;
  return 0;
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
// 注意：整个绕行过程不会被贪心打断，必须完整到达有效的推箱位才返回
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
    
    // 使用贴障碍绕行到目标推箱位，禁止中途贪心恢复
    int detour_ok = planner_v3_follow_obstacle_ex(rows, cols, car, target_push_pos,
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
    // 如果失败，继续尝试下一个推箱位
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

        // 特殊情况：如果新位置就是目标点，简化检查逻辑
        // 问题：箱子在目标点旁边，推最后一步时可能被错误阻止
        // 原因：原来的代码对目标点也做障碍/箱子检测，但目标点：
        //   1. 不应该是障碍（如果是，那是用户输入错误）
        //   2. 即使有其他箱子，那个箱子应该已"消失"（标记为-1）
        //   3. 不需要死点检测（已经是终点）
        // 解决：只检查推箱位，不检查目标点本身
        if (new_box_row == target.row && new_box_col == target.col) {
          // 目标点不检查障碍，只检查推箱位是否可达
          if (planner_v3_is_obstacle(obstacles, obstacle_count, push_row, push_col)) {
            continue;  // 推箱位是障碍，无法推
          }
          if (planner_v3_is_box_at(current_boxes, goal_count, push_row, push_col, box_idx)) {
            continue;  // 推箱位有其他箱子
          }
          // 推箱位OK，允许推进目标点（跳过死点检测和目标点检查）
        } else {
          // 非目标点，正常检查
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

          // 检查是否会死锁（非目标点才需要检查）
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

        int reverse_pen = (last_dr == -dirs[i][0] && last_dc == -dirs[i][1]) ? 50 : 0;

        int score = dist_after + adj_pen * 5 + reverse_pen;
        
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
          
          int detour_ok = planner_v3_follow_obstacle(
              rows, cols, current_car, candidate.push_from, obstacles, obstacle_count,
              current_boxes, goal_count, 1, detour_path, 256, &detour_len);
          
          if (!detour_ok || detour_len == 0) {
            detour_ok = planner_v3_follow_obstacle(
                rows, cols, current_car, candidate.push_from, obstacles, obstacle_count,
                current_boxes, goal_count, 0, detour_path, 256, &detour_len);
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
