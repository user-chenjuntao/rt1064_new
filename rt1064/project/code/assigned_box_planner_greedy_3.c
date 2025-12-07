#include "assigned_box_planner_greedy_3.h"
#include "assigned_box_planner_greedy_2.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

#define MAX_BOMBS 5
#define MAX_BOMB_SEGMENTS 20
#define MAX_OBSTACLES_V3 200
#define MAX_BOMB_PLANS 100

// 优化参数：限制计算量
#define MAX_SEGMENTS_TO_TRY 5      // 最多尝试前5个高价值炸段
#define MAX_INSERT_POSITIONS 2     // 最多尝试2个插入位置（开头和结尾）
#define MAX_BOMB_PLAN_ATTEMPTS 10  // 最多尝试10个炸弹方案

// 静态全局缓冲区（避免栈溢出）
static Point g_baseline_path[2048];
static Point g_test_path[2048];
static Point g_temp_path[512];
static Point g_current_obstacles[MAX_OBSTACLES_V3];
static Point g_new_obstacles[MAX_OBSTACLES_V3];

// 炸段结构：同方向连续的障碍组
typedef struct {
  Point obstacles[10];  // 该炸段包含的障碍位置
  size_t count;         // 障碍数量
  Point center;         // 炸段中心（炸弹目标位置）
  int direction;        // 0=水平, 1=垂直
} BombSegment;

// 炸弹使用方案
typedef struct {
  size_t bomb_idx;      // 使用的炸弹索引
  size_t segment_idx;   // 目标炸段索引
  size_t insert_pos;    // 在推箱顺序中的插入位置（0=最先，box_count=最后）
} BombPlan;

// 执行方案（完整的执行序列）
typedef struct {
  BombPlan plans[MAX_BOMBS];
  size_t plan_count;
  size_t path_steps;
  uint8_t used_bombs[MAX_BOMBS];
} ExecutionPlan;

// 工具函数
static int planner_v3_bomb_abs(int v) { return v >= 0 ? v : -v; }

static int planner_v3_bomb_manhattan(Point a, Point b) {
  return planner_v3_bomb_abs(a.row - b.row) + planner_v3_bomb_abs(a.col - b.col);
}

static int planner_v3_bomb_in_bounds(int rows, int cols, int row, int col) {
  return row >= 0 && row < rows && col >= 0 && col < cols;
}

static int planner_v3_bomb_is_obstacle(const Point *obstacles, size_t count, int row, int col) {
  for (size_t i = 0; i < count; ++i) {
    if (obstacles[i].row == row && obstacles[i].col == col) {
      return 1;
    }
  }
  return 0;
}

// 识别所有关键炸段（优化版：最多重叠1个障碍）
// 核心思想：
// 1. 识别连续障碍序列（水平和垂直）
// 2. 对于长度≥3的序列，每隔2个位置选一个炸点
// 3. 这样相邻炸段最多重叠1个障碍
static size_t planner_v3_bomb_identify_segments(int rows, int cols, 
                                                 const Point *obstacles, size_t obstacle_count,
                                                 BombSegment *segments, size_t segment_capacity) {
  if (!obstacles || !segments || segment_capacity == 0) {
    return 0;
  }
  
  uint8_t processed[MAX_OBSTACLES_V3] = {0};
  size_t segment_count = 0;
  
  // 水平方向扫描
  for (int r = 0; r < rows; ++r) {
    // 收集该行的所有障碍
    Point row_obstacles[MAX_OBSTACLES_V3];
    size_t row_indices[MAX_OBSTACLES_V3];
    size_t row_count = 0;
    
    for (size_t i = 0; i < obstacle_count; ++i) {
      if (obstacles[i].row == r) {
        row_obstacles[row_count] = obstacles[i];
        row_indices[row_count] = i;
        row_count++;
      }
    }
    
    // 按列排序（防止row_count为0/1时size_t下溢导致死循环）
    if (row_count > 1) {
      for (size_t i = 0; i + 1 < row_count; ++i) {
        for (size_t j = 0; j + 1 < row_count - i; ++j) {
          if (row_obstacles[j].col > row_obstacles[j + 1].col) {
            Point tmp = row_obstacles[j];
            row_obstacles[j] = row_obstacles[j + 1];
            row_obstacles[j + 1] = tmp;
            
            size_t tmp_idx = row_indices[j];
            row_indices[j] = row_indices[j + 1];
            row_indices[j + 1] = tmp_idx;
          }
        }
      }
    }
    
    // 识别连续序列并选择炸点
    size_t seq_start = 0;
    while (seq_start < row_count) {
      size_t seq_end = seq_start;
      
      // 找到连续序列的结束位置
      while (seq_end + 1 < row_count && 
             row_obstacles[seq_end + 1].col == row_obstacles[seq_end].col + 1) {
        seq_end++;
      }
      
      size_t seq_len = seq_end - seq_start + 1;
      
      // 如果序列长度≥3，选择炸点
      if (seq_len >= 3) {
        // 从索引1开始，每隔2个选一个（索引1, 3, 5...）
        for (size_t i = seq_start + 1; i <= seq_end && segment_count < segment_capacity; i += 2) {
          BombSegment seg = {0};
          seg.direction = 0;  // 水平
          seg.center = row_obstacles[i];
          
          // 记录这个炸点能炸掉的障碍（十字形5个位置）
          seg.obstacles[seg.count++] = row_obstacles[i];  // 中心
          
          // 左右（水平连续）
          if (i > seq_start) {
            seg.obstacles[seg.count++] = row_obstacles[i - 1];  // 左
          }
          if (i < seq_end) {
            seg.obstacles[seg.count++] = row_obstacles[i + 1];  // 右
          }
          
          // 上下（十字）
          for (size_t j = 0; j < obstacle_count; ++j) {
            if (obstacles[j].row == seg.center.row - 1 && 
                obstacles[j].col == seg.center.col) {
              seg.obstacles[seg.count++] = obstacles[j];  // 上
            }
            if (obstacles[j].row == seg.center.row + 1 && 
                obstacles[j].col == seg.center.col) {
              seg.obstacles[seg.count++] = obstacles[j];  // 下
            }
          }
          
          segments[segment_count++] = seg;
          processed[row_indices[i]] = 1;
        }
      }
      
      seq_start = seq_end + 1;
    }
  }
  
  // 垂直方向扫描
  for (int c = 0; c < cols; ++c) {
    // 收集该列的所有障碍
    Point col_obstacles[MAX_OBSTACLES_V3];
    size_t col_indices[MAX_OBSTACLES_V3];
    size_t col_count = 0;
    
    for (size_t i = 0; i < obstacle_count; ++i) {
      if (obstacles[i].col == c) {
        col_obstacles[col_count] = obstacles[i];
        col_indices[col_count] = i;
        col_count++;
      }
    }
    
    // 按行排序（防止col_count为0/1时size_t下溢导致死循环）
    if (col_count > 1) {
      for (size_t i = 0; i + 1 < col_count; ++i) {
        for (size_t j = 0; j + 1 < col_count - i; ++j) {
          if (col_obstacles[j].row > col_obstacles[j + 1].row) {
            Point tmp = col_obstacles[j];
            col_obstacles[j] = col_obstacles[j + 1];
            col_obstacles[j + 1] = tmp;
            
            size_t tmp_idx = col_indices[j];
            col_indices[j] = col_indices[j + 1];
            col_indices[j + 1] = tmp_idx;
          }
        }
      }
    }
    
    // 识别连续序列并选择炸点
    size_t seq_start = 0;
    while (seq_start < col_count) {
      size_t seq_end = seq_start;
      
      // 找到连续序列的结束位置
      while (seq_end + 1 < col_count && 
             col_obstacles[seq_end + 1].row == col_obstacles[seq_end].row + 1) {
        seq_end++;
      }
      
      size_t seq_len = seq_end - seq_start + 1;
      
      // 如果序列长度≥3，选择炸点（跳过已被水平方向处理的点）
      if (seq_len >= 3) {
        for (size_t i = seq_start + 1; i <= seq_end && segment_count < segment_capacity; i += 2) {
          // 如果该点已被水平方向处理，跳过（避免重复）
          if (processed[col_indices[i]]) {
            continue;
          }
          
          BombSegment seg = {0};
          seg.direction = 1;  // 垂直
          seg.center = col_obstacles[i];
          
          // 记录这个炸点能炸掉的障碍（十字形5个位置）
          seg.obstacles[seg.count++] = col_obstacles[i];  // 中心
          
          // 上下（垂直连续）
          if (i > seq_start) {
            seg.obstacles[seg.count++] = col_obstacles[i - 1];  // 上
          }
          if (i < seq_end) {
            seg.obstacles[seg.count++] = col_obstacles[i + 1];  // 下
          }
          
          // 左右（十字）
          for (size_t j = 0; j < obstacle_count; ++j) {
            if (obstacles[j].row == seg.center.row && 
                obstacles[j].col == seg.center.col - 1) {
              seg.obstacles[seg.count++] = obstacles[j];  // 左
            }
            if (obstacles[j].row == seg.center.row && 
                obstacles[j].col == seg.center.col + 1) {
              seg.obstacles[seg.count++] = obstacles[j];  // 右
            }
          }
          
          segments[segment_count++] = seg;
        }
      }
      
      seq_start = seq_end + 1;
    }
  }
  
  return segment_count;
}

// 炸弹爆炸：移除障碍及其相邻障碍
static size_t planner_v3_bomb_explode(const Point *obstacles, size_t obstacle_count,
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
  uint8_t remove[MAX_OBSTACLES_V3] = {0};
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

// 计算使用炸弹方案的路径
static int planner_v3_bomb_calc_plan_path(int rows, int cols, Point car,
                                           const Point *boxes, size_t box_count,
                                           const Point *targets, size_t target_count,
                                           const Point *bombs, size_t bomb_count,
                                           const Point *obstacles, size_t obstacle_count,
                                           const ExecutionPlan *plan,
                                           Point *path_buffer, size_t path_capacity,
                                           size_t *out_steps) {
  if (!plan || plan->plan_count == 0) {
    // 无炸弹方案，直接调用原算法
    return plan_boxes_greedy_v3_bfs(rows, cols, car, boxes, box_count,
                                    targets, target_count, obstacles, obstacle_count,
                                    path_buffer, path_capacity, out_steps, NULL);
  }
  
  // 构建任务列表：箱子 + 炸弹
  Point all_items[15];  // 最多10箱子+5炸弹
  Point all_targets[15];
  size_t all_count = 0;
  
  // 记录哪些是炸弹（用于后续判断是否需要更新障碍）
  uint8_t is_bomb[15] = {0};
  size_t bomb_plan_idx[15];
  
  size_t box_idx = 0;
  size_t bomb_plan_cursor = 0;
  
  // 按插入位置合并箱子和炸弹
  for (size_t pos = 0; pos <= box_count && all_count < 15; ++pos) {
    // 插入该位置的所有炸弹
    while (bomb_plan_cursor < plan->plan_count && 
           plan->plans[bomb_plan_cursor].insert_pos == pos) {
      if (all_count >= 15) break;
      
      size_t bi = plan->plans[bomb_plan_cursor].bomb_idx;
      size_t si = plan->plans[bomb_plan_cursor].segment_idx;
      
      // 这里需要传入炸段信息，暂时简化：直接使用炸弹初始位置
      // 实际应该根据segment_idx找到目标位置
      all_items[all_count] = bombs[bi];
      all_targets[all_count] = bombs[bi];  // 临时，需要外部传入炸段中心
      is_bomb[all_count] = 1;
      bomb_plan_idx[all_count] = bomb_plan_cursor;
      all_count++;
      bomb_plan_cursor++;
    }
    
    // 插入一个箱子
    if (box_idx < box_count && all_count < 15) {
      all_items[all_count] = boxes[box_idx];
      all_targets[all_count] = targets[box_idx];
      is_bomb[all_count] = 0;
      all_count++;
      box_idx++;
    }
  }
  
  // 动态障碍物列表（使用全局缓冲区）
  size_t current_obstacle_count = obstacle_count;
  if (obstacle_count > MAX_OBSTACLES_V3) {
    current_obstacle_count = MAX_OBSTACLES_V3;
  }
  memcpy(g_current_obstacles, obstacles, current_obstacle_count * sizeof(Point));
  
  // 逐个处理任务
  *out_steps = 0;
  Point current_car = car;
  
  for (size_t i = 0; i < all_count; ++i) {
    // 如果是炸弹，规划到炸段中心，然后更新障碍
    if (is_bomb[i]) {
      // 将炸弹推到目标位置（使用全局缓冲区）
      size_t temp_steps = 0;
      
      Point single_bomb = all_items[i];
      Point single_target = all_targets[i];
      
      int ret = plan_boxes_greedy_v3_bfs(rows, cols, current_car, 
                                          &single_bomb, 1,
                                          &single_target, 1,
                                          g_current_obstacles, current_obstacle_count,
                                          g_temp_path, 512, &temp_steps, NULL);
      if (ret != 0) {
        return ret;
      }
      
      // 复制路径
      for (size_t j = 0; j < temp_steps && *out_steps < path_capacity; ++j) {
        path_buffer[(*out_steps)++] = g_temp_path[j];
      }
      if (temp_steps > 0) {
        current_car = g_temp_path[temp_steps - 1];
      }
      
      // 炸弹爆炸，更新障碍物
      current_obstacle_count = planner_v3_bomb_explode(
          g_current_obstacles, current_obstacle_count, 
          single_target, g_new_obstacles, MAX_OBSTACLES_V3);
      memcpy(g_current_obstacles, g_new_obstacles, current_obstacle_count * sizeof(Point));
      
    } else {
      // 普通箱子（使用全局缓冲区）
      size_t temp_steps = 0;
      
      Point single_box = all_items[i];
      Point single_target = all_targets[i];
      
      // 将其余未处理的箱子当作障碍物
      Point temp_obstacles_with_boxes[MAX_OBSTACLES_V3];
      size_t temp_obstacle_count = current_obstacle_count;
      if (temp_obstacle_count > MAX_OBSTACLES_V3) {
        temp_obstacle_count = MAX_OBSTACLES_V3;
      }
      memcpy(temp_obstacles_with_boxes, g_current_obstacles, temp_obstacle_count * sizeof(Point));
      
      // 添加后续未处理的箱子（不是炸弹）作为障碍
      for (size_t j = i + 1; j < all_count && temp_obstacle_count < MAX_OBSTACLES_V3; ++j) {
        if (!is_bomb[j]) {
          temp_obstacles_with_boxes[temp_obstacle_count++] = all_items[j];
        }
      }
      
      int ret = plan_boxes_greedy_v3_bfs(rows, cols, current_car,
                                          &single_box, 1,
                                          &single_target, 1,
                                          temp_obstacles_with_boxes, temp_obstacle_count,
                                          g_temp_path, 512, &temp_steps, NULL);
      if (ret != 0) {
        return ret;
      }
      
      // 复制路径
      for (size_t j = 0; j < temp_steps && *out_steps < path_capacity; ++j) {
        path_buffer[(*out_steps)++] = g_temp_path[j];
      }
      if (temp_steps > 0) {
        current_car = g_temp_path[temp_steps - 1];
      }
    }
  }
  
  return 0;
}

// 主函数：枚举所有炸弹使用方案，选择最优路径
int plan_boxes_with_bombs_v3(int rows, int cols, PlannerPointV3_Bomb car,
                              const PlannerPointV3_Bomb *boxes, size_t box_count,
                              const PlannerPointV3_Bomb *targets, size_t target_count,
                              const PlannerPointV3_Bomb *bombs, size_t bomb_count,
                              const PlannerPointV3_Bomb *obstacles, size_t obstacle_count,
                              PlannerPointV3_Bomb *path_buffer, size_t path_capacity,
                              size_t *out_steps, size_t *out_box_target_indices,
                              size_t *out_used_bombs, size_t *out_used_bomb_count) {
  if (!boxes || !targets || !path_buffer || !out_steps) {
    return -1;
  }
  if (box_count == 0 || target_count == 0) {
    return -2;
  }
  if (box_count > 10) {
    return -3;
  }
  if (path_capacity == 0) {
    return -4;
  }
  if (bomb_count > MAX_BOMBS) {
    return -9;
  }
  
  *out_steps = 0;
  if (out_used_bomb_count) {
    *out_used_bomb_count = 0;
  }
  
  // 方案1：不使用炸弹的基准路径（使用全局缓冲区）
  size_t baseline_steps = 0;
  
  int baseline_ret = plan_boxes_greedy_v3_bfs(rows, cols, car, boxes, box_count,
                                               targets, target_count, obstacles, 
                                               obstacle_count, g_baseline_path, 2048,
                                               &baseline_steps, out_box_target_indices);
  
  // 记录最优方案信息
  size_t best_steps = SIZE_MAX;
  ExecutionPlan best_plan = {0};
  int best_has_plan = 0;           // 是否有任何成功的方案
  int best_uses_bomb = 0;          // 最优方案是否使用炸弹
  int baseline_success = (baseline_ret == 0);  // baseline是否成功
  
  if (baseline_success) {
    best_steps = baseline_steps;
    best_has_plan = 1;
    best_uses_bomb = 0;  // baseline不使用炸弹
  }
  
  // 如果没有炸弹，直接返回基准路径
  if (bomb_count == 0 || !bombs) {
    if (best_has_plan) {
      size_t copy_len = baseline_steps < path_capacity ? baseline_steps : path_capacity;
      memcpy(path_buffer, g_baseline_path, copy_len * sizeof(Point));
      *out_steps = copy_len;
      return 0;
    }
    return baseline_ret;
  }
  
  // 识别所有炸段
  BombSegment segments[MAX_BOMB_SEGMENTS];
  size_t segment_count = planner_v3_bomb_identify_segments(rows, cols, obstacles,
                                                            obstacle_count, segments,
                                                            MAX_BOMB_SEGMENTS);
  
  if (segment_count == 0) {
    // 没有可炸的炸段，返回基准路径
    if (best_has_plan) {
      size_t copy_len = baseline_steps < path_capacity ? baseline_steps : path_capacity;
      memcpy(path_buffer, g_baseline_path, copy_len * sizeof(Point));
      *out_steps = copy_len;
      return 0;
    }
    return baseline_ret;
  }
  
  // 用户要求：有炸弹就强制使用，不跳过
  
  // 优化：按炸段价值排序（能炸掉的障碍数量）
  typedef struct {
    size_t index;
    size_t value;  // 能炸掉的障碍数量
  } SegmentPriority;
  
  SegmentPriority sorted_segments[MAX_BOMB_SEGMENTS];
  for (size_t i = 0; i < segment_count; ++i) {
    sorted_segments[i].index = i;
    sorted_segments[i].value = segments[i].count;
  }
  
  // 按价值降序排序（冒泡排序）
  for (size_t i = 0; i < segment_count - 1; ++i) {
    for (size_t j = 0; j < segment_count - 1 - i; ++j) {
      if (sorted_segments[j].value < sorted_segments[j + 1].value) {
        SegmentPriority tmp = sorted_segments[j];
        sorted_segments[j] = sorted_segments[j + 1];
        sorted_segments[j + 1] = tmp;
      }
    }
  }
  
  // 用户要求：尝试所有炸段，不过度限制
  size_t total_attempts = 0;  // 记录尝试次数
  
  // 放宽限制，允许尝试更多方案
  size_t max_attempts = segment_count * 2 + 10;  // 动态上限：炸段数×2+10
  
  for (size_t bi = 0; bi < bomb_count; ++bi) {
    // 尝试所有炸段（用户要求不限制）
    for (size_t sorted_idx = 0; sorted_idx < segment_count; ++sorted_idx) {
      size_t si = sorted_segments[sorted_idx].index;
      
      // 检查炸段中心是否是障碍（安全检查）
      if (!planner_v3_bomb_is_obstacle(obstacles, obstacle_count, 
                                        segments[si].center.row, 
                                        segments[si].center.col)) {
        continue;  // 中心不是障碍，跳过
      }
      
      // 【优化】距离剪枝：考虑炸弹的最佳推位，而不是直接距离
      // 计算炸弹到炸段中心的基础距离（用于初步筛选）
      int bomb_to_target_dist = planner_v3_bomb_manhattan(bombs[bi], segments[si].center);
      
      // 动态阈值：允许一定的绕路空间
      int max_dist = (rows + cols) / 2 + 5;  // 适当放宽阈值
      
      // 初步距离剪枝（避免明显不合理的组合）
      if (bomb_to_target_dist > max_dist) {
        continue;  // 距离太远，跳过
      }
      
      // 尝试所有插入位置（0到box_count）
      for (size_t insert_pos = 0; insert_pos <= box_count; ++insert_pos) {
        
        // 限制总尝试次数（根据基准方案是否成功动态调整）
        if (++total_attempts > max_attempts) {
          goto finish_enumeration;  // 达到上限，停止枚举
        }
        ExecutionPlan test_plan = {0};
        test_plan.plan_count = 1;
        test_plan.plans[0].bomb_idx = bi;
        test_plan.plans[0].segment_idx = si;
        test_plan.plans[0].insert_pos = insert_pos;
        test_plan.used_bombs[bi] = 1;
        
        // 构建临时任务列表
        Point test_items[15];
        Point test_targets_list[15];
        size_t test_count = 0;
        uint8_t test_is_bomb[15] = {0};
        
        size_t box_cursor = 0;
        for (size_t pos = 0; pos <= box_count && test_count < 15; ++pos) {
          if (pos == insert_pos) {
            // 插入炸弹
            test_items[test_count] = bombs[bi];
            test_targets_list[test_count] = segments[si].center;
            test_is_bomb[test_count] = 1;
            test_count++;
          }
          if (box_cursor < box_count && test_count < 15) {
            test_items[test_count] = boxes[box_cursor];
            test_targets_list[test_count] = targets[box_cursor];
            test_is_bomb[test_count] = 0;
            test_count++;
            box_cursor++;
          }
        }
        
        // 计算这个方案的路径（使用全局缓冲区）
        size_t test_steps = 0;
        Point current_car_test = car;
        size_t current_obstacle_count_test = obstacle_count;
        if (obstacle_count > MAX_OBSTACLES_V3) {
          current_obstacle_count_test = MAX_OBSTACLES_V3;
        }
        memcpy(g_current_obstacles, obstacles, current_obstacle_count_test * sizeof(Point));
        
        int all_ok = 1;
        for (size_t ti = 0; ti < test_count; ++ti) {
          size_t temp_steps = 0;

          // 炸弹要落在障碍物中心，规划时需要临时移除该中心点，否则BFS认为目标格被占用
          const Point *obstacles_for_step = g_current_obstacles;
          size_t obstacle_count_for_step = current_obstacle_count_test;
          Point temp_obstacles[MAX_OBSTACLES_V3];
          if (test_is_bomb[ti]) {
            obstacle_count_for_step = 0;
            for (size_t k = 0; k < current_obstacle_count_test && obstacle_count_for_step < MAX_OBSTACLES_V3; ++k) {
              if (g_current_obstacles[k].row == test_targets_list[ti].row &&
                  g_current_obstacles[k].col == test_targets_list[ti].col) {
                continue;  // 允许炸弹占据此障碍格
              }
              temp_obstacles[obstacle_count_for_step++] = g_current_obstacles[k];
            }
            obstacles_for_step = temp_obstacles;
          } else {
            // 普通箱子：将其余未处理的箱子当作障碍物
            obstacle_count_for_step = 0;
            // 先复制当前障碍物
            for (size_t k = 0; k < current_obstacle_count_test && obstacle_count_for_step < MAX_OBSTACLES_V3; ++k) {
              temp_obstacles[obstacle_count_for_step++] = g_current_obstacles[k];
            }
            // 添加后续未处理的箱子（不是炸弹）作为障碍
            for (size_t j = ti + 1; j < test_count && obstacle_count_for_step < MAX_OBSTACLES_V3; ++j) {
              if (!test_is_bomb[j]) {
                temp_obstacles[obstacle_count_for_step++] = test_items[j];
              }
            }
            obstacles_for_step = temp_obstacles;
          }

          int ret = plan_boxes_greedy_v3_bfs(rows, cols, current_car_test,
                                              &test_items[ti], 1,
                                              &test_targets_list[ti], 1,
                                              obstacles_for_step, obstacle_count_for_step,
                                              g_temp_path, 512, &temp_steps, NULL);
          if (ret != 0) {
            all_ok = 0;
            break;
          }
          
          // 累加路径
          for (size_t j = 0; j < temp_steps && test_steps < 2048; ++j) {
            g_test_path[test_steps++] = g_temp_path[j];
          }
          if (temp_steps > 0) {
            current_car_test = g_temp_path[temp_steps - 1];
          }
          
          // 如果是炸弹，爆炸后更新障碍
          if (test_is_bomb[ti]) {
            current_obstacle_count_test = planner_v3_bomb_explode(
                g_current_obstacles, current_obstacle_count_test,
                test_targets_list[ti], g_new_obstacles, MAX_OBSTACLES_V3);
            memcpy(g_current_obstacles, g_new_obstacles, current_obstacle_count_test * sizeof(Point));
          }
        }
        
        if (all_ok && test_steps < best_steps) {
          best_steps = test_steps;
          best_plan = test_plan;
          best_has_plan = 1;
          best_uses_bomb = 1;  // 标记使用了炸弹
          
          // 保存最优路径（炸弹方案）
          size_t copy_len = test_steps < path_capacity ? test_steps : path_capacity;
          memcpy(path_buffer, g_test_path, copy_len * sizeof(Point));
          *out_steps = copy_len;
          
          // 记录使用的炸弹
          if (out_used_bombs && out_used_bomb_count) {
            *out_used_bomb_count = 0;
            for (size_t i = 0; i < bomb_count; ++i) {
              if (best_plan.used_bombs[i]) {
                out_used_bombs[(*out_used_bomb_count)++] = i;
              }
            }
          }
          
          // 早停机制：找到足够好的方案就停止
          // （用户要求尝试所有炸段，暂时禁用早停）
          // if (test_steps < 某个阈值) {
          //   goto finish_enumeration;
          // }
        }
      }
    }
  }
  
finish_enumeration:
  // 决策逻辑：
  // 1. 优先选择无错误的方案（baseline vs 炸弹方案）
  // 2. 如果都成功，选择步数少的
  // 3. 如果只有一个成功，选择成功的
  // 4. 如果都失败，返回baseline的错误码
  
  if (best_has_plan) {
    // 有至少一个成功的方案
    
    // 如果最优方案是baseline（不使用炸弹），需要复制baseline路径
    if (!best_uses_bomb) {
      size_t copy_len = baseline_steps < path_capacity ? baseline_steps : path_capacity;
      memcpy(path_buffer, g_baseline_path, copy_len * sizeof(Point));
      *out_steps = copy_len;
      if (out_used_bomb_count) {
        *out_used_bomb_count = 0;  // baseline不使用炸弹
      }
    }
    // 如果使用炸弹，路径已经在枚举时保存到path_buffer了（700-703行）
    
    return 0;
  }
  
  // 两种方案都失败，返回baseline的错误码
  return baseline_ret;
}

