#ifndef ASSIGNED_BOX_PLANNER_GREEDY_2_H
#define ASSIGNED_BOX_PLANNER_GREEDY_2_H

#include <stddef.h>
#include "assigned_box_planner_greedy.h"

#ifdef __cplusplus
extern "C" {
#endif

extern int last_err_stage;   // 错误阶段
extern int last_err_detail;  // 错误详情

typedef Point PlannerPointV3_BFS;



// 箱子路径输出结构
typedef struct {
  int valid;                    // 路径是否有效
  size_t path_len;              // 路径长度
  Point path[400];              // 箱子路径（最多400步）
  Point obstacle_to_clear;      // 特殊路径：需炸掉的目标障碍坐标（仅 special_paths 使用）
  int has_obstacle_to_clear;    // 特殊路径：是否有需炸掉的障碍
  Point bomb_exploded_at;       // 炸弹实际爆炸位置（炸掉的障碍坐标）
  int has_bomb_exploded;        // 是否使用了炸弹并已爆炸
  Point bomb_extra_exploded_at[5];  // 炸弹B/C等清除推位障碍时的爆炸位置（用于主菜单显示）
  int bomb_extra_exploded_count;    // 上述辅助炸弹爆炸数量，0..5
  Point bomb_on_path[5];           // 特殊路径经过的炸弹坐标（用于菜单标蓝）
  int bomb_on_path_count;          // 上述数量，0..5
} PlannerBoxPathOutput;

// 所有箱子的路径输出结构
typedef struct {
  size_t box_count;             // 箱子数量
  PlannerBoxPathOutput box_paths[10];  // 每个箱子的路径
} PlannerAllBoxPaths;


/**
 * 推箱规划算法 v3_BFS - 纯 BFS+A* 版本
 * 
 * 算法核心：
 * - 从终点做全局BFS，计算每个可达格子到终点的真实距离 dist[x][y]
 * - 使用A*搜索时，启发函数 h(x,y) = dist[x][y]
 * - 完全移除贪心算法和区域BFS
 * - 所有距离（箱子到目标、车到推箱位）都使用BFS真实距离
 * 
 * 算法流程：
 * 1. 按距离选择箱子推行顺序（车到箱子+箱子到目标距离最小优先）
 * 2. 对每个箱子：
 *    a. 从目标点做全局BFS，预计算 dist_target[x][y]（箱子到目标的真实距离）
 *    b. 推箱方向评分：
 *       - 箱子到目标距离：使用 dist_target[x][y]（BFS真实距离）
 *       - 车到推箱位距离：对每个推箱位做BFS，计算真实距离
 *       - 评分公式：dist_target×13 + 拥挤度×2 + 反向惩罚 + dist_car×5
 *    c. 车移动到推箱位：使用全局BFS+A*
 *    d. 遇到死点需换推箱位：使用全局BFS+A*绕箱子
 * 3. 所有路径搜索统一使用全局BFS+A*，无贪心，无区域限制
 *
 * @param rows/cols                 网格尺寸
 * @param car                       小车初始坐标
 * @param boxes/box_count           箱子列表（最多10个）/ 箱子数量
 * @param targets/target_count      目标点列表 / 目标点数量
 * @param obstacles/count           障碍物列表 / 数量
 * @param path_buffer/path_capacity 输出路径缓存 / 容量
 * @param out_steps                 实际路径步数
 * @param out_box_target_indices    箱子到目标的映射（可选，传NULL则不输出），未分配为SIZE_MAX
 * @param out_final_paths           最终路径规划的所有箱子路径（可选，传NULL则不输出）
 *
 * @return 0  成功
 *         -1 参数为空
 *         -2 箱子或目标为空
 *         -3 箱子数超限(10)
 *         -4 路径缓存为0
 *         -5 单箱迭代超步数
 *         -6 无可行推动方向
 *         -7 路径缓存不足
 *         -8 目标数少于箱子数
 */
int plan_boxes_greedy_v3_bfs(int rows, int cols, PlannerPointV3_BFS car,
                             const PlannerPointV3_BFS *boxes, size_t box_count,
                             const PlannerPointV3_BFS *targets, size_t target_count,
                             const PlannerPointV3_BFS *obstacles,
                             size_t obstacle_count, PlannerPointV3_BFS *path_buffer,
                             size_t path_capacity, size_t *out_steps,
                             size_t *out_box_target_indices,
                             PlannerAllBoxPaths *out_final_paths);

#ifdef __cplusplus
}
#endif

#endif
