#ifndef ASSIGNED_BOX_PLANNER_GREEDY_3_H
#define ASSIGNED_BOX_PLANNER_GREEDY_3_H

#include <stddef.h>
#include "assigned_box_planner_greedy.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef Point PlannerPointV3_Bomb;

/**
 * 炸弹推箱规划算法 v3_Bomb - 带炸弹优化的版本
 * 
 * 算法核心：
 * - 基于 v3_BFS 算法，增加炸弹机制
 * - 炸弹可以被推到障碍物上，炸毁该障碍及其上下左右相邻障碍
 * - 识别"关键炸段"：同方向连续的障碍组，推荐炸其中心位置
 * - 枚举不同的炸弹使用方案（包括不用炸弹）
 * - 对每种方案计算路径，选择最短路径
 * 
 * 算法流程：
 * 1. 先计算不使用炸弹的基准路径长度
 * 2. 识别地图上所有"关键炸段"（同方向连续障碍组）
 * 3. 对每个炸弹，枚举可能的炸段目标
 * 4. 枚举不同的执行顺序（炸弹在箱子之前/之间/之后）
 * 5. 对每种方案：
 *    a. 将炸弹作为特殊箱子加入规划
 *    b. 炸弹"到达"目标后，更新障碍物列表
 *    c. 使用更新后的地图继续推箱子
 * 6. 比较所有方案，返回最优路径
 *
 * @param rows/cols                 网格尺寸
 * @param car                       小车初始坐标
 * @param boxes/box_count           箱子列表（最多10个）/ 箱子数量
 * @param targets/target_count      目标点列表 / 目标点数量
 * @param bombs/bomb_count          炸弹列表（最多5个）/ 炸弹数量
 * @param obstacles/obstacle_count  障碍物列表 / 数量
 * @param path_buffer/path_capacity 输出路径缓存 / 容量
 * @param out_steps                 实际路径步数
 * @param out_box_target_indices    箱子到目标的映射（可选，传NULL则不输出）
 * @param out_used_bombs            使用的炸弹索引列表（可选，传NULL则不输出）
 * @param out_used_bomb_count       实际使用的炸弹数量
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
 *         -9 炸弹数超限(5)
 */
int plan_boxes_with_bombs_v3(int rows, int cols, PlannerPointV3_Bomb car,
                              const PlannerPointV3_Bomb *boxes, size_t box_count,
                              const PlannerPointV3_Bomb *targets, size_t target_count,
                              const PlannerPointV3_Bomb *bombs, size_t bomb_count,
                              const PlannerPointV3_Bomb *obstacles, size_t obstacle_count,
                              PlannerPointV3_Bomb *path_buffer, size_t path_capacity,
                              size_t *out_steps, size_t *out_box_target_indices,
                              size_t *out_used_bombs, size_t *out_used_bomb_count);

#ifdef __cplusplus
}
#endif

#endif

