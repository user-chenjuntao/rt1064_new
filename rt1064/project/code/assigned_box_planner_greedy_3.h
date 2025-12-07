#ifndef ASSIGNED_BOX_PLANNER_GREEDY_3_H
#define ASSIGNED_BOX_PLANNER_GREEDY_3_H

#include <stddef.h>
#include "assigned_box_planner_greedy.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef Point PlannerPointV3_Bomb;

/**
 * 炸弹推箱规划算法 v3_Bomb - 带炸弹优化的版本（基于动态贪心）
 * 
 * 算法核心：
 * - 基于 v3_BFS 动态贪心算法，增加炸弹机制
 * - 炸弹可以被推到障碍物上，炸毁该障碍及其上下左右相邻障碍
 * - 识别"关键炸段"：同方向连续的障碍组，推荐炸其中心位置
 * - 枚举不同的炸弹使用方案，比较baseline（不用炸弹）和炸弹方案
 * - 优先选择无错误的方案，在都成功时选择步数更少的方案
 * - 【继承特性】底层使用动态贪心：推完一个箱子/炸弹后重新选择下一个
 * - 【继承优化】底层考虑最佳推箱位：计算车到推箱位的真实距离而非车到箱子距离
 * 
 * 算法流程：
 * 1. 先计算不使用炸弹的baseline路径（底层使用推箱位优化的动态贪心）
 * 2. 识别地图上所有"关键炸段"（同方向连续障碍组）
 * 3. 对每个炸弹，枚举可能的炸段目标和插入位置
 *    - 距离剪枝：过滤距离过远的炸弹-炸段组合
 * 4. 对每种炸弹方案：
 *    a. 将炸弹插入到指定位置（在某些箱子之前/之后）
 *    b. 逐个推炸弹和箱子（底层自动应用推箱位优化）
 *    c. 炸弹爆炸后更新障碍物列表
 *    d. 推箱子时将其余箱子当作障碍物
 * 5. 比较baseline和所有炸弹方案：
 *    - 优先选择无错误的方案
 *    - 如果都成功，选择步数更少的
 * 6. 返回最优路径
 * 
 * 底层优化说明：
 * - 每次推箱子/炸弹时，底层算法会自动找到最佳推位
 * - 使用BFS计算车到推位的真实距离（而非曼哈顿距离）
 * - 考虑死点检测，过滤不可行的推位
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

