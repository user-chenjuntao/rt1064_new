#ifndef ASSIGNED_BOX_PLANNER_GREEDY_1_H
#define ASSIGNED_BOX_PLANNER_GREEDY_1_H

#include <stddef.h>
#include "assigned_box_planner_greedy.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef Point PlannerPointV3;

/**
 * 推箱规划算法 v3 - 简化版：贪心 + 绕行
 * 
 * 算法流程：
 * 1. 按贪心顺序选择箱子推行顺序（距离最近优先）
 * 2. 对每个箱子：
 *    a. 车到推箱位：先贪心移动，遇障碍/重复则贴障碍绕行
 *    b. 推箱子：贪心选择方向（最小化到目标距离），检查死点
 *    c. 遇到死点：贴箱子绕行到另一个推箱位
 * 3. 取消拐点最小化和路径简化算法
 *
 * @param rows/cols                 网格尺寸
 * @param car                       小车初始坐标
 * @param boxes/box_count           箱子列表（最多10个）/ 箱子数量
 * @param targets/target_count      目标点列表 / 目标点数量
 * @param obstacles/count           障碍物列表 / 数量
 * @param path_buffer/path_capacity 输出路径缓存 / 容量
 * @param out_steps                 实际路径步数
 * @param out_box_target_indices    箱子到目标的映射（可选，传NULL则不输出），未分配为SIZE_MAX
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
int plan_boxes_greedy_v3(int rows, int cols, PlannerPointV3 car,
                         const PlannerPointV3 *boxes, size_t box_count,
                         const PlannerPointV3 *targets, size_t target_count,
                         const PlannerPointV3 *obstacles,
                         size_t obstacle_count, PlannerPointV3 *path_buffer,
                         size_t path_capacity, size_t *out_steps,
                         size_t *out_box_target_indices);

#ifdef __cplusplus
}
#endif

#endif
