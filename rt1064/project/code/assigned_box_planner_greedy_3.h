#ifndef ASSIGNED_BOX_PLANNER_GREEDY_3_H
#define ASSIGNED_BOX_PLANNER_GREEDY_3_H

#include <stddef.h>
#include <stdint.h>
#include "assigned_box_planner_greedy.h"
#include "assigned_box_planner_greedy_2.h"

#ifdef __cplusplus
extern "C" {
#endif

extern int last_err_stage;   // 错误阶段
extern int last_err_detail;  // 错误详情

/** 推箱/推炸弹时是否仅使用 BFS+A* 路径（方便配置）
 *  0 = 路径与评分取优（默认）：有路径则与评分策略比较步数，取少者
 *  1 = 仅用 BFS+A* 路径：只按 BFS+A* 算出的路径推动，无有效路径则直接失败 */
extern int planner_v3_push_only_bfs_astar_path;

// 特殊路径（不把炸弹当作障碍、允许经过一次障碍）的显示数据
// 用于在菜单中单独查看"特殊路径"，并保持每次规划后的最新结果
extern PlannerAllBoxPaths special_paths;

/**
 * 炸弹推箱规划算法 v3 - 基于 v2 的炸弹版本
 * 
 * 算法核心：
 * - 基础流程和 assigned_box_planner_greedy_2.c 一致
 * - 把炸弹看作障碍
 * - 分配目标完后，先使用 BFS+A* 给该箱子规划路径；规划过程中任何错误导致不成功则直接进行特殊路径计算
 * - 特殊路径计算时不需要把炸弹当作障碍，路径允许经过一次障碍（该障碍为需炸掉的障碍）
 * - 特殊路径计算出来后：先推炸弹。选炸弹：按「距特殊路径经过的障碍」BFS 距离升序；选目标：备选为「该障碍及其上下左右障碍」，按该炸弹到目标的 BFS 可达距离升序。
 *   先试最优目标，不行则试次优目标，直至所有目标；该炸弹都推不到则试次优炸弹，依次尝试所有炸弹。推炸弹时仍为 BFS+A* 与评分方式取步数少者；除目标障碍外其他障碍、其他炸弹和箱子均当作障碍。
 *   推完炸弹后进入推箱子环节（模拟 BFS+A* 路径 vs 评分方式推到目标点，取车步数少者；所有障碍、箱子、炸弹均考虑），推箱子环节失败则不再计算特殊路径，直接返回失败
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
 *         -9 炸弹数超限(5)
 */
int plan_boxes_greedy_v3(int rows, int cols, PlannerPointV3_BFS car,
                         const PlannerPointV3_BFS *boxes, size_t box_count,
                         const PlannerPointV3_BFS *targets, size_t target_count,
                         const PlannerPointV3_BFS *bombs, size_t bomb_count,
                         const PlannerPointV3_BFS *obstacles,
                         size_t obstacle_count, PlannerPointV3_BFS *path_buffer,
                         size_t path_capacity, size_t *out_steps,
                         size_t *out_box_target_indices,
                         PlannerAllBoxPaths *out_final_paths);

#ifdef __cplusplus
}
#endif

#endif
