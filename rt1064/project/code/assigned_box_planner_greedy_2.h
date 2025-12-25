#ifndef ASSIGNED_BOX_PLANNER_GREEDY_2_H
#define ASSIGNED_BOX_PLANNER_GREEDY_2_H

#include <stddef.h>
#include "assigned_box_planner_greedy.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef Point PlannerPointV3_BFS;

#define PLANNER_V3_BFS_MAX_CHAIN_LEN 10
#define PLANNER_V3_BFS_MAX_CHAIN_COUNT 10
#define PLANNER_V3_BFS_MAX_BOXES 10
#define PLANNER_V3_BFS_MAX_CELLS 400
#define PLANNER_V3_BFS_MAX_PATH_LEN PLANNER_V3_BFS_MAX_CELLS

extern int last_err_stage;   // 错误阶段：1=初始分配失败, 2=动态选目标失败, 3=第二次规划失败, 4=车辆路径连续性失败, 5=链式推动内部失败
extern int last_err_detail;  // 错误详情：对应阶段的子错误码

/**
 * 链式箱子结构 - 存储主箱到链尾的所有箱子
 * 链的顺序: indices[0]=主箱, indices[1]=第一个副箱, ..., indices[count-1]=链尾副箱
 */
typedef struct {
  size_t indices[PLANNER_V3_BFS_MAX_CHAIN_LEN];       // 链中箱子的索引，从主箱开始
  size_t overlap_lens[PLANNER_V3_BFS_MAX_CHAIN_LEN];  // 每个箱子的重叠路径长度
  size_t steps_walked[PLANNER_V3_BFS_MAX_CHAIN_LEN];  // 每个箱子已走的步数
  size_t count;                                        // 链的长度（包含主箱）
  Point dir;                                           // 当前推动方向
} PlannerBoxChain;

/**
 * 箱子重叠信息结构 - 用于存储主箱-副箱的重叠关系
 */
typedef struct {
  size_t primary;                    // 主箱索引
  size_t primary_start;             // 主箱路径中重叠起始位置
  size_t secondary_start;           // 副箱路径中重叠起始位置
  size_t overlap_len;               // 第一次路径规划的重叠长度（用于判断是否构成链式）
  size_t overlap_len_second;       // 第二次路径规划的重叠长度（用于检测何时路径不再重叠）
  Point overlap_end_pos;            // 第二次路径规划的重叠路径末端坐标（用于脱离检测）
  Point dir;                        // 推动方向
  int valid;                        // 是否有效
} PlannerBoxOverlap;

/**
 * 链式信息输出结构 - 用于输出所有检测到的链式组合
 */
typedef struct {
  size_t chain_indices[PLANNER_V3_BFS_MAX_CHAIN_COUNT][PLANNER_V3_BFS_MAX_CHAIN_LEN];  // 每个链的箱子索引
  size_t chain_lengths[PLANNER_V3_BFS_MAX_CHAIN_COUNT];  // 每个链的长度
  Point chain_overlap_end_pos[PLANNER_V3_BFS_MAX_CHAIN_COUNT][PLANNER_V3_BFS_MAX_CHAIN_LEN];  // 每个链中每个箱子的重叠末端坐标
  size_t chain_count;                                     // 检测到的链式总数
} PlannerChainInfo;

/**
 * 单个箱子的路径信息
 */
typedef struct {
  Point path[PLANNER_V3_BFS_MAX_PATH_LEN];  // 路径点序列
  size_t path_len;                           // 路径长度
  int valid;                                 // 路径是否有效
} PlannerBoxPathOutput;

/**
 * 所有箱子的路径信息输出结构
 */
typedef struct {
  PlannerBoxPathOutput box_paths[PLANNER_V3_BFS_MAX_BOXES];  // 每个箱子的路径
  size_t box_count;                                            // 箱子数量
} PlannerAllBoxPaths;

/**
 * 重构链状态信息结构 - 用于记录重构链的构建结果和执行结果
 */
typedef struct {
  int success;                    // 是否成功：1=成功, 0=失败
  int fail_step;                  // 失败步骤：1=第一次路径规划失败, 2=第二次路径规划失败, 3=无法构建链式, 0=成功
  size_t fail_box_idx;            // 失败的箱子索引（如果失败）
  Point fail_box_pos;             // 失败箱子的位置
  size_t detached_count;          // 脱离箱子数量
  size_t detached_indices[PLANNER_V3_BFS_MAX_BOXES];  // 脱离箱子索引列表
  int exec_success;               // 执行是否成功：1=链式推动执行成功, 0=执行失败或未执行, -1=未尝试执行
  int exec_res;                   // 执行结果：0=成功, 负数=失败错误码
  Point exec_fail_pos;            // 执行失败的位置（如果执行失败）
  size_t exec_fail_box_idx;       // 执行失败的箱子索引（如果执行失败）
} PlannerRebuildChainStatus;

/**
 * 推箱规划算法 v3_BFS - 纯 BFS+A* 版本（动态贪心选择 + 多副箱子链式推动）
 * 
 * 算法核心：
 * - 从终点做全局BFS，计算每个可达格子到终点的真实距离 dist[x][y]
 * - 使用A*搜索时，启发函数 h(x,y) = dist[x][y]
 * - 完全移除贪心算法和区域BFS
 * - 所有距离（箱子到目标、车到推箱位）都使用BFS真实距离
 * - 【新特性】动态选择：推完一个箱子后，从当前车位置重新选择下一个最优箱子-目标组合
 * - 【新特性】多副箱子链式推动：支持主箱推副箱1，副箱1推副箱2...的链式推动
 * 
 * 链式推动规则：
 * 1. 当主箱路径上有多个副箱子排列在同一方向时，构建完整链
 * 2. 直行时：推动主箱，链中所有箱子同时向推动方向移动一格
 * 3. 拐弯时：
 *    a. 车先去推链尾副箱朝新方向移动一格
 *    b. 车回到主箱推箱位，推主箱朝原方向移动一格（整个链同时移动）
 *    c. 此时原来的倒数第二个箱子到达拐弯点，成为新的链尾
 *    d. 重复a-c直到主箱到达拐弯点
 * 4. 如果链中有箱子到达目标点，则自动剔除该箱子，缩减链长度
 * 
 * 算法流程：
 * 1. 【动态选择】每轮从当前车位置，选择最优的箱子-目标组合
 * 2. 检测路径上的副箱子，构建完整链（可能包含多个副箱子）
 * 3. 使用链式推动执行推箱操作
 * 4. 推完后回到步骤1，重新选择下一个最优组合
 *
 * @param rows/cols                 网格尺寸
 * @param car                       小车初始坐标
 * @param boxes/box_count           箱子列表（最多10个）/ 箱子数量
 * @param targets/target_count      目标点列表 / 目标点数量
 * @param obstacles/count           障碍物列表 / 数量
 * @param path_buffer/path_capacity 输出路径缓存 / 容量
 * @param out_steps                 实际路径步数
 * @param out_box_target_indices    箱子到目标的映射（可选，传NULL则不输出），未分配为SIZE_MAX
 * @param out_chain_info            链式信息输出（可选，传NULL则不输出）
 * @param out_first_paths           第一次路径规划（忽略箱子阻挡）的所有箱子路径（可选，传NULL则不输出）
 * @param out_final_paths           最终路径规划（考虑箱子阻挡）的所有箱子路径（可选，传NULL则不输出）
 * @param out_overlaps               重叠信息输出（可选，传NULL则不输出），数组大小应为 box_count
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
                             PlannerChainInfo *out_chain_info,
                             PlannerAllBoxPaths *out_first_paths,
                             PlannerAllBoxPaths *out_final_paths,
                             PlannerBoxOverlap *out_overlaps);

#ifdef __cplusplus
}
#endif

#endif
