#ifndef ASSIGNED_BOX_PLANNER_GREEDY_H
#define ASSIGNED_BOX_PLANNER_GREEDY_H

#include <stddef.h>
#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int8_t row;
  int8_t col;
} Point;

typedef struct {
  Point box;
  Point target;
} BoxTargetPair;

/**
 * 贪心推箱规划：逐箱逐步推，按距离+邻域拥挤度+反向惩罚评分，兼顾绕障。
 *
 * @param rows/cols        网格尺寸
 * @param car              小车初始坐标
 * @param pairs/pair_count 箱子-目标列表
 * @param obstacles/count  障碍列表
 * @param path_buffer      输出的小车行走路径（不含箱子轨迹）
 * @param path_capacity    路径缓存可容纳的步数
 * @param out_steps        实际写入的步数
 *
 * @return 0 成功；
 *         -1 参数为空；
 *         -2 没有箱子；
 *         -3 箱子数量超上限(10)；
 *         -4 路径缓存容量为0；
 *         -5 单箱迭代超步数；
 *         -6 当前箱子无可行推动方向；
 *         -7 路径缓存不足。
 */
int plan_boxes_greedy(int rows, int cols, Point car, const BoxTargetPair *pairs,
                      size_t pair_count, const Point *obstacles,
                      size_t obstacle_count, Point *path_buffer,
                      size_t path_capacity, size_t *out_steps);

#ifdef __cplusplus
}
#endif

#endif
