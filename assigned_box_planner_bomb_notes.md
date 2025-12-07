# 带炸弹的推箱规划算法说明文档

## 文件说明
- `assigned_box_planner_greedy_3.h` - 头文件
- `assigned_box_planner_greedy_3.c` - 实现文件

## 版本修改记录

### 当前修改（版本 1.0 - 初始版本）
- 基于 `assigned_box_planner_greedy_2` 的纯BFS+A*算法，增加炸弹机制
- 实现关键炸段识别：连续3个及以上障碍，每隔2个位置选一个炸点，相邻炸段最多重叠1个障碍
- 枚举炸弹使用方案：哪个炸弹、炸哪个炸段、在第几个箱子前炸
- 炸弹爆炸影响：移除中心障碍及上下左右相邻的障碍
- 比较baseline和炸弹方案，选择步数最少的路径

### 当前修改（版本 1.1 - 栈溢出修复）
- 将所有大数组改为静态全局变量，避免栈溢出（从40KB栈占用降至<2KB）
- 全局变量：`g_baseline_path[2048]`, `g_test_path[2048]`, `g_temp_path[512]`, `g_current_obstacles[200]`, `g_new_obstacles[200]`
- 注意：全局变量不可重入，多任务环境需加锁保护

### 当前修改（版本 1.2 - 计算量优化）
- 添加多重优化限制：最多尝试5个高价值炸段、只尝试首尾2个插入位置、总尝试次数上限10次
- 添加早停机制：路径<100步直接返回baseline，炸弹方案比baseline好20%就停止枚举
- 按炸段价值（能炸掉的障碍数）排序，优先尝试高价值炸段
- 动态距离剪枝：炸弹到炸段距离 > (rows+cols)/2 则跳过

### 当前修改（版本 1.3 - 强制使用炸弹）
- 用户要求：有炸弹就必须使用，优先返回包含推炸弹路径的方案
- 移除"路径<100步直接返回baseline"的逻辑
- 修改返回优先级：优先返回炸弹方案，只有炸弹方案全部失败才返回baseline
- 尝试所有炸段（不再限制为5个）和所有插入位置（不再限制为首尾2个）
- 放宽尝试次数上限：从10次增加到 `segment_count × 2 + 10`
- 禁用早停机制，确保尝试完所有可能的方案

### 当前修改（版本 1.4 - 炸弹推到障碍物修复）
- 修复关键问题：推炸弹到障碍物中心时，临时移除该中心障碍点
- 原因：炸弹目标格本身是障碍，BFS会认为目标不可达
- 解决：在规划推炸弹路径时，从障碍物列表中临时排除炸弹目标格
- 爆炸后该障碍及其相邻障碍正常移除

### 当前修改（版本 1.5 - 推箱子时考虑其他箱子为障碍）
- 在炸弹爆炸后推某个箱子时，将其余未处理的箱子当作障碍物
- 避免路径规划时小车穿过其他箱子
- 同时应用于炸弹方案枚举测试中的箱子推送

### 当前修改（版本 1.6 - baseline与炸弹方案比较优化）
- 优先选择无错误的方案（baseline vs 炸弹方案）
- 如果都成功，选择步数少的
- 如果只有一个成功，选择成功的
- 如果都失败，返回baseline的错误码

### 当前修改（版本 1.7 - 继承底层动态贪心+推箱位优化）
- **底层优化自动继承**：底层 `plan_boxes_greedy_v3_bfs` 已升级为动态贪心+推箱位优化
- **动态选择**：每推完一个箱子/炸弹，底层会动态选择下一个最优箱子-目标组合
- **推箱位优化**：底层会找到箱子的最佳推箱位，计算车到推箱位的真实距离
- **距离剪枝优化**：炸弹距离剪枝阈值放宽（`max_dist = (rows+cols)/2 + 5`），适应绕路情况
- **无需额外修改**：炸弹版本的枚举逻辑保持不变，自动受益于底层改进

## 算法概述

基于 `assigned_box_planner_greedy_2` 的纯BFS+A*算法（v1.7 含动态贪心+推箱位优化），增加炸弹机制，用于优化推箱路径。

### 核心思想

1. **炸弹作用**：
   - 炸弹可以被推到障碍物上
   - 炸弹与障碍重合时，该障碍消失
   - 同时，与该障碍**上下左右**相邻的障碍也会消失
   - 炸弹本身也消失

2. **关键炸段识别**：
   - 扫描地图，找出**同方向连续**的障碍组
   - 水平方向：同一行连续的障碍
   - 垂直方向：同一列连续的障碍
   - 每组障碍记为一个"炸段"
   - 推荐炸段的**中心位置**作为炸弹目标

3. **方案枚举与比较**：
   - 计算**不使用炸弹**的baseline路径（使用底层的动态贪心+推箱位优化）
   - 枚举各种炸弹使用方案：
     - 使用哪个炸弹
     - 炸哪个炸段
     - 在什么时机炸（在第几个箱子之前）
   - 对每种方案计算完整路径（底层自动应用推箱位优化）
   - 比较baseline和炸弹方案：
     - 优先选择无错误的方案
     - 如果都成功，选择步数少的

4. **底层优化继承**（版本 1.7）：
   - **动态选择**：推箱子/炸弹时，底层动态选择最优箱子-目标组合
   - **推箱位优化**：底层找到最佳推箱位，计算车到推箱位的真实距离
   - **死点预检**：底层在选择时提前检测死点，过滤不可行推箱位
   - **全局BFS+A***：所有路径规划使用真实距离，无区域限制

## 算法流程（版本 1.4）

```
1. 输入：地图、箱子、目标、炸弹、障碍物

2. 计算基准路径（不使用炸弹，作为后备）
   └─> 调用 plan_boxes_greedy_v3_bfs，记录步数

3. 识别所有"关键炸段"
   ├─> 扫描水平方向：同一行连续≥3个障碍，每隔2个选1个炸点
   ├─> 扫描垂直方向：同一列连续≥3个障碍，每隔2个选1个炸点
   ├─> 按能炸掉的障碍数量降序排序
   └─> 记录每个炸段的中心位置和影响范围

4. 枚举所有炸弹使用方案（强制使用炸弹）
   for 每个炸弹 bi:
       for 每个炸段 si (全部尝试):
           检查炸段中心是否是障碍 ✓
           检查炸弹距离是否合理 ✓
           for 每个插入位置 pos (0到box_count):
               ├─> 构建任务序列（炸弹+箱子按位置混合）
               ├─> 逐个执行任务：
               │   ├─> 如果是箱子：
               │   │   调用 plan_boxes_greedy_v3_bfs 推到目标
               │   └─> 如果是炸弹：
               │       ├─> 临时移除炸弹目标格的障碍 ⚠️
               │       ├─> 调用 plan_boxes_greedy_v3_bfs 推到炸段中心
               │       ├─> 炸弹爆炸：移除中心及上下左右障碍
               │       └─> 更新障碍物列表，后续任务使用新地图
               ├─> 累加路径步数
               └─> 如果步数 < 当前最优，更新最优方案

5. 决策与返回（版本 1.6+）
   ├─> 如果有成功的方案：
   │   ├─> baseline成功 且 炸弹方案成功：返回步数少的
   │   ├─> 仅baseline成功：返回baseline
   │   └─> 仅炸弹方案成功：返回炸弹方案
   └─> 如果都失败：返回baseline的错误码

关键改进说明:
⚠️ v1.4: 推炸弹时临时移除目标障碍，否则BFS认为目标不可达
⚠️ v1.5: 推箱子时将其余箱子当作障碍物
⚡ v1.7: 底层自动应用推箱位优化和动态贪心选择
```

## 底层优化详解（版本 1.7）

### 推箱位优化原理

当选择箱子-目标组合时，不是简单计算"车到箱子"的距离，而是：

```
for 每个箱子-目标组合:
    # 找到该箱子的最佳推箱位
    best_push_dist = ∞
    
    for 推箱位 in [上, 下, 左, 右]:
        # 1. 检查合法性
        if 推箱位有障碍 or 推箱位有其他箱子:
            continue
        
        # 2. 检查推后位置
        推后箱子位置 = 箱子位置 + 推动方向
        if 推后位置有障碍 or 推后位置有其他箱子:
            continue
        
        # 3. 死点检测（关键！）
        if 推后位置 != 目标:
            if 箱子从推后位置无法到达目标:
                continue  # 这是一个死点，跳过
        
        # 4. 计算车到推箱位的真实距离（使用BFS）
        距离 = BFS(车位置 -> 推箱位, 考虑障碍和其他箱子)
        
        if 距离 < best_push_dist:
            best_push_dist = 距离
    
    # 5. 计算最终评分
    score = best_push_dist + 箱子到目标距离
```

### 优化效果对比

**场景示例**：
```
C: 车    B: 箱子    T: 目标    #: 障碍

  0 1 2 3 4
0 C . . . .
1 . . . . .
2 . . B # .
3 . . . . .
4 . . . . T
```

**旧方法（直接距离）**：
- 车到箱子距离 = Manhattan(C→B) = |2-0| + |2-2| = 2
- 箱子到目标距离 = Manhattan(B→T) = |2-4| + |2-4| = 4
- 评分 = 2 + 4 = 6

**新方法（推箱位优化）**：
- 推箱位候选：上(1,2)、下(3,2)、左(2,1)、右(2,3-被障碍物占据)
- 推箱位"右"不可行（有障碍）
- 车到推箱位距离：
  - 上(1,2): BFS(C→上) = 3
  - 下(3,2): BFS(C→下) = 5
  - 左(2,1): BFS(C→左) = 4
- 最佳推箱位 = 上(1,2)，距离 = 3
- 评分 = 3 + 4 = 7

**结果**：虽然评分略高，但选择的是**可行且合理**的推箱位，避免了选择被障碍阻挡的方向。

### 动态选择优势

**预分配 vs 动态选择**：

| 时机 | 预分配策略 | 动态选择策略 |
|------|-----------|-------------|
| 开始时 | 一次性分配所有箱子-目标映射 | 只选择第一个最优箱子-目标 |
| 推完箱子1后 | 按预定顺序推箱子2 | 从当前车位置重新选择最优 |
| 推完箱子2后 | 按预定顺序推箱子3 | 再次重新选择最优 |

**示例**：
```
初始: 车在左上，箱子A在左下，箱子B在右上
目标: 目标1在右下，目标2在左下

预分配: 
  分配 A→目标1, B→目标2
  车 → A → 目标1 (长距离移动)
  然后 → B → 目标2

动态选择:
  评估: B→目标2距离更近
  车 → B → 目标2 (短距离)
  然后 → A → 目标1 (车已在右侧，更近)
  
结果: 动态选择总路径更短
```

## 数据结构

### BombSegment（炸段）
```c
typedef struct {
  Point obstacles[10];  // 该炸段包含的障碍位置
  size_t count;         // 障碍数量
  Point center;         // 炸段中心（炸弹目标位置）
  int direction;        // 0=水平, 1=垂直
} BombSegment;
```

### BombPlan（单个炸弹使用计划）
```c
typedef struct {
  size_t bomb_idx;      // 使用的炸弹索引
  size_t segment_idx;   // 目标炸段索引
  size_t insert_pos;    // 在推箱顺序中的插入位置
} BombPlan;
```

### ExecutionPlan（完整执行方案）
```c
typedef struct {
  BombPlan plans[MAX_BOMBS];
  size_t plan_count;
  size_t path_steps;
  uint8_t used_bombs[MAX_BOMBS];
} ExecutionPlan;
```

## 核心函数说明

### 1. `planner_v3_bomb_identify_segments`
**功能**：识别地图上所有的关键炸段

**逻辑（v1.0优化版）**：
- 按行扫描，识别水平连续障碍序列（≥3个）
  - 对每个序列，从索引1开始，每隔2个选一个炸点（索引1, 3, 5...）
  - 例如：A-B-C-D-E → 选B和D（B炸ABC，D炸CDE，重叠C）
- 按列扫描，识别垂直连续障碍序列（≥3个）
  - 跳过已被水平方向处理的点
  - 同样每隔2个选一个炸点
- 计算每个炸点能炸掉的障碍：
  - 中心障碍 + 上下左右4个相邻障碍（十字形）
- 返回所有炸段，相邻炸段最多重叠1个障碍

### 2. `planner_v3_bomb_explode`
**功能**：模拟炸弹爆炸，生成新的障碍物列表

**逻辑**：
1. 找到炸弹位置对应的障碍（中心障碍）
2. 如果没找到，返回原障碍列表（炸空了）
3. 标记要移除的障碍：
   - 中心障碍
   - 中心的上下左右4个相邻障碍
4. 复制未被移除的障碍到新列表

**示例**：
```
原始地图：
. . . . .
. # # # .
. # O # .    O = 炸弹目标（中心障碍）
. # # # .
. . . . .

爆炸后：
. . . . .
. # . # .
. . . . .    中心和上下左右都消失
. # . # .
. . . . .
```

### 3. `plan_boxes_with_bombs_v3`（主规划函数）
**功能**：枚举所有炸弹方案并返回最优路径（优先返回炸弹方案）

**关键修复（v1.4）**：
推炸弹到障碍物中心时的处理：
```c
// 问题：炸弹目标格本身是障碍，BFS认为目标不可达
// 解决：临时移除炸弹目标格的障碍
if (test_is_bomb[ti]) {
  // 构建临时障碍物列表，排除炸弹目标格
  for (size_t k = 0; k < current_obstacle_count_test; ++k) {
    if (g_current_obstacles[k] == test_targets_list[ti]) {
      continue;  // 跳过炸弹目标障碍
    }
    temp_obstacles[new_count++] = g_current_obstacles[k];
  }
  obstacles_for_step = temp_obstacles;
}

// 使用临时障碍物列表规划推炸弹路径
plan_boxes_greedy_v3_bfs(..., obstacles_for_step, ...);

// 炸弹到位后，正常执行爆炸，移除中心及相邻障碍
```

**参数**：
- `rows, cols` - 地图尺寸
- `car` - 小车初始位置
- `boxes, box_count` - 箱子列表（最多10个）
- `targets, target_count` - 目标点列表
- `bombs, bomb_count` - 炸弹列表（最多5个）
- `obstacles, obstacle_count` - 障碍物列表
- `path_buffer, path_capacity` - 输出路径缓冲区
- `out_steps` - 输出实际步数
- `out_box_target_indices` - 输出箱子到目标的映射（可选）
- `out_used_bombs` - 输出使用的炸弹索引列表（可选）
- `out_used_bomb_count` - 输出使用的炸弹数量

**返回值**：
- `0` - 成功（优先返回炸弹方案，路径包含推炸弹步骤）
- `-1` - 参数为空
- `-2` - 箱子或目标为空
- `-3` - 箱子数超限（>10）
- `-4` - 路径缓存为0
- `-5` - 单箱迭代超步数
- `-6` - 无可行推动方向（所有方案都失败）
- `-7` - 路径缓存不足
- `-8` - 目标数少于箱子数
- `-9` - 炸弹数超限（>5）

**返回优先级（v1.3+）**：
1. 优先返回炸弹方案（即使比baseline长，也优先用炸弹）
2. 只有所有炸弹方案失败时，才返回baseline
3. 如果两者都失败，返回错误码-6

**输出路径**：
- 包含推炸弹的完整路径：`[推炸弹步骤] + [推箱子步骤]`
- `out_used_bomb_count` 会指示使用的炸弹数量
- `out_used_bombs[]` 包含使用的炸弹索引

## 关键修复说明（v1.4）

### 问题描述
在v1.3及之前版本中，推炸弹到障碍物中心时会失败，因为：
- 炸弹的目标位置本身是障碍物
- BFS算法认为障碍物格子不可通行
- 导致无法规划出推炸弹的路径，返回失败

### 解决方案
在规划推炸弹路径时，**临时从障碍物列表中移除炸弹目标格**：

```c
// 检测是否推炸弹
if (test_is_bomb[ti]) {
  // 构建临时障碍物列表
  size_t new_count = 0;
  for (size_t k = 0; k < current_obstacle_count_test; ++k) {
    if (g_current_obstacles[k].row == test_targets_list[ti].row &&
        g_current_obstacles[k].col == test_targets_list[ti].col) {
      continue;  // 跳过炸弹目标障碍，允许炸弹进入
    }
    temp_obstacles[new_count++] = g_current_obstacles[k];
  }
  
  // 使用临时列表规划
  plan_boxes_greedy_v3_bfs(..., temp_obstacles, new_count, ...);
}
```

### 影响范围
- ✅ 修复后，炸弹可以成功推到障碍物上
- ✅ 爆炸后该障碍及相邻障碍正常移除
- ✅ 后续推箱使用更新后的地图
- ⚠️ 仅影响推炸弹阶段，不影响推箱子逻辑

## 使用示例

### 示例1：基本用法（v1.4）

```c
#include "assigned_box_planner_greedy_3.h"
#include "zf_common_headfile.h"

void basic_example(void) {
    // 定义地图参数
    int rows = 10, cols = 10;
    
    // 小车起始位置
    PlannerPointV3_Bomb car = {1, 1};
    
    // 箱子位置
    PlannerPointV3_Bomb boxes[3] = {{2, 2}, {3, 3}, {4, 4}};
    size_t box_count = 3;
    
    // 目标位置
    PlannerPointV3_Bomb targets[3] = {{8, 8}, {8, 7}, {8, 6}};
    size_t target_count = 3;
    
    // 炸弹位置
    PlannerPointV3_Bomb bombs[2] = {{1, 5}, {5, 1}};
    size_t bomb_count = 2;
    
    // 障碍物（形成一堵墙）
    PlannerPointV3_Bomb obstacles[5] = {{5, 3}, {5, 4}, {5, 5}, {5, 6}, {5, 7}};
    size_t obstacle_count = 5;
    
    // 输出缓冲区
    PlannerPointV3_Bomb path[1000];
    size_t path_steps = 0;
    size_t box_target_mapping[3];
    size_t used_bombs[5];
    size_t used_bomb_count = 0;
    
    // 调用规划算法
    int ret = plan_boxes_with_bombs_v3(
        rows, cols, car,
        boxes, box_count,
        targets, target_count,
        bombs, bomb_count,
        obstacles, obstacle_count,
        path, 1000, &path_steps,
        box_target_mapping,
        used_bombs, &used_bomb_count
    );
    
    if (ret == 0) {
        printf("✅ 规划成功！总步数：%zu\n", path_steps);
        printf("使用了 %zu 个炸弹\n", used_bomb_count);
        
        // v1.3+：有炸弹优先使用炸弹，used_bomb_count通常>0
        if (used_bomb_count > 0) {
            printf("使用的炸弹：");
            for (size_t i = 0; i < used_bomb_count; ++i) {
                printf("炸弹%zu ", used_bombs[i]);
            }
            printf("\n");
        }
        
        // 打印路径
        for (size_t i = 0; i < path_steps; ++i) {
            printf("步骤%zu: (%d, %d)\n", i+1, path[i].row, path[i].col);
        }
    } else {
        printf("规划失败，错误码：%d\n", ret);
    }
}
```

### 示例2：场景一 - 不需要炸弹（路径畅通）

```c
void example_no_bomb_needed(void) {
    printf("\n=== 场景1：路径畅通，不需要炸弹 ===\n");
    
    int rows = 8, cols = 8;
    PlannerPointV3_Bomb car = {1, 1};
    
    // 1个箱子，可以直接推到目标
    PlannerPointV3_Bomb boxes[1] = {{2, 2}};
    PlannerPointV3_Bomb targets[1] = {{6, 6}};
    
    // 炸弹存在但不需要使用
    PlannerPointV3_Bomb bombs[1] = {{1, 5}};
    
    // 少量障碍，不阻挡路径
    PlannerPointV3_Bomb obstacles[3] = {{0, 3}, {7, 3}, {3, 0}};
    
    PlannerPointV3_Bomb path[1000];
    size_t path_steps = 0;
    size_t used_bombs[5];
    size_t used_bomb_count = 0;
    
    int ret = plan_boxes_with_bombs_v3(
        rows, cols, car,
        boxes, 1, targets, 1,
        bombs, 1, obstacles, 3,
        path, 1000, &path_steps,
        NULL, used_bombs, &used_bomb_count
    );
    
    if (ret == 0) {
        printf("✅ 成功！步数：%zu，使用炸弹：%zu\n", path_steps, used_bomb_count);
        printf("预期：不使用炸弹（直接推箱更快）\n");
    }
}
```

### 示例3：场景二 - 墙体阻挡，需要炸弹

```c
void example_wall_blocking(void) {
    printf("\n=== 场景2：墙体阻挡，需要炸弹 ===\n");
    
    int rows = 10, cols = 10;
    PlannerPointV3_Bomb car = {1, 1};
    
    // 2个箱子在墙的一侧
    PlannerPointV3_Bomb boxes[2] = {{2, 2}, {3, 2}};
    // 目标在墙的另一侧
    PlannerPointV3_Bomb targets[2] = {{8, 7}, {8, 8}};
    
    // 1个炸弹，可用于炸墙
    PlannerPointV3_Bomb bombs[1] = {{2, 5}};
    
    // 一堵垂直墙，完全阻挡路径（第5列）
    PlannerPointV3_Bomb obstacles[5] = {
        {3, 5}, {4, 5}, {5, 5}, {6, 5}, {7, 5}
    };
    
    // 地图示意：
    // . C . . . . . . . .
    // . . B . . . . . . .
    // . . B . . * . . . .
    // . . . . . # . . . .
    // . . . . . # . . . .
    // . . . . . # . . . .
    // . . . . . # . . . .
    // . . . . . # . . T T
    
    PlannerPointV3_Bomb path[1000];
    size_t path_steps = 0;
    size_t used_bombs[5];
    size_t used_bomb_count = 0;
    
    int ret = plan_boxes_with_bombs_v3(
        rows, cols, car,
        boxes, 2, targets, 2,
        bombs, 1, obstacles, 5,
        path, 1000, &path_steps,
        NULL, used_bombs, &used_bomb_count
    );
    
    if (ret == 0) {
        printf("✅ 成功！步数：%zu，使用炸弹：%zu\n", path_steps, used_bomb_count);
        if (used_bomb_count > 0) {
            printf("炸开中间墙体，路径大幅缩短！\n");
        }
    }
}
```

### 示例4：场景三 - 多个炸弹和箱子

```c
void example_multiple_bombs(void) {
    printf("\n=== 场景3：多个炸弹，复杂地形 ===\n");
    
    int rows = 12, cols = 12;
    PlannerPointV3_Bomb car = {1, 1};
    
    // 3个箱子分散在地图各处
    PlannerPointV3_Bomb boxes[3] = {{2, 2}, {2, 8}, {8, 2}};
    PlannerPointV3_Bomb targets[3] = {{10, 10}, {10, 9}, {10, 8}};
    
    // 2个炸弹可供选择
    PlannerPointV3_Bomb bombs[2] = {{3, 5}, {5, 3}};
    
    // 十字形障碍，阻挡多个方向
    PlannerPointV3_Bomb obstacles[10] = {
        // 横向墙
        {5, 3}, {5, 4}, {5, 5}, {5, 6}, {5, 7},
        // 纵向墙
        {3, 5}, {4, 5}, {6, 5}, {7, 5}, {8, 5}
    };
    
    PlannerPointV3_Bomb path[2000];
    size_t path_steps = 0;
    size_t box_target_mapping[3];
    size_t used_bombs[5];
    size_t used_bomb_count = 0;
    
    int ret = plan_boxes_with_bombs_v3(
        rows, cols, car,
        boxes, 3, targets, 3,
        bombs, 2, obstacles, 10,
        path, 2000, &path_steps,
        box_target_mapping, used_bombs, &used_bomb_count
    );
    
    if (ret == 0) {
        printf("✅ 成功！步数：%zu，使用炸弹：%zu\n", path_steps, used_bomb_count);
        printf("箱子到目标的映射：\n");
        for (size_t i = 0; i < 3; ++i) {
            printf("  箱子%zu -> 目标%zu\n", i, box_target_mapping[i]);
        }
    }
}
```

### 示例5：场景四 - 炸弹位置不佳

```c
void example_bomb_far_away(void) {
    printf("\n=== 场景4：炸弹太远，不如绕路 ===\n");
    
    int rows = 10, cols = 10;
    PlannerPointV3_Bomb car = {1, 1};
    
    PlannerPointV3_Bomb boxes[1] = {{2, 2}};
    PlannerPointV3_Bomb targets[1] = {{8, 8}};
    
    // 炸弹在地图边缘，距离很远
    PlannerPointV3_Bomb bombs[1] = {{9, 1}};
    
    // 一小段墙，可以轻松绕过
    PlannerPointV3_Bomb obstacles[3] = {{4, 4}, {4, 5}, {4, 6}};
    
    PlannerPointV3_Bomb path[1000];
    size_t path_steps = 0;
    size_t used_bombs[5];
    size_t used_bomb_count = 0;
    
    int ret = plan_boxes_with_bombs_v3(
        rows, cols, car,
        boxes, 1, targets, 1,
        bombs, 1, obstacles, 3,
        path, 1000, &path_steps,
        NULL, used_bombs, &used_bomb_count
    );
    
    if (ret == 0) {
        printf("✅ 成功！步数：%zu，使用炸弹：%zu\n", path_steps, used_bomb_count);
        printf("预期：绕路比去拿炸弹再炸墙更快\n");
    }
}
```

### 示例6：在RT1064主程序中集成

```c
#include "zf_common_headfile.h"
#include "assigned_box_planner_greedy_3.h"

// 全局变量存储地图信息
static PlannerPointV3_Bomb g_current_car_pos;
static PlannerPointV3_Bomb g_boxes[10];
static size_t g_box_count;
static PlannerPointV3_Bomb g_targets[10];
static size_t g_target_count;
static PlannerPointV3_Bomb g_bombs[5];
static size_t g_bomb_count;
static PlannerPointV3_Bomb g_obstacles[100];
static size_t g_obstacle_count;

// 规划路径并执行
void plan_and_execute_with_bombs(void) {
    printf("开始路径规划...\n");
    
    // 输出缓冲区
    PlannerPointV3_Bomb path[1000];
    size_t path_steps = 0;
    size_t box_target_mapping[10];
    size_t used_bombs[5];
    size_t used_bomb_count = 0;
    
    // 调用规划算法
    int ret = plan_boxes_with_bombs_v3(
        20, 20,  // 地图尺寸
        g_current_car_pos,
        g_boxes, g_box_count,
        g_targets, g_target_count,
        g_bombs, g_bomb_count,
        g_obstacles, g_obstacle_count,
        path, 1000, &path_steps,
        box_target_mapping,
        used_bombs, &used_bomb_count
    );
    
    if (ret != 0) {
        printf("规划失败，错误码：%d\n", ret);
        
        // 在IPS200屏幕上显示错误信息
        ips200_show_string(0, 0, "Plan Failed!");
        
        switch (ret) {
            case -2:
                ips200_show_string(0, 16, "No boxes/targets");
                break;
            case -6:
                ips200_show_string(0, 16, "No valid path");
                break;
            case -7:
                ips200_show_string(0, 16, "Buffer too small");
                break;
            default:
                ips200_show_string(0, 16, "Unknown error");
                break;
        }
        return;
    }
    
    // 规划成功，显示结果
    printf("规划成功！总步数：%zu\n", path_steps);
    printf("使用炸弹数：%zu\n", used_bomb_count);
    
    // 在IPS200上显示规划结果
    char buf[64];
    sprintf(buf, "Steps: %zu", path_steps);
    ips200_show_string(0, 0, buf);
    sprintf(buf, "Bombs: %zu", used_bomb_count);
    ips200_show_string(0, 16, buf);
    
    // 逐步执行路径
    for (size_t i = 0; i < path_steps; ++i) {
        printf("移动到: (%d, %d)\n", path[i].row, path[i].col);
        
        // 在屏幕上显示当前位置
        sprintf(buf, "Pos: (%d,%d)", path[i].row, path[i].col);
        ips200_show_string(0, 32, buf);
        
        // TODO: 调用电机控制函数，实际移动小车
        // motor_move_to(path[i].row, path[i].col);
        
        // 延时，便于观察
        systick_delay_ms(100);
    }
    
    printf("路径执行完成！\n");
    ips200_show_string(0, 48, "Complete!");
}

// 菜单中的规划功能入口
void menu_plan_with_bombs(void) {
    ips200_clear();
    ips200_show_string(0, 0, "Planning...");
    
    // 执行规划
    plan_and_execute_with_bombs();
    
    // 等待按键返回菜单
    ips200_show_string(0, 64, "Press KEY to return");
    while (key1_get() != 0) {
        systick_delay_ms(10);
    }
}
```

### 示例7：处理各种返回值

```c
void handle_planning_result(int ret, size_t path_steps) {
    switch (ret) {
        case 0:
            printf("✅ 规划成功！路径长度：%zu\n", path_steps);
            break;
            
        case -1:
            printf("❌ 参数错误：空指针\n");
            printf("检查：boxes, targets, path_buffer, out_steps不能为NULL\n");
            break;
            
        case -2:
            printf("❌ 输入错误：箱子或目标为空\n");
            printf("检查：box_count和target_count必须>0\n");
            break;
            
        case -3:
            printf("❌ 箱子数量超限\n");
            printf("当前最多支持10个箱子\n");
            break;
            
        case -4:
            printf("❌ 路径缓冲区为0\n");
            printf("path_capacity必须>0\n");
            break;
            
        case -5:
            printf("❌ 迭代超步数\n");
            printf("推箱子陷入循环，可能是地图设计有问题\n");
            break;
            
        case -6:
            printf("❌ 无可行路径\n");
            printf("可能原因：\n");
            printf("  1. 箱子被障碍完全包围\n");
            printf("  2. 目标点不可达\n");
            printf("  3. 地图设计导致死局\n");
            break;
            
        case -7:
            printf("❌ 路径缓冲区不足\n");
            printf("建议：增大path_capacity到2048或更大\n");
            break;
            
        case -8:
            printf("❌ 目标数少于箱子数\n");
            printf("每个箱子需要一个对应的目标点\n");
            break;
            
        case -9:
            printf("❌ 炸弹数量超限\n");
            printf("当前最多支持5个炸弹\n");
            break;
            
        default:
            printf("❌ 未知错误码：%d\n", ret);
            break;
    }
}

// 使用示例
void example_with_error_handling(void) {
    PlannerPointV3_Bomb car = {1, 1};
    PlannerPointV3_Bomb boxes[2] = {{2, 2}, {3, 3}};
    PlannerPointV3_Bomb targets[2] = {{8, 8}, {8, 7}};
    PlannerPointV3_Bomb bombs[1] = {{2, 5}};
    PlannerPointV3_Bomb obstacles[5] = {{5, 3}, {5, 4}, {5, 5}, {5, 6}, {5, 7}};
    
    PlannerPointV3_Bomb path[1000];
    size_t path_steps = 0;
    
    int ret = plan_boxes_with_bombs_v3(
        10, 10, car,
        boxes, 2, targets, 2,
        bombs, 1, obstacles, 5,
        path, 1000, &path_steps,
        NULL, NULL, NULL
    );
    
    handle_planning_result(ret, path_steps);
}
```

### 示例8：不使用可选参数

```c
void example_minimal_usage(void) {
    // 最简化用法：不需要箱子映射和炸弹使用信息
    
    int rows = 10, cols = 10;
    PlannerPointV3_Bomb car = {1, 1};
    PlannerPointV3_Bomb boxes[1] = {{2, 2}};
    PlannerPointV3_Bomb targets[1] = {{8, 8}};
    PlannerPointV3_Bomb bombs[1] = {{2, 5}};
    PlannerPointV3_Bomb obstacles[3] = {{5, 5}, {5, 6}, {5, 7}};
    
    PlannerPointV3_Bomb path[1000];
    size_t path_steps = 0;
    
    // 所有可选参数传NULL
    int ret = plan_boxes_with_bombs_v3(
        rows, cols, car,
        boxes, 1, targets, 1,
        bombs, 1, obstacles, 3,
        path, 1000, &path_steps,
        NULL,  // 不需要箱子映射
        NULL,  // 不需要使用的炸弹列表
        NULL   // 不需要炸弹数量
    );
    
    if (ret == 0) {
        printf("规划成功！步数：%zu\n", path_steps);
        // 直接使用path执行路径
    }
}
```

## 算法特点与优势

### 优势
1. **自动寻找最优方案**：
   - 自动比较使用炸弹和不使用炸弹的路径
   - 枚举不同的炸弹使用时机
   - 返回步数最少的方案

2. **智能识别炸段**：
   - 自动识别连续障碍组
   - 优先炸中心位置，收益最大化
   - 避免浪费炸弹

3. **动态地图更新**：
   - 炸弹爆炸后实时更新障碍物
   - 后续推箱基于新地图规划
   - 支持多炸弹连续使用

4. **基于成熟算法**：
   - 底层使用v3_BFS的纯BFS+A*算法
   - 路径质量有保证
   - 继承了死点检测、绕箱等特性

### 当前限制

1. **简化版枚举**：
   - 当前实现只尝试使用**单个炸弹**
   - 多炸弹组合需扩展枚举逻辑
   - 可在`plan_boxes_with_bombs_v3`中添加多炸弹循环

2. **计算复杂度**：
   - 枚举所有方案：O(B × S × N)
     - B = 炸弹数量
     - S = 炸段数量
     - N = 箱子数量（插入位置）
   - 每个方案需完整规划：O(N × M)
     - M = 单个推箱的BFS/A*复杂度
   - 总体：O(B × S × N² × M)

3. **内存使用**：
   - 最多支持10个箱子、5个炸弹
   - 路径缓冲区建议≥2048步
   - 障碍物最多200个

## 扩展建议

### 1. 多炸弹组合优化
当前只尝试单炸弹，可扩展为：
```c
// 尝试2个炸弹的组合
for (size_t b1 = 0; b1 < bomb_count; ++b1) {
    for (size_t s1 = 0; s1 < segment_count; ++s1) {
        for (size_t pos1 = 0; pos1 <= box_count; ++pos1) {
            for (size_t b2 = b1+1; b2 < bomb_count; ++b2) {
                for (size_t s2 = 0; s2 < segment_count; ++s2) {
                    for (size_t pos2 = pos1; pos2 <= box_count; ++pos2) {
                        // 测试使用b1和b2的方案
                    }
                }
            }
        }
    }
}
```

### 2. 启发式剪枝
减少枚举量：
- 只考虑阻挡当前推箱路径的炸段
- 预判炸弹使用收益（炸开后路径缩短量）
- 动态调整枚举深度

### 3. 更智能的炸段评分
当前所有炸段平等对待，可增加评分机制：
- 炸段长度越长，优先级越高
- 位于关键路径上的炸段，优先级越高
- 距离小车/箱子太远的炸段，优先级降低

### 4. 交互式规划
对于嵌入式环境，可增加：
- 分步计算，避免阻塞
- 提前终止（找到足够好的方案即返回）
- 实时显示进度

## 与原算法的关系

```
assigned_box_planner_greedy.c (v1)
           |
           ├─> 基础贪心算法
           |
           v
assigned_box_planner_greedy_2.c (v3_BFS)
           |
           ├─> 纯BFS+A*，移除贪心
           ├─> 全局距离预计算
           ├─> 更精确的路径搜索
           |
           v
assigned_box_planner_greedy_3.c (v3_Bomb)
           |
           ├─> 增加炸弹机制
           ├─> 识别关键炸段
           ├─> 方案枚举与优化
           └─> 动态地图更新
```

## 测试建议

### 测试用例1：无需炸弹
```
地图：开阔，箱子可直达目标
预期：返回基准路径，不使用炸弹
```

### 测试用例2：单墙阻挡
```
地图：一堵墙挡住箱子去路
炸弹：可炸开中间位置
预期：使用炸弹，路径明显缩短
```

### 测试用例3：多路径选择
```
地图：可绕路，也可炸墙
预期：比较两种方案，选择更短的
```

### 测试用例4：多炸弹
```
地图：多处障碍，多个炸弹
预期：正确选择炸弹使用顺序
```

### 测试用例5：炸弹无用
```
地图：炸弹位置偏远或炸不开关键路径
预期：不使用炸弹，返回基准路径
```

## 调试技巧

1. **打印中间状态**：
   ```c
   printf("识别到 %zu 个炸段\n", segment_count);
   printf("测试方案：炸弹%zu -> 炸段%zu，位置%zu\n", bi, si, pos);
   printf("方案步数：%zu，当前最优：%zu\n", test_steps, best_steps);
   ```

2. **可视化地图**：
   - 打印障碍物分布
   - 显示炸段位置
   - 标记炸弹和箱子

3. **分步验证**：
   - 先测试炸段识别是否正确
   - 再测试单个方案的路径规划
   - 最后测试方案比较逻辑

## 总结

该算法在原有推箱规划的基础上，增加了炸弹这一战术元素，通过枚举和比较，自动找到最优路径。适用于需要动态改变地图、寻找最短路径的推箱类游戏或机器人路径规划场景。

核心思想：**枚举 + 模拟 + 比较 + 选择**，用计算换取最优解。

## 版本历史总结

| 版本 | 主要修改 | 适用场景 |
|------|---------|---------|
| v1.0 | 初始实现，基础炸弹机制 | 测试验证 |
| v1.1 | 修复栈溢出，使用全局变量 | 嵌入式环境 |
| v1.2 | 优化计算量，多重限制 | 性能敏感场景 |
| v1.3 | 强制使用炸弹，完整枚举 | 需要炸弹效果 |
| v1.4 | 修复推炸弹到障碍问题 | **当前稳定版本** ⭐ |

## 已知问题和限制

1. **全局变量不可重入** - 多任务环境需加锁
2. **计算量较大** - 枚举所有炸段×插入位置，可能需要数秒
3. **只支持单炸弹优化** - 多炸弹组合未实现
4. **炸段识别简单** - 只识别直线连续障碍

## 维护约定

后续如再修改该算法，请在本文件**版本修改记录**部分追加条目，包括：
- 版本号（递增）
- 修改内容（具体说明）
- 修改原因（解决什么问题）
- 影响范围（接口、性能等）

> 最后更新：v1.4 - 2025年，修复推炸弹到障碍物的关键问题

