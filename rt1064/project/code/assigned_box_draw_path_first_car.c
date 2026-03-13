#include "assigned_box_draw_path_first_car.h"

#include <limits.h>
#include <string.h>

#define DRAW_PATH_FIRST_CAR_CANDIDATE_COUNT 8
#define DRAW_PATH_FIRST_CAR_MAX_OBSTACLES 200
#define DRAW_PATH_FIRST_CAR_MAX_SPECIAL_RETRIES PLANNER_V3_TWO_PHASE_MAX_BOMBS

typedef struct {
    size_t display_index;
    PlannerPointV3_BFS anchor;
    int sort_dist;
} DrawPathFirstCarItem;

PlannerAllBoxPaths draw_path_first_car_paths = {0};
PlannerPointV3_BFS draw_path_first_car_full_path[DRAW_PATH_FIRST_CAR_FULL_PATH_CAPACITY] = {0};
size_t draw_path_first_car_full_steps = 0;
size_t draw_path_first_car_visit_order[DRAW_PATH_FIRST_CAR_MAX_ITEMS] = {0};
size_t draw_path_first_car_visit_count = 0;

static int draw_path_first_car_point_equal(PlannerPointV3_BFS a, PlannerPointV3_BFS b);

static void draw_path_first_car_set_error(int detail)
{
    last_err_stage = DRAW_PATH_FIRST_CAR_STAGE;
    last_err_detail = detail;
}

static void draw_path_first_car_clear_entry(PlannerBoxPathOutput *entry)
{
    memset(entry, 0, sizeof(*entry));
    entry->planned_order = -1;
}

void clear_draw_path_first_car_paths(void)
{
    draw_path_first_car_paths.box_count = 0;
    draw_path_first_car_full_steps = 0;
    draw_path_first_car_visit_count = 0;
    for (size_t i = 0; i < DRAW_PATH_FIRST_CAR_MAX_ITEMS; ++i)
    {
        draw_path_first_car_visit_order[i] = SIZE_MAX;
        draw_path_first_car_clear_entry(&draw_path_first_car_paths.box_paths[i]);
    }
}

static int draw_path_first_car_append_full_segment(const PlannerPointV3_BFS *segment,
                                                   size_t segment_len)
{
    size_t copy_start = 0;
    size_t copy_len = segment_len;

    if (segment_len == 0)
    {
        return 1;
    }
    if (draw_path_first_car_full_steps > 0 &&
        draw_path_first_car_point_equal(
            draw_path_first_car_full_path[draw_path_first_car_full_steps - 1],
            segment[0]))
    {
        copy_start = 1;
        copy_len--;
    }
    if (draw_path_first_car_full_steps + copy_len > DRAW_PATH_FIRST_CAR_FULL_PATH_CAPACITY)
    {
        return 0;
    }
    if (copy_len > 0)
    {
        memcpy(&draw_path_first_car_full_path[draw_path_first_car_full_steps],
               &segment[copy_start],
               copy_len * sizeof(segment[0]));
        draw_path_first_car_full_steps += copy_len;
    }
    return 1;
}

static int draw_path_first_car_point_equal(PlannerPointV3_BFS a, PlannerPointV3_BFS b)
{
    return a.row == b.row && a.col == b.col;
}

static int draw_path_first_car_in_bounds(int rows, int cols, int row, int col)
{
    return row >= 0 && row < rows && col >= 0 && col < cols;
}

static int draw_path_first_car_point_in_array(PlannerPointV3_BFS point,
                                              const PlannerPointV3_BFS *points,
                                              size_t point_count)
{
    for (size_t i = 0; i < point_count; ++i)
    {
        if (draw_path_first_car_point_equal(point, points[i]))
        {
            return 1;
        }
    }
    return 0;
}

static int draw_path_first_car_cell_is_empty(int rows, int cols,
                                             PlannerPointV3_BFS point,
                                             const PlannerPointV3_BFS *obstacles,
                                             size_t obstacle_count,
                                             const PlannerPointV3_BFS *bombs,
                                             size_t bomb_count,
                                             const PlannerPointV3_BFS *boxes,
                                             size_t box_count)
{
    if (!draw_path_first_car_in_bounds(rows, cols, point.row, point.col))
    {
        return 0;
    }
    if (draw_path_first_car_point_in_array(point, obstacles, obstacle_count))
    {
        return 0;
    }
    if (draw_path_first_car_point_in_array(point, bombs, bomb_count))
    {
        return 0;
    }
    if (draw_path_first_car_point_in_array(point, boxes, box_count))
    {
        return 0;
    }
    return 1;
}

static int draw_path_first_car_segment_is_clear(
    int rows, int cols, PlannerPointV3_BFS anchor,
    int dir_row, int dir_col, int radius,
    const PlannerPointV3_BFS *obstacles, size_t obstacle_count,
    const PlannerPointV3_BFS *bombs, size_t bomb_count,
    const PlannerPointV3_BFS *boxes, size_t box_count)
{
    for (int step = 1; step < radius; ++step)
    {
        PlannerPointV3_BFS between = {
            (int8_t)(anchor.row + dir_row * step),
            (int8_t)(anchor.col + dir_col * step)
        };

        if (!draw_path_first_car_cell_is_empty(rows, cols, between,
                                               obstacles, obstacle_count,
                                               bombs, bomb_count,
                                               boxes, box_count))
        {
            return 0;
        }
    }
    return 1;
}

static size_t draw_path_first_car_collect_candidates(
    int rows, int cols, PlannerPointV3_BFS anchor,
    const PlannerPointV3_BFS *obstacles, size_t obstacle_count,
    const PlannerPointV3_BFS *bombs, size_t bomb_count,
    const PlannerPointV3_BFS *boxes, size_t box_count,
    PlannerPointV3_BFS *out_candidates)
{
    static const int dirs[4][2] = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1}
    };
    static const int radii[2] = {1, 3};
    size_t count = 0;

    for (size_t ri = 0; ri < 2; ++ri)
    {
        for (size_t di = 0; di < 4; ++di)
        {
            PlannerPointV3_BFS candidate = {
                (int8_t)(anchor.row + dirs[di][0] * radii[ri]),
                (int8_t)(anchor.col + dirs[di][1] * radii[ri])
            };

            if (!draw_path_first_car_segment_is_clear(
                    rows, cols, anchor,
                    dirs[di][0], dirs[di][1], radii[ri],
                    obstacles, obstacle_count,
                    bombs, bomb_count,
                    boxes, box_count))
            {
                continue;
            }
            if (!draw_path_first_car_cell_is_empty(rows, cols, candidate,
                                                   obstacles, obstacle_count,
                                                   bombs, bomb_count,
                                                   boxes, box_count))
            {
                continue;
            }
            if (draw_path_first_car_point_in_array(candidate, out_candidates, count))
            {
                continue;
            }
            out_candidates[count++] = candidate;
        }
    }

    return count;
}

static int draw_path_first_car_pick_best_candidate(
    int rows, int cols, PlannerPointV3_BFS car, PlannerPointV3_BFS anchor,
    const PlannerPointV3_BFS *boxes, size_t box_count,
    const PlannerPointV3_BFS *targets, size_t target_count,
    const PlannerPointV3_BFS *bombs, size_t bomb_count,
    const PlannerPointV3_BFS *obstacles, size_t obstacle_count,
    PlannerPointV3_BFS *out_candidate, int *out_dist)
{
    PlannerPointV3_BFS candidates[DRAW_PATH_FIRST_CAR_CANDIDATE_COUNT];
    size_t candidate_count = draw_path_first_car_collect_candidates(
        rows, cols, anchor, obstacles, obstacle_count, bombs, bomb_count,
        boxes, box_count, candidates);
    int best_dist = INT_MAX;
    size_t best_idx = SIZE_MAX;

    (void)targets;
    (void)target_count;

    if (candidate_count == 0)
    {
        return DRAW_PATH_FIRST_CAR_ERR_NO_CANDIDATE;
    }

    for (size_t i = 0; i < candidate_count; ++i)
    {
        int dist = planner_v3_query_car_distance(
            rows, cols, car, candidates[i],
            obstacles, obstacle_count,
            bombs, bomb_count,
            boxes, box_count,
            1);
        if (dist < best_dist)
        {
            best_dist = dist;
            best_idx = i;
        }
    }

    if (best_idx == SIZE_MAX || best_dist == INT_MAX)
    {
        return DRAW_PATH_FIRST_CAR_ERR_UNREACHABLE_ITEM;
    }

    *out_candidate = candidates[best_idx];
    *out_dist = best_dist;
    return 0;
}

static int draw_path_first_car_estimate_sort_distance(
    int rows, int cols, PlannerPointV3_BFS car, PlannerPointV3_BFS anchor,
    const PlannerPointV3_BFS *boxes, size_t box_count,
    const PlannerPointV3_BFS *targets, size_t target_count,
    const PlannerPointV3_BFS *bombs, size_t bomb_count,
    const PlannerPointV3_BFS *obstacles, size_t obstacle_count)
{
    PlannerPointV3_BFS candidate = {0, 0};
    int dist = INT_MAX;
    int pick_res = draw_path_first_car_pick_best_candidate(
        rows, cols, car, anchor,
        boxes, box_count,
        targets, target_count,
        bombs, bomb_count,
        obstacles, obstacle_count,
        &candidate, &dist);

    if (pick_res == 0)
    {
        return dist;
    }
    return INT_MAX;
}

static int draw_path_first_car_execute_special_push(
    int rows, int cols, PlannerPointV3_BFS *car,
    PlannerPointV3_BFS anchor,
    const PlannerPointV3_BFS *boxes, size_t box_count,
    PlannerPointV3_BFS *bombs, size_t bomb_count,
    PlannerPointV3_BFS *obstacles, size_t *obstacle_count,
    PlannerPointV3_BFS *segment, size_t segment_capacity,
    size_t *segment_len)
{
    PlannerPointV3_BFS candidates[DRAW_PATH_FIRST_CAR_CANDIDATE_COUNT];
    size_t candidate_count = draw_path_first_car_collect_candidates(
        rows, cols, anchor,
        obstacles, *obstacle_count,
        bombs, bomb_count,
        boxes, box_count,
        candidates);

    for (size_t i = 0; i < candidate_count; ++i)
    {
        size_t bomb_idx = SIZE_MAX;
        PlannerPointV3_BFS bomb_target = {-1, -1};
        PlannerPointV3_BFS exploded_obstacles[PLANNER_V3_TWO_PHASE_MAX_EXPLODED_OBSTACLES];
        size_t exploded_obstacle_count = 0;
        int plan_res = planner_v3_plan_first_push_bomb_special_path(
            rows, cols, *car, candidates[i],
            obstacles, *obstacle_count,
            bombs, bomb_count,
            boxes, box_count,
            &bomb_idx,
            &bomb_target,
            exploded_obstacles,
            PLANNER_V3_TWO_PHASE_MAX_EXPLODED_OBSTACLES,
            &exploded_obstacle_count);

        (void)bomb_idx;
        (void)bomb_target;
        (void)exploded_obstacle_count;

        if (plan_res != 1)
        {
            continue;
        }

        int resolve_res = planner_v3_resolve_first_push_bomb_special_path(
            rows, cols, car, candidates[i],
            obstacles, obstacle_count,
            bombs, bomb_count,
            boxes, box_count,
            segment, segment_capacity, segment_len);

        if (resolve_res == 1)
        {
            return 1;
        }
        if (resolve_res == -7)
        {
            return -7;
        }
    }

    return 0;
}

static int draw_path_first_car_pick_candidate_with_special_retry(
    int rows, int cols, PlannerPointV3_BFS *car, PlannerPointV3_BFS anchor,
    const PlannerPointV3_BFS *boxes, size_t box_count,
    const PlannerPointV3_BFS *targets, size_t target_count,
    PlannerPointV3_BFS *bombs, size_t bomb_count,
    PlannerPointV3_BFS *obstacles, size_t *obstacle_count,
    PlannerPointV3_BFS *segment, size_t segment_capacity, size_t *segment_len,
    PlannerPointV3_BFS *out_candidate, int *out_dist)
{
    for (size_t attempt = 0; attempt <= DRAW_PATH_FIRST_CAR_MAX_SPECIAL_RETRIES; ++attempt)
    {
        int pick_res = draw_path_first_car_pick_best_candidate(
            rows, cols, *car, anchor,
            boxes, box_count,
            targets, target_count,
            bombs, bomb_count,
            obstacles, *obstacle_count,
            out_candidate, out_dist);

        if (pick_res == 0)
        {
            return 0;
        }
        if (pick_res != DRAW_PATH_FIRST_CAR_ERR_UNREACHABLE_ITEM)
        {
            return pick_res;
        }
        if (attempt == DRAW_PATH_FIRST_CAR_MAX_SPECIAL_RETRIES)
        {
            break;
        }

        int special_res = draw_path_first_car_execute_special_push(
            rows, cols, car, anchor,
            boxes, box_count,
            bombs, bomb_count,
            obstacles, obstacle_count,
            segment, segment_capacity, segment_len);

        if (special_res == 1)
        {
            continue;
        }
        if (special_res == -7)
        {
            return DRAW_PATH_FIRST_CAR_ERR_PATH_OVERFLOW;
        }
        return DRAW_PATH_FIRST_CAR_ERR_SPECIAL_PUSH_FAIL;
    }

    return DRAW_PATH_FIRST_CAR_ERR_SPECIAL_PUSH_FAIL;
}

static void draw_path_first_car_sort_items(DrawPathFirstCarItem *items, size_t item_count)
{
    for (size_t i = 0; i < item_count; ++i)
    {
        size_t best = i;
        for (size_t j = i + 1; j < item_count; ++j)
        {
            if (items[j].sort_dist < items[best].sort_dist ||
                (items[j].sort_dist == items[best].sort_dist &&
                 items[j].display_index < items[best].display_index))
            {
                best = j;
            }
        }
        if (best != i)
        {
            DrawPathFirstCarItem tmp = items[i];
            items[i] = items[best];
            items[best] = tmp;
        }
    }
}

static int draw_path_first_car_fail(int detail, int result)
{
    clear_draw_path_first_car_paths();
    draw_path_first_car_set_error(detail);
    return result;
}

int build_draw_path_first_car_paths_v3(
    int rows, int cols, PlannerPointV3_BFS car,
    const PlannerPointV3_BFS *boxes, size_t box_count,
    const PlannerPointV3_BFS *targets, size_t target_count,
    const PlannerPointV3_BFS *bombs, size_t bomb_count,
    const PlannerPointV3_BFS *obstacles, size_t obstacle_count)
{
    DrawPathFirstCarItem items[DRAW_PATH_FIRST_CAR_MAX_ITEMS];
    PlannerPointV3_BFS mutable_bombs[PLANNER_V3_TWO_PHASE_MAX_BOMBS];
    PlannerPointV3_BFS mutable_obstacles[DRAW_PATH_FIRST_CAR_MAX_OBSTACLES];
    size_t total_items = box_count + target_count;
    size_t mutable_bomb_count = bomb_count;
    size_t mutable_obstacle_count = obstacle_count;
    PlannerPointV3_BFS current_car = car;

    if ((box_count > 0 && !boxes) || (target_count > 0 && !targets))
    {
        return draw_path_first_car_fail(DRAW_PATH_FIRST_CAR_ERR_INVALID_ARG, -1);
    }
    if ((bomb_count > 0 && !bombs) || (obstacle_count > 0 && !obstacles))
    {
        return draw_path_first_car_fail(DRAW_PATH_FIRST_CAR_ERR_INVALID_ARG, -1);
    }
    if (rows <= 0 || cols <= 0)
    {
        return draw_path_first_car_fail(DRAW_PATH_FIRST_CAR_ERR_INVALID_ARG, -1);
    }
    if (bomb_count > PLANNER_V3_TWO_PHASE_MAX_BOMBS ||
        obstacle_count > DRAW_PATH_FIRST_CAR_MAX_OBSTACLES)
    {
        return draw_path_first_car_fail(DRAW_PATH_FIRST_CAR_ERR_INVALID_ARG, -1);
    }
    if ((size_t)(rows * cols) > PLANNER_V3_MAX_GRID_CELLS)
    {
        return draw_path_first_car_fail(DRAW_PATH_FIRST_CAR_ERR_GRID_TOO_LARGE, -3);
    }
    if (total_items > DRAW_PATH_FIRST_CAR_MAX_ITEMS)
    {
        return draw_path_first_car_fail(DRAW_PATH_FIRST_CAR_ERR_TOO_MANY_ITEMS, -2);
    }

    clear_draw_path_first_car_paths();
    draw_path_first_car_paths.box_count = total_items;
    if (mutable_bomb_count > 0)
    {
        memcpy(mutable_bombs, bombs, mutable_bomb_count * sizeof(mutable_bombs[0]));
    }
    if (mutable_obstacle_count > 0)
    {
        memcpy(mutable_obstacles, obstacles, mutable_obstacle_count * sizeof(mutable_obstacles[0]));
    }

    if (total_items == 0)
    {
        return 0;
    }

    for (size_t i = 0; i < box_count; ++i)
    {
        items[i].display_index = i;
        items[i].anchor = boxes[i];
        items[i].sort_dist = draw_path_first_car_estimate_sort_distance(
            rows, cols, car, boxes[i],
            boxes, box_count,
            targets, target_count,
            bombs, bomb_count,
            obstacles, obstacle_count);
    }

    for (size_t i = 0; i < target_count; ++i)
    {
        size_t display_index = box_count + i;
        items[display_index].display_index = display_index;
        items[display_index].anchor = targets[i];
        items[display_index].sort_dist = draw_path_first_car_estimate_sort_distance(
            rows, cols, car, targets[i],
            boxes, box_count,
            targets, target_count,
            bombs, bomb_count,
            obstacles, obstacle_count);
    }

    draw_path_first_car_sort_items(items, total_items);

    for (size_t order_idx = 0; order_idx < total_items; ++order_idx)
    {
        PlannerPointV3_BFS segment[400];
        PlannerPointV3_BFS candidate = {0, 0};
        PlannerPointV3_BFS segment_car = current_car;
        PlannerBoxPathOutput *out = &draw_path_first_car_paths.box_paths[items[order_idx].display_index];
        size_t segment_len = 1;
        int pick_res;
        int move_res;
        int unused_dist = INT_MAX;

        segment[0] = current_car;
        pick_res = draw_path_first_car_pick_candidate_with_special_retry(
            rows, cols, &segment_car, items[order_idx].anchor,
            boxes, box_count,
            targets, target_count,
            mutable_bombs, mutable_bomb_count,
            mutable_obstacles, &mutable_obstacle_count,
            segment, sizeof(segment) / sizeof(segment[0]), &segment_len,
            &candidate, &unused_dist);
        if (pick_res != 0)
        {
            if (pick_res == DRAW_PATH_FIRST_CAR_ERR_PATH_OVERFLOW)
            {
                return draw_path_first_car_fail(DRAW_PATH_FIRST_CAR_ERR_PATH_OVERFLOW, -6);
            }
            return draw_path_first_car_fail(pick_res, -5);
        }

        move_res = planner_v3_append_car_path_with_astar(
            rows, cols, &segment_car, candidate,
            mutable_obstacles, mutable_obstacle_count,
            mutable_bombs, mutable_bomb_count,
            boxes, box_count,
            segment, sizeof(segment) / sizeof(segment[0]),
            &segment_len, 1);
        if (move_res == -7)
        {
            return draw_path_first_car_fail(DRAW_PATH_FIRST_CAR_ERR_PATH_OVERFLOW, -6);
        }
        if (move_res <= 0)
        {
            return draw_path_first_car_fail(DRAW_PATH_FIRST_CAR_ERR_ASTAR_FAILED, -7);
        }

        draw_path_first_car_clear_entry(out);
        out->valid = 1;
        out->pre_reachable = 1;
        out->planned_order = (int)order_idx;
        out->path_len = segment_len;
        memcpy(out->path, segment, segment_len * sizeof(segment[0]));
        if (!draw_path_first_car_append_full_segment(segment, segment_len))
        {
            return draw_path_first_car_fail(DRAW_PATH_FIRST_CAR_ERR_PATH_OVERFLOW, -6);
        }

        draw_path_first_car_visit_order[order_idx] = items[order_idx].display_index;
        draw_path_first_car_visit_count = order_idx + 1;
        current_car = segment_car;
    }

    return 0;
}
