#include "Filter.h"
#include <stddef.h>

static const int FILTER_MOVING_AVG_LEN = 5;

static int filter_is_valid(const filter_t *me)
{
    return (me != NULL) && (me->magic == FILTER_MAGIC);
}

static void update_history(filter_t *me, float input)
{
    int i;

    if (!filter_is_valid(me)) {
        return;
    }

    if ((me->history == NULL) || (me->history_len <= 0) || (me->history_len > FILTER_HISTORY_MAX_LEN)) {
        return;
    }

    for (i = me->history_len - 1; i > 0; i--) {
        me->history[i] = me->history[i - 1];
    }

    me->history[0] = input;
}

static float calculate_average(const filter_t *me)
{
    float sum = 0.0f;
    int i;
    int count;

    if (!filter_is_valid(me) || (me->history == NULL) || (me->history_len <= 0) || (me->history_len > FILTER_HISTORY_MAX_LEN)) {
        return 0.0f;
    }

    count = me->history_count;
    if (count <= 0) {
        return 0.0f;
    }
    if (count > me->history_len) {
        count = me->history_len;
    }

    for (i = 0; i < count; i++) {
        sum += me->history[i];
    }

    return sum / (float)count;
}

// 真正的滑动平均滤波
static void avg_update(filter_t *me, float input)
{
    if (!filter_is_valid(me)) {
        return;
    }

    update_history(me, input);

    if (me->history_count < me->history_len) {
        me->history_count++;
    }

    me->last_output = calculate_average(me);
}

static float avg_get_output(filter_t *me)
{
    if (!filter_is_valid(me)) {
        return 0.0f;
    }

    return me->last_output;
}

static void kalman_update(filter_t *me, float input)
{
    float p_pred;

    if (!filter_is_valid(me)) {
        return;
    }

    if (me->kalman_p <= 1.0f && me->kalman_x == 0.0f) {
        me->kalman_x = input;
        me->last_output = input;
        me->kalman_p = 1.0f;
        return;
    }

    p_pred = me->kalman_p + me->kalman_q;
    me->kalman_k = p_pred / (p_pred + me->kalman_r);
    me->kalman_x = me->kalman_x + me->kalman_k * (input - me->kalman_x);
    me->kalman_p = (1.0f - me->kalman_k) * p_pred;
    me->last_output = me->kalman_x;
}

static float kalman_get_output(filter_t *me)
{
    if (!filter_is_valid(me)) {
        return 0.0f;
    }

    return me->kalman_x;
}

int Filter_Init(filter_t *me, filter_type_t type, float cutoff_freq, float sample_rate)
{
    static filter_ops_t avg_ops = {
        .update = avg_update,
        .get_output = avg_get_output
    };
    static filter_ops_t kalman_ops = {
        .update = kalman_update,
        .get_output = kalman_get_output
    };
    int i;

    if (me == NULL) {
        return -1;
    }

    me->type = type;
    me->cutoff_freq = cutoff_freq;
    me->sample_rate = sample_rate;
    me->history_len = FILTER_MOVING_AVG_LEN;
    me->history_count = 0;
    me->last_output = 0.0f;
    me->kalman_q = cutoff_freq;
    me->kalman_r = sample_rate;
    if (me->kalman_q <= 0.0f) {
        me->kalman_q = 0.01f;
    }
    if (me->kalman_r <= 0.0f) {
        me->kalman_r = 1.0f;
    }
    me->kalman_p = 1.0f;
    me->kalman_x = 0.0f;
    me->kalman_k = 0.0f;

    me->history = me->history_buf;

    for (i = 0; i < me->history_len; i++) {
        me->history[i] = 0.0f;
    }

    if (type == Fiter_Type_KF) {
        me->ops = &kalman_ops;
    } else {
        me->ops = &avg_ops;
    }
    me->magic = FILTER_MAGIC;

    return 0;
}

void Filter_update(filter_t *me, float input)
{
    if (!filter_is_valid(me) || (me->ops == NULL) || (me->ops->update == NULL)) {
        return;
    }

    me->ops->update(me, input);
}

float Filter_get_output(filter_t *me)
{
    if (!filter_is_valid(me) || (me->ops == NULL) || (me->ops->get_output == NULL)) {
        return 0.0f;
    }

    return me->ops->get_output(me);
}

void Filter_deinit(filter_t *me)
{
    if (me == NULL) {
        return;
    }

    me->history = NULL;
    me->history_len = 0;
    me->history_count = 0;
    me->last_output = 0.0f;
    me->ops = NULL;
    me->magic = 0;
}