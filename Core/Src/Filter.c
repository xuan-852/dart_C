#include "Filter.h"

#include <stddef.h>

static const int FILTER_MOVING_AVG_LEN = 3;

static int filter_is_valid(const filter_t *me)
{
    return (me != NULL) && (me->magic == FILTER_MAGIC);
}

static float get_smoothing_alpha(const filter_t *me)//进行限制，确保alpha在0到1之间
{
    float alpha;

    if (!filter_is_valid(me)) {
        return 1.0f;
    }

    alpha = me->cutoff_freq;//截止频率
    if (alpha <= 0.0f) {
        alpha = 1.0f;
    }
    if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    return alpha;
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

    if (!filter_is_valid(me) || (me->history == NULL) || (me->history_len <= 0) || (me->history_len > FILTER_HISTORY_MAX_LEN)) {
        return 0.0f;
    }

    for (i = 0; i < me->history_len; i++) {
        sum += me->history[i];
    }

    return sum / (float)me->history_len;
}

static void avg_update(filter_t *me, float input)
{
    float avg_value;
    float alpha;

    if (!filter_is_valid(me) || (me->history == NULL) || (me->history_len <= 0) || (me->history_len > FILTER_HISTORY_MAX_LEN)) {
        return;
    }

    update_history(me, input);
    avg_value = calculate_average(me);
    alpha = get_smoothing_alpha(me);
    me->last_output = me->last_output + alpha * (avg_value - me->last_output);
}

static float avg_get_output(filter_t *me)
{
    if (!filter_is_valid(me)) {
        return 0.0f;
    }

    return me->last_output;
}

int Filter_Init(filter_t *me, filter_type_t type, float cutoff_freq, float sample_rate)
{
    static filter_ops_t avg_ops = {
        .update = avg_update,
        .get_output = avg_get_output
    };
    int i;

    if (me == NULL) {
        return -1;
    }

    me->type = type;
    me->cutoff_freq = cutoff_freq;
    me->sample_rate = sample_rate;
    me->history_len = FILTER_MOVING_AVG_LEN;
    me->last_output = 0.0f;

    me->history = me->history_buf;

    for (i = 0; i < me->history_len; i++) {
        me->history[i] = 0.0f;
    }

    me->ops = &avg_ops;
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
    me->last_output = 0.0f;
    me->ops = NULL;
    me->magic = 0;
}
