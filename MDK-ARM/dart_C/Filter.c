#include "Filter.h"

static int16_t Filter_Median3_Int16(int16_t a, int16_t b, int16_t c)
{
  if (a > b) {
    int16_t t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    int16_t t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    int16_t t = a;
    a = b;
    b = t;
  }
  return b;
}

static void Filter_Init_Impl(Filter_base_t *me, const Filter_data_set_t *cfg)
{
  if (me == 0 || cfg == 0) {
    return;
  }

  me->data_set = *cfg;
  me->history[0] = 0;
  me->history[1] = 0;
  me->history[2] = 0;
  me->ready = 0U;
  me->value = 0.0;
}

static void Filter_DeInit_Impl(Filter_base_t *me)
{
  if (me == 0) {
    return;
  }

  me->history[0] = 0;
  me->history[1] = 0;
  me->history[2] = 0;
  me->ready = 0U;
  me->value = 0.0;
}

static double Filter_Update_Impl(Filter_base_t *me, int16_t sample)
{
  if (me == 0) {
    return (double)sample;
  }

  if (me->ready == 0U) {
    me->history[0] = sample;
    me->history[1] = sample;
    me->history[2] = sample;
    me->value = (double)sample;
    me->ready = 1U;
    return me->value;
  }

  me->history[0] = me->history[1];
  me->history[1] = me->history[2];
  me->history[2] = sample;

  {
    double input = (double)sample;
    if (me->data_set.useMedian3 != 0U) {
      input = (double)Filter_Median3_Int16(
        me->history[0],
        me->history[1],
        me->history[2]
      );
    }

    me->value = me->value + me->data_set.alpha * (input - me->value);
  }

  return me->value;
}

void Filter_Create(Filter_base_t *me, const Filter_data_set_t *cfg)
{
  if (me == 0) {
    return;
  }

  me->ob.Init = Filter_Init_Impl;
  me->ob.DeInit = Filter_DeInit_Impl;
  me->ob.Update = Filter_Update_Impl;

  if (cfg != 0) {
    me->ob.Init(me, cfg);
  }
}

void filter_median_lpf_init(filter_median_lpf_t *me, double alpha)
{
  Filter_data_set_t cfg;
  cfg.alpha = alpha;
  cfg.useMedian3 = 1U;
  Filter_Create(me, &cfg);
}

double filter_median_lpf_update(filter_median_lpf_t *me, int16_t sample)
{
  if (me == 0 || me->ob.Update == 0) {
    return (double)sample;
  }

  return me->ob.Update(me, sample);
}

void filter_median_lpf_deinit(filter_median_lpf_t *me)
{
  if (me == 0 || me->ob.DeInit == 0) {
    return;
  }

  me->ob.DeInit(me);
}
