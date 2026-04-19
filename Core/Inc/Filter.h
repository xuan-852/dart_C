#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct Filter_base_t Filter_base_t;
typedef struct Filter_ob_t Filter_ob_t;
typedef struct Filter_data_set_t Filter_data_set_t;

struct Filter_ob_t {
  void (*Init)(Filter_base_t *me, const Filter_data_set_t *cfg);
  void (*DeInit)(Filter_base_t *me);
  double (*Update)(Filter_base_t *me, int16_t sample);
};

struct Filter_data_set_t {
  double alpha;
  uint8_t useMedian3;
};

struct Filter_base_t {
  Filter_ob_t ob;
  Filter_data_set_t data_set;
  int16_t history[3];
  uint8_t ready;
  double value;
};

void Filter_Create(Filter_base_t *me, const Filter_data_set_t *cfg);

/* 兼容旧调用：避免改动 main.c 现有接口 */
typedef Filter_base_t filter_median_lpf_t;

void filter_median_lpf_init(filter_median_lpf_t *me, double alpha);
double filter_median_lpf_update(filter_median_lpf_t *me, int16_t sample);
void filter_median_lpf_deinit(filter_median_lpf_t *me);

#ifdef __cplusplus
}
#endif

#endif
