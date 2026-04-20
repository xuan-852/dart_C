#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct filter filter_t;

// Filter type enum
typedef enum {//滤波器类型
    Fiter_Type_LPF = 0,
    Fiter_Type_HPF = 1
} filter_type_t;

// Filter operation table
typedef struct {//滤波器操作函数表
    void (*update)(filter_t *me, float input);
    float (*get_output)(filter_t *me);
} filter_ops_t;

// Filter base object
struct filter {//滤波器基对象
    filter_ops_t *ops;
    filter_type_t type;
    float cutoff_freq;
    float sample_rate;
    float *history;
    int history_len;

    // Internal state for 3-sample moving average
    float last_output;
};

int Filter_Init(filter_t *me, filter_type_t type, float cutoff_freq, float sample_rate);
void Filter_update(filter_t *me, float input);
float Filter_get_output(filter_t *me);
void Filter_deinit(filter_t *me);

#ifdef __cplusplus
}
#endif

#endif
