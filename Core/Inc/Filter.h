#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct filter filter_t;

#define FILTER_MAGIC 0x46494C54u
#define FILTER_HISTORY_MAX_LEN 5

// Filter type enum
typedef enum {//滤波器类型
    Fiter_Type_LPF = 0,
    Fiter_Type_HPF = 1,
    Fiter_Type_KF = 2
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
    int history_count;
    uint32_t magic;
    float history_buf[FILTER_HISTORY_MAX_LEN];

    // Filter state
    float last_output;
    float kalman_q;
    float kalman_r;
    float kalman_p;
    float kalman_x;
    float kalman_k;
};

int Filter_Init(filter_t *me, filter_type_t type, float cutoff_freq, float sample_rate);
void Filter_update(filter_t *me, float input);//更新滤波器的buffer
float Filter_get_output(filter_t *me);//读取滤波后数值
void Filter_deinit(filter_t *me);

#ifdef __cplusplus
}
#endif

#endif
