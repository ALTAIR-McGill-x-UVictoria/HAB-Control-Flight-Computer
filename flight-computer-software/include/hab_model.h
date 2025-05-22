#ifndef HAB_WEIGHTS_H
#define HAB_WEIGHTS_H

#include <stdint.h>
#include <math.h>

// Input feature indices
#define IDX_X_ACCEL 0
#define IDX_Y_ACCEL 1
#define IDX_X_VEL 2
#define IDX_Y_VEL 3
#define IDX_X_POS 4
#define IDX_Y_POS 5
#define IDX_ROLL 6
#define IDX_PITCH 7
#define IDX_YAW 8
#define IDX_X_ANG_VEL 9
#define IDX_Y_ANG_VEL 10
#define IDX_Z_ANG_VEL 11
#define IDX_PREV_ACTION_0 12
#define IDX_PREV_ACTION_1 13
#define IDX_ACTION_DIFF_0 14
#define IDX_ACTION_DIFF_1 15

// The TFLite model data - properly declared as extern
#ifdef __cplusplus
extern "C" {
#endif
extern const unsigned char hab_model_tflite[];
extern const unsigned int hab_model_tflite_len;
#ifdef __cplusplus
}
#endif

#endif // HAB_WEIGHTS_H