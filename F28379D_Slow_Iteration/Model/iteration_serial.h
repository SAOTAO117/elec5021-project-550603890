#ifndef ITERATION_SERIAL_H
#define ITERATION_SERIAL_H

#include "rtwtypes.h"

void iteration_serial_frame_step(int16_T frame[7], uint16_T *valid);
void iteration_serial_result_step(const int16_T result[6]);
void pwm_replay_step(int16_T duty[3], uint16_T *active, uint16_T *index);

#endif
