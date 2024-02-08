#ifndef BSP_IMU_PWM_H
#define BSP_IMU_PWM_H
#include "struct_typedef.h"

extern void imu_pwm_set(uint16_t pwm);
extern void imu_temp_control_task(void const * argument);

#endif
