#ifndef __PID_H__
#define __PID_H__

#include "struct_typedef.h"
enum PID_MODE
{
    PID_POSITION = 0, PID_DELTA
};

typedef struct
{
    uint8_t mode;

    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;
    fp32 max_iout;

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

    /*...other...*/
    fp32 measure;

} pid_type_def;

extern void PID_init(pid_type_def* pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_calc(pid_type_def* pid, fp32 ref, fp32 set);
extern void PID_clear(pid_type_def* pid);

#endif
