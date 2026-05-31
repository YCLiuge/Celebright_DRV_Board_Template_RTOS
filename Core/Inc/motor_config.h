/**
 * @file motor_config.h
 * @brief 电机 PID 参数与限幅 —— 4 电机增量式速度环
 */

#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

#include "platform_config.h"

/* 占空比 100% 对应 PID 输出的 1000（提高计算精度） */
#define ZOOM_PID_TO_DUTY        0.001F

/* 电机输出最大 PWM 占空比 */
#define MAX_MOTOR_DUTY          0.3F

/* ====== 增量式 PID 增量限幅 ====== */
#define LIMIT_INC_LF            0.5F * MAX_MOTOR_DUTY / ZOOM_PID_TO_DUTY
#define LIMIT_INC_LR            0.5F * MAX_MOTOR_DUTY / ZOOM_PID_TO_DUTY
#define LIMIT_INC_RF            0.5F * MAX_MOTOR_DUTY / ZOOM_PID_TO_DUTY
#define LIMIT_INC_RR            0.5F * MAX_MOTOR_DUTY / ZOOM_PID_TO_DUTY

/* ====== 位置式 PID 输出限幅 ====== */
#define LIMIT_POS_LF            (MAX_MOTOR_DUTY * 1000)
#define LIMIT_POS_LR            (MAX_MOTOR_DUTY * 1000)
#define LIMIT_POS_RF            (MAX_MOTOR_DUTY * 1000)
#define LIMIT_POS_RR            (MAX_MOTOR_DUTY * 1000)

/* ====== 位置式 PID 积分限幅 ====== */
#define LIMIT_ITGR_MAX          1000000000.0F
#define LIMIT_ITGR_LF           LIMIT_ITGR_MAX
#define LIMIT_ITGR_LR           LIMIT_ITGR_MAX
#define LIMIT_ITGR_RF           LIMIT_ITGR_MAX
#define LIMIT_ITGR_RR           LIMIT_ITGR_MAX

/* ====== 电机 PID 参数 (P/I/D) ====== */
#define P_LF            0.02F
#define P_LR            0.02F
#define P_RF            0.02F
#define P_RR            0.02F

#define I_LF            0.01F
#define I_LR            0.01F
#define I_RF            0.01F
#define I_RR            0.01F

#define D_LF            0.015F
#define D_LR            0.015F
#define D_RF            0.015F
#define D_RR            0.015F

#endif /* __MOTOR_CONFIG_H__ */
