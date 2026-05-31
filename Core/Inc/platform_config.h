/**
 * @file platform_config.h
 * @brief 小车平台参数 —— 轮径、轮距、编码器线数、最大速度
 */

#ifndef __PLATFORM_CONFIG_H__
#define __PLATFORM_CONFIG_H__

/* 数学常量 */
#define RAD_TO_DEGREE   57.29578018F
#define DEGREE_TO_RAD   0.01745329238F

/* 电机个数：1=四轮, 0=两轮(前轮) */
#define USE_4_MOTOR             1

/* 编码器四倍频 */
#define USE_4_TIMES_ENCODER     1

/* 编码器：轮趣科技 GMR 编码器电机, 500线 */
#define ENC_LINES               500         /* 编码器线数 (PPR) */

/* 四倍频：CubeMX 需配置 TIMx EncoderMode = TI12, IC Polarity = BothEdge */
#define ENC_MULTIPLIER          4           /* 4x 计数 */

/* 减速比 1:28 (电机轴 : 轮轴) */
#define GEAR_RATIO              28

/* 每圈轮子编码器脉冲数 = ENC_LINES × ENC_MULTIPLIER × GEAR_RATIO */
#define ENC_EVERY_CIRCLE        ((float)(ENC_LINES * ENC_MULTIPLIER * GEAR_RATIO))

/* 轮径 mm */
#define WHEEL_DIR               72.0F

/* 轮周长 mm */
#define WHEEL_PERIMETER         226.194671F

/* 左右轮距的一半 mm（用 IMU 测算） */
#define FRAME_W_HALF            122.50F

/* 前后轮轴距的一半 mm */
#define FRAME_L_HALF            102.50F

/* 编码器速度 → 实际速度 换算系数 */
#define V_REAL_TO_ENC           (ENC_EVERY_CIRCLE / WHEEL_PERIMETER)

/* 电机目标编码器速度最大值 (enc/s) */
#define MAX_V_ENC               (2 * ENC_EVERY_CIRCLE)

/* 电机最大实际速度 (mm/s) */
#define MAX_V_REAL              (MAX_V_ENC / V_REAL_TO_ENC)

/* 最大角速度 (degree/s)
 * 调低可减小 spin 接近目标时的实际转速, 从而减小动量滑行造成的过冲
 * (实测 20°/s 时停在目标后仍滑 ~2-3°)。这是过冲调节的主旋钮;
 * 若仍过冲可继续调低, 或提高 control_config.h 的 D_SPIN 加强末端制动。
 * 注意: 过低会使 90° 旋转耗时超过 AppReserved_Task 的 MOVE_TIMEOUT_MS(15s)
 *       而误判超时。10°/s 时 90°≈9s, 安全。 */
#define MAX_V_ANGLE             10.0F

#endif /* __PLATFORM_CONFIG_H__ */
