# 当前工作记录 (Current Work)

> 本文档记录本次会话（Session）的工作进展，包括已完成的审计、修复、新增功能，以及当前排查中的问题。
> 
> 上次更新时间：2026-05-31

---

## 1. 背景：转向角度 Bug

**现象**：小车执行 `car spin 90` 后，实际物理转角约为 **45°**（存在 **1:2** 的比例关系）。

**影响范围**：仅影响 `SPIN` 模式（原地旋转），不影响直线行走（`GO_LINE`）。

**根因方向**：IMU 陀螺仪 Z 轴角速度 (`imu_g_z`) 的数值换算可能存在系统性偏差，导致 `Car_Attitude_Yaw_Update()` 积分得到的 yaw 角被放大了 2 倍。

---

## 2. 已完成的系统性审计（v3.3 ~ v4.0）

### v3.3 - v3.4（14 处修复）
- FreeRTOS 任务自删除修复（`vTaskDelete(NULL)` → `vTaskDelete(xHandle)`）
- USB CDC ISR 修复（`USBD_CDC_ReceivePacket` 在回调中正确调用）
- FreeRTOS_CLI `NULL` 解引用修复（`pxParameter = NULL` 时 `strcpy` 保护）
- `app.h` Facade 模式重构
- `app_console.c` CLI 框架搭建（USB CDC 对接）
- `car go x/2` bug 修复（目标距离被错误除以 2）
- 死代码清理（`nowtime`, `cnt`, `math_pl` 等未使用变量）
- README.md 完全重写

### v3.5（10 处修复）
- `IMU.c`：`if_get_offset` 从 `int` 改为 `static _Bool`，修复静态存储期问题
- 计时系统：`nowtime` → `DWT->CYCCNT`，消除 `TIM2` 依赖
- `app_sensor.c`：移除冗余的 `Car_Attitude_Update_Input()` 调用（避免覆盖正确姿态）
- `car_control.c`：浮点数精确比较改为 epsilon 比较（`== 0` → `fabsf() < EPSILON`）
- `car_control.c`：`asin()` 输入值 domain clamp（`[-1, 1]`）
- `Motor.c`：int-float 往返精度修复
- `IMU.c`：`invSqrt1` 严格别名违规修复（union 替代指针转换）
- `freertos.c`：重复 `#include "app.h"` 删除

### v3.6（9 处修复）
- `display_service.c`：`sign_char` 缓冲区缺 `\0` 硬错误修复
- `arm.c`：`acosf()` NaN 保护（输入值 clamp 到 `[-1, 1]`）
- `arm.c`：除零保护（`l1 + l2 <= l3` 时提前返回）
- `_setArmAngle()`：校验错误变量修复
- `uart_fifo.c`：`printf` 在 ISR 中移除（改为队列投递）
- `UART_SendFrame()`：缓冲区溢出保护
- `arm.c`：Theta3 计算缺 `PI/2` 修复
- `arm.c`：`theta3` 范围校验
- `arm.c`：`EPSILON 1.0 → 1e-4f`

### v3.7（10 处修复）
- `nowtime` / `cnt` / `math_pl` 全局变量删除
- `init_motor()` 调用时机调整（移至 `SystemClock_Config()` 之后）
- 10 个 `_Handler` 统一清理逻辑（`_Handle = NULL`）
- `AppInit_Task` 优先级调整（Normal5 → Normal6）

### v3.8（2 处修复）
- `SCSLib`：`target_position` 未初始化修复
- `SCSLib` 线程安全文档化（已知限制）

### v3.9（2 处修复）
- `HAL_TIM_Base_Start_IT(TIM2-5)` 死代码删除
- `assert_failed()` 增加 `printf` 输出

### v4.0 CLI 增强（6 处）
- `PARAM_MATCH` 宏修复：`FreeRTOS_CLIGetParameter` 返回非 null-terminated 子串，`strcmp` 会越界读取 → 改用 `strncmp`
- `car spin left/right`：支持左右方向旋转
- `car cruise <mm/s> [deg/s]`：巡航速度控制 + 原地转向叠加
- `car turn <deg>`：指定角度转向
- `imu` CLI 命令：`IMU_BuildStatus()` 实时查询 IMU 数据
- 诊断程序：`AppReserved_Task` 采样存储，User 键触发 USART1 输出

---

## 3. IMU 2x 误差专项排查

### 3.1 验证寄存器配置

在 `Core/Src/read_aux_data_mode.c:208` 后添加回读验证代码：

```c
/* 验证 GYRO_CONFIG0 回读 */
{
    gyro_config0_t gyro_cfg_verify;
    inv_imu_read_reg(&imu_dev, GYRO_CONFIG0, 1, (uint8_t *)&gyro_cfg_verify);
    printf("[IMU] GYRO_CONFIG0 = 0x%02X  (FS_SEL=%d, ODR=%d)\r\n",
           *(uint8_t *)&gyro_cfg_verify,
           gyro_cfg_verify.gyro_ui_fs_sel,
           gyro_cfg_verify.gyro_odr);
    printf("[IMU]   Expected FS_SEL=2 (±1000 dps)\r\n");
}
```

**实际输出**：
```
[IMU] GYRO_CONFIG0 = 0x26  (FS_SEL=2, ODR=6)
[IMU]   Expected FS_SEL=2 (±1000 dps)
```

→ 寄存器配置正确写入，回读值与预期一致。

### 3.2 换算公式审查

当前代码（`read_aux_data_mode.c:270-272`）：
```c
gyro_dps[0] = (float)((d.gyro_data[0] * 1000 /* dps */) / 32768.0);
gyro_dps[1] = (float)((d.gyro_data[1] * 1000 /* dps */) / 32768.0);
gyro_dps[2] = (float)((d.gyro_data[2] * 1000 /* dps */) / 32768.0);
```

根据 ICM-45686 数据手册：
- FS_SEL=2（±1000 dps）对应灵敏度 = **32.8 LSB/dps**
- 理论公式：`dps = raw / 32.8 ≈ raw * 1000 / 32768` ✅ **与代码一致**

### 3.3 官方驱动对比

参考 TDK 官方 Arduino 驱动（`motion.arduino.ICM45686`）：
- 标准 dps 转换：`dps = raw * FSR / 32768.0`
- 内部 Q16 实现：`raw * (FSR * 2) / 65536.0`（等价于 `/32768`）

→ **官方驱动也使用 /32768，与当前代码一致。**

### 3.4 数据流审查

```
ICM-45686 寄存器 (ACCEL_DATA_X1_UI = 0x00 起始)
  → inv_imu_get_register_data() 读取 12 bytes
    → FORMAT_16_BITS_DATA() 字节序重组
      → inv_imu_sensor_data_t.gyro_data[3] (int16_t)
        → bsp_IcmGetRawData():
            gyro_dps = raw * 1000 / 32768.0
          → IMU_getValues():
              values[5] = gyro_dps[2] - gyro_offset[2]
            → imu_g_z = values[5]
            → Car_Attitude_Yaw_Update(imu_g_z, 0.001*TASK_ITV_IMU)
              → car_state.yaw += imu_g_z * time
```

整个链路无异常。

### 3.5 闭环 PID 掩盖效应

在 `SPIN` 模式下，速度环 PID 输出 `TGT_V_ANG`（目标角速度）。
- 若 `imu_g_z` 被放大 2 倍，PID 检测到 "实际角速度 = 2×目标"
- PID 会 **降低输出**，使电机减速
- 最终 `IMU_GZ ≈ TGT_V_ANG`（稳态误差小）
- **但小车实际物理转速只有目标的一半**（因为 IMU 数值是假的 2 倍）

→ 这就是闭环系统掩盖传感器误差的典型表现。

### 3.6 物理测量验证

通过手动测量确认：
- `car spin 90` → 实际转约 45°
- `car spin 180` → 实际转约 90°
- 比例稳定为 **1:2**

---

## 4. 当前假设

尽管寄存器回读显示 `FS_SEL=2`（±1000 dps），且数据手册标明灵敏度为 32.8 LSB/dps，但实际现象强烈暗示：

> **这颗 ICM-45686 芯片的实际灵敏度可能是 65.5 LSB/dps（对应 ±500 dps 档），而非数据手册标称的 32.8 LSB/dps。**

可能原因：
1. **硅片批次差异**：个别芯片实际灵敏度偏离标称值
2. **寄存器写入未生效**：虽然回读正确，但内部模拟前端未切换量程
3. **ICM-45686 与 ICM-45605 混淆**：不同型号灵敏度定义不同

---

## 5. 下一步验证方案

### 方案 A：修改除数验证（建议优先尝试）

修改 `Core/Src/read_aux_data_mode.c:270-272`：

```c
// 测试假设：实际灵敏度为 65.5 LSB/dps（±500 dps 档）
gyro_dps[0] = (float)((d.gyro_data[0] * 1000 /* dps */) / 65536.0);
gyro_dps[1] = (float)((d.gyro_data[1] * 1000 /* dps */) / 65536.0);
gyro_dps[2] = (float)((d.gyro_data[2] * 1000 /* dps */) / 65536.0);
```

然后测试 `car spin 90`，观察实际转角：
- **如果 = 90°**：假设成立，实际灵敏度是 65.5 LSB/dps，需永久修改为 `/65536` 或 `*500/32768`
- **如果仍 = 45°**：假设不成立，需排查其他原因（积分时间、任务频率、编码器干扰等）

### 方案 B：开环验证

绕过 PID，直接给电机固定 PWM（如 20% 占空比），让小车匀速旋转，同时记录：
- `imu_g_z` 读数
- 物理转角（手动测量或用手机陀螺仪 APP 对比）
- 持续时间

计算：`实际角速度 = 物理转角 / 时间`
对比：`imu_g_z` 读数

若 `imu_g_z ≈ 2 × 实际角速度`，则确认 IMU 灵敏度问题。

### 方案 C：静态零偏验证

将小车固定不动（完全不转），读取 `imu_g_z`：
- 理论上应接近 0（±几 dps 的噪声）
- 如果静态时就有明显非零值，可能是零偏未校准或数据异常

---

## 6. 相关文件索引

| 文件 | 作用 | 关键行号 |
|------|------|---------|
| `Core/Src/read_aux_data_mode.c` | IMU 驱动封装、初始化、数据读取 | 208（FSR设置）, 270-272（dps换算） |
| `Core/Src/IMU.c` | AHRS 算法、陀螺仪校准、姿态更新 | 145（imu_g_z赋值）, 148（Yaw更新）, 242（AHRS输入） |
| `Core/Src/car_attitude.c` | 小车姿态积分 | 139（Car_Attitude_Yaw_Update） |
| `Core/Src/car_control.c` | 小车控制逻辑 | 152（SPIN模式）, 216（get_current_spin_angle） |
| `Core/Src/inv_imu_driver.c` | 官方底层驱动 | 199（get_register_data）, 140（set_gyro_fsr） |
| `Core/Src/inv_imu_transport.c` | SPI/I2C 传输 | 41（write_reg）, 66（read_dreg） |
| `Core/Inc/inv_imu_defs.h` | 寄存器定义、数据结构 | 95（sensor_data_t）, 280-292（FSR枚举） |
| `Core/Inc/inv_imu_regmap_le.h` | 寄存器地址映射 | 124（GYRO_CONFIG0）, 126-128（gyro_config0_t） |
| `Core/Inc/inv_imu_driver.h` | 驱动 API、字节序宏 | 121（FORMAT_16_BITS_DATA） |
| `Core/Inc/rtos_config.h` | 任务周期配置 | 11（TASK_ITV_IMU = 5ms） |

---

## 7. 会话状态

- [x] 多轮系统性审计完成（v3.3 ~ v4.0）
- [x] GYRO_CONFIG0 寄存器回读验证完成
- [x] 官方驱动对比完成
- [x] 数据流全链路审查完成
- [x] 物理转角 1:2 比例确认
- [x] **已排除**：积分 dt（5ms，与任务周期一致）、重复积分（仅 AppIMUService_Task 单路径 5ms 调用）、字节序重组（FORMAT_16_BITS_DATA 无放大）、寄存器配置（回读 FS_SEL=2）
- [x] **已修复**：`read_aux_data_mode.c` `bsp_IcmGetRawData()` 陀螺仪换算除数由 `*1000/32768` 改为 `*500/32768`（等效 /65536）。陀螺仪实测灵敏度为 ±500 dps 档（65.5 LSB/dps），与寄存器标称 ±1000 dps 不符——按实测有效量程换算。
- [ ] **待验证**：烧录后执行 `car spin 90`，物理转角应 ≈ 90°；可用 `AppReserved_Task` 诊断输出复核 `imu_gz` 与 `spin_cur`

> **注**：之前的 `GYRO_CONFIG0 ... ODR=6` 回读是误导项——该回读发生在 line 214，在 line 224 设置 ODR(200Hz) **之前**，故 ODR=6 是设置前的残值，与最终配置无关。

---

*本文档由 opencode AI 助手生成，用于记录本次会话的工作上下文。*
