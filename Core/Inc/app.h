/**
 * @file app.h
 * @brief Unified Facade header — single entry point for all user modules
 *
 * Core architecture principle: high cohesion, low coupling.
 * All CubeMX-generated files (main.c, freertos.c, etc.) and user modules
 * access cross-module APIs through this file.
 *
 * Include rules:
 *  - CubeMX-generated .c files → only #include "app.h"
 *  - User module .c files     → may #include "app.h" or individual headers
 *  - Cross-module shared types/variables → declared in this file
 *
 * Module Trimming:
 *  Set APP_ENABLE_* to 0 to disable optional modules at compile time.
 *  This removes the code from the build (saves FLASH).
 *  For maximum RAM savings, also delete the corresponding task in CubeMX.
 */

#ifndef __APP_H__
#define __APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* CMSIS RTOS2 + FreeRTOS types (osThreadId_t, TickType_t, vTaskDelayUntil, etc.) */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/* ========================================================================
 * Module Enable Switches (1 = enabled, 0 = disabled)
 *
 * Disabling a module:
 *   - Removes its #include from this file
 *   - Removes its function declarations from this file
 *   - Guards its freertos.c handler body (disabled task calls vTaskDelete(NULL))
 *   - Guards its app_device.c function body
 *
 * To fully reclaim RAM, also delete the task in CubeMX → FreeRTOS → Tasks.
 * ======================================================================== */
#define APP_ENABLE_SYSMON       1   /* System monitor (heap/stack display) */
#define APP_ENABLE_CONSOLE      1   /* CLI console via USB CDC */
#define APP_ENABLE_QSPI         1   /* QSPI Flash + LittleFS filesystem */
#define APP_ENABLE_DISPLAY      1   /* OLED/LCD display */
#define APP_ENABLE_KEYBOARD     1   /* 4x4 matrix keyboard */
#define APP_ENABLE_MUSIC        0   /* Buzzer music player */
#define APP_ENABLE_UART_FIFO    1   /* UART frame protocol (UART3/UART4) */
#define APP_ENABLE_ROBOTIC_ARM  1   /* Robotic arm (arm/ARobot/roboticArm/holder2D/SCSLib) */
#define APP_ENABLE_IMU          1   /* IMU sensor (ICM-45686) */
#define APP_ENABLE_LEDBLINK     1   /* LED blink task */
#define APP_ENABLE_BROADCAST    0   /* Bluetooth broadcast via UART4 */
#define APP_ENABLE_RESERVED     0   /* 自主行驶测试任务: 开启会在上电后接管小车,
                                     * 与 CLI(car cruise/go/...) 冲突。默认关闭,
                                     * 仅在做整车动作测试时临时置 1。 */

/* Display type: DISPLAY_TYPE_OLED or DISPLAY_TYPE_LCD */
#define DISPLAY_TYPE_OLED  0
#define DISPLAY_TYPE_LCD   1
#define APP_DISPLAY_TYPE   DISPLAY_TYPE_OLED

/* ========================================================================
 * Common Layer — utilities and global config
 * ======================================================================== */
#include "config.h"
#include "delay.h"
#include "sys_time.h"

/* ========================================================================
 * Control Layer — PID → PWM → Motor → Car motion control
 * (Always enabled — core system)
 * ======================================================================== */
#include "pid.h"
#include "PWM.h"
#include "Motor.h"
#include "car_attitude.h"
#include "car_control.h"

/* ========================================================================
 * Sensor Layer — IMU sensor driver and AHRS
 * ======================================================================== */
#if APP_ENABLE_IMU
#include "inv_imu_defs.h"
#include "inv_imu_version.h"
#include "inv_imu_transport.h"
#include "inv_imu.h"
#include "inv_imu_driver.h"
#include "read_aux_data_mode.h"
#include "IMU.h"
#endif

/* ========================================================================
 * Device Layer — display / input / communication / music
 *
 * Note: pure data headers (oledfont.h, oledpicture_V0.2.h, lcd_fonts.h,
 * lcd_image.h, MusicScore.h) are NOT included here. They are included
 * directly by their respective .c files to avoid duplicate symbols.
 * ======================================================================== */
#if APP_ENABLE_UART_FIFO
#include "uart_fifo.h"
#endif

#if APP_ENABLE_DISPLAY
#include "oled_spi_V0.2.h"
#include "lcd_st7789.h"
#include "lcd_model.h"
#endif

#if APP_ENABLE_KEYBOARD
#include "keyboard.h"
#endif

#if APP_ENABLE_MUSIC
#include "MusicPlayer.h"
#endif

/* ========================================================================
 * Robot Layer — robotic arm / servo
 * ======================================================================== */
#if APP_ENABLE_ROBOTIC_ARM
#include "SCServo.h"
#include "arm.h"
#include "roboticArm.h"
#include "ARobot.h"
#include "holder2D.h"
#endif

/* ========================================================================
 * Cross-module Shared Data
 * ======================================================================== */

/* IMU sensor data */
#if APP_ENABLE_IMU
extern float ypr[3];
extern float motion6[7];
extern float imu_g_z;
#endif

/* Software PWM duty cycles (defined in PWM.c, used by TIM6 ISR and Motor) */
extern volatile uint16_t dutyA;
extern volatile uint16_t dutyB;
extern volatile uint16_t dutyC;
extern volatile uint16_t dutyD;
extern volatile uint16_t pwm_counter;

/* FreeRTOS cross-module handles */
#if APP_ENABLE_UART_FIFO
extern QueueHandle_t uart3_frame_queue;
extern QueueHandle_t uart4_frame_queue;
#endif
#if APP_ENABLE_KEYBOARD
extern osMessageQueueId_t queue_keyHandle;
#endif

/* Console StreamBuffer (USB CDC RX) */
#if APP_ENABLE_CONSOLE
#include "stream_buffer.h"
extern StreamBufferHandle_t console_rx_stream;
#endif

/* Task handles (for SysMon watermark monitoring) */
#if APP_ENABLE_SYSMON || APP_ENABLE_DISPLAY
extern osThreadId_t DisplayHandle;
extern osThreadId_t CarControlHandle;
extern osThreadId_t IMUServiceHandle;
#endif
#if APP_ENABLE_SYSMON
extern osThreadId_t ConsoleHandle;
#endif

/* ========================================================================
 * Cross-module Shared Types
 * ======================================================================== */

/**
 * @brief Shared car state — used by car_attitude and car_control
 *        to eliminate circular dependency between the two modules.
 */
typedef struct {
    float yaw;               /* Yaw angle [0, 360) degrees */
    float yaw_all;           /* Accumulated yaw (including circles) */
    int   yaw_circles;       /* Yaw overflow count, positive = 360→0 (CCW) */
    float v_line;            /* Current linear velocity mm/s */
    float v_angle;           /* Current angular velocity deg/s */
    float v_line_target;     /* Target linear velocity mm/s */
    float v_angle_target;    /* Target angular velocity deg/s */
    _Bool flag_stop;         /* Car stopped flag */
} car_state_t;

extern car_state_t car_state;
extern float g_cruise_speed;  /* 巡航速度 mm/s, 0=关闭 — 运动完成后自动恢复 */

/* ========================================================================
 * Display Abstraction Layer
 * ======================================================================== */
#if APP_ENABLE_DISPLAY
void DisplayService_Init(void);
void DisplayService_Clear(void);
void DisplayService_ShowText(uint16_t x, uint16_t y, const char *text);
void DisplayService_ShowNumber(uint16_t x, uint16_t y, int32_t num, uint8_t len);
#endif

/* ========================================================================
 * FreeRTOS Task Service Functions (Thin Delegation targets)
 *
 * Called by freertos.c Xxx_Handler, implemented in app_*.c
 * ======================================================================== */
void AppInit_Task(void *argument);
void AppCarControl_Task(void *argument);
#if APP_ENABLE_IMU
void AppIMUService_Task(void *argument);
#endif
#if APP_ENABLE_DISPLAY
void AppDisplay_Task(void *argument);
#endif
#if APP_ENABLE_BROADCAST
void AppBroadcast_Task(void *argument);
#endif
#if APP_ENABLE_UART_FIFO
void AppUart3Rx_Task(void *argument);
void AppUart4Rx_Task(void *argument);
#endif
#if APP_ENABLE_KEYBOARD
void AppKeyScan_Task(void *argument);
#endif
#if APP_ENABLE_MUSIC
void AppBuzzer_Task(void *argument);
#endif
#if APP_ENABLE_RESERVED
void AppReserved_Task(void *argument);
#endif

#if APP_ENABLE_SYSMON
void AppSysMon_Task(void *argument);
void AppSysMon_UpdateSnapshot(void);
size_t AppSysMon_BuildSystemSummary(char *buf, size_t buf_len);
size_t AppSysMon_BuildRuntimeStats(char *buf, size_t buf_len);
#endif

#if APP_ENABLE_CONSOLE
void AppConsole_Task(void *argument);
void AppConsole_Init(void);
#endif

/* ========================================================================
 * QSPI Flash Service Layer (W25Q256 + LittleFS)
 * ======================================================================== */
#if APP_ENABLE_QSPI
bool        AppQSPI_Init(void);
bool        AppQSPI_IsPresent(void);
bool        AppQSPI_Mount(void);
void        AppQSPI_Umount(void);
bool        AppQSPI_IsMounted(void);
bool        AppQSPI_Format(void);
size_t      AppQSPI_ListDir(char *buf, size_t buf_len);
size_t      AppQSPI_ReadFile(char *buf, size_t buf_len, const char *path, uint32_t max_bytes);
const char* AppQSPI_GetName(void);
uint32_t    AppQSPI_ReadID(void);
size_t      AppQSPI_BuildInfo(char *buf, size_t buf_len);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __APP_H__ */
