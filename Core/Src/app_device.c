/**
 * @file app_device.c
 * @brief Device services — display / broadcast / UART / keyboard / music
 *
 * Each function is guarded by APP_ENABLE_* switch in app.h.
 * Set to 0 to disable the module at compile time.
 */
#include "app.h"

/* ========================================================================
 * Display — OLED/LCD (100ms)
 * ======================================================================== */
#if APP_ENABLE_DISPLAY
void AppDisplay_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        DisplayService_ShowText(0, 0, "Hello World!");
        osDelay(100);
    }
}
#endif

/* ========================================================================
 * Broadcast — Bluetooth broadcast (50ms)
 * ======================================================================== */
#if APP_ENABLE_BROADCAST
void AppBroadcast_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        char *msg = "HelloFromStation\n";
        HAL_UART_Transmit(&huart4, (uint8_t *)msg, strlen(msg), 100);
        osDelay(50);
    }
}
#endif

/* ========================================================================
 * Uart3Rx — UART3 frame receive (blocks on queue)
 * ======================================================================== */
#if APP_ENABLE_UART_FIFO
void AppUart3Rx_Task(void *argument)
{
    (void)argument;
    char frame[UART_FRAME_MAX_LEN];

    for (;;)
    {
        memset(frame, 0, sizeof(frame));
        if (xQueueReceive(uart3_frame_queue, frame, portMAX_DELAY) == pdPASS)
        {
            /* TODO: parse frame data */
        }
    }
}

/* ========================================================================
 * Uart4Rx — UART4 frame receive (blocks on queue)
 * ======================================================================== */
void AppUart4Rx_Task(void *argument)
{
    (void)argument;
    char frame[UART_FRAME_MAX_LEN];

    for (;;)
    {
        memset(frame, 0, sizeof(frame));
        if (xQueueReceive(uart4_frame_queue, frame, portMAX_DELAY) == pdPASS)
        {
            /* TODO: parse frame data */
        }
    }
}
#endif

/* ========================================================================
 * KeyScan — keyboard scan
 * ======================================================================== */
#if APP_ENABLE_KEYBOARD

#define ROW_NUM  4
#define COL_NUM  4

static const char KEY_MAP[ROW_NUM][COL_NUM] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

static const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} ROWS[ROW_NUM] = {
    {MATRIX_ROW1_GPIO_Port, MATRIX_ROW1_Pin},
    {MATRIX_ROW2_GPIO_Port, MATRIX_ROW2_Pin},
    {MATRIX_ROW3_GPIO_Port, MATRIX_ROW3_Pin},
    {MATRIX_ROW4_GPIO_Port, MATRIX_ROW4_Pin}
};

static const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} COLS[COL_NUM] = {
    {MATRIX_COL1_GPIO_Port, MATRIX_COL1_Pin},
    {MATRIX_COL2_GPIO_Port, MATRIX_COL2_Pin},
    {MATRIX_COL3_GPIO_Port, MATRIX_COL3_Pin},
    {MATRIX_COL4_GPIO_Port, MATRIX_COL4_Pin}
};

void AppKeyScan_Task(void *argument)
{
    (void)argument;
    static TickType_t xLastTick[ROW_NUM][COL_NUM] = {0};
    static uint8_t keyState[ROW_NUM][COL_NUM] = {0};
    char keyChar = 0;

    for (;;)
    {
        for (int row = 0; row < ROW_NUM; row++)
        {
            HAL_GPIO_WritePin(ROWS[row].port, ROWS[row].pin, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(1));

            for (int col = 0; col < COL_NUM; col++)
            {
                GPIO_PinState state = HAL_GPIO_ReadPin(COLS[col].port, COLS[col].pin);

                switch (keyState[row][col])
                {
                case 0:
                    if (state == GPIO_PIN_RESET)
                    {
                        keyState[row][col] = 1;
                        xLastTick[row][col] = xTaskGetTickCount();
                    }
                    break;

                case 1:
                    if ((xTaskGetTickCount() - xLastTick[row][col]) > DEBOUNCE_TICKS)
                    {
                        if (state == GPIO_PIN_RESET)
                        {
                            keyChar = KEY_MAP[row][col];
                            xQueueSend(queue_keyHandle, &keyChar, 0);
                            keyState[row][col] = 2;
                        }
                        else
                        {
                            keyState[row][col] = 0;
                        }
                    }
                    break;

                case 2:
                    if (state == GPIO_PIN_SET)
                    {
                        keyState[row][col] = 0;
                    }
                    break;
                }
            }

            HAL_GPIO_WritePin(ROWS[row].port, ROWS[row].pin, GPIO_PIN_SET);
            taskYIELD();
        }
        vTaskDelay(pdMS_TO_TICKS(KEY_SCAN_INTERVAL_MS));
    }
}
#endif

/* ========================================================================
 * Buzzer — music player (disabled by default)
 * ======================================================================== */
#if APP_ENABLE_MUSIC
void AppBuzzer_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        /* Enable: PlayNote(M1, 100); */
        osDelay(1);
    }
}
#endif

/* ========================================================================
 * Reserved — 运动序列测试
 *
 * 上电 3s + 陀螺仪校准后依次执行:
 *   直行 1000mm → 右转 90° → 直行 100mm → 左转 90°
 *   → 100mm/s 巡航 10s → 1000mm/s 巡航 3s。
 * 距离段阻塞等待 car_control.oprate_done; 巡航段按时间 osDelay。
 * 全部完成后 LED2 常亮。
 * 方向约定 (与 car_control.c 一致): 负角 = CW(右转), 正角 = CCW(左转)。
 * 距离段任一超时 → 强制停车 + LED2 闪烁。
 * ======================================================================== */
#if APP_ENABLE_RESERVED

/* 单段动作最大允许时间 (ms)。MAX_V_ANGLE=10°/s 时 90° 旋转约 9s,
 * MAX_V_REAL≈452mm/s 时 1000mm 约 3s, 15s 留足裕量。
 * 若进一步调低 MAX_V_ANGLE, 需同步增大此值以免误判超时。 */
#define MOVE_TIMEOUT_MS     15000U

/* 阻塞等待当前运动指令完成。oprate_done 由下一次 Set_Car_Control()
 * (内部 clear_car_control) 清零, 完成时由 Car_Control_Update_Input() 置 1。
 * 超时返回 false 并强制停车 (防止 spin 因 yaw 未更新而无限旋转)。 */
static bool wait_operation_done(uint32_t timeout_ms)
{
    uint32_t waited = 0;
    while (!car_control.oprate_done)
    {
        osDelay(10);
        waited += 10;
        if (waited >= timeout_ms)
        {
            Set_Car_Control(0.0F, 0.0F, 0.0F);  /* 强制停车 */
            return false;
        }
    }
    return true;
}

void AppReserved_Task(void *argument)
{
    (void)argument;

    osDelay(3000);  /* 等系统初始化 */

#if APP_ENABLE_IMU
    /* 等陀螺仪零偏校准完成: 校准前 car_state.yaw 冻结, SPIN 永不结束。
     * 最多等 5s, 超时仍继续 (单段超时保护会兜底强制停车)。 */
    {
        uint32_t cal_waited = 0;
        while (!IMU_IsCalibrated() && cal_waited < 5000U)
        {
            osDelay(50);
            cal_waited += 50;
        }
    }
#endif

    /* ====== Step 1: 直行 1000mm ====== */
    Set_Car_Control(1000.0F, 0.0F, 0.0F);
    if (!wait_operation_done(MOVE_TIMEOUT_MS)) goto test_fault;
    osDelay(500);   /* 段间停顿, 让动量/姿态稳定 */

    /* ====== Step 2: 右转 90° (负角 = CW) ====== */
    Set_Car_Control(0.0F, 0.0F, -90.0F);
    if (!wait_operation_done(MOVE_TIMEOUT_MS)) goto test_fault;
    osDelay(500);

    /* ====== Step 3: 直行 100mm ====== */
    Set_Car_Control(100.0F, 0.0F, 0.0F);
    if (!wait_operation_done(MOVE_TIMEOUT_MS)) goto test_fault;
    osDelay(500);

    /* ====== Step 4: 左转 90° (正角 = CCW) ====== */
    Set_Car_Control(0.0F, 0.0F, 90.0F);
    if (!wait_operation_done(MOVE_TIMEOUT_MS)) goto test_fault;
    osDelay(500);

    /* ====== Step 5: 100mm/s 巡航前进 10s ======
     * 巡航靠 g_cruise_speed: 前几步结束后 mode 已为 STOP,
     * Car_Control_Update_Output 的 default 分支每周期施加该速度。 */
    g_cruise_speed = 100.0F;
    osDelay(10000);

    /* ====== Step 6: 1000mm/s 巡航前进 3s ======
     * 注意: 1000mm/s 超过 MAX_V_REAL(≈452mm/s), 会被 Set_Car_Attitude 限幅。 */
    g_cruise_speed = 1000.0F;
    osDelay(3000);

    g_cruise_speed = 0.0F;  /* 关闭巡航, 控制环 default 分支下一周期停车 */

    /* ====== 测试完成, LED2 常亮 ====== */
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    vTaskDelete(NULL);

test_fault:
    /* 某段动作超时, 已强制停车; LED2 闪烁提示故障 */
    for (;;)
    {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        osDelay(200);
    }
}
#endif /* APP_ENABLE_RESERVED */
