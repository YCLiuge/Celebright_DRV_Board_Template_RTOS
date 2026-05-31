/**
 * @file app_console.c
 * @brief CLI Console via USB CDC Virtual COM Port
 *
 * Uses FreeRTOS+CLI for command parsing.
 * RX: USB CDC -> StreamBuffer (ISR-safe) -> Console Task
 * TX: Console Task -> CDC_Transmit_FS
 *
 * Commands: help / tasks / stats / sys / flash / car
 */
#include "app_console.h"

#if APP_ENABLE_CONSOLE

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"
#include "stream_buffer.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#define CONSOLE_OUTPUT_MAX_LEN  768U
#define CONSOLE_PROMPT          "> "

/* FreeRTOS_CLIGetParameter returns a pointer into the raw command string;
 * the parameter is NOT null-terminated at its boundary — e.g., for
 * "car go 1000", parameter 1 points to "go 1000", not "go\0".
 * Use this macro instead of strcmp() to safely compare against literals. */
#define PARAM_MATCH(p, plen, literal) \
    ((p) != NULL && (plen) > 0 && \
     (size_t)(plen) == (sizeof(literal) - 1U) && \
     strncmp((p), (literal), (size_t)(plen)) == 0)

static char g_cli_output[CONSOLE_OUTPUT_MAX_LEN];
static bool g_cli_registered = false;

static void Console_Write(const char *str);
static void Console_WritePrompt(void);
static void Console_ProcessLine(char *line);
static void Console_RegisterCommands(void);

static BaseType_t CLI_TaskListCommand(char *buf, size_t len, const char *cmd);
static BaseType_t CLI_RunTimeStatsCommand(char *buf, size_t len, const char *cmd);
static BaseType_t CLI_SysCommand(char *buf, size_t len, const char *cmd);
static BaseType_t CLI_FlashCommand(char *buf, size_t len, const char *cmd);
static BaseType_t CLI_CarCommand(char *buf, size_t len, const char *cmd);
#if APP_ENABLE_IMU
static BaseType_t CLI_ImuCommand(char *buf, size_t len, const char *cmd);
#endif

void AppConsole_Init(void)
{
    Console_RegisterCommands();
}

void AppConsole_Task(void *argument)
{
    char line_buf[96];
    uint8_t ch;
    size_t rx_idx = 0U;

    (void)argument;

    if (!g_cli_registered)
    {
        Console_RegisterCommands();
    }

    Console_Write("\r\n=== Celebright CLI ===\r\n");
    Console_WritePrompt();

    for (;;)
    {
        /* Block on StreamBuffer, waiting for USB CDC data */
        if (xStreamBufferReceive(console_rx_stream, &ch, 1, portMAX_DELAY) == 1)
        {
            if (ch == '\r' || ch == '\n')
            {
                if (rx_idx > 0U)
                {
                    line_buf[rx_idx] = '\0';
                    Console_Write("\r\n");
                    Console_ProcessLine(line_buf);
                    rx_idx = 0U;
                }
                Console_WritePrompt();
            }
            else if (ch == '\b' || ch == 0x7F)
            {
                if (rx_idx > 0U)
                {
                    rx_idx--;
                    Console_Write("\b \b");
                }
            }
            else if (rx_idx < (sizeof(line_buf) - 1U) && ch >= 0x20)
            {
                line_buf[rx_idx++] = (char)ch;
            }
        }
    }
}

static void Console_Write(const char *str)
{
    USBD_CDC_HandleTypeDef *hcdc;
    uint16_t len;
    volatile int retry;

    if (str == NULL) return;
    len = (uint16_t)strlen(str);
    if (len == 0U) return;

    if (hUsbDeviceFS.pClassData == NULL) return;
    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    /* Wait for previous USB IN transfer to complete.  CDC_Transmit_FS
       returns USBD_BUSY if TxState != 0.  The USB TX-complete ISR
       clears TxState after the host ACKs the packet (~50us). */
    for (retry = 0; retry < 500000; retry++)
    {
        if (hcdc->TxState == 0) break;
    }

    if (hcdc->TxState == 0)
    {
        (void)CDC_Transmit_FS((uint8_t *)str, len);
    }
}

static void Console_WritePrompt(void)
{
    Console_Write("\r\n" CONSOLE_PROMPT);
}

static void Console_RegisterCommands(void)
{
    /* The built-in "help" command (prvHelpCommand in FreeRTOS_CLI.c) already
       lists all registered commands and their help strings. Do NOT register a
       custom "help" — it would conflict with the built-in multi-output iterator. */
    static const CLI_Command_Definition_t cmds[] = {
        { "tasks", "tasks: List all tasks\r\n",            CLI_TaskListCommand,       0 },
        { "stats", "stats: Runtime CPU stats\r\n",         CLI_RunTimeStatsCommand,   0 },
        { "sys",   "sys: System summary (heap/stack)\r\n", CLI_SysCommand,            0 },
        { "flash", "flash [mount|umount|format|ls|cat|info]: QSPI Flash\r\n",  CLI_FlashCommand,  -1 },
        { "car",   "car [go|spin|arc|stop|start|speed|status]: Car motion control\r\n", CLI_CarCommand, -1 },
#if APP_ENABLE_IMU
        { "imu",   "imu: Show IMU status (yaw/pitch/roll/accel/gyro/quat)\r\n", CLI_ImuCommand, 0 },
#endif
    };

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++)
    {
        FreeRTOS_CLIRegisterCommand(&cmds[i]);
    }
    g_cli_registered = true;
}

static void Console_ProcessLine(char *line)
{
    char *trimmed = line;
    BaseType_t xMore;
    while (*trimmed == ' ') trimmed++;

    if (trimmed[0] == '\0') return;

    /* FreeRTOS+CLI commands may return pdTRUE to indicate more output
       follows (e.g. the built-in "help" command iterates through all
       registered commands one per call). Loop until pdFALSE. */
    do
    {
        g_cli_output[0] = '\0';
        xMore = FreeRTOS_CLIProcessCommand(trimmed, g_cli_output, sizeof(g_cli_output));
        Console_Write(g_cli_output);
    } while (xMore == pdTRUE);
}

/* ====== CLI Commands ====== */

static BaseType_t CLI_TaskListCommand(char *buf, size_t len, const char *cmd)
{
    (void)cmd;
    buf[0] = '\0';
    vTaskList(buf);
    return pdFALSE;
}

static BaseType_t CLI_RunTimeStatsCommand(char *buf, size_t len, const char *cmd)
{
    (void)cmd;
    buf[0] = '\0';
    vTaskGetRunTimeStats(buf);
    return pdFALSE;
}

static BaseType_t CLI_SysCommand(char *buf, size_t len, const char *cmd)
{
    (void)cmd;
#if APP_ENABLE_SYSMON
    AppSysMon_UpdateSnapshot();
    AppSysMon_BuildSystemSummary(buf, len);
#else
    (void)snprintf(buf, len, "SysMon disabled (APP_ENABLE_SYSMON=0).\r\n");
#endif
    return pdFALSE;
}

static BaseType_t CLI_FlashCommand(char *buf, size_t len, const char *cmd)
{
#if APP_ENABLE_QSPI
    BaseType_t plen = 0;
    const char *param = FreeRTOS_CLIGetParameter(cmd, 1, &plen);

    if (!AppQSPI_IsPresent())
    {
        (void)snprintf(buf, len, "Flash not detected.\r\n");
        return pdFALSE;
    }

    if (param == NULL || PARAM_MATCH(param, plen, "info"))
    {
        AppQSPI_BuildInfo(buf, len);
    }
    else if (PARAM_MATCH(param, plen, "mount"))
    {
        if (AppQSPI_Mount())
            (void)snprintf(buf, len, "Flash mounted (LittleFS).\r\n");
        else
            (void)snprintf(buf, len, "Mount failed.\r\n");
    }
    else if (PARAM_MATCH(param, plen, "umount"))
    {
        AppQSPI_Umount();
        (void)snprintf(buf, len, "Flash unmounted.\r\n");
    }
    else if (PARAM_MATCH(param, plen, "format"))
    {
        (void)snprintf(buf, len, "Formatting... ");
        if (AppQSPI_Format())
            (void)snprintf(buf + strlen(buf), len - strlen(buf), "OK\r\n");
        else
            (void)snprintf(buf + strlen(buf), len - strlen(buf), "FAILED\r\n");
    }
    else if (PARAM_MATCH(param, plen, "ls"))
    {
        if (AppQSPI_IsMounted())
            AppQSPI_ListDir(buf, len);
        else
            (void)snprintf(buf, len, "Flash not mounted. Use 'flash mount' first.\r\n");
    }
    else if (PARAM_MATCH(param, plen, "cat"))
    {
        const char *path = FreeRTOS_CLIGetParameter(cmd, 2, NULL);
        if (path && AppQSPI_IsMounted())
            AppQSPI_ReadFile(buf, len, path, 256U);
        else
            (void)snprintf(buf, len, "Usage: flash cat <filename>\r\n");
    }
    else
    {
        (void)snprintf(buf, len, "flash [mount|umount|format|ls|cat|info]\r\n");
    }
#else
    (void)snprintf(buf, len, "QSPI Flash disabled (APP_ENABLE_QSPI=0).\r\n");
#endif
    return pdFALSE;
}

static BaseType_t CLI_CarCommand(char *buf, size_t len, const char *cmd)
{
    BaseType_t sublen = 0, p1len = 0, p2len = 0;
    const char *sub = FreeRTOS_CLIGetParameter(cmd, 1, &sublen);
    const char *p1  = FreeRTOS_CLIGetParameter(cmd, 2, &p1len);
    const char *p2  = FreeRTOS_CLIGetParameter(cmd, 3, &p2len);
    float v, x, angle;

    if (sub == NULL || PARAM_MATCH(sub, sublen, "help"))
    {
        (void)snprintf(buf, len,
            "car commands:\r\n"
            "  car go <mm>            Go straight N mm\r\n"
            "  car spin <deg>          Spin N degrees (pos=CCW, neg=CW)\r\n"
            "  car spin left <deg>     Spin N degrees left (CCW)\r\n"
            "  car spin right <deg>    Spin N degrees right (CW)\r\n"
            "  car arc <mm> <deg>      Move N mm while turning M deg\r\n"
            "  car stop                Stop immediately\r\n"
            "  car start               Resume after stop\r\n"
            "  car cruise <mm/s> [deg/s] Cruise + optional turn rate\r\n"
            "  car turn <deg>           Turn N deg, auto-resume cruise\r\n"
            "  car status               Show current state (yaw/speed)\r\n");
        return pdFALSE;
    }

    if (PARAM_MATCH(sub, sublen, "stop"))
    {
        g_cruise_speed = 0.0F;
        Set_Car_Control(0, 0, 0);
        (void)snprintf(buf, len, "Car: stopped.\r\n");
    }
    else if (PARAM_MATCH(sub, sublen, "start"))
    {
        Set_Car_Start();
        (void)snprintf(buf, len, "Car: resumed.\r\n");
    }
    else if (PARAM_MATCH(sub, sublen, "status"))
    {
        (void)snprintf(buf, len,
            "--- Car Status ---\r\n"
            "Yaw:   %6.1f deg (total %7.1f, circles %d)\r\n"
            "V_Lin: %6.1f / %6.1f mm/s  (cur / tgt)\r\n"
            "V_Ang: %6.1f / %6.1f deg/s (cur / tgt)\r\n"
            "Stop:  %s\r\n",
            (double)car_state.yaw, (double)car_state.yaw_all, car_state.yaw_circles,
            (double)car_attitude.current_v_line, (double)car_attitude.target_v_line,
            (double)car_attitude.current_v_angle, (double)car_attitude.target_v_angle,
            car_attitude.flag_stop ? "YES" : "NO");
    }
    else if (PARAM_MATCH(sub, sublen, "speed"))
    {
        if (p1 == NULL)
        { (void)snprintf(buf, len, "Usage: car speed <mm/s>\r\n"); return pdFALSE; }
        v = (float)atof(p1);
        Set_Car_Attitude(v, 0);
        (void)snprintf(buf, len, "Car speed: %.1f mm/s\r\n", (double)v);
    }
    else if (PARAM_MATCH(sub, sublen, "go"))
    {
        if (p1 == NULL)
        { (void)snprintf(buf, len, "Usage: car go <mm>\r\n"); return pdFALSE; }
        x = (float)atof(p1);
        Set_Car_Control(x, 0, 0);
        (void)snprintf(buf, len, "Car: go %.1f mm\r\n", (double)x);
    }
    else if (PARAM_MATCH(sub, sublen, "spin"))
    {
        if (p1 != NULL && PARAM_MATCH(p1, p1len, "left"))
        {
            if (p2 == NULL)
            { (void)snprintf(buf, len, "Usage: car spin left <deg>\r\n"); return pdFALSE; }
            angle = fabsf((float)atof(p2));
            Set_Car_Control(0, 0, angle);
            (void)snprintf(buf, len, "Car: spin left %.1f deg\r\n", (double)angle);
        }
        else if (p1 != NULL && PARAM_MATCH(p1, p1len, "right"))
        {
            if (p2 == NULL)
            { (void)snprintf(buf, len, "Usage: car spin right <deg>\r\n"); return pdFALSE; }
            angle = -fabsf((float)atof(p2));
            Set_Car_Control(0, 0, angle);
            (void)snprintf(buf, len, "Car: spin right %.1f deg\r\n", (double)fabsf(angle));
        }
        else
        {
            if (p1 == NULL)
            { (void)snprintf(buf, len, "Usage: car spin <deg>\r\n"); return pdFALSE; }
            angle = (float)atof(p1);
            Set_Car_Control(0, 0, angle);
            (void)snprintf(buf, len, "Car: spin %.1f deg\r\n", (double)angle);
        }
    }
    else if (PARAM_MATCH(sub, sublen, "arc"))
    {
        if (p1 == NULL || p2 == NULL)
        { (void)snprintf(buf, len, "Usage: car arc <mm> <deg>\r\n"); return pdFALSE; }
        x = (float)atof(p1);
        angle = (float)atof(p2);
        Set_Car_Control(x, 0, angle);
        (void)snprintf(buf, len, "Car: go %.1f mm, spin %.1f deg\r\n", (double)x, (double)angle);
    }
    else if (PARAM_MATCH(sub, sublen, "cruise"))
    {
        if (p1 == NULL)
        { (void)snprintf(buf, len, "Usage: car cruise <mm/s> [deg/s]\r\n"); return pdFALSE; }
        v = (float)atof(p1);
        angle = (p2 != NULL) ? (float)atof(p2) : 0.0F;
        g_cruise_speed = (fabsf(v) < 0.5F) ? 0.0F : v;
        if (g_cruise_speed == 0.0F)
        {
            Set_Car_Attitude(0, 0);
            (void)snprintf(buf, len, "Cruise: stopped.\r\n");
        }
        else
        {
            Set_Car_Attitude(v, angle);
            (void)snprintf(buf, len, "Cruise: %.1f mm/s, %.1f deg/s\r\n", (double)v, (double)angle);
        }
    }
    else if (PARAM_MATCH(sub, sublen, "turn"))
    {
        if (p1 == NULL)
        { (void)snprintf(buf, len, "Usage: car turn <deg>\r\n"); return pdFALSE; }
        angle = (float)atof(p1);
        /* Save current speed for auto-resume after turn completes */
        g_cruise_speed = car_attitude.target_v_line;
        Set_Car_Control(0, 0, angle);
        (void)snprintf(buf, len, "Car: turn %.1f deg (cruise %.1f mm/s)\r\n", (double)angle, (double)g_cruise_speed);
    }
    else
    {
        (void)snprintf(buf, len, "Unknown: car %s.  Type 'car' for help.\r\n", sub);
    }
    return pdFALSE;
}

#if APP_ENABLE_IMU
static BaseType_t CLI_ImuCommand(char *buf, size_t len, const char *cmd)
{
    (void)cmd;
    IMU_BuildStatus(buf, len);
    return pdFALSE;
}
#endif

#endif /* APP_ENABLE_CONSOLE */
