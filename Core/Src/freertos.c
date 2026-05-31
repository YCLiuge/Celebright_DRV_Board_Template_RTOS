/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
#if APP_ENABLE_IMU
extern float motion6[7];
extern float ypr[3];          // yaw pitch roll
#endif
#if APP_ENABLE_UART_FIFO
QueueHandle_t uart3_frame_queue;
QueueHandle_t uart4_frame_queue;
#endif
#if APP_ENABLE_CONSOLE
StreamBufferHandle_t console_rx_stream = NULL;
#endif


/* USER CODE END Variables */
/* Definitions for LEDBlink */
osThreadId_t LEDBlinkHandle;
uint32_t LEDBlinkBuffer[ 128 ];
osStaticThreadDef_t LEDBlinkControlBlock;
const osThreadAttr_t LEDBlink_attributes = {
  .name = "LEDBlink",
  .cb_mem = &LEDBlinkControlBlock,
  .cb_size = sizeof(LEDBlinkControlBlock),
  .stack_mem = &LEDBlinkBuffer[0],
  .stack_size = sizeof(LEDBlinkBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Broadcast */
osThreadId_t BroadcastHandle;
uint32_t BroadcastBuffer[ 256 ];
osStaticThreadDef_t BroadcastControlBlock;
const osThreadAttr_t Broadcast_attributes = {
  .name = "Broadcast",
  .cb_mem = &BroadcastControlBlock,
  .cb_size = sizeof(BroadcastControlBlock),
  .stack_mem = &BroadcastBuffer[0],
  .stack_size = sizeof(BroadcastBuffer),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Uart4Rx */
osThreadId_t Uart4RxHandle;
uint32_t Uart4RxBuffer[ 256 ];
osStaticThreadDef_t Uart4RxControlBlock;
const osThreadAttr_t Uart4Rx_attributes = {
  .name = "Uart4Rx",
  .cb_mem = &Uart4RxControlBlock,
  .cb_size = sizeof(Uart4RxControlBlock),
  .stack_mem = &Uart4RxBuffer[0],
  .stack_size = sizeof(Uart4RxBuffer),
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for IMUService */
osThreadId_t IMUServiceHandle;
uint32_t IMUServiceBuffer[ 128 ];
osStaticThreadDef_t IMUServiceControlBlock;
const osThreadAttr_t IMUService_attributes = {
  .name = "IMUService",
  .cb_mem = &IMUServiceControlBlock,
  .cb_size = sizeof(IMUServiceControlBlock),
  .stack_mem = &IMUServiceBuffer[0],
  .stack_size = sizeof(IMUServiceBuffer),
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for CarControl */
osThreadId_t CarControlHandle;
uint32_t CarControlBuffer[ 128 ];
osStaticThreadDef_t CarControlControlBlock;
const osThreadAttr_t CarControl_attributes = {
  .name = "CarControl",
  .cb_mem = &CarControlControlBlock,
  .cb_size = sizeof(CarControlControlBlock),
  .stack_mem = &CarControlBuffer[0],
  .stack_size = sizeof(CarControlBuffer),
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for KeyScan */
osThreadId_t KeyScanHandle;
uint32_t KeyScanBuffer[ 128 ];
osStaticThreadDef_t KeyScanControlBlock;
const osThreadAttr_t KeyScan_attributes = {
  .name = "KeyScan",
  .cb_mem = &KeyScanControlBlock,
  .cb_size = sizeof(KeyScanControlBlock),
  .stack_mem = &KeyScanBuffer[0],
  .stack_size = sizeof(KeyScanBuffer),
  .priority = (osPriority_t) osPriorityNormal6,
};
/* Definitions for Buzzer */
osThreadId_t BuzzerHandle;
uint32_t BuzzerBuffer[ 128 ];
osStaticThreadDef_t BuzzerControlBlock;
const osThreadAttr_t Buzzer_attributes = {
  .name = "Buzzer",
  .cb_mem = &BuzzerControlBlock,
  .cb_size = sizeof(BuzzerControlBlock),
  .stack_mem = &BuzzerBuffer[0],
  .stack_size = sizeof(BuzzerBuffer),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Display */
osThreadId_t DisplayHandle;
uint32_t DisplayBuffer[ 128 ];
osStaticThreadDef_t DisplayControlBlock;
const osThreadAttr_t Display_attributes = {
  .name = "Display",
  .cb_mem = &DisplayControlBlock,
  .cb_size = sizeof(DisplayControlBlock),
  .stack_mem = &DisplayBuffer[0],
  .stack_size = sizeof(DisplayBuffer),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Reserved */
osThreadId_t ReservedHandle;
uint32_t ReservedBuffer[ 1024 ];
osStaticThreadDef_t ReservedControlBlock;
const osThreadAttr_t Reserved_attributes = {
  .name = "Reserved",
  .cb_mem = &ReservedControlBlock,
  .cb_size = sizeof(ReservedControlBlock),
  .stack_mem = &ReservedBuffer[0],
  .stack_size = sizeof(ReservedBuffer),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Uart3Rx */
osThreadId_t Uart3RxHandle;
uint32_t Uart3HandleBuffer[ 256 ];
osStaticThreadDef_t Uart3HandleControlBlock;
const osThreadAttr_t Uart3Rx_attributes = {
  .name = "Uart3Rx",
  .cb_mem = &Uart3HandleControlBlock,
  .cb_size = sizeof(Uart3HandleControlBlock),
  .stack_mem = &Uart3HandleBuffer[0],
  .stack_size = sizeof(Uart3HandleBuffer),
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for SysMon */
osThreadId_t SysMonHandle;
uint32_t SysMonBuffer[ 512 ];
osStaticThreadDef_t SysMonControlBlock;
const osThreadAttr_t SysMon_attributes = {
  .name = "SysMon",
  .cb_mem = &SysMonControlBlock,
  .cb_size = sizeof(SysMonControlBlock),
  .stack_mem = &SysMonBuffer[0],
  .stack_size = sizeof(SysMonBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Console */
osThreadId_t ConsoleHandle;
uint32_t ConsoleBuffer[ 512 ];
osStaticThreadDef_t ConsoleControlBlock;
const osThreadAttr_t Console_attributes = {
  .name = "Console",
  .cb_mem = &ConsoleControlBlock,
  .cb_size = sizeof(ConsoleControlBlock),
  .stack_mem = &ConsoleBuffer[0],
  .stack_size = sizeof(ConsoleBuffer),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for queue_key */
osMessageQueueId_t queue_keyHandle;
const osMessageQueueAttr_t queue_key_attributes = {
  .name = "queue_key"
};
/* Definitions for semphr_buzzer_trigger */
osSemaphoreId_t semphr_buzzer_triggerHandle;
const osSemaphoreAttr_t semphr_buzzer_trigger_attributes = {
  .name = "semphr_buzzer_trigger"
};
/* Definitions for semphr_uart_receive */
osSemaphoreId_t semphr_uart_receiveHandle;
const osSemaphoreAttr_t semphr_uart_receive_attributes = {
  .name = "semphr_uart_receive"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void LedBlink_Handler(void *argument);
void Broadcast_Handler(void *argument);
void Uart4Rx_Handler(void *argument);
void IMUService_Handler(void *argument);
void CarControl_Handler(void *argument);
void KeyScan_Handler(void *argument);
void Buzzer_Handler(void *argument);
void Display_Handler(void *argument);
void Reserved_Handler(void *argument);
void Uart3Rx_Handler(void *argument);
void SysMon_Handler(void *argument);
void Console_Handler(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semphr_buzzer_trigger */
  semphr_buzzer_triggerHandle = osSemaphoreNew(1, 1, &semphr_buzzer_trigger_attributes);

  /* creation of semphr_uart_receive */
  semphr_uart_receiveHandle = osSemaphoreNew(16, 0, &semphr_uart_receive_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of queue_key */
  queue_keyHandle = osMessageQueueNew (1, sizeof(char), &queue_key_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
#if APP_ENABLE_UART_FIFO
	uart3_frame_queue = xQueueCreate(5, UART_FRAME_MAX_LEN);
	uart4_frame_queue = xQueueCreate(5, UART_FRAME_MAX_LEN);
#endif
#if APP_ENABLE_CONSOLE
	console_rx_stream = xStreamBufferCreate(256, 1);
#endif
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LEDBlink */
  LEDBlinkHandle = osThreadNew(LedBlink_Handler, NULL, &LEDBlink_attributes);

  /* creation of Broadcast */
  BroadcastHandle = osThreadNew(Broadcast_Handler, NULL, &Broadcast_attributes);

  /* creation of Uart4Rx */
  Uart4RxHandle = osThreadNew(Uart4Rx_Handler, NULL, &Uart4Rx_attributes);

  /* creation of IMUService */
  IMUServiceHandle = osThreadNew(IMUService_Handler, NULL, &IMUService_attributes);

  /* creation of CarControl */
  CarControlHandle = osThreadNew(CarControl_Handler, NULL, &CarControl_attributes);

  /* creation of KeyScan */
  KeyScanHandle = osThreadNew(KeyScan_Handler, NULL, &KeyScan_attributes);

  /* creation of Buzzer */
  BuzzerHandle = osThreadNew(Buzzer_Handler, NULL, &Buzzer_attributes);

  /* creation of Display */
  DisplayHandle = osThreadNew(Display_Handler, NULL, &Display_attributes);

  /* creation of Reserved */
  ReservedHandle = osThreadNew(Reserved_Handler, NULL, &Reserved_attributes);

  /* creation of Uart3Rx */
  Uart3RxHandle = osThreadNew(Uart3Rx_Handler, NULL, &Uart3Rx_attributes);

  /* creation of SysMon */
  SysMonHandle = osThreadNew(SysMon_Handler, NULL, &SysMon_attributes);

  /* creation of Console */
  ConsoleHandle = osThreadNew(Console_Handler, NULL, &Console_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	xTaskCreate(AppInit_Task, "AppInit", 512, NULL, osPriorityNormal6, NULL);
#if APP_ENABLE_CONSOLE
	AppConsole_Init();
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_LedBlink_Handler */
/**
  * @brief  Function implementing the LEDBlink thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LedBlink_Handler */
__weak void LedBlink_Handler(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN LedBlink_Handler */
#if APP_ENABLE_LEDBLINK
  for(;;)
  {
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    osDelay(1000);
  }
#else
  LEDBlinkHandle = NULL;
  vTaskDelete(NULL);
#endif
  /* USER CODE END LedBlink_Handler */
}

/* USER CODE BEGIN Header_Broadcast_Handler */
/**
* @brief Function implementing the Broadcast thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Broadcast_Handler */
__weak void Broadcast_Handler(void *argument)
{
  /* USER CODE BEGIN Broadcast_Handler */
#if APP_ENABLE_BROADCAST
  AppBroadcast_Task(argument);
#else
  BroadcastHandle = NULL;
  vTaskDelete(NULL);
#endif
  /* USER CODE END Broadcast_Handler */
}

/* USER CODE BEGIN Header_Uart4Rx_Handler */
/**
* @brief Function implementing the Uart4Rx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Uart4Rx_Handler */
__weak void Uart4Rx_Handler(void *argument)
{
  /* USER CODE BEGIN Uart4Rx_Handler */
#if APP_ENABLE_UART_FIFO
  AppUart4Rx_Task(argument);
#else
  Uart4RxHandle = NULL;
  vTaskDelete(NULL);
#endif
  /* USER CODE END Uart4Rx_Handler */
}

/* USER CODE BEGIN Header_IMUService_Handler */
/**
* @brief Function implementing the IMUService thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMUService_Handler */
__weak void IMUService_Handler(void *argument)
{
  /* USER CODE BEGIN IMUService_Handler */
#if APP_ENABLE_IMU
  AppIMUService_Task(argument);
#else
  IMUServiceHandle = NULL;
  vTaskDelete(NULL);
#endif
  /* USER CODE END IMUService_Handler */
}

/* USER CODE BEGIN Header_CarControl_Handler */
/**
* @brief Function implementing the CarControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CarControl_Handler */
__weak void CarControl_Handler(void *argument)
{
  /* USER CODE BEGIN CarControl_Handler */
  AppCarControl_Task(argument);
  /* USER CODE END CarControl_Handler */
}

/* USER CODE BEGIN Header_KeyScan_Handler */
/**
* @brief Function implementing the KeyScan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KeyScan_Handler */
__weak void KeyScan_Handler(void *argument)
{
  /* USER CODE BEGIN KeyScan_Handler */
#if APP_ENABLE_KEYBOARD
  AppKeyScan_Task(argument);
#else
  KeyScanHandle = NULL;
  vTaskDelete(NULL);
#endif
  /* USER CODE END KeyScan_Handler */
}

/* USER CODE BEGIN Header_Buzzer_Handler */
/**
* @brief Function implementing the Buzzer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Buzzer_Handler */
__weak void Buzzer_Handler(void *argument)
{
  /* USER CODE BEGIN Buzzer_Handler */
#if APP_ENABLE_MUSIC
  AppBuzzer_Task(argument);
#else
  BuzzerHandle = NULL;
  vTaskDelete(NULL);
#endif
  /* USER CODE END Buzzer_Handler */
}

/* USER CODE BEGIN Header_Display_Handler */
/**
* @brief Function implementing the Display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Display_Handler */
__weak void Display_Handler(void *argument)
{
  /* USER CODE BEGIN Display_Handler */
#if APP_ENABLE_DISPLAY
  AppDisplay_Task(argument);
#else
  DisplayHandle = NULL;
  vTaskDelete(NULL);
#endif
  /* USER CODE END Display_Handler */
}

/* USER CODE BEGIN Header_Reserved_Handler */
/**
* @brief Function implementing the Reserved thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Reserved_Handler */
__weak void Reserved_Handler(void *argument)
{
  /* USER CODE BEGIN Reserved_Handler */
#if APP_ENABLE_RESERVED
  AppReserved_Task(argument);
#else
  ReservedHandle = NULL;
  vTaskDelete(NULL);
#endif
  /* USER CODE END Reserved_Handler */
}

/* USER CODE BEGIN Header_Uart3Rx_Handler */
/**
* @brief Function implementing the Uart3Rx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Uart3Rx_Handler */
__weak void Uart3Rx_Handler(void *argument)
{
  /* USER CODE BEGIN Uart3Rx_Handler */
#if APP_ENABLE_UART_FIFO
  AppUart3Rx_Task(argument);
#else
  Uart3RxHandle = NULL;
  vTaskDelete(NULL);
#endif
  /* USER CODE END Uart3Rx_Handler */
}

/* USER CODE BEGIN Header_SysMon_Handler */
/**
* @brief Function implementing the SysMon thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SysMon_Handler */
__weak void SysMon_Handler(void *argument)
{
  /* USER CODE BEGIN SysMon_Handler */
#if APP_ENABLE_SYSMON
  AppSysMon_Task(argument);
#else
  SysMonHandle = NULL;
  vTaskDelete(NULL);
#endif
  /* USER CODE END SysMon_Handler */
}

/* USER CODE BEGIN Header_Console_Handler */
/**
* @brief Function implementing the Console thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Console_Handler */
__weak void Console_Handler(void *argument)
{
  /* USER CODE BEGIN Console_Handler */
#if APP_ENABLE_CONSOLE
  AppConsole_Task(argument);
#else
  ConsoleHandle = NULL;
  vTaskDelete(NULL);
#endif
  /* USER CODE END Console_Handler */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

