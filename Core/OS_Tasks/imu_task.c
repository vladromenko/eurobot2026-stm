/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         imu_task.c
  * @brief        MPU-6050 on I2C1 polled at 100 Hz.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "os_tasks.h"
#include "mpu6050.h"

/* Diagnostics, readable over SWD */
volatile uint8_t imu_ready = 0;         // 1 - the sensor answers and the data is fresh
volatile uint32_t imu_read_errors = 0;


#define TASK_STACK_SIZE             256
#define IMU_PERIOD_MS               10      // the sensor samples at 100 Hz
#define IMU_RETRY_MS                500     // pause before the next init after a failure
#define IMU_I2C_ADDR                MPU6050_ADDR_AD0_HIGH   // AD0 is wired to VCC


static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;

static struct mpu6050_data_s imu_data;


/* Copy of the last sample, 1 if the sensor answers */
uint8_t IMU_GetData(struct mpu6050_data_s* data)
{
  taskENTER_CRITICAL();
  *data = imu_data;
  taskEXIT_CRITICAL();
  return imu_ready;
}

static void Task(void *arg)
{
  struct mpu6050_data_s sample;
  TickType_t last_wake_time;

  (void)arg;
  vTaskDelay(50);

	while(1)
	{
    // Init again after any failure, e.g. the sensor was connected after the start
    if (MPU6050.Init(I2C1, IMU_I2C_ADDR) != 0) {
      vTaskDelay(pdMS_TO_TICKS(IMU_RETRY_MS));
      continue;
    }

    last_wake_time = xTaskGetTickCount();
    while (MPU6050.Read(&sample) == 0) {
      taskENTER_CRITICAL();
      imu_data = sample;
      taskEXIT_CRITICAL();
      imu_ready = 1;
      vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
    imu_ready = 0;
    imu_read_errors++;
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return task_handle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    task_handle = xTaskCreateStatic(Task, "ImuTask", TASK_STACK_SIZE,
                              NULL, uxPriority, stack_buffer, &task_buffer);
}


const struct os_task_base_s ImuTask = { .CreateTask = CreateTask,
                                        .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
