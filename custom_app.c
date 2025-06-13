//* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
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
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* uart_pipe */
  /* ESS */
  uint8_t               Counter_Notification_Status;
  /* Temp_read */
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define ENVIRONMENT_UPDATE_PERIOD       (uint32_t)(0.5*1000*1000/CFG_TS_TICK_VAL) /*500ms*/
#define ENVIRONMENT_UPDATE_PERIOD       (uint32_t)(1*1000*1000/CFG_TS_TICK_VAL) /*1000ms*/
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */

uint8_t Counter_Task_Timer_Id;																		//HW timer ID for the custom app/task

char str[60];
uint8_t str_len;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* uart_pipe */
/* ESS */
static void Custom_Counter_Update_Char(void);
static void Custom_Counter_Send_Notification(void);
/* Temp_read */

/* USER CODE BEGIN PFP */
int32_t compensate_temperature(int32_t sensor_adc_readout, uint16_t dig_T1, int16_t dig_T2, int16_t dig_T3);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* uart_pipe */
    case CUSTOM_STM_PIPE_STR_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PIPE_STR_WRITE_EVT */

    	//this is the WRITE reaction

		  //we wipe the screen
		  ssd1315_advance_clear();

		  //we send the string to the screen
		  ssd1315_advance_string(0, 0, str, str_len, 1, 0x10);

      /* USER CODE END CUSTOM_STM_PIPE_STR_WRITE_EVT */
      break;

    /* ESS */
    case CUSTOM_STM_COUNTER_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_COUNTER_NOTIFY_ENABLED_EVT */
    	//this is the switch in the phone to allow the notification to execute
    	  Custom_App_Context.Counter_Notification_Status = 1;							//we enable the notification
    	  HW_TS_Start(Counter_Task_Timer_Id, ENVIRONMENT_UPDATE_PERIOD);					//we start the task timer
    	    																					//if the notification is not enabled, the sequenced task will not execute
    	    																					//or if it does, it will just exit without doing anything

      /* USER CODE END CUSTOM_STM_COUNTER_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_COUNTER_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_COUNTER_NOTIFY_DISABLED_EVT */

		 Custom_App_Context.Counter_Notification_Status = 0;							//we disable the notification
		 HW_TS_Stop(Counter_Task_Timer_Id);					//we sttop the task timer

      /* USER CODE END CUSTOM_STM_COUNTER_NOTIFY_DISABLED_EVT */
      break;

    /* Temp_read */
    case CUSTOM_STM_TEMP_VALUE_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TEMP_VALUE_READ_EVT */

		  //--------------------//

    	  while(0);																			//this is to remove the IDE complaining

		  //read BMP280 sensor
		  uint8_t T_out[3] = {0xFA, 0xFB, 0xFC};											//this is where the ADC temp values will go

		  //BMP280 temperature ADC readout
		  I2CReadout(device_addr, 3, &T_out);												//we read out the temperature compensation parameters
		  int32_t adc_T = (T_out[0] << 12) | (T_out[1] << 4) | (T_out[2]>>4);				//we rebuild the 20 bit temperature value

		  temp = compensate_temperature(adc_T, dig_T1, dig_T2, dig_T3);

		  temp = temp/100;																	//we keep only the degrees

		  //--------------------//

		  //we update the characteristic
		  Custom_STM_App_Update_Char(CUSTOM_STM_TEMP_VALUE, (uint8_t *)&temp);

		  //--------------------//

		  //we wipe the screen
		  ssd1315_advance_clear();

		  //publish temperature value to screen
		  char str_temp[2];

		  sprintf(str_temp, "%u", temp);

		  ssd1315_advance_string(0, 0, str_temp, 2, 1, 0x10);

      /* USER CODE END CUSTOM_STM_TEMP_VALUE_READ_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
	  UTIL_SEQ_RegTask(1<< CFG_COUNTER_TASK_ID, UTIL_SEQ_RFU, Counter_Send_Notification_Task);			//we create the task

	  HW_TS_Create(CFG_TIM_PROC_ID_ISR,																//we create the task timer by attaching the callback to the timer id
	        &(Counter_Task_Timer_Id),
	        hw_ts_Repeated,
			Counter_Update_Timer_Callback);														//this is what is going to be executed by the timer firing

	  Custom_APP_context_Init();

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* uart_pipe */
/* ESS */
void Custom_Counter_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Counter_UC_1*/

  /* USER CODE END Counter_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_COUNTER, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Counter_UC_Last*/

  /* USER CODE END Counter_UC_Last*/
  return;
}

void Custom_Counter_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Counter_NS_1*/

  /* USER CODE END Counter_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_COUNTER, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Counter_NS_Last*/

  /* USER CODE END Counter_NS_Last*/

  return;
}

/* Temp_read */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void Counter_Update_Timer_Callback(void)
{

  UTIL_SEQ_SetTask(1<<CFG_COUNTER_TASK_ID, CFG_SCH_PRIO_0);							//we reset the task execution

}

void Custom_APP_context_Init(void)
{

	 Custom_App_Context.Counter_Notification_Status = 0;							//we enable the notification from the start

}


//task to send a notification
//this is what is going to be executed periodically by the sequencer
void Counter_Send_Notification_Task(void)												//this is what the task will do
{

  if(Custom_App_Context.Counter_Notification_Status)										//if we have the notification enabled
  {

	  Counter_Update();																	//then we get the characteristic updated

  }
  else
  {

	  //do nothing

  }

  return;
}

int32_t compensate_temperature(int32_t sensor_adc_readout, uint16_t dig_T1, int16_t dig_T2, int16_t dig_T3) {

	int32_t var1, var2;

	var1 = ((((sensor_adc_readout >> 3) - (dig_T1 << 1)))	* dig_T2) >> 11;
	var2 = (((((sensor_adc_readout >> 4) - dig_T1) * ((sensor_adc_readout >> 4) - dig_T1)) >> 12) * dig_T3) >> 14;

	return ((var1 + var2) * 5 + 128) >> 8;

}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
