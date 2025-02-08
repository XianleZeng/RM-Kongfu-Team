/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "1_Middleware/1_Driver/CAN/drv_CAN_receive.h"
#include "1_Middleware/1_Driver/UART/drv_uart.h"
#include "2_Device/Serialplot/dvc_serialplot.h"
#include "1_Middleware/1_Driver/BSP/drv_bsp.h"
#include "1_Middleware/2_Algorithm/PID/alg_pid.h"

#include "2_Device/Remote_Control/remote_control.h"
#include "2_Device/Motor/dvc_motor.h"
#include "bsp_usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint8_t hello[5] = "hello";
extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
Class_Serialplot serialplot;
Class_PID pid_omega;
Class_PID pid_omega_1;
Class_PID pid_omega_2;
Class_PID pid_omega_3;

// PID 角度环控制
Class_PID PID_Angle;
// PID 速度环控制
Class_PID PID_Omega;
// PID电流环控制
Class_PID PID_Current;


Class_Motor_GM6020 Motor_Yaw;

float Rx_Omega[4];
float Rx_Torque[4];
float Rx_Encoder[4];
float Rx_Temperature[4];

float Now_Omega_0, Target_Omega_0;
float Now_Omega_1, Target_Omega_1;
float Now_Omega_2, Target_Omega_2;
float Now_Omega_3, Target_Omega_3;
float Now_Angle, Target_Angle;

float Target_Omega_c0, Target_Omega_c1, Target_Omega_c2, Target_Omega_c3;
float vx_set, vy_set, omega_set;
uint32_t Counter = 0;
int32_t Output_val_0, Output_val_1, Output_val_2,Output_val_3;
float test_val = 0;
float Right_X, Right_Y, Left_X, Left_Y;
//摇杆偏移量
float Rocker_Offset = 1011.0f;
//摇杆总刻度
float Rocker_Num = 783.5f;
//地盘最大速度
float velocity_x_max = 10;
float velocity_y_max = 10;
float omega_max = 0.07;
float r_wheel_to_core = 228.73f;

extern float Current_To_Out;

const motor_measure_t *gimbal_motor_measure;

int16_t delta_encoder;

uint16_t Encoder_Num_Per_Round = 8192;

int32_t Encoder_Offset = -2740;

//static char Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
//  // ������PID
//  "po",
//  "io",
//  "do",
//	"po1",
//  "io1",
//  "do1",
//	"po2",
//  "io2",
//  "do2",
//	"po3",
//  "io3",
//  "do3",
//};

static char Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
  // ������PID
  "po",


	"po1",

	"po2",

	"po3",

};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const RC_ctrl_t *local_rc_ctrl;

void usart_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    //return length of string 
    //?????????????
    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);

		HAL_UART_Transmit_DMA(&huart1, tx_buf, len);

}

void UART_Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
  serialplot.UART_RxCpltCallback(Buffer);
  switch (serialplot.Get_Variable_Index())
  {
    //�����PID
    case(0):
    {
      PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
    }
    break;
		case(1):
    {
      pid_omega_1.Set_K_P(serialplot.Get_Variable_Value());
    }
    break;
		case(2):
    {
      pid_omega_2.Set_K_P(serialplot.Get_Variable_Value());
    }
    break;
		case(3):
    {
      pid_omega_3.Set_K_P(serialplot.Get_Variable_Value());
    }
    break;
  }

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
	  can_filter_init();
		UART_Init(&huart1, UART_Serialplot_Call_Back, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);
    PID_Omega.Init(0.03f, 0.3f, 0.0f, 0.0f, 3.0f, 3.0f);

		
		serialplot.Init(&huart1, 12, (char **)Variable_Assignment_List);
		
		remote_control_init();
    usart1_tx_dma_init();
		local_rc_ctrl = get_remote_control_point();
		
		
		// 电机初始化
		Motor_Yaw.Init(&hcan1, Motor_DJI_Control_Method_ANGLE, 25000 ,16384);
		// PID初始化
    Motor_Yaw.PID_Angle.Init(7.0f, 0.0f, 0.0f, 0.0f, 2.0f * PI, 2.0f * PI);
    Motor_Yaw.PID_Omega.Init(0.366f, 0.916f, 0.0f, 0.0f, 3.0f, 3.0f);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Counter++;
		Target_Omega_0 = 10 * sin(6.0f * Counter / 1000.0f);
//		Motor_Yaw.Target_Omega = Target_Omega_0;
//		Motor_Yaw.Set_Target_Angle(PI * sin(2.0f * Counter / 1000.0f));
		Motor_Yaw.Set_Target_Angle(0);
			/*读取云台电机数据*/
		gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
//		Now_Omega_0 = gimbal_motor_measure->speed_rpm * 2.0f * PI / 60.0f;
//		Motor_Yaw.Rx_Data.Now_Omega = Now_Omega_0;
		delta_encoder = gimbal_motor_measure->ecd - gimbal_motor_measure->last_ecd;
    if (delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        // 正方向转过了一圈
        Motor_Yaw.Rx_Data.Total_Round++;
    }
    else if (delta_encoder > Encoder_Num_Per_Round / 2)
    {
        // 反方向转过了一圈
        Motor_Yaw.Rx_Data.Total_Round--;
    }
		Motor_Yaw.Rx_Data.Total_Encoder = Motor_Yaw.Rx_Data.Total_Round * Encoder_Num_Per_Round + gimbal_motor_measure->ecd + Encoder_Offset;
		Motor_Yaw.Rx_Data.Now_Angle = (float) Motor_Yaw.Rx_Data.Total_Encoder / (float) Encoder_Num_Per_Round * 2.0f * PI;
		
		/*计算输入电流值（PID）*/
		Motor_Yaw.PID_Calculate();
		
		/*输出电流*/
		Output_val_0 = Motor_Yaw.Out;
		
//		
		/*画出目标角度和电机转速*/	
		serialplot.Set_Data(3, &Motor_Yaw.Rx_Data.Now_Angle, &Motor_Yaw.Target_Angle, &Output_val_0);
		serialplot.TIM_Write_PeriodElapsedCallback();
		TIM_UART_PeriodElapsedCallback();

		CAN_cmd_gimbal(0, 0, 5000, 0);
		HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
