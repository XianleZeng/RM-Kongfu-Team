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
#include "CAN_receive.h"
#include "drv_uart.h"
#include "dvc_serialplot.h"
#include "drv_bsp.h"
#include "alg_pid.h"

#include "remote_control.h"
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

float Rx_Omega[4];
float Rx_Torque[4];
float Rx_Encoder[4];
float Rx_Temperature[4];

float Now_Omega_0, Target_Omega_0;
float Now_Omega_1, Target_Omega_1;
float Now_Omega_2, Target_Omega_2;
float Now_Omega_3, Target_Omega_3;

float Target_Omega_c0, Target_Omega_c1, Target_Omega_c2, Target_Omega_c3;
float vx_set, vy_set, omega_set;
uint32_t Counter = 0;
int32_t Output_val_0, Output_val_1, Output_val_2,Output_val_3;
float test_val = 0;
float Right_X, Right_Y, Left_X, Left_Y, Switch_2;
//摇杆偏移量
float Rocker_Offset = 1011.0f;
// 开关偏移量
float Switch_Offset = 240.0f;
//摇杆总刻度
float Rocker_Num = 783.5f;
// 开关总刻度
float Switch_Num = 1567.0f;
//地盘最大速度
float velocity_x_max = 20;
float velocity_y_max = 20;
float omega_max = 0.07;
float r_wheel_to_core = 228.73f;

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
      pid_omega.Set_K_P(serialplot.Get_Variable_Value());
    }
    break;
//    case(1):
//    {
//      pid_omega.Set_K_I(serialplot.Get_Variable_Value());
//    }
//    break;
//    case(2):
//    {
//      pid_omega.Set_K_D(serialplot.Get_Variable_Value());
//    }
		case(1):
    {
      pid_omega_1.Set_K_P(serialplot.Get_Variable_Value());
    }
    break;
//    case(4):
//    {
//      pid_omega_1.Set_K_I(serialplot.Get_Variable_Value());
//    }
//    break;
//    case(5):
//    {
//      pid_omega_1.Set_K_D(serialplot.Get_Variable_Value());
//			
//    }
		case(2):
    {
      pid_omega_2.Set_K_P(serialplot.Get_Variable_Value());
    }
    break;
//    case(7):
//    {
//      pid_omega_2.Set_K_I(serialplot.Get_Variable_Value());
//    }
//    break;
//    case(8):
//    {
//      pid_omega_2.Set_K_D(serialplot.Get_Variable_Value());
//    }
		case(3):
    {
      pid_omega_3.Set_K_P(serialplot.Get_Variable_Value());
    }
    break;
//    case(10):
//    {
//      pid_omega_3.Set_K_I(serialplot.Get_Variable_Value());
//    }
//    break;
//    case(11):
//    {
//      pid_omega_3.Set_K_D(serialplot.Get_Variable_Value());
//    }

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
//  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
	  can_filter_init();
		UART_Init(&huart1, UART_Serialplot_Call_Back, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);
    pid_omega.Init(100.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 15000.0f);
		pid_omega_1.Init(100.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 15000.0f);
		pid_omega_2.Init(100.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 15000.0f);
		pid_omega_3.Init(100.0f, 0.0f, 0.0f, 0.0f, 2500.0f, 15000.0f);
		
		serialplot.Init(&huart1, 12, (char **)Variable_Assignment_List);
		
		remote_control_init();
    usart1_tx_dma_init();
		local_rc_ctrl = get_remote_control_point();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		/*从遥控器读取数据*/
		Right_X = (local_rc_ctrl->rc.ch[0] -  Rocker_Offset) / Rocker_Num; /* 右侧左右 */
		Right_Y = (local_rc_ctrl->rc.ch[1] -  Rocker_Offset) / Rocker_Num; /* 右侧上下 */
		Left_Y = (local_rc_ctrl->rc.ch[2] -  Rocker_Offset) / Rocker_Num; /* 左侧上下 */
		Left_X = (local_rc_ctrl->rc.ch[3] -  Rocker_Offset) / Rocker_Num; /* 左侧左右 */
		Switch_2 = (local_rc_ctrl->rc.ch[4] - Switch_Offset) / Switch_Num;
		
//		serialplot.Set_Data(4, &Right_X, &Right_Y, &Left_Y, &Left_X);
//		serialplot.TIM_Write_PeriodElapsedCallback();
//		TIM_UART_PeriodElapsedCallback();
		
		vx_set = Right_X*velocity_x_max;
		vy_set = Right_Y*velocity_y_max;
		omega_set = Left_X*omega_max;
		
		    
		Target_Omega_0 = (-0.707f * vx_set + 0.707f * vy_set + omega_set*r_wheel_to_core) / 0.075f;
		Target_Omega_1 = (-0.707f * vx_set - 0.707f * vy_set + omega_set*r_wheel_to_core) / 0.075f;
		Target_Omega_2 = ( 0.707f * vx_set - 0.707f * vy_set + omega_set*r_wheel_to_core) / 0.075f;
		Target_Omega_3 = ( 0.707f * vx_set + 0.707f * vy_set + omega_set*r_wheel_to_core) / 0.075f;
		
//		serialplot.Set_Data(4, &Target_Omega_0, &Target_Omega_1, &Target_Omega_2, &Target_Omega_3);
//		serialplot.TIM_Write_PeriodElapsedCallback();
//		TIM_UART_PeriodElapsedCallback();
			
		/*读取底盘电机转速*/
		Rx_Omega[0] = get_chassis_motor_measure_point(0)->speed_rpm;
		Rx_Omega[1] = get_chassis_motor_measure_point(1)->speed_rpm;
		Rx_Omega[2] = get_chassis_motor_measure_point(2)->speed_rpm;
		Rx_Omega[3] = get_chassis_motor_measure_point(3)->speed_rpm;
		
		//单位转换
		Now_Omega_0 = Rx_Omega[0] * 2.0f * PI / 60.0f;
		Now_Omega_1 = Rx_Omega[1] * 2.0f * PI / 60.0f;
		Now_Omega_2 = Rx_Omega[2] * 2.0f * PI / 60.0f;
		Now_Omega_3 = Rx_Omega[3] * 2.0f * PI / 60.0f;
		
			
		/*画出目标角度和电机转速*/	
//		serialplot.Set_Data(3, &Now_Omega_0, &Now_Omega_1, &Target_Omega_c2);
//		serialplot.TIM_Write_PeriodElapsedCallback();
//		TIM_UART_PeriodElapsedCallback();
			
		/*计算输入电流值（PID）*/
		pid_omega.Set_Target(Target_Omega_0);
		pid_omega.Set_Now(Now_Omega_0);
		pid_omega.TIM_Adjust_PeriodElapsedCallback();
		
		pid_omega_1.Set_Target(Target_Omega_1);
		pid_omega_1.Set_Now(Now_Omega_1);
		pid_omega_1.TIM_Adjust_PeriodElapsedCallback();
		
		pid_omega_2.Set_Target(Target_Omega_2);
		pid_omega_2.Set_Now(Now_Omega_2);
		pid_omega_2.TIM_Adjust_PeriodElapsedCallback();
		
		pid_omega_3.Set_Target(Target_Omega_3);
		pid_omega_3.Set_Now(Now_Omega_3);
		pid_omega_3.TIM_Adjust_PeriodElapsedCallback();
			
			
		/*输出电流*/
			Output_val_0 = pid_omega.Get_Out();
			Output_val_1 = pid_omega_1.Get_Out();
			Output_val_2 = pid_omega_2.Get_Out();
			Output_val_3 = pid_omega_3.Get_Out();
//		Output_val_0 = test_val;
//		Output_val_1 = test_val;
//		Output_val_2 = test_val;
//		Output_val_3 = test_val;
//		
		serialplot.Set_Data(4, &Target_Omega_0, &Target_Omega_1, &Target_Omega_2, &Output_val_0);
		serialplot.TIM_Write_PeriodElapsedCallback();
		TIM_UART_PeriodElapsedCallback();
  if (Switch_2 == 1)
	{
		CAN_cmd_chassis(Output_val_0, Output_val_1, Output_val_2, Output_val_3);
	}
//			CAN_cmd_gimbal(0, 0, 0, 0);
			
		HAL_Delay(0);
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
