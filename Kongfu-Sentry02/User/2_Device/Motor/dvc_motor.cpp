#include "2_Device/Motor/dvc_motor.h"

#include "main.h"

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 绑定的CAN ID
 * @param __Motor_DJI_Control_Method 电机控制方式, 默认角度
 * @param __Encoder_Offset 编码器偏移, 默认0
 * @param __Driver_Version 6020电机驱动方式, 默认旧版电压控制
 * @param __Power_Limit_Status 是否开启功率控制
 * @param __Voltage_Max 最大速度, 需根据不同负载测量后赋值, 也就开环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
 * @param __Current_Max 最大电流
 */
void Class_Motor_GM6020::Init(CAN_HandleTypeDef *hcan, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method, float __Voltage_Max, float __Current_Max)
{
	Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
	Voltage_Max = __Voltage_Max;
  Current_Max = __Current_Max;
};


void Class_Motor_GM6020::PID_Calculate()
{
	switch (Motor_DJI_Control_Method)
	{
//	case (Motor_DJI_Control_Method_CURRENT):
//	{
//		PID_Current.Set_Target(Target_Current);
//		PID_Current.Set_Now(Rx_Data.Now_Current);
//		PID_Current.TIM_Adjust_PeriodElapsedCallback();

//		Target_Voltage = PID_Current.Get_Out();
//		break;
//	}
	case (Motor_DJI_Control_Method_OMEGA):
	{
		PID_Omega.Set_Target(Target_Omega);
		PID_Omega.Set_Now(Rx_Data.Now_Omega);
		PID_Omega.TIM_Adjust_PeriodElapsedCallback();

		Target_Current = PID_Omega.Get_Out();
		
		float tmp_value = Target_Current;
		Math_Constrain(&tmp_value, -Current_Max, Current_Max);
		Out = tmp_value * Current_To_Out;
		break;
	}
	case (Motor_DJI_Control_Method_ANGLE):
	{
		PID_Angle.Set_Target(Target_Angle);
		PID_Angle.Set_Now(Rx_Data.Now_Angle);
		PID_Angle.TIM_Adjust_PeriodElapsedCallback();

		Target_Omega = PID_Angle.Get_Out();
		
		PID_Omega.Set_Target(Target_Omega);
		PID_Omega.Set_Now(Rx_Data.Now_Omega);
		PID_Omega.TIM_Adjust_PeriodElapsedCallback();
		
		Target_Current = PID_Omega.Get_Out();
		
		float tmp_value = Target_Current;
		Math_Constrain(&tmp_value, -Current_Max, Current_Max);
		Out = tmp_value * Current_To_Out;
		break;
	}
	
	}


}

