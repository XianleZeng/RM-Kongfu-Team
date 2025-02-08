#include "2_Device/Motor/dvc_motor.h"

#include "main.h"


/**
 * @brief �����ʼ��
 *
 * @param hcan �󶨵�CAN����
 * @param __CAN_Rx_ID �󶨵�CAN ID
 * @param __Motor_DJI_Control_Method ������Ʒ�ʽ, Ĭ�ϽǶ�
 * @param __Encoder_Offset ������ƫ��, Ĭ��0
 * @param __Driver_Version 6020���������ʽ, Ĭ�Ͼɰ��ѹ����
 * @param __Power_Limit_Status �Ƿ������ʿ���
 * @param __Voltage_Max ����ٶ�, ����ݲ�ͬ���ز�����ֵ, Ҳ�Ϳ�������õõ�, �����Ҹо�Ӧ��û������ϲ���������������
 * @param __Current_Max ������
 */
void Class_Motor_GM6020::Init(CAN_HandleTypeDef *hcan, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method, float __Voltage_Max, float __Current_Max)
{
	Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
	Voltage_Max = __Voltage_Max;
    Current_Max = __Current_Max;
}

void Class_Motor_DJI_C610::Init(CAN_HandleTypeDef *hcan, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method, float __Gearbox_Rate, float __Current_Max)
{
    Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
    Current_Max = __Current_Max;
}

void Class_Motor_DJI_C620::Init(CAN_HandleTypeDef *hcan, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method, float __Gearbox_Rate, float __Current_Max)
{
    Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
	Gearbox_Rate = __Gearbox_Rate;
    Current_Max = __Current_Max;
}


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

void Class_Motor_DJI_C610::PID_Calculate()
{
    switch (Motor_DJI_Control_Method)
    {

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
    default:
    {
        Target_Current = 0.0f;

        break;
    }
    }
}

void Class_Motor_DJI_C620::PID_Calculate()
{
    switch (Motor_DJI_Control_Method)
    {

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
    default:
    {
        Target_Current = 0.0f;

        break;
    }
    }
}
