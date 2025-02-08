
#ifndef DVC_MOTOR
#define DVC_MOTOR

#include "struct_typedef.h"
#include "1_Middleware/2_Algorithm/PID/alg_pid.h"
#include "1_Middleware/1_Driver/CAN/drv_CAN_receive.h"
#include "main.h"

/**
 * @brief 大疆电机控制方式
 *
 */
enum Enum_Motor_DJI_Control_Method
{
    Motor_DJI_Control_Method_VOLTAGE = 0,
    Motor_DJI_Control_Method_CURRENT,
    Motor_DJI_Control_Method_TORQUE,
    Motor_DJI_Control_Method_OMEGA,
    Motor_DJI_Control_Method_ANGLE,
};


/**
 * @brief 大疆电机源数据
 *
 */
struct Struct_Motor_DJI_CAN_Rx_Data
{
    uint16_t Encoder_Reverse;
    int16_t Omega_Reverse;
    int16_t Current_Reverse;
    uint8_t Temperature;
    uint8_t Reserved;
} __attribute__((packed));

/**
 * @brief 大疆电机经过处理的数据
 *
 */
struct Struct_Motor_DJI_Rx_Data
{
    float Now_Angle;
    float Now_Omega;
    float Now_Current;
    float Now_Temperature;
    float Now_Power;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
};




class Class_Motor_GM6020
{
public:
	// PID 角度环控制
	Class_PID PID_Angle;
	// PID 角速度环控制
	Class_PID PID_Omega;
	// PID 电流环控制
	Class_PID PID_Current;
	
	void Init(CAN_HandleTypeDef *hcan, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method, float __Voltage_Max, float __Current_Max);
	
	inline void Set_Target_Angle(float __Target_Angle);

//protected:
//    // 初始化相关变量

//    // 绑定的CAN
//    Struct_CAN_Manage_Object *CAN_Manage_Object;
//    // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
//    Enum_Motor_DJI_ID CAN_Rx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 编码器偏移
    int32_t Encoder_Offset;
//    // 电机驱动方式
//    Enum_Motor_DJI_GM6020_Driver_Version Driver_Version;
//    // 是否开启功率控制
//    Enum_Motor_DJI_Power_Limit_Status Power_Limit_Status;
    // 最大电压
    float Voltage_Max;
    // 最大电流
    float Current_Max;

    // 常量

    // 功率计算系数
    float Power_K_0 = 0.8130f;
    float Power_K_1 = -0.0005f;
    float Power_K_2 = 6.0021f;
    float Power_A = 1.3715f;

    // 一圈编码器刻度
    uint16_t Encoder_Num_Per_Round = 8192;

    // 电压到输出的转化系数
    float Voltage_To_Out = 25000.0f / 24.0f;
    // 电流到输出的转化系数
    float Current_To_Out = 16384.0f / 3.0f;
    // 理论最大输出电压
    float Theoretical_Output_Voltage_Max = 24.0f;
    // 理论最大输出电流
    float Theoretical_Output_Current_Max = 3.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
    // 输出量
    float Out = 0.0f;

    // 读变量

    // 电机状态
//    Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_DJI_Rx_Data Rx_Data;
    // 下一时刻的功率估计值, W
    float Power_Estimate;

    // 写变量

    // 读写变量

    // 电机控制方式
    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE;
    // 目标的角度, rad
    float Target_Angle = 0.0f;
    // 目标的速度, rad/s
    float Target_Omega = 0.0f;
    // 目标的电流, A
    float Target_Current = 0.0f;
    // 目标的电压, V
    float Target_Voltage = 0.0f;
    // 前馈的速度, rad/s
    float Feedforward_Omega = 0.0f;
    // 前馈的电流, A
    float Feedforward_Current = 0.0f;
    // 前馈的电压, V
    float Feedforward_Voltage = 0.0f;


    // 内部函数

    void Data_Process();

    void PID_Calculate();

    void Power_Limit_Control();

    void Output();
	
};

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
inline void Class_Motor_GM6020::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

		
		

#endif
