
#ifndef DVC_MOTOR
#define DVC_MOTOR

#include "struct_typedef.h"
#include "1_Middleware/2_Algorithm/PID/alg_pid.h"
#include "1_Middleware/1_Driver/CAN/drv_CAN_receive.h"
#include "main.h"

/**
 * @brief Control method of the DJI Motor
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
 * @brief Raw data of the DJI motor
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
 * @brief Processed data of the DJI motor
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


/**
 * @brief GM6020 brushless motor
 * 
 */
class Class_Motor_GM6020
{
public:
	// PID angle control loop 
	Class_PID PID_Angle;
	// PID angular velocity control loop
	Class_PID PID_Omega;
	// PID current control loop
	Class_PID PID_Current;
	
	void Init(CAN_HandleTypeDef *hcan, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method, float __Voltage_Max, float __Current_Max);

    inline void Set_Target_Angle(float __Target_Angle);

    inline void Set_Target_Omega(float __Target_Omega);

    inline float Get_Now_Angle();


// protected:
    // initialized related variables

    // CAN
    // Struct_CAN_Manage_Object *CAN_Manage_Object;
//    // �����ݰ󶨵�CAN ID, C6ϵ��0x201~0x208, GMϵ��0x205~0x20b
//    Enum_Motor_DJI_ID CAN_Rx_ID;

    // Transimission buffer
    uint8_t *Tx_Data;
    // Encoder Offset 
    int32_t Encoder_Offset;

    // Maximum voltage 
    float Voltage_Max;
    // Maximum current 
    float Current_Max;

    // constants 

    // Parameters for power calculation 
    float Power_K_0 = 0.8130f;
    float Power_K_1 = -0.0005f;
    float Power_K_2 = 6.0021f;
    float Power_A = 1.3715f;

    // Number of scale per round in the encoder
    uint16_t Encoder_Num_Per_Round = 8192;

    float Voltage_To_Out = 25000.0f / 24.0f;

    float Current_To_Out = 16384.0f / 3.0f;

    float Theoretical_Output_Voltage_Max = 24.0f;

    float Theoretical_Output_Current_Max = 3.0f;

    // internal variables


    uint32_t Flag = 0;

    uint32_t Pre_Flag = 0;

    float Out = 0.0f;

    // read variables

    // ���״̬
//    Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;

    Struct_Motor_DJI_Rx_Data Rx_Data;

    float Power_Estimate;

    // Motor control method
    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE;
    // rad
    float Target_Angle = 0.0f;
    // rad/s
    float Target_Omega = 0.0f;
    // A
    float Target_Current = 0.0f;
    // V
    float Target_Voltage = 0.0f;
    // rad/s
    float Feedforward_Omega = 0.0f;
    // A
    float Feedforward_Current = 0.0f;
    // V
    float Feedforward_Voltage = 0.0f;


    // internal functions

    void Data_Process();

    void PID_Calculate();

    void Power_Limit_Control();

    void Output();
};

class Class_Motor_DJI_C610
{
public:
    // PID?????
    Class_PID PID_Angle;
    // PID??????
    Class_PID PID_Omega;

    void Init(CAN_HandleTypeDef *hcan, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method, float __Gearbox_Rate = 36.0f, float __Current_Max = 10.0f);


    inline void Set_Target_Angle(float __Target_Angle);

	inline void Set_Target_Omega(float __Target_Omega);

    inline float Get_Target_Angle();

    inline float Get_Now_Angle();

    uint8_t *Tx_Data;
    // ???, ??????
    float Gearbox_Rate;
    // ????
    float Current_Max;

    // ??

    // ???????
    uint16_t Encoder_Num_Per_Round = 8192;

    // ??????????
    float Current_To_Out = 10000.0f / 10.0f;
    // ????????
    float Theoretical_Output_Current_Max = 10.0f;

    // ????

    // ?????????flag
    uint32_t Flag = 0;
    // ?????????flag
    uint32_t Pre_Flag = 0;
    // ???
    float Out = 0.0f;

 
    Struct_Motor_DJI_Rx_Data Rx_Data;

    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE;

    float Target_Angle = 0.0f;

    float Target_Omega = 0.0f;

    float Target_Current = 0.0f;

    float Feedforward_Omega = 0.0f;

    float Feedforward_Current = 0.0f;



    void Data_Process();

    void PID_Calculate();

    void Output();
};


class Class_Motor_DJI_C620
{
public:
	// PID angle control loop 
	Class_PID PID_Angle;
	// PID angular velocity control loop
	Class_PID PID_Omega;
	// PID current control loop
	Class_PID PID_Current;
	
	void Init(CAN_HandleTypeDef *hcan, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method = Motor_DJI_Control_Method_OMEGA, float __Gearbox_Rate = 3591.0f / 187.0f, float __Current_Max = 20.0f);

    inline void Set_Target_Angle(float __Target_Angle);

    inline void Set_Target_Omega(float __Target_Omega);


// protected:
    // initialized related variables

    // CAN
    // Struct_CAN_Manage_Object *CAN_Manage_Object;

    // Transimission buffer
    uint8_t *Tx_Data;
    // Gear ratio, = 3591.0f / 187.0f in defult 
    float Gearbox_Rate;
    // Encoder Offset 
    int32_t Encoder_Offset;

    // Maximum voltage 
    float Voltage_Max;
    // Maximum current 
    float Current_Max;

    // constants 

    // Parameters for power calculation 
    float Power_K_0 = 0.8130f;
    float Power_K_1 = -0.0005f;
    float Power_K_2 = 6.0021f;
    float Power_A = 1.3715f;

    // Number of scale per round in the encoder
    uint16_t Encoder_Num_Per_Round = 8192;

    float Current_To_Out = 16384.0f / 20.0f;

    float Theoretical_Output_Current_Max = 20.0f;

    // internal variables


    uint32_t Flag = 0;

    uint32_t Pre_Flag = 0;

    float Out = 0.0f;

    // read variables

    // ���״̬
//    Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;

    Struct_Motor_DJI_Rx_Data Rx_Data;

    float Power_Estimate;

    // Motor control method
    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE;
    // rad
    float Target_Angle = 0.0f;
    // rad/s
    float Target_Omega = 0.0f;
    // A
    float Target_Current = 0.0f;
    // V
    float Target_Voltage = 0.0f;
    // rad/s
    float Feedforward_Omega = 0.0f;
    // A
    float Feedforward_Current = 0.0f;


    // internal functions

    void Data_Process();

    void PID_Calculate();

    void Power_Limit_Control();

    void Output();
};




/**
 * @brief Set Motor Target Angle, rad
 *
 * @param __Target_Angle Target Angle, rad
 */
inline void Class_Motor_GM6020::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief Set Target Speed, rad/s
 *
 * @param __Target_Omega Target Speed, rad/s
 */
inline void Class_Motor_GM6020::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
inline float Class_Motor_GM6020::Get_Now_Angle()
{
    return (Rx_Data.Now_Angle);
}



inline void Class_Motor_DJI_C610::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

inline void Class_Motor_DJI_C610::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
inline float Class_Motor_DJI_C610::Get_Target_Angle()
{
    return (Target_Angle);
}

inline float Class_Motor_DJI_C610::Get_Now_Angle()
{
    return (Rx_Data.Now_Angle);
}

inline void Class_Motor_DJI_C620::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

inline void Class_Motor_DJI_C620::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}



#endif
