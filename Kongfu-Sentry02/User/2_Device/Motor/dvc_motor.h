
#ifndef DVC_MOTOR
#define DVC_MOTOR

#include "struct_typedef.h"
#include "1_Middleware/2_Algorithm/PID/alg_pid.h"
#include "1_Middleware/1_Driver/CAN/drv_CAN_receive.h"
#include "main.h"

/**
 * @brief �󽮵�����Ʒ�ʽ
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
 * @brief �󽮵��Դ����
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
 * @brief �󽮵���������������
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
	// PID �ǶȻ�����
	Class_PID PID_Angle;
	// PID ���ٶȻ�����
	Class_PID PID_Omega;
	// PID ����������
	Class_PID PID_Current;
	
	void Init(CAN_HandleTypeDef *hcan, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method, float __Voltage_Max, float __Current_Max);
	
	inline void Set_Target_Angle(float __Target_Angle);

//protected:
//    // ��ʼ����ر���

//    // �󶨵�CAN
//    Struct_CAN_Manage_Object *CAN_Manage_Object;
//    // �����ݰ󶨵�CAN ID, C6ϵ��0x201~0x208, GMϵ��0x205~0x20b
//    Enum_Motor_DJI_ID CAN_Rx_ID;
    // ���ͻ�����
    uint8_t *Tx_Data;
    // ������ƫ��
    int32_t Encoder_Offset;
//    // ���������ʽ
//    Enum_Motor_DJI_GM6020_Driver_Version Driver_Version;
//    // �Ƿ������ʿ���
//    Enum_Motor_DJI_Power_Limit_Status Power_Limit_Status;
    // ����ѹ
    float Voltage_Max;
    // ������
    float Current_Max;

    // ����

    // ���ʼ���ϵ��
    float Power_K_0 = 0.8130f;
    float Power_K_1 = -0.0005f;
    float Power_K_2 = 6.0021f;
    float Power_A = 1.3715f;

    // һȦ�������̶�
    uint16_t Encoder_Num_Per_Round = 8192;

    // ��ѹ�������ת��ϵ��
    float Voltage_To_Out = 25000.0f / 24.0f;
    // �����������ת��ϵ��
    float Current_To_Out = 16384.0f / 3.0f;
    // ������������ѹ
    float Theoretical_Output_Voltage_Max = 24.0f;
    // ��������������
    float Theoretical_Output_Current_Max = 3.0f;

    // �ڲ�����

    // ��ǰʱ�̵ĵ������flag
    uint32_t Flag = 0;
    // ǰһʱ�̵ĵ������flag
    uint32_t Pre_Flag = 0;
    // �����
    float Out = 0.0f;

    // ������

    // ���״̬
//    Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    // �������ӿ���Ϣ
    Struct_Motor_DJI_Rx_Data Rx_Data;
    // ��һʱ�̵Ĺ��ʹ���ֵ, W
    float Power_Estimate;

    // д����

    // ��д����

    // ������Ʒ�ʽ
    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE;
    // Ŀ��ĽǶ�, rad
    float Target_Angle = 0.0f;
    // Ŀ����ٶ�, rad/s
    float Target_Omega = 0.0f;
    // Ŀ��ĵ���, A
    float Target_Current = 0.0f;
    // Ŀ��ĵ�ѹ, V
    float Target_Voltage = 0.0f;
    // ǰ�����ٶ�, rad/s
    float Feedforward_Omega = 0.0f;
    // ǰ���ĵ���, A
    float Feedforward_Current = 0.0f;
    // ǰ���ĵ�ѹ, V
    float Feedforward_Voltage = 0.0f;


    // �ڲ�����

    void Data_Process();

    void PID_Calculate();

    void Power_Limit_Control();

    void Output();
	
};

/**
 * @brief �趨Ŀ��ĽǶ�, rad
 *
 * @param __Target_Angle Ŀ��ĽǶ�, rad
 */
inline void Class_Motor_GM6020::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

		
		

#endif
