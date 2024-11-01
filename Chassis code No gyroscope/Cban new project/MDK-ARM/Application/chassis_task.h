#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "pid.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "Mode_Set.h"

#define Chassis_Motor_Speed_Kp  980     //���̵��P
#define Chassis_Motor_Speed_Ki  0.01   //���̵��I
#define Chassis_Motor_Speed_Kd  0     //���̵��D
#define Chassis_Motor_Speed_Maxout 10000
#define Chassis_Motor_Turn_Kp 12
#define Chassis_Motor_Turn_Ki 0
#define Chassis_Motor_Turn_Kd 0	
#define Chassis_Motor_Turn_Maxout 5
#define Chassis_Motor_Turn_IMaxout 0

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

#define Chassis_Min_Set_SPEED 5                         //�����ٶ�
#define Chassis_Max_Set_SPEED 10                         //����ٶ�
#define Chassis_Shift_Max_Speed   30                      //Shift��������ٶ�

#define Chassis_Max_Speed_Sett 30

  
void Chassis_Task(void);
void Chassis_PID_Init(void); 

typedef struct
{
	//�������
	const motor_measure_t *motor_data;
	//����ת����ĵ��ת��
	float Chassis_Speed_Get;
}Chassos_Motor_Msg_t;


typedef struct                                   
{
	//����ģʽ
	Chassis_Mode_t Chassis_Mode;
	
	 //��������Ϣ
	Chassos_Motor_Msg_t Chassis_Motor_Msg[4];
             
	                                         
	                                         
	//����ϵͳ������ݻ�ȡ                   
//	Chassos_Judge_Msg_t Chassos_Judge_Msg;
	
	//��������������ݻ�ȡ
//	Chassis_Cap_Msg_t Chassos_Cap_Msg;
	
	//���̵�����pid����
	PID Chassis_Motor_Pid[4];
	PID Chassis_Motor_Turn_Pid;
	
	//������������ݻ�ȡ
//	const float* Chassis_IMU_Angle;
//	const float* Chassis_IMU_Aspeed;
	//ң��������ֵ��ȡ
	const RC_ctrl_t *rc_ctrl;	
	
	//�����ٶ�����
	float Chassis_Control_Speed_Set[4];
	float Chassis_X_Speed_Set;
	float Chassis_Y_Speed_Set;
	float Chassis_LR_Speed_Set;
	//���̸�����̨-��̨��е�ǶȲ�
//	float* Chassis_Follow_Gimbal_Angle_TM;
//	first_order_filter_type_t chassis_cmd_slow_set_vx;
//  first_order_filter_type_t chassis_cmd_slow_set_vy;
	//��������ٶ�
	float Chassis_Speed_Max;
	//���̵���ٶ�����
	float Chassis_Motor_Speed_Set[4];
	
	//���̷��͵���ֵ
	float Chassis_Motor_Curent_Send[4];
	

}Chassis_t;


//motor_measure_t *Get_DJI_Motor_Data();

#endif

