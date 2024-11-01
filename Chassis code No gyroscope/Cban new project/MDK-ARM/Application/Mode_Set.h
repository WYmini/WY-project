#ifndef __Mode_Set_H
#define __Mode_Set_H

#include "main.h"
#include "remote_control.h"
#include "pid.h" 
#include "chassis_task.h"

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#define Chassis_First_Mode Chassis_Zero                  //���̵�һ��ģʽ    
#define Chassis_Second_Mode Chassis_RC_Control           //���̵ڶ���ģʽ
//#define Chassis_Third_Mode Chassis_Follow_Gimbal         //���̵�����ģʽ

typedef enum     //ö������   �����ǵ��̵�һЩģʽ
{
	Chassis_Zero = 0,//�޿��ƣ�����0
	Chassis_Motionless,//�޿��ƣ��õ���Ƕȱ��ֵ�ǰλ��
	Chassis_RC_Control,//ң��������
//	Chassis_Follow_Gimbal,//���̸�����̨
//	Chassis_Follow_Chassis,//�Ҹ����Լ�
//	Chassis_Spin_Left,//С����ģʽ
//	Chassis_Spin_Right,//С����ģʽ
//	Chassis_Spin//С����ģʽ
}Chassis_Mode_t;

#endif
