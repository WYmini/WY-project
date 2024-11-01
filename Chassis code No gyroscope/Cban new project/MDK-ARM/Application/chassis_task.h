#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "pid.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "Mode_Set.h"

#define Chassis_Motor_Speed_Kp  980     //底盘电机P
#define Chassis_Motor_Speed_Ki  0.01   //底盘电机I
#define Chassis_Motor_Speed_Kd  0     //底盘电机D
#define Chassis_Motor_Speed_Maxout 10000
#define Chassis_Motor_Turn_Kp 12
#define Chassis_Motor_Turn_Ki 0
#define Chassis_Motor_Turn_Kd 0	
#define Chassis_Motor_Turn_Maxout 5
#define Chassis_Motor_Turn_IMaxout 0

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

#define Chassis_Min_Set_SPEED 5                         //最慢速度
#define Chassis_Max_Set_SPEED 10                         //最快速度
#define Chassis_Shift_Max_Speed   30                      //Shift加速最快速度

#define Chassis_Max_Speed_Sett 30

  
void Chassis_Task(void);
void Chassis_PID_Init(void); 

typedef struct
{
	//电机数据
	const motor_measure_t *motor_data;
	//经过转换后的电机转速
	float Chassis_Speed_Get;
}Chassos_Motor_Msg_t;


typedef struct                                   
{
	//底盘模式
	Chassis_Mode_t Chassis_Mode;
	
	 //电机相关信息
	Chassos_Motor_Msg_t Chassis_Motor_Msg[4];
             
	                                         
	                                         
	//裁判系统相关数据获取                   
//	Chassos_Judge_Msg_t Chassos_Judge_Msg;
	
	//超级电容相关数据获取
//	Chassis_Cap_Msg_t Chassos_Cap_Msg;
	
	//底盘电机相关pid设置
	PID Chassis_Motor_Pid[4];
	PID Chassis_Motor_Turn_Pid;
	
	//陀螺仪相关数据获取
//	const float* Chassis_IMU_Angle;
//	const float* Chassis_IMU_Aspeed;
	//遥控器控制值获取
	const RC_ctrl_t *rc_ctrl;	
	
	//底盘速度设置
	float Chassis_Control_Speed_Set[4];
	float Chassis_X_Speed_Set;
	float Chassis_Y_Speed_Set;
	float Chassis_LR_Speed_Set;
	//底盘跟随云台-云台机械角度差
//	float* Chassis_Follow_Gimbal_Angle_TM;
//	first_order_filter_type_t chassis_cmd_slow_set_vx;
//  first_order_filter_type_t chassis_cmd_slow_set_vy;
	//底盘最大速度
	float Chassis_Speed_Max;
	//底盘电机速度设置
	float Chassis_Motor_Speed_Set[4];
	
	//底盘发送电流值
	float Chassis_Motor_Curent_Send[4];
	

}Chassis_t;


//motor_measure_t *Get_DJI_Motor_Data();

#endif

