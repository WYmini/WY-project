#ifndef __Mode_Set_H
#define __Mode_Set_H

#include "main.h"
#include "remote_control.h"
#include "pid.h" 
#include "chassis_task.h"

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#define Chassis_First_Mode Chassis_Zero                  //底盘第一个模式    
#define Chassis_Second_Mode Chassis_RC_Control           //底盘第二个模式
//#define Chassis_Third_Mode Chassis_Follow_Gimbal         //底盘第三个模式

typedef enum     //枚举类型   里面是底盘的一些模式
{
	Chassis_Zero = 0,//无控制，传送0
	Chassis_Motionless,//无控制，用电机角度保持当前位置
	Chassis_RC_Control,//遥控器控制
//	Chassis_Follow_Gimbal,//底盘跟随云台
//	Chassis_Follow_Chassis,//我跟我自己
//	Chassis_Spin_Left,//小陀螺模式
//	Chassis_Spin_Right,//小陀螺模式
//	Chassis_Spin//小陀螺模式
}Chassis_Mode_t;

#endif
