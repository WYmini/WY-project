#include "stm32f4xx.h"                  // Device header

#include "chassis_task.h"
#include "pid.h"
#include "remote_control.h"
#include "bsp_rc.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "Mode_Set.h"
#include "usart.h"
#include "can.h"

Chassis_t Chassis;

//extern motor_measure_t motor_data[4];	//��������ṹ��ָ��
 


extern RC_ctrl_t *rc_ctrl;


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

struct PID Chassis_PID[4];


float Chassis_Speed_Get[4];

float Chassis_Speed_Get[4];


void Chassis_Data_Set_In_Four(uint16_t*  Data_Address,uint16_t Num)
{  
	for(int i = 0;i < 4;i++)
	Data_Address[i] = Num;
}


float Chassis_X_Speed_Set,Chassis_Y_Speed_Set,Chassis_LR_Speed_Set;

void Chassis_Init(Chassis_t* Chassis_Data_Init)   //����ģʽ��ʼ��  ���ݳ�ʼ��
{ 
	//����ģʽ��ʼ��
	Chassis_Data_Init -> Chassis_Mode =  Chassis_Zero ;   //���̳�ʼ������ -> ����ģʽö������ �ٵ� ����ö�����������0 
	
	 //���̵��PID��ʼ��
	PID_Init(&Chassis_Data_Init->Chassis_Motor_Pid[0],Chassis_Motor_Speed_Kp,Chassis_Motor_Speed_Ki,Chassis_Motor_Speed_Kd,Chassis_Motor_Speed_Maxout,0,2);
	PID_Init(&Chassis_Data_Init->Chassis_Motor_Pid[1],Chassis_Motor_Speed_Kp,Chassis_Motor_Speed_Ki,Chassis_Motor_Speed_Kd,Chassis_Motor_Speed_Maxout,0,2);
	PID_Init(&Chassis_Data_Init->Chassis_Motor_Pid[2],Chassis_Motor_Speed_Kp,Chassis_Motor_Speed_Ki,Chassis_Motor_Speed_Kd,Chassis_Motor_Speed_Maxout,0,2);
	PID_Init(&Chassis_Data_Init->Chassis_Motor_Pid[3],Chassis_Motor_Speed_Kp,Chassis_Motor_Speed_Ki,Chassis_Motor_Speed_Kd,Chassis_Motor_Speed_Maxout,0,2);
	PID_Init(&Chassis_Data_Init->Chassis_Motor_Turn_Pid,Chassis_Motor_Turn_Kp,Chassis_Motor_Turn_Ki,Chassis_Motor_Turn_Kd,Chassis_Motor_Turn_Maxout,Chassis_Motor_Turn_Kp,2);
	

	  Chassis_Data_Init->Chassis_Motor_Msg[0].motor_data = get_barrier_l_motor_measure_point();
		Chassis_Data_Init->Chassis_Motor_Msg[1].motor_data = get_barrier_r_motor_measure_point();
		Chassis_Data_Init->Chassis_Motor_Msg[2].motor_data = get_saver_l_motor_measure_point();
		Chassis_Data_Init->Chassis_Motor_Msg[3].motor_data = get_saver_r_motor_measure_point();
	
	//��ȡ���̵������
//   Chassis_Data_Init ->Chassis_Motor_Msg[0] =    *get_barrier_l_motor_measure_point();
//   Chassis_Data_Init ->Chassis_Motor_Msg[1] =  *get_barrier_r_motor_measure_point(); 
//   Chassis_Data_Init ->Chassis_Motor_Msg[2] =  *get_saver_l_motor_measure_point();   
//   Chassis_Data_Init ->Chassis_Motor_Msg[3] =  *get_saver_r_motor_measure_point();   
	//���̿�����������
	for(int i = 0;i < 4;i++)
	Chassis_Data_Init->Chassis_Motor_Speed_Set[i] = 0;
	
		//���̷�����������
	Chassis_Data_Set_In_Four((uint16_t*)Chassis_Data_Init->Chassis_Motor_Curent_Send,0);

		//ң��������ֵ��ȡ��ַ
	Chassis_Data_Init->rc_ctrl = get_remote_control_point();
	                                                  
}


/*****������ݸ��º���*****/
void Chassis_Motor_Data_Update(Chassos_Motor_Msg_t* Chassos_Motor_Msg_Update)
{
	Chassos_Motor_Msg_Update->Chassis_Speed_Get = ((float)(Chassos_Motor_Msg_Update->motor_data->speed_rpm)*0.0012708333f);///60*2*3.1415926 *152.5/2/1000
}

/*****����PID���º���*****/
float Chassis_pid_calc(PID*pid, float now, float set)
	{
    pid->now = now;
    pid->set = set;
		pid->now_error = pid->set - pid->now;	//set - measure
    if(pid->pid_mode == 1) //λ�û�PID
    {
	      pid->pout = pid->kp * pid->now_error;
        pid->iout = pid->ki * pid->sum_of_error;
        pid->dout = pid->kd * (pid->now_error - pid->Last_error);
				pid->sum_of_error+=pid->now_error;	
				PID_limit(&(pid->sum_of_error), 10000);
				PID_limit(&(pid->iout), pid->IntegralLimit);
        pid->out = pid->pout + pid->iout + pid->dout;
        PID_limit(&(pid->out), pid->MaxOutput);
    }	
		
    else if(pid->pid_mode == 2)//����ʽPID
    {
        pid->pout = pid->kp * (pid->now_error - pid->Last_error);
        pid->iout = pid->ki * pid->now_error;
        pid->dout = pid->kd * (pid->now_error - 2*pid->Last_error + pid->Last_Last_error);        
				PID_limit(&(pid->iout), pid->IntegralLimit);
        pid->plus = pid->pout + pid->iout + pid->dout;
        pid->plus_out = pid->last_plus_out + pid->plus;
			  pid->out = pid->plus_out; 
				PID_limit(&(pid->out), pid->MaxOutput);
        pid->last_plus_out = pid->plus_out;	//update last time
    }
		
    pid->Last_Last_error= pid->Last_error;
    pid->Last_error = pid->now_error;

    return pid->out;
}

/*****����PID���º���*****/
void Chassis_PID_Calculate_Data(Chassis_t* Chassis_PID)
{
	if(Chassis_PID->Chassis_Mode == Chassis_Zero)
	{
		Chassis_PID->Chassis_Motor_Curent_Send[0] = 0;
		Chassis_PID->Chassis_Motor_Curent_Send[1] = 0;
		Chassis_PID->Chassis_Motor_Curent_Send[2] = 0;
		Chassis_PID->Chassis_Motor_Curent_Send[3] = 0;
	}
	else  
	{
		Chassis_PID->Chassis_Motor_Curent_Send[0] = Chassis_pid_calc(&Chassis_PID->Chassis_Motor_Pid[0],Chassis_PID->Chassis_Motor_Msg[0].Chassis_Speed_Get,Chassis_PID->Chassis_Motor_Speed_Set[0]);
		Chassis_PID->Chassis_Motor_Curent_Send[1] = Chassis_pid_calc(&Chassis_PID->Chassis_Motor_Pid[1],Chassis_PID->Chassis_Motor_Msg[1].Chassis_Speed_Get,Chassis_PID->Chassis_Motor_Speed_Set[1]);
		Chassis_PID->Chassis_Motor_Curent_Send[2] = Chassis_pid_calc(&Chassis_PID->Chassis_Motor_Pid[2],Chassis_PID->Chassis_Motor_Msg[2].Chassis_Speed_Get,Chassis_PID->Chassis_Motor_Speed_Set[2]);
		Chassis_PID->Chassis_Motor_Curent_Send[3] = Chassis_pid_calc(&Chassis_PID->Chassis_Motor_Pid[3],Chassis_PID->Chassis_Motor_Msg[3].Chassis_Speed_Get,Chassis_PID->Chassis_Motor_Speed_Set[3]);
	}
}
 
/*****�������ݸ��º���*****/
void Chassis_Data_Update(Chassis_t* Chassis_Update)
{
	for(int i = 0;i < 4;i++)
	Chassis_Motor_Data_Update(&Chassis_Update->Chassis_Motor_Msg[i]);
//	Chassis_Cap_Data_Update(&Chassis_Update->Chassos_Cap_Msg);
//	Chassis_Judge_Data_Update(&Chassis_Update->Chassos_Judge_Msg);
}

/*     ��һ��ģʽ     */
void Chassis_Mode_Zero(Chassis_t* Chassis_Zero_Set)
{
			 Chassis_Zero_Set->Chassis_X_Speed_Set = 0;
			 Chassis_Zero_Set->Chassis_Y_Speed_Set = 0;
			 Chassis_Zero_Set->Chassis_LR_Speed_Set = 0;
}
/*     ��һ��ģʽ     */
void Chassis_Mode_Motionless(Chassis_t* Chassis_Motionless_Set)
{
			Chassis_Motionless_Set->Chassis_X_Speed_Set = 0;
			Chassis_Motionless_Set->Chassis_Y_Speed_Set = 0;
			Chassis_Motionless_Set->Chassis_LR_Speed_Set = 0;
}
/*     �ڶ���ģʽ     */
void Chassis_Mode_RC(Chassis_t * Chassis_RC_Mode_Set)
{
     	Chassis_RC_Mode_Set->Chassis_X_Speed_Set = ((float)(Chassis_RC_Mode_Set->rc_ctrl->rc.ch[0])/660 * Chassis_Max_Set_SPEED);
			Chassis_RC_Mode_Set->Chassis_Y_Speed_Set = ((float)(Chassis_RC_Mode_Set->rc_ctrl->rc.ch[1])/660 * Chassis_Max_Set_SPEED);

}


/*****���̿������ݸ��º���*****/

void Chassis_Control_Data_Get(Chassis_t* Chassis_Control_Data)
{
	switch(Chassis_Control_Data->Chassis_Mode)
	{
		case Chassis_Zero:  //����Ϊ0 ���ϵ�
			Chassis_Mode_Zero(Chassis_Control_Data);
		break;
		
		case Chassis_Motionless:
			Chassis_Mode_Motionless(Chassis_Control_Data);			
		break;
		
		case Chassis_RC_Control:	   
		Chassis_Mode_RC(Chassis_Control_Data);
		break;
		
//    case Chassis_Follow_Chassis:
//    Chassis_Control_Data->Chassis_X_Speed_Set = ((float)(Chassis_Control_Data->rc_ctrl->rc.ch[0])/660.000f * 20.00f);
//    Chassis_Control_Data->Chassis_Y_Speed_Set = ((float)(Chassis_Control_Data->rc_ctrl->rc.ch[1])/660.000f * 20.00f);
//    Chassis_Control_Data->Chassis_LR_Speed_Set = ((float)(Chassis_Control_Data->rc_ctrl->rc.ch[2])/660.000f * 20.00f);// - (float)((Chassis_Control_Data->Chassis_RC_Ctl_Data->mouse.x) *Chassis_Mouse_X_Set);			
	}
	
		Chassis_Control_Data->Chassis_Motor_Speed_Set[0] = Chassis_Control_Data->Chassis_X_Speed_Set - Chassis_Control_Data->Chassis_Y_Speed_Set - Chassis_Control_Data->Chassis_LR_Speed_Set;
		Chassis_Control_Data->Chassis_Motor_Speed_Set[1] = Chassis_Control_Data->Chassis_X_Speed_Set + Chassis_Control_Data->Chassis_Y_Speed_Set - Chassis_Control_Data->Chassis_LR_Speed_Set;
		Chassis_Control_Data->Chassis_Motor_Speed_Set[2] = -Chassis_Control_Data->Chassis_X_Speed_Set + Chassis_Control_Data->Chassis_Y_Speed_Set - Chassis_Control_Data->Chassis_LR_Speed_Set;
		Chassis_Control_Data->Chassis_Motor_Speed_Set[3] = -Chassis_Control_Data->Chassis_X_Speed_Set - Chassis_Control_Data->Chassis_Y_Speed_Set - Chassis_Control_Data->Chassis_LR_Speed_Set;	
		
	for(int i = 0;i < 4;i++)
	{
		if(Chassis_Control_Data->Chassis_Motor_Speed_Set[i] > Chassis_Shift_Max_Speed)
		{
			Chassis_Control_Data->Chassis_Motor_Speed_Set[i] = Chassis_Control_Data->Chassis_Motor_Speed_Set[i] * Chassis_Control_Data->Chassis_Motor_Speed_Set[i]/Chassis_Shift_Max_Speed;
		}
	}
		
}


/********   ����ģʽ���º���   ********/
void Chassis_Mode_Set(Chassis_t *Chassis_Mode)     //ģʽ����
{
    if(Chassis_Mode->rc_ctrl->rc.s[1] == 2 )       //�����ұ� ��
		{
			Chassis_Mode->Chassis_Mode = Chassis_First_Mode;
    } 
 		else if (Chassis_Mode->rc_ctrl->rc.s[1] == 3 )    //�����ұ�  ��
		{
			Chassis_Mode->Chassis_Mode = Chassis_Second_Mode;
		}
//		else if (Chassis_Mode->rc_ctrl->rc.s[1] == 1 )    //�����ұ�  ��
//		{
//		    Chassis_Mode->Chassis_Mode = Chassis_Third_Mode;
//		}
}


void Chassis_Task()   //�������� 
{
	Chassis_Init(&Chassis);	  //���̳�ʼ��
	for(int ii = 0;ii < 4;ii++)
	PID_Init(&Chassis_PID[ii],8.8,0,0,9000,5000,2);
	
	while(1)
	{		
		
		Chassis_Mode_Set(&Chassis);
		//Chassis_Motter_Data(&Chassis);//��ȡ���������
		Chassis_Control_Data_Get(&Chassis);
		Chassis_Data_Update(&Chassis);
		Chassis_Mode_RC(&Chassis);   //�ڶ���ģʽ��	
    Chassis_PID_Calculate_Data(&Chassis);
    CAN_cmd_chassis((int16_t)Chassis.Chassis_Motor_Curent_Send[0],(int16_t)Chassis.Chassis_Motor_Curent_Send[1],(int16_t)Chassis.Chassis_Motor_Curent_Send[2],(int16_t)Chassis.Chassis_Motor_Curent_Send[3]);
		HAL_Delay(50);
	}
}

Chassis_Mode_t* Return_Chassis_Mode_Add(void)
{
	return &Chassis.Chassis_Mode;
}



