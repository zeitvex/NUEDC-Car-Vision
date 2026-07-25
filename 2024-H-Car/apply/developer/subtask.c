#include "headfile.h"
#include "subtask.h"

#define SUBTASK_NUM 30
#define flight_subtask_delta 5//5ms



unsigned char mp3_1[15]={0xAA, 0x08, 0x0B, 0x01, 0x2F, 0x30, 0x30, 0x30, 0x30, 0x31, 0x2A, 0x4D, 0x50, 0x33, 0xD8};
unsigned char mp3_2[15]={0xAA, 0x08, 0x0B, 0x01, 0x2F, 0x30, 0x30, 0x30, 0x30, 0x32, 0x2A, 0x4D, 0x50, 0x33, 0xD9};
char mp3_3[15]={0xAA, 0x08, 0x0B, 0x01, 0x2F, 0x30, 0x30, 0x30, 0x30, 0x33, 0x2A, 0x4D, 0x50, 0x33, 0xDA};
char mp3_4[15]={0xAA, 0x08, 0x0B, 0x01, 0x2F, 0x30, 0x30, 0x30, 0x30, 0x34, 0x2A, 0x4D, 0x50, 0x33, 0xDB};
char mp3_5[15]={0xAA, 0x08, 0x0B, 0x01, 0x2F, 0x30, 0x30, 0x30, 0x30, 0x35, 0x2A, 0x4D, 0x50, 0x33, 0xDC};

extern char gray_flag_1;

uint16_t flight_subtask_cnt[SUBTASK_NUM]={0};//飞行任务子线程计数器，可以用于控制每个航点子线程的执行
uint32_t flight_global_cnt[SUBTASK_NUM]={0}; //飞行任务子线全局计数器，可以结合位置偏差用于判断判断航点是否到达
uint32_t execute_time_ms[SUBTASK_NUM]={0};//飞行任务子线执行时间，可以用于设置某个子线程的执行时间

void flight_subtask_reset(void)
{
	for(uint16_t i=0;i<SUBTASK_NUM;i++)
	{
		flight_subtask_cnt[i]=0;
		execute_time_ms[i]=0;
		flight_global_cnt[i]=0;
	}
}



void flight_subtask_1(void)//顺时针转90°
{
	static uint8_t n=0;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =45;//顺时针90度	
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;
		trackless_output.yaw_outer_control_output  =0;
		if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段	
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
	}
	else//其它情形
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];		
	}
}

void flight_subtask_2(void)//逆时针转90°
{
	static uint8_t n=1;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =40;//逆时针90度
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;
		trackless_output.yaw_outer_control_output  =0;
		
		if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段	
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
	}
	else//其它情形
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];		
	}
}

char turn_r(float a)
{
	static uint8_t n=1;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =a;
		flight_subtask_cnt[n]=1;		
		return 0;
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;
		trackless_output.yaw_outer_control_output  =0;
		if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段	
		return 0;
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
		return 1;
	}
	else//其它情形
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];	
		return 1;		
	}
}

char turn_l(float a)
{
	static uint8_t n=1;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =a;
		flight_subtask_cnt[n]=1;		
		return 0;
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;
		trackless_output.yaw_outer_control_output  =0;
		if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段	
		return 0;
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
		return 1;
	}
	else//其它情形
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];	
		return 1;		
	}
}























































char turn_jdz(float a)
{
	static uint8_t n=1;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=AZIMUTH;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =a;
		flight_subtask_cnt[n]=1;		
		return 0;
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;
		trackless_output.yaw_outer_control_output  =0;
		if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段	
		return 0;
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
		return 1;
	}
	else//其它情形
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];	
		return 1;		
	}
}
//以30deg/s的角速度顺时针转动3000ms，完成后降落
void flight_subtask_3(void)
{
	static uint8_t n=2;
	if(flight_subtask_cnt[n]==0)
	{
		
		trackless_output.yaw_ctrl_mode=CLOCKWISE_TURN;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =30;//以30deg/s的角速度顺时针转动3000ms
		trackless_output.execution_time_ms=3000;//执行时间
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE_TURN;
		trackless_output.yaw_outer_control_output  =0;
		
	  if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段		
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
	}
	else
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
	}
}



void flight_subtask_5(void)
{
	static uint8_t n=4;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
		speed_setup=40;//前进
		if(rangefinder.distance<50)//前方距离比较小，继续转动
		{
			speed_setup=0;//停止
			flight_subtask_cnt[n]=1;	
		}			
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =90;//顺时针90度
		
		speed_setup=0;//停止		
		flight_subtask_cnt[n]=2;		
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;
		trackless_output.yaw_outer_control_output  =0;
		
		if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=3;//执行完毕后，切换到下一阶段	
	}
	else if(flight_subtask_cnt[n]==3)//转动完毕后，继续判断前方距离
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
		if(rangefinder.distance<50)//前方距离比较小，继续转动
		{
			flight_subtask_cnt[n]=1;//继续转动	
		}
		else
		{
			flight_subtask_cnt[n]=0;//恢复前进	
		}
	}
	else//其它情形
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
		speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定
	}
} 


#define start_point_precision_cm 0.5f
#define distance_precision_cm 1.0f
#define steer_execute_time_ms 500
#define steer_value_default 300
#define freedom_time_ms 5000					//在车库里面停留时间



_park_params park_params={
	._track_speed_cmps=track_speed_cmps_default,
	._start_point_adjust1=start_point_adjust1_default,
	._forward_distance_cm=forward_distance_cm_default,
	._backward_distance1_cm=backward_distance1_cm_default,
	._backward_distance2_cm=backward_distance2_cm_default,
	._out_forward_distance1_cm=out_forward_distance1_cm_default,
	._out_forward_distance2_cm=out_forward_distance2_cm_default,
	._start_point_adjust2=start_point_adjust2_default,
	._parallel_backward_distance1_cm=parallel_backward_distance1_cm_default,
	._parallel_backward_distance2_cm=parallel_backward_distance2_cm_default,
	._parallel_backward_distance3_cm=parallel_backward_distance3_cm_default
};


void auto_reverse_stall_park(void)
{
	static uint8_t n=5;
	static float steer_gradient_cnt=0;
	static float servo_ctrl_value=300;
	if(flight_subtask_cnt[n]==0)//第一阶段自主寻迹
	{
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		vision_turn_control_50hz(&turn_ctrl_pwm);//基于OPENMV视觉处理的转向控制
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+turn_ctrl_pwm);	
		//期望速度
		speed_expect[0]=park_params._track_speed_cmps;//左边轮子速度期望
		speed_expect[1]=park_params._track_speed_cmps;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		//判断途径车库边线情况
		if(camera1.carpark_num==3)//经过库2
		{
			flight_subtask_cnt[n]++;
			//继续前进5cm
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._start_point_adjust1;			
		}
	}
	else if(flight_subtask_cnt[n]==1)//视觉识别到车库特征点后,执行继续前进或者后退
	{
		if(park_params._start_point_adjust1>0)//如果是向前调整开始倒车点，则继续寻迹
		{
			vision_turn_control_50hz(&turn_ctrl_pwm);//基于OPENMV视觉处理的转向控制
		}
		else turn_ctrl_pwm=0;//轮胎回正后继续倒车
		
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+turn_ctrl_pwm);	
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		if(flight_global_cnt[n]<20)//连续20次满足位置偏差很小,即认为位置控制完成
		{
			if(ABS(distance_ctrl.error)<start_point_precision_cm)	flight_global_cnt[n]++;	
			else flight_global_cnt[n]/=2;			
		}
		else 
		{
			flight_global_cnt[n]=0;
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=-steer_value_default;
			steer_gradient_cnt=execute_time_ms[n];
		}
	}
	else if(flight_subtask_cnt[n]==2)//第二阶段停车后,轮子左打
	{
		float steer_gradient_value=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value);
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._forward_distance_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==3)//第三阶段
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=steer_value_default;
			//期望速度
			//speed_expect[0]=0;//左边轮子速度期望
			//speed_expect[1]=0;//右边轮子速度期望
			steer_gradient_cnt=execute_time_ms[n];	
		}
	}
	else	if(flight_subtask_cnt[n]==4)//第四阶段
	{
		float steer_gradient_value0_1=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);//从0渐变到1
		float steer_gradient_value_n1_p1=(steer_gradient_value0_1-0.5f)/0.5f;//从-1渐变到1
		
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value_n1_p1);
		
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
	  //期望速度
		speed_expect[0]=0;//左边轮子速度期望
		speed_expect[1]=0;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance-park_params._backward_distance1_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==5)//第五阶段
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=steer_value_default;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			steer_gradient_cnt=execute_time_ms[n];		
		}
	}
	else	if(flight_subtask_cnt[n]==6)//第六阶段
	{
		float steer_gradient_value0_1=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);//从0渐变到1
		float steer_gradient_value_1_0=1.0f-steer_gradient_value0_1;//从1渐变到0
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value_1_0);
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance-park_params._backward_distance2_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==7)//第五阶段
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=freedom_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=0;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望		
		}
	}
	else if(flight_subtask_cnt[n]==8)//在车尾停泊5S
	{
		//转向控制
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._out_forward_distance1_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==9)//出库先前进
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=300;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望		
		}	
	}
	else if(flight_subtask_cnt[n]==10)
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		speed_control_100hz(speed_ctrl_mode);
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._out_forward_distance2_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==11)//出库转向
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=0;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望		
		}	
	}
	else if(flight_subtask_cnt[n]==12)//倒车入库结束，接下一进程
	{
		for(uint16_t i=0;i<4;i++)
		{
			camera1.carpark_flag[0][i]=0;
			camera1.carpark_flag[1][i]=0;
		}
		camera1.carpark_num=0;
		sdk_work_mode+=1;
	}
	else
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//速度控制
		speed_control_100hz(speed_ctrl_mode);	
	}
}



void auto_parallel_park(void)
{
	static uint8_t n=6;	
	static float steer_gradient_cnt=0;
	static float servo_ctrl_value=300;
	
	if(flight_subtask_cnt[n]==0)//第一阶段自主寻迹
	{
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		vision_turn_control_50hz(&turn_ctrl_pwm);//基于OPENMV视觉处理的转向控制
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+turn_ctrl_pwm);	
		//期望速度
		speed_expect[0]=park_params._track_speed_cmps;//左边轮子速度期望
		speed_expect[1]=park_params._track_speed_cmps;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);

		//判断途径车库边线情况
		if(camera1.carpark_num==3)//经过库2
		{
			flight_subtask_cnt[n]++;
			//继续前进5cm
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._start_point_adjust2;			
		}
	}
	else if(flight_subtask_cnt[n]==1)//执行继续前进10cm
	{
		if(park_params._start_point_adjust2>0)//如果是向前调整开始倒车点，则继续寻迹
		{
			vision_turn_control_50hz(&turn_ctrl_pwm);//基于OPENMV视觉处理的转向控制
		}
		else turn_ctrl_pwm=0;//轮胎回正后继续倒车
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+turn_ctrl_pwm);	
		
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		if(flight_global_cnt[n]<20)
		{
			if(ABS(distance_ctrl.error)<start_point_precision_cm)	flight_global_cnt[n]++;	
			else flight_global_cnt[n]/=2;			
		}
		else 
		{
			flight_global_cnt[n]=0;
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=steer_value_default;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			steer_gradient_cnt=execute_time_ms[n];
		}
	}
	else if(flight_subtask_cnt[n]==2)//第二阶段刹车、转向
	{
		float steer_gradient_value=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value);
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance-park_params._parallel_backward_distance1_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==3)//第三阶段
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=steer_value_default;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			steer_gradient_cnt=execute_time_ms[n];	
		}
	}
	else if(flight_subtask_cnt[n]==4)//车头回正
	{
		float steer_gradient_value0_1=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);//从0渐变到1
		float steer_gradient_value_1_0=1.0f-steer_gradient_value0_1;//从1渐变到0
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value_1_0);
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance-park_params._parallel_backward_distance2_cm;
		}			
	}
	else if(flight_subtask_cnt[n]==5)//竖直后退
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=-steer_value_default;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			steer_gradient_cnt=execute_time_ms[n];	
		}	
	}
	else if(flight_subtask_cnt[n]==6)//车轮向左侧打
	{
		float steer_gradient_value=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value);
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance-park_params._parallel_backward_distance3_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==7)//车轮向左侧打并后退
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=freedom_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=0;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
		}		
	}
	else if(flight_subtask_cnt[n]==8)//车轮直接回正,原地等待5s
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		speed_setup=0;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=-steer_value_default;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			steer_gradient_cnt=execute_time_ms[n];
		}	
	}
	else if(flight_subtask_cnt[n]==9)//轮胎向左打，准备出库
	{
		float steer_gradient_value=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value);	
		//期望速度
		speed_expect[0]=0;//左边轮子速度期望
		speed_expect[1]=0;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);	
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=-steer_value_default;	
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._parallel_backward_distance3_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==10)//车轮向左侧打并前进
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=-steer_value_default;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			steer_gradient_cnt=execute_time_ms[n];
		}		
	}
	else if(flight_subtask_cnt[n]==11)//轮胎逐渐回正后继续前进
	{
		float steer_gradient_value0_1=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);//从0渐变到1
		float steer_gradient_value_1_0=1.0f-steer_gradient_value0_1;//从1渐变到0
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value_1_0);
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._parallel_backward_distance2_cm;
		}			
	}
	else if(flight_subtask_cnt[n]==12)//回正后继续前进
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=steer_value_default;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			steer_gradient_cnt=execute_time_ms[n];
		}		
	}
	else if(flight_subtask_cnt[n]==13)//轮胎向右打
	{
		float steer_gradient_value=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value);	
		//期望速度
		speed_expect[0]=0;//左边轮子速度期望
		speed_expect[1]=0;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);	
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=steer_value_default;	
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._parallel_backward_distance1_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==14)//轮胎向右打后继续前进
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//距离控制
		distance_control();
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup;//左边轮子速度期望
		speed_expect[1]=speed_setup;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=3000/flight_subtask_delta;//子任务执行时间;
			servo_ctrl_value=0;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
		}
	}
	else if(flight_subtask_cnt[n]==15)//继续巡线
	{
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		vision_turn_control_50hz(&turn_ctrl_pwm);//基于OPENMV视觉处理的转向控制
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+turn_ctrl_pwm);	
		//期望速度
		speed_expect[0]=park_params._track_speed_cmps;//左边轮子速度期望
		speed_expect[1]=park_params._track_speed_cmps;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);	
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			servo_ctrl_value=0;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
		}	
	}
	else//巡线结束后停止
	{
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//速度控制
		speed_expect[0]=0;//左边轮子速度期望
		speed_expect[1]=0;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);		
	}
}

bool mp3_flag=0;
void mp3(){
	if(mp3_flag==0)return;
	mp3_flag=0;
	usart1_send_bytes(mp3_1,15);
	
}

void kaishi1(){
	static uint8_t n=10;
	bool a=0;
	static float steer_gradient_cnt=0;
	mp3();
	if(flight_subtask_cnt[n]==0){
		execute_time_ms[n]=1000/5;
		mp3_flag=1;
		flight_subtask_cnt[n]=1;
	}
	else if(flight_subtask_cnt[n]==1){
		speed_ctrl_mode=1;		
		turn_r(38.6);
		steer_control(&turn_ctrl_pwm);
		speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
		//期望速度			
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);		
	  if(execute_time_ms[n]>0)execute_time_ms[n]--;
		else if(execute_time_ms[n]==0){
			distance_ctrl.expect=128;
			flight_subtask_cnt[n]=2;
		}
	}
	else if(flight_subtask_cnt[n]==2){
		distance_control();
		turn_r(0);
		steer_control(&turn_ctrl_pwm);
		speed_setup=distance_ctrl.output;
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);	
		if(ABS(distance_ctrl.error)<0.5f){
			flight_subtask_cnt[n]=3;
		}
	}
	else if(flight_subtask_cnt[n]==3){
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		//gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		if(execute_time_ms[n]>0)execute_time_ms[n]--;
		
		
		else if(execute_time_ms[n]==0){
			execute_time_ms[n]=6000/5;
			flight_subtask_cnt[n]=4;
		}
	}
	if(flight_subtask_cnt[n]==4){
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		steer_control(&turn_ctrl_pwm);
		if(read_gray_bit3){
			speed_expect[0]=25;//左边轮子速度期望
			speed_expect[1]=15;
		}
		else {
			speed_expect[0]=20;
			speed_expect[1]=20;
		}
		//speed_expect[1]=20;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);	
		if(execute_time_ms[n]>0)execute_time_ms[n]--;
		else if(execute_time_ms[n]==0){
			flight_subtask_cnt[n]=10;
		}
	}
}

// 低通滤波函数
float low_pass_filter(float input, float prev_output, float alpha) {
    return alpha * input + (1 - alpha) * prev_output;
}
float gra_con =0;
void kaishi() {
    static uint8_t n = 10;
    static float steer_gradient_cnt = 0;
    static float filtered_turn_ctrl_pwm = 0;
    static float filtered_distance_output = 0;
    const float alpha = 0.1; // 低通滤波器系数
    if (flight_subtask_cnt[n] == 0) {
        execute_time_ms[n] = 1000 / 5;
        mp3_flag = 1;
        flight_subtask_cnt[n] = 1;
    } 
    else if (flight_subtask_cnt[n] == 1) {
        speed_ctrl_mode = 1;
        turn_r(40);
        steer_control(&turn_ctrl_pwm);
        filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        speed_setup = RC_Data.rc_rpyt[RC_PITCH]; // 速度期望来源于俯仰杆给定
        // 期望速度
        speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale; // 左边轮子速度期望
        speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale; // 右边轮子速度期望  
        // 速度控制
        speed_control_100hz(speed_ctrl_mode);
        if(trackless_output.yaw_ctrl_end==1){
					flight_subtask_cnt[n]=2;
					distance_ctrl.expect=116;
					flight_subtask_cnt[1]=0;
				}
    } 
    else if (flight_subtask_cnt[n] == 2) {
        distance_control();
        turn_r(0);
        steer_control(&turn_ctrl_pwm);
        filtered_distance_output = low_pass_filter(distance_ctrl.output, filtered_distance_output, alpha);
        filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        // 期望速度
        speed_expect[0] = filtered_distance_output + filtered_turn_ctrl_pwm * steer_gyro_scale; // 左边轮子速度期望
        speed_expect[1] = filtered_distance_output - filtered_turn_ctrl_pwm * steer_gyro_scale; // 右边轮子速度期望
        speed_control_100hz(speed_ctrl_mode);       
        if (ABS(distance_ctrl.error) < 0.5f) {
            execute_time_ms[n] = 500 / 5;
						flight_subtask_cnt[1] = 0;
            flight_subtask_cnt[n] = 3;
        }
    } 
		else if (flight_subtask_cnt[n] == 3) {
      speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			turn_l(40);
			steer_control(&turn_ctrl_pwm);
      filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
      speed_setup = RC_Data.rc_rpyt[RC_PITCH]; // 速度期望来源于俯仰杆给定
      // 期望速度
      speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale; // 左边轮子速度期望
      speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale; // 右边轮子速度期望  
      // 速度控制
      speed_control_100hz(speed_ctrl_mode);
			if(trackless_output.yaw_ctrl_end==1){
					flight_subtask_cnt[n]=4;
					execute_time_ms[n] = 500000 / 5;
					flight_subtask_cnt[1]=0;
				}
    }
    else if (flight_subtask_cnt[n] == 4) {
      speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			//turn_l(180);
			//steer_control(&turn_ctrl_pwm);
			speed_expect[0]=25;//左边轮子速度期望
			speed_expect[1]=25;//右边轮子速度期望
			speed_control_100hz(speed_ctrl_mode);
			if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) {
					flight_subtask_cnt[n]=5;
					flight_subtask_cnt[1]=0;
				}
    }
    else if (flight_subtask_cnt[n] == 5){ 
        speed_ctrl_mode = 1;
        speed_expect[0] = 0;
        speed_expect[1] = 0;
        speed_control_100hz(speed_ctrl_mode);
				//flight_subtask_reset();
    }
}

void go_work(){
	static uint8_t n=16;
	static float steer_gradient_cnt=0;
	flight_subtask_cnt[n]=0;
	if(flight_subtask_cnt[n]==0){
		execute_time_ms[n]=4000/5;
		flight_subtask_cnt[n]=1;
	}
	else if (flight_subtask_cnt[n] == 1) {
      speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			//turn_l(180);
			//steer_control(&turn_ctrl_pwm);
			speed_expect[0]=25*0.68;//左边轮子速度期望
			speed_expect[1]=25;//右边轮子速度期望
			speed_control_100hz(speed_ctrl_mode);
			if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) {
					flight_subtask_cnt[n]=2;
					flight_subtask_cnt[1]=0;
				}
    }
    else if (flight_subtask_cnt[n] == 2){ 
        speed_ctrl_mode = 1;
        //speed_setup = 0; // 速度期望来源于俯仰杆给定
        speed_expect[0] = 0;
        speed_expect[1] = 0;
        speed_control_100hz(speed_ctrl_mode);
				//flight_subtask_reset();
    }
}

void renwu_1(){
	static uint8_t n = 19;
  static float steer_gradient_cnt = 0;
  static float filtered_turn_ctrl_pwm = 0;
  static float filtered_distance_output = 0;
  const float alpha = 0.1; // 低通滤波器系数
	mp3();
	if(flight_subtask_cnt[n]==0){
		mp3_flag=1;
		flight_subtask_cnt[n]=1;
	}
	//线程1，空白区域直线行走
	else if (flight_subtask_cnt[n] == 1) {
		//distance_control();
		turn_r(0);
		steer_control(&turn_ctrl_pwm);
		filtered_distance_output = low_pass_filter(distance_ctrl.output, filtered_distance_output, alpha);
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
		speed_expect[0] = 80 + filtered_turn_ctrl_pwm * steer_gyro_scale; // 左边轮子速度期望
		speed_expect[1] = 80 - filtered_turn_ctrl_pwm * steer_gyro_scale; // 右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
    if(read_gray_bit3==0x01||read_gray_bit4==0x01||read_gray_bit5==0x01||read_gray_bit6==0x01){
			distance_ctrl.expect= 0;
			distance_ctrl.output=0;
			flight_subtask_cnt[1]=0;
			execute_time_ms[n] = 2600/5;
		  buzzer_setup(200, 0.5, 2);//报警
      flight_subtask_cnt[n] = 2;
    }
	}
	else if (flight_subtask_cnt[n] == 2){
		speed_ctrl_mode = 1;
    speed_expect[0] = 0;
    speed_expect[1] = 0;
    speed_control_100hz(speed_ctrl_mode);
	}
}
void renwu_2(){//ok
	static uint8_t n = 19;
  static float steer_gradient_cnt = 0;
  static float filtered_turn_ctrl_pwm = 0;
  static float filtered_distance_output = 0;
  const float alpha = 0.1; // 低通滤波器系数
	mp3();
	if(flight_subtask_cnt[n]==0){
		mp3_flag=1;
	//	distance_ctrl.expect = 102;
		flight_subtask_cnt[n]=1;
		//execute_time_ms[n] = 2000 / 5;
	}
	//线程1，空白区域直线行走
	else if (flight_subtask_cnt[n] == 1) {
		//distance_control();
		turn_r(0);
		steer_control(&turn_ctrl_pwm);
		filtered_distance_output = low_pass_filter(distance_ctrl.output, filtered_distance_output, alpha);
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
		speed_expect[0] = 55 + filtered_turn_ctrl_pwm * steer_gyro_scale; // 左边轮子速度期望
		speed_expect[1] = 55 - filtered_turn_ctrl_pwm * steer_gyro_scale; // 右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
    if(read_gray_bit3==0x01||read_gray_bit4==0x01||read_gray_bit5==0x01||read_gray_bit6==0x01){
			distance_ctrl.expect= 0;
			distance_ctrl.output=0;
			flight_subtask_cnt[1]=0;
			execute_time_ms[n] = 2600/5;
		  buzzer_setup(200, 0.5, 2);//报警
      flight_subtask_cnt[n] = 2;
    }
	}
	
	//线程2，红外循迹
	else if (flight_subtask_cnt[n] == 2) {
		char a;
		float b;
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		gray_turn_control_200hz(&turn_ctrl_pwm,&a);//基于灰度对管的转向控制
		//steer_control(&b);
			//filtered_distance_output = low_pass_filter(distance_ctrl.output, filtered_distance_output, alpha);
			filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
			//期望速度
			speed_expect[0]=45+turn_ctrl_pwm*turn_scale*(a?0:1);//左边轮子速度期望
			speed_expect[1]=45-turn_ctrl_pwm*turn_scale*(a?0:1);//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);		
			if(execute_time_ms[n]>0) execute_time_ms[n]--;
			else if(execute_time_ms[n]==0){
				speed_expect[0]=0;
				speed_expect[1]=0;
				turn_ctrl_pwm=0;
				flight_subtask_cnt[n] = 3;
				flight_subtask_cnt[1]=0;
				buzzer_setup(200, 0.5, 2);
		}
	}
	
	//线程3，循迹过后矫正姿态
	else if (flight_subtask_cnt[n] == 3) {	 
	speed_ctrl_mode = 1;
    turn_jdz(-179);
		//buzzer_setup(200, 0.5, 2);//报警	
    steer_control(&turn_ctrl_pwm);
    filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
    speed_setup = RC_Data.rc_rpyt[RC_PITCH]; // 速度期望来源于俯仰杆给定
    // 期望速度
    speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale; // 左边轮子速度期望
    speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale; // 右边轮子速度期望  
    // 速度控制
    speed_control_100hz(speed_ctrl_mode);
    if(trackless_output.yaw_ctrl_end==1){
			flight_subtask_cnt[1]=0;
			flight_subtask_cnt[n]=4;
		}
	}
	//线程4，通过直线空白区域
	else if (flight_subtask_cnt[n] == 4) {
		turn_r(0);
		steer_control(&turn_ctrl_pwm);
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
		// 期望速度
		speed_expect[0] = 55 + filtered_turn_ctrl_pwm * steer_gyro_scale; // 左边轮子速度期望
		speed_expect[1] = 55 - filtered_turn_ctrl_pwm * steer_gyro_scale; // 右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);       
		if(read_gray_bit3==0x01||read_gray_bit4==0x01||read_gray_bit5==0x01||read_gray_bit6==0x01){
			execute_time_ms[n] = 2600/5;
			flight_subtask_cnt[1] = 0;
			flight_subtask_cnt[n] = 5;
			buzzer_setup(200, 0.5, 2);//报警	
		}
	} 
	//线程5，完成最后半圆的循迹
	else if (flight_subtask_cnt[n] == 5) {
		char a;
		float b;
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		//turn_jdz(-180);
		gray_turn_control_200hz(&turn_ctrl_pwm,&a);//基于灰度对管的转向控制
		//steer_control(&b);
			//filtered_distance_output = low_pass_filter(distance_ctrl.output, filtered_distance_output, alpha);
			filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
			//期望速度
			speed_expect[0]=45+turn_ctrl_pwm*turn_scale*(a?0:1);//左边轮子速度期望
			speed_expect[1]=45-turn_ctrl_pwm*turn_scale*(a?0:1);//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);		
			if(execute_time_ms[n]>0) execute_time_ms[n]--;
			else if(execute_time_ms[n]==0){
				flight_subtask_cnt[n] = 6;
				flight_subtask_cnt[1]=0;
				buzzer_setup(200, 0.5, 2);
		}
	}
	//线程6，任务完成，停止运动
	else if (flight_subtask_cnt[n] == 6){
		speed_ctrl_mode = 1;
    speed_expect[0] = 0;
    speed_expect[1] = 0;
    speed_control_100hz(speed_ctrl_mode);
	}
}


void renwu_3(){
    static uint8_t n = 21;
    static float steer_gradient_cnt = 0;
    static float filtered_turn_ctrl_pwm = 0;
    static float filtered_distance_output = 0;
    const float alpha = 0.2; // 低通滤波器系数
    if (flight_subtask_cnt[n] == 0) {
        mp3_flag = 1;
        flight_subtask_cnt[n] = 1;
    }
    // 线程1，转向特定角度
    else if (flight_subtask_cnt[n] == 1) {
        speed_ctrl_mode = 1;
        turn_r(39);//38.5
        steer_control(&turn_ctrl_pwm);
        filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        speed_setup = RC_Data.rc_rpyt[RC_PITCH]; // 速度期望来源于俯仰杆给定
        // 期望速度
        speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale; // 左边轮子速度期望
        speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale; // 右边轮子速度期望
        // 速度控制
        speed_control_100hz(speed_ctrl_mode);
        if (trackless_output.yaw_ctrl_end == 1) {
            flight_subtask_cnt[n] = 2;
            flight_subtask_cnt[1] = 0;
        }
    }
    // 线程2，直线行驶
    else if (flight_subtask_cnt[n] == 2) {
			turn_r(0);
			steer_control(&turn_ctrl_pwm);
			filtered_distance_output = low_pass_filter(distance_ctrl.output, filtered_distance_output, alpha);
			filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
			speed_expect[0] = 45 + filtered_turn_ctrl_pwm * steer_gyro_scale; // 左边轮子速度期望
			speed_expect[1] = 45 - filtered_turn_ctrl_pwm * steer_gyro_scale; // 右边轮子速度期望
			speed_control_100hz(speed_ctrl_mode);
			if(read_gray_bit3==0x01||read_gray_bit4==0x01||read_gray_bit5==0x01||read_gray_bit6==0x01||read_gray_bit2==0x01||read_gray_bit7==0x01||read_gray_bit9==0x01||read_gray_bit10==0x01){
				flight_subtask_cnt[1]=0;
				execute_time_ms[n] = 2920/5;
				buzzer_setup(200, 0.5, 2);//报警
				speed_control_100hz(speed_ctrl_mode);
				flight_subtask_cnt[n] = 3;
			}
    }
    else if (flight_subtask_cnt[n] == 3) {
			char a;
			float b;
			float turn_ctrl_pwm_t=0;
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			turn_jdz(35);
			if(execute_time_ms[n]>2450)steer_control(&turn_ctrl_pwm_t);
			gray_turn_control_200hz(&turn_ctrl_pwm,&a);//基于灰度对管的转向控制
			filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
			speed_expect[0]=45+(turn_ctrl_pwm+turn_ctrl_pwm_t*4)*turn_scale;//左边轮子速度期望
			speed_expect[1]=45-(turn_ctrl_pwm+turn_ctrl_pwm_t*4)*turn_scale;//右边轮子速度期望
			speed_control_100hz(speed_ctrl_mode);//速度控制
			if(execute_time_ms[n]>0) execute_time_ms[n]--;
			else if(execute_time_ms[n]==0){
				flight_subtask_cnt[n] = 4;
				flight_subtask_cnt[1]=0;
				buzzer_setup(200, 0.5, 2);
			}
    }
    else if (flight_subtask_cnt[n] == 4) {
        speed_ctrl_mode = 1;
        turn_jdz(180);
        steer_control(&turn_ctrl_pwm);
        filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        speed_setup = RC_Data.rc_rpyt[RC_PITCH]; // 速度期望来源于俯仰杆给定
        // 期望速度
        speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale; // 左边轮子速度期望
        speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale; // 右边轮子速度期望
        // 速度控制
        speed_control_100hz(speed_ctrl_mode);
        if (trackless_output.yaw_ctrl_end == 1) {
            flight_subtask_cnt[1] = 0;
            flight_subtask_cnt[n] = 5;
        }
    }
    else if (flight_subtask_cnt[n] == 5) {
        speed_ctrl_mode = 1;
        turn_l(38.6);
        steer_control(&turn_ctrl_pwm);
        filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        speed_setup = RC_Data.rc_rpyt[RC_PITCH]; // 速度期望来源于俯仰杆给定
        // 期望速度
        speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale; // 左边轮子速度期望
        speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale; // 右边轮子速度期望
        // 速度控制
        speed_control_100hz(speed_ctrl_mode);
        if (trackless_output.yaw_ctrl_end == 1) {
            flight_subtask_cnt[n] = 6;
            flight_subtask_cnt[1] = 0;
        }
    }
		else if (flight_subtask_cnt[n] == 6) {
			turn_r(0);
			steer_control(&turn_ctrl_pwm);
			filtered_distance_output = low_pass_filter(distance_ctrl.output, filtered_distance_output, alpha);
			filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
			speed_expect[0] = 45 + filtered_turn_ctrl_pwm * steer_gyro_scale; // 左边轮子速度期望
			speed_expect[1] = 45 - filtered_turn_ctrl_pwm * steer_gyro_scale; // 右边轮子速度期望
			speed_control_100hz(speed_ctrl_mode);
			if(read_gray_bit3==0x01||read_gray_bit4==0x01||read_gray_bit5==0x01||read_gray_bit6==0x01||read_gray_bit2==0x01||read_gray_bit7==0x01||read_gray_bit9==0x01||read_gray_bit10==0x01){
				flight_subtask_cnt[1]=0;
				execute_time_ms[n] = 2920/5;
				buzzer_setup(200, 0.5, 2);//报警
				speed_control_100hz(speed_ctrl_mode);
				flight_subtask_cnt[n] = 7;
			}
    }
		else if (flight_subtask_cnt[n] == 7) {
			char a;
			float b;
			float turn_ctrl_pwm_t=0;
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			turn_jdz(145);
			if(execute_time_ms[n]>2450)steer_control(&turn_ctrl_pwm_t);
			gray_turn_control_200hz(&turn_ctrl_pwm,&a);//基于灰度对管的转向控制
			filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
			speed_expect[0]=45+(turn_ctrl_pwm+turn_ctrl_pwm_t*4)*turn_scale;//左边轮子速度期望
			speed_expect[1]=45-(turn_ctrl_pwm+turn_ctrl_pwm_t*4)*turn_scale;//右边轮子速度期望
			speed_control_100hz(speed_ctrl_mode);//速度控制
			if(execute_time_ms[n]>0) execute_time_ms[n]--;
			else if(execute_time_ms[n]==0){
				flight_subtask_cnt[n] = 8;
				flight_subtask_cnt[1]=0;
				buzzer_setup(200, 0.5, 2);
			}
    }
		else if (flight_subtask_cnt[n] == 8) {
        speed_ctrl_mode = 1;
        turn_jdz(0);
        steer_control(&turn_ctrl_pwm);
        filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        speed_setup = RC_Data.rc_rpyt[RC_PITCH]; // 速度期望来源于俯仰杆给定
        // 期望速度
        speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale; // 左边轮子速度期望
        speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale; // 右边轮子速度期望
        // 速度控制
        speed_control_100hz(speed_ctrl_mode);
        if (trackless_output.yaw_ctrl_end == 1) {
            flight_subtask_cnt[1] = 0;
            flight_subtask_cnt[n] = 0;
        }
    }
		
	// 暂停
	else if (flight_subtask_cnt[n] == 12) {
		speed_ctrl_mode = 1;
		speed_expect[0] = 0;
		speed_expect[1] = 0;
		speed_control_100hz(speed_ctrl_mode);
	}
}