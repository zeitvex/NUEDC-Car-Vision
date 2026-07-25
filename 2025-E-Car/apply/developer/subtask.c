#include "headfile.h"
#include "subtask.h"

#define SUBTASK_NUM 20
#define flight_subtask_delta 5//5ms



uint16_t flight_subtask_cnt[SUBTASK_NUM]={0};//飞行任务子线程计数器，可以用于控制每个航点子线程的执行
uint32_t flight_global_cnt[SUBTASK_NUM]={0}; //飞行任务子线全局计数器，可以结合位置偏差用于判断判断航点是否到达
uint32_t execute_time_ms[SUBTASK_NUM]={0};//飞行任务子线执行时间，可以用于设置某个子线程的执行时间


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








void flight_subtask_reset(void)
{
	for(uint16_t i=0;i<SUBTASK_NUM;i++)
	{
		flight_subtask_cnt[i]=0;
		execute_time_ms[i]=0;
		flight_global_cnt[i]=0;
	}
}

void flight_subtask_reset_num(uint16_t num)
{
	flight_subtask_cnt[num]=0;
	execute_time_ms[num]=0;
	flight_global_cnt[num]=0;
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

//以30deg/s的角速度逆时针转动3000ms
void flight_subtask_4(void)
{
	static uint8_t n=3;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE_TURN;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =30;//以30deg/s的角速度顺时针转动3000ms
		trackless_output.execution_time_ms=3000;//执行时间
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE_TURN;
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



//2024年电赛H题-自动行驶小车demo
uint8_t flight_subtask_yaw_angle_ctrl(float target_angle)//绝对航向角度控制,正方形为逆时针方向旋转0~360
{
	uint8_t finish_flag=0;
	static uint8_t n=7;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=AZIMUTH;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =target_angle;//期望角度
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=AZIMUTH;
		trackless_output.yaw_outer_control_output  =0;
		
		if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段	
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
		finish_flag=1;
	}
	else//其它情形
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];		
	}
	return finish_flag;
}




void auto_drive_smartcar_duty1(void)
{
	static uint8_t n=8;	
	if(flight_subtask_cnt[n]==0)
	{
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		distance_ctrl.expect=smartcar_imu.state_estimation.distance+100;
		flight_subtask_cnt[n]=2;
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		
		//距离控制
		distance_control_with_speed_limit(50);
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			
			bling_set(&light_red  ,2000,500,0.5,0,0);//红色
			beep.reset = 1;
			beep.times = 2;
		}	
	}
	else
	{
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);				
	}
}
	 


//#define self_guided_tracking_speed 60//60
//#define distance_ctrl_speed_max 80   //50
//#define move_diagonal_angle    35 //38.65
//#define move_diagonal_distance 125//128

#define speed_zero_check 1.0f
float _distance_ctrl_speed_max,_self_guided_tracking_speed,_move_diagonal_angle1,_move_diagonal_distance1,_move_diagonal_angle2,_move_diagonal_distance2;
void auto_drive_smartcar_duty2(void)
{
	static uint8_t n=9;	
	if(flight_subtask_cnt[n]==0)
	{
		//1、航向控制
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		//2、速度控制-叠加了航向
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		//给定距离期望
		distance_ctrl.expect=smartcar_imu.state_estimation.distance+100;
		//距离控制
		distance_control_with_speed_limit(_distance_ctrl_speed_max);
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		flight_subtask_cnt[n]=2;
	}
	else if(flight_subtask_cnt[n]==2)
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
			
		distance_control_with_speed_limit(_distance_ctrl_speed_max);//距离控制
		speed_setup=distance_ctrl.output;//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		//实时判断距离误差是否满足
		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			
			//执行完毕后,锁定当前偏航角
			steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
						
			bling_set(&light_red  ,2000,500,0.5,0,0);//红色
			beep.reset = 1;
			beep.times = 2;
		}	
	}
	else if(flight_subtask_cnt[n]==3)//将前进速度控制到0，为转向控制做准备
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);

		//2、速度控制-叠加了航向
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
			
		if(ABS(speed_error[0])<speed_zero_check&&ABS(speed_error[1])<speed_zero_check)//速度控制完毕
		{
			flight_subtask_cnt[n]++;
			road_miss_flag=0;
			road_miss_cnt=0;			
		}
	}
	else if(flight_subtask_cnt[n]==4)
	{
		gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
		speed_setup=_self_guided_tracking_speed;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);	

		if(road_miss_flag==1)//灰度传感器丢线，本阶段循迹完毕 
		{
			flight_subtask_cnt[n]++;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			
			//执行完毕后,锁定当前偏航角
			steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
						
			bling_set(&light_red  ,2000,500,0.5,0,0);//红色
			beep.reset = 1;
			beep.times = 2;
		}
	}
	else if(flight_subtask_cnt[n]==5)//将前进速度控制到0，为转向控制做准备
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);

		//2、速度控制-叠加了航向
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
			
		if(ABS(speed_error[0])<speed_zero_check&&ABS(speed_error[1])<speed_zero_check)//速度控制完毕
		{
			flight_subtask_cnt[n]++;	
		}
	}
	else if(flight_subtask_cnt[n]==6)
	{
		if(flight_subtask_yaw_angle_ctrl(180))//判断航向角是否执行完毕 
		{
			flight_subtask_cnt[n]++;//线程计数器自加
			flight_subtask_reset_num(7);//复位
		}
		steer_control(&turn_ctrl_pwm);
		speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);			
	}
	else if(flight_subtask_cnt[n]==7)
	{		
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		//给定距离期望
		distance_ctrl.expect=smartcar_imu.state_estimation.distance+100;
		//距离控制
		distance_control_with_speed_limit(_distance_ctrl_speed_max);
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		flight_subtask_cnt[n]++;
	}
	else if(flight_subtask_cnt[n]==8)
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
			
		distance_control_with_speed_limit(_distance_ctrl_speed_max);//距离控制
		speed_setup=distance_ctrl.output;//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		//实时判断距离误差是否满足
		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			
			//执行完毕后,锁定当前偏航角
			steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
						
			bling_set(&light_red  ,2000,500,0.5,0,0);//红色
			beep.reset = 1;
			beep.times = 2;
			road_miss_flag=0;
			road_miss_cnt=0;
		}	
	}
	else if(flight_subtask_cnt[n]==9)
	{
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
		speed_setup=_self_guided_tracking_speed;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);	

		if(road_miss_flag==1)//灰度传感器丢线，本阶段循迹完毕 
		{
			flight_subtask_cnt[n]++;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			
			//执行完毕后,锁定当前偏航角
			steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
						
			bling_set(&light_red  ,2000,500,0.5,0,0);//红色
			beep.reset = 1;
			beep.times = 2;
		}
	}
	else if(flight_subtask_cnt[n]==10)//将前进速度控制到0
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);

		//2、速度控制-叠加了航向
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
	}
	else
	{
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);				
	}
}
	 

void auto_drive_smartcar_duty3(uint16_t times)
{
	static uint8_t n=10;	
	if(flight_subtask_cnt[n]==0)
	{
		//1、航向控制
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		//2、速度控制-叠加了航向
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		if(flight_subtask_yaw_angle_ctrl(-_move_diagonal_angle1))//判断航向角是否执行完毕 
		{
			flight_subtask_cnt[n]++;//线程计数器自加
			flight_subtask_reset_num(7);//复位
		}
		steer_control(&turn_ctrl_pwm);
		speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);			
	}
	else if(flight_subtask_cnt[n]==2)
	{		
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		//给定距离期望
		distance_ctrl.expect=smartcar_imu.state_estimation.distance+_move_diagonal_distance1;
		//距离控制
		distance_control_with_speed_limit(_distance_ctrl_speed_max);
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		flight_subtask_cnt[n]++;
	}
	else if(flight_subtask_cnt[n]==3)
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
			
		distance_control_with_speed_limit(_distance_ctrl_speed_max);//距离控制
		speed_setup=distance_ctrl.output;//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		//实时判断距离误差是否满足
		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			
			//执行完毕后,锁定当前偏航角
			steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
						
			bling_set(&light_red  ,2000,500,0.5,0,0);//红色
			beep.reset = 1;
			beep.times = 2;
			road_miss_flag=0;
			road_miss_cnt=0;
		}	
	}
	else if(flight_subtask_cnt[n]==4)
	{
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
		speed_setup=_self_guided_tracking_speed;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);	

		if(road_miss_flag==1)//灰度传感器丢线，本阶段循迹完毕 
		{
			flight_subtask_cnt[n]++;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			
			//执行完毕后,锁定当前偏航角
			steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
						
			bling_set(&light_red  ,2000,500,0.5,0,0);//红色
			beep.reset = 1;
			beep.times = 2;
		}
	}
	else if(flight_subtask_cnt[n]==5)//将前进速度控制到0，为转向控制做准备
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);

		//2、速度控制-叠加了航向
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
			
		if(ABS(speed_error[0])<speed_zero_check&&ABS(speed_error[1])<speed_zero_check)//速度控制完毕
		{
			flight_subtask_cnt[n]++;	
		}
	}
	else if(flight_subtask_cnt[n]==6)
	{
		if(flight_subtask_yaw_angle_ctrl(-180+_move_diagonal_angle2))//判断航向角是否执行完毕 
		{
			flight_subtask_cnt[n]++;//线程计数器自加
			flight_subtask_reset_num(7);//复位
		}
		steer_control(&turn_ctrl_pwm);
		speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);			
	}
	else if(flight_subtask_cnt[n]==7)
	{		
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		//给定距离期望
		distance_ctrl.expect=smartcar_imu.state_estimation.distance+_move_diagonal_distance2;
		//距离控制
		distance_control_with_speed_limit(_distance_ctrl_speed_max);
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		flight_subtask_cnt[n]++;
	}
	else if(flight_subtask_cnt[n]==8)
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
			
		distance_control_with_speed_limit(_distance_ctrl_speed_max);//距离控制
		speed_setup=distance_ctrl.output;//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		//实时判断距离误差是否满足
		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			
			//执行完毕后,锁定当前偏航角
			steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
						
			bling_set(&light_red  ,2000,500,0.5,0,0);//红色
			beep.reset = 1;
			beep.times = 2;
			road_miss_flag=0;
			road_miss_cnt=0;
		}	
	}
	else if(flight_subtask_cnt[n]==9)
	{
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
		speed_setup=_self_guided_tracking_speed;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);	

		if(road_miss_flag==1)//灰度传感器丢线，本阶段循迹完毕 
		{
			flight_subtask_cnt[n]++;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			
			//执行完毕后,锁定当前偏航角
			steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
						
			bling_set(&light_red  ,2000,500,0.5,0,0);//红色
			beep.reset = 1;
			beep.times = 2;
		}
	}
	else if(flight_subtask_cnt[n]==10)//将前进速度控制到0，为转向控制做准备
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);

		//2、速度控制-叠加了航向
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
			
		if(ABS(speed_error[0])<speed_zero_check&&ABS(speed_error[1])<speed_zero_check)//速度控制完毕
		{
			flight_subtask_cnt[n]++;	
		}
	}
	else if(flight_subtask_cnt[n]==11)//重复4圈 
	{
			flight_global_cnt[n]++;
			if(flight_global_cnt[n]<times)	 flight_subtask_cnt[n]=1;//重复执行4次
			else flight_subtask_cnt[n]++;
	}
	else
	{
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);		
	}
}




void auto_drive_smartcar_duty4(uint16_t times)
{
	static uint8_t n=10;	
	if(flight_subtask_cnt[n]==0)
	{
		//1、航向控制
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		//2、速度控制-叠加了航向
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		if(flight_subtask_yaw_angle_ctrl(-_move_diagonal_angle1))//判断航向角是否执行完毕 
		{
			flight_subtask_cnt[n]++;//线程计数器自加
			flight_subtask_reset_num(7);//复位
		}
		steer_control(&turn_ctrl_pwm);
		speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);			
	}
	else if(flight_subtask_cnt[n]==2)
	{		
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		//给定距离期望
		distance_ctrl.expect=smartcar_imu.state_estimation.distance+_move_diagonal_distance1;
		//距离控制
		distance_control_with_speed_limit(_distance_ctrl_speed_max);
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		flight_subtask_cnt[n]++;
	}
	else if(flight_subtask_cnt[n]==3)
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
			
		distance_control_with_speed_limit(_distance_ctrl_speed_max);//距离控制
		speed_setup=distance_ctrl.output;//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		//实时判断距离误差是否满足
		if(ABS(distance_ctrl.error)<distance_precision_cm||road_restore_flag==1)
		{
			flight_subtask_cnt[n]++;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			
			//执行完毕后,锁定当前偏航角
			steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
						
			bling_set(&light_red  ,2000,500,0.5,0,0);//红色
			beep.reset = 1;
			beep.times = 2;
			road_miss_flag=0;
			road_miss_cnt=0;
		}	
	}
	else if(flight_subtask_cnt[n]==4)
	{
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
		speed_setup=_self_guided_tracking_speed;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);	

		if(road_miss_flag==1)//灰度传感器丢线，本阶段循迹完毕 
		{
			flight_subtask_cnt[n]++;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			
			//执行完毕后,锁定当前偏航角
			steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
						
			bling_set(&light_red  ,2000,500,0.5,0,0);//红色
			beep.reset = 1;
			beep.times = 2;
		}
	}
	else if(flight_subtask_cnt[n]==5)//将前进速度控制到0，为转向控制做准备
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);

		//2、速度控制-叠加了航向
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
			
		if(ABS(speed_error[0])<speed_zero_check&&ABS(speed_error[1])<speed_zero_check)//速度控制完毕
		{
			flight_subtask_cnt[n]++;	
		}
	}
	else if(flight_subtask_cnt[n]==6)
	{
		if(flight_subtask_yaw_angle_ctrl(-180+_move_diagonal_angle2))//判断航向角是否执行完毕 
		{
			flight_subtask_cnt[n]++;//线程计数器自加
			flight_subtask_reset_num(7);//复位
		}
		steer_control(&turn_ctrl_pwm);
		speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);			
	}
	else if(flight_subtask_cnt[n]==7)
	{		
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		//给定距离期望
		distance_ctrl.expect=smartcar_imu.state_estimation.distance+_move_diagonal_distance2;
		//距离控制
		distance_control_with_speed_limit(_distance_ctrl_speed_max);
		speed_setup=distance_ctrl.output;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		flight_subtask_cnt[n]++;
	}
	else if(flight_subtask_cnt[n]==8)
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
			
		distance_control_with_speed_limit(_distance_ctrl_speed_max);//距离控制
		speed_setup=distance_ctrl.output;//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		speed_control_100hz(speed_ctrl_mode);
		
		//实时判断距离误差是否满足
		if(ABS(distance_ctrl.error)<distance_precision_cm||road_restore_flag==1)
		{
			flight_subtask_cnt[n]++;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			
			//执行完毕后,锁定当前偏航角
			steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
						
			bling_set(&light_red  ,2000,500,0.5,0,0);//红色
			beep.reset = 1;
			beep.times = 2;
			road_miss_flag=0;
			road_miss_cnt=0;
		}	
	}
	else if(flight_subtask_cnt[n]==9)
	{
		speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
		speed_setup=_self_guided_tracking_speed;
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);	

		if(road_miss_flag==1)//灰度传感器丢线，本阶段循迹完毕 
		{
			flight_subtask_cnt[n]++;
			//期望速度
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			
			//执行完毕后,锁定当前偏航角
			steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
						
			bling_set(&light_red  ,2000,500,0.5,0,0);//红色
			beep.reset = 1;
			beep.times = 2;
		}
	}
	else if(flight_subtask_cnt[n]==10)//将前进速度控制到0，为转向控制做准备
	{
		//1、航向控制
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);

		//2、速度控制-叠加了航向
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);
			
		if(ABS(speed_error[0])<speed_zero_check&&ABS(speed_error[1])<speed_zero_check)//速度控制完毕
		{
			flight_subtask_cnt[n]++;	
		}
	}
	else if(flight_subtask_cnt[n]==11)//重复times圈 
	{
			flight_global_cnt[n]++;
			if(flight_global_cnt[n]<times)	 flight_subtask_cnt[n]=1;//重复执行4次
			else flight_subtask_cnt[n]++;
	}
	else
	{
		trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
		steer_control(&turn_ctrl_pwm);
		speed_setup=0;//速度期望给0
		//期望速度
		speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
		speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		//速度控制
		speed_control_100hz(speed_ctrl_mode);		
	}
}


//空地协调智能消防系统

void beep_notify(void)
{
	beep.period=200;//200*5ms
	beep.light_on_percent=0.5f;			
	beep.reset=1;
	beep.times=1;		
}

void firetruck_nav_ctrl(float fixed_threshold_cm,uint16_t feed_times)
{
	speed_ctrl_mode=1;//速度控制方式为两轮单独控制
	position_control(fixed_threshold_cm,feed_times);
	turn_ctrl_pwm=steer_gyro_output;
	speed_setup=distance_ctrl.output;
	//期望速度
	speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
	speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
	//速度控制
	speed_control_100hz(speed_ctrl_mode);
}


/************************************************
	D     E     F
		G2  G3
	A     B     C
U	G	G1
************************************************/
const int16_t nav_point[30][3]=
{
	{0,10,0},  //A'
	{40,50,0}, //O
	{80,90,0}, //C'
	{80,100,1},//C
	{77,115,0},
	{68,128,0},//G
	{55,137,0},
	{40,140,0},//E
	{25,137,0},
	{12,128,0},//H
	{3,115,0},
	{0,100,1}, //B
	{0,90,0},  //B'
	{40,50,0}, //O
	{80,10,0}, //D'
	{80,0,1},  //D
	{77,-15,0},
	{68,-28,0},//I
	{55,-37,0},
	{40,-40,0},//F
	{25,-37,0},
	{12,-28,0},//J
	{3,-15,0},
	{0,0,1}    //A
};


void auto_nav_point(uint16_t times)
{
	static uint8_t n=7;
	static uint16_t _times=0;
	if(flight_subtask_cnt[n]==0)
	{
		firetruck_nav_ctrl(5.0f,5);//基本的导航控制
		//将坐标值赋值给导航相关变量
		ngs_nav_ctrl.update_flag=1;
		ngs_nav_ctrl.x=0;//坐标偏置x
		ngs_nav_ctrl.y=0;//坐标偏置y
		ngs_nav_ctrl.ctrl_finish_flag=0;
		flight_subtask_cnt[n]++;
	}
	else if(flight_subtask_cnt[n]==1)
	{
		firetruck_nav_ctrl(5.0f,1);//基本的导航控制
		if(ngs_nav_ctrl.ctrl_finish_flag==1)//航点遍历完成
		{
			if(nav_point[flight_global_cnt[n]][2]==1)	beep_notify();//判断是否需要蜂鸣器提示	
			if(flight_global_cnt[n]>23)	flight_subtask_cnt[n]++;
			else
			{
				//将火焰更新标志位、坐标值赋值给导航相关变量
				ngs_nav_ctrl.update_flag=1;
				ngs_nav_ctrl.x=nav_point[flight_global_cnt[n]][0];//坐标偏置x
				ngs_nav_ctrl.y=nav_point[flight_global_cnt[n]][1];//坐标偏置y
				ngs_nav_ctrl.ctrl_finish_flag=0;			
				flight_global_cnt[n]++;			
			}
		}
	}
	else
	{
		_times++;
		if(_times<times)
		{
			flight_subtask_cnt[n]=0;
			flight_global_cnt[n]=0;
		}	
		firetruck_nav_ctrl(5.0f,5);//基本的导航控制
	}
}

void renwu1_2025(int repeat_times)
{
    static uint8_t n = 11;
    static int current_repeat = 0;
    static float target_angles[4] = {0, 90, 180, 270};
	if(trackless_output.unlock_flag==LOCK)return;
    if(flight_subtask_cnt[n] == 0) // 初始化
    {
        speed_ctrl_mode = 1;
        trackless_output.yaw_ctrl_mode = ROTATE;
        trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
        steer_control(&turn_ctrl_pwm);
        speed_setup = 0;
        speed_expect[0] = 0;
        speed_expect[1] = 0;
        speed_control_100hz(speed_ctrl_mode);
		
        if(current_repeat == 0)
        {
            distance_ctrl.expect = smartcar_imu.state_estimation.distance + 9; // 第一次前进5cm
            flight_subtask_cnt[n] = 1;
        }
        else
        {
            flight_subtask_cnt[n] = 2; // 跳过5cm直接左转
        }
    }
    else if(flight_subtask_cnt[n] == 1) // 前进5cm（仅第一次）
    {
        trackless_output.yaw_ctrl_mode = ROTATE;
        trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
        steer_control(&turn_ctrl_pwm);

        distance_control_with_speed_limit(30);
        speed_setup = distance_ctrl.output;

        speed_expect[0] = speed_setup + turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = speed_setup - turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);

        if(ABS(distance_ctrl.error) < distance_precision_cm)
        {
            flight_subtask_cnt[n] = 2;
            speed_expect[0] = 0;
            speed_expect[1] = 0;

            bling_set(&light_red, 2000, 500, 0.5, 0, 0);
            beep.reset = 1;
            beep.times = 1;
        }
    }
    else if(flight_subtask_cnt[n] == 2) // 第一次左转90°
    {
        if(flight_subtask_yaw_angle_ctrl(target_angles[1]))
        {
            flight_subtask_cnt[n] = 3;
            flight_subtask_reset_num(7);
            distance_ctrl.expect = smartcar_imu.state_estimation.distance + 95;
        }

        steer_control(&turn_ctrl_pwm);
        speed_expect[0] = turn_ctrl_pwm * steer_gyro_scale*0.8;
        speed_expect[1] = -turn_ctrl_pwm * steer_gyro_scale*0.8;
        speed_control_100hz(speed_ctrl_mode);
    }
    else if(flight_subtask_cnt[n] == 3) // 第一段直行
    {
        trackless_output.yaw_ctrl_mode = ROTATE;
        trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
        steer_control(&turn_ctrl_pwm);

        distance_control_with_speed_limit(50);
        speed_setup = distance_ctrl.output;

        speed_expect[0] = speed_setup + turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = speed_setup - turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);

        if(ABS(distance_ctrl.error) < distance_precision_cm)
        {
            flight_subtask_cnt[n] = 4;
            speed_expect[0] = 0;
            speed_expect[1] = 0;

            bling_set(&light_red, 2000, 500, 0.5, 0, 0);
            beep.reset = 1;
            beep.times = 1;
        }
    }
    else if(flight_subtask_cnt[n] == 4) // 第二次左转
    {
        if(flight_subtask_yaw_angle_ctrl(target_angles[2]))
        {
            flight_subtask_cnt[n] = 5;
            flight_subtask_reset_num(7);
            distance_ctrl.expect = smartcar_imu.state_estimation.distance + 93;
        }

        steer_control(&turn_ctrl_pwm);
        speed_expect[0] = turn_ctrl_pwm * steer_gyro_scale*0.8;
        speed_expect[1] = -turn_ctrl_pwm * steer_gyro_scale*0.8;
        speed_control_100hz(speed_ctrl_mode);
    }
    else if(flight_subtask_cnt[n] == 5) // 第二段直行
    {
        trackless_output.yaw_ctrl_mode = ROTATE;
        trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
        steer_control(&turn_ctrl_pwm);

        distance_control_with_speed_limit(50);
        speed_setup = distance_ctrl.output;

        speed_expect[0] = speed_setup + turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = speed_setup - turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);

        if(ABS(distance_ctrl.error) < distance_precision_cm)
        {
            flight_subtask_cnt[n] = 6;
            speed_expect[0] = 0;
            speed_expect[1] = 0;

            bling_set(&light_red, 2000, 500, 0.5, 0, 0);
            beep.reset = 1;
            beep.times = 1;
        }
    }
    else if(flight_subtask_cnt[n] == 6) // 第三次左转
    {
        if(flight_subtask_yaw_angle_ctrl(target_angles[3]))
        {
            flight_subtask_cnt[n] = 7;
            flight_subtask_reset_num(7);
            distance_ctrl.expect = smartcar_imu.state_estimation.distance + 93;
        }

        steer_control(&turn_ctrl_pwm);
        speed_expect[0] = turn_ctrl_pwm * steer_gyro_scale*0.8;
        speed_expect[1] = -turn_ctrl_pwm * steer_gyro_scale*0.8;
        speed_control_100hz(speed_ctrl_mode);
    }
    else if(flight_subtask_cnt[n] == 7) // 第三段直行
    {
        trackless_output.yaw_ctrl_mode = ROTATE;
        trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
        steer_control(&turn_ctrl_pwm);

        distance_control_with_speed_limit(50);
        speed_setup = distance_ctrl.output;

        speed_expect[0] = speed_setup + turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = speed_setup - turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);

        if(ABS(distance_ctrl.error) < distance_precision_cm)
        {
            flight_subtask_cnt[n] = 8;
            speed_expect[0] = 0;
            speed_expect[1] = 0;

            bling_set(&light_red, 2000, 500, 0.5, 0, 0);
            beep.reset = 1;
            beep.times = 1;
        }
    }
    else if(flight_subtask_cnt[n] == 8) // 第四次左转
    {
        if(flight_subtask_yaw_angle_ctrl(target_angles[0]))
        {
            distance_ctrl.expect = smartcar_imu.state_estimation.distance + 93;
            flight_subtask_cnt[n] = 9;
            flight_subtask_reset_num(7);
        }

        steer_control(&turn_ctrl_pwm);
        speed_expect[0] = turn_ctrl_pwm * steer_gyro_scale*0.8;
        speed_expect[1] = -turn_ctrl_pwm * steer_gyro_scale*0.8;
        speed_control_100hz(speed_ctrl_mode);
    }
    else if(flight_subtask_cnt[n] == 9) // 第四段直行（回正后）
    {
        trackless_output.yaw_ctrl_mode = ROTATE;
        trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
        steer_control(&turn_ctrl_pwm);

        distance_control_with_speed_limit(45);
        speed_setup = distance_ctrl.output;

        speed_expect[0] = speed_setup + turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = speed_setup - turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);

        if(ABS(distance_ctrl.error) < distance_precision_cm)
        {
            flight_subtask_cnt[n] = 10;
            speed_expect[0] = 0;
            speed_expect[1] = 0;

            bling_set(&light_red, 2000, 500, 0.5, 0, 0);
            beep.reset = 1;
            beep.times = 2;
        }
    }
    else // cnt == 10：任务结束（重复控制）
    {
        if(current_repeat < repeat_times - 1)
        {
            current_repeat++;
            flight_subtask_cnt[n] = 0;
            flight_subtask_reset_num(7);
        }

        trackless_output.yaw_ctrl_mode = ROTATE;
        trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
        steer_control(&turn_ctrl_pwm);

        speed_setup = 0;
        speed_expect[0] = turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = -turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);
    }
}
float low_pass_filter(float input, float prev_output, float alpha) {
    return alpha * input + (1 - alpha) * prev_output;
}
void arc_straight_stop(char fl)
{
	const float alpha = 0.05; // 低通滤波器系数
    static uint8_t n = 12; // 为新任务分配一个唯一的子任务计数器ID
    static float target_angle_90 = 90; // 目标旋转角度：90度
    static float distance_50cm = 75;   // 目标直行距离：50厘米
	static float last_dis=0;
	static float filtered_turn_ctrl_pwm = 0;
    if(flight_subtask_cnt[n] == 0) // 初始化阶段
    {
        // 设置速度控制模式
        speed_ctrl_mode = 1;
        // 设置航向控制模式为旋转，并使用RC_ROLL作为外部输入（与原函数保持一致）
        trackless_output.yaw_ctrl_mode = ROTATE;
        trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
        steer_control(&turn_ctrl_pwm); // 初始化转向控制

        // 确保车辆停止
        speed_setup = 0;
        speed_expect[0] = 0;
        speed_expect[1] = 0;
        speed_control_100hz(speed_ctrl_mode);

        // 重置子任务状态，准备开始第一个动作
        flight_subtask_reset_num(7); // 这里的7可能是一个通用的复位编号，沿用原代码
        flight_subtask_cnt[n] = 1;   // 进入旋转阶段
		last_dis=smartcar_imu.state_estimation.distance;//记录进入任务之前的距离
    }
    else if(flight_subtask_cnt[n] == 1) // 旋转90度
    {
		flight_subtask_yaw_angle_ctrl(90.0/25*(smartcar_imu.state_estimation.distance-last_dis)+90);
		flight_subtask_reset_num(7);
        steer_control(&turn_ctrl_pwm); // 计算转向PWM
        // 应用差速驱动实现原地旋转
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        speed_expect[0] = 15+turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = 15-turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);
        // 调用角度控制函数，当达到目标角度时返回true
        if((smartcar_imu.state_estimation.distance-last_dis)>=25)
        {
            // 旋转完成，进入直行阶段
            flight_subtask_cnt[n] = 2;
            flight_subtask_reset_num(7); // 重置子任务状态
            distance_ctrl.expect = smartcar_imu.state_estimation.distance + distance_50cm;
            // 旋转完成提示
            bling_set(&light_red, 2000, 500, 0.5, 0, 0);
            //beep.reset = 1;
            //beep.times = 1;
        }
    }
    else if(flight_subtask_cnt[n] == 2) // 直行50厘米
    {
		flight_subtask_yaw_angle_ctrl(180);
		flight_subtask_reset_num(7);
        // 保持航向控制，确保直行
        trackless_output.yaw_ctrl_mode = ROTATE;
        trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
        steer_control(&turn_ctrl_pwm); // 继续计算转向PWM以进行航向修正
        // 基于距离误差控制前进速度，速度上限为30
        distance_control_with_speed_limit(35);
        speed_setup = distance_ctrl.output;
        // 应用前进速度和转向补偿到两轮
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);

        // 检查是否达到目标距离
        if(ABS(distance_ctrl.error) < distance_precision_cm)
        {
            flight_subtask_cnt[n] = 3;
			last_dis=smartcar_imu.state_estimation.distance;//记录进入任务之前的距离
            // 直行完成提示
			flight_subtask_reset_num(7);
            bling_set(&light_red, 2000, 500, 0.5, 0, 0);
            //beep.reset = 1;
            beep.times = 2; // 用不同的蜂鸣次数表示任务完成
        }
    }
    else if(flight_subtask_cnt[n] == 3) // 停止（任务最终状态）
    {
        flight_subtask_yaw_angle_ctrl(90.0/25*(smartcar_imu.state_estimation.distance-last_dis)+180);
		flight_subtask_reset_num(7);
        steer_control(&turn_ctrl_pwm); // 计算转向PWM
        // 应用差速驱动实现原地旋转
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        speed_expect[0] = 15+turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = 15-turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);
        // 调用角度控制函数，当达到目标角度时返回true
        if((smartcar_imu.state_estimation.distance-last_dis)>=25)
        {
            flight_subtask_cnt[n] = 4;
            flight_subtask_reset_num(7); // 重置子任务状态
            distance_ctrl.expect = smartcar_imu.state_estimation.distance + distance_50cm;
            // 旋转完成提示
            bling_set(&light_red, 2000, 500, 0.5, 0, 0);
            //beep.reset = 1;
            //beep.times = 1;
						
        }
    }
	else if(flight_subtask_cnt[n] == 4) // 直行50厘米
    {
		flight_subtask_yaw_angle_ctrl(270);
		flight_subtask_reset_num(7);
        // 保持航向控制，确保直行
        trackless_output.yaw_ctrl_mode = ROTATE;
        trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
        steer_control(&turn_ctrl_pwm); // 继续计算转向PWM以进行航向修正
        // 基于距离误差控制前进速度，速度上限为30
        distance_control_with_speed_limit(30);
        speed_setup = distance_ctrl.output;
        // 应用前进速度和转向补偿到两轮
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);

        // 检查是否达到目标距离
        if(ABS(distance_ctrl.error) < distance_precision_cm)
        {
			flight_subtask_cnt[n]++;
			flight_subtask_reset_num(7);
			last_dis=smartcar_imu.state_estimation.distance;//记录进入任务之前的距离
            // 直行完成提示
            bling_set(&light_red, 2000, 500, 0.5, 0, 0);
            //beep.reset = 1;
            //beep.times = 2; // 用不同的蜂鸣次数表示任务完成
        }
    }
	else if(flight_subtask_cnt[n] == 5) // 旋转90度
    {
		flight_subtask_yaw_angle_ctrl(90.0/25*(smartcar_imu.state_estimation.distance-last_dis)+270);
		flight_subtask_reset_num(7);
        steer_control(&turn_ctrl_pwm); // 计算转向PWM
        // 应用差速驱动实现原地旋转
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        speed_expect[0] = 20+turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = 20-turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);
        // 调用角度控制函数，当达到目标角度时返回true
        if((smartcar_imu.state_estimation.distance-last_dis)>=25)
        {
            // 旋转完成，进入直行阶段
            flight_subtask_cnt[n] ++;
            flight_subtask_reset_num(7); // 重置子任务状态
            distance_ctrl.expect = smartcar_imu.state_estimation.distance + distance_50cm;
            // 旋转完成提示
            bling_set(&light_red, 2000, 500, 0.5, 0, 0);
            //beep.reset = 1;
            //beep.times = 1;
        }
    }
	else if(flight_subtask_cnt[n] == 6) // 直行50厘米
    {
		flight_subtask_yaw_angle_ctrl(360);
		flight_subtask_reset_num(7);
        // 保持航向控制，确保直行
        trackless_output.yaw_ctrl_mode = ROTATE;
        trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
        steer_control(&turn_ctrl_pwm); // 继续计算转向PWM以进行航向修正
        // 基于距离误差控制前进速度，速度上限为30
        distance_control_with_speed_limit(20);
        speed_setup = distance_ctrl.output;
        // 应用前进速度和转向补偿到两轮
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);

        // 检查是否达到目标距离
        if(ABS(distance_ctrl.error) < distance_precision_cm)
        {
			flight_subtask_cnt[n]++;
			flight_subtask_reset_num(7);
			last_dis=smartcar_imu.state_estimation.distance;//记录进入任务之前的距离
            bling_set(&light_red, 2000, 500, 0.5, 0, 0);
            //beep.reset = 1;
            //beep.times = 2; // 用不同的蜂鸣次数表示任务完成
        }
    }
	else if(flight_subtask_cnt[n] == 7) // 停止（任务最终状态）
    {
        flight_subtask_yaw_angle_ctrl(90.0/25*(smartcar_imu.state_estimation.distance-last_dis)+360);
		flight_subtask_reset_num(7);
        steer_control(&turn_ctrl_pwm); // 计算转向PWM
        // 应用差速驱动实现原地旋转
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        speed_expect[0] = 10+turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = 10-turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);
        // 调用角度控制函数，当达到目标角度时返回true
        if((smartcar_imu.state_estimation.distance-last_dis)>=25)
        {
			turn_ctrl_pwm=0;
            // 旋转完成，进入直行阶段
            flight_subtask_cnt[n]++;
            flight_subtask_reset_num(7); // 重置子任务状态
            distance_ctrl.expect = smartcar_imu.state_estimation.distance + distance_50cm;
            // 旋转完成提示
            bling_set(&light_red, 2000, 500, 0.5, 0, 0);
            //beep.reset = 1;
            //beep.times = 1;
						
        }
    }
	else if(flight_subtask_cnt[n] == 8) // 直行50厘米
    {
		flight_subtask_yaw_angle_ctrl(450);
		flight_subtask_reset_num(7);
        // 保持航向控制，确保直行
        trackless_output.yaw_ctrl_mode = ROTATE;
        trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
        steer_control(&turn_ctrl_pwm); // 继续计算转向PWM以进行航向修正
        // 基于距离误差控制前进速度，速度上限为30
        distance_control_with_speed_limit(30);
        speed_setup = distance_ctrl.output;
        // 应用前进速度和转向补偿到两轮
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
        speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale;
        speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale;
        speed_control_100hz(speed_ctrl_mode);

        // 检查是否达到目标距离
        if(ABS(distance_ctrl.error) < distance_precision_cm)
        {
			
			if(fl==4)flight_subtask_cnt[n]=17;
			else if(fl==5)flight_subtask_cnt[n]++;
			speed_setup = 0;
			speed_expect[0] = 0;
			speed_expect[1] = 0;
			speed_control_100hz(speed_ctrl_mode);
			flight_subtask_reset_num(7);
			last_dis=smartcar_imu.state_estimation.distance;//记录进入任务之前的距离
            bling_set(&light_red, 2000, 500, 0.5, 0, 0);
            //beep.reset = 1;
            //beep.times = 2; // 用不同的蜂鸣次数表示任务完成
        }
    }
	else if(flight_subtask_cnt[n] == 9) // 旋转90度
	{
		flight_subtask_yaw_angle_ctrl(90.0/25*(smartcar_imu.state_estimation.distance-last_dis)+450);
		flight_subtask_reset_num(7);
		steer_control(&turn_ctrl_pwm);
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
		speed_expect[0] = 10 + turn_ctrl_pwm * steer_gyro_scale;
		speed_expect[1] = 10 - turn_ctrl_pwm * steer_gyro_scale;
		speed_control_100hz(speed_ctrl_mode);

		if((smartcar_imu.state_estimation.distance - last_dis) >= 25)
		{
			flight_subtask_cnt[n]++;
			flight_subtask_reset_num(7);
			distance_ctrl.expect = smartcar_imu.state_estimation.distance + distance_50cm;
			bling_set(&light_red, 2000, 500, 0.5, 0, 0);
		}
	}
	else if(flight_subtask_cnt[n] == 10) // 直行50厘米
	{
		flight_subtask_yaw_angle_ctrl(540);
		flight_subtask_reset_num(7);
		trackless_output.yaw_ctrl_mode = ROTATE;
		trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
		steer_control(&turn_ctrl_pwm);
		distance_control_with_speed_limit(30);
		speed_setup = distance_ctrl.output;
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
		speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale;
		speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale;
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error) < distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			flight_subtask_reset_num(7);
			last_dis = smartcar_imu.state_estimation.distance;
			bling_set(&light_red, 2000, 500, 0.5, 0, 0);
		}
	}
	else if(flight_subtask_cnt[n] == 11) // 旋转90度
	{
		flight_subtask_yaw_angle_ctrl(90.0/25*(smartcar_imu.state_estimation.distance-last_dis)+540);
		flight_subtask_reset_num(7);
		steer_control(&turn_ctrl_pwm);
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
		speed_expect[0] = 15 + turn_ctrl_pwm * steer_gyro_scale;
		speed_expect[1] = 15 - turn_ctrl_pwm * steer_gyro_scale;
		speed_control_100hz(speed_ctrl_mode);

		if((smartcar_imu.state_estimation.distance - last_dis) >= 25)
		{
			flight_subtask_cnt[n]++;
			flight_subtask_reset_num(7);
			distance_ctrl.expect = smartcar_imu.state_estimation.distance + distance_50cm;
			bling_set(&light_red, 2000, 500, 0.5, 0, 0);
		}
	}
	else if(flight_subtask_cnt[n] == 12) // 直行50厘米
	{
		flight_subtask_yaw_angle_ctrl(630);
		flight_subtask_reset_num(7);
		trackless_output.yaw_ctrl_mode = ROTATE;
		trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
		steer_control(&turn_ctrl_pwm);
		distance_control_with_speed_limit(30);
		speed_setup = distance_ctrl.output;
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
		speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale;
		speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale;
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error) < distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			flight_subtask_reset_num(7);
			last_dis = smartcar_imu.state_estimation.distance;
			bling_set(&light_red, 2000, 500, 0.5, 0, 0);
		}
	}
	else if(flight_subtask_cnt[n] == 13) // 旋转90度
	{
		flight_subtask_yaw_angle_ctrl(90.0/25*(smartcar_imu.state_estimation.distance-last_dis)+630);
		flight_subtask_reset_num(7);
		steer_control(&turn_ctrl_pwm);
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
		speed_expect[0] = 15 + turn_ctrl_pwm * steer_gyro_scale;
		speed_expect[1] = 15 - turn_ctrl_pwm * steer_gyro_scale;
		speed_control_100hz(speed_ctrl_mode);

		if((smartcar_imu.state_estimation.distance - last_dis) >= 25)
		{
			flight_subtask_cnt[n]++;
			flight_subtask_reset_num(7);
			distance_ctrl.expect = smartcar_imu.state_estimation.distance + distance_50cm;
			bling_set(&light_red, 2000, 500, 0.5, 0, 0);
		}
	}
	else if(flight_subtask_cnt[n] == 14) // 直行50厘米
	{
		flight_subtask_yaw_angle_ctrl(720);
		flight_subtask_reset_num(7);
		trackless_output.yaw_ctrl_mode = ROTATE;
		trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
		steer_control(&turn_ctrl_pwm);
		distance_control_with_speed_limit(30);
		speed_setup = distance_ctrl.output;
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
		speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale;
		speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale;
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error) < distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			flight_subtask_reset_num(7);
			last_dis = smartcar_imu.state_estimation.distance;
			bling_set(&light_red, 2000, 500, 0.5, 0, 0);
		}
	}
	else if(flight_subtask_cnt[n] == 15) // 最后旋转90度到810°
	{
		flight_subtask_yaw_angle_ctrl(90.0/25*(smartcar_imu.state_estimation.distance-last_dis)+720);
		flight_subtask_reset_num(7);
		steer_control(&turn_ctrl_pwm);
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
		speed_expect[0] = 15 + turn_ctrl_pwm * steer_gyro_scale;
		speed_expect[1] = 15 - turn_ctrl_pwm * steer_gyro_scale;
		speed_control_100hz(speed_ctrl_mode);

		if((smartcar_imu.state_estimation.distance - last_dis) >= 25)
		{
			flight_subtask_cnt[n]++; // 到16，完成
			flight_subtask_reset_num(7);
			speed_expect[0] = 0;
			speed_expect[1] = 0;
			speed_control_100hz(speed_ctrl_mode);
			bling_set(&light_red, 2000, 500, 0.5, 0, 0);
			distance_ctrl.expect = smartcar_imu.state_estimation.distance + 65;
			//beep.times = 3; // 可选提示
		}
	}
	else if(flight_subtask_cnt[n] == 16) // 直行50厘米
	{
		flight_subtask_yaw_angle_ctrl(810);
		flight_subtask_reset_num(7);
		trackless_output.yaw_ctrl_mode = ROTATE;
		trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
		steer_control(&turn_ctrl_pwm);
		distance_control_with_speed_limit(30);
		speed_setup = distance_ctrl.output;
		filtered_turn_ctrl_pwm = low_pass_filter(turn_ctrl_pwm, filtered_turn_ctrl_pwm, alpha);
		speed_expect[0] = speed_setup + filtered_turn_ctrl_pwm * steer_gyro_scale;
		speed_expect[1] = speed_setup - filtered_turn_ctrl_pwm * steer_gyro_scale;
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error) < distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			flight_subtask_reset_num(7);
			last_dis = smartcar_imu.state_estimation.distance;
			bling_set(&light_red, 2000, 500, 0.5, 0, 0);
		}
	}
	else if(flight_subtask_cnt[n] == 17) // 停止
	{
		trackless_output.yaw_ctrl_mode = ROTATE;
		trackless_output.yaw_outer_control_output = RC_Data.rc_rpyt[RC_ROLL];
		speed_setup = 0;
		speed_expect[0] = 0;
		speed_expect[1] = 0;
		speed_control_100hz(speed_ctrl_mode);
	}

}



// --- 传感器数据滤波的静态变量 ---
static float filtered_car_yaw_angular_velocity = 0.0f;
static float filtered_v_left = 0.0f;
static float filtered_v_right = 0.0f;

// 滤波系数 (0.0 到 1.0之间)，越小滤波越强，但延迟越大
const float ALPHA_GYRO = 0.3f;      // 陀螺仪角速度滤波系数
const float ALPHA_WHEEL_SPEED = 0.3f; // 轮速滤波系数

// --- 输出死区 (Deadband) 阈值 ---
// 期望速度调整量小于此值时，设为0 (例如，0.05 度/控制周期)
const float DEADZONE_SPEED_THRESHOLD = 0.05f; 
// 像素误差小于此值时，也允许速度调整量被设为0 (例如，3.0 像素)
const float DEADZONE_ERROR_THRESHOLD = 3.0f;

// --- 新增：小车停止判断阈值 ---
// 如果小车陀螺仪角速度低于此阈值，则认为小车在转动方面已停止 (单位：度/秒)
const float ANGULAR_VELOCITY_STOP_THRESHOLD = 0.8f; 
// 如果小车平均线速度低于此阈值，则认为小车在移动方面已停止 (单位：cm/s)
const float LINEAR_VELOCITY_STOP_THRESHOLD = 3.0f; 


void tracking_task() {
    static Gimbal gimbal = {0, 0};
    // 你的PID只有P项，这里D和I项的参数也为0
    // 尝试适当增大Kd，有助于抑制超调，但需注意噪声
    static PIDController pid = {0.002, 0, 0.08, 0, 0, 0, 0}; 
    // 状态控制变量
    static int task_step = 0;
    static int counter = 0;  // 多用途计数器，替代多个单一用途计数器
    static int last_error_x = 0;
    static int target_lost_count = 0;
	static int target_find_count = 0;
    static int direction = -1;  // -1: 向左移动, 1: 向右移动
    static float angle_increment = 45.0f / 200.0f;  // 每步0.225度
    static float max_angle = 45.0f;

    float extended_yaw = unwrap_yaw(smartcar_imu.rpy_deg[2]);
    float speed_x = 0, speed_y = 0;

    if (task_step == 0) {
        // 第一步：仅使用陀螺仪
        speed_x = 0;
        speed_y = 0;
        counter++;
        if (counter > 30) {
            task_step = 1;  // 进入扫描模式
            counter = 0;
            direction = -1;
            gimbal.angle_x = 0.0f;
            gimbal.angle_y = 0.0f;
        }
    } else if (task_step == 1) {
        // 扫描模式：左右移动寻找目标
        float error_x = get_maixcam_x() - 224 + get_goal_x()+27;
        float error_y = get_maixcam_y() - 224 + get_goal_y()-17;
        // 左右移动控制
		gimbal.angle_y =0;
        gimbal.angle_x += direction * angle_increment;
        // 检查是否到达边界
        if (gimbal.angle_x <= -max_angle) {
            gimbal.angle_x = -max_angle;
            direction = 1;  // 改变方向向右
        } else if (gimbal.angle_x >= max_angle) {
            gimbal.angle_x = max_angle;
            direction = -1;  // 改变方向向左
        }

        // 检测是否找到目标
        if((get_maixcam_x()-224)!=666){
						//目标找到，检查是否可以进入跟踪模式
						target_find_count++;
						if (target_find_count >= 5){
							task_step = 2;  // 进入跟踪模式
							target_find_count = 0;
					}
				}

        // 舵机控制
        uint16_t servo_pos_x = angle_to_encoder(extended_yaw + gimbal.angle_x);
        uint16_t servo_pos_y = angle_to_encoder(gimbal.angle_y);
        set_servo(servo_pos_x, servo_pos_y);

    } else if (task_step == 2) {
        // 跟踪模式：摄像头控制 + 激光判断
        float error_x = get_maixcam_x() - 224 + get_goal_x()+27;
        float error_y = get_maixcam_y() - 224 + get_goal_y()-17;
        // 检测目标丢失
        if ((get_maixcam_x()-224)==666) {
				target_lost_count++;
				if (target_lost_count >= 20) {
						// 目标丢失，回到扫描模式
						task_step = 1;
						direction = -1;
						target_lost_count = 0;
				}
		} else {
            target_lost_count = 0;
            // 计算PID输出
            compute_pid(&pid, error_x, error_y, &speed_x, &speed_y);

            // --- 传感器数据滤波 ---
            // 陀螺仪角速度滤波
            filtered_car_yaw_angular_velocity = ALPHA_GYRO * smartcar_imu.rpy_gyro_dps[2] + (1.0f - ALPHA_GYRO) * filtered_car_yaw_angular_velocity;

            // 轮速滤波 (假设 cmps 即为 cm/s，如果需要转换单位，在这里进行)
            filtered_v_left = ALPHA_WHEEL_SPEED * smartcar_imu.left_motor_speed_cmps + (1.0f - ALPHA_WHEEL_SPEED) * filtered_v_left;
            filtered_v_right = ALPHA_WHEEL_SPEED * smartcar_imu.right_motor_speed_cmps + (1.0f - ALPHA_WHEEL_SPEED) * filtered_v_right;

            float compensation_speed_x = 0.0f; // 初始化补偿量为0

            // 计算小车平均线速度的绝对值，用于判断是否静止
            float avg_linear_speed_abs = (fabs(smartcar_imu.left_motor_speed_cmps) + fabs(smartcar_imu.right_motor_speed_cmps)) / 2.0f;

            // --- 核心改动：基于实际速度判断小车是否停止，并消除前馈反冲 ---
            // 如果小车原始陀螺仪角速度和平均线速度都非常小，认为小车已停止或即将停止
            if (fabs(smartcar_imu.rpy_gyro_dps[2]) < ANGULAR_VELOCITY_STOP_THRESHOLD && 
                avg_linear_speed_abs < LINEAR_VELOCITY_STOP_THRESHOLD) {
                
                // 小车停止时，强制将所有相关的前馈补偿量设为0
                // 同时，将滤波后的历史数据也强制清零，避免残余值导致的反冲
                compensation_speed_x = 0.0f;
                filtered_car_yaw_angular_velocity = 0.0f; // 强制清零滤波后的角速度
                filtered_v_left = 0.0f; // 强制清零滤波后的左轮速
                filtered_v_right = 0.0f; // 强制清零滤波后的右轮速
                
                // 此时，speed_x 将只由PID控制器的输出决定，没有前馈补偿
            } else {
                // 小车正在运动：正常应用前馈补偿

                // 1. 小车自身旋转（陀螺仪yaw角速度）补偿
                float car_yaw_angular_velocity_deg_s = filtered_car_yaw_angular_velocity;
                float K_yaw_rate_feedforward = -0.00025f; // 你的系数
                compensation_speed_x -= K_yaw_rate_feedforward * car_yaw_angular_velocity_deg_s;

                // 2. 小车线速度引起的视场漂移补偿
                // 即使你的 K_linear_speed_comp 为0，此处逻辑也保留
                float car_linear_velocity = (filtered_v_left + filtered_v_right) / 2.0f; // 小车前进线速度 (cm/s)
                float gimbal_angle_x_rad = gimbal.angle_x * (3.1415 / 180.0f); // 使用 M_PI
                float K_linear_speed_comp = 0; // 你的系数
                compensation_speed_x += K_linear_speed_comp * car_linear_velocity * sin(gimbal_angle_x_rad);
            }
            
            // 将补偿项加入到PID输出中
            speed_x += compensation_speed_x;

            // --- 输出死区 (Deadband) ---
            // 只有当期望的调整速度大于阈值，或者像素误差仍然较大时，才进行实际调整
            if (fabs(speed_x) < DEADZONE_SPEED_THRESHOLD && fabs(error_x) < DEADZONE_ERROR_THRESHOLD) {
                speed_x = 0; // 如果期望调整速度很小且误差在死区内，则不调整X轴
            }
            // 对Y轴也可以应用类似的死区
            if (fabs(speed_y) < DEADZONE_SPEED_THRESHOLD && fabs(error_y) < DEADZONE_ERROR_THRESHOLD) {
                speed_y = 0; // 如果期望调整速度很小且误差在死区内，则不调整Y轴
            }

            // 调整云台角度
            adjust_angle(&gimbal, speed_x, speed_y);

            if (fabs(error_x) < 3 && fabs(error_y) < 3) {
                counter++;
                if (counter > 10) {
                    //laser_open();
                    // laser_on();
                }
            } else {
                counter = 0;
                //laser_close();
            }
        }

        uint16_t servo_pos_x = angle_to_encoder(extended_yaw + gimbal.angle_x);
        uint16_t servo_pos_y = angle_to_encoder(gimbal.angle_y);
        set_servo(servo_pos_x, servo_pos_y);
    }
}

void renwu4_2025(char aaa) {
	if(trackless_output.unlock_flag==LOCK)return;
    static Gimbal gimbal = {0, 0};
    // 你的PID只有P项，这里D和I项的参数也为0
    // 尝试适当增大Kd，有助于抑制超调，但需注意噪声
    static PIDController pid = {0.0027, 0.0001, 0.06, 0, 0, 0, 0}; 
    // 状态控制变量
	static char run_flag=0;
    static int task_step = 0;
    static int counter = 0;  // 多用途计数器，替代多个单一用途计数器
    static int last_error_x = 0;
    static int target_lost_count = 0;
	static int target_find_count = 0;
    static int direction = 1;  // -1: 向左移动, 1: 向右移动
    static float angle_increment = 45.0f / 250.0f;  // 每步0.225度
    static float max_angle = 45.0f;

    float extended_yaw = unwrap_yaw(smartcar_imu.rpy_deg[2]);
    float speed_x = 0, speed_y = 0;

    if (task_step == 0) {
        // 第一步：仅使用陀螺仪
        speed_x = 0;
        speed_y = 0;
        counter++;
        if (counter > 30) {
            task_step = 1;  // 进入扫描模式
            counter = 0;
            direction = -1;
            gimbal.angle_x = 0.0f;
            gimbal.angle_y = 0.0f;
        }
    } else if (task_step == 1) {
        // 扫描模式：左右移动寻找目标
        float error_x = get_maixcam_x() - 224 + get_goal_x()+15;
        float error_y = get_maixcam_y() - 224 + get_goal_y()-28;
        // 左右移动控制
		gimbal.angle_y =0;
        gimbal.angle_x += direction * angle_increment;
        // 检查是否到达边界
        if (gimbal.angle_x <= -max_angle) {
            gimbal.angle_x = -max_angle;
            direction = 1;  // 改变方向向右
        } else if (gimbal.angle_x >= max_angle) {
            gimbal.angle_x = max_angle;
            direction = -1;  // 改变方向向左
        }

        // 检测是否找到目标
        if((get_maixcam_x()-224)!=666){
						//目标找到，检查是否可以进入跟踪模式
						target_find_count++;
						if (target_find_count >= 8){
							task_step = 2;  // 进入跟踪模式
							target_find_count = 0;
					}
				}

        // 舵机控制
        uint16_t servo_pos_x = angle_to_encoder(extended_yaw + gimbal.angle_x);
        uint16_t servo_pos_y = angle_to_encoder(gimbal.angle_y);
        set_servo(servo_pos_x, servo_pos_y);

    } else if (task_step == 2) {
        // 跟踪模式：摄像头控制 + 激光判断
        float error_x = get_maixcam_x() - 224 + get_goal_x()+15;
        float error_y = get_maixcam_y() - 224 + get_goal_y()-28;

        // 检测目标丢失
        if ((get_maixcam_x()-224)==666) {
				target_lost_count++;
				if (target_lost_count >= 25) {
						// 目标丢失，回到扫描模式
						task_step = 1;
						direction = -1;
						target_lost_count = 0;
				}
		} else {
            target_lost_count = 0;
            // 计算PID输出
            compute_pid(&pid, error_x, error_y, &speed_x, &speed_y);

            // --- 传感器数据滤波 ---
            // 陀螺仪角速度滤波
            filtered_car_yaw_angular_velocity = ALPHA_GYRO * smartcar_imu.rpy_gyro_dps[2] + (1.0f - ALPHA_GYRO) * filtered_car_yaw_angular_velocity;

            // 轮速滤波 (假设 cmps 即为 cm/s，如果需要转换单位，在这里进行)
            filtered_v_left = ALPHA_WHEEL_SPEED * smartcar_imu.left_motor_speed_cmps + (1.0f - ALPHA_WHEEL_SPEED) * filtered_v_left;
            filtered_v_right = ALPHA_WHEEL_SPEED * smartcar_imu.right_motor_speed_cmps + (1.0f - ALPHA_WHEEL_SPEED) * filtered_v_right;

            float compensation_speed_x = 0.0f; // 初始化补偿量为0

            // 计算小车平均线速度的绝对值，用于判断是否静止
            float avg_linear_speed_abs = (fabs(smartcar_imu.left_motor_speed_cmps) + fabs(smartcar_imu.right_motor_speed_cmps)) / 2.0f;

            // --- 核心改动：基于实际速度判断小车是否停止，并消除前馈反冲 ---
            // 如果小车原始陀螺仪角速度和平均线速度都非常小，认为小车已停止或即将停止
            if (fabs(smartcar_imu.rpy_gyro_dps[2]) < ANGULAR_VELOCITY_STOP_THRESHOLD && 
                avg_linear_speed_abs < LINEAR_VELOCITY_STOP_THRESHOLD) {
                
                // 小车停止时，强制将所有相关的前馈补偿量设为0
                // 同时，将滤波后的历史数据也强制清零，避免残余值导致的反冲
                compensation_speed_x = 0.0f;
                filtered_car_yaw_angular_velocity = 0.0f; // 强制清零滤波后的角速度
                filtered_v_left = 0.0f; // 强制清零滤波后的左轮速
                filtered_v_right = 0.0f; // 强制清零滤波后的右轮速
                
                // 此时，speed_x 将只由PID控制器的输出决定，没有前馈补偿
            } else {
                // 小车正在运动：正常应用前馈补偿

                // 1. 小车自身旋转（陀螺仪yaw角速度）补偿
                float car_yaw_angular_velocity_deg_s = filtered_car_yaw_angular_velocity;
                float K_yaw_rate_feedforward = -0.00020f; // 你的系数
                compensation_speed_x -= K_yaw_rate_feedforward * car_yaw_angular_velocity_deg_s;

                // 2. 小车线速度引起的视场漂移补偿
                // 即使你的 K_linear_speed_comp 为0，此处逻辑也保留
                float car_linear_velocity = (filtered_v_left + filtered_v_right) / 2.0f; // 小车前进线速度 (cm/s)
                float gimbal_angle_x_rad = smartcar_imu.rpy_deg[2] * (3.1415 / 180.0f); // 使用 M_PI
                float K_linear_speed_comp = 0.0020; // 你的系数
                compensation_speed_x += K_linear_speed_comp * car_linear_velocity * sin(gimbal_angle_x_rad);
            }
            // 将补偿项加入到PID输出中
            speed_x += compensation_speed_x;

            // --- 输出死区 (Deadband) ---
            // 只有当期望的调整速度大于阈值，或者像素误差仍然较大时，才进行实际调整
            if (fabs(speed_x) < DEADZONE_SPEED_THRESHOLD && fabs(error_x) < DEADZONE_ERROR_THRESHOLD) {
                speed_x = 0; // 如果期望调整速度很小且误差在死区内，则不调整X轴
            }
            // 对Y轴也可以应用类似的死区
            if (fabs(speed_y) < DEADZONE_SPEED_THRESHOLD && fabs(error_y) < DEADZONE_ERROR_THRESHOLD) {
                speed_y = 0; // 如果期望调整速度很小且误差在死区内，则不调整Y轴
            }

            // 调整云台角度
            adjust_angle(&gimbal, speed_x, speed_y);

            if (fabs(error_x) < 3 && fabs(error_y) < 3) {
                counter++;
                if (counter > 10) {
                    laser_open();
					run_flag=1;
           
                }
            } else {
                counter = 0;
            }
        }

        uint16_t servo_pos_x = angle_to_encoder(extended_yaw + gimbal.angle_x);
        uint16_t servo_pos_y = angle_to_encoder(gimbal.angle_y);
        set_servo(servo_pos_x, servo_pos_y);
    }
	if(run_flag==1){
		//arc_straight_stop(aaa);
	}
}
void renwu4_r1_2025(char aaa) {
    if(trackless_output.unlock_flag == LOCK) return;
    
    static Gimbal gimbal = {0, 0};
    // 优化PID参数，增加微分作用抑制过冲
    static PIDController pid = {0.0027, 0, 0.08, 0, 0, 0, 0};  // 增大Kd至0.08
    
    // 状态控制变量
    static char run_flag = 0;
    static int task_step = 0;
    static int counter = 0;
    static int last_error_x = 0;
    static int target_lost_count = 0;
    static int target_find_count = 0;
    static int direction = 1;
    static float angle_increment = 45.0f / 250.0f;
    static float max_angle = 45.0f;

    // 分段前馈控制参数 - 角速度
    static const float ANG_VEL_THRES_LOW = 30.0f;   // 低角速度阈值(°/s)
    static const float ANG_VEL_THRES_HIGH = 65.0f;  // 高角速度阈值(°/s)
    static const float K_FF_LOW = -0.00023f;       // 低角速度区补偿系数
    static const float K_FF_MID = -0.00018f;       // 中角速度区补偿系数(原系数)
    static const float K_FF_HIGH = -0.0001f;      // 高角速度区补偿系数
    
    // 分段前馈控制参数 - 线速度
    static const float LIN_VEL_THRES_LOW = 10.0f;   // 低线速度阈值(cm/s)
    static const float LIN_VEL_THRES_HIGH = 40.0f;  // 高线速度阈值(cm/s)
    static const float K_LIN_LOW = 0.0022f;        // 低线速度区补偿系数
    static const float K_LIN_MID = 0.0028f;        // 中线速度区补偿系数(原系数)
    static const float K_LIN_HIGH = 0.0015f;       // 高线速度区补偿系数

    // 新增：加速度前馈控制参数
    static const float ACCEL_THRESHOLD = 45.0f;    // 加速度阈值(deg/s2)
    static const float K_ACCEL_FEEDFORWARD = 0.0015f; // 加速度补偿系数
    
    // 新增：运动状态标志
    static int accelerating = 0;
    static int decelerating = 0;
    static float last_ang_vel = 0.0f;
    static float last_lin_vel = 0.0f;

    float extended_yaw = unwrap_yaw(smartcar_imu.rpy_deg[2]);
    float speed_x = 0, speed_y = 0;

    if (task_step == 0) {
        // 第一步：仅使用陀螺仪
        speed_x = 0;
        speed_y = 0;
        counter++;
        if (counter > 30) {
            task_step = 1;  // 进入扫描模式
            counter = 0;
            direction = -1;
            gimbal.angle_x = 0.0f;
            gimbal.angle_y = 0.0f;
            
            // 重置运动状态
            accelerating = 0;
            decelerating = 0;
            last_ang_vel = 0.0f;
            last_lin_vel = 0.0f;
        }
    } else if (task_step == 1) {
        // 扫描模式：左右移动寻找目标
        float error_x = get_maixcam_x() - 224 + get_goal_x() + 15;
        float error_y = get_maixcam_y() - 224 + get_goal_y() - 28;
        
        // 左右移动控制
        gimbal.angle_y = 0;
        gimbal.angle_x += direction * angle_increment;
        
        // 检查是否到达边界
        if (gimbal.angle_x <= -max_angle) {
            gimbal.angle_x = -max_angle;
            direction = 1;  // 改变方向向右
        } else if (gimbal.angle_x >= max_angle) {
            gimbal.angle_x = max_angle;
            direction = -1;  // 改变方向向左
        }

        // 检测是否找到目标
        if((get_maixcam_x() - 224) != 666) {
            // 目标找到，检查是否可以进入跟踪模式
            target_find_count++;
            if (target_find_count >= 8) {
                task_step = 2;  // 进入跟踪模式
                target_find_count = 0;
                
                // 重置运动状态
                accelerating = 0;
                decelerating = 0;
                last_ang_vel = 0.0f;
                last_lin_vel = 0.0f;
            }
        }

        // 舵机控制
        uint16_t servo_pos_x = angle_to_encoder(extended_yaw + gimbal.angle_x);
        uint16_t servo_pos_y = angle_to_encoder(gimbal.angle_y);
        set_servo(servo_pos_x, servo_pos_y);

    } else if (task_step == 2) {
        // 跟踪模式：摄像头控制 + 激光判断
        float error_x = get_maixcam_x() - 224 + get_goal_x() + 15;
        float error_y = get_maixcam_y() - 224 + get_goal_y() - 28;

        // 检测目标丢失
        if ((get_maixcam_x() - 224) == 666) {
            target_lost_count++;
            if (target_lost_count >= 25) {
                // 目标丢失，回到扫描模式
                task_step = 1;
                direction = -1;
                target_lost_count = 0;
                
                // 重置运动状态
                accelerating = 0;
                decelerating = 0;
                last_ang_vel = 0.0f;
                last_lin_vel = 0.0f;
            }
        } else {
            target_lost_count = 0;
            // 计算PID输出
            compute_pid(&pid, error_x, error_y, &speed_x, &speed_y);

            // --- 传感器数据滤波 ---
            // 陀螺仪角速度滤波
            filtered_car_yaw_angular_velocity = ALPHA_GYRO * smartcar_imu.rpy_gyro_dps[2] 
                                             + (1.0f - ALPHA_GYRO) * filtered_car_yaw_angular_velocity;

            // 轮速滤波
            filtered_v_left = ALPHA_WHEEL_SPEED * smartcar_imu.left_motor_speed_cmps 
                            + (1.0f - ALPHA_WHEEL_SPEED) * filtered_v_left;
            filtered_v_right = ALPHA_WHEEL_SPEED * smartcar_imu.right_motor_speed_cmps 
                             + (1.0f - ALPHA_WHEEL_SPEED) * filtered_v_right;

            float compensation_speed_x = 0.0f; // 初始化补偿量为0

            // 计算小车平均线速度的绝对值，用于判断是否静止
            float avg_linear_speed_abs = (fabs(smartcar_imu.left_motor_speed_cmps) 
                                       + fabs(smartcar_imu.right_motor_speed_cmps)) / 2.0f;

            // --- 核心改动：基于实际速度判断小车是否停止 ---
            if (fabs(smartcar_imu.rpy_gyro_dps[2]) < ANGULAR_VELOCITY_STOP_THRESHOLD && 
                avg_linear_speed_abs < LINEAR_VELOCITY_STOP_THRESHOLD) {
                
                // 小车停止时，强制清零所有相关值
                compensation_speed_x = 0.0f;
                filtered_car_yaw_angular_velocity = 0.0f;
                filtered_v_left = 0.0f;
                filtered_v_right = 0.0f;
                
                // 重置运动状态
                accelerating = 0;
                decelerating = 0;
                last_ang_vel = 0.0f;
                last_lin_vel = 0.0f;
            } else {
                // 小车正在运动：应用分段前馈补偿
                
                // 1. 角速度分段前馈补偿
                float car_yaw_angular_velocity_deg_s = filtered_car_yaw_angular_velocity;
                float abs_ang_vel = fabs(car_yaw_angular_velocity_deg_s);
                
                // 根据角速度大小选择不同的前馈系数
                float K_yaw_rate_feedforward = K_FF_MID;  // 默认中速区系数
                
                if (abs_ang_vel < ANG_VEL_THRES_LOW) {
                    K_yaw_rate_feedforward = K_FF_LOW;
                } 
                else if (abs_ang_vel > ANG_VEL_THRES_HIGH) {
                    K_yaw_rate_feedforward = K_FF_HIGH;
                }
                
                compensation_speed_x -= K_yaw_rate_feedforward * car_yaw_angular_velocity_deg_s;

                // 2. 线速度分段前馈补偿
                float car_linear_velocity = (filtered_v_left + filtered_v_right) / 2.0f;
                float abs_lin_vel = fabs(car_linear_velocity);
                float gimbal_angle_x_rad = smartcar_imu.rpy_deg[2] * (3.1415926f / 180.0f);
                
                // 根据线速度大小选择不同的前馈系数
                float K_linear_speed_comp = K_LIN_MID;  // 默认中速区系数
                
                if (abs_lin_vel < LIN_VEL_THRES_LOW) {
                    K_linear_speed_comp = K_LIN_LOW;
                } 
                else if (abs_lin_vel > LIN_VEL_THRES_HIGH) {
                    K_linear_speed_comp = K_LIN_HIGH;
                }
                
                compensation_speed_x += K_linear_speed_comp * car_linear_velocity * sin(gimbal_angle_x_rad);
                
                // ===== 新增：加速度前馈补偿 =====
                // 计算角加速度 (deg/s2)
                static float CONTROL_PERIOD = 0.02f; // 假设控制周期20ms
                float ang_accel = (car_yaw_angular_velocity_deg_s - last_ang_vel) / CONTROL_PERIOD;
                last_ang_vel = car_yaw_angular_velocity_deg_s;
                
                // 计算线加速度 (cm/s2)
                float lin_accel = (car_linear_velocity - last_lin_vel) / CONTROL_PERIOD;
                last_lin_vel = car_linear_velocity;
                
                // 检测加速/减速状态
                accelerating = (fabs(ang_accel) > ACCEL_THRESHOLD && ang_accel > 0) || 
                              (fabs(lin_accel) > ACCEL_THRESHOLD && lin_accel > 0);
                decelerating = (fabs(ang_accel) > ACCEL_THRESHOLD && ang_accel < 0) || 
                              (fabs(lin_accel) > ACCEL_THRESHOLD && lin_accel < 0);
                
                // 应用加速度前馈补偿
                if (accelerating || decelerating) {
                    // 增加加速度补偿项
                    compensation_speed_x += K_ACCEL_FEEDFORWARD * ang_accel;
                    
                    // 在加减速阶段临时调整滤波参数（减少延迟）
                    float temp_alpha = ALPHA_GYRO * 1.5f; // 增加50%的响应速度
                    if (temp_alpha > 0.9f) temp_alpha = 0.9f;
                    
                    // 重新计算滤波值（减少延迟）
                    filtered_car_yaw_angular_velocity = temp_alpha * smartcar_imu.rpy_gyro_dps[2] 
                                                     + (1.0f - temp_alpha) * filtered_car_yaw_angular_velocity;
                }
            }
            
            // 将补偿项加入到PID输出中
            speed_x += compensation_speed_x;

            // --- 输出死区 (Deadband) ---
            // 动态调整死区阈值：在加减速阶段放宽死区
            float dynamic_deadzone_threshold = DEADZONE_SPEED_THRESHOLD;
            if (accelerating || decelerating) {
                dynamic_deadzone_threshold *= 1.5f; // 增加50%死区范围
            }
            
            if (fabs(speed_x) < dynamic_deadzone_threshold && fabs(error_x) < DEADZONE_ERROR_THRESHOLD) {
                speed_x = 0; // 如果期望调整速度很小且误差在死区内，则不调整X轴
            }
            if (fabs(speed_y) < dynamic_deadzone_threshold && fabs(error_y) < DEADZONE_ERROR_THRESHOLD) {
                speed_y = 0; // 不调整Y轴
            }

            // 调整云台角度
            adjust_angle(&gimbal, speed_x, speed_y);

            // 检查是否对准目标并打开激光
            // 在加减速阶段放宽对准阈值
            float align_threshold = 3.0f;
            if (accelerating || decelerating) {
                align_threshold = 5.0f; // 放宽对准阈值
            }
            
            if (fabs(error_x) < align_threshold && fabs(error_y) < align_threshold) {
                counter++;
                if (counter > 10) {
                    laser_open();
                    run_flag = 1;
                }
            } else {
                counter = 0;
            }
        }

        uint16_t servo_pos_x = angle_to_encoder(extended_yaw + gimbal.angle_x);
        uint16_t servo_pos_y = angle_to_encoder(gimbal.angle_y);
        set_servo(servo_pos_x, servo_pos_y);
    }
    
    if(run_flag == 1) {
        // 激光已打开，执行后续动作
        arc_straight_stop(aaa);
    }
}
void get_circle_point(char radius, int* x, int* y) {
    static int call_count = 0;        // 调用计数器
    static double current_angle = 0;  // 当前角度（弧度）
    
    // 确保半径不超过30
    if (radius > 30) radius = 30;
    
    // 每4000次调用重置（20秒 * 200次/秒 = 4000次）
    if (call_count >= 4000) {
        call_count = 0;
        current_angle = 0;
    }
    
    // 计算动态比例因子
    float scale_factor;
    if (call_count < 2000) {
        // 前2000次：比例从1.5线性减少到0.5
        scale_factor = 1.5f - (call_count / 2000.0f);
    } else {
        // 后2000次：比例从0.5线性增加到1.5
        scale_factor = 0.5f + ((call_count - 2000) / 2000.0f);
    }
    
    // 计算当前角度（每次增加π/2000弧度）
    current_angle = (2 * 3.1415 * call_count) / 4000.0;	
    
    // 计算浮点坐标并四舍五入为整数
    *x = (int)(radius * cos(current_angle) * scale_factor);
    *y = (int)(radius * sin(current_angle) * scale_factor);
    
    // 增加调用计数
    call_count++;
}
//void get_circle_point(char radius, int* x, int* y) {
//    // 静态变量保存状态
//    static int call_count = 0;        // 调用计数器
//    static double current_angle = 0;  // 当前角度（弧度）
//	static float a=1.5;
//    
//    // 确保半径不超过30
//    if (radius > 30) radius = 30;
//    
//    // 每4000次调用重置（20秒 * 200次/秒 = 4000次）
//    if (call_count >= 4000) {
//        call_count = 0;
//        current_angle = 0;
//    }
//    // 计算当前角度（每次增加π/2000弧度）
//    current_angle = (2 * 3.1415 * call_count) / 4000.0;	
//    // 计算浮点坐标并四舍五入为整数
//    *x = (int)(radius * cos(current_angle)*a);
//    *y = (int)(radius * sin(current_angle)*a);
//    // 调试输出（实际使用时可以移除）
//    call_count++;
//}
void renwu6_2025(char aaa) {
    if(trackless_output.unlock_flag == LOCK) return;
    if(trackless_output.unlock_flag == LOCK) return;
    static int yuan_x=0;
	static int yuan_y=0;
    static Gimbal gimbal = {0, 0};
    // 优化PID参数，增加微分作用抑制过冲
    static PIDController pid = {0.0027, 0, 0.08, 0, 0, 0, 0};  // 增大Kd至0.08
    
    // 状态控制变量
    static char run_flag = 0;
    static int task_step = 0;
    static int counter = 0;
    static int last_error_x = 0;
    static int target_lost_count = 0;
    static int target_find_count = 0;
    static int direction = 1;
    static float angle_increment = 45.0f / 250.0f;
    static float max_angle = 45.0f;

    // 分段前馈控制参数 - 角速度
    static const float ANG_VEL_THRES_LOW = 30.0f;   // 低角速度阈值(°/s)
    static const float ANG_VEL_THRES_HIGH = 65.0f;  // 高角速度阈值(°/s)
    static const float K_FF_LOW = -0.00023f;       // 低角速度区补偿系数
    static const float K_FF_MID = -0.00018f;       // 中角速度区补偿系数(原系数)
    static const float K_FF_HIGH = -0.0001f;      // 高角速度区补偿系数
    
    // 分段前馈控制参数 - 线速度
    static const float LIN_VEL_THRES_LOW = 10.0f;   // 低线速度阈值(cm/s)
    static const float LIN_VEL_THRES_HIGH = 40.0f;  // 高线速度阈值(cm/s)
    static const float K_LIN_LOW = 0.0022f;        // 低线速度区补偿系数
    static const float K_LIN_MID = 0.0028f;        // 中线速度区补偿系数(原系数)
    static const float K_LIN_HIGH = 0.0015f;       // 高线速度区补偿系数

    // 新增：加速度前馈控制参数
    static const float ACCEL_THRESHOLD = 45.0f;    // 加速度阈值(deg/s2)
    static const float K_ACCEL_FEEDFORWARD = 0.0012f; // 加速度补偿系数
    
    // 新增：运动状态标志
    static int accelerating = 0;
    static int decelerating = 0;
    static float last_ang_vel = 0.0f;
    static float last_lin_vel = 0.0f;

    float extended_yaw = unwrap_yaw(smartcar_imu.rpy_deg[2]);
    float speed_x = 0, speed_y = 0;

    if (task_step == 0) {
        // 第一步：仅使用陀螺仪
        speed_x = 0;
        speed_y = 0;
        counter++;
        if (counter > 30) {
            task_step = 1;  // 进入扫描模式
            counter = 0;
            direction = -1;
            gimbal.angle_x = 0.0f;
            gimbal.angle_y = 0.0f;
            
            // 重置运动状态
            accelerating = 0;
            decelerating = 0;
            last_ang_vel = 0.0f;
            last_lin_vel = 0.0f;
        }
    } else if (task_step == 1) {
        // 扫描模式：左右移动寻找目标
        float error_x = get_maixcam_x() - 224 + get_goal_x() + 15 +yuan_x;
        float error_y = get_maixcam_y() - 224 + get_goal_y() - 28 + yuan_y;
        
        // 左右移动控制
        gimbal.angle_y = 0;
        gimbal.angle_x += direction * angle_increment;
        
        // 检查是否到达边界
        if (gimbal.angle_x <= -max_angle) {
            gimbal.angle_x = -max_angle;
            direction = 1;  // 改变方向向右
        } else if (gimbal.angle_x >= max_angle) {
            gimbal.angle_x = max_angle;
            direction = -1;  // 改变方向向左
        }

        // 检测是否找到目标
        if((get_maixcam_x() - 224) != 666) {
            // 目标找到，检查是否可以进入跟踪模式
            target_find_count++;
            if (target_find_count >= 8) {
                task_step = 2;  // 进入跟踪模式
                target_find_count = 0;
                
                // 重置运动状态
                accelerating = 0;
                decelerating = 0;
                last_ang_vel = 0.0f;
                last_lin_vel = 0.0f;
            }
        }

        // 舵机控制
        uint16_t servo_pos_x = angle_to_encoder(extended_yaw + gimbal.angle_x);
        uint16_t servo_pos_y = angle_to_encoder(gimbal.angle_y);
        set_servo(servo_pos_x, servo_pos_y);

    } else if (task_step == 2) {
        // 跟踪模式：摄像头控制 + 激光判断
        float error_x = get_maixcam_x() - 224 + get_goal_x() + 15+yuan_x;
        float error_y = get_maixcam_y() - 224 + get_goal_y() - 28 + yuan_y;

        // 检测目标丢失
        if ((get_maixcam_x() - 224) == 666) {
            target_lost_count++;
            if (target_lost_count >= 25) {
                // 目标丢失，回到扫描模式
                task_step = 1;
                direction = -1;
                target_lost_count = 0;
                
                // 重置运动状态
                accelerating = 0;
                decelerating = 0;
                last_ang_vel = 0.0f;
                last_lin_vel = 0.0f;
            }
        } else {
            target_lost_count = 0;
            // 计算PID输出
            compute_pid(&pid, error_x, error_y, &speed_x, &speed_y);

            // --- 传感器数据滤波 ---
            // 陀螺仪角速度滤波
            filtered_car_yaw_angular_velocity = ALPHA_GYRO * smartcar_imu.rpy_gyro_dps[2] 
                                             + (1.0f - ALPHA_GYRO) * filtered_car_yaw_angular_velocity;

            // 轮速滤波
            filtered_v_left = ALPHA_WHEEL_SPEED * smartcar_imu.left_motor_speed_cmps 
                            + (1.0f - ALPHA_WHEEL_SPEED) * filtered_v_left;
            filtered_v_right = ALPHA_WHEEL_SPEED * smartcar_imu.right_motor_speed_cmps 
                             + (1.0f - ALPHA_WHEEL_SPEED) * filtered_v_right;

            float compensation_speed_x = 0.0f; // 初始化补偿量为0

            // 计算小车平均线速度的绝对值，用于判断是否静止
            float avg_linear_speed_abs = (fabs(smartcar_imu.left_motor_speed_cmps) 
                                       + fabs(smartcar_imu.right_motor_speed_cmps)) / 2.0f;

            // --- 核心改动：基于实际速度判断小车是否停止 ---
            if (fabs(smartcar_imu.rpy_gyro_dps[2]) < ANGULAR_VELOCITY_STOP_THRESHOLD && 
                avg_linear_speed_abs < LINEAR_VELOCITY_STOP_THRESHOLD) {
                
                // 小车停止时，强制清零所有相关值
                compensation_speed_x = 0.0f;
                filtered_car_yaw_angular_velocity = 0.0f;
                filtered_v_left = 0.0f;
                filtered_v_right = 0.0f;
                
                // 重置运动状态
                accelerating = 0;
                decelerating = 0;
                last_ang_vel = 0.0f;
                last_lin_vel = 0.0f;
            } else {
                // 小车正在运动：应用分段前馈补偿
                
                // 1. 角速度分段前馈补偿
                float car_yaw_angular_velocity_deg_s = filtered_car_yaw_angular_velocity;
                float abs_ang_vel = fabs(car_yaw_angular_velocity_deg_s);
                
                // 根据角速度大小选择不同的前馈系数
                float K_yaw_rate_feedforward = K_FF_MID;  // 默认中速区系数
                
                if (abs_ang_vel < ANG_VEL_THRES_LOW) {
                    K_yaw_rate_feedforward = K_FF_LOW;
                } 
                else if (abs_ang_vel > ANG_VEL_THRES_HIGH) {
                    K_yaw_rate_feedforward = K_FF_HIGH;
                }
                
                compensation_speed_x -= K_yaw_rate_feedforward * car_yaw_angular_velocity_deg_s;

                // 2. 线速度分段前馈补偿
                float car_linear_velocity = (filtered_v_left + filtered_v_right) / 2.0f;
                float abs_lin_vel = fabs(car_linear_velocity);
                float gimbal_angle_x_rad = smartcar_imu.rpy_deg[2] * (3.1415926f / 180.0f);
                
                // 根据线速度大小选择不同的前馈系数
                float K_linear_speed_comp = K_LIN_MID;  // 默认中速区系数
                
                if (abs_lin_vel < LIN_VEL_THRES_LOW) {
                    K_linear_speed_comp = K_LIN_LOW;
                } 
                else if (abs_lin_vel > LIN_VEL_THRES_HIGH) {
                    K_linear_speed_comp = K_LIN_HIGH;
                }
                
                compensation_speed_x += K_linear_speed_comp * car_linear_velocity * sin(gimbal_angle_x_rad);
                
                // ===== 新增：加速度前馈补偿 =====
                // 计算角加速度 (deg/s2)
                static float CONTROL_PERIOD = 0.02f; // 假设控制周期20ms
                float ang_accel = (car_yaw_angular_velocity_deg_s - last_ang_vel) / CONTROL_PERIOD;
                last_ang_vel = car_yaw_angular_velocity_deg_s;
                
                // 计算线加速度 (cm/s2)
                float lin_accel = (car_linear_velocity - last_lin_vel) / CONTROL_PERIOD;
                last_lin_vel = car_linear_velocity;
                
                // 检测加速/减速状态
                accelerating = (fabs(ang_accel) > ACCEL_THRESHOLD && ang_accel > 0) || 
                              (fabs(lin_accel) > ACCEL_THRESHOLD && lin_accel > 0);
                decelerating = (fabs(ang_accel) > ACCEL_THRESHOLD && ang_accel < 0) || 
                              (fabs(lin_accel) > ACCEL_THRESHOLD && lin_accel < 0);
                
                // 应用加速度前馈补偿
                if (accelerating || decelerating) {
                    // 增加加速度补偿项
                    compensation_speed_x += K_ACCEL_FEEDFORWARD * ang_accel;
                    
                    // 在加减速阶段临时调整滤波参数（减少延迟）
                    float temp_alpha = ALPHA_GYRO * 1.5f; // 增加50%的响应速度
                    if (temp_alpha > 0.9f) temp_alpha = 0.9f;
                    
                    // 重新计算滤波值（减少延迟）
                    filtered_car_yaw_angular_velocity = temp_alpha * smartcar_imu.rpy_gyro_dps[2] 
                                                     + (1.0f - temp_alpha) * filtered_car_yaw_angular_velocity;
                }
            }
            
            // 将补偿项加入到PID输出中
            speed_x += compensation_speed_x;

            // --- 输出死区 (Deadband) ---
            // 动态调整死区阈值：在加减速阶段放宽死区
            float dynamic_deadzone_threshold = DEADZONE_SPEED_THRESHOLD;
            if (accelerating || decelerating) {
                dynamic_deadzone_threshold *= 1.5f; // 增加50%死区范围
            }
            
            if (fabs(speed_x) < dynamic_deadzone_threshold && fabs(error_x) < DEADZONE_ERROR_THRESHOLD) {
                speed_x = 0; // 如果期望调整速度很小且误差在死区内，则不调整X轴
            }
            if (fabs(speed_y) < dynamic_deadzone_threshold && fabs(error_y) < DEADZONE_ERROR_THRESHOLD) {
                speed_y = 0; // 不调整Y轴
            }

            // 调整云台角度
            adjust_angle(&gimbal, speed_x, speed_y);

            // 检查是否对准目标并打开激光
            // 在加减速阶段放宽对准阈值
            float align_threshold = 3.0f;
            if (accelerating || decelerating) {
                align_threshold = 5.0f; // 放宽对准阈值
            }
            
            if (fabs(error_x) < align_threshold && fabs(error_y) < align_threshold) {
                counter++;
                if (counter > 10) {
                    laser_open();
                    run_flag = 1;
                }
            } else {
                counter = 0;
            }
        }

        uint16_t servo_pos_x = angle_to_encoder(extended_yaw + gimbal.angle_x);
        uint16_t servo_pos_y = angle_to_encoder(gimbal.angle_y);
        set_servo(servo_pos_x, servo_pos_y);
    }
    
    if(run_flag == 1) {
        // 激光已打开，执行后续动作
        arc_straight_stop(aaa);
		get_circle_point(15,&yuan_x,&yuan_y);
    }
}




