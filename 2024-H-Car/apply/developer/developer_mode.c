#include "headfile.h"
#include "sdk.h"
#include "subtask.h"
#include "user.h"
#include "developer_mode.h"

int16_t sdk_work_mode=0;


#define wheel_space_cm  12.8f//轮间距  12.8cm



void sdk_duty_run(void)
{
	if(trackless_output.init==0)
	{		
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output=0;
		trackless_output.init=1;
		flight_subtask_reset();//复位sdk子任务状态量
	}
	if(smartcar_imu.imu_convergence_flag!=1) return;//姿态解算系统就位
	
	switch(sdk_work_mode)
	{
		case -10://初始调试模式，用于确定电机运动方向时使用
		{
			speed_ctrl_mode=0;  //直接开环输出指定PWM数值，用于调试电机方向
			motion_ctrl_pwm=motion_test_pwm_default;//默认输出百分之50占空的pwm
		}
		break;
		case -9://初始调试模式，用于确定舵机中值时使用
		{
			speed_ctrl_mode=0;  //直接开环输出指定PWM数值，用于调试电机方向
			motion_ctrl_pwm=0;//默认输出百分之50占空的pwm
			steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		}
		break;
		case -2://调速测试模式——速度期望来源于遥控
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定
			speed_expect[0]=speed_setup;//左边轮子速度期望
			speed_expect[1]=speed_setup;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);				
		}
		break;
		case -1://调速测试模式——速度期望来源于按键设定
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=speed_setup;//左边轮子速度期望
			speed_expect[1]=speed_setup;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);				
		}
		break;
		case 0://遥控控制
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
			trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);				
		}
		break;		
		case 1://基于灰度管的自主寻迹
		{
			renwu_1();
		}
		break;		
		case 2://顺时针转动90°
		{
			renwu_2();
		}
		break;
		case 3://逆时针转动90°
		{
			renwu_3();
		}
		break;
		case 4://顺时针转动90°
		{
			
		}
		break;
		case 5:{
			kaishi();
		}
		break;
		case 6://基于超声波测距的前向避撞小车
		{
			go_work();
		}
		break;		
		case 7://两轮平衡车
		{
			speed_ctrl_mode=2;//速度控制方式为平衡速度控制
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定
			speed_setup=remote_data_remap(&RC_Data ,RC_PITCH_CHANNEL ,50,50,false);//将遥杆动作位映射成期望速度
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
			
			trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
			trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定
			steer_control(&turn_ctrl_pwm);	//转向陀螺仪控制
			balance_control_single_control();			
		}
		break;
		case 8://基于两轮差速模型的速度、角速度控制,用于机载计算机ROS端发生运动指令控制下位机差速平台
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
	
			trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定
			turn_ctrl_pwm=trackless_output.yaw_outer_control_output*DEG2RAD;//期望角速度转换成弧度制
			
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定
			//期望速度
			speed_expect[0]=speed_setup-turn_ctrl_pwm*wheel_space_cm*0.5f;//左边轮子速度期望
			speed_expect[1]=speed_setup+turn_ctrl_pwm*wheel_space_cm*0.5f;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);	
		}
		break;
		case 9://地面站航点控制模式，通过无名创新地面站V1.0.6版本发布航点
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			position_control();
			turn_ctrl_pwm=steer_gyro_output;
			speed_setup=distance_ctrl.output;
			//期望速度
			speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);			
		}
		break;		
		case 10://OPENMV视觉自主寻迹
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			vision_turn_control_50hz(&turn_ctrl_pwm);//基于OPENMV视觉处理的转向控制
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 11://双电机+前轮舵机转向遥控控制
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+RC_Data.rcdata[RC_YAW_CHANNEL]-1500);	
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
			//期望速度
			speed_expect[0]=speed_setup;//左边轮子速度期望
			speed_expect[1]=speed_setup;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);	
		}	
		break;
		case 12://双电机+前轮舵机转向，视觉自主寻迹
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			vision_turn_control_50hz(&turn_ctrl_pwm);//基于OPENMV视觉处理的转向控制
			steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+turn_ctrl_pwm);	
			//期望速度
			speed_expect[0]=speed_setup;//左边轮子速度期望
			speed_expect[1]=speed_setup;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);	
		}
		break;
		case 13://倒车入库
		{
			auto_reverse_stall_park();
		}
		break;
		case 14://侧方停车
		{
			auto_parallel_park();
		}		
		break;
		case 15://2022年7月份省赛小车跟随行驶系统赛道,内外圈交替循迹
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			//gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 16:
		{
			//用户预留任务，编写后注意加上break跳出
		}
		case 17:
		{
			//用户预留任务，编写后注意加上break跳出
		}
		case 18:
		{
			//用户预留任务，编写后注意加上break跳出
		}
		case 19:
		{
			//用户预留任务，编写后注意加上break跳出
		}
		case 20:
		{
			
			
			
			
			
			
			
		}
		case 21:
		{
			//用户预留任务，编写后注意加上break跳出
		}
		case 22:
		{
			//用户预留任务，编写后注意加上break跳出
		}
		default:
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
			trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);			
		}
	}
}

