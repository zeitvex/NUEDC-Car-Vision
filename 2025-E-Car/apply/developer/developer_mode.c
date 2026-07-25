#include "headfile.h"
#include "sdk.h"
#include "subtask.h"
#include "user.h"
#include "developer_mode.h"
#include "nhds.h"

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
		case 0://遥控控制
		{
			
		}
		break;		
		case 1://基于灰度管的自主寻迹
		{	
			if(trackless_output.unlock_flag==LOCK)return;
			renwu1_2025(1);
			//tracking_task();	
			
			//arc_straight_stop();
		}
		break;		
		case 2://顺时针转动90°
		{
			if(trackless_output.unlock_flag==LOCK)return;
			renwu1_2025(2);
		}
		break;
		case 3://逆时针转动90°
		{
			if(trackless_output.unlock_flag==LOCK)return;
			renwu1_2025(3);
		}
		break;
		case 4://顺时针转动90°
		{
			if(trackless_output.unlock_flag==LOCK)return;
			renwu1_2025(4);
		}
		break;
		case 5://以30deg/s的角速度逆时针转动3000ms
		{
			if(trackless_output.unlock_flag==LOCK)return;
			renwu1_2025(5);
			//tracking_task();
		}
		break;
		case 6://基于超声波测距的前向避撞小车
		{
			if(trackless_output.unlock_flag == LOCK)return;
			static Gimbal gimbal = {0, 0};
			static PIDController pid = {0.0025, 0, 0.06, 0, 0, 0, 0};//已经被修改
			// 状态控制变量
			static int task_step = 0;
			static int counter = 0;  // 多用途计数器，替代多个单一用途计数器
			
			static int last_error_x = 0;
			static int target_find_count = 0;
			static int target_lost_count = 0;
			static int direction = -1;  // -1: 向左移动, 1: 向右移动
			static float angle_increment = 45.0f / 300.0f;  // 每步0.225度
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
					float error_x = get_maixcam_x() - 224 + 15;
					float error_y = get_maixcam_y() - 224 - 28;
					gimbal.angle_y=0;
					// 左右移动控制
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
					float error_x = get_maixcam_x() - 224 + 15;
					float error_y = get_maixcam_y() - 224 - 28;

					// 检测目标丢失
					if ((get_maixcam_x()-224)==666) {
							target_lost_count++;
							if (target_lost_count >= 50) {
									// 目标丢失，回到扫描模式
									task_step = 1;
									direction = -1;
									target_lost_count = 0;
							}
					} else {
							compute_pid(&pid, error_x, error_y, &speed_x, &speed_y);
							adjust_angle(&gimbal, speed_x, speed_y);

							if (fabs(error_x) < 5 && fabs(error_y) < 5) {
									counter++;
									if (counter > 4) {
										laser_open();
									}
							} else {
									//counter = 0;
								//laser_close();
							}
					}

					uint16_t servo_pos_x = angle_to_encoder(extended_yaw + gimbal.angle_x);
					uint16_t servo_pos_y = angle_to_encoder(gimbal.angle_y);
					set_servo(servo_pos_x, servo_pos_y);
			}
		}
		break;		
		case 7://两轮平衡车
		{
			if(trackless_output.unlock_flag==LOCK)return;
			renwu4_r1_2025(4);
		}
		break;
		case 8://基于两轮差速模型的速度、角速度控制,用于机载计算机ROS端发生运动指令控制下位机差速平台
		{
			if(trackless_output.unlock_flag==LOCK)return;
			renwu4_r1_2025(5);
		}
		break;
		case 9://地面站航点控制模式，通过无名创新地面站V1.0.6版本发布航点
		{
			if(trackless_output.unlock_flag==LOCK)return;
			renwu6_2025(4);
		}
		break;		
		case 10://OPENMV视觉自主寻迹
		{
			if(trackless_output.unlock_flag==LOCK)return;
			static uint8_t co=0;
			if(co++<=2)return;
			else co=0;
			ServoPosition b;
			static uint16_t i=0;
			robot_path_data_point_t xh;
			xh=robot_path_data[i++];
			if(xh.pen_state==0)laser_close();
			else laser_open();
			b=calculate_servo_position(xh.x,xh.y,500);
			set_servo(b.yaw_encoder,b.pitch_encoder);
			if(i>=1116)i=0;
		}
		break;
		case 11://双电机+前轮舵机转向遥控控制
		{
				
		}	
		break;
		case 12://双电机+前轮舵机转向，视觉自主寻迹
		{
			
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
			
		}
		break;
		case 16://2024年电赛H题第1问
		{
			
		}
		break;
		case 17://2024年电赛H题第2问
		{
			
		}
		break;
		case 18://2024年电赛H题第3问
		{
			
		}
		break;
		case 19://2024年电赛H题第4问
		{
			
		}
		break;
		case 20://2024年电赛H题发挥部分
		{
			
		}
		break;
		case 21://直接偏航控制-原地掉头学习案例
		{
			
		}
		break;
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
