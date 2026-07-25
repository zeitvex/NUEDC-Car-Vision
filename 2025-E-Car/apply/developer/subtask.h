#ifndef __SUBTASK_H
#define __SUBTASK_H


/**********************************************************************************************************************/
#define track_speed_cmps_default  20				//默认侧面轨迹寻迹时的巡航速度10 
#define start_point_adjust1_default -2				//识别到倒车入库视觉特征点后，向前或者向后需要调整的距离-8.0f

#define forward_distance_cm_default 20				//倒车入库第一个动作执行的距离——轮胎左打并前进
#define backward_distance1_cm_default 27			//倒车入库第二个动作执行的距离——轮胎右打并后退
#define backward_distance2_cm_default 30      //倒车入库第三个动作执行的距离——轮胎回正并后退

#define out_forward_distance1_cm_default 25		//倒车入库后出库第一动作执行距离——轮胎回正并前进
#define out_forward_distance2_cm_default 40   //倒车入库后出库第二动作执行距离——轮胎右打并前进

/**********************************************************************************************************************/
#define start_point_adjust2_default 25////识别到侧方入库视觉特征点后，向前或者向后需要调整的距离30  25
#define parallel_backward_distance1_cm_default 22.0f//侧方入库第一个动作执行的距离——前轮右打时，回退距离  20 25
#define parallel_backward_distance2_cm_default 17.5f//侧方入库第二个动作执行的距离——前轮回正时，回退距离  20
#define parallel_backward_distance3_cm_default 15.0f//侧方入库第三个动作执行的距离——前轮左打时，回退距离	15
/**********************************************************************************************************************/


typedef struct
{
	float _track_speed_cmps;
	float _start_point_adjust1;
	float _forward_distance_cm;
	float _backward_distance1_cm;
	float _backward_distance2_cm;
	float _out_forward_distance1_cm;
	float _out_forward_distance2_cm;
	float _start_point_adjust2;
	float _parallel_backward_distance1_cm;
	float _parallel_backward_distance2_cm;
	float _parallel_backward_distance3_cm;
}_park_params;




void flight_subtask_reset(void);


//void flight_subtask_1(void);
//void flight_subtask_2(void);
//void flight_subtask_3(void);
//void flight_subtask_4(void);
//void flight_subtask_5(void);
void auto_reverse_stall_park(void);
void auto_parallel_park(void);


//uint8_t flight_subtask_yaw_angle_ctrl(float target_angle);
//void auto_drive_smartcar_duty1(void);
//void auto_drive_smartcar_duty2(void);	
//void auto_drive_smartcar_duty3(uint16_t times);
//void auto_drive_smartcar_duty4(uint16_t times);
//void auto_nav_point(uint16_t times);


void renwu1_2025(int repeat_times);
void renwu4_2025(char aaa);
void renwu4_r1_2025(char aaa);
void renwu6_2025(char aaa);
void tracking_task();
void arc_straight_stop(char fl);
extern _park_params park_params;




#define distance_ctrl_speed_max     50   //50
#define self_guided_tracking_speed  60   //60
#define move_diagonal_angle1        35   //38.65
#define move_diagonal_distance1     125  //128
#define move_diagonal_angle2        35   //38.65
#define move_diagonal_distance2     125  //128



extern float _distance_ctrl_speed_max,_self_guided_tracking_speed,_move_diagonal_angle1,_move_diagonal_distance1,_move_diagonal_angle2,_move_diagonal_distance2;


#endif
