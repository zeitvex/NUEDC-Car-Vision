#include "maixcam.h"

uint8_t sdk_state = 0;
uint8_t sdk_receive_buf[9];
uint8_t sdk_data_payload_cnt = 0;

uint16_t g_parsed_int1 = 224;
uint16_t g_parsed_int2 = 224;

int8_t goal_x=0;
int8_t goal_y=0;


int8_t get_goal_x(){
	return goal_x;
}
int8_t get_goal_y(){
	return goal_y;
}
void add_goal_cam(int8_t a,int8_t b){
	goal_x+=a;
	goal_y+=b;
}


void SDK_ProcessCompleteFrame(uint8_t *frame_buffer){
	uint8_t calculated_checksum = 0;
	for (uint8_t i = 0; i < 8; i++)calculated_checksum += frame_buffer[i];
	uint8_t received_checksum = frame_buffer[8]; // 接收到的校验和
	if (calculated_checksum == received_checksum){
		g_parsed_int1 = ((uint16_t)frame_buffer[4] << 8) | frame_buffer[5];
		g_parsed_int2 = ((uint16_t)frame_buffer[6] << 8) | frame_buffer[7];
	}
	else
	{
	
  }
}
uint16_t get_maixcam_x(){
	return g_parsed_int1;
}

uint16_t get_maixcam_y(){
	return g_parsed_int2;
}
/**
 * @brief SDK 数据接收准备函数（状态机实现）。
 *
 * 此函数为字节流方式的数据接收接口，每次接收一个字节。
 * 它通过状态机解析固定的帧协议。
 *
 * @param data 接收到的一个字节数据。
 */
void maixcam_rec(uint8_t data)
{
	switch (sdk_state){
		case 0: // 状态0：等待帧头1 (0xFF)
			if (data == 0xFF){
				sdk_state = 1;
				sdk_receive_buf[0] = data; // 存储帧头1
			}break;
		case 1: // 状态1：等待帧头2 (0xFE)
			if (data == 0xFE){
				sdk_state = 2;
				sdk_receive_buf[1] = data; // 存储帧头2
			}
			else sdk_state = 0;
			break;
		case 2: // 状态2：等待控制字 (0x55)
			if (data == 0x55){
				sdk_state = 3;
				sdk_receive_buf[2] = data; // 存储控制字
			}
			else sdk_state = 0;
			break;
		case 3: // 状态3：等待数据长度 (0x04，固定值)
				if (data == 0x04)
				{
						sdk_state = 4;
						sdk_receive_buf[3] = data;      // 存储数据长度字节
						sdk_data_payload_cnt = 0;       // 重置数据载荷字节计数器
				}
				else // 协议错误，重置状态机
				{
						sdk_state = 0;
				}
				break;

		case 4: // 状态4：接收数据载荷 (总共4个字节)
				// 将数据字节存储到缓冲区中，从索引4开始
				sdk_receive_buf[4 + sdk_data_payload_cnt] = data;
				sdk_data_payload_cnt++; // 递增已接收数据字节计数

				if (sdk_data_payload_cnt == 4) // 如果已经接收了所有4个数据字节
				{
						sdk_state = 5; // 进入等待校验和的状态
				}
				// 否则，继续留在状态4，等待更多数据字节
				break;

		case 5: // 状态5：接收校验和字节
				sdk_receive_buf[8] = data; // 存储校验和字节 (帧的最后一个字节)
				sdk_state = 0;             // 完整帧已接收，重置状态机，准备接收下一帧

				// 调用处理函数，对完整的帧进行校验和验证和数据解析
				SDK_ProcessCompleteFrame(sdk_receive_buf);
				break;

		default: // 默认情况：如果状态异常，则重置状态机
				sdk_state = 0;
				break;
	}
}

// 定义一些常量
#define FRAME_RATE 30                // 每秒30帧
#define MAX_ANGLE 90                // 云台最大角度范围
#define MIN_ANGLE -90               // 云台最小角度范围
#define MAX_SPEED 7                 // 最大速度
#define MIN_SPEED -7                // 最小速度（负值，表示反方向）
#define DEAD_ZONE 3                  // 差值小于这个值时不做调整

void compute_pid(PIDController *pid, float error_x, float error_y, float *speed_x, float *speed_y) {
    // 如果误差在死区内，不做调整
    if (fabs(error_x) < DEAD_ZONE) error_x = 0;
    if (fabs(error_y) < DEAD_ZONE) error_y = 0;
    
    // 计算积分和微分项
    pid->integral_x += error_x;
    pid->integral_y += error_y;

    float derivative_x = error_x - pid->prev_error_x;
    float derivative_y = error_y - pid->prev_error_y;

    // 非线性分段函数处理误差
    float nonlinear_error_x = apply_nonlinear_mapping(error_x);
    float nonlinear_error_y = apply_nonlinear_mapping(error_y);

    // 计算 PID 输出
    *speed_x = pid->Kp * nonlinear_error_x + pid->Ki * pid->integral_x + pid->Kd * derivative_x;
    *speed_y = pid->Kp * nonlinear_error_y + pid->Ki * pid->integral_y + pid->Kd * derivative_y;

    // 保存当前误差，作为下一次计算的上一误差
    pid->prev_error_x = error_x;
    pid->prev_error_y = error_y;

    // 限制速度范围（允许正负值）
    *speed_x = fmin(MAX_SPEED, fmax(MIN_SPEED, *speed_x));
    *speed_y = fmin(MAX_SPEED, fmax(MIN_SPEED, *speed_y));
}

// 非线性分段函数映射
float apply_nonlinear_mapping(float error) {
    float abs_error = fabs(error);
    float sign = (error >= 0) ? 1.0f : -1.0f;
    float result;
    
    // 分段函数：根据误差大小使用不同的映射关系
    if (abs_error <= 15.0f) {
        // 小误差区域：线性响应，保持原始误差
        result = error*1.5;
    } else if (abs_error <= 35.0f) {
        // 中等误差区域：平方关系，增强响应
        result = sign * (10.0f + 1.5f * (abs_error - 10.0f) * (abs_error - 10.0f) / 20.0f);
    } else if (abs_error <= 80.0f) {
        // 大误差区域：更强的非线性响应
        float base = 30.0f ; // 中等区域的最大值
        result = sign * (base + 2.0f * (abs_error - 30.0f));
    } else {
        // 极大误差区域：最大响应
        float base = 35.0f; 
        result = sign * (base + 3.0f * (abs_error - 60.0f));
    }
    
    return result;
}

// 调整云台的角度
void adjust_angle(Gimbal *gimbal, float speed_x, float speed_y) {
    // 只有当速度不为0时才调整角度
    if (speed_x != 0) gimbal->angle_x += speed_x;
    if (speed_y != 0) gimbal->angle_y += speed_y;
    // 限制云台角度在最大最小值范围内
    gimbal->angle_x = fmin(MAX_ANGLE, fmax(MIN_ANGLE, gimbal->angle_x));
    gimbal->angle_y = fmin(MAX_ANGLE, fmax(MIN_ANGLE, gimbal->angle_y));
}

void laser_open(){
	DL_GPIO_setPins(PORTB_PORT,PORTB_laser_PIN);
}

void laser_close(){
	DL_GPIO_clearPins(PORTB_PORT,PORTB_laser_PIN);
}
