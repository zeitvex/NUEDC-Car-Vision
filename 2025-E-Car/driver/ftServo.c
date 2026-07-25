#include "ftServo.h"
#include <math.h>
yuntai yuntai_out;
/*******************************************************************************
* 名    称： 乱七八糟的函数
* 功    能： 可以直接从外部更改变量的值，也可以通过函数更改变量的值，以免被错误修改，
* 创建日期： 2025-07-24
*******************************************************************************/
void open_servo(){
	yuntai_out.is_open=1;
}
void close_servo(){
	yuntai_out.is_open=0;
}
void set_servo(uint16_t x,uint16_t y){
	yuntai_out.pitch_pos=y;
	yuntai_out.yaw_pos=x;
}
uint16_t get_encoder_yaw(){
	return yuntai_out.yaw_pos;
}
uint16_t get_encoder_pitch(){
	return yuntai_out.pitch_pos;
}
/*******************************************************************************
* 名    称： angle_to_encoder
* 功    能： 将指定角度转换为舵机数值
* 输    入： 指定的角度
* 输    出： 舵机编码器对应的数值
* 创建日期： 2025-07-24
*******************************************************************************/
uint16_t angle_to_encoder(float extended_angle) {
    float servo_value = (extended_angle + 180.0f) *11.376388;
    // 限制范围
    if (servo_value < 0) servo_value = 0;
    if (servo_value > 12287) servo_value = 12287;

    return (uint16_t)servo_value;
}
/*******************************************************************************
* 名    称： Servo_init
* 功    能： 将舵机的各项输出归位
* 创建日期： 2025-07-24
*******************************************************************************/
void Servo_init(){
	yuntai_out.is_open=1;
	yuntai_out.pitch_pos=SERVO_CENTER;
	yuntai_out.yaw_pos=SERVO_CENTER;
	yuntai_out.pitch_speed=0xe803;
	yuntai_out.yaw_speed=0xe803;
	Servo_MoveToPosition(1,yuntai_out.yaw_pos,yuntai_out.yaw_speed);
	Servo_MoveToPosition(2,yuntai_out.pitch_pos,yuntai_out.pitch_speed);
}
/*******************************************************************************
* 名    称： calculate_servo_position
* 功    能： 将uv纸上的坐标转换成舵机的编码器数值
* 输    入： 图像相对于uv纸的坐标x，y，以及和舵机的距离
* 输    出： 舵机编码器对应的数值
* 创建日期： 2025-07-24
*******************************************************************************/
ServoPosition calculate_servo_position(float x, float y, float distance) {
	ServoPosition result;
  float yaw, pitch;
  if (distance < 0.1f) distance = 0.1f;
  yaw = atan2f(x, distance) * 180.0f / 3.14159265f;
  float projected_distance = sqrtf(x*x + distance*distance);
	pitch = atan2f(y, projected_distance) * 180.0f / 3.14159265f;
	result.yaw_encoder = angle_to_encoder(yaw);
	result.pitch_encoder = angle_to_encoder(pitch);
	return result;
}
/*******************************************************************************
* 名    称： Servo_loop
* 功    能： 舵机事件循环,控制串口发送的时间间隔
* 输    入： 无
* 输    出： 无
* 创建日期： 2025-07-24
*******************************************************************************/
void Servo_loop(){
	static uint8_t cont=0;
	if(yuntai_out.is_open==0)return;
	if(cont++<4)return;
	cont=0;
	Servo_MoveToPosition(1,yuntai_out.yaw_pos,yuntai_out.yaw_speed);
	Servo_MoveToPosition(2,yuntai_out.pitch_pos,yuntai_out.pitch_speed);
}
/*******************************************************************************
* 名    称： Servo_MoveToPosition
* 功    能： 自定义的数据帧
* 输    入： 无
* 输    出： 无
* 创建日期： 2025-07-24
*******************************************************************************/
void Servo_MoveToPosition(uint8_t id,uint16_t position, uint16_t speed)
{
    uint8_t cmd[13];
    uint8_t checksum = 0;
    // 指令帧结构
    cmd[0] = 0xFF; // 字头
    cmd[1] = 0xFF; // 字头
    cmd[2] = id; // ID
    cmd[3] = 0x09; // 长度
    cmd[4] = 0x03; // 指令
    cmd[5] = 0x2A; // 参数首地址
    cmd[6] = position & 0xFF;        // 位置低字节
    cmd[7] = (position >> 8) & 0xFF; // 位置高字节
    cmd[8] = 0x00; // 预留低字节
    cmd[9] = 0x00; // 预留高字节
    cmd[10] = speed & 0xFF;        // 速度低字节
    cmd[11] = (speed >> 8) & 0xFF; // 速度高字节
    for(int i=2; i<12; i++)checksum += cmd[i];
    cmd[12] = ~checksum;
    UART_SendBytes(UART_3_INST, cmd, 13);
}


// 定义常量
float unwrap_yaw(float angle) {
    static float last_angle = 0.0f;
    static int revolutions = 0;
    float delta = angle - last_angle;
    if (delta > 180.0f)
        revolutions -= 1;
    else if (delta < -180.0f)
        revolutions += 1;
    last_angle = angle;
    return angle + revolutions * 360.0f;
}
