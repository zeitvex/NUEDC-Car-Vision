#ifndef __ftServo
#define __ftServo
#include "headfile.h"
#include "user.h"



// 舵机参数定义
#define SERVO_CENTER 2048    // 舵机中心位置编码器值
#define SERVO_MAX 360       // 舵机最大角度范围(±180度)

// 舵机控制结构体
typedef struct {
	uint16_t yaw_encoder;    // 水平舵机编码器值
  uint16_t pitch_encoder;  // 垂直舵机编码器值
} ServoPosition;

typedef struct {
	uint16_t yaw_speed;    // 水平速度
  uint16_t pitch_speed;  // 垂直速度
} ServoSpeed;

typedef struct {
	uint16_t yaw_speed;    // 水平速度
  uint16_t pitch_speed;  // 垂直速度
	uint16_t yaw_pos;    // 水平舵机编码器值
  uint16_t pitch_pos;  // 垂直舵机编码器值
	bool is_open;
} yuntai;



// 函数声明
ServoPosition calculate_servo_position(float x, float y, float distance);
void Servo_MoveToPosition(uint8_t id,uint16_t position, uint16_t speed);
void Servo_loop();
float unwrap_yaw(float angle);
float gyro_to_servo_angle(float gyro_angle);
float calculate_servo_angle(float last_gyro_angle, float current_gyro_angle, int *turns);

uint16_t get_encoder_yaw();
uint16_t get_encoder_pitch();
void set_servo(uint16_t x,uint16_t y);
void add_servo(uint16_t x,uint16_t y);
uint16_t angle_to_encoder(float angle);
void open_servo();
void close_servo();
void Servo_init();
#endif