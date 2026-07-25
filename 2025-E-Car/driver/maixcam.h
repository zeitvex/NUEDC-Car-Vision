#ifndef __maixcam
#define __maixcam
#include "headfile.h"
#include "user.h"

void maixcam_rec(uint8_t data);
uint16_t get_maixcam_x();
uint16_t get_maixcam_y();


// 云台舵机的角度结构体
typedef struct {
    float angle_x; // 水平角度
    float angle_y; // 垂直角度
} Gimbal;

// PID 控制结构体
typedef struct {
    float Kp;     // 比例系数
    float Ki;     // 积分系数
    float Kd;     // 微分系数
    float prev_error_x;  // 上一次的误差值（x轴）
    float prev_error_y;  // 上一次的误差值（y轴）
    float integral_x;    // 积分项（x轴）
    float integral_y;    // 积分项（y轴）
} PIDController;
float apply_nonlinear_mapping(float error);
void compute_pid(PIDController *pid, float error_x, float error_y, float *speed_x, float *speed_y);
void adjust_angle(Gimbal *gimbal, float speed_x, float speed_y);
void control_gimbal(Gimbal *gimbal, PIDController *pid, float target_x, float target_y);
void laser_open();
void add_goal_cam(int8_t a,int8_t b);
void laser_close();
int8_t get_goal_y();
int8_t get_goal_x();
#endif