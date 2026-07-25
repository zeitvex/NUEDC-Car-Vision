/****************************************************************************************
	MSPM0G3507电赛小车开源方案资源分配表--MSPM0学习中心交流群828746221		
	功能	单片机端口	外设端口
	无名创新地面站通讯	
		PA10-->UART0-TXD	USB转TTL-RXD
		PA11-->UART0-RXD	USB转TTL-TXD
	机器视觉OPENMV4 MINI	
		PA8-UART1-TXD	UART3-RXD
		PA9-->UART1-RXD	UART3-TXD
	手机蓝牙APP地面站	
		PA21-UART2-TXD	蓝牙串口模块RXD
		PA22-->UART2-RXD	蓝牙串口模块TXD
	US100超声波模块	
		PB2-UART3-TXD	US100-TX/TRIG
		PB3-->UART3-RXD	US100-RX/ECHO
	12路灰度传感器FPC	
	  PA31-->P1
		PA28-->P2
		PA1-->P3
		PA0-->P4
		PA25-->P5
		PA24-->P6
		PB24-->P7
		PB23-->P8
		PB19-->P9
		PB18-->P10
		PA16-->P11
		PB13-->P12
	电机控制MPWM	
		PA4-A0-PWM-CH3	  右边电机调速INA1
		PA7-->A0-PWM-CH2	右边电机调速INA2
		PA3-->A0-PWM-CH1	左边电机调速INB1
		PB14-->A0-PWM-CH0	左边电机调速INB2		
	舵机控制SPWM	
		PA15-A1-PWM-CH0	  预留1
		PB1-->A1-PWM-CH1	预留2
		PA23-->G7-PWM-CH0	预留3
		PA2-->G7-PWM-CH1	前轮舵机转向控制PWM
	编码器测速ENC	
		PB4-RIGHT-PULSE	  右边电机脉冲倍频输出P1
		PB5-->LEFT-PULSE	左边电机脉冲倍频输出P2
		PB6-->RIGHT-DIR	  右边电机脉冲鉴相输出D1
		PB7-->LEFT-DIR	  左边电机脉冲鉴相输出D2
	外置IMU接口IMU	
		PA29-I2C-SCL	MPU6050-SCL
		PA30-->I2C-SDA	MPU6050-SDA
		PB0-->HEATER	温控IO可选
	电池电压采集	
		PA26-ADC-VBAT	需要外部分压后才允许接入
****************************************************************************************/

#include "ti_msp_dl_config.h"
#include "headfile.h"


int main(void)
{
	usart_irq_config();     //串口中断配置
  SYSCFG_DL_init();	      //系统资源配置初始化	
	OLED_Init();						//显示屏初始化
	nADC_Init();					  //ADC初始化
	nGPIO_Init();						//蜂鸣器初始化
	w25qxx_gpio_init();     //板载w25q64初始化
	ctrl_params_init();			//控制参数初始化
	trackless_params_init();//硬件配置初始化
	simulation_pwm_init();  //模拟PWM初始化
	rc_range_init();				//遥控器行程初始化
	rangefinder_init();			//测距传感器-uart3
	while(ICM206xx_Init());	//加速度计/陀螺仪初始化
	rgb_init();							//RGB灯初始化	
	Encoder_Init();					//编码器资源初始化
	Button_Init();					//板载按键初始化
	PPM_Init();							//接收机PPM信号初始化
	timer_irq_config();
	gray_flag_1=0;
  while(1)
  {
		screen_display();//屏幕显示
		adc_statemachine();//adc采集状态机
  }
}





/***************************************
函数名:	void duty_200hz(void)
说明: 200hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void maple_duty_200hz(void)
{
	rc_data_input();							 //遥控器PPM数据处理
	get_wheel_speed();					   //获取轮胎转速
	sdk_duty_run();					  		 //SDK总任务控制
	motor_output(speed_ctrl_mode); //控制器输出
	rangefinder_statemachine();		 //超声波传感器数据获取
	imu_data_sampling();					 //加速度计、陀螺仪数据获采集
	get_battery_voltage();				 //ADC数据获取
	trackless_ahrs_update();			 //ahrs姿态更新
	imu_temperature_ctrl();				 //传感器恒温控制
	read_button_state_all();       //按键状态读取
	battery_voltage_detection();	 //电池电压检测
  laser_light_work(&beep);       //电源板蜂鸣器驱动
	bling_working(0);							 //RGB灯状态机
	
}
/***************************************
函数名:	void duty_1000hz(void)
说明: 1000hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void duty_1000hz(void)
{

		gpio_input_check_channel_7();//检测12路灰度灰度管状态
		//gary_con_4();
}


/***************************************
函数名:	void duty_100hz(void)
说明: 100hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void duty_100hz(void)
{
	NCLink_SEND_StateMachine();//无名创新地面站发送	
}


/***************************************
函数名:	void duty_10hz(void)
说明: 10hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void duty_10hz(void)
{
	//手机端app地面站发送	
	bluetooth_app_send(smartcar_imu.rpy_deg[ROL],
										 smartcar_imu.rpy_deg[PIT],
										 smartcar_imu.rpy_deg[YAW],
										 speed_setup,
										 smartcar_imu.left_motor_speed_cmps,
										 smartcar_imu.right_motor_speed_cmps,
										 smartcar_imu.state_estimation.pos.x,
										 smartcar_imu.state_estimation.pos.y);
}