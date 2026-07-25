#include "headfile.h"
#include "gray_detection.h"
#include "user.h"


#define steer_deadzone 50//50  转向控制死区

typedef struct
{
	uint16_t state;
  bool gray_bit[16];
}_gray_state; 
_gray_state gray_state;
float gray_status[2]={0},gray_status_backup[2][20]={0};//灰度传感器状态与历史值
float turn_output=0,turn_output_last=0;//控制器输出值
float	turn_scale=turn_scale_default;//转向控制差速系数  0.15
uint32_t gray_status_worse=0;	//灰度管异常状态计数器
controller seektrack_ctrl[2];		//自主寻迹控制器结构体
uint32_t vision_status_worse=0;



/***************************************************
函数名: void gpio_input_init(void)
说明:	12路灰度管gpio检测初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
****************************************************/
void gpio_input_init(void)
{

}

#define read_gray_bit1   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT0_PIN ) ? 0x01 : 0x00)
#define read_gray_bit2   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT1_PIN ) ? 0x01 : 0x00)
#define read_gray_bit3   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT2_PIN ) ? 0x01 : 0x00)
#define read_gray_bit4   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT3_PIN ) ? 0x01 : 0x00)
#define read_gray_bit5   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT4_PIN ) ? 0x01 : 0x00)
#define read_gray_bit6   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT5_PIN ) ? 0x01 : 0x00)
#define read_gray_bit7   ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT6_PIN ) ? 0x01 : 0x00)
#define read_gray_bit8   ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT7_PIN ) ? 0x01 : 0x00)
#define read_gray_bit9   ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT8_PIN ) ? 0x01 : 0x00)
#define read_gray_bit10  ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT9_PIN ) ? 0x01 : 0x00)
#define read_gray_bit11  ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT10_PIN) ? 0x01 : 0x00)
#define read_gray_bit12  ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT11_PIN) ? 0x01 : 0x00)


/***************************************************
函数名: void gpio_input_check_channel_7(void)
说明:	7路灰度管gpio检测
入口:	无
出口:	无
备注:	无
作者:	无名创新
****************************************************/
void gpio_input_check_channel_7(void)
{
	
}


/***************************************************
函数名: void gpio_input_check_channel_12(void)
说明:	12路灰度管gpio检测
入口:	无
出口:	无
备注:	无
作者:	无名创新
****************************************************/
void gpio_input_check_channel_12(void)
{
	
}


#define startpoint_check_state_keep_cm  50.0f
float startpoint_distance_cm=0;
uint8_t startpoint_check_flag=0;
uint8_t track_switch_flag=0;
float startpoint_straightaway_cm=0;
/***************************************************
函数名: void gpio_input_check_channel_12_with_handle(void)
说明:	12路灰度管gpio检测,带赛道信息检测
入口:	无
出口:	无
备注:	无
作者:	无名创新
****************************************************/
void gpio_input_check_channel_12_with_handle(void)
{

}



/***************************************************
函数名: void gray_turn_control_200hz(float *output)
说明:	灰度管寻迹时的转向控制
入口:	float *output-控制器输出
出口:	无
备注:	无
作者:	无名创新
****************************************************/
void gray_turn_control_200hz(float *output)
{
	
	//保存上次控制器输出
	turn_output_last=turn_output;
	//转向PD控制输出，舵向控制实时性要求高，引入积分I会使舵向响应滞后
	seektrack_ctrl[0].expect=0;							//期望
	seektrack_ctrl[0].feedback=gray_status[0];	//反馈
	pid_control_run(&seektrack_ctrl[0]);		  //控制器运算
	turn_output=seektrack_ctrl[0].output;
	//叠加死区控制
	if(turn_output>0) turn_output+=steer_deadzone;
	if(turn_output<0) turn_output-=steer_deadzone;
	//输出限幅
	turn_output=constrain_float(turn_output,-500,500);//转向控制输出限幅
	*output=0.85f*turn_output+0.15f*turn_output_last;//输出为前后两次计算的均值
}
/***************************************************
函数名: void gpio_input_check_from_vision(void)
说明:	openmv视觉寻迹时的偏差检测
入口:	无
出口:	无
备注:	无
作者:	无名创新
****************************************************/
void gpio_input_check_from_vision(void)
{
	
}

/***************************************************
函数名: void vision_turn_control_50hz(float *output)
说明:	openmv视觉寻迹时的转向控制
入口:	float *output-控制器输出
出口:	无
备注:	无
作者:	无名创新
****************************************************/
void vision_turn_control_50hz(float *output)
{
	
}



/**********************************************************/
uint16_t road_miss_cnt=0;
uint8_t road_miss_flag=0;
uint8_t road_restore_flag=0;//赛道恢复标志位
void gpio_input_check_channel_12_2024(void)//已修改
{
	gray_state.gray_bit[0]=read_gray_bit1;//最右边
	gray_state.gray_bit[1]=read_gray_bit2;
	gray_state.gray_bit[2]=read_gray_bit3;
	gray_state.gray_bit[3]=read_gray_bit4;

	gray_state.gray_bit[4]=read_gray_bit5;
	gray_state.gray_bit[5]=read_gray_bit6;
	gray_state.gray_bit[6]=read_gray_bit7;
	gray_state.gray_bit[7]=read_gray_bit8;
	
	gray_state.gray_bit[8] =read_gray_bit9;
	gray_state.gray_bit[9] =read_gray_bit10;
	gray_state.gray_bit[10]=read_gray_bit11;
	gray_state.gray_bit[11]=read_gray_bit12;//最左边

	if(gray_state.gray_bit[11])
	{
		gray_status[0]=9;return;
	}
	gray_state.state=0x0000;


	for(uint16_t i=0;i<12;i++)
	{
		gray_state.state|=gray_state.gray_bit[i]<<i;
	}	
	for(uint16_t i=19;i>0;i--)
	{
		gray_status_backup[0][i]=gray_status_backup[0][i-1];
	}
	gray_status_backup[0][0]=gray_status[0];
	switch(gray_state.state)
	{
		
		case 0x0001:{gray_status[0]=-7;}break;									//000000000001b
		case 0x0003:{gray_status[0]=-6;}break;									//000000000011b
		case 0x0002:{gray_status[0]=-6;}break;									//000000000010b
		case 0x0007:{gray_status[0]=-5;}break;									//000000000111b
		case 0x0006:{gray_status[0]=-5;}break;									//000000000110b
		case 0x0004:{gray_status[0]=-4;}break;									//000000000100b
		case 0x000E:{gray_status[0]=-4;}break;									//000000001110b
		case 0x000C:{gray_status[0]=-3;}break;									//000000001100b
		case 0x0008:{gray_status[0]=-3;}break;									//000000001000b
		case 0x001C:{gray_status[0]=-2;}break;									//000000011100b
		case 0x0018:{gray_status[0]=-2;}break;									//000000011000b
		case 0x0010:{gray_status[0]=-2;}break;									//000000010000b
		case 0x0038:{gray_status[0]=-1;}break;									//000000111000b
		case 0x0030:{gray_status[0]=-1;}break;									//000000110000b
		case 0x0020:{gray_status[0]=-1;}break;									//000000100000b
		case 0x0070:{gray_status[0]= 0;}break;									//000001110000b
		case 0x0060:{gray_status[0]= 0;}break;									//000001100000b
		case 0x0040:{gray_status[0]= 0;}break;									//000001000000b
		case 0x00E0:{gray_status[0]= 1;}break;									//000011100000b
		case 0x00C0:{gray_status[0]= 1;}break;									//000011000000b
		case 0x0080:{gray_status[0]= 1;}break;									//000010000000b
		case 0x01C0:{gray_status[0]= 2;}break;									//000111000000b
		case 0x0180:{gray_status[0]= 2;}break;									//000110000000b
		case 0x0100:{gray_status[0]= 2;}break;									//000100000000b
		case 0x0380:{gray_status[0]= 3;}break;									//001110000000b
		case 0x0300:{gray_status[0]= 3;}break;									//001100000000b
		case 0x0200:{gray_status[0]= 4;}break;									//001000000000b
		case 0x0700:{gray_status[0]= 4;}break;									//011100000000b
		case 0x0600:{gray_status[0]= 5;}break;									//011000000000b
		case 0x0400:{gray_status[0]= 5;}break;									//010000000000b
		case 0x0E00:{gray_status[0]= 6;}break;									//111000000000b
		case 0x0C00:{gray_status[0]= 6;}break;									//110000000000b
		case 0x0800:{gray_status[0]= 7;}break;									//100000000000b
		case 0x0000:{gray_status[0]=gray_status_backup[0][0];}break;//00000000b
		break;
		default://其它特殊情况单独判断
		{
			if(gray_state.state==0xf00)gray_status[0]= 8;
			if(gray_state.state==0xf80)gray_status[0]= 8;
			if(gray_state.state==0xfc0)gray_status[0]= 8;
		}
	}	
	
}

	
		
		