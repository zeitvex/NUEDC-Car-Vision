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

/***************************************************
函数名: void gpio_input_check_channel_7(void)
说明:	7路灰度管gpio检测
入口:	无
出口:	无
备注:	无
****************************************************/
void gpio_input_check_channel_7(void)
{
	//gray_state.gray_bit[0]=read_gray_bit1;
 
	gray_state.gray_bit[0]=read_gray_bit3;
	gray_state.gray_bit[1]=read_gray_bit2;
	gray_state.gray_bit[2]=read_gray_bit4;
  gray_state.gray_bit[3]=read_gray_bit5;
	gray_state.gray_bit[4]=read_gray_bit7;
	gray_state.gray_bit[5]=read_gray_bit6;
	
	gray_state.gray_bit[6]=read_gray_bit9;
	gray_state.gray_bit[7]=read_gray_bit10;
	
	gray_state.state=0x0000;
	for(uint16_t i=0;i<8;i++)
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
		case 0x0001:gray_status[0]=-1.3;	gray_status_worse/=2;break;									//000001b
		case 0x0003:gray_status[0]=-0.9;	gray_status_worse/=2;break;									//000011b
	  case 0x0002:gray_status[0]=-0.65;	gray_status_worse/=2;break;									//000010b
		case 0x0006:gray_status[0]=-0.25;	gray_status_worse/=2;break;									//000110b
		case 0x0004:gray_status[0]=-0.15;	gray_status_worse/=2;break;									//000100b
		case 0x000C:gray_status[0]=0;			gray_status_worse/=2;break;									//001100b
		case 0x0008:gray_status[0]=0.15;	gray_status_worse/=2;break;								//001000b
		case 0x0018:gray_status[0]=0.25;  gray_status_worse/=2;break;								//011000b
		case 0x0010:gray_status[0]=0.65;  gray_status_worse/=2;break;									//010000b
		case 0x0030:gray_status[0]=0.9;   gray_status_worse/=2;break;									//110000b
		case 0x0020:gray_status[0]=1.3;   gray_status_worse/=2;break;									//100000b
		
		case 0x0080:gray_status[0]=-2;   gray_status_worse/=2;break;									//10 00 00 00
		case 0x0040:gray_status[0]=2;    gray_status_worse/=2;break;									//01 00 00 00
		
		case 0x0000:gray_status[0]=gray_status_backup[0][0];gray_status_worse++;break; //0000000b
		default:
		{
			gray_status[0]=gray_status_backup[0][0];
			gray_status_worse++;
		}
	}
}

void gpio_input_check_channel_4(void) {
  gray_state.gray_bit[0]=0;
  gray_state.gray_bit[1]=0;
	
	gray_state.gray_bit[2]=0;//read_gray_bit3;
	gray_state.gray_bit[3]=1;//read_gray_bit4;
  gray_state.gray_bit[4]=0;//read_gray_bit5;
	gray_state.gray_bit[5]=0;//read_gray_bit6;
	
	gray_state.gray_bit[6]=0;
	
	
	gray_state.state=0x0000;
	for(uint16_t i=0;i<6;i++)
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
//		case 0x0001:gray_status[0]=-7;	gray_status_worse/=2;break;									//0000001b
//		case 0x0003:gray_status[0]=-5;	gray_status_worse/=2;break;									//0000011b
		
	  case 0x0002:gray_status[0]=-5;	gray_status_worse/=2;break;									//0000010b
		case 0x0006:gray_status[0]=-3;	gray_status_worse/=2;break;									//0000110b
		case 0x0004:gray_status[0]=-1;	gray_status_worse/=2;break;									//0000100b
		case 0x000C:gray_status[0]=0;	  gray_status_worse/=2;break;									//0001100b
		case 0x0008:gray_status[0]=1;	  gray_status_worse/=2;break;									//0001000b
		case 0x0018:gray_status[0]=3;   gray_status_worse/=2;break;									//0011000b
		case 0x0010:gray_status[0]=5;   gray_status_worse/=2;break;									//0010000b
		
	//	case 0x0030:gray_status[0]=3;   gray_status_worse/=2;break;									//0110000b
		//case 0x0020:gray_status[0]=4;   gray_status_worse/=2;break;									//0100000b
//		case 0x0060:gray_status[0]=5;   gray_status_worse/=2;break;									//1100000b
//		case 0x0040:gray_status[0]=7;   gray_status_worse/=2;break;									//1000000b
		case 0x0000:gray_status[0]=gray_status_backup[0][0];gray_status_worse++;break; //0000000b
		default:
		{
			gray_status[0]=gray_status_backup[0][0];
			gray_status_worse++;
		}
	}
}
/***************************************************
函数名: void gpio_input_check_channel_12(void)
说明:	12路灰度管gpio检测
入口:	无
出口:	无
备注:	无
作者:	无名创新
****************************************************/
void gpio_input_check_channel_12(void) {
  gray_state.gray_bit[0]=read_gray_bit3;
  gray_state.gray_bit[1]=read_gray_bit4;
	gray_state.gray_bit[2]=read_gray_bit5;
	gray_state.gray_bit[3]=read_gray_bit6;
	gray_state.state=0x0000;	
	for(uint16_t i=0;i<4;i++)
	{
		gray_state.state|=gray_state.gray_bit[i]<<i;
	}
	
	for(uint16_t i=19;i>0;i--)
	{
		gray_status_backup[0][i]=gray_status_backup[0][i-1];
	}
	gray_status_backup[0][0]=gray_status[0];
   // if(gray_state.state==0x0000)gray_flag_1=0;
    //else gray_flag_1=1;
	if(gray_state.state!=0x0000)gray_flag_1=1;
	switch(gray_state.state)
	{
		case 0x0001:gray_status[0]=-1.2;	gray_status_worse/=2;break;									//0001
		case 0x0003:gray_status[0]=-0.7;	gray_status_worse/=2;break;									//0011
	  case 0x0002:gray_status[0]=-0.2;	gray_status_worse/=2;break;							 		//0010
		case 0x0006:gray_status[0]=0;	    gray_status_worse/=2;break;									//0110
		case 0x0004:gray_status[0]=0.2;	  gray_status_worse/=2;break;							    //0100
		case 0x000C:gray_status[0]=0.7;   gray_status_worse/=2;break;									//1100
		case 0x0008:gray_status[0]=1.2;   gray_status_worse/=2;break;                 //1000
		case 0x0000:gray_status[0]=gray_status_backup[0][0];gray_status_worse++;break; //0000000b
		default:
		{
             gray_status[0]=gray_status_backup[0][0];
			gray_status_worse++;
		}
	}
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
	gray_state.gray_bit[0]=read_gray_bit1;
	gray_state.gray_bit[1]=read_gray_bit2;
	gray_state.gray_bit[2]=read_gray_bit3;
	gray_state.gray_bit[3]=read_gray_bit4;

	gray_state.gray_bit[4]=read_gray_bit5;
	gray_state.gray_bit[5]=read_gray_bit6;
	gray_state.gray_bit[6]=read_gray_bit7;
	gray_state.gray_bit[7]=read_gray_bit8;
	
	gray_state.gray_bit[8]=read_gray_bit9;
	gray_state.gray_bit[9]=read_gray_bit10;
	gray_state.gray_bit[10]=read_gray_bit11;
	gray_state.gray_bit[11]=read_gray_bit12;
	
	gray_state.state=0x0000;	
	for(uint16_t i=0;i<11;i++)
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
		case 0x0001:gray_status[0]=-11; gray_status_worse/=2;break;									//000000000001b
		case 0x0003:gray_status[0]=-10; gray_status_worse/=2;break;									//000000000011b
	  case 0x0002:gray_status[0]=-9;	gray_status_worse/=2;break;									//000000000010b
		case 0x0006:gray_status[0]=-8;	gray_status_worse/=2;break;									//000000000110b
		case 0x0004:gray_status[0]=-7;	gray_status_worse/=2;break;									//000000000100b
		case 0x000C:gray_status[0]=-6;	gray_status_worse/=2;break;									//000000001100b
		case 0x0008:gray_status[0]=-5;	gray_status_worse/=2;break;									//000000001000b
		case 0x0018:gray_status[0]=-4;	gray_status_worse/=2;break;									//000000011000b
		case 0x0010:gray_status[0]=-3;	gray_status_worse/=2;break;									//000000010000b
		case 0x0030:gray_status[0]=-2;	gray_status_worse/=2;break;									//000000110000b
		case 0x0020:gray_status[0]=-1;	gray_status_worse/=2;break;									//000000100000b
		case 0x0060:gray_status[0]=0;		gray_status_worse/=2;break;									//000001100000b
		case 0x0040:gray_status[0]=1;		gray_status_worse/=2;break;									//000001000000b
		case 0x00C0:gray_status[0]=2;		gray_status_worse/=2;break;									//000011000000b
		case 0x0080:gray_status[0]=3;		gray_status_worse/=2;break;									//000010000000b
		case 0x0180:gray_status[0]=4;		gray_status_worse/=2;break;									//000110000000b
		case 0x0100:gray_status[0]=5;		gray_status_worse/=2;break;									//000100000000b
		case 0x0300:gray_status[0]=6;		gray_status_worse/=2;break;									//001100000000b
		case 0x0200:gray_status[0]=7;		gray_status_worse/=2;break;									//001000000000b
		case 0x0600:gray_status[0]=8;		gray_status_worse/=2;break;									//011000000000b
		case 0x0400:gray_status[0]=9;		gray_status_worse/=2;break;									//010000000000b
		case 0x0C00:gray_status[0]=10;	gray_status_worse/=2;break;									//110000000000b
		case 0x0800:gray_status[0]=11;	gray_status_worse/=2;break;									//100000000000b
		case 0x0000:gray_status[0]=gray_status_backup[0][0];gray_status_worse++;break;//000000000000b
		default://其它特殊情况单独判断
		{
			uint16_t tmp_state=gray_state.state;
			if(startpoint_check_flag==1&&track_switch_flag==1)//切换内圈
			{
				//只响应左侧灰度管的状态
				//tmp_state&=0x0FC0;//&&111111000000
				tmp_state&=0x0F80;//&&111110000000
				switch(tmp_state)
				{
					case 0x0040:gray_status[0]=1;	break;//000001000000b
					case 0x00C0:gray_status[0]=2;	break;//000011000000b
					case 0x0080:gray_status[0]=3;	break;//000010000000b
					case 0x0180:gray_status[0]=4;	break;//000110000000b
					case 0x0100:gray_status[0]=5;	break;//000100000000b
					case 0x0300:gray_status[0]=6;	break;//001100000000b
					case 0x0200:gray_status[0]=7;	break;//001000000000b
					case 0x0600:gray_status[0]=8;	break;//011000000000b
					case 0x0400:gray_status[0]=9;	break;//010000000000b
					case 0x0C00:gray_status[0]=10;break;//110000000000b
					case 0x0800:gray_status[0]=11;break;//100000000000b				
					default:gray_status[0]=0;
				}
			}
			else if(startpoint_check_flag==1&&track_switch_flag==0)//切换外圈
			{
				//只响应中间+右侧侧灰度管的状态
				tmp_state&=0x00FF;//&&000011111111
				switch(tmp_state)
				{
					case 0x0001:gray_status[0]=-11;break;//000000000001b
					case 0x0003:gray_status[0]=-10;break;//000000000011b
					case 0x0002:gray_status[0]=-9; break;//000000000010b
					case 0x0006:gray_status[0]=-8; break;//000000000110b
					case 0x0004:gray_status[0]=-7; break;//000000000100b
					case 0x000C:gray_status[0]=-6; break;//000000001100b
					case 0x0008:gray_status[0]=-5; break;//000000001000b
					case 0x0018:gray_status[0]=-4; break;//000000011000b
					case 0x0010:gray_status[0]=-3; break;//000000010000b
					case 0x0030:gray_status[0]=-2; break;//000000110000b	
					case 0x0020:gray_status[0]=-1;break;//000000100000b
					case 0x0060:gray_status[0]=0;	break;//000001100000b
					case 0x0040:gray_status[0]=1;	break;//000001000000b
					case 0x00C0:gray_status[0]=2;	break;//000011000000b		
					default:gray_status[0]=0;

				}				
			}
			else
			{
				gray_status[0]=gray_status_backup[0][0];
				gray_status_worse++;			
			}
		}
	}	
	float pos_delta=smartcar_imu.state_estimation.speed*0.001f;
	switch(gray_state.state)//出发点检测特殊处理
	{
		case 0x0001:startpoint_straightaway_cm=0;break;									//000000000001b
		case 0x0003:startpoint_straightaway_cm=0;break;									//000000000011b
	  case 0x0002:startpoint_straightaway_cm=0;break;									//000000000010b
		case 0x0006:startpoint_straightaway_cm=0;break;									//000000000110b
		case 0x0004:startpoint_straightaway_cm=0;break;									//000000000100b
		case 0x000C:startpoint_straightaway_cm=0;break;									//000000001100b
		case 0x0008:startpoint_straightaway_cm=0;break;									//000000001000b
		case 0x0018:startpoint_straightaway_cm=0;break;									//000000011000b
		case 0x0010:startpoint_straightaway_cm=0;break;									//000000010000b
		case 0x0030:startpoint_straightaway_cm+=pos_delta;break;				//000000110000b
		case 0x0020:startpoint_straightaway_cm+=pos_delta;break;				//000000100000b
		case 0x0060:startpoint_straightaway_cm+=pos_delta;break;				//000001100000b   正中间
		case 0x0040:startpoint_straightaway_cm+=pos_delta;break;				//000001000000b
		case 0x00C0:startpoint_straightaway_cm+=pos_delta;break;				//000011000000b
		case 0x0080:startpoint_straightaway_cm=0;break;									//000010000000b
		case 0x0180:startpoint_straightaway_cm=0;break;									//000110000000b
		case 0x0100:startpoint_straightaway_cm=0;break;									//000100000000b
		case 0x0300:startpoint_straightaway_cm=0;break;									//001100000000b
		case 0x0200:startpoint_straightaway_cm=0;break;									//001000000000b
		case 0x0600:startpoint_straightaway_cm=0;break;									//011000000000b
		case 0x0400:startpoint_straightaway_cm=0;break;									//010000000000b
		case 0x0C00:startpoint_straightaway_cm=0;break;									//110000000000b
		case 0x0800:startpoint_straightaway_cm=0;break;									//100000000000b
		//上述情形表明是正常赛道，也可以仅仅对靠近正中间的情形自加，相当于直道路约束
		//case 0x00F0://000011110000b//连续四位
		//case 0x01E0://000111100000b//连续四位
		//case 0x0078://000001111000b//连续四位
		case 0x01F0://000111110000b//连续五位
		case 0x00F8://000011111000b//连续五位
		case 0x01F8://000111111000b//连续六位
		{
			if(startpoint_check_flag==1)//已检测到停止点，标志位未清除时提前退出
			{
					break;
			}
			
			//30cm直道约束
			if(startpoint_straightaway_cm>=30)//当存在误判时,可尝试加大此值
			{
				startpoint_straightaway_cm=0;
				startpoint_check_flag=1;
				startpoint_distance_cm=smartcar_imu.state_estimation.distance;//记录识别到的出发点所在的距离
				
				if(track_switch_flag==0) 
				{
					track_switch_flag=1;
					bling_set(&light_red,2000,200,0.5,0,0);
				}
				else 
				{
					track_switch_flag=0;
					bling_set(&light_blue,2000,200,0.5,0,0);
				}
				
				beep.period=100;//200*5ms
				beep.light_on_percent=0.5f;			
				beep.reset=1;
				beep.times=1;			
			}
		}
		break;
		default:
		{
			//startpoint_straightaway_cm=0;
		}
	}	

	if(startpoint_check_flag==1)//检测到停止点
	{
		if(smartcar_imu.state_estimation.distance-startpoint_distance_cm>=startpoint_check_state_keep_cm)//检测标志位保持距离为50cm	
		{
			startpoint_check_flag=0;
			beep.period=50;//200*5ms
			beep.light_on_percent=0.5f;			
			beep.reset=1;
			beep.times=2;					
		}
	}
}



/***************************************************
函数名: void gray_turn_control_200hz(float *output)
说明:	灰度管寻迹时的转向控制
入口:	float *output-控制器输出
出口:	无
备注:	无
****************************************************/
void gray_turn_control_200hz(float *output,char *a)
{
		//数据异常处理
	*a=0;
	if(gray_status_worse>=400)//持续0.4S内检测不到正常黑线，可以认定为跑出赛道丢线处理
	{
		//trackless_output.unlock_flag=LOCK;//上锁
		*a=1;
	}	
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
	*output=0.75f*turn_output+0.25f*turn_output_last;//输出为前后两次计算的均值
	
}
void gary_con_4(){
	gray_state.gray_bit[0] = read_gray_bit3;
  gray_state.gray_bit[1] = read_gray_bit4;
  gray_state.gray_bit[2] = read_gray_bit5;
  gray_state.gray_bit[3] = read_gray_bit6;
	if(!gray_state.gray_bit[0]&&gray_state.gray_bit[1]&&gray_state.gray_bit[2]&&gray_state.gray_bit[3]){//右面碰线应该右转
		gray_status[0]=-3;	gray_status_worse/=2;
	}
	else if(!gray_state.gray_bit[0]&&!gray_state.gray_bit[1]&&gray_state.gray_bit[2]&&gray_state.gray_bit[3]){//右面碰线应该右转
		gray_status[0]=-2;	gray_status_worse/=2;
	}
	else if(gray_state.gray_bit[0]&&!gray_state.gray_bit[1]&&gray_state.gray_bit[2]&&gray_state.gray_bit[3]){//右面碰线应该右转
		gray_status[0]=-1;	gray_status_worse/=2;
	}
	else if(gray_state.gray_bit[0]&&!gray_state.gray_bit[1]&&!gray_state.gray_bit[2]&&gray_state.gray_bit[3]){//右面碰线应该右转
		gray_status[0]=0;	gray_status_worse/=2;
	}
	else if(gray_state.gray_bit[0]&&gray_state.gray_bit[1]&&!gray_state.gray_bit[2]&&gray_state.gray_bit[3]){//右面碰线应该右转
		gray_status[0]=1;	gray_status_worse/=2;
	}
	else if(gray_state.gray_bit[0]&&gray_state.gray_bit[1]&&!gray_state.gray_bit[2]&&!gray_state.gray_bit[3]){//右面碰线应该右转
		gray_status[0]=2;	gray_status_worse/=2;
	}
	else if(gray_state.gray_bit[0]&&gray_state.gray_bit[1]&&gray_state.gray_bit[2]&&!gray_state.gray_bit[3]){//右面碰线应该右转
		gray_status[0]=3;	gray_status_worse/=2;
	}
	else if(gray_state.gray_bit[0]&&gray_state.gray_bit[1]&&gray_state.gray_bit[2]&&gray_state.gray_bit[3]){//右面碰线应该右转
		gray_status[0]=0;	gray_status_worse++;
	}
	else if(!gray_state.gray_bit[0]&&!gray_state.gray_bit[1]&&!gray_state.gray_bit[2]&&!gray_state.gray_bit[3]){//右面碰线应该右转
		gray_status[0]=0;	gray_status_worse++;
	}
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
	static uint16_t cnt=0;
	cnt++;if(cnt<=20) return;cnt=0;//周期控制4*5ms=20ms
	
	//视觉输入偏差以50hz的频率输入与存储
	for(uint16_t i=19;i>0;i--)
	{
		gray_status_backup[1][i]=gray_status_backup[1][i-1];
	}
	gray_status_backup[1][0]=gray_status[1];

	//gray_status[1]=camera1.sdk_target.x;
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
	static uint16_t cnt=0;
	cnt++;if(cnt<=4) return;cnt=0;//周期控制4*5ms=20ms
	
	//保存上次控制器输出
	turn_output_last=turn_output;
	//转向PD控制输出，舵向控制实时性要求高，引入积分I会使舵向响应滞后
	seektrack_ctrl[1].expect=0;							//期望
	seektrack_ctrl[1].feedback=gray_status[1];	//反馈
	pid_control_run(&seektrack_ctrl[1]);		  //控制器运算
	turn_output=seektrack_ctrl[1].output;
	//叠加死区控制
	if(turn_output>0) turn_output+=0;
	if(turn_output<0) turn_output-=0;
	//输出限幅
	turn_output=constrain_float(turn_output,-500,500);//转向控制输出限幅
	
	*output=turn_output;
}

