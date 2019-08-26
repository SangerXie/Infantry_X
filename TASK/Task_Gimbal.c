#include "Task_Gimbal.h"


#include "arm_math.h"
#include "kalman.h"
#include "kalman_filter.h"

#include "can1.h"
#include "remote.h"
#include "control.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "Task_Chassis.h"

#include "vision.h"
#include "friction.h"
#include "magazine.h"
#include "led.h"


//注意,自瞄模式和打符模式的PID和普通模式是不同的

/*-------------------------云台角度预编译---------------------------*/
#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
	//云台
	#define Mech_Min_Pitch     2800   //up
	#define Mech_Mid_Pitch     3500
	#define Mech_Max_Pitch     3920    //down
	#define Mech_Min_Yaw       2300     //right
	#define Mech_Mid_Yaw       4055
	#define Mech_Max_Yaw       5900     //left

	//云台抬头角度,用于摩擦轮开启
	#define CLOUD_FRIC_PIT_UP  (Mech_Min_Pitch + 200)
	
	//云台底盘分离角度,用于yaw限位
	#define CLOUD_SEPAR_ANGLE  950		//小心会跟扭头模式冲突
	float down_sb_pitch = 530;//450
	float up_sb_pitch   = 100;//
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
	//云台
	#define Mech_Min_Pitch     4140   //up
	#define Mech_Mid_Pitch     5050
	#define Mech_Max_Pitch     5360    //down
	#define Mech_Min_Yaw       1065     //right
	#define Mech_Mid_Yaw       2770
	#define Mech_Max_Yaw       4450     //left

	//云台抬头角度,用于摩擦轮开启
	#define CLOUD_FRIC_PIT_UP  (Mech_Min_Pitch + 200)//加得越多抬头越小
	
	//云台底盘分离角度,用于yaw限位
	#define CLOUD_SEPAR_ANGLE  800		//小心会跟扭头模式冲突
	
	float down_sb_pitch = 650;
	float up_sb_pitch   = 100;//
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
	//云台
	#define Mech_Min_Pitch     3380   //up
	#define Mech_Mid_Pitch     4095		
	#define Mech_Max_Pitch     4340    //down
	#define Mech_Min_Yaw       6100     //right
	#define Mech_Mid_Yaw       4134		
	#define Mech_Max_Yaw       1960     //left

	//云台抬头角度,用于摩擦轮开启
	#define CLOUD_FRIC_PIT_UP  (Mech_Min_Pitch + 200)
	
	//云台底盘分离角度,用于yaw限位
	#define CLOUD_SEPAR_ANGLE  800		//小心会跟扭头模式冲突
	float down_sb_pitch = 470;//430;
	float up_sb_pitch   = 0;//100;//
	
	//吊射角度
	float base_mech_pitch = 3965;//Mech_Mid_Pitch;
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
	//云台
	#define Mech_Min_Pitch     3400   //up
	#define Mech_Mid_Pitch     4100
	#define Mech_Max_Pitch     4430    //down
	#define Mech_Min_Yaw       6100     //right
	#define Mech_Mid_Yaw       4080
	#define Mech_Max_Yaw       2030     //left

	//云台抬头角度,用于摩擦轮开启
	#define CLOUD_FRIC_PIT_UP  (Mech_Min_Pitch + 200)
	
	//云台底盘分离角度,用于yaw限位
	#define CLOUD_SEPAR_ANGLE  800		//小心会跟扭头模式冲突
	float down_sb_pitch = 430;
	float up_sb_pitch   = 0;//100;//
	
	//吊射角度
	float base_mech_pitch = 3890;
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
	//云台
	#define Mech_Min_Pitch     3355   //up
	#define Mech_Mid_Pitch     4150
	#define Mech_Max_Pitch     4390    //down
	#define Mech_Min_Yaw       6170     //right
	#define Mech_Mid_Yaw       4093
	#define Mech_Max_Yaw       2020     //left

	//云台抬头角度,用于摩擦轮开启
	#define CLOUD_FRIC_PIT_UP  (Mech_Min_Pitch + 200)
	
	//云台底盘分离角度,用于yaw限位
	#define CLOUD_SEPAR_ANGLE  800		//小心会跟扭头模式冲突
	float down_sb_pitch = 530;
	float up_sb_pitch   = 0;//100;//
	
	//吊射角度
	float base_mech_pitch = 3960;
	
#endif
/*-----------------------------------------------------------*/


/*---------------------陀螺仪角速度补偿------------------------*/
#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
	#define PALST_COMPS_YAW        (38)
	#define PALST_COMPS_PITCH      (81)
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
	#define PALST_COMPS_YAW        (0)
	#define PALST_COMPS_PITCH      (58)
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
	#define PALST_COMPS_YAW        (10)
	#define PALST_COMPS_PITCH      (102)
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
	#define PALST_COMPS_YAW        (-5)
	#define PALST_COMPS_PITCH      (62)
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
	#define PALST_COMPS_YAW        (-10)
	#define PALST_COMPS_PITCH      (35)
	
#endif
/*--------------------------------------------------------------*/

//云台模式选择
typedef enum
{
	CLOUD_MECH_MODE = 0,
	CLOUD_GYRO_MODE = 1,
} eGimbalCtrlMode;
eGimbalCtrlMode  modeGimbal;


/* 云台操作模式:
   
   普通             	NORMAL
   调头180°             AROUND
   打符             	BUFF
   补弹,pitch水平   	LEVEL
   机械模式pitch抬头	HIGH
   快速扭头90°          TURN
*/
typedef enum
{
	GIMBAL_NORMAL  = 0,//正常模式,进行模式选择
	GIMBAL_AROUND  = 1,//180°调头
	GIMBAL_BUFF    = 2,//打符模式,大
	GIMBAL_LEVEL   = 3,//弹仓开启,云台水平
	GIMBAL_MANUAL  = 4,//手动打符模式
	GIMBAL_SM_BUFF = 5,//小符
	GIMBAL_TURN    = 7,//90°扭头
	GIMBAL_AUTO    = 8,//自瞄
	GIMBAL_BASE    = 9,//桥头吊射基地
	
}eGimbalAction;
eGimbalAction  actGimbal;

Critical_t Yaw_Gyro_Angle;

/*************灵敏度************/

//机械模式下比例系数,控制摇杆响应速度
float kRc_Mech_Pitch, kRc_Mech_Yaw;

//陀螺仪模式下比例系数,控制摇杆响应速度
float kRc_Gyro_Pitch, kRc_Gyro_Yaw;
float krc_gyro_yaw = 0.018;

//机械模式下比例系数,控制键盘响应速度
float kKey_Mech_Pitch, kKey_Mech_Yaw;

//陀螺仪模式下比例系数,控制键盘响应速度
float kKey_Gyro_Pitch, kKey_Gyro_Yaw;
float kkey_gyro_yaw = 0.38;

/*******************PID参数**********************/

//陀螺仪参数
float angleMpuPitch,	angleMpuYaw,	angleMpuRoll;//陀螺仪角度值
short palstanceMpuPitch,	palstanceMpuYaw,	palstanceMpuRoll;//陀螺仪角速度值

//机械角度中间变量,从CAN中读取数据
int16_t  angleMotorPit,  angleMotorYaw; 

//角度误差
float Cloud_Angle_Error[2][2];//  pitch/yaw    mech/gyro
float Cloud_Palstance_Error[2][2];//  pitch/yaw    mech/gyro

//角速度误差累加和
float Cloud_Palstance_Error_Sum[2][2];//  pitch/yaw    mech/gyro

//期望角度
float Cloud_Angle_Target[2][2];//  pitch/yaw    mech/gyro

//测量角度
float Cloud_Angle_Measure[2][2];//  pitch/yaw    mech/gyro

//测量角速度
float Cloud_Palstance_Measure[2][2];//  pitch/yaw    mech/gyro

//PID参数
float Cloud_Angle_kpid[2][2][3];//  pitch/yaw    mech/gyro    kp/ki/kd
float Cloud_Palstance_kpid[2][2][3];


//PID
float  pTermPit[2],       pTermYaw[2][2];//   outer/inner    outer/inner//mech/gyro
float  iTermPit[2],       iTermYaw[2][2];
float  pidTermPit[2],     pidTermYaw[2][2];

/**************限幅****************/
//限制云台电机电流最大输出量
float PID_Out_Max;
float PID_Outter_Max;
float PID_Iterm_Max;


/**************斜坡***************/
float Slope_Mouse_Pitch, Slope_Mouse_Yaw;//摩擦轮开启时抬头快慢
	
//上电斜坡变量
float Slope_Begin_Pitch, Slope_Begin_Yaw;//刚上电时移动快慢

//键盘陀螺仪模式下鼠标统计yaw偏移量,此值会自己缓慢减小,防止甩头过快
float Mouse_Gyro_Yaw, Mouse_Gyro_Pitch;

//键盘陀螺仪模式下QEC扭头快慢
float Slope_Turn_Yaw;
float Slope_Back_Yaw;

//开启摩擦轮抬头斜坡
float Slope_Fric_Pitch;

//自瞄斜坡
float Slope_Auto_Yaw;
float Slope_Auto_Pitch;

   
/****************云台键盘模式下各小函数辅助变量********************/
//调头模式角度目标
float TURNMode_Yaw_Back_Total;//按下C,yaw需要改变的角度值
float TURNMode_Yaw_Turn_Total;//按下QE,yaw需要改变的角度值,正负代表左右转


/**********************手打大符*******************/
#define CONFIRM_BEGIN		0//刚进入手动打符，随便确认位置
#define CONFIRM_CENTRE		1//确认圆心
#define CONFIRM_RADIUS		2//确认半径
#define CONFIRM_HIGH		3//确认高度
#define CONFIRM_LOCATION	4//确认位置
int Manual_Step = CONFIRM_BEGIN;//第一步确定圆心，第二步确定半径，第三步WASD确定位置
float Manual_Pitch_Comp = 70;//手动抬头补偿
float Buff_Pitch_Comp;//打符抬头自动补偿
float Buff_Yaw_Comp;//打符红外左右自动补偿，因为红外可能装歪
float Buff_Pitch_Comp_Gimbal;//打符抬头补偿,摄像头在云台
float Buff_Yaw_Comp_Gimbal;

float Base_Yaw_Comp_Gimbal;//吊射左右校准

//打符校正镜头畸变
float Buff_Pitch_Correct_Chassis;
float Buff_Yaw_Correct_Chassis;
float Buff_Pitch_Correct_Gimbal;
float Buff_Yaw_Correct_Gimbal;

float debug_y_mid;// = 5785;//5797;//视觉调坐标转换
float debug_p_mid;// = 3450;//3522;//7.15m抬头差72机械角度,飓风电机,600pwm

float gb_yaw_posit_error = 0;//位置差，用来判断是否移动到位
float gb_pitch_posit_error = 0;//位置差，用来判断是否移动到位

//刚进入打符判断
bool is_firstime_into_buff = TRUE;

/***************自瞄******************/

//误差
float Auto_Error_Yaw[2];//    now/last
float Auto_Error_Pitch[2];
float Auto_Distance;//距离单目

//基地吊射自瞄
float Base_Error_Yaw;
bool first_time_into_base = TRUE;

//自瞄突然开启,卡尔曼滤波开启延时
uint16_t Auto_KF_Delay = 0;


float debug_y_sk;// = 38;//35;//30;//移动预测系数,越大预测越多
float debug_y_sb_sk;//哨兵预测系数
float debug_y_sb_brig_sk;//桥头哨兵
float debug_p_sk;//移动预测系数,越大预测越多
float debug_auto_err_y;// = 10;//15;//10;//15;//yaw角度过大关闭预测
float debug_auto_err_p;//pitch角度过大关闭预测
float debug_kf_delay;// = 150;//100;//200;//120;//150;//预测延时开启
float debug_kf_speed_yl;//yaw速度过低关闭预测
float debug_kf_speed_yl_sb;//抬头打哨兵时减小最低可开预测量
float debug_kf_speed_yh;//yaw速度过高关闭预测
float debug_kf_speed_pl;//pitch速度过低关闭预测
float debug_kf_y_angcon;// = 130;//125;//115;//135;//yaw预测量限幅
float debug_kf_p_angcon;//pitch预测量限幅


float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//卡尔曼滤波速度测量值
float *yaw_kf_result, *pitch_kf_result;//二阶卡尔曼滤波结果,0角度 1速度

/*************卡尔曼滤波**************/
/*一阶卡尔曼*/
//云台角度误差卡尔曼
extKalman_t Gimbal_Pitch_Mech_Error_Kalman;//定义一个kalman指针
extKalman_t Gimbal_Pitch_Gyro_Error_Kalman;//定义一个kalman指针
extKalman_t Gimbal_Yaw_Mech_Error_Kalman;//定义一个kalman指针
extKalman_t Gimbal_Yaw_Gyro_Error_Kalman;//定义一个kalman指针

extKalman_t Vision_Distance_Kalman;

extKalman_t Gimbal_Buff_Yaw_Error_Kalman;//底盘打符
extKalman_t Gimbal_Buff_Pitch_Error_Kalman;//

extKalman_t Gimbal_Buff_Yaw_Error_Gim_Kalman;//云台打符
extKalman_t Gimbal_Buff_Pitch_Error_Gim_Kalman;//


/*二阶卡尔曼*/
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2

speed_calc_data_t Vision_Yaw_speed_Struct;
speed_calc_data_t Vision_Pitch_speed_Struct;

kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}//500 1000
};//初始化yaw的部分kalman参数

kalman_filter_init_t pitch_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}
};//初始化pitch的部分kalman参数

kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;

/*自动打弹用的一些标志位*/
bool Mobility_Prediction_Yaw = FALSE;//预测是否开启标志位
bool Mobi_Pre_Yaw_Fire = FALSE;//默认预测没到位，禁止开枪

uint16_t mobpre_yaw_left_delay = 0;//向左预测延时判断可开火消抖
uint16_t mobpre_yaw_right_delay = 0;//向右预测延时判断可开火消抖
uint16_t mobpre_yaw_stop_delay = 0;//预测关闭延时判断可开火消抖
/********************************************************************************/
/********************************************************************************/

//每2ms执行一次任务函数
void Task_Gimbal(void *pvParameters)
{
	portTickType currentTime;	
	
	for(;;)
	{	
		currentTime = xTaskGetTickCount();//当前系统时间
		
		/* 代码段 */
		if (SYSTEM_GetSystemState() == SYSTEM_STARTING)//初始化模式
		{
			  GIMBAL_InitCtrl();
		}
		else
		{
			if (SYSTEM_GetRemoteMode() == RC)//遥控控制模式
			{
				GIMBAL_Rc_Ctrl();//遥控模式
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				GIMBAL_Key_Ctrl();//键盘模式
			}
		}
		
		if(modeGimbal == CLOUD_MECH_MODE)
		{
			iTermYaw[INNER][GYRO] = 0;
		}
		else
		{
			iTermYaw[INNER][MECH] = 0;
		}
		
		GIMBAL_kPID_Init();//根据操作模式变换kpid,每次都要变
		
		GIMBAL_PositionLoop();
		GIMBAL_CanbusCtrlMotors();
		
		if(actGimbal != GIMBAL_AUTO)
		{
			//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
			Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[YAW][GYRO]);
			Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[PITCH][MECH]);
			//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
			yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
			pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
		}
		
		if(VisionRecvData.distance == 999.99f)
		{
			Orange_On;
		}
		else
		{
			Orange_Off;
		}
		
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//绝对延时
	}
}


/**
  * @brief  云台参数初始化
  * @param  void
  * @retval void
  * @attention 没有加I会有误差,只在系统启动时调用一次
  */
void GIMBAL_InitArgument(void)
{
	/* 灵敏度,响应快慢 */	
	kRc_Mech_Yaw   = 0.01;
	kRc_Mech_Pitch = 0.01;
	
	kRc_Gyro_Yaw   = krc_gyro_yaw;//0.015;//调节陀螺仪模式下转头速度快慢，影响陀螺仪模式下车扭头速度
	kRc_Gyro_Pitch = 0.01;
	
	kKey_Mech_Yaw   = 0;
	kKey_Mech_Pitch = 0.45;
	
	kKey_Gyro_Yaw   = -kkey_gyro_yaw;//-0.38;//注意正负,否则会反向
	kKey_Gyro_Pitch = 0.38;//0.45;
	
	/* 斜坡,变化快慢 */
	Slope_Begin_Pitch =  1;//上电时移动快慢
  	Slope_Begin_Yaw   =  1;//上电时移动快慢
	
	Slope_Mouse_Pitch = 15;//20;//鼠标响应,抬头低头速度
	Slope_Mouse_Yaw   = 15;//鼠标响应,扭头速度
	
	Slope_Turn_Yaw = 20;//25;//QE扭头快慢
	Slope_Back_Yaw = 20;//30;//C调头快慢
	
	Slope_Auto_Yaw   = 4;//1;//50;
	Slope_Auto_Pitch = 3;//1;//50;
	
	Slope_Fric_Pitch = 8;
	
	/* 限幅 */
	#if YAW_POSITION == YAW_UP 
		PID_Out_Max    = 4999;//限制云台电机电流最大输出量
		PID_Outter_Max = 6000;
		PID_Iterm_Max  = 3000;
	#else
		PID_Out_Max    = 29999;
		PID_Outter_Max = 29999;
		PID_Iterm_Max  = 18000;
	#endif
		
	/* 键盘模式选择,默认正常模式 */
	actGimbal = GIMBAL_NORMAL;
	GIMBAL_kPID_Init();//PID参数初始化
	
	/*--------------自瞄角度补偿初始化----------------*/
	#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
		
		debug_y_sk = 14.8;//80;//72;//80;//移动预测系数,越大预测越多
		debug_y_sb_sk = 10;//105;//120;//140;//
		debug_p_sk = 75;//26;//移动预测系数,越大预测越多
		debug_auto_err_y = 150;//25;//角度过大关闭预测
		debug_auto_err_p = 8;
		debug_kf_delay = 50;//40;//200;//220;//预测延时开启
		debug_kf_speed_yl = 0.35;//0.35;//0.45;//速度过低关闭预测
		debug_kf_speed_yl_sb = 0.3;//
		debug_kf_speed_yh = 25;//4;//6.5;//速度过高关闭预测
		debug_kf_speed_pl = 0.05;//0.25;//0.08;//pitch速度过低关闭预测
		debug_kf_y_angcon = 220;//预测量限幅
		debug_kf_p_angcon = 120;//45;//pitch预测量限幅
		
		//打符,底盘
		debug_y_mid     	= 4050;//Mech_Mid_Yaw;
		debug_p_mid     	= 3172;//Mech_Mid_Pitch;		
		Buff_Pitch_Comp 	= 0;//-30;//-33;//-47;//-49;//-59;//-53;//射速25.5~26.5
		Buff_Yaw_Comp   	= 0;//-10;//10;//-7;//-12;//-28;//-42;//-45;//-38;//-23;
		Buff_Pitch_Comp_Gimbal = 50;//云台打符校准
		Buff_Yaw_Comp_Gimbal   = 45;//云台打符校准
		Buff_Pitch_Correct_Chassis  = 1;//0.93;
		Buff_Yaw_Correct_Chassis 	= 1;//0.97;
		Buff_Pitch_Correct_Gimbal	= 1;
		Buff_Yaw_Correct_Gimbal		= 1;
		
		Base_Yaw_Comp_Gimbal = 0;
		
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
		
		debug_y_sk = 88;//38;//35;//30;//移动预测系数,越大预测越多
		debug_y_sb_sk = 10;
		debug_p_sk = 26;//移动预测系数,越大预测越多
		debug_auto_err_y = 16;//15;//10;//15;//角度过大关闭预测
		debug_auto_err_p = 8;
		debug_kf_delay = 180;//100;//200;//120;//150;//预测延时开启
		debug_kf_speed_yl = 0.38;//0.1;//0.2;//0.1;//0.08;//0.1;//0.3;//速度过低关闭预测
		debug_kf_speed_yh = 6;//速度过高关闭预测
		debug_kf_speed_yl_sb = 0.1;//
		debug_kf_speed_pl = 0.1;//pitch速度过低关闭预测
		debug_kf_y_angcon = 260;//125;//115;//135;//预测量限幅
		debug_kf_p_angcon = 45;//pitch预测量限幅
		
		//打符
		//底盘
		debug_y_mid 		= Mech_Mid_Yaw;
		debug_p_mid 		= Mech_Mid_Pitch;
		Buff_Pitch_Comp 	= 0;
		Buff_Yaw_Comp   	= 0;
		
		//云台
		Buff_Pitch_Comp_Gimbal = 0;
		Buff_Yaw_Comp_Gimbal   = 0;

		Buff_Pitch_Correct_Chassis  = 1;
		Buff_Yaw_Correct_Chassis 	= 1;
		Buff_Pitch_Correct_Gimbal	= 1;
		Buff_Yaw_Correct_Gimbal		= 1;
		
		Base_Yaw_Comp_Gimbal = 0;
		
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
		
		debug_y_sk = 52;//45;//14.8;//移动预测系数,越大预测越多
		debug_y_sb_sk = 62;//61;//55;
		debug_y_sb_brig_sk = 88;//
		debug_p_sk = 20;//移动预测系数,越大预测越多
		debug_auto_err_y = 120;//角度过大关闭预测
		debug_auto_err_p = 150;
		debug_kf_delay = 80;//预测延时开启
		debug_kf_speed_yl = 0.2;//0.35;//速度过低关闭预测
		debug_kf_speed_yl_sb = 0.2;//
		debug_kf_speed_yh = 4.2;//速度过高关闭预测
		debug_kf_speed_pl = 0.15;//pitch速度过低关闭预测
		debug_kf_y_angcon = 220;//125;//115;//135;//预测量限幅
		debug_kf_p_angcon = 45;//pitch预测量限幅
		
		//打符
		//底盘
		debug_y_mid 		= 4122;//5883;
		debug_p_mid 		= 3860;//3565;
		Buff_Pitch_Comp 	= 0;
		Buff_Yaw_Comp   	= 0;
		
		//云台
		//7.1米	
		Buff_Pitch_Comp_Gimbal = -78;//-88;//家里-88，单项赛-96	
		//8米
		//Buff_Pitch_Comp_Gimbal = -86;//-96;
		Buff_Yaw_Comp_Gimbal   = -8;			//28m/s

		Buff_Pitch_Correct_Chassis  = 1;
		Buff_Yaw_Correct_Chassis 	= 0.99;
		Buff_Pitch_Correct_Gimbal	= 1;
		Buff_Yaw_Correct_Gimbal		= 1;
		
		Base_Yaw_Comp_Gimbal = -20;
		
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
		
		debug_y_sk = 45;//14.8;//移动预测系数,越大预测越多
		debug_y_sb_sk = 55;
		debug_y_sb_brig_sk = 90;//
		debug_p_sk = 20;//移动预测系数,越大预测越多
		debug_auto_err_y = 120;//角度过大关闭预测
		debug_auto_err_p = 150;
		debug_kf_delay = 80;//预测延时开启
		debug_kf_speed_yl = 0.2;//速度过低关闭预测
		debug_kf_speed_yl_sb = 0.2;//
		debug_kf_speed_yh = 4;//速度过高关闭预测
		debug_kf_speed_pl = 0.15;//pitch速度过低关闭预测
		debug_kf_y_angcon = 220;//125;//115;//135;//预测量限幅
		debug_kf_p_angcon = 45;//pitch预测量限幅
		
		//打符
		//底盘
		debug_y_mid 		= Mech_Mid_Yaw;
		debug_p_mid 		= Mech_Mid_Pitch;
		Buff_Pitch_Comp 	= 0;
		Buff_Yaw_Comp   	= 0;
		
		//云台
		//7.1米
		Buff_Pitch_Comp_Gimbal = -22;//-30;//-24;//-20;//-10;//家里-24，单项赛-30		
		//8米
		//Buff_Pitch_Comp_Gimbal = -30;
		
		Buff_Yaw_Comp_Gimbal   = -13;//19;

		Buff_Pitch_Correct_Chassis  = 1;
		Buff_Yaw_Correct_Chassis 	= 1;
		Buff_Pitch_Correct_Gimbal	= 1;
		Buff_Yaw_Correct_Gimbal		= 1;
		
		Base_Yaw_Comp_Gimbal = -13;
		
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
		
		debug_y_sk = 50;//45;//35;//14.8;//移动预测系数,越大预测越多
		debug_y_sb_sk = 59;//55;
		debug_y_sb_brig_sk = 90;//
		debug_p_sk = 20;//移动预测系数,越大预测越多
		debug_auto_err_y = 120;//角度过大关闭预测
		debug_auto_err_p = 150;
		debug_kf_delay = 80;//预测延时开启
		debug_kf_speed_yl = 0.2;//0.35;//速度过低关闭预测
		debug_kf_speed_yl_sb = 0.2;//0.2;//
		debug_kf_speed_yh = 5;//速度过高关闭预测
		debug_kf_speed_pl = 0.15;//pitch速度过低关闭预测
		debug_kf_y_angcon = 220;//125;//115;//135;//预测量限幅
		debug_kf_p_angcon = 45;//pitch预测量限幅
		
		//打符
		//底盘
		debug_y_mid 		= Mech_Mid_Yaw;
		debug_p_mid 		= Mech_Mid_Pitch;
		Buff_Pitch_Comp 	= 0;
		Buff_Yaw_Comp   	= 0;
		
		//云台
		//7.1米
		Buff_Pitch_Comp_Gimbal = -54;//-52;//-25;//家里-52，单项赛-60		
		//8米
		//Buff_Pitch_Comp_Gimbal = -63;
		
		Buff_Yaw_Comp_Gimbal   = -27;//-30;

		Buff_Pitch_Correct_Chassis  = 1;
		Buff_Yaw_Correct_Chassis 	= 1;
		Buff_Pitch_Correct_Gimbal	= 1;
		Buff_Yaw_Correct_Gimbal		= 1;
		
		Base_Yaw_Comp_Gimbal = -27;//-30;
		
	#endif	
	
	//卡尔曼滤波器初始化
	  /*PID角度误差卡尔曼,一阶*/
	KalmanCreate(&Gimbal_Pitch_Mech_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Pitch_Gyro_Error_Kalman, 1, 40);
	
	KalmanCreate(&Gimbal_Yaw_Mech_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Yaw_Gyro_Error_Kalman, 1, 40);
	
	KalmanCreate(&Vision_Distance_Kalman, 1, 2000);
	
	
	KalmanCreate(&Gimbal_Buff_Yaw_Error_Kalman, 1, 0);//底盘打符
	KalmanCreate(&Gimbal_Buff_Pitch_Error_Kalman, 1, 0);//
	
	KalmanCreate(&Gimbal_Buff_Yaw_Error_Gim_Kalman, 1, 0);//云台打符
	KalmanCreate(&Gimbal_Buff_Pitch_Error_Gim_Kalman, 1, 0);//
	
	  /*自瞄卡尔曼滤波,二阶*/
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
	
	mat_init(&pitch_kalman_filter.Q,2,2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R,2,2, pitch_kalman_filter_para.R_data);
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);

}

/**
  * @brief  云台pid参数初始化
  * @param  void
  * @retval void
  * @attention 打符/自瞄模式和普通模式是不一样的,打符/自瞄要更硬
  *            还没加打符模式的kpid
  */
/*- 自瞄 -*/
float  v_y_k = 9/*7.2*//*6*/,  v_y_p = 23/*9*//*8*/,  v_y_i = 800/*25*/;
float gv_p_k = 8/*7.7*//*16*/, gv_p_p = 18/*5.2*/, gv_p_i = 400;
/*- 打符 -*/
float gb_y_k = 3.1/*11*/, gb_y_p = 26/*7.8*/, gb_y_i = 600/*70*/;//外kp 内kp 内ki
float gb_p_k = 3.1/*14*/, gb_p_p = 23/*5*/, gb_p_i = 500/*50*/;
void GIMBAL_kPID_Init(void)
{
	//如果为自瞄模式且找到了目标
	if( actGimbal ==  GIMBAL_AUTO && VisionRecvData.identify_target == TRUE )
	{
		/*-------------云台PID参数预编译---------------*/
		#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
			/* kPID,机械模式 */
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP]=gv_p_k;//7;//7;//7.5;//gv_p_k;//10;
			
			Cloud_Angle_kpid[YAW][MECH][KP]=20;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP]=gv_p_p;//3.7;//4.5;//gv_p_p;//4;
			Cloud_Palstance_kpid[PITCH][MECH][KI]=gv_p_i;//70;//100;//gv_p_i;//80;
			Cloud_Palstance_kpid[PITCH][MECH][KD]=0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]=2.2;
			Cloud_Palstance_kpid[YAW][MECH][KI]=0;
			Cloud_Palstance_kpid[YAW][MECH][KD]=0;
			
			/* kPID,陀螺仪模式 */
				//outer
			Cloud_Angle_kpid[YAW][GYRO][KP]=v_y_k;//6;//6.5;//v_y_k;//8;
			
				//inner		
			Cloud_Palstance_kpid[YAW][GYRO][KP]=v_y_p;//8;//v_y_p;//5;
			Cloud_Palstance_kpid[YAW][GYRO][KI]=v_y_i;//100;//v_y_i;//50;
			Cloud_Palstance_kpid[YAW][GYRO][KD]=0;

		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
			/* kPID,机械模式 */
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP] = 9;//10;//gv_p_k;//16;
			
			Cloud_Angle_kpid[YAW][MECH][KP]   = 0;//13;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP] = 4.2;//4.8;//gv_p_p;//2.2;
			Cloud_Palstance_kpid[PITCH][MECH][KI] = 65;//75;//gv_p_i;//0.08;
			Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]   = 0;//2.2;
			Cloud_Palstance_kpid[YAW][MECH][KI]   = 0;//0.2;
			Cloud_Palstance_kpid[YAW][MECH][KD]   = 0;
			
			/* kPID,陀螺仪模式 */
				//outer	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 5;//v_y_k;//0;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 9;//v_y_p;//0;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 50;//v_y_i;//0;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;

		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
			/* kPID,机械模式 */
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP] = 9.1;//11;//gv_p_k;//7.5;//gv_p_k;//16;
			
			Cloud_Angle_kpid[YAW][MECH][KP]   = 10;//13;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP] = 14;//18;//gv_p_p;//4.5;//gv_p_p;//6;
			Cloud_Palstance_kpid[PITCH][MECH][KI] = 300;//gv_p_i;//100;//gv_p_i;//1;
			Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]   = 6;//2.2;
			Cloud_Palstance_kpid[YAW][MECH][KI]   = 1.2;//0.2;
			Cloud_Palstance_kpid[YAW][MECH][KD]   = 0;
			
			/* kPID,陀螺仪模式 */
				//outer	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 8.6;//9.2;//12;//v_y_k;//6.5;//v_y_k;//17;//gv_y_k;//14;//10;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 16;//19;//22;//26;//v_y_p;//8;//v_y_p;//2.9;//gv_y_p;//2.6;//2.2;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 400;//600;//v_y_i;//100;//v_y_i;//0.8;//gv_y_i;//0.1;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;

		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
			/* kPID,机械模式 */
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP] = 9.1;//gv_p_k;//0;//16;
			
			Cloud_Angle_kpid[YAW][MECH][KP]   = 0;//v_y_k;//0;//13;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP] = 15.5;//gv_p_p;//0;//2.2;
			Cloud_Palstance_kpid[PITCH][MECH][KI] = 400;//600;//gv_p_i;//0;//0.08;
			Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]   = 0;//2.2;
			Cloud_Palstance_kpid[YAW][MECH][KI]   = 0;//0.2;
			Cloud_Palstance_kpid[YAW][MECH][KD]   = 0;
			
			/* kPID,陀螺仪模式 */
				//outer	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 9.2;//v_y_k;//14;//10;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 15;//25;//v_y_p;//2.6;//2.2;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 500;//700;//v_y_i;//0.1;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;

		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
			/* kPID,机械模式 */
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP] = 8.1;//gv_p_k;//0;//16;
			
			Cloud_Angle_kpid[YAW][MECH][KP]   = 0;//13;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP] = 15;//20;//gv_p_p;//0;//2.2;
			Cloud_Palstance_kpid[PITCH][MECH][KI] = 400;//450;//gv_p_i;//0;//0.08;
			Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]   = 0;//2.2;
			Cloud_Palstance_kpid[YAW][MECH][KI]   = 0;//0.2;
			Cloud_Palstance_kpid[YAW][MECH][KD]   = 0;
			
			/* kPID,陀螺仪模式 */
				//outer	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 9.2;//v_y_k;//0;//14;//10;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 15;//21;//v_y_p;//0;//2.6;//2.2;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 400;//v_y_i;//0;//0.1;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;

		#endif
		/*---------------------------------------------*/
	}
	else if( (GIMBAL_IfBuffHit() == TRUE && VisionRecvData.identify_buff == TRUE) )//打符模式，只有机械
	{
		/*-------------云台PID参数预编译---------------*/
		#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
			/* kPID,机械模式 */
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP]=gb_p_k;//14;//gb_p_k;//16;//gb_p_k;//15;
			
			Cloud_Angle_kpid[YAW][MECH][KP]=11;//gb_y_k;//12;//13;//gb_y_k;//15;
		
			Cloud_Angle_kpid[YAW][GYRO][KP]=gb_y_k;//12;//13;//gb_y_k;//15;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP]=gb_p_p;//5;//gb_p_p;//4;//3;//gb_p_p;//3;//3.3;
			Cloud_Palstance_kpid[PITCH][MECH][KI]=gb_p_i;//50;//gb_p_i;//70;//30;//gb_p_i;//20;//0.1;
			Cloud_Palstance_kpid[PITCH][MECH][KD]=0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]=7.8;//gb_y_p;//8;//gb_y_p;//3;
			Cloud_Palstance_kpid[YAW][MECH][KI]=70;//gb_y_i;//125;//50;//gb_y_i;//5;
			Cloud_Palstance_kpid[YAW][MECH][KD]=0;

			Cloud_Palstance_kpid[YAW][GYRO][KP]=gb_y_p;//8;//gb_y_p;//3;
			Cloud_Palstance_kpid[YAW][GYRO][KI]=gb_y_i;//125;//50;//gb_y_i;//5;
			Cloud_Palstance_kpid[YAW][GYRO][KD]=0;
			
		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
			/* kPID,机械模式 */
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP] = 11;//12;//16;
			
			Cloud_Angle_kpid[YAW][MECH][KP]   = 10.5;//11;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP] = 4.8;//5;//2.2;
			Cloud_Palstance_kpid[PITCH][MECH][KI] = 70;//75;//0.08;
			Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]   = 5.4;//5.8;
			Cloud_Palstance_kpid[YAW][MECH][KI]   = 85;//100;
			Cloud_Palstance_kpid[YAW][MECH][KD]   = 0;
			
			Cloud_Palstance_kpid[YAW][GYRO][KP]=gb_y_p;//8;//gb_y_p;//3;
			Cloud_Palstance_kpid[YAW][GYRO][KI]=gb_y_i;//125;//50;//gb_y_i;//5;
			Cloud_Palstance_kpid[YAW][GYRO][KD]=0;
				
		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
		
			#if 	BUFF_CAM_TYPE == BUFF_CAM_CHAS
					//outer
				Cloud_Angle_kpid[PITCH][MECH][KP] = 10;//gb_p_k;//15;				
				Cloud_Angle_kpid[YAW][MECH][KP]   = 12;//gb_y_k;//10;
				
					//inner
				Cloud_Palstance_kpid[PITCH][MECH][KP] = 12;//gb_p_p;//5;
				Cloud_Palstance_kpid[PITCH][MECH][KI] = 350;//gb_p_i;//40;
				Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;
				
				Cloud_Palstance_kpid[YAW][MECH][KP]   = 24;//gb_y_p;//4;
				Cloud_Palstance_kpid[YAW][MECH][KI]   = 200;//gb_y_i;//3;
				Cloud_Palstance_kpid[YAW][MECH][KD]   = 0;

			#elif 	BUFF_CAM_TYPE == BUFF_CAM_GIMB
					//outer
				Cloud_Angle_kpid[PITCH][MECH][KP] = 8;//5.5;//gb_p_k;			
				Cloud_Angle_kpid[YAW][GYRO][KP]   = 8;//5;//gb_y_k;
				
					//inner
				Cloud_Palstance_kpid[PITCH][MECH][KP] = 20;//22;//gb_p_p;
				Cloud_Palstance_kpid[PITCH][MECH][KI] = 300;//500;//gb_p_i;
				Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;

				Cloud_Palstance_kpid[YAW][GYRO][KP]   = 22;//gb_y_p;
				Cloud_Palstance_kpid[YAW][GYRO][KI]   = 300;//500;//gb_y_i;
				Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;
			
			#endif

		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
			#if 	BUFF_CAM_TYPE == BUFF_CAM_CHAS
					//outer
				Cloud_Angle_kpid[PITCH][MECH][KP] = 10;//gb_p_k;//15;				
				Cloud_Angle_kpid[YAW][MECH][KP]   = 12;//gb_y_k;//10;
				
					//inner
				Cloud_Palstance_kpid[PITCH][MECH][KP] = 12;//gb_p_p;//5;
				Cloud_Palstance_kpid[PITCH][MECH][KI] = 350;//gb_p_i;//40;
				Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;
				
				Cloud_Palstance_kpid[YAW][MECH][KP]   = 24;//gb_y_p;//4;
				Cloud_Palstance_kpid[YAW][MECH][KI]   = 200;//gb_y_i;//3;
				Cloud_Palstance_kpid[YAW][MECH][KD]   = 0;

			#elif 	BUFF_CAM_TYPE == BUFF_CAM_GIMB
					//outer
				Cloud_Angle_kpid[PITCH][MECH][KP] = 9;//gb_p_k;			
				Cloud_Angle_kpid[YAW][GYRO][KP]   = 9;//gb_y_k;
				
					//inner
				Cloud_Palstance_kpid[PITCH][MECH][KP] = 16;//gb_p_p;
				Cloud_Palstance_kpid[PITCH][MECH][KI] = 500;//gb_p_i;
				Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;

				Cloud_Palstance_kpid[YAW][GYRO][KP]   = 20;//gb_y_p;
				Cloud_Palstance_kpid[YAW][GYRO][KI]   = 500;//gb_y_i;
				Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;

			#endif

		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
			#if 	BUFF_CAM_TYPE == BUFF_CAM_CHAS
					//outer
				Cloud_Angle_kpid[PITCH][MECH][KP] = 10;//gb_p_k;//15;				
				Cloud_Angle_kpid[YAW][MECH][KP]   = 12;//gb_y_k;//10;
				
					//inner
				Cloud_Palstance_kpid[PITCH][MECH][KP] = 12;//gb_p_p;//5;
				Cloud_Palstance_kpid[PITCH][MECH][KI] = 350;//gb_p_i;//40;
				Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;
				
				Cloud_Palstance_kpid[YAW][MECH][KP]   = 24;//gb_y_p;//4;
				Cloud_Palstance_kpid[YAW][MECH][KI]   = 200;//gb_y_i;//3;
				Cloud_Palstance_kpid[YAW][MECH][KD]   = 0;

			#elif 	BUFF_CAM_TYPE == BUFF_CAM_GIMB
					//outer
				Cloud_Angle_kpid[PITCH][MECH][KP] = 9;//9;//gb_p_k;			
				Cloud_Angle_kpid[YAW][GYRO][KP]   = 9;//gb_y_k;
				
					//inner
				Cloud_Palstance_kpid[PITCH][MECH][KP] = 13;//gb_p_p;
				Cloud_Palstance_kpid[PITCH][MECH][KI] = 300;//gb_p_i;
				Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;

				Cloud_Palstance_kpid[YAW][GYRO][KP]   = 20;//gb_y_p;
				Cloud_Palstance_kpid[YAW][GYRO][KI]   = 300;//gb_y_i;
				Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;

			#endif

		#endif
		/*---------------------------------------------*/
	}
	else if( GIMBAL_If_Base() == TRUE && VisionRecvData.identify_target == 8 )
	{
		/*-------------云台PID参数预编译---------------*/
		#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO		
			/* kPID,陀螺仪模式 */
				//outer	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 4.8;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 20;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 200;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;

		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE			
			/* kPID,陀螺仪模式 */
				//outer	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 4.8;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 20;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 200;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;
				
		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
			/* kPID,陀螺仪模式 */
				//outer0	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 4;//4.8;//8;//11;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 18;//20;//12;//38;//5.5;//6;//5;//2.6;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 300;//0;//400;//800;//180;//200;//2;//0.6;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;
			
		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
			/* kPID,陀螺仪模式 */
				//outer	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 4.8;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 20;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 200;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;

		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR			
			/* kPID,陀螺仪模式 */
				//outer	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 4.8;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 20;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 200;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;

		#endif
		/*---------------------------------------------*/
	}  
	//除了自瞄与打符模式的情况
	else
	{
		/*-------------云台PID参数预编译---------------*/
		#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
			/* kPID,机械模式 */ 
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP]=10;//12;//15;//16;//16;//16;
			
			Cloud_Angle_kpid[YAW][MECH][KP]=11;//10;//10;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP]=2.5;//3.8;//4.1;//3;//2.2;
			Cloud_Palstance_kpid[PITCH][MECH][KI]=50;//75;//100;//40;//0.5;//0.08;
			Cloud_Palstance_kpid[PITCH][MECH][KD]=0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]=5.5;//2.5;//2.5;
			Cloud_Palstance_kpid[YAW][MECH][KI]=150;//70;//0.21;//0.2;
			Cloud_Palstance_kpid[YAW][MECH][KD]=0;
			
			/* kPID,陀螺仪模式 */
				//outer		
			Cloud_Angle_kpid[YAW][GYRO][KP]=9.5;
			
				//inner		
			Cloud_Palstance_kpid[YAW][GYRO][KP]=8.5;//9;
			Cloud_Palstance_kpid[YAW][GYRO][KI]=50;//100;//150;//75;
			Cloud_Palstance_kpid[YAW][GYRO][KD]=0;

		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
			/* kPID,机械模式 */
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP] = 10;//16;
			
			Cloud_Angle_kpid[YAW][MECH][KP]   = 10;//13;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP] = 3.2;//2.2;
			Cloud_Palstance_kpid[PITCH][MECH][KI] = 60;//0.08;
			Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]   = 5;//2.2;
			Cloud_Palstance_kpid[YAW][MECH][KI]   = 100;//0.2;
			Cloud_Palstance_kpid[YAW][MECH][KD]   = 0;
			
			/* kPID,陀螺仪模式 */
				//outer	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 9.5;//10;//14;//10;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 8.5;//9;//2.6;//2.2;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 50;//35;//0.1;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;
				
		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
			/* kPID,机械模式 */
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP] = 12;
			
			Cloud_Angle_kpid[YAW][MECH][KP]   = 12;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP] = 14;//3.5;//4.2;//4;//2.2;
			Cloud_Palstance_kpid[PITCH][MECH][KI] = 300;//70;//100;//70;//0.05;
			Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]   = 24;
			Cloud_Palstance_kpid[YAW][MECH][KI]   = 300;//1.2;//0.2;
			Cloud_Palstance_kpid[YAW][MECH][KD]   = 0;
			
			/* kPID,陀螺仪模式 */
				//outer0	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 9;//8;//11;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 26;//38;//5.5;//6;//5;//2.6;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 600;//800;//180;//200;//2;//0.6;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;
			
		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
			/* kPID,机械模式 */
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP] = 12;
			
			Cloud_Angle_kpid[YAW][MECH][KP]   = 11;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP] = 12;
			Cloud_Palstance_kpid[PITCH][MECH][KI] = 300;
			Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]   = 23;
			Cloud_Palstance_kpid[YAW][MECH][KI]   = 400;
			Cloud_Palstance_kpid[YAW][MECH][KD]   = 0;
			
			/* kPID,陀螺仪模式 */
				//outer	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 9;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 24;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 600;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;

		#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
			/* kPID,机械模式 */
				//outer
			Cloud_Angle_kpid[PITCH][MECH][KP] = 11;//16;
			
			Cloud_Angle_kpid[YAW][MECH][KP]   = 11;//13;
			
				//inner
			Cloud_Palstance_kpid[PITCH][MECH][KP] = 11;//2.2;
			Cloud_Palstance_kpid[PITCH][MECH][KI] = 350;//0.08;
			Cloud_Palstance_kpid[PITCH][MECH][KD] = 0;
			
			Cloud_Palstance_kpid[YAW][MECH][KP]   = 20;//2.2;
			Cloud_Palstance_kpid[YAW][MECH][KI]   = 500;//250;//0.2;
			Cloud_Palstance_kpid[YAW][MECH][KD]   = 0;
			
			/* kPID,陀螺仪模式 */
				//outer	
			Cloud_Angle_kpid[YAW][GYRO][KP]   = 9;//14;//10;
			
				//inner	
			Cloud_Palstance_kpid[YAW][GYRO][KP]   = 20;//22;//2.6;//2.2;
			Cloud_Palstance_kpid[YAW][GYRO][KI]   = 500;//300;//0.1;
			Cloud_Palstance_kpid[YAW][GYRO][KD]   = 0;

		#endif
		/*---------------------------------------------*/
	}
}

/**
  * @brief  云台失控保护
  * @param  void
  * @retval void
  * @attention 所有输出置0
  */
void GIMBAL_StopMotor(void)
{
	float fMotorOutput[2] = {0};
		
	//外环输出置0
	pidTermPit[INNER] 		= 0;
	pidTermYaw[INNER][MECH] = 0;
	pidTermYaw[INNER][GYRO] = 0;
	
	iTermPit[INNER]   	  = 0;
	iTermYaw[INNER][MECH] = 0;
	iTermYaw[INNER][GYRO] = 0;
	
	fMotorOutput[ PITCH ] = 0;
	fMotorOutput[ YAW   ] = 0;
	
	Critical_Handle_Init(&Yaw_Gyro_Angle, Cloud_Angle_Measure[YAW][GYRO]);//重置测量角度

	//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
	Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[YAW][GYRO]);
	Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[PITCH][MECH]);
	//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
	yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
	pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);

	
	CAN1_Cloud_Send( fMotorOutput );
	actGimbal = GIMBAL_NORMAL;
}

/**
  * @brief  云台初始化
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_InitCtrl(void)
{
	static bool bAngleRecord  = FALSE;
	static portTickType  ulTimeCurrent = 0;


	if (xTaskGetTickCount( ) - ulTimeCurrent > TIME_STAMP_100MS)//保证不断电情况下下次可用
	{
		  bAngleRecord = FALSE;
	}

	ulTimeCurrent = xTaskGetTickCount( );

	//记录上电时云台机械角度
	if (bAngleRecord == FALSE)
	{
		bAngleRecord = TRUE;
			
		Cloud_Angle_Target[PITCH][MECH] = angleMotorPit;
		Cloud_Angle_Target[YAW][MECH] = angleMotorYaw;
	}
	
	Critical_Handle_Init(&Yaw_Gyro_Angle, Cloud_Angle_Measure[YAW][GYRO]);//记录陀螺仪初始角度
	
	//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
	Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[YAW][GYRO]);
	Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[PITCH][MECH]);
	//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
	yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
	pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
	
	modeGimbal = CLOUD_MECH_MODE;//默认以机械模式启动
	
	//平缓地让云台移动到中间,防止刚上电狂甩
	Cloud_Angle_Target[PITCH][MECH] = RAMP_float( Mech_Mid_Pitch, Cloud_Angle_Target[PITCH][MECH], Slope_Begin_Pitch);
	Cloud_Angle_Target[YAW][MECH]   = RAMP_float( Mech_Mid_Yaw, Cloud_Angle_Target[YAW][MECH], Slope_Begin_Yaw);
}

/**
  * @brief  遥控控制云台模式
  * @param  void
  * @retval void
  * @attention 在此改变角度环目标值
  */
void GIMBAL_Rc_Ctrl( void )
{	
	if(modeGimbal == CLOUD_GYRO_MODE)
	{
		//限制云台与底盘分离角度
		Gimbal_Chass_Separ_Limit();
	}
	
	if (IF_RC_SW2_DOWN)//s2拨到下,陀螺仪模式
	{
		modeGimbal = CLOUD_GYRO_MODE;
	}
	else if (IF_RC_SW2_MID)//S2中
	{
		modeGimbal = CLOUD_MECH_MODE;//S2中,机械模式
	}
	
	/*-----遥控控制弹仓开启------*/
	if ( Magazine_IfWait() == TRUE			
				&& Magazine_IfOpen() == TRUE )
	{
		Cloud_Angle_Target[PITCH][MECH] = Mech_Mid_Pitch;

		if (modeGimbal == CLOUD_MECH_MODE)//机械模式下yaw归中
		{
			Cloud_Angle_Target[YAW][MECH] = Mech_Mid_Yaw;
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)//陀螺仪模式下摇杆控制
		{
			Cloud_Angle_Target[YAW][GYRO] += -RC_CH0_RLR_OFFSET*kRc_Gyro_Yaw;
		}

	}
	else    
	{
		//正常控制pitch
		if (modeGimbal == CLOUD_MECH_MODE)
		{
			Cloud_Angle_Target[PITCH][MECH] += -RC_CH1_RUD_OFFSET*kRc_Mech_Pitch; 
			Cloud_Angle_Target[YAW][MECH]    =  Mech_Mid_Yaw; //机械模式,yaw固定在中
			
			Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
			Cloud_Angle_Target[PITCH][MECH] += -RC_CH1_RUD_OFFSET*kRc_Mech_Pitch;//pitch仍用机械处理方式  
			Cloud_Angle_Target[YAW][GYRO]   += -RC_CH0_RLR_OFFSET*kRc_Gyro_Yaw; 
		}
	}	
}

/**
  * @brief  键盘控制云台模式
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_Key_Ctrl(void)
{	
	if(modeGimbal == CLOUD_GYRO_MODE)
	{
		//限制云台与底盘分离角度
		Gimbal_Chass_Separ_Limit();
	}
	
	switch(actGimbal)//SB keil会有警告
	{
		/*--------------云台模式选择----------------*/
		case GIMBAL_NORMAL:
			GIMBAL_NORMAL_Mode_Ctrl();//在此选择控制模式
		break;
		
		/*--------------V  180°调头----------------*/
		case GIMBAL_AROUND:
			modeGimbal = CLOUD_GYRO_MODE;//进入陀螺仪模式
		
			if (TURNMode_Yaw_Back_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Back_Total, Cloud_Angle_Target[YAW][GYRO], Slope_Back_Yaw );
			}
		break;
		
		/*------------弹仓开启,禁止抬头-----------------*/
		case GIMBAL_LEVEL:
			GIMBAL_LEVEL_Mode_Ctrl();
		break;
		
		/*--------------Q E  90°调头----------------*/
		case GIMBAL_TURN:				
			modeGimbal = CLOUD_GYRO_MODE;//进入陀螺仪模式

		    if (TURNMode_Yaw_Turn_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Turn_Total, Cloud_Angle_Target[YAW][GYRO], Slope_Back_Yaw );
			}
		break;
			
		/*--------------右键自瞄----------------*/	
		case GIMBAL_AUTO:
			modeGimbal = CLOUD_GYRO_MODE;//进入陀螺仪模式
		
			if(!IF_MOUSE_PRESSED_RIGH)//松开右键退出自瞄
			{
				actGimbal = GIMBAL_NORMAL;
				
				//自瞄目标偏差清零,避免切换时云台跳动
				VisionRecvData.identify_target = FALSE;
				Auto_KF_Delay = 0;//清零给下次延迟预测用
				Mobility_Prediction_Yaw = FALSE;//标记预测没开启
				Mobi_Pre_Yaw_Fire = FALSE;//默认标记预测没到位，禁止开火
				
				mobpre_yaw_left_delay  = 0;//重置左预测的开火延迟
				mobpre_yaw_right_delay = 0;//重置右预测的开火延迟	
				mobpre_yaw_stop_delay = 0;//停止预测开火延时重置
			}
			else
			{
				GIMBAL_AUTO_Mode_Ctrl();//自瞄控制函数
			}
		break;
		
		/*--------------Ctrl+V键打小符----------------*/	
		case GIMBAL_SM_BUFF:
			//任意方向移动退出打符
			if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
					|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E)
			{
				actGimbal = GIMBAL_NORMAL;
				modeGimbal = CLOUD_GYRO_MODE;//退出打符切回陀螺仪模式
				
				gb_yaw_posit_error   = 1000;
				gb_pitch_posit_error = 1000;
				
				is_firstime_into_buff = TRUE;
			}
			else
			{
				#if 	BUFF_CAM_TYPE == BUFF_CAM_CHAS
					modeGimbal = CLOUD_MECH_MODE;//打符时底盘不动
					GIMBAL_BUFF_Mode_Ctrl_Chassis();//底盘打符
				
				#elif	BUFF_CAM_TYPE == BUFF_CAM_TYPE == BUFF_CAM_GIMB
					modeGimbal = CLOUD_GYRO_MODE;//打符时底盘不动
					GIMBAL_BUFF_Mode_Ctrl_Gimbal();//云台打符
				
				#endif
			}		
		break;	
			
		/*--------------Ctrl+F键打大符----------------*/	
		case GIMBAL_BUFF:
			//任意方向移动退出打符
			if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
					|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)
			{
				actGimbal = GIMBAL_NORMAL;
				modeGimbal = CLOUD_GYRO_MODE;//退出打符切回陀螺仪模式
				
				gb_yaw_posit_error   = 1000;
				gb_pitch_posit_error = 1000;
				
				is_firstime_into_buff = TRUE;
			}
			else
			{
				#if 	BUFF_CAM_TYPE == BUFF_CAM_CHAS
					modeGimbal = CLOUD_MECH_MODE;//打符时底盘不动
					GIMBAL_BUFF_Mode_Ctrl_Chassis();//底盘打符
				
				#elif	BUFF_CAM_TYPE == BUFF_CAM_TYPE == BUFF_CAM_GIMB
					modeGimbal = CLOUD_GYRO_MODE;//打符时底盘不动
					GIMBAL_BUFF_Mode_Ctrl_Gimbal();//云台打符
				
				#endif
			}		
		break;
			
		/*--------------C键吊射----------------*/
		case GIMBAL_BASE:
			modeGimbal = CLOUD_GYRO_MODE;//进入陀螺仪模式
		
			if(!IF_KEY_PRESSED_C)//松开右键退出自瞄
			{
				actGimbal = GIMBAL_NORMAL;
				first_time_into_base = TRUE;
			}
			else
			{
				GIMBAL_BASE_Mode_Ctrl();
			}
		break;
		/*--------------Ctrl+G键手打大符----------------*/	
//		case GIMBAL_MANUAL:
//			//QEV退出
//			if(IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)
//			{
//				actGimbal   = GIMBAL_NORMAL;
//				modeGimbal  = CLOUD_GYRO_MODE;//退出打符切回陀螺仪模式
//				Manual_Step = CONFIRM_BEGIN;
//				Manual_Pitch_Comp = 72;//重置补偿
//			}
//			else
//			{
//				modeGimbal = CLOUD_MECH_MODE;//打符时底盘不动
//				GIMBAL_MANUAL_Mode_Ctrl();
//			}		
//		break;
	}
	
}

/**
  * @brief  限制云台与底盘分离角度
  * @param  void
  * @retval void
  * @attention 
  */
void Gimbal_Chass_Separ_Limit(void)
{
	if ( (GIMBAL_GetOffsetAngle() <= -CLOUD_SEPAR_ANGLE				//右过大
			&& (RC_Ctl.mouse.x>0||RC_Ctl.rc.ch0>RC_CH_VALUE_OFFSET))	
				|| ( GIMBAL_GetOffsetAngle() >= CLOUD_SEPAR_ANGLE	//左过大
					&& ( RC_Ctl.mouse.x<0 || RC_Ctl.rc.ch0<RC_CH_VALUE_OFFSET) ) )
	{
//		RC_Ctl.mouse.x = 0;
//		//调试的时候记得注释，否则会出现ch0有半边无数据的情况
//		RC_Ctl.rc.ch0  = RC_CH_VALUE_OFFSET;
		
		kRc_Gyro_Yaw   -= krc_gyro_yaw/300;//0.005;
		kKey_Gyro_Yaw   -= -krc_gyro_yaw/300;//-0.15;
		
		if(abs(kRc_Gyro_Yaw) < abs(krc_gyro_yaw/290)
			|| abs(kKey_Gyro_Yaw) < abs(krc_gyro_yaw/290))
		{
			kRc_Gyro_Yaw = 0;
			kKey_Gyro_Yaw = 0;
		}
	}
	else
	{
		kRc_Gyro_Yaw   = krc_gyro_yaw;//0.015;
		kKey_Gyro_Yaw   = -kkey_gyro_yaw;//-0.38;
	}
	
	#if YAW_POSITION == YAW_UP
		if ( (GIMBAL_GetOffsetAngle() <= (Mech_Min_Yaw - Mech_Mid_Yaw + 50)	&& RC_Ctl.mouse.x>0)//右过大	
					|| ( GIMBAL_GetOffsetAngle() >= (Mech_Max_Yaw - Mech_Mid_Yaw - 50) && RC_Ctl.mouse.x<0 )	//左过大
		   )
		{
			kKey_Gyro_Yaw = 0;
		}
		else
		{
			kKey_Gyro_Yaw   = -kkey_gyro_yaw;
		}
	#else
		if ( (GIMBAL_GetOffsetAngle() <= (Mech_Max_Yaw - Mech_Mid_Yaw +500) && RC_Ctl.mouse.x>0)//右过大	
					|| ( GIMBAL_GetOffsetAngle() >= (Mech_Min_Yaw - Mech_Mid_Yaw - 500) && RC_Ctl.mouse.x<0 )//左过大	
		   )
		{
			kKey_Gyro_Yaw = 0;
		}
		else
		{
			kKey_Gyro_Yaw   = -kkey_gyro_yaw;
		}
	#endif
}


/*******************云台键盘模式各类模式小函数*******************/

/**
  * @brief  云台键盘模式选择,按键响应
  * @param  void
  * @retval void
  * @attention 云台键盘控制状态下的所有模式切换都在这
  * 无模式切换时一直处于此模式
  */
void GIMBAL_NORMAL_Mode_Ctrl(void)
{
	//按键延时响应,防止手贱狂按
	static portTickType  Key_Ctrl_CurrentTime = 0;
	static uint32_t PressV_Time  = 0;//调头,500ms延时响应,1秒最多按2下
	static uint32_t PressQ_Time  = 0;//90°,250ms延时响应,1秒最多按4下
    static uint32_t PressE_Time  = 0;//90°,250ms延时响应,1秒最多按4下
	static uint32_t PressCF_Time  = 0;//打大符,400ms延时响应
//	static uint32_t PressCG_Time  = 0;//手动打符,400ms延时响应
	static uint32_t PressCV_Time  = 0;//打小符,400ms延时响应
	static uint32_t Mouse_Yaw_Stop  = 0;//鼠标不动，结束响应
	static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响应
	
	Key_Ctrl_CurrentTime = xTaskGetTickCount( );//获取实时时间,用来做按键延时判断	
	
	
	if ( CHASSIS_IfActiveMode() == TRUE || Magazine_IfOpen() ==	 TRUE)//获取底盘模式,true为机械模式
	{
		modeGimbal = CLOUD_MECH_MODE;
	} 
	else					//注释掉loop中的底盘会令陀螺仪模式失效
	{
		modeGimbal = CLOUD_GYRO_MODE;
	}

	Manual_Step = CONFIRM_BEGIN;//退出手打大符步骤清零
	
	if ( !IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_V
					&& Key_Ctrl_CurrentTime > PressV_Time)
	{   //Ctrl不处于按下状态时按V调头
		actGimbal  =  GIMBAL_AROUND;//切换成调头模式

		PressV_Time = Key_Ctrl_CurrentTime + TIME_STAMP_500MS;//500ms延时防手贱狂按

		if(IF_KEY_PRESSED_A)//AV左调头
		{
			TURNMode_Yaw_Back_Total = 3579;
		}
		else if(IF_KEY_PRESSED_D)//DV右调头
		{
			TURNMode_Yaw_Back_Total = -3579;
		}
		else//默认右调头
		{
				TURNMode_Yaw_Back_Total = -3579;//因为角度放大了20倍,所以是180°*20约等于3579
		}
	}
	/*---------------------------------*/	
	else if ( !IF_KEY_PRESSED_CTRL
				&& ( (IF_KEY_PRESSED_Q && Key_Ctrl_CurrentTime > PressQ_Time)
					|| (IF_KEY_PRESSED_E && Key_Ctrl_CurrentTime > PressE_Time) ) )
	{   //Ctrl不处于按下状态时按Q(左),E(右)90°调头
		actGimbal = GIMBAL_TURN;//切换成快速扭头模式
		
		//注意方向
		if ( IF_KEY_PRESSED_Q)
		{
			PressQ_Time = Key_Ctrl_CurrentTime + TIME_STAMP_250MS;//250ms延时防手贱狂按
			
			TURNMode_Yaw_Turn_Total = 1789;//Q左转约8192/4度
		}
		else if (IF_KEY_PRESSED_E)
		{
			PressE_Time = Key_Ctrl_CurrentTime + TIME_STAMP_250MS;//250ms延时防手贱狂按
			
			TURNMode_Yaw_Turn_Total = -1789;//E右转约8192/4度
		}
			
	}	
	/*---------------------------------*/
	else if ( Magazine_IfWait() == TRUE			//弹仓开启或正在开启,云台归中不给动
				|| Magazine_IfOpen() == TRUE )
	{
		actGimbal = GIMBAL_LEVEL;

	}
	/*---------------------------------*/
	else if (IF_MOUSE_PRESSED_RIGH && !IF_RC_SW1_MID)//若SW1不在中,则右键自瞄
	{
		actGimbal = GIMBAL_AUTO;

	}
	/*----------------小符-----------------*/
	else if(IF_KEY_PRESSED_V && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCV_Time)//Ctrl+F打符,400ms响应一次
	{
		PressCV_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
		actGimbal = GIMBAL_SM_BUFF;
	}
	/*----------------大符-----------------*/
	else if(IF_KEY_PRESSED_F && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCF_Time)//Ctrl+F打符,400ms响应一次
	{
		PressCF_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
		actGimbal = GIMBAL_BUFF;
	}
	/*----------------吊射-----------------*/
	else if(IF_KEY_PRESSED_C && !IF_MOUSE_PRESSED_RIGH && !IF_RC_SW1_MID)
	{
		actGimbal = GIMBAL_BASE;
	}
	/*---------------------------------*/
//	else if(IF_KEY_PRESSED_G && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCG_Time)//Ctrl+G手动打符,400ms响应一次
//	{
//		PressCG_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
//		actGimbal = GIMBAL_MANUAL;
//	}
	/*---------------------------------*/
	else       //最后做云台角度计算,这是普通模式下的角度计算,优先级最低,所以放最后面
	{
		if (modeGimbal == CLOUD_MECH_MODE)//机械模式
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Mech_Pitch;
			Cloud_Angle_Target[YAW][MECH]    = Mech_Mid_Yaw;	//yaw保持不动,永远在中间
			
			Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
//			Critical_Handle_Init(&Yaw_Gyro_Angle, Cloud_Angle_Measure[YAW][GYRO]);//重置测量角度			
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
			Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//记录目标变化角度
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch仍旧使用机械模式
//			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch仍旧使用机械模式
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Yaw = 0;
				}
			}
			else
			{
				Mouse_Yaw_Stop = 0;
			}
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Pitch = 0;
				}
			}
			else
			{
				Mouse_Pitch_Stop = 0;
			}
			Cloud_Angle_Target[YAW][GYRO]   = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], Slope_Mouse_Yaw );
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Slope_Mouse_Pitch );
		}
	}
}

/**
  * @brief  摩擦轮开启,云台抬头
  * @param  void
  * @retval void
  * @attention 此模式下禁止控制pitch
  */
void GIMBAL_HIGH_Mode_Ctrl(void)
{
	if (FRIC_IfOpen( ) == TRUE)//摩擦轮开启完毕,可以切换回正常模式了
	{
		actGimbal = GIMBAL_NORMAL;
		Cloud_Angle_Target[PITCH][MECH] = Mech_Mid_Pitch;//开启完毕pitch归中
	}
	else		//pitch抬头等摩擦轮开完
	{
		Cloud_Angle_Target[PITCH][MECH] = RAMP_float(CLOUD_FRIC_PIT_UP, Cloud_Angle_Target[PITCH][MECH], Slope_Fric_Pitch);
	}
	
	if (modeGimbal == CLOUD_MECH_MODE)
	{
		Cloud_Angle_Target[YAW][MECH]    =  Mech_Mid_Yaw; //机械模式,yaw固定在中
	}
	else
	{
		Mouse_Gyro_Yaw += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//记录目标变化角度
		Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], Slope_Mouse_Yaw );
	}
}

/**
  * @brief  补弹模式
  * @param  void
  * @retval void
  * @attention 此模式下禁止控制pitch
  */
void GIMBAL_LEVEL_Mode_Ctrl(void)
{
	modeGimbal = CLOUD_MECH_MODE;//补弹时进入机械模式
	
	//补弹完毕,退出补弹模式
	if( Magazine_IfWait() == FALSE			
			&& Magazine_IfOpen() == FALSE )
	{
		actGimbal = GIMBAL_NORMAL;
	}
	else//补弹未完成,角度固定在中间
	{
		Cloud_Angle_Target[YAW][MECH]   = Mech_Mid_Yaw;
		Cloud_Angle_Target[PITCH][MECH] = Mech_Mid_Pitch;
	}
}

/**********************************************************************************/
/**
  * @brief  自瞄控制函数
  * @param  void
  * @retval void
  * @attention 中间坐标(0,0),左正右负,上负下正
  *            yaw为陀螺仪模式,pitch为机械模式(其实pitch全程都在用机械模式)
  *            只有当新数据来了才在当前实时角度上累加误差当成目标角度
  *            新数据一来,目标角度就实时更新
  */
float debug_y_dk = 450;//yaw距离预测比例，越大预测越少
uint32_t Vision_Time[2];// NOW/LAST

int vision_time_js;
float error_yaw_k   = 1;//7.5;//5.6;//2.2;//误差放大
float error_pitch_k = 10;//5;//3;//2.1;//误差放大
float debug_kf_y_angle;//yaw预测暂存
float debug_kf_p_angle;//pitch预测暂存

//根据距离调节预测比例和限幅
float yaw_speed_k = 0;
float kf_yaw_angcon = 0;

float pitch_speed_k = 0;
float kf_pitch_angcon = 0;

float debug_kf_angle_temp;//预测角度斜坡暂存量
float debug_kf_angle_ramp = 20;//预测角度斜坡变化量
float debug_kf_dist;
float debug_dist_bc = 0;
float gim_auto_ramp_y = 5;//10;//刚开启自瞄时缓慢移过去，防止视觉拖影掉帧
float gim_auto_ramp_p = 5;//刚开启自瞄时缓慢移过去，防止视觉拖影掉帧
int js_yaw = 0;
int js_pitch = 0;

float kf_speed_yl = 0;//

void GIMBAL_AUTO_Mode_Ctrl(void)
{
	static uint32_t Mouse_Yaw_Stop  = 0;//鼠标不动，结束响应
	static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响应
	
	static float yaw_angle_raw, pitch_angle_raw;//卡尔曼滤波角度测量值
	static float yaw_angle_ref;//记录目标角度
	static float pitch_angle_ref;//记录目标角度
	
	float kf_delay_open = 0;
	
	Mobility_Prediction_Yaw = FALSE;//默认标记预测没开启
	Mobi_Pre_Yaw_Fire = FALSE;//默认标记预测没到位，禁止开火
	
	kf_speed_yl = debug_kf_speed_yl;

	//获取角度偏差量,欧拉角类型,过分依赖于视觉的精准度
	Vision_Error_Angle_Yaw(&Auto_Error_Yaw[NOW]);
	Vision_Error_Angle_Pitch(&Auto_Error_Pitch[NOW]);
	Vision_Get_Distance(&Auto_Distance);
	
	Auto_Distance = KalmanFilter(&Vision_Distance_Kalman, Auto_Distance);
	
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓数据更新↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	if(Vision_If_Update() == TRUE)//视觉数据更新了
	{
		//更新目标角度//记录当前时刻的目标位置,为卡尔曼做准备
		yaw_angle_ref   = Cloud_Angle_Measure[YAW][GYRO]   + Auto_Error_Yaw[NOW]   * error_yaw_k;
		pitch_angle_ref = Cloud_Angle_Measure[PITCH][MECH] + Auto_Error_Pitch[NOW] * error_pitch_k;
		
		Vision_Clean_Update_Flag();//一定要记得清零,否则会一直执行
		Vision_Time[NOW] = xTaskGetTickCount();//获取新数据到来时的时间
	}
	/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑数据更新↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
	
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓二阶卡尔曼计算↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	if(Vision_Time[NOW] != Vision_Time[LAST])//更新新数据到来的时间
	{
		vision_time_js = Vision_Time[NOW] - Vision_Time[LAST];//计算视觉延迟
		yaw_angle_raw  = yaw_angle_ref;//更新二阶卡尔曼滤波测量值
		pitch_angle_raw = pitch_angle_ref;
		Vision_Time[LAST] = Vision_Time[NOW];
	}
	
	//目标速度解算
	if(VisionRecvData.identify_target == TRUE)//识别到了目标
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, Vision_Time[NOW], yaw_angle_raw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, Vision_Time[NOW], pitch_angle_raw);
		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, Vision_Angle_Speed_Yaw);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, Vision_Angle_Speed_Pitch);

	}
	else
	{
//		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[YAW][GYRO]);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[PITCH][MECH]);
//		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
		debug_kf_angle_temp = 0;
	}
	
	if(Chassis_IfCORGI() == TRUE && GIMBAL_AUTO_PITCH_SB() == FALSE)//扭腰且不在打哨兵
	{
		kf_delay_open = debug_kf_delay*3.f;
	}
	else
	{
		kf_delay_open = debug_kf_delay;
	}
	
	//未识别到目标时鼠标可随意控制云台
	if(VisionRecvData.identify_target == TRUE)//识别到了目标
	{
		Auto_KF_Delay++;//滤波延时开启

		if(VisionRecvData.auto_too_close == TRUE 
			&& (Chassis_IfCORGI() == FALSE || GIMBAL_AUTO_PITCH_SB() == FALSE) )//目标距离太近，减小预测
		{
			yaw_speed_k = debug_y_sk;///4.f;//3.f;//预测系数减半
			kf_yaw_angcon = debug_kf_y_angcon;//3.f;//2.f;
			kf_speed_yl = debug_kf_speed_yl;
		}
		else//正常预测量
		{
			if( GIMBAL_AUTO_PITCH_SB() == TRUE )
			{
				yaw_speed_k = debug_y_sb_sk;			
				kf_yaw_angcon = debug_kf_y_angcon;
				kf_speed_yl = debug_kf_speed_yl_sb;
				
				if(IF_KEY_PRESSED_G)
				{
					yaw_speed_k = debug_y_sb_brig_sk;
					kf_yaw_angcon = debug_kf_y_angcon*1.1f;
					kf_speed_yl = debug_kf_speed_yl_sb*0.4f;//0.9f;
				}
			}
			else
			{
				yaw_speed_k = debug_y_sk;
				kf_yaw_angcon = debug_kf_y_angcon;
				kf_speed_yl = debug_kf_speed_yl;
			}
		}
		/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑二阶卡尔曼计算↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
		js_yaw = yaw_kf_result[KF_SPEED]*1000;
		js_pitch = yaw_kf_result[KF_ANGLE]*1000;
		
		/*预测开启条件*/
		if(fabs(Auto_Error_Yaw[NOW]) < debug_auto_err_y//debug看 
				&& Auto_KF_Delay > kf_delay_open 
					&& fabs(yaw_kf_result[KF_SPEED]) >= kf_speed_yl 
						&& fabs(yaw_kf_result[KF_SPEED]) < debug_kf_speed_yh )
		{
			
			if(yaw_kf_result[KF_SPEED]>=0)
			{
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] - kf_speed_yl) * 1;//debug_kf_dist;
			}
			else if(yaw_kf_result[KF_SPEED]<0)
			{
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] + kf_speed_yl) * 1;//debug_kf_dist;			
			}
//			debug_kf_angle_temp = debug_y_sk * yaw_kf_result[KF_SPEED];//此处不需要速度太慢关预测
			debug_kf_angle_temp = constrain_float(debug_kf_angle_temp, -debug_kf_y_angcon, debug_kf_y_angcon);//预测暂存量限幅
			debug_kf_y_angle = RAMP_float(debug_kf_angle_temp, debug_kf_y_angle, debug_kf_angle_ramp);//预测量缓慢变化
			
			debug_kf_y_angle = constrain_float(debug_kf_y_angle, -debug_kf_y_angcon, debug_kf_y_angcon);
			Cloud_Angle_Target[YAW][GYRO] = yaw_kf_result[KF_ANGLE] + debug_kf_y_angle;//debug_y_sk * (yaw_kf_result[KF_SPEED] - debug_kf_speed);//Vision_Gyro_MovProj_Yaw(yaw_kf_result[1]);//yaw_kf_result[0];
			
			/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓预测到位判断↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
			/*yaw_kf_result[1]左移正，右移负，debug看出来的*/
			/*给自动打弹范围做个小标记*/
			if( (yaw_kf_result[KF_SPEED]>0) //目标向左移且误差值显示说目标在右边，则说明预测到位置，可打弹
					&& (Auto_Error_Yaw[NOW] < 0.3f) )
			{
				mobpre_yaw_right_delay = 0;//向右预测开火延时重置

				mobpre_yaw_left_delay++;
				if(mobpre_yaw_left_delay > 0/*75*/)//稳定150ms
				{
					Mobi_Pre_Yaw_Fire = TRUE;//预测到位，可开火
				}
				else
				{
					Mobi_Pre_Yaw_Fire = FALSE;//预测没到位，不可开火
				}
			}
			else if( (yaw_kf_result[KF_SPEED]<0) //目标向右移且误差值显示说目标在左边，则说明预测到位置，可打弹
						&& (Auto_Error_Yaw[NOW] > -0.3f) )
			{
				mobpre_yaw_left_delay = 0;//向左预测开火延时重置
				
				mobpre_yaw_right_delay++;
				if(mobpre_yaw_right_delay > 0/*75*/)//稳定150ms
				{
					Mobi_Pre_Yaw_Fire = TRUE;//预测到位，可开火
				}
				else
				{
					Mobi_Pre_Yaw_Fire = FALSE;//预测没到位，不可开火
				}
			}
			else
			{
				Mobi_Pre_Yaw_Fire = FALSE;//标记预测没到位，禁止开火
				
				mobpre_yaw_left_delay = 0;//向左预测开火延时重置
				mobpre_yaw_right_delay = 0;//向右预测开火延时重置
			}
			/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑预测到位判断↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
			Mobility_Prediction_Yaw = TRUE;//标记预测已开启

			mobpre_yaw_stop_delay = 0;//重置静止时的开火延迟
			
			
			Green_On;
			Red_On;
			Orange_On;
		}
		/*预测条件没达到，关闭预测*/
		else
		{
			Cloud_Angle_Target[YAW][GYRO] = yaw_angle_ref;
			Mobility_Prediction_Yaw = FALSE;//标记预测没开启
			mobpre_yaw_left_delay  = 0;//重置左预测的开火延迟
			mobpre_yaw_right_delay = 0;//重置右预测的开火延迟	
			
			if( fabs(Auto_Error_Yaw[NOW]) < 1.5f )//接近目标
			{
				mobpre_yaw_stop_delay++;
				if(mobpre_yaw_stop_delay > 25)//停止稳定50ms
				{
					Mobi_Pre_Yaw_Fire = TRUE;//此时根据视觉开火标志位做射击判断，记得一定要置TRUE
				}
			}
			else
			{
				Mobi_Pre_Yaw_Fire = FALSE;//标记没回复到位，禁止开火
			}	
			Green_Off;
			Red_Off;
			Orange_Off;
		}
		
		/*---------------pitch给个很小的预测------------------*/

		if( Auto_KF_Delay > debug_kf_delay 
				&& fabs(Auto_Error_Pitch[NOW]) < debug_auto_err_p
					&& fabs(pitch_kf_result[KF_SPEED]) > debug_kf_speed_pl
						&& (GIMBAL_AUTO_PITCH_SB_SK() == FALSE || GIMBAL_AUTO_PITCH_SB() == FALSE)
							&& VisionRecvData.distance/100 < 4.4f
		  )
		{	
			if(VisionRecvData.auto_too_close == TRUE)//目标距离太近，减小预测
			{
				pitch_speed_k = debug_p_sk/2.f;//预测系数减半
				kf_pitch_angcon = debug_kf_p_angcon/1.5f;
			}
			else//正常预测量
			{
				pitch_speed_k = debug_p_sk;
				kf_pitch_angcon = debug_kf_p_angcon;
			}
			
			if(pitch_kf_result[KF_SPEED]>=0)
			{
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] - debug_kf_speed_pl);
			}
			else
			{
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] + debug_kf_speed_pl);			
			}
			//pitch预测量限幅
			debug_kf_p_angle = constrain_float(debug_kf_p_angle, -kf_pitch_angcon, kf_pitch_angcon);
			
			Cloud_Angle_Target[PITCH][MECH] = pitch_kf_result[KF_ANGLE] + debug_kf_p_angle;
		}
		/*预测条件没达到，关闭预测*/
		else
		{
			Cloud_Angle_Target[PITCH][MECH] = pitch_angle_ref;
		}
	}
	else		//未识别到目标,可随意控制云台
	{
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
		
		if (modeGimbal == CLOUD_MECH_MODE)//机械模式
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Mech_Pitch;
			Cloud_Angle_Target[YAW][MECH]    = Mech_Mid_Yaw;	//yaw保持不动,永远在中间			
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
			Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//记录目标变化角度
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch仍旧使用机械模式
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Yaw = 0;
				}
			}
			else
			{
				Mouse_Yaw_Stop = 0;
			}
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Pitch = 0;
				}
			}
			else
			{
				Mouse_Pitch_Stop = 0;
			}
			Cloud_Angle_Target[YAW][GYRO]   = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], Slope_Mouse_Yaw );
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Slope_Mouse_Pitch );
		}
		Auto_KF_Delay = 0;	

		mobpre_yaw_left_delay  = 0;//重置左预测的开火延迟
		mobpre_yaw_right_delay = 0;//重置右预测的开火延迟	
		mobpre_yaw_stop_delay = 0;//停止预测开火延时重置		
	}
}

float speed_threshold = 5.f;//速度过快
float debug_speed;//左正右负,一般都在1左右,debug看
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//计算速度

//		if ((S->speed - S->processed_speed) < -speed_threshold)
//		{
//			S->processed_speed = S->processed_speed - speed_threshold;//速度斜坡变化
//		}
//		else if ((S->speed - S->processed_speed) > speed_threshold)
//		{
//			S->processed_speed = S->processed_speed + speed_threshold;//速度斜坡变化
//		}

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//时间过长则认为速度不变
	}
	debug_speed = S->processed_speed;
	return S->processed_speed;//计算出的速度
}


float debug_angle_diff = 2;
bool Gimb_If_Small_Top(float angle)
{
	static float last_angle = 0;
	static float prev_angle = 0;
		   float angle_diff = 0;
	static uint16_t top_change_times = 0;//切装甲次数，用来判断是否小陀螺
	
	prev_angle = angle;
	
	angle_diff = prev_angle - last_angle;
	if(angle_diff >= debug_angle_diff)//用来判断是否小陀螺，一般情况下小陀螺会有装甲切换
	{
		top_change_times++;
	}
	else
	{
		top_change_times = 0;
	}
	
	last_angle = angle;
	
	if(top_change_times > 3)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  打符模式，摄像头位于底盘
  * @param  void
  * @retval void
  * @attention 红方打红色,蓝方打蓝色,停留3秒激活打符,可能需要关激光
  *  5倍热量冷却,桥面离地1米2,大风车中心离地2米5,内径70cm,外径80cm
  *  选取绝对坐标（云台机械中心为0），在此基础上叠加视觉坐标，将
  *  角度值转换成云台机械角度     x/8192 = angle/360;
  */
uint32_t Vision_Buff_Time[2];//测试帧率
int vision_buff_time_js;

float gb_yaw_angle = 0;//获取角度
float gb_pitch_angle = 0;//获取角度
float debug_gb_y_error = 0;//机械角度测量值与目标值之间的误差
float debug_gb_p_error = 0;//机械角度测量值与目标值之间的误差
float buff_y_raw;//滤波后的值
float buff_p_raw;//滤波后的值
float gb_pitch_compe = 0;//抬头补偿
//float gb_t;//子弹飞行时间
//float gb_dist = 7.15;//打符距离
//float gb_v = 27.5;//子弹出膛速度
//float gb_vd;//子弹水平方向速度
void GIMBAL_BUFF_Mode_Ctrl_Chassis(void)
{
	float gb_yaw_mech = 0;
	float gb_pitch_mech = 0;
	float gb_y_ref = 0;
	float gb_p_ref = 0;
	
	
	
	/*- 帧率测试 -*/
	if( Vision_If_Update() == TRUE //identify_buff=2也是识别到了
			&& (VisionRecvData.identify_buff == TRUE || VisionRecvData.identify_buff == 2) )//视觉数据更新了
	{
		//获取角度
		Vision_Buff_Error_Angle_Yaw(&gb_yaw_angle);
		Vision_Buff_Error_Angle_Pitch(&gb_pitch_angle);
		
		Vision_Buff_Time[NOW] = xTaskGetTickCount();//获取新数据到来时的时间
		Vision_Clean_Update_Flag();//一定要记得清零,否则会一直执行
	}
	
	if(Vision_Buff_Time[NOW] != Vision_Buff_Time[LAST])//更新新数据到来的时间
	{
		vision_buff_time_js = Vision_Buff_Time[NOW] - Vision_Buff_Time[LAST];//计算视觉延迟
		Vision_Buff_Time[LAST] = Vision_Buff_Time[NOW];
	}
	/*--------------------------------*/
	
	
	buff_y_raw = KalmanFilter(&Gimbal_Buff_Yaw_Error_Kalman, gb_yaw_angle);
	buff_p_raw = KalmanFilter(&Gimbal_Buff_Pitch_Error_Kalman, gb_pitch_angle);

//	//抬头补偿
//	// 1/2gt^2  g=10  t = dist/vd   vd=v*cos(p)
//	gb_vd = gb_v * cos(fabs(buff_p_raw*PI/180.0f));//计算子弹水平方向速度，角度还没想好怎么算，是否要用实际角度，因为云台会滞后
//	gb_t = gb_dist / gb_vd;//计算子弹飞行时间	
//	gb_pitch_compe = 0.5f * 10 * gb_t * gb_t;//  1/2 g t^2
	
	//欧拉角转换成机械角
//	gb_yaw_mech   = gb_yaw_angle   / 360.0f * 8192;//未滤波
//	gb_pitch_mech = gb_pitch_angle / 360.0f * 8192;//未滤波
	gb_yaw_mech   = buff_y_raw / 360.0f * 8192 * Buff_Yaw_Correct_Chassis;
	gb_pitch_mech = buff_p_raw / 360.0f * 8192 * Buff_Pitch_Correct_Chassis;//未加抬头补偿
//	gb_pitch_mech = buff_p_raw / 360.0f * 8192;// - gb_pitch_compe/VisionRecvData.distance*180/PI/360.0f*8192;//加了抬头补偿
	
	//暂存目标位置
	gb_y_ref = /*Mech_Mid_Yaw*/debug_y_mid   + gb_yaw_mech;
	gb_p_ref = /*Mech_Mid_Pitch*/debug_p_mid + gb_pitch_mech;
	
	//移动,identify_buff为1或2都是识别到
	if(VisionRecvData.identify_buff == TRUE || VisionRecvData.identify_buff == 2)
	{
		Cloud_Angle_Target[YAW][MECH]   = gb_y_ref + Buff_Yaw_Comp;
		Cloud_Angle_Target[PITCH][MECH] = gb_p_ref + Buff_Pitch_Comp;
	}
	else
	{
		Cloud_Angle_Target[YAW][MECH]   += 0;//debug_y_mid;//Mech_Mid_Yaw;
		Cloud_Angle_Target[PITCH][MECH] += 0;//debug_p_mid;//Mech_Mid_Pitch;
	}
	
	//计算误差，用来判断是否瞄准到位
	gb_yaw_posit_error   = Cloud_Angle_Measure[YAW][MECH]   - Cloud_Angle_Target[YAW][MECH];
	gb_pitch_posit_error = Cloud_Angle_Measure[PITCH][MECH] - Cloud_Angle_Target[PITCH][MECH];
	
	//测试误差，看看有无滞后现象
	debug_gb_y_error = gb_yaw_posit_error;
	debug_gb_p_error = gb_pitch_posit_error;
}

/**
  * @brief  打符模式，摄像头位于云台
  * @param  void
  * @retval void
  * @attention 红方打红色,蓝方打蓝色,停留3秒激活打符,可能需要关激光
  *  5倍热量冷却,桥面离地1米2,大风车中心离地2米5,内径70cm,外径80cm
  *  pitch仍是机械模式，跟自瞄一样
  */
float gb_yaw_angle_gim = 0;//获取角度
float gb_pitch_angle_gim = 0;//获取角度
float buff_gimb_ramp_yaw = 72;//120;//国赛72，单项赛120
float buff_gimb_ramp_pitch = 72;//120;
float buff_into_time = 0;
void GIMBAL_BUFF_Mode_Ctrl_Gimbal(void)
{
	float gb_yaw_gyro = 0;
	float gb_pitch_mech = 0;
	static float y_mid = 0;
	static float p_mid = 0;
	
	static float yaw_buff_angle_raw, pitch_buff_angle_raw;//卡尔曼滤波角度测量值
//	static float yaw_buff_angle_ref;//记录目标角度
//	static float pitch_buff_angle_ref;//记录目标角度
	static float shoot_time = 0;
	static float lost_time  = 0;//一段时间没识别到，归中
	
	
	if(is_firstime_into_buff == TRUE)
	{
		is_firstime_into_buff = FALSE;
		buff_into_time = 0;
		
		//记录进入时的角度
		y_mid = Cloud_Angle_Measure[YAW][GYRO];
		p_mid = Cloud_Angle_Target[PITCH][MECH];
	}
	
	if(is_firstime_into_buff == FALSE)//进入一段时间
	{
		buff_into_time++;
	}
	
	/*- 帧率测试 -*/
	if( Vision_If_Update() == TRUE //identify_buff=2也是识别到了
			&& (VisionRecvData.identify_buff == TRUE || VisionRecvData.identify_buff == 2) )//视觉数据更新了
	{	
		//像素点
		Vision_Error_Yaw(&gb_yaw_angle_gim);
		Vision_Error_Pitch(&gb_pitch_angle_gim);
		gb_yaw_gyro   = gb_yaw_angle_gim + Buff_Yaw_Comp_Gimbal;
		gb_pitch_mech = gb_pitch_angle_gim + Buff_Pitch_Comp_Gimbal;//未加抬头补偿
		/******************************************************************/
	
		yaw_buff_angle_raw 	 = Cloud_Angle_Measure[YAW][GYRO]   + gb_yaw_gyro;
		pitch_buff_angle_raw = Cloud_Angle_Measure[PITCH][MECH] + gb_pitch_mech;
		
		Vision_Buff_Time[NOW] = xTaskGetTickCount();//获取新数据到来时的时间
		Vision_Clean_Update_Flag();//一定要记得清零,否则会一直执行
	}
	
	if(Vision_Buff_Time[NOW] != Vision_Buff_Time[LAST])//更新新数据到来的时间
	{
		vision_buff_time_js = Vision_Buff_Time[NOW] - Vision_Buff_Time[LAST];//计算视觉延迟
		Vision_Buff_Time[LAST] = Vision_Buff_Time[NOW];
	}
	/*--------------------------------*/
	
//	yaw_buff_angle_ref   = KalmanFilter(&Gimbal_Buff_Yaw_Error_Gim_Kalman, yaw_buff_angle_raw);
//	pitch_buff_angle_ref = KalmanFilter(&Gimbal_Buff_Pitch_Error_Gim_Kalman, pitch_buff_angle_raw);

	//移动,identify_buff为1或2都是识别到
	if(VisionRecvData.identify_buff == TRUE || VisionRecvData.identify_buff == 2)
	{
		if(buff_into_time > 100)//第一次进入打符，不给太快响应
		{
			Cloud_Angle_Target[YAW][GYRO]   = RAMP_float(yaw_buff_angle_raw, Cloud_Angle_Measure[YAW][GYRO], buff_gimb_ramp_yaw);
			Cloud_Angle_Target[PITCH][MECH] = RAMP_float(pitch_buff_angle_raw, Cloud_Angle_Measure[PITCH][MECH], buff_gimb_ramp_pitch);
		}
		else
		{
			Cloud_Angle_Target[YAW][GYRO]   = y_mid;
			Cloud_Angle_Target[PITCH][MECH] = p_mid;
		}
		lost_time = 0;
	}
	else
	{
		Cloud_Angle_Target[YAW][GYRO]   += 0;//debug_y_mid;//Mech_Mid_Yaw;
		Cloud_Angle_Target[PITCH][MECH] += 0;//debug_p_mid;//Mech_Mid_Pitch;
		
		lost_time++;
		
		//连续一段时间没识别到
		if(lost_time>300)
		{
			Cloud_Angle_Target[YAW][GYRO]	= y_mid;
			Cloud_Angle_Target[PITCH][MECH] = p_mid;
		}
	}
	

	
	//计算误差，用来判断是否瞄准到位
	if(VisionRecvData.identify_buff == FALSE)
	{
		shoot_time++;
		if(shoot_time > 100)//连续200MS没识别到,误差加大，防止退出重进后立马打弹
		{
			gb_yaw_posit_error = 1000;
			gb_pitch_posit_error = 1000;
		}
	}
	else
	{
		shoot_time = 0;
		
		if(buff_into_time > 200)
		{
			gb_yaw_posit_error   = Cloud_Angle_Measure[YAW][GYRO]   - yaw_buff_angle_raw;
			gb_pitch_posit_error = Cloud_Angle_Measure[PITCH][MECH] - pitch_buff_angle_raw;
		}
	}

}

/**
  * @brief  桥头吊射模式
  * @param  void
  * @retval void
  * @attention 像素点，方便调节，只有YAW，PITCH靠操作手，吊射识别到目标发8
  */
void GIMBAL_BASE_Mode_Ctrl(void)
{
	static uint32_t Mouse_Yaw_Stop  = 0;//鼠标不动，结束响应
	static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响应
	static float yaw_base_angle_raw = 0;
	
	float base_yaw_gyro = 0;
	
	Vision_Base_Yaw_Pixel(&Base_Error_Yaw);
	
	if(first_time_into_base == TRUE && VisionRecvData.identify_target == 8)
	{
		Cloud_Angle_Target[PITCH][MECH] = base_mech_pitch;
		first_time_into_base = FALSE;
	}
	
	if(Vision_If_Update() == TRUE)//视觉数据更新了
	{	
		base_yaw_gyro = Base_Error_Yaw + Base_Yaw_Comp_Gimbal;
		
		yaw_base_angle_raw = Cloud_Angle_Measure[YAW][GYRO] + base_yaw_gyro;
		
		Vision_Clean_Update_Flag();//一定要记得清零,否则会一直执行
		Vision_Time[NOW] = xTaskGetTickCount();//获取新数据到来时的时间
	}
	
	//未识别到目标时鼠标可随意控制云台
	if(VisionRecvData.identify_target == 8)//识别到了目标，注意别和自瞄搞混
	{
		//yaw自瞄
		Cloud_Angle_Target[YAW][GYRO] = yaw_base_angle_raw;
		
		//pitch正常控制
		if (modeGimbal == CLOUD_MECH_MODE)//机械模式
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * (kKey_Mech_Pitch/3);
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * (kKey_Gyro_Pitch/3);//pitch仍旧使用机械模式
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Pitch = 0;
				}
			}
			else
			{
				Mouse_Pitch_Stop = 0;
			}
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Slope_Mouse_Pitch );
		}
	}
	else		//未识别到目标,可随意控制云台
	{	
		if (modeGimbal == CLOUD_MECH_MODE)//机械模式
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Mech_Pitch;
			Cloud_Angle_Target[YAW][MECH]    = Mech_Mid_Yaw;	//yaw保持不动,永远在中间			
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
			Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//记录目标变化角度
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch仍旧使用机械模式
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Yaw = 0;
				}
			}
			else
			{
				Mouse_Yaw_Stop = 0;
			}
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Pitch = 0;
				}
			}
			else
			{
				Mouse_Pitch_Stop = 0;
			}
			Cloud_Angle_Target[YAW][GYRO]   = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], Slope_Mouse_Yaw );
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Slope_Mouse_Pitch );
		}	
	}
}

/**
  * @brief  手动打符模式
  * @param  void
  * @retval void
  * @attention 红方打红色,蓝方打蓝色,停留3秒激活打符,可能需要关激光
  *  5倍热量冷却,桥面离地1米2,大风车中心离地2米5,内径70cm,外径80cm
  *  Ctrl+F进入，第一次右键选择圆心，第二次右键选择左右半径
  *  第三次右键选择上下高度，鼠标粗调，WASD微调
  *  调节完毕后WASD表示上下左右，此时鼠标仅能抬头低头调抬头补偿
  *  选择圆心后粗略左移，给操作手细调
  *  选择半径后粗略沿中心上移，给操作手细调
  */
//切换标志
bool Manual_Switch_Right = 1;//用来判断右键响应

float Manual_Centre_Yaw, Manual_Centre_Pitch;//圆心
float Manual_Radius;//半径
float Manual_High;//高度
//int Manual_Step = 0;//第一步确定圆心，第二步确定半径，第三步WASD确定位置
void GIMBAL_MANUAL_Mode_Ctrl(void)
{
	if(!IF_MOUSE_PRESSED_RIGH)//右键松开
	{
		Manual_Switch_Right = 1;//可以切换步骤
	}
	
	/*- 0 保持进入前角度不变 -*/
	if(Manual_Step == CONFIRM_BEGIN)//刚进入手动打符,角度定住不动
	{
		Cloud_Angle_Target[YAW][MECH]   = Cloud_Angle_Measure[YAW][MECH];
		Cloud_Angle_Target[PITCH][MECH] = Cloud_Angle_Measure[PITCH][MECH];
		Manual_Step = CONFIRM_CENTRE;//进入下一步确认圆心
	}
	
	/*- 1 确定圆心 -*/
	if(Manual_Step == CONFIRM_CENTRE)//调节圆心
	{
		//鼠标粗调
		Cloud_Angle_Target[YAW][MECH]   += MOUSE_X_MOVE_SPEED * (kKey_Gyro_Yaw/5)*YAW_POSITION;
		Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * (kKey_Mech_Pitch/5);
		//键盘微调
		if(IF_KEY_PRESSED_W)//上
		{
			Cloud_Angle_Target[PITCH][MECH] -= 0.05f;
		}
		else if(IF_KEY_PRESSED_S)//下
		{
			Cloud_Angle_Target[PITCH][MECH] += 0.05f;
		}
		else if(IF_KEY_PRESSED_A)//左
		{
			Cloud_Angle_Target[YAW][MECH]   += 0.05f*YAW_POSITION;
		}
		else if(IF_KEY_PRESSED_D)//右
		{
			Cloud_Angle_Target[YAW][MECH]   -= 0.05f*YAW_POSITION;
		}
	}
	
	if( IF_MOUSE_PRESSED_RIGH && Manual_Step == CONFIRM_CENTRE
			&& Manual_Switch_Right == 1 )
	{
		//确定圆心
		Manual_Centre_Yaw   = Cloud_Angle_Measure[YAW][MECH];
		Manual_Centre_Pitch = Cloud_Angle_Measure[PITCH][MECH];
		Cloud_Angle_Target[YAW][MECH] = Manual_Centre_Yaw + 100*YAW_POSITION;//提前动，方便操作手调半径
		Manual_Step = CONFIRM_RADIUS;//进入下一步确认半径
		Manual_Switch_Right = 0;
	}
	
	/*- 2 确定半径 -*/
	if(Manual_Step == CONFIRM_RADIUS)//调节半径
	{
		//鼠标粗调
		Cloud_Angle_Target[YAW][MECH]   += MOUSE_X_MOVE_SPEED * (kKey_Gyro_Yaw/5)*YAW_POSITION;
		//键盘微调
		if(IF_KEY_PRESSED_A)//左
		{
			Cloud_Angle_Target[YAW][MECH]   += 0.05f*YAW_POSITION;
		}
		else if(IF_KEY_PRESSED_D)//右
		{
			Cloud_Angle_Target[YAW][MECH]   -= 0.05f*YAW_POSITION;
		}
	}
	
	if( IF_MOUSE_PRESSED_RIGH && Manual_Step == CONFIRM_RADIUS
			&& Manual_Switch_Right == 1 )
	{
		//计算半径
		Manual_Radius = fabs(Cloud_Angle_Measure[YAW][MECH] - Manual_Centre_Yaw);
		Cloud_Angle_Target[PITCH][MECH] = Manual_Centre_Pitch - Manual_Radius;//方便操作手调高度
		Manual_Step = CONFIRM_HIGH;//进入下一步确认位置
		Manual_Switch_Right = 0;
	}
	
	/*- 3 确认上下高度 -*/
	if(Manual_Step == CONFIRM_HIGH)//调节高度
	{
		//鼠标粗调
		Cloud_Angle_Target[YAW][MECH] = Manual_Centre_Yaw;
		Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * (kKey_Mech_Pitch/5);
		
		//键盘微调
		if(IF_KEY_PRESSED_W)//上
		{
			Cloud_Angle_Target[PITCH][MECH] -= 0.05f;
		}
		else if(IF_KEY_PRESSED_S)//下
		{
			Cloud_Angle_Target[PITCH][MECH] += 0.05f;
		}
	}
	
	if( IF_MOUSE_PRESSED_RIGH && Manual_Step == CONFIRM_HIGH
			&& Manual_Switch_Right == 1 )
	{
		Manual_High = fabs(Cloud_Angle_Measure[PITCH][MECH] - Manual_Centre_Pitch);
		Manual_Step = CONFIRM_LOCATION;//进入下一步确认位置
		Manual_Switch_Right = 0;
	}
	
	/*- 4 正式开始手打，注意云台方向 -*/
	if(Manual_Step == CONFIRM_LOCATION)
	{
		//鼠标粗调抬头补偿
		Manual_Pitch_Comp -= MOUSE_Y_MOVE_SPEED * (kKey_Mech_Pitch/10);
	}		
	
	if(Manual_Step == CONFIRM_LOCATION
			&& IF_KEY_PRESSED_W)//上
	{
		Cloud_Angle_Target[YAW][MECH]   = Manual_Centre_Yaw;
		Cloud_Angle_Target[PITCH][MECH] = Manual_Centre_Pitch - Manual_High + (-Manual_Pitch_Comp);
	}
	else if(Manual_Step == CONFIRM_LOCATION
				&& IF_KEY_PRESSED_S)//下
	{
		Cloud_Angle_Target[YAW][MECH]   = Manual_Centre_Yaw;
		Cloud_Angle_Target[PITCH][MECH] = Manual_Centre_Pitch + Manual_High + (-Manual_Pitch_Comp);
	}
	else if(Manual_Step == CONFIRM_LOCATION
				&& IF_KEY_PRESSED_A)//左
	{
		Cloud_Angle_Target[YAW][MECH]   = Manual_Centre_Yaw + Manual_Radius*YAW_POSITION;
		Cloud_Angle_Target[PITCH][MECH] = Manual_Centre_Pitch + (-Manual_Pitch_Comp);
	}
	else if(Manual_Step == CONFIRM_LOCATION
				&& IF_KEY_PRESSED_D)//右
	{
		Cloud_Angle_Target[YAW][MECH]   = Manual_Centre_Yaw - Manual_Radius*YAW_POSITION;
		Cloud_Angle_Target[PITCH][MECH] = Manual_Centre_Pitch + (-Manual_Pitch_Comp);
	}
	else if(Manual_Step == CONFIRM_LOCATION )//不按归中
	{
		Cloud_Angle_Target[YAW][MECH]   = Manual_Centre_Yaw;
		Cloud_Angle_Target[PITCH][MECH] = Manual_Centre_Pitch;
	}
}



/************************云台测量值更新及发送**********************/

/**
  * @brief  更新云台机械角度,can1中断中调用
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_UpdateAngle( char eAxis, int16_t angle )
{
	if (eAxis == PITCH)
	{
		angleMotorPit = angle;
		Cloud_Angle_Measure[PITCH][MECH]  = angleMotorPit;
	}
	else if (eAxis == YAW)
	{
		angleMotorYaw = angle;
		Cloud_Angle_Measure[YAW][MECH]  = angleMotorYaw;
	}
}

/**
  * @brief  更新云台姿态,500HZ,loop中调用
  * @param  void
  * @retval void
  * @attention 角度适度放大
  */
int js_ang_p = 0;
int js_pal_p = 0;
void GIMBAL_UpdatePalstance(void)
{		
	//读取陀螺仪  角度   角速度   
	mpu_dmp_get_data( &angleMpuRoll, &angleMpuPitch, &angleMpuYaw );
	MPU_Get_Gyroscope( &palstanceMpuPitch, &palstanceMpuRoll, &palstanceMpuYaw );
	
	#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
		//将陀螺仪角度放大,不放大则陀螺仪模式内环P要给很大,会不好调
		Cloud_Angle_Measure[PITCH][GYRO]  =  angleMpuPitch * 20;
	
		//角速度更新
		Cloud_Palstance_Measure[PITCH][MECH] = (palstanceMpuPitch + PALST_COMPS_PITCH);
		Cloud_Palstance_Measure[YAW][MECH]   = -(palstanceMpuYaw  + PALST_COMPS_YAW)*YAW_POSITION;
		
		Cloud_Palstance_Measure[PITCH][GYRO] = (palstanceMpuPitch + PALST_COMPS_PITCH);
		Cloud_Palstance_Measure[YAW][GYRO]   = -(palstanceMpuYaw  + PALST_COMPS_YAW);
		
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE	//主控翻面
		//将陀螺仪角度放大,不放大则陀螺仪模式内环P要给很大,会不好调
		Cloud_Angle_Measure[PITCH][GYRO]  =  angleMpuPitch * 20;
	
		//角速度更新
		Cloud_Palstance_Measure[PITCH][MECH] = (palstanceMpuPitch + PALST_COMPS_PITCH);
		Cloud_Palstance_Measure[YAW][MECH]   = -(palstanceMpuYaw  + PALST_COMPS_YAW)*YAW_POSITION;
		
		Cloud_Palstance_Measure[PITCH][GYRO] = (palstanceMpuPitch + PALST_COMPS_PITCH);
		Cloud_Palstance_Measure[YAW][GYRO]   = -(palstanceMpuYaw  + PALST_COMPS_YAW);
		
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
		//将陀螺仪角度放大,不放大则陀螺仪模式内环P要给很大,会不好调
		Cloud_Angle_Measure[PITCH][GYRO]  =  angleMpuPitch * 20;
	
		//角速度更新
		Cloud_Palstance_Measure[PITCH][MECH] = (palstanceMpuPitch + PALST_COMPS_PITCH);
		Cloud_Palstance_Measure[YAW][MECH]   = -(palstanceMpuYaw  + PALST_COMPS_YAW)*YAW_POSITION;
		
		Cloud_Palstance_Measure[PITCH][GYRO] = (palstanceMpuPitch + PALST_COMPS_PITCH);
		Cloud_Palstance_Measure[YAW][GYRO]   = -(palstanceMpuYaw  + PALST_COMPS_YAW);
		
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
		//将陀螺仪角度放大,不放大则陀螺仪模式内环P要给很大,会不好调
		Cloud_Angle_Measure[PITCH][GYRO]  =  angleMpuPitch * 20;
	
		//角速度更新
		Cloud_Palstance_Measure[PITCH][MECH] = (palstanceMpuPitch + PALST_COMPS_PITCH);
		Cloud_Palstance_Measure[YAW][MECH]   = -(palstanceMpuYaw  + PALST_COMPS_YAW)*YAW_POSITION;
		
		Cloud_Palstance_Measure[PITCH][GYRO] = (palstanceMpuPitch + PALST_COMPS_PITCH);
		Cloud_Palstance_Measure[YAW][GYRO]   = -(palstanceMpuYaw  + PALST_COMPS_YAW);
	
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
		//将陀螺仪角度放大,不放大则陀螺仪模式内环P要给很大,会不好调
		Cloud_Angle_Measure[PITCH][GYRO]  =  angleMpuPitch * 20;
	
		//角速度更新
		Cloud_Palstance_Measure[PITCH][MECH] = (palstanceMpuPitch + PALST_COMPS_PITCH);
		Cloud_Palstance_Measure[YAW][MECH]   = -(palstanceMpuYaw  + PALST_COMPS_YAW)*YAW_POSITION;
		
		Cloud_Palstance_Measure[PITCH][GYRO] = (palstanceMpuPitch + PALST_COMPS_PITCH);
		Cloud_Palstance_Measure[YAW][GYRO]   = -(palstanceMpuYaw  + PALST_COMPS_YAW);
	
	#endif
	
	Cloud_Angle_Measure[YAW][GYRO] = Gimbal_Yaw_Gryo_AngleSum(&Yaw_Gyro_Angle , (angleMpuYaw * 20));
	
	js_ang_p = angleMpuPitch * 1000;
	js_pal_p = palstanceMpuPitch * 1000;
}

/**
  * @brief  发送电流值给CAN1
  * @param  void
  * @retval void
  * @attention 注意一定是发送内环数据
  */
float gc_y = 0;
void GIMBAL_CanbusCtrlMotors(void)
{
	float fMotorOutput[2] = {0};
	
	#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
		//注意一定是发送内环数据,注意方向(正负号)
		fMotorOutput[PITCH] = -pidTermPit[INNER]*YAW_POSITION;
		if(modeGimbal == CLOUD_MECH_MODE)
		{
			fMotorOutput[YAW]   = -pidTermYaw[INNER][MECH]*YAW_POSITION;
		}
		else
		{
			fMotorOutput[YAW]   = -pidTermYaw[INNER][GYRO]*YAW_POSITION;
		}
	
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
		//注意一定是发送内环数据,注意方向(正负号)
		fMotorOutput[PITCH] = -pidTermPit[INNER]*YAW_POSITION;
		if(modeGimbal == CLOUD_MECH_MODE)
		{
			fMotorOutput[YAW]   = -pidTermYaw[INNER][MECH]*YAW_POSITION;
		}
		else
		{
			fMotorOutput[YAW]   = -pidTermYaw[INNER][GYRO]*YAW_POSITION;
		}
	
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
		//注意一定是发送内环数据,注意方向(正负号)
		fMotorOutput[PITCH] = -pidTermPit[INNER]*YAW_POSITION;
		if(modeGimbal == CLOUD_MECH_MODE)
		{
			fMotorOutput[YAW]   = -pidTermYaw[INNER][MECH]*YAW_POSITION;
		}
		else
		{
			fMotorOutput[YAW]   = -pidTermYaw[INNER][GYRO]*YAW_POSITION;
		};
	
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
		//注意一定是发送内环数据,注意方向(正负号)
		fMotorOutput[PITCH] = -pidTermPit[INNER]*YAW_POSITION;
		if(modeGimbal == CLOUD_MECH_MODE)
		{
			fMotorOutput[YAW]   = -pidTermYaw[INNER][MECH]*YAW_POSITION;
		}
		else
		{
			fMotorOutput[YAW]   = -pidTermYaw[INNER][GYRO]*YAW_POSITION;
		}
	
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
		//注意一定是发送内环数据,注意方向(正负号)
		fMotorOutput[PITCH] = -pidTermPit[INNER]*YAW_POSITION;
		if(modeGimbal == CLOUD_MECH_MODE)
		{
			fMotorOutput[YAW]   = -pidTermYaw[INNER][MECH]*YAW_POSITION;
		}
		else
		{
			fMotorOutput[YAW]   = -pidTermYaw[INNER][GYRO]*YAW_POSITION;
		}
		
	#endif
	gc_y = 	fMotorOutput[PITCH] ;
	CAN1_Cloud_QueueSend(fMotorOutput);   
}

/*****************************云台位置PID控制***********************************/

/**
  * @brief  pid计算
  * @param  void
  * @retval void
  * @attention 此处不能改变目标角度,只能用来做限幅和调用PID计算函数
  */
void GIMBAL_PositionLoop(void)
{
	if (modeGimbal == CLOUD_MECH_MODE)//机械模式
	{
		//pitch
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
		//pitch角度限制
		Cloud_Angle_Target[PITCH][MECH] = constrain_float( Cloud_Angle_Target[PITCH][MECH], Mech_Min_Pitch, Mech_Max_Pitch );
		vPitch_Mech_PositionLoop();
		
		//yaw
		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];//机械模式下实时记录陀螺仪角度,防止切换模式时甩头
		//yaw角度限制
		#if	YAW_POSITION == YAW_UP
			Cloud_Angle_Target[YAW][MECH] = constrain_float( Cloud_Angle_Target[YAW][MECH], Mech_Min_Yaw, Mech_Max_Yaw );
		#else
			Cloud_Angle_Target[YAW][MECH] = constrain_float( Cloud_Angle_Target[YAW][MECH], Mech_Max_Yaw,  Mech_Min_Yaw);
		#endif
		vYaw_Mech_PositionLoop();
	}
	
	else if(modeGimbal == CLOUD_GYRO_MODE)//陀螺仪模式
	{
		//pitch
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
		//pitch角度限制
		Cloud_Angle_Target[PITCH][MECH] = constrain_float( Cloud_Angle_Target[PITCH][MECH], Mech_Min_Pitch, Mech_Max_Pitch );
		vPitch_Mech_PositionLoop();//陀螺仪模式下PITCH用的仍是机械模式
		
		
		//yaw
		Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];//陀螺仪模式下实时记录机械角度,防止切换模式时甩头
		
		vYaw_Gyro_PositionLoop();//YAW陀螺仪模式输出
	}
}


/**
  * @brief  pitch机械模式
  * @param  void
  * @retval void
  * @attention 计算PID输出量
  */
int js_pid_p = 0;
void vPitch_Mech_PositionLoop(void)
{
	//角度误差
	Cloud_Angle_Error[PITCH][MECH] = Cloud_Angle_Target[PITCH][MECH] - Cloud_Angle_Measure[PITCH][MECH];
	//误差进行卡尔曼滤波,消除低频低幅度抖动
	Cloud_Angle_Error[PITCH][MECH] = KalmanFilter(&Gimbal_Pitch_Mech_Error_Kalman, Cloud_Angle_Error[PITCH][MECH]);
	//外环输出
	pTermPit[OUTER] = Cloud_Angle_Error[PITCH][MECH] * Cloud_Angle_kpid[PITCH][MECH][KP];
	pidTermPit[OUTER] = pTermPit[OUTER];
	pidTermPit[OUTER] = constrain_float(pidTermPit[OUTER],-PID_Outter_Max,PID_Outter_Max);
	
	//角速度误差
	Cloud_Palstance_Error[PITCH][MECH] = pidTermPit[OUTER] - Cloud_Palstance_Measure[PITCH][MECH];
	//内环输出
	pTermPit[INNER] = Cloud_Palstance_Error[PITCH][MECH] * Cloud_Palstance_kpid[PITCH][MECH][KP];
	iTermPit[INNER] += Cloud_Palstance_Error[PITCH][MECH] * Cloud_Palstance_kpid[PITCH][MECH][KI] * 0.002f;
	iTermPit[INNER] = constrain_float(iTermPit[INNER], -PID_Iterm_Max, PID_Iterm_Max);

	pidTermPit[INNER] = pTermPit[INNER] + iTermPit[INNER];
	pidTermPit[INNER] = constrain_float(pidTermPit[INNER], -PID_Out_Max, PID_Out_Max);
	js_pid_p = pidTermPit[INNER]*1000;
}

/**
  * @brief  pitch陀螺仪模式
  * @param  void
  * @retval void
  * @attention 计算PID输出量
  */
void vPitch_Gyro_PositionLoop(void)
{
	//角度误差
	Cloud_Angle_Error[PITCH][GYRO] = Cloud_Angle_Target[PITCH][GYRO] - Cloud_Angle_Measure[PITCH][GYRO];
	//误差进行卡尔曼滤波,消除低频低幅度抖动
	Cloud_Angle_Error[PITCH][GYRO] = KalmanFilter(&Gimbal_Pitch_Gyro_Error_Kalman, Cloud_Angle_Error[PITCH][GYRO]);
	//外环输出
	pTermPit[OUTER] = Cloud_Angle_Error[PITCH][GYRO] * Cloud_Angle_kpid[PITCH][GYRO][KP];
	pidTermPit[OUTER] = pTermPit[OUTER];
	pidTermPit[OUTER] = constrain_float(pidTermPit[OUTER],-PID_Outter_Max,PID_Outter_Max);
	
	//角速度误差
	Cloud_Palstance_Error[PITCH][GYRO] = pidTermPit[OUTER] - Cloud_Palstance_Measure[PITCH][GYRO];
	//内环输出
	pTermPit[INNER] = Cloud_Palstance_Error[PITCH][GYRO] * Cloud_Palstance_kpid[PITCH][GYRO][KP];
	iTermPit[INNER] += Cloud_Palstance_Error[PITCH][GYRO] * Cloud_Palstance_kpid[PITCH][GYRO][KI];
	pidTermPit[INNER] = constrain_float(pidTermPit[INNER], -PID_Out_Max, PID_Out_Max);
}

/**
  * @brief  yaw机械模式
  * @param  void
  * @retval void
  * @attention 计算PID输出量
  */
void vYaw_Mech_PositionLoop(void)
{
	//角度误差
	Cloud_Angle_Error[YAW][MECH] = Cloud_Angle_Target[YAW][MECH] - Cloud_Angle_Measure[YAW][MECH];
	//误差进行卡尔曼滤波,消除低频低幅度抖动
	Cloud_Angle_Error[YAW][MECH] = KalmanFilter(&Gimbal_Yaw_Mech_Error_Kalman, Cloud_Angle_Error[YAW][MECH]);
	//外环输出
	pTermYaw[OUTER][MECH] = Cloud_Angle_Error[YAW][MECH] * Cloud_Angle_kpid[YAW][MECH][KP];
	pidTermYaw[OUTER][MECH] = pTermYaw[OUTER][MECH];
	pidTermYaw[OUTER][MECH] = constrain_float(pidTermYaw[OUTER][MECH],-PID_Outter_Max,PID_Outter_Max);
	
	//角速度误差
	Cloud_Palstance_Error[YAW][MECH] = pidTermYaw[OUTER][MECH] - Cloud_Palstance_Measure[YAW][MECH];
	//内环输出
	pTermYaw[INNER][MECH]  = Cloud_Palstance_Error[YAW][MECH] * Cloud_Palstance_kpid[YAW][MECH][KP];
	iTermYaw[INNER][MECH] += Cloud_Palstance_Error[YAW][MECH] * Cloud_Palstance_kpid[YAW][MECH][KI] * 0.002f;
	iTermYaw[INNER][MECH]  = constrain_float(iTermYaw[INNER][MECH], -PID_Iterm_Max, PID_Iterm_Max);
	
	pidTermYaw[INNER][MECH] = pTermYaw[INNER][MECH] + iTermYaw[INNER][MECH];
	pidTermYaw[INNER][MECH] = constrain_float(pidTermYaw[INNER][MECH], -PID_Out_Max, PID_Out_Max);
}

/**
  * @brief  yaw陀螺仪模式
  * @param  void
  * @retval void
  * @attention 计算PID输出量
  */
void vYaw_Gyro_PositionLoop(void)
{
	//角度误差
	Cloud_Angle_Error[YAW][GYRO] = Cloud_Angle_Target[YAW][GYRO] - Cloud_Angle_Measure[YAW][GYRO];

	//误差进行卡尔曼滤波,消除低频低幅度抖动
	Cloud_Angle_Error[YAW][GYRO] = KalmanFilter(&Gimbal_Yaw_Gyro_Error_Kalman, Cloud_Angle_Error[YAW][GYRO]);

	//外环输出
	pTermYaw[OUTER][GYRO] = Cloud_Angle_Error[YAW][GYRO] * Cloud_Angle_kpid[YAW][GYRO][KP];
	pidTermYaw[OUTER][GYRO] = pTermYaw[OUTER][GYRO];
	pidTermYaw[OUTER][GYRO] = constrain_float(pidTermYaw[OUTER][GYRO],-PID_Outter_Max,PID_Outter_Max);
	
	//角速度误差
	Cloud_Palstance_Error[YAW][GYRO] = pidTermYaw[OUTER][GYRO] - Cloud_Palstance_Measure[YAW][GYRO];
	//内环输出
	pTermYaw[INNER][GYRO]  = Cloud_Palstance_Error[YAW][GYRO] * Cloud_Palstance_kpid[YAW][GYRO][KP];
	iTermYaw[INNER][GYRO] += Cloud_Palstance_Error[YAW][GYRO] * Cloud_Palstance_kpid[YAW][GYRO][KI] * 0.002f;
	iTermYaw[INNER][GYRO]  = constrain_float(iTermYaw[INNER][GYRO], -PID_Iterm_Max, PID_Iterm_Max);
	
	pidTermYaw[INNER][GYRO] = pTermYaw[INNER][GYRO] + iTermYaw[INNER][GYRO];
	pidTermYaw[INNER][GYRO] = constrain_float(pidTermYaw[INNER][GYRO], -PID_Out_Max, PID_Out_Max)*YAW_POSITION;
}

/****************************辅助函数***********************************************/

/**
  * @brief 临界值结构体初始化
  * @param  critical:临界值结构体指针
  *    get:当前读取到的角度（陀螺仪角或机械角度）
  * @retval void
  */
void Critical_Handle_Init(Critical_t *critical, float get)
{
	
	critical->AngleSum = get;//0;
	critical->CurAngle = get;
	critical->LastAngle = get;
	
	Cloud_Angle_Target[YAW][GYRO] = get; 

}

float Gimbal_Yaw_Gryo_AngleSum(Critical_t *critical, float get)
{
	critical->CurAngle = get - critical->LastAngle;	//当前陀螺仪角度减去上一次读取的陀螺仪角度，作为你的参考反馈值
	critical->LastAngle = get;
	/*  临界处理，保证每次经过零点反馈角度都是连续的  */
	if(critical->CurAngle < -3600)		//注意此处是陀螺仪角度放大后的临界值
		critical->CurAngle += 7200;
	if(critical->CurAngle > 3600)
		critical->CurAngle -= 7200;
	critical->AngleSum += critical->CurAngle;	
	
	return critical->AngleSum;				//返回新的反馈角度，作为PID算法的反馈值
}

/**
  * @brief  计算YAW偏离中心角度,底盘跟随模式用
  * @param  void
  * @retval sAngleError,偏离角度值,CAN反馈的机械角度
  */
int16_t GIMBAL_GetOffsetAngle(void)
{
	int16_t sAngleError = 0;

	sAngleError = (Cloud_Angle_Measure[YAW][MECH] - Mech_Mid_Yaw)*YAW_POSITION;


	//过零处理,统一成劣弧
	if (sAngleError > 8192 / 2)
	{
		return (sAngleError - 8192) ;
	}
	else if (sAngleError < -8192 / 2)
	{
		return (sAngleError + 8192);
	}
	else
	{
		return  sAngleError;
	}
}

/**
  * @brief  等待Pitch轴水平
  * @param  void
  * @retval 1回到水平位置,0未回到水平位置
  * @attention 角度小于50则认为回到水平
  */
uint8_t GIMBAL_IfPitchLevel(void)
{	  
	if ( fabs( Cloud_Angle_Measure[PITCH][MECH] - Mech_Mid_Pitch ) <= 50 )
	{
        return 1;
	}
		
	return 0;
}

/**
  * @brief  PITCH抬头判断
  * @param  void
  * @retval 是否抬头
  * @attention 用于摩擦轮开启
  */
uint8_t GIMBAL_IfPitchHigh(void)
{
	if (fabs(Cloud_Angle_Measure[PITCH][MECH] - CLOUD_FRIC_PIT_UP) <= 20)
	{
		return 1;
	}

	return 0;
}

/**
  * @brief  是否处于开弹仓模式
  * @param  void
  * @retval TRUE/FALSE
  * @attention 用于停止扭腰模式和45°对敌模式
  */
bool GIMBAL_IfGIMBAL_LEVEL(void)
{
	if (actGimbal == GIMBAL_LEVEL)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/**
  * @brief  是否开启打符模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfBuffHit(void)
{
    if (actGimbal == GIMBAL_BUFF || actGimbal == GIMBAL_SM_BUFF|| actGimbal == GIMBAL_MANUAL)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否开启打大符模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_If_Big_Buff(void)
{
    if (actGimbal == GIMBAL_BUFF)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否开启打小符模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_If_Small_Buff(void)
{
    if (actGimbal == GIMBAL_SM_BUFF)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否开启吊射模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_If_Base(void)
{
    if (actGimbal == GIMBAL_BASE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否开启手动打符模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfManulHit(void)
{
    if (actGimbal == GIMBAL_MANUAL)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否开启自瞄
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfAutoHit(void)
{
    if(actGimbal == GIMBAL_AUTO)//鼠标右键按下
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  自瞄yaw轴预测是否已经开启
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfAuto_MobPre_Yaw(void)
{
    if(actGimbal == GIMBAL_AUTO)//鼠标右键按下
	{
		return Mobility_Prediction_Yaw;//TRUE/FALSE
	}
	else//没开自瞄不可能有预测
	{
		return FALSE;
	}
}

/**
  * @brief  yaw轴开启预测的时候云台是否到位
  * @param  void
  * @retval TRUE到位可打弹   FALSE没到位禁止打弹
  * @attention 左右各有延迟，换向时记得清零反向和静止时的延迟
  */
bool GIMBAL_MOBPRE_YAW_FIRE(void)
{
	return Mobi_Pre_Yaw_Fire;
}

/**
  * @brief  打符yaw是否移动到位
  * @param  void
  * @retval TRUE到位可打弹   FALSE没到位禁止打弹
  * @attention 
  */
float debug_y_ready = 30;
float debug_pix_y = 0;
bool GIMBAL_BUFF_YAW_READY(void)
{
	debug_pix_y = fabs(gb_yaw_angle_gim + Buff_Yaw_Comp_Gimbal);
	if( (fabs(gb_yaw_posit_error) < debug_y_ready) 
			&& (VisionRecvData.yaw_angle != 0) 
				&& fabs(gb_yaw_angle_gim + Buff_Yaw_Comp_Gimbal) <= 35 )//(VisionRecvData.identify_buff == TRUE) )//识别到了目标
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  打符pitch是否移动到位
  * @param  void
  * @retval TRUE到位可打弹   FALSE没到位禁止打弹
  * @attention 
  */
float debug_p_ready = 30;
float debug_pix_p = 0;
bool GIMBAL_BUFF_PITCH_READY(void)
{
	debug_pix_p = fabs(gb_pitch_angle_gim + Buff_Pitch_Comp_Gimbal);
	if( (fabs(gb_pitch_posit_error) < debug_p_ready)
			&& (VisionRecvData.pitch_angle != 0) 
				&& fabs(gb_pitch_angle_gim + Buff_Pitch_Comp_Gimbal) <= 35)//(VisionRecvData.identify_buff == TRUE) )//识别到了目标
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否在自瞄哨兵
  * @param  void
  * @retval TRUE   FALSE
  * @attention 自瞄抬头角度太大则认为在打哨兵
  */
bool GIMBAL_AUTO_PITCH_SB(void)
{
	if( Cloud_Angle_Measure[PITCH][MECH] - Mech_Min_Pitch <= down_sb_pitch/*300*/ 
			|| IF_KEY_PRESSED_G)//抬头接近限位
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否在中等距离自瞄哨兵,加大预测
  * @param  void
  * @retval TRUE   FALSE
  * @attention 自瞄抬头角度太大则认为在打哨兵
  */
float pitch_sb_error = 0;
bool GIMBAL_AUTO_PITCH_SB_SK(void)
{
	pitch_sb_error = Cloud_Angle_Measure[PITCH][MECH] - Mech_Min_Pitch;
	if( (Cloud_Angle_Measure[PITCH][MECH] - Mech_Min_Pitch <= down_sb_pitch/*450*//*550*/)
			&& (Cloud_Angle_Measure[PITCH][MECH] - Mech_Min_Pitch > up_sb_pitch) )//抬头接近限位
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/**
  * @brief  云台抬头角度
  * @param  void
  * @retval 欧拉角
  * @attention 以最低点为0
  */
float GIMBAL_PITCH_Judge_Angle(void)
{
	float angle_pitch = 180;
	
	//计算欧拉角
	angle_pitch = -(Cloud_Angle_Measure[PITCH][MECH] - Mech_Max_Pitch)*1;///8192*360.f;
	
	return angle_pitch;
}

/**
  * @brief  云台吊射角度
  * @param  void
  * @retval pitch_mech_angle
  * @attention 
  */
int Base_Angle_Measure(void)
{
	return (int)Cloud_Angle_Measure[PITCH][MECH];
}

