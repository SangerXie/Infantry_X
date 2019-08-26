#include "Task_Chassis.h"
#include "math.h"

#include "control.h"
#include "remote.h"
#include "can1.h"
#include "Task_Gimbal.h"
#include "judge.h"
#include "magazine.h"
#include "super_cap.h"

#include "arm_math.h"
#include "led.h"
#include "kalman.h"



//底盘电流限幅
#define iTermChassis_Max             3000     //微分限幅

#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO 
	#define		Omni_Speed_Max            9000//7600     //底盘水平移动速度限幅,防止键盘模式下速度超过这个值
	#define		STANDARD_MAX_NORMAL       9000//7600     //平地开车最快速度，防止摇杆比例*660超过这个值
	#define		REVOLVE_MAX_NORMAL        9000//7600     //平地扭头最快速度
	#define     REVOLVE_KD                (235.f)
	#define     REVOLVE_ANGLE             35
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
	#define		Omni_Speed_Max            9000//7300//6800     //底盘水平移动速度限幅,防止键盘模式下速度超过这个值
	#define		STANDARD_MAX_NORMAL       9000//7300//6800     //平地开车最快速度，防止摇杆比例*660超过这个值
	#define		REVOLVE_MAX_NORMAL        9000//7300//6800    //平地扭头最快速度
	#define     REVOLVE_KD                (350.f)
	#define     REVOLVE_ANGLE             35
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
	#define 	Omni_Speed_Max            9000//9000     //底盘水平移动速度限幅,防止键盘模式下速度超过这个值
	#define		STANDARD_MAX_NORMAL       9000     //平地开车最快速度，防止摇杆比例*660超过这个值
	#define		REVOLVE_MAX_NORMAL        9000     //平地扭头最快速度
	#define     REVOLVE_KD                (350.f)
	#define     REVOLVE_ANGLE             35

#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
	#define		Omni_Speed_Max            9000     //底盘水平移动速度限幅,防止键盘模式下速度超过这个值
	#define		STANDARD_MAX_NORMAL       9000     //平地开车最快速度，防止摇杆比例*660超过这个值
	#define		REVOLVE_MAX_NORMAL        9000     //平地扭头最快速度
	#define     REVOLVE_KD                (350.f)
	#define     REVOLVE_ANGLE             35

#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
	#define 	Omni_Speed_Max            9000     //底盘水平移动速度限幅,防止键盘模式下速度超过这个值
	#define		STANDARD_MAX_NORMAL       9000     //平地开车最快速度，防止摇杆比例*660超过这个值
	#define		REVOLVE_MAX_NORMAL        9000     //平地扭头最快速度
	#define     REVOLVE_KD                (350.f)
	#define     REVOLVE_ANGLE             35

#endif

#define Omni_SupCap_Max				 10000		//电容放电情况下最大速度

//不同模式下,斜坡函数对应的时间值,一般用普通速度就行
#define    TIME_INC_NORMAL           10//6//4	  //键盘斜坡,越大增加速度越快,完成时间越短
#define    TIME_DEC_NORMAL           500//180        //键盘斜坡,越大减小速度越快(一般要比INC大一点,这样松开键盘能更快为0,太大则会造成底盘停下来的时候跳跃)

#define    TIME_INC_SLOW             1		  //补弹模式下速度变化快慢
#define    TIME_INC_SZUPUP           3		  //手动爬坡模式下速度变化快慢

#define    TIME_INC_SALTATION        1        //突然变速情况下速度变化快慢

#define    REVOLVE_SLOPE_NORMAL      80       //底盘普通模式斜坡,越大越快,完成时间越短
#define    REVOLVE_SLOPE_CORGI       50       //底盘扭屁股模式斜坡,越大越快,完成时间越短


//不同模式下的最高速度
#define    REVOLVE_MAX_CORGI         9000//5000     //底盘扭屁股最快速度,太大会让角度过大

#define    STANDARD_MAX_SLOW         1500//3000	  //补弹模式下水平移动速度
#define    REVOLVE_MAX_SLOW          2000//4000     //补弹模式下扭头速度

#define    STANDARD_MAX_SZUPUP       3000//5000//6000//4000//3600	  //手动爬坡模式下水平移动速度
#define    REVOLVE_MAX_SZUPUP        9000     //手动爬坡模式下扭头速度

#define    LIMIT_CHASSIS_MAX         9000     //功率限制情况下底盘单个电机最大输出
#define    CHAS_CURRENT_LIMIT        36000//40000    //四个轮子的速度总和最大值,单个输出*4,限功率调比例可用

//注意旋转和平移的比例要分开

/*************灵敏度************/
//机械模式下底盘比例系数,控制摇杆响应速度,如果过小也会限制最高转速,max = 此系数 *660
float kRc_Mech_Chassis_Standard, kRc_Mech_Chassis_Revolve;//平移，旋转,

//陀螺仪模式下底盘比例系数,控制摇杆响应速度,如果过小也会限制最高转速,max = 此系数 *660
float kRc_Gyro_Chassis_Standard, kRc_Gyro_Chassis_Revolve;//平移，旋转

//机械模式下底盘比例系数,控制键盘斜坡变化率
float kKey_Mech_Chassis_Standard, kKey_Mech_Chassis_Revolve;//平移，旋转

//陀螺仪模式下底盘比例系数,控制键盘斜坡变化率
float kKey_Gyro_Chassis_Standard, kKey_Gyro_Chassis_Revolve;//平移，旋转

/**************全向移动变量****************/
float Chassis_Move_X;//前后
float Chassis_Move_Y;//左右平移
float Chassis_Move_Z;//左右旋转


/***********PID参数***************/
//底盘期望速度
float Chassis_Speed_Target[4];//ID

//底盘测量速度
int16_t Chassis_Speed_Measure[4];

//底盘测量角度
int16_t Chassis_Angle_Measure[4];

//底盘测量速度
int16_t Chassis_Current_Measure[4];

//底盘速度误差
float Chassis_Speed_Error[4];//ID

//底盘速度误差和
float Chassis_Speed_Error_Sum[4];//ID
float Chassis_Speed_Error_NOW[4], Chassis_Speed_Error_LAST[4];

//陀螺仪模式下底盘偏差(相对YAW的中心分离机械角度)
int16_t Chassis_Gyro_Error;

//单级PID参数
float Chassis_Speed_kpid[4][3];//	motorID kp/ki/kd

float   pTermChassis[4], iTermChassis[4], dTermChassis[4];//ID
float	pidTermChassis[4];//ID,计算输出量

//底盘电机输出量
float Chassis_Final_Output[4];

//底盘扭头
float Chassis_Z_kpid[3] = {-11, 0, 5};
float pTermChassisZ;
float dTermChassisZ;

/**************限幅**************/
float Chassis_Final_Output_Max = LIMIT_CHASSIS_MAX;//限制PID运算最终输出值,底盘功率限制,根据电机转速实时变化
float Chassis_Standard_Move_Max;//底盘前后左右平移限速
float Chassis_Revolve_Move_Max;//底盘左右旋转限速,根据不同运动模式实时更改,所以不能用宏定义

float Chassis_Limit_Output_Max=LIMIT_CHASSIS_MAX;//底盘功率限幅


/**************斜坡***************/
float Slope_Chassis_Move_Z;//斜坡计算出的移动变量,这是目标输出量的实时斜坡值

uint16_t timeInc;//斜坡增加变化时间
uint16_t timeInc_Saltation;//前后方向突变下的斜坡增加量,比正常情况下要小
int16_t   timeXFron,    timeXBack,    timeYLeft,    timeYRigh;//键盘  s  w  d   a

//键盘模式下全向移动计算,斜坡量
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back; 
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;

//键盘模式下扭头斜坡,主要用在扭屁股模式中
float Slope_Chassis_Revolve_Move;


/**************裁判系统功率限制数据*****************/
#if JUDGE_VERSION == JUDGE_18
	float WARNING_REMAIN_POWER = 50;//裁判系统剩余焦耳能量低于这个数值则开始限功率,40扭屁股会超功率,平地开不会超
#elif JUDGE_VERSION == JUDGE_19
	float WARNING_REMAIN_POWER = 60;
#endif

float fChasCurrentLimit = CHAS_CURRENT_LIMIT;//限制4个轮子的速度总和
float fTotalCurrentLimit;//电流分配,平地模式下分配是均匀的


/***********底盘各类模式的一些辅助定义*************/

//扭屁股
bool Chass_Switch_F = 1;
u8 	 Chass_Key_F_Change = 0;

//45°
bool Chass_Switch_X = 1;
u8 	 Chass_Key_X_Change = 0;

//自动闪避
#define   MISS_MAX_TIME    1000    //自动闪避最大归位时间,单位2*ms
uint32_t  Miss_Mode_Time = 0;//自动闪避已过时长

//float  Corgi_XY_Compensation = 1.1;//扭屁股模式下的Z轴速度补偿


#define RADIUS     76      //麦轮半径
#define PERIMETER  478     //麦轮周长
#define WHEELTRACK 360//398     //左右轮距
#define WHEELBASE  300     //前后轴距
#define GIMBAL_X_OFFSET 0//75  //云台相对底盘中心的前后偏移量
#define GIMBAL_Y_OFFSET 0 //云台相对底盘中心的左右偏移量
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)  //电机减数比
#define RADIAN_COEF 57.3f  

extKalman_t Chassis_Error_Kalman;//定义一个kalman指针
//extKalman_t Chassis_Speed_Kalman[4];//定义一个kalman指针

/************************************************************************************/
/************************************************************************************/
extern bool Hurt_Data_Update;

eChassisCtrlMode  modeChassis;
eChassisAction  actChassis;


//每2ms执行一次任务函数，绝对延时
uint8_t remot_change = TRUE;
void Task_Chassis(void *pvParameters)
{
	portTickType currentTime;	
	
	for(;;)
	{	
		currentTime = xTaskGetTickCount();//当前系统时间
		Blue_On;
		/* 代码段 */
		if(SYSTEM_GetSystemState() == SYSTEM_STARTING)//初始化模式
		{
			CHASSIS_REST();
			actChassis = CHASSIS_NORMAL;
		}
		else
		{
			if (SYSTEM_GetRemoteMode() == RC) //遥控模式
			{
				CHAS_Rc_Ctrl();
				actChassis = CHASSIS_NORMAL;
				Chass_Switch_F = 1;//重置扭腰
				Chass_Key_F_Change = 0;//重置扭腰
				remot_change = TRUE;//标记切回了遥控模式
				Hurt_Data_Update = FALSE;//防止受到伤害切键盘模式时会动
			}
			else 								//键盘模式
			{
				CHAS_Key_Ctrl();
			}
		}
		
		Chassis_Omni_Move_Calculate();//移动计算

		Chassis_MotorOutput();//底盘PID计算
//		if(Cap_Out_Can_Open() == TRUE)
//		{}//电容放电，解除功率限制，防止电容充电消耗剩余能量造成速度减小
//		else
//		{
			Chassis_Power_Limit();//功率限制,电流重新分配
//		}
		CHASSIS_CANbusCtrlMotor();//队列填充
//		CHASSIS_StopMotor();
		
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//绝对延时
	}
}

/********************************************************************************/
/**
  * @brief  底盘参数初始化
  * @param  void
  * @retval void
  * @attention 
  */
void CHASSIS_InitArgument(void)
{
	/* 灵敏度 */
	kRc_Mech_Chassis_Standard = 14.f;//调节摇杆灵敏度,调节速度
	kRc_Mech_Chassis_Revolve  = 11.4f;//10.6;//调节机械模式摇杆扭头灵敏度(太小会影响最高速度)
	kRc_Gyro_Chassis_Standard = 14;//陀螺仪模式摇杆平移灵敏度
	kRc_Gyro_Chassis_Revolve  = -10;//10;//-7.1;//陀螺仪模式摇杆扭头灵敏度,太大会震荡
	
	kKey_Mech_Chassis_Revolve = 40;//键盘机械模式下扭头速度响应快慢,别太大,不然扭头太快
	kKey_Gyro_Chassis_Revolve = -10;//-8.1;//注意正负,键盘陀螺仪模式下底盘跟随云台转动的速度,别给太大,否则震荡会很严重
	
	/* 底盘PID参数 */
#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO 
	Chassis_Speed_kpid[LEFT_FRON_201][KP] = 8;
	Chassis_Speed_kpid[LEFT_FRON_201][KI] = 100;
	Chassis_Speed_kpid[LEFT_FRON_201][KD] = 10;
	
	Chassis_Speed_kpid[RIGH_FRON_202][KP] = 8;
	Chassis_Speed_kpid[RIGH_FRON_202][KI] = 100;
	Chassis_Speed_kpid[RIGH_FRON_202][KD] = 10;
	
	Chassis_Speed_kpid[LEFT_BACK_203][KP] = 8;
	Chassis_Speed_kpid[LEFT_BACK_203][KI] = 100;
	Chassis_Speed_kpid[LEFT_BACK_203][KD] = 10;
	
	Chassis_Speed_kpid[RIGH_BACK_204][KP] = 8;
	Chassis_Speed_kpid[RIGH_BACK_204][KI] = 100;
	Chassis_Speed_kpid[RIGH_BACK_204][KD] = 10;
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
	Chassis_Speed_kpid[LEFT_FRON_201][KP] = 3;//18;
	Chassis_Speed_kpid[LEFT_FRON_201][KI] = 0;//100;//0.08;
	Chassis_Speed_kpid[LEFT_FRON_201][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_FRON_202][KP] = 18;
	Chassis_Speed_kpid[RIGH_FRON_202][KI] = 100;//0.08;
	Chassis_Speed_kpid[RIGH_FRON_202][KD] = 0;
	
	Chassis_Speed_kpid[LEFT_BACK_203][KP] = 18;
	Chassis_Speed_kpid[LEFT_BACK_203][KI] = 100;//0.08;
	Chassis_Speed_kpid[LEFT_BACK_203][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_BACK_204][KP] = 3;
	Chassis_Speed_kpid[RIGH_BACK_204][KI] = 0;//100;//0.08;
	Chassis_Speed_kpid[RIGH_BACK_204][KD] = 0;
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
	Chassis_Speed_kpid[LEFT_FRON_201][KP] = 18;
	Chassis_Speed_kpid[LEFT_FRON_201][KI] = 100;//0.08;
	Chassis_Speed_kpid[LEFT_FRON_201][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_FRON_202][KP] = 18;
	Chassis_Speed_kpid[RIGH_FRON_202][KI] = 100;//0.08;
	Chassis_Speed_kpid[RIGH_FRON_202][KD] = 0;
	
	Chassis_Speed_kpid[LEFT_BACK_203][KP] = 18;
	Chassis_Speed_kpid[LEFT_BACK_203][KI] = 100;//0.08;
	Chassis_Speed_kpid[LEFT_BACK_203][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_BACK_204][KP] = 18;
	Chassis_Speed_kpid[RIGH_BACK_204][KI] = 100;//0.08;
	Chassis_Speed_kpid[RIGH_BACK_204][KD] = 0;
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
	Chassis_Speed_kpid[LEFT_FRON_201][KP] = 18;
	Chassis_Speed_kpid[LEFT_FRON_201][KI] = 100;//0.08;
	Chassis_Speed_kpid[LEFT_FRON_201][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_FRON_202][KP] = 18;
	Chassis_Speed_kpid[RIGH_FRON_202][KI] = 100;//0.08;
	Chassis_Speed_kpid[RIGH_FRON_202][KD] = 0;
	
	Chassis_Speed_kpid[LEFT_BACK_203][KP] = 18;
	Chassis_Speed_kpid[LEFT_BACK_203][KI] = 100;//0.08;
	Chassis_Speed_kpid[LEFT_BACK_203][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_BACK_204][KP] = 18;
	Chassis_Speed_kpid[RIGH_BACK_204][KI] = 100;//0.08;
	Chassis_Speed_kpid[RIGH_BACK_204][KD] = 0;
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
	Chassis_Speed_kpid[LEFT_FRON_201][KP] = 18;
	Chassis_Speed_kpid[LEFT_FRON_201][KI] = 100;//0.08;
	Chassis_Speed_kpid[LEFT_FRON_201][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_FRON_202][KP] = 18;
	Chassis_Speed_kpid[RIGH_FRON_202][KI] = 100;//0.08;
	Chassis_Speed_kpid[RIGH_FRON_202][KD] = 0;
	
	Chassis_Speed_kpid[LEFT_BACK_203][KP] = 18;
	Chassis_Speed_kpid[LEFT_BACK_203][KI] = 100;//0.08;
	Chassis_Speed_kpid[LEFT_BACK_203][KD] = 0;
	
	Chassis_Speed_kpid[RIGH_BACK_204][KP] = 18;
	Chassis_Speed_kpid[RIGH_BACK_204][KI] = 100;//0.08;
	Chassis_Speed_kpid[RIGH_BACK_204][KD] = 0;

#endif

	KalmanCreate(&Chassis_Error_Kalman, 1, 0);
	
	/* 限幅 */
	#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO 
		Chassis_Standard_Move_Max  = Omni_Speed_Max;//6800;//9000;//摇杆水平移动限幅
		Chassis_Revolve_Move_Max   = Omni_Speed_Max;//6800;//7000;//摇杆左右扭头限幅,可以稍微大一点,否则云台扭太快会导致云台撞到限位,太大又会导致到目标位置后晃动
	
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
		Chassis_Standard_Move_Max  = Omni_Speed_Max;//6800;//9000;//摇杆水平移动限幅
		Chassis_Revolve_Move_Max   = Omni_Speed_Max;//6800;//7000;//摇杆左右扭头限幅,可以稍微大一点,否则云台扭太快会导致云台撞到限位,太大又会导致到目标位置后晃动
	
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_TWO
		Chassis_Standard_Move_Max  = Omni_Speed_Max;//7300;//摇杆水平移动限幅
		Chassis_Revolve_Move_Max   = Omni_Speed_Max;//7300;//摇杆左右扭头限幅,可以稍微大一点,否则云台扭太快会导致云台撞到限位,太大又会导致到目标位置后晃动
	
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_THREE
		Chassis_Standard_Move_Max  = Omni_Speed_Max;//6800;//9000;//摇杆水平移动限幅
		Chassis_Revolve_Move_Max   = Omni_Speed_Max;//6800;//7000;//摇杆左右扭头限幅,可以稍微大一点,否则云台扭太快会导致云台撞到限位,太大又会导致到目标位置后晃动
	
	#elif	INFANTRY_DEBUG_ID == DEBUG_ID_FOUR
		Chassis_Standard_Move_Max  = Omni_Speed_Max;//6800;//9000;//摇杆水平移动限幅
		Chassis_Revolve_Move_Max   = Omni_Speed_Max;//6800;//7000;//摇杆左右扭头限幅,可以稍微大一点,否则云台扭太快会导致云台撞到限位,太大又会导致到目标位置后晃动

	#endif
}

/**
  * @brief  底盘失控停止
  * @param  void
  * @retval void
  * @attention 输出赋值为零,记得要把零发送出去
  */
void CHASSIS_StopMotor(void)
{
	Chassis_Final_Output[ 0 ] = 0;
	Chassis_Final_Output[ 1 ] = 0;
	Chassis_Final_Output[ 2 ] = 0;
	Chassis_Final_Output[ 3 ] = 0;

	CAN1_Chassis_Send( Chassis_Final_Output );
}

/**
  * @brief  底盘启动状态
  * @param  void
  * @retval void
  * @attention 状态值0
  */
void CHASSIS_REST(void)
{
	Slope_Chassis_Move_Z = 0;//扭屁股实时输出斜坡
	Chassis_Move_X = 0;
	Chassis_Move_Y = 0;
	Chassis_Move_Z = 0;
}

/******************************/
/**
  * @brief  遥控控制底盘移动
  * @param  void
  * @retval void
  * @attention 在此计算移动方向
  */
void CHAS_Rc_Ctrl(void)
{
	float k_rc_z = 1;//根据Z速度调节前后左右平移移速比
	
	if (IF_RC_SW2_DOWN)//S2下陀螺仪
	{
		modeChassis = CHASSIS_GYRO_MODE;
	}
	else
	{
		modeChassis = CHASSIS_MECH_MODE;
	}
	
	//扭头斜坡切换成普通模式(扭屁股模式斜坡复位)
	Slope_Chassis_Revolve_Move = REVOLVE_SLOPE_NORMAL;
	//移动速度限制
	Chassis_Standard_Move_Max = STANDARD_MAX_NORMAL;//平移限幅
	Chassis_Revolve_Move_Max  = REVOLVE_MAX_NORMAL;//扭头限幅
	
	//记录摇杆移动数据
	if(modeChassis == CHASSIS_MECH_MODE)//机械模式
	{	
		Chassis_Move_Z = constrain_float( kRc_Mech_Chassis_Revolve*RC_CH0_RLR_OFFSET, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);//旋转
		
		if(fabs(Chassis_Move_Z) > 800)//扭头速度越快,前后速度越慢,防止转弯半径过大
		{
			k_rc_z = ( (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) * (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) )
						/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
			
			k_rc_z = constrain_float(k_rc_z,0,1);
		}
		else
		{
			k_rc_z = 1;
		}
		
		Chassis_Move_X = constrain_float( kRc_Mech_Chassis_Standard*RC_CH3_LUD_OFFSET, -Chassis_Standard_Move_Max*k_rc_z, Chassis_Standard_Move_Max*k_rc_z);//前后
		
		Chassis_Move_Y = constrain_float( kRc_Mech_Chassis_Standard*RC_CH2_LLR_OFFSET, -Chassis_Standard_Move_Max*k_rc_z, Chassis_Standard_Move_Max*k_rc_z);//水平
	
	}
	else if(modeChassis == CHASSIS_GYRO_MODE)//陀螺仪模式
	{
		Chassis_Gyro_Error = GIMBAL_GetOffsetAngle( );//底盘相对YAW的偏差

		Chassis_Move_Z = Chassis_SpeedZ_PID(Chassis_Gyro_Error, kRc_Gyro_Chassis_Revolve);
	
		if(fabs(Chassis_Move_Z) > 800)//扭头速度越快,前后速度越慢,防止转弯半径过大
		{
			k_rc_z = ( (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) * (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) )
						/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
			
			k_rc_z = constrain_float(k_rc_z,0,1);
		}
		else
		{
			k_rc_z = 1;
		}
		Chassis_Move_X = constrain_float( kRc_Gyro_Chassis_Standard*RC_CH3_LUD_OFFSET, -Chassis_Standard_Move_Max * k_rc_z, Chassis_Standard_Move_Max * k_rc_z);//前后
			
		Chassis_Move_Y = constrain_float( kRc_Gyro_Chassis_Standard*RC_CH2_LLR_OFFSET, -Chassis_Standard_Move_Max * k_rc_z, Chassis_Standard_Move_Max * k_rc_z);//水平

	}
	
}

/**
  * @brief  键盘控制底盘移动
  * @param  void
  * @retval void
  * @attention 模式选择,进入某模式后记得写退出到普通模式的判断
  * 无按键按下会一直处于自动闪避模式,除了模式切换外的按键按下则处于模式切换选择模式
  */
void CHAS_Key_Ctrl(void)
{
	if(remot_change == TRUE)//刚从遥控模式切过来,默认为陀螺仪模式
	{
		modeChassis = CHASSIS_GYRO_MODE;
		remot_change = FALSE;
	}
	
	if(GIMBAL_IfBuffHit() == TRUE)//打符模式为机械
	{
		modeChassis = CHASSIS_MECH_MODE;
	}
	
	switch (actChassis)      //SB keil 有警告,键盘模式选择，测试限死是普通模式
	{
		/*------------------普通模式,进行模式切换判断-------------------*/	
		case CHASSIS_NORMAL:		
			Chassis_NORMAL_Mode_Ctrl();						
		break;
		
		/*------------------扭屁股模式-------------------*/			
		case CHASSIS_CORGI:	
			if(!IF_KEY_PRESSED_F)//F松开
			{
				Chass_Switch_F = 1;
			}
			
			if(IF_KEY_PRESSED_F && Chass_Switch_F == 1)
			{
				Chass_Switch_F = 0;
				Chass_Key_F_Change ++;
				Chass_Key_F_Change %= 2;
			}
			//可以前后移动,按下左右平移、QE、切换机械模式退出
			if(Chass_Key_F_Change)
			{
				modeChassis = CHASSIS_GYRO_MODE;//陀螺仪模式,底盘跟随云台动
				
				Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
				CHASSIS_CORGI_Mode_Ctrl( REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);
//				CHASSIS_CORGI_Mode_Ctrl_Time( REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);				
			}
			else
			{
				actChassis = CHASSIS_NORMAL;//退出扭屁股模式
			}
			
			//弹仓开启时要关闭扭腰
			if((IF_KEY_PRESSED_CTRL || GIMBAL_IfGIMBAL_LEVEL()==TRUE ) && !IF_KEY_PRESSED_F)
			{
				Chass_Switch_F = 1;
				Chass_Key_F_Change ++;
				Chass_Key_F_Change %= 2;
				actChassis = CHASSIS_NORMAL;//退出扭屁股模式
			}
			else if(IF_KEY_PRESSED_X)//可随时进入45°模式
			{
				Chass_Switch_F = 1;
				Chass_Key_F_Change ++;
				Chass_Key_F_Change %= 2;
				actChassis = CHASSIS_PISA;
			}			
		break;
			
		/*-------------打符模式-------------*/
		case CHASSIS_ROSHAN:
						
			if(GIMBAL_IfBuffHit( ) != TRUE)
			{
				actChassis = CHASSIS_NORMAL;//退出打符模式
				modeChassis = CHASSIS_GYRO_MODE;//切回陀螺仪模式
			}	
			else
			{
				modeChassis = CHASSIS_MECH_MODE;//打符底盘进入机械模式
				CHASSIS_REST();//目标速度置0
			}				
		break;
				
		/*---------------补弹模式,底盘低速----------------*/
		case CHASSIS_SLOW:
			if (Magazine_IfOpen() != TRUE)//弹仓关闭
			{
				actChassis = CHASSIS_NORMAL;//底盘退出补弹模式
			}
			else
			{
				modeChassis = CHASSIS_MECH_MODE;//补弹时底盘进入机械模式
			
				Chassis_Keyboard_Move_Calculate( STANDARD_MAX_SLOW, TIME_INC_SLOW );
				Chassis_Mouse_Move_Calculate( REVOLVE_MAX_SLOW );
			}
		break;
				
		/*-------------手动爬坡模式-------------*/
		case CHASSIS_SZUPUP:
			CHASSIS_SZUPUP_Mode_Ctrl();
		break;
			
		/*-------------自动闪避模式-------------*/
//		case CHASSIS_MISS:
//			CHASSIS_MISS_Mode_Ctrl();
//		break;
		
		/*-------------45°对敌模式-------------*/
		case CHASSIS_PISA:
			if(!IF_KEY_PRESSED_X)//F松开
			{
				Chass_Switch_X = 1;
			}
			
			if(IF_KEY_PRESSED_X && Chass_Switch_X == 1)
			{
				Chass_Switch_X = 0;
				Chass_Key_X_Change ++;
				Chass_Key_X_Change %= 2;
			}
			
			if(Chass_Key_X_Change)//前后退出
			{
				Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
				CHASSIS_PISA_Mode_Ctrl();
			}
			else
			{
				actChassis = CHASSIS_NORMAL;
			}
			
			//弹仓开启时要关闭
			if( (IF_KEY_PRESSED_CTRL 
					|| GIMBAL_IfGIMBAL_LEVEL()==TRUE 
						|| IF_KEY_PRESSED_W || IF_KEY_PRESSED_S) 
							&& !IF_KEY_PRESSED_X)
			{
				Chass_Switch_X = 1;
				Chass_Key_X_Change ++;
				Chass_Key_X_Change %= 2;
				actChassis = CHASSIS_NORMAL;//退出扭屁股模式
			}
			else if(IF_KEY_PRESSED_F)//可随时进入扭腰模式
			{
				Chass_Switch_X = 1;
				Chass_Key_X_Change ++;
				Chass_Key_X_Change %= 2;
				actChassis = CHASSIS_CORGI;
			}
		break;
	}
}

/*************************底盘键盘模式各类模式小函数****************************/

/**
  * @brief  键盘模式下底盘运动计算
  * @param  速度最大输出量    增加速度(最大293)
  * @retval void
  * @attention  键盘控制前后左右平移,平移无机械和陀螺仪模式之分
  *             需要获取时间来进行斜坡函数计算
  */
void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp )
{
	static portTickType  ulCurrentTime = 0;
	static uint32_t  ulDelay = 0;
	float k_rc_z = 1;//根据Z速度调节前后左右平移移速比
	
	Chassis_Standard_Move_Max = sMoveMax;//调整速度限幅,水平移动
	timeInc      = sMoveRamp;
	
	ulCurrentTime = xTaskGetTickCount();//当前系统时间
	
	if(fabs(Chassis_Move_Z) > 800)//扭头速度越快,前后速度越慢,防止转弯半径过大
	{
		k_rc_z = ( (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) * (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) )
					/ ( Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max );
		
		k_rc_z = constrain_float(k_rc_z,0,1);
	}
	else
	{
		k_rc_z = 1;
	}
	
	if (ulCurrentTime >= ulDelay)//每10ms变化一次斜坡量
	{
		ulDelay = ulCurrentTime + TIME_STAMP_10MS;


		if(actChassis == CHASSIS_NORMAL && !KEY_PRESSED_OFFSET_SHIFT)//只有一般模式下才判断速度突变情况,防止打滑
		{
			if (IF_KEY_PRESSED_W)//等超级电容出来再测试电容放电时是否要全力加速
			{
				timeXBack = 0;//按下前进则后退斜坡归零,方便下次计算后退斜坡
				//前进X是正数,可在此加入超级电容按键判断
				if( Chassis_Move_X < sMoveMax/2.5 )//转向突变,刚开始的一小段时间斜坡降低,防止轮子打滑浪费功率
				{//利用速度是否到最大速度的1/5来判断不知道是否合理
					timeInc_Saltation = TIME_INC_SALTATION;//以爬坡模式处理速度方向突变
				}
				else			//已经过了打滑时间且轮子有了一定的速度
				{
					timeInc_Saltation = sMoveRamp;
				}
			}

			if (IF_KEY_PRESSED_S)
			{
				timeXFron = 0;//同理
				//后退X是负
				if( Chassis_Move_X > (-sMoveMax)/2.5 )//转向突变,刚开始的一小段时间斜坡降低,防止轮子打滑浪费功率
				{
					timeInc_Saltation = TIME_INC_SALTATION;//以爬坡模式处理速度方向突变
				}
				else			//已经过了打滑时间且轮子有了一定的速度
				{
					timeInc_Saltation = sMoveRamp;
				}
			}

			if (IF_KEY_PRESSED_D)
			{
				timeYRigh = 0;
			}

			if (IF_KEY_PRESSED_A)
			{
				timeYLeft = 0;
			}
		
			//键盘模式下全向移动,斜坡量计算,注意正负,最大输出量*斜坡比例得到缓慢增加的值,模拟摇杆
			//前后的增加斜坡是变化的
			Slope_Chassis_Move_Fron = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_W, &timeXFron, timeInc_Saltation, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Back = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_S, &timeXBack, timeInc_Saltation, TIME_DEC_NORMAL ) );

			//左右的增加斜坡跟前后不一样,别搞错
			Slope_Chassis_Move_Left = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_A, &timeYRigh, timeInc/1.5, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Righ = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_D, &timeYLeft, timeInc/1.5, TIME_DEC_NORMAL ) );


			Chassis_Move_X  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z;//前后计算
			Chassis_Move_Y  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;//左右计算
		}
		else		//其他模式不需要进行速度方向突变特殊处理
		{
			if (IF_KEY_PRESSED_W)
			{
				timeXBack = 0;//按下前进则后退斜坡归零,方便下次计算后退斜坡
			}

			if (IF_KEY_PRESSED_S)
			{
				timeXFron = 0;//同理
			}

			if (IF_KEY_PRESSED_D)
			{
				timeYRigh = 0;
			}

			if (IF_KEY_PRESSED_A)
			{
				timeYLeft = 0;
			}
			
			Slope_Chassis_Move_Fron = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_W, &timeXFron, timeInc, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Back = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_S, &timeXBack, timeInc, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Left = (int16_t)( -Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_A, &timeYRigh, timeInc, TIME_DEC_NORMAL ) );

			Slope_Chassis_Move_Righ = (int16_t)( Chassis_Standard_Move_Max * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_D, &timeYLeft, timeInc, TIME_DEC_NORMAL ) );

			if(actChassis != CHASSIS_CORGI)
			{
				Chassis_Move_X  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z;//前后计算
				Chassis_Move_Y  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;//左右计算
			}
		}
	}
}

/**
  * @brief  鼠标控制底盘旋转,键盘QEC控制快速转圈
  * @param  速度最大输出量 
  * @retval void
  * @attention  鼠标控制左右旋转
  */
void Chassis_Mouse_Move_Calculate( int16_t sRevolMax )
{
//	static int16_t sErrorPrev = 0;//上次偏离误差
	int16_t sErrorReal = 0;//yaw偏离中心误差
	
	
	Chassis_Revolve_Move_Max = sRevolMax;//左右旋转限速
	
	if(modeChassis == CHASSIS_GYRO_MODE)//陀螺仪模式
	{
		sErrorReal = GIMBAL_GetOffsetAngle();//获取实时偏差,用于扭屁股下的底盘位置补偿
		Chassis_Move_Z = Chassis_SpeedZ_PID(sErrorReal, kKey_Gyro_Chassis_Revolve);
	}
	else     //机械模式
	{
		Chassis_Move_Z = constrain_float( MOUSE_X_MOVE_SPEED*kKey_Mech_Chassis_Revolve, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);
	}
}

/**
  * @brief  底盘键盘模式选择,按键响应
  * @param  void
  * @retval void
  * @attention 底盘键盘控制状态下的所有模式切换都在这
  */
void Chassis_NORMAL_Mode_Ctrl(void)
{
	if(!IF_KEY_PRESSED_F)//F松开
	{
		Chass_Switch_F = 1;
	}
	
	if(!IF_KEY_PRESSED_X)//X松开
	{
		Chass_Switch_X = 1;
	}
	
	if (IF_KEY_PRESSED_F && !IF_KEY_PRESSED_CTRL 
			&& GIMBAL_IfBuffHit() == FALSE && Chass_Switch_F == 1)//F按下,切换成扭屁股(不用一直按F)
	{
		Chass_Switch_F = 0;
		Chass_Key_F_Change ++;
		Chass_Key_F_Change %= 2;
		actChassis = CHASSIS_CORGI;//记得写个能退出扭屁股模式的函数
	}
	else if(Magazine_IfOpen() == TRUE)//弹仓开启,进入补弹模式
	{
		actChassis = CHASSIS_SLOW;
	}
	else if(IF_KEY_PRESSED_W && IF_KEY_PRESSED_CTRL)//W Ctrl一起按进入爬坡模式
	{
		actChassis = CHASSIS_SZUPUP;//爬坡模式
	}
//	else if(JUDGE_IfArmorHurt() == TRUE && GIMBAL_IfBuffHit() == FALSE)//只要伤害更新且无按键按下,就会进入自动闪避模式
//	{
//		actChassis = CHASSIS_MISS;//打符时关闭自动闪避
//	}
	else if(GIMBAL_IfBuffHit() == TRUE)//打符模式
	{
		actChassis = CHASSIS_ROSHAN;
	}
	else if (IF_KEY_PRESSED_X && !IF_KEY_PRESSED_CTRL 
			&& GIMBAL_IfBuffHit() == FALSE && Chass_Switch_X == 1)//x按下,切换成45°
	{
		Chass_Switch_X = 0;
		Chass_Key_X_Change ++;
		Chass_Key_X_Change %= 2;
		actChassis = CHASSIS_PISA;
	}
	else 					
	{		
		//打符时强制进入机械模式
		if(IF_KEY_PRESSED_CTRL || GIMBAL_IfBuffHit() == TRUE)      //按住CTRL进入机械模式
		{
			modeChassis = CHASSIS_MECH_MODE;
		}
		else			//松开CTRL进入陀螺仪模式
		{
			modeChassis = CHASSIS_GYRO_MODE;
		}

		//移动速度控制
		if(Cap_Out_Can_Open() == TRUE)//电容放电
		{
			Chassis_Keyboard_Move_Calculate(Omni_SupCap_Max, TIME_INC_NORMAL);//放电时最大速度要变
			Chassis_Mouse_Move_Calculate(REVOLVE_MAX_NORMAL);
		}
		else
		{
			Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);//设置速度最大值与斜坡时间
			Chassis_Mouse_Move_Calculate(REVOLVE_MAX_NORMAL);			
		}
	}	
}

/**
  * @brief  扭屁股模式(位置不变版)
  * @param  速度最大输出量    增加到最大量所需时间
  * @retval void
  * @attention  不管时间，扭到位了就换向
  */
//扭屁股换向选择
#define    CORGI_BEGIN    0    
#define    CORGI_LEFT     1
#define    CORGI_RIGH     2

uint16_t  stateCorgi = CORGI_BEGIN;//标记往哪扭,默认不扭
bool    IfCorgiChange = FALSE;//是否扭到了另一边
int16_t  corgi_angle_target = 0;//左右目标角度
void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp)
{
	int16_t  sAngleError   = 0;
	float    vectorXBuffer = 0;
	float    vectorYBuffer = 0;
	float    angle         = 0;


	Chassis_Revolve_Move_Max = sRevolMax;//最大速度设置
	Slope_Chassis_Revolve_Move = sRevolRamp;//扭头斜坡设置

	sAngleError = GIMBAL_GetOffsetAngle();//计算yaw中心偏离,保证底盘扭屁股的时候也能跟随云台动

	//计算角度偏差,机械角度转换成欧拉角,用于前进速度补偿
	angle = -(float)sAngleError / (float)8192 * 6.283f;

	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//暂存实时X变化
	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;

	Chassis_Move_X = vectorXBuffer * cos( angle ) - vectorYBuffer * sin( angle );
	Chassis_Move_Y = vectorXBuffer * sin( angle ) + vectorYBuffer * cos( angle );

	//秘技:反复横跳......
	switch (stateCorgi)
	{
		case CORGI_BEGIN:	//以后可以试试用个随机(标志位不停取反),来让开始扭头的方向随机	  
			corgi_angle_target = -900;//可改最大移动角度,自动闪避模式下被击打时的扭腰角度
			IfCorgiChange = FALSE;
			stateCorgi    = CORGI_LEFT;
		break;
		
		case CORGI_LEFT:
			corgi_angle_target = -1024;//可改最大移动角度			
			IfCorgiChange = FALSE;

			if (sAngleError < -700)//角度误差大于700
			{
					stateCorgi = CORGI_RIGH;
				  IfCorgiChange = TRUE;//标记可以换向
			}			
		break;
			
		case CORGI_RIGH:		
			corgi_angle_target = 1024;
			IfCorgiChange = FALSE;

			if (sAngleError > 700)//角度误差大于700
			{
				stateCorgi = CORGI_LEFT;
				IfCorgiChange = TRUE;//标记可以换向
			}			
		break;
	}

	Chassis_Move_Z = Chassis_SpeedZ_PID( (sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
}

/**
  * @brief  扭屁股模式(时间不变版)
  * @param  速度最大输出量    增加到最大量所需时间
  * @retval void
  * @attention  不管扭没扭到位，时间到了就换向
  */
void CHASSIS_CORGI_Mode_Ctrl_Time(int16_t sRevolMax, int16_t sRevolRamp)
{
	static uint32_t twist_count;
	static float  chass_yaw_set;
	float    angle         = 0;
	int16_t  sAngleError   = 0;
	float    vectorXBuffer = 0;
	float    vectorYBuffer = 0;
	
	
	static int16_t twist_period = 800;//500*2ms为一个扭腰周期
	static int16_t twist_angle  = 40;//左右最大分离欧拉角度设置
	
	
	twist_count++; 

	Chassis_Revolve_Move_Max = sRevolMax;//最大速度设置
	Slope_Chassis_Revolve_Move = sRevolRamp;//扭头斜坡设置

	sAngleError = GIMBAL_GetOffsetAngle();//计算yaw中心偏离,保证底盘扭屁股的时候也能跟随云台动

	//计算角度偏差,机械角度转换成欧拉角,用于前进速度补偿
	angle = -(float)sAngleError / (float)8192 * 360;//6.283f;

	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//暂存实时X变化
	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;
	
	//正弦函数扭屁股
	Chassis_Move_X = vectorXBuffer * cos( angle/57.3f ) - vectorYBuffer * sin( angle/57.3f );
	Chassis_Move_Y = vectorXBuffer * sin( angle/57.3f ) + vectorYBuffer * cos( angle/57.3f );

	chass_yaw_set = twist_angle * sin(2 * PI / twist_period * twist_count);//计算底盘目标分离角度
	Chassis_Move_Z = -Chassis_Z_Corgi(angle, chass_yaw_set);
}

/**
  * @brief  手动爬坡模式
  * @param  void
  * @retval void
  * @attention  
  */
void CHASSIS_SZUPUP_Mode_Ctrl(void)
{
	if( !IF_KEY_PRESSED_W || !IF_KEY_PRESSED_CTRL)//松开任意一个退出爬坡模式
	{
		actChassis = CHASSIS_NORMAL;//底盘退出爬坡模式
	}
	else
	{
		modeChassis = CHASSIS_GYRO_MODE;//陀螺仪模式
		
		Chassis_Keyboard_Move_Calculate( STANDARD_MAX_SZUPUP, TIME_INC_SZUPUP );
		Chassis_Mouse_Move_Calculate( REVOLVE_MAX_SZUPUP );
	}
}

/**
  * @brief  自动闪避模式
  * @param  void
  * @retval void
  * @attention  
  */
void CHASSIS_MISS_Mode_Ctrl(void)
{
	int16_t  sAngleError   = 0;
	sAngleError = GIMBAL_GetOffsetAngle();//计算yaw中心偏离,保证底盘扭屁股的时候也能跟随云台动

	//有按键按下或者长时间没再受到攻击则退出自动闪避,保证操作跟手
	if( IF_KEY_PRESSED || Miss_Mode_Time > MISS_MAX_TIME )
	{
		actChassis = CHASSIS_NORMAL;//底盘切换成正常模式
		Miss_Mode_Time = 0;
	}
	else
	{
		modeChassis = CHASSIS_GYRO_MODE;
		
		if(JUDGE_IfArmorHurt() == TRUE 	//装甲板数据更新,即受到新的伤害
			|| IfCorgiChange == FALSE)  //屁股没有扭到旁边
		{
			//开启扭屁股一次
			CHASSIS_CORGI_Mode_Ctrl( REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);
			Miss_Mode_Time = 0;
		}
		else
		{
			Slope_Chassis_Move_Z = 0;//扭屁股实时输出斜坡
			Chassis_Move_X = 0;
			Chassis_Move_Y = 0;
			Chassis_Move_Z = Chassis_SpeedZ_PID( (sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
			Miss_Mode_Time++;
		}	
	}
}

/**
  * @brief  45°模式
  * @param  void
  * @retval void
  * @attention  
  */
void CHASSIS_PISA_Mode_Ctrl(void)
{
	int16_t  corgi_angle_target = 0;//左右目标角度
	int16_t  sAngleError   = 0;
	float    vectorXBuffer = 0;
	float    vectorYBuffer = 0;
	float    angle         = 0;

	sAngleError = GIMBAL_GetOffsetAngle();//计算yaw中心偏离,保证底盘扭屁股的时候也能跟随云台动

	//计算角度偏差,机械角度转换成欧拉角,用于前进速度补偿
	angle = -(float)sAngleError / (float)8192 * 6.283f;

	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//暂存实时X变化
	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;

	Chassis_Move_X = vectorXBuffer * cos( angle ) - vectorYBuffer * sin( angle );
	Chassis_Move_Y = vectorXBuffer * sin( angle ) + vectorYBuffer * cos( angle );

	corgi_angle_target = -1024;//可改最大移动角度

	Chassis_Move_Z = Chassis_SpeedZ_PID( (sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
}

/*******************底盘电机数据更新,CAN1中断中调用*********************/

/**
  * @brief  获取电机角度
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void CHASSIS_UpdateMotorAngle( eChassisWheel eWheel, int16_t angle )
{
    Chassis_Angle_Measure[ eWheel ] = angle;
}

/**
  * @brief  获取电机转速
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void CHASSIS_UpdateMotorSpeed( eChassisWheel eWheel, int16_t speed )
{
	Chassis_Speed_Measure[ eWheel ] = speed;
}

/**
  * @brief  获取电机转矩电流
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void CHASSIS_UpdateMotorCur( eChassisWheel eWheel, int16_t current )
{
	Chassis_Current_Measure[ eWheel ] = current;
}

/**
  * @brief  发送底盘电流值
  * @param  void
  * @retval void
  * @attention 
  */
void CHASSIS_CANbusCtrlMotor(void)
{	
	CAN1_Chassis_QueueSend(Chassis_Final_Output);
}

/**********************底盘输出计算**************************/

/**
  * @brief  底盘全向算法,计算各电机转速
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后)
  *              	X前(+)后(-)     Y左(-)右(+)     Z扭头
  */
void Chassis_Omni_Move_Calculate(void)
{
	/* OLD */
//	float speed_max;

//	//全向算法
//	Chassis_Speed_Target[LEFT_FRON_201] = +( +Chassis_Move_Y +Chassis_Move_X +Chassis_Move_Z );
//	Chassis_Speed_Target[RIGH_FRON_202] = -( -Chassis_Move_Y +Chassis_Move_X -Chassis_Move_Z );
//	Chassis_Speed_Target[LEFT_BACK_203] = +( -Chassis_Move_Y +Chassis_Move_X +Chassis_Move_Z );
//	Chassis_Speed_Target[RIGH_BACK_204] = -( +Chassis_Move_Y +Chassis_Move_X -Chassis_Move_Z );
//	
//	//限幅
//	if(Cap_Out_Can_Open() == TRUE)//电容放电
//	{
//		speed_max = Omni_SupCap_Max;
//	}
//	else
//	{
//		speed_max = Omni_Speed_Max;
//	}
//	
//	Chassis_Speed_Target[LEFT_FRON_201] = constrain_float( Chassis_Speed_Target[LEFT_FRON_201], -speed_max, speed_max);
//	Chassis_Speed_Target[RIGH_FRON_202] = constrain_float( Chassis_Speed_Target[RIGH_FRON_202], -speed_max, speed_max);
//	Chassis_Speed_Target[LEFT_BACK_203] = constrain_float( Chassis_Speed_Target[LEFT_BACK_203], -speed_max, speed_max);
//	Chassis_Speed_Target[RIGH_BACK_204] = constrain_float( Chassis_Speed_Target[RIGH_BACK_204], -speed_max, speed_max);

//	if(actChassis == CHASSIS_SZUPUP)//爬坡前轮会打滑，所以限得比后轮要小
//	{
//		//前轮限幅根据实际情况做调整，后轮不用管
//		Chassis_Speed_Target[LEFT_FRON_201] = constrain_float( Chassis_Speed_Target[LEFT_FRON_201], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
//		Chassis_Speed_Target[RIGH_FRON_202] = constrain_float( Chassis_Speed_Target[RIGH_FRON_202], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
//		Chassis_Speed_Target[LEFT_BACK_203] = constrain_float( Chassis_Speed_Target[LEFT_BACK_203], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
//		Chassis_Speed_Target[RIGH_BACK_204] = constrain_float( Chassis_Speed_Target[RIGH_BACK_204], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
//	}

	/* NEW */
	static float rotate_ratio_fl;//前左
	static float rotate_ratio_fr;//前右
  	static float rotate_ratio_bl;//后左
  	static float rotate_ratio_br;//后右
  	static float wheel_rpm_ratio;
	
	float speed_max;

	if(1)//云台不在底盘中心
	{
		rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET)/RADIAN_COEF;
		rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET)/RADIAN_COEF;
		rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET)/RADIAN_COEF;
		rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET)/RADIAN_COEF;
	}
	else
	{
		rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
		rotate_ratio_fl = rotate_ratio_fr;
		rotate_ratio_bl = rotate_ratio_fr;
		rotate_ratio_br = rotate_ratio_fr;
	}
	wheel_rpm_ratio = 60.0f/(PERIMETER * CHASSIS_DECELE_RATIO);             //	60/周长*减数比
	
	//全向算法
	Chassis_Speed_Target[LEFT_FRON_201] = +( +Chassis_Move_Y/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_X/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_Z / ( (((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF) * (60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) )*rotate_ratio_fl ) * wheel_rpm_ratio;
	Chassis_Speed_Target[RIGH_FRON_202] = -( -Chassis_Move_Y/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_X/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) -Chassis_Move_Z / ( (((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF) * (60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) )*rotate_ratio_fr ) * wheel_rpm_ratio;
	Chassis_Speed_Target[LEFT_BACK_203] = +( -Chassis_Move_Y/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_X/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_Z / ( (((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF) * (60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) )*rotate_ratio_bl ) * wheel_rpm_ratio;
	Chassis_Speed_Target[RIGH_BACK_204] = -( +Chassis_Move_Y/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) +Chassis_Move_X/(60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) -Chassis_Move_Z / ( (((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF) * (60.0f/(PERIMETER * CHASSIS_DECELE_RATIO)) )*rotate_ratio_br ) * wheel_rpm_ratio;
	
	//限幅
	if(Cap_Out_Can_Open() == TRUE)//电容放电
	{
		speed_max = Omni_SupCap_Max;
	}
	else
	{
		speed_max = Omni_Speed_Max;
	}
	
//	float   max = 0;
//	//find max item
//	for (uint8_t i = 0; i < 4; i++)
//	{
//		if (abs(Chassis_Speed_Target[i]) > max)
//		max = abs(Chassis_Speed_Target[i]);
//	}
//	//equal proportion
//	if (max > speed_max)
//	{
//		float rate = speed_max / max;
//		for (uint8_t i = 0; i < 4; i++)
//			Chassis_Speed_Target[i] *= rate;
//	}
	
	Chassis_Speed_Target[LEFT_FRON_201] = constrain_float( Chassis_Speed_Target[LEFT_FRON_201], -speed_max, speed_max);
	Chassis_Speed_Target[RIGH_FRON_202] = constrain_float( Chassis_Speed_Target[RIGH_FRON_202], -speed_max, speed_max);
	Chassis_Speed_Target[LEFT_BACK_203] = constrain_float( Chassis_Speed_Target[LEFT_BACK_203], -speed_max, speed_max);
	Chassis_Speed_Target[RIGH_BACK_204] = constrain_float( Chassis_Speed_Target[RIGH_BACK_204], -speed_max, speed_max);

	if(actChassis == CHASSIS_SZUPUP)//爬坡前轮会打滑，所以限得比后轮要小
	{
		//前轮限幅根据实际情况做调整，后轮不用管
		Chassis_Speed_Target[LEFT_FRON_201] = constrain_float( Chassis_Speed_Target[LEFT_FRON_201], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
		Chassis_Speed_Target[RIGH_FRON_202] = constrain_float( Chassis_Speed_Target[RIGH_FRON_202], -STANDARD_MAX_SZUPUP, STANDARD_MAX_SZUPUP);
		Chassis_Speed_Target[LEFT_BACK_203] = constrain_float( Chassis_Speed_Target[LEFT_BACK_203], -Omni_Speed_Max, Omni_Speed_Max);
		Chassis_Speed_Target[RIGH_BACK_204] = constrain_float( Chassis_Speed_Target[RIGH_BACK_204], -Omni_Speed_Max, Omni_Speed_Max);
	}
}

/**
  * @brief  底盘电机PID计算,单级
  * @param  电机ID
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后)
  */
void Chassis_Motor_Speed_PID( eChassisWheel eWheel ) 
{
	//计算速度误差
	Chassis_Speed_Error[eWheel] = Chassis_Speed_Target[eWheel] - Chassis_Speed_Measure[eWheel];
//	Chassis_Speed_Error[eWheel] = KalmanFilter(&Chassis_Speed_Kalman[eWheel], Chassis_Speed_Error[eWheel]);
	Chassis_Speed_Error_Sum[eWheel] += Chassis_Speed_Error[eWheel];
	
	pTermChassis[eWheel] =  Chassis_Speed_Error[eWheel]*Chassis_Speed_kpid[eWheel][KP];
	iTermChassis[eWheel] =  Chassis_Speed_Error_Sum[eWheel]*Chassis_Speed_kpid[eWheel][KI] * 0.002f;
	//积分限幅
	iTermChassis[eWheel] = constrain_float(iTermChassis[eWheel],-iTermChassis_Max,iTermChassis_Max);
	
	Chassis_Speed_Error_NOW[eWheel] = Chassis_Speed_Error[eWheel];
	dTermChassis[eWheel] = (Chassis_Speed_Error_NOW[eWheel] - Chassis_Speed_Error_LAST[eWheel])*Chassis_Speed_kpid[eWheel][KD];
	Chassis_Speed_Error_LAST[eWheel] = Chassis_Speed_Error_NOW[eWheel];
	
	//积分项缓慢减小,防止误差为0时突然失力
	if( pTermChassis[eWheel] * iTermChassis[eWheel] < 0 )
	{
		Chassis_Speed_Error_Sum[eWheel] = constrain_float(Chassis_Speed_Error_Sum[eWheel],
															-(iTermChassis_Max/Chassis_Speed_kpid[eWheel][KI]/5.f),
															 (iTermChassis_Max/Chassis_Speed_kpid[eWheel][KI]/5.f));
	}
	
	pidTermChassis[eWheel] = pTermChassis[eWheel] + iTermChassis[eWheel] + dTermChassis[eWheel];

	pidTermChassis[eWheel] = constrain_float(pidTermChassis[eWheel],-Chassis_Final_Output_Max,Chassis_Final_Output_Max);	
	
	//记录输出电流
	Chassis_Final_Output[eWheel] = pidTermChassis[eWheel];
}

/**
  * @brief  分别对4个电机做PID运算,计算最终输出电流(发送给电调的值)
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后)
  */
void Chassis_MotorOutput(void)
{
	Chassis_Motor_Speed_PID(LEFT_FRON_201);
	Chassis_Motor_Speed_PID(RIGH_FRON_202);
	Chassis_Motor_Speed_PID(LEFT_BACK_203);
	Chassis_Motor_Speed_PID(RIGH_BACK_204);
}


float Chassis_Z_Speed_PID(void)
{
	static float error[2];
	
	error[NOW] = Chassis_Gyro_Error;
	
	Chassis_Move_Z = error[NOW]*Chassis_Z_kpid[KP] + Chassis_Z_kpid[KD]*(error[NOW] - error[LAST]);
	Chassis_Move_Z = constrain(Chassis_Move_Z, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);

	error[LAST] = error[NOW];
	
	return Chassis_Move_Z;
}


/*****************底盘功率*************************/

/**
  * @brief  底盘功率限制
  * @param  void
  * @retval void
  * @attention  在底盘输出计算后调用,主要是比例的算法,ICRA
  */
void Chassis_Power_Limit(void)
{	
	/*********************祖传算法*************************/
	float    kLimit = 0;//功率限制系数
	float    chassis_totaloutput = 0;//统计总输出电流
	float    Joule_Residue = 0;//剩余焦耳缓冲能量
	int16_t  judgDataCorrect = 0;//裁判系统数据是否可用	
	static int32_t judgDataError_Time = 0;
	
	
	judgDataCorrect = JUDGE_sGetDataState();//裁判系统数据是否可用
	Joule_Residue = JUDGE_fGetRemainEnergy();//剩余焦耳能量	
	
	//统计底盘总输出
	chassis_totaloutput = abs(Chassis_Final_Output[0]) + abs(Chassis_Final_Output[1])
							+ abs(Chassis_Final_Output[2]) + abs(Chassis_Final_Output[3]);
	
	if(judgDataCorrect == JUDGE_DATA_ERROR)//裁判系统无效时强制限速
	{
		judgDataError_Time++;
		if(judgDataError_Time > 100)
		{
			fTotalCurrentLimit = 9000;//降为最大的1/4
		}
	}
	else
	{
		judgDataError_Time = 0;
		//剩余焦耳量过小,开始限制输出,限制系数为平方关系
		if(Joule_Residue < WARNING_REMAIN_POWER)
		{
			kLimit = (float)(Joule_Residue / WARNING_REMAIN_POWER)
						* (float)(Joule_Residue / WARNING_REMAIN_POWER);
			
			fTotalCurrentLimit = kLimit * fChasCurrentLimit;
		}
		else   //焦耳能量恢复到一定数值
		{
			fTotalCurrentLimit = fChasCurrentLimit;
		}
	}

	//底盘各电机电流重新分配
	if (chassis_totaloutput > fTotalCurrentLimit)
	{
		Chassis_Final_Output[0] = (int16_t)(Chassis_Final_Output[0] / chassis_totaloutput * fTotalCurrentLimit);
		Chassis_Final_Output[1] = (int16_t)(Chassis_Final_Output[1] / chassis_totaloutput * fTotalCurrentLimit);
		Chassis_Final_Output[2] = (int16_t)(Chassis_Final_Output[2] / chassis_totaloutput * fTotalCurrentLimit);
		Chassis_Final_Output[3] = (int16_t)(Chassis_Final_Output[3] / chassis_totaloutput * fTotalCurrentLimit);	
	}
}


/**************键盘模式辅助函数********************/

/**
  * @brief  底盘键盘斜坡函数
  * @param  判断按键是否被按下, 时间量, 每次增加的量, 一共要减小的量
  * @retval 斜坡比例系数
  * @attention  0~1
  */
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec )
{
	float  factor = 0;

	
	factor = 0.15 * sqrt( 0.15 * (*time) );  //计算速度斜坡,time累加到296.3斜坡就完成
	
	if (status == 1)//按键被按下
	{
		if (factor < 1)//防止time太大
		{
			*time += inc;
		}
	}
	else		//按键松开
	{
		if (factor > 0)
		{
			*time -= dec;

			if (*time < 0)
			{
				*time = 0;
			}
		}
	}

	factor = constrain_float( factor, 0, 1 );//注意一定是float类型限幅

	return factor;  //注意方向
}

/**
  * @brief  获取底盘移动模式
  * @param  void
  * @retval TRUE:机械模式    false:陀螺仪模式
  * @attention  
  */
bool CHASSIS_IfActiveMode(void)
{
	if (modeChassis == CHASSIS_MECH_MODE)
	{
		return TRUE;//机械
	}
	else
	{
		return FALSE;//陀螺仪
	}
}

/**
  * @brief  底盘是否处于爬坡模式
  * @param  void
  * @retval TRUE FALSE
  * @attention  
  */
bool Chassis_IfSZUPUP(void)
{
	if(actChassis == CHASSIS_SZUPUP)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  底盘是否处于扭屁股模式
  * @param  void
  * @retval TRUE FALSE
  * @attention  
  */
bool Chassis_IfCORGI(void)
{
	if(actChassis == CHASSIS_CORGI)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  底盘是否处于45°模式
  * @param  void
  * @retval TRUE FALSE
  * @attention  
  */
bool Chassis_IfPISA(void)
{
	if(actChassis == CHASSIS_PISA)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*********************底盘数据接口*************************/

/**
  * @brief  获取底盘实际转速
  * @param  void
  * @retval 底盘总转速
  * @attention  
  */
float Chassis_All_Speed(void)
{
	return ( abs(Chassis_Speed_Measure[0]) + abs(Chassis_Speed_Measure[1])
				 + abs(Chassis_Speed_Measure[2]) + abs(Chassis_Speed_Measure[3]) );
}

/**
  * @brief  获取底盘目标转速
  * @param  void
  * @retval 底盘总转速
  * @attention  
  */
float Chassis_All_Speed_Target(void)
{
	return ( abs(Chassis_Speed_Target[0]) + abs(Chassis_Speed_Target[1])
				 + abs(Chassis_Speed_Target[2]) + abs(Chassis_Speed_Target[3]) );
}


/**
  * @brief  获取底盘移动目标值
  * @param  void
  * @retval 底盘目标移动量
  * @attention  用来判断底盘是否发生了移动变化趋势,超级电容充电用
  */
float Get_Chass_X(void)
{
	return Chassis_Move_X;
}

float Get_Chass_Y(void)
{
	return Chassis_Move_Y;
}

float Get_Chass_Z(void)
{
	return Chassis_Move_Z;
}

/**
  * @brief  扭屁股专用（时间不变版）
  * @param  当前云台偏离量，目标偏移量
  * @retval 旋转速度
  * @attention 
  */
float Chassis_Z_Corgi(float get, float set)
{
	float error[2];
	float z = 0;
	
	error[NOW] = set - get;
	
	z = 15*((error[NOW]*8) + 10*(error[NOW] - error[LAST]));//PD计算
	z = constrain_float(z, -5000, 5000);

	error[LAST] = error[NOW];
	
	return z;
}

/**
  * @brief  旋转速度PID计算
  * @param  当前云台偏离量，目标偏移量
  * @retval 旋转速度
  * @attention 
  */
float speed_z_pterm = 0;
float speed_z_iterm = 0;
float speed_z_dterm = 0;
float Chassis_SpeedZ_PID(int16_t ErrorReal, float kp)
{
	static int16_t ErrorPrev = 0;//上次偏离误差
	static int32_t ErrorSum = 0;//上次偏离误差
	static int32_t ErrorPR = 0;
	static int32_t ErrorPR_KF = 0;
	
	float speed_z = 0;

	ErrorPR_KF = KalmanFilter(&Chassis_Error_Kalman, ErrorReal);
	
	//P
	speed_z_pterm = ErrorReal * kp;//根据yaw偏离中心计算电流
	speed_z_pterm = constrain_float(speed_z_pterm, -REVOLVE_MAX_NORMAL, REVOLVE_MAX_NORMAL);
	
	//I
	ErrorSum -= ErrorPR_KF;
	speed_z_iterm = ErrorSum*3*0.002f;
	if( abs(ErrorReal) <= 10)
	{
		ErrorSum = 0;
	}
	//积分限幅
	speed_z_iterm = constrain_float(speed_z_iterm,-5000,5000);
	
	//D
	ErrorPR = ErrorPR_KF - ErrorPrev;
	
	if( abs(ErrorPR_KF) > REVOLVE_ANGLE )
	{
		speed_z_dterm = -(ErrorPR) * REVOLVE_KD;//600;//650;//125.f;
	}
	else
	{
		speed_z_dterm = 0;
	}
	//扭头最大速度限幅
	speed_z = speed_z_pterm + speed_z_dterm;// + speed_z_iterm;//// + ;
	speed_z = constrain(speed_z, -Chassis_Revolve_Move_Max, +Chassis_Revolve_Move_Max);

	ErrorPrev = ErrorPR_KF;//记录上次误差
	
	return speed_z;
}

