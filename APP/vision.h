#ifndef _VISION_H
#define _VISION_H

#include "system.h"


/*--------------------------------暂定协议-------------------------------------*/

#define    VISION_LENGTH        22     		 //暂定22字节,头3字节,数据17字节,尾2字节

//起始字节,协议固定为0xA5
#define    VISION_SOF         (0xA5)

//长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取
#define    VISION_LEN_HEADER    3         //帧头长
#define    VISION_LEN_DATA      17        //数据段长度,可自定义
#define    VISION_LEN_TAIL      2	      //帧尾CRC16
#define    VISION_LEN_PACKED    22        //数据包长度,可自定义

#define    VISION_OFF         		(0x00)
#define    VISION_RED           	(0x01)
#define    VISION_BLUE          	(0x02)
#define    VISION_RBUFF_ANTI   	 	(0x03)//红逆大符
#define    VISION_BBUFF_ANTI   		(0x04)//蓝逆大符
#define    VISION_RBUFF_CLOCKWISE   (0x05)//红顺大符
#define    VISION_BBUFF_CLOCKWISE   (0x06)//蓝顺大符
#define    VISION_RBUFF_STAND   	(0x07)//红小符
#define    VISION_BBUFF_STAND   	(0x08)//蓝小符

/* 	STM32 -> PC

	CmdID   0x00   关闭视觉
	CmdID   0x01   识别红色装甲
	CmdID   0x02   识别蓝色装甲
	CmdID   0x03   红符
	CmdID   0x04   蓝符
*/

/* 	PC -> STM32

	CmdID   0x00   关闭视觉
	CmdID   0x01   识别红色装甲
	CmdID   0x02   识别蓝色装甲
	CmdID   0x03   小符
	CmdID   0x04   大符
*/

//可利用收和发的指令码进行比较,当收和发的指令码相同时,可判定为数据可用

//帧头加CRC8校验,保证发送的指令是正确的

//PC收发与STM32收发成镜像关系,以下结构体适用于STM32,PC需稍作修改

typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
}extVisionSendHeader_t;


//STM32接收,直接将串口接收到的数据拷贝进结构体
typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//距离
	uint8_t   centre_lock;		//是否瞄准到了中间  0没有  1瞄准到了
	uint8_t	  identify_target;	//视野内是否有目标/是否识别到了目标   0否  1是	
	uint8_t   identify_buff;	//打符时是否识别到了目标，1是，2识别到切换了装甲，0没识别到
	
	uint8_t	  blank_b;			//预留
	uint8_t	  auto_too_close;   //目标距离太近,视觉发1，否则发0
	
	
	/* 尾 */
	uint16_t  CRC16;       
	
}extVisionRecvData_t;


//STM32发送,直接将打包好的数据一个字节一个字节地发送出去
typedef struct
{
//	/* 头 */
//	uint8_t   SOF;			//帧头起始位,暂定0xA5
//	uint8_t   CmdID;		//指令
//	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//距离
	uint8_t   lock_sentry;		//是否在抬头识别哨兵
	uint8_t   base;				//吊射
	
	uint8_t   blank_a;		//预留
	uint8_t	  blank_b;
	uint8_t	  blank_c;	
	
	/* 尾 */
	uint16_t  CRC16;
	
}extVisionSendData_t;



//关于如何写入CRC校验值
//我们可以直接利用官方给的CRC代码

//注意,CRC8和CRC16所占字节不一样,8为一个字节,16为2个字节

//写入    CRC8 调用    Append_CRC8_Check_Sum( param1, param2)
//其中 param1代表写好了帧头数据的数组(帧头后的数据还没写没有关系),
//     param2代表CRC8写入后数据长度,我们定义的是头的最后一位,也就是3

//写入    CRC16 调用   Append_CRC16_Check_Sum( param3, param4)
//其中 param3代表写好了   帧头 + 数据  的数组(跟上面是同一个数组)
//     param4代表CRC16写入后数据长度,我们定义的整个数据长度是22,所以是22

/*----------------------------------------------------------*/

#define ATTACK_NONE    0	//不识别
#define ATTACK_RED     1	//识别红方
#define ATTACK_BLUE    2	//识别蓝方

extern extVisionRecvData_t    VisionRecvData;//视觉接收结构体


void Vision_Read_Data(uint8_t *ReadFormUsart);
void Vision_Send_Data( uint8_t CmdID );


/****视觉控制*****/
void Vision_Ctrl(void);

void Vision_Buff_Ctrl(void);
void Vision_Auto_Attack_Ctrl(void);
void Vision_Auto_Attack_Off(void);

/*****视觉偏差获取******/
void Vision_Error_Yaw(float *error);
void Vision_Error_Pitch(float *error);
void Vision_Error_Angle_Yaw(float *error);
void Vision_Error_Angle_Pitch(float *error);
void Vision_Buff_Error_Angle_Yaw(float *error);
void Vision_Buff_Error_Angle_Yaw_Gimbal(float *error);
void Vision_Buff_Error_Angle_Pitch(float *error);
void Vision_Base_Yaw_Pixel(float *error);

void Vision_Get_Distance(float *distance);
void Vision_Compensation(float *compe_yaw, float *compe_pitch);

/********视觉辅助函数*********/
uint8_t VISION_isColor(void);
uint8_t VISION_BuffType(void);
bool VISION_IfCmdID_Identical(void);
bool Vision_If_Update(void);
void Vision_Clean_Update_Flag(void);
bool Vision_If_Armor(void);
void Vision_Clean_Ammor_Flag(void);

#endif
