#include "adda.h"
#include "stm32f4xx_dac.h"  

/**
  * @brief  超级电容数模转化器初始化
  * @param  void
  * @retval void
  */
void SuperCap_DAC_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef   DAC_InitType;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//使能DAC时钟

		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//不使用触发功能 TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//不使用波形发生
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//屏蔽、幅值设置
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1输出缓存关闭 BOFF1=1
	DAC_Init(DAC_Channel_1,&DAC_InitType);	 //初始化DAC通道1

	DAC_Cmd(DAC_Channel_1, ENABLE);  //使能DAC通道1
  
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值
}



/**
  * @brief  超级电容模数转化器初始化
  * @param  void
  * @retval void
  */
void SuperCap_ADC_Init(void)
{
	GPIO_InitTypeDef  GPIO_ADCInitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

	//先初始化ADC1通道10 IO口
	GPIO_ADCInitStructure.GPIO_Pin = GPIO_Pin_0;//PC0 通道10
	GPIO_ADCInitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
	GPIO_ADCInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
	GPIO_Init(GPIOC, &GPIO_ADCInitStructure);//初始化  

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//初始化

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化

	ADC_Cmd(ADC1, ENABLE);//开启AD转换器
}


/**
  * @brief  超级电容io口初始化
  * @param  void
  * @retval void
  * @attention PA5->CAP_IN   PA->CAP_OUT
  */
void SuperCap_IO_Init(void)
{
	GPIO_InitTypeDef gpioa;	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	gpioa.GPIO_PuPd = GPIO_PuPd_DOWN;
	gpioa.GPIO_Speed = GPIO_Speed_50MHz;	
	gpioa.GPIO_Mode = GPIO_Mode_OUT;		
	gpioa.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_2;		
	GPIO_Init(GPIOA,&gpioa);	
	
	CAP_Charge_Off;
	CAP_OUT_Off;
}



/**
  * @brief  设置输出电压
  * @param  vol:0~3300,代表0~3.3V
  * @retval void
  */
void Dac1_Set_Vol(u16 vol)
{
	double temp=vol;
	temp/=1000;
	temp=temp*4096/3.3;
	DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12位右对齐数据格式设置DAC值
}



/**
  * @brief  获得ADC值
  * @param  void
  * @retval 转换结果
  */
u16 Get_Adc(void)   
{
	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, 10, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道10,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 	
	return ADC_GetConversionValue(ADC1);

}

/**
  * @brief  获取通道ch的转换值，取times次,然后平均 
  * @param  times:获取次数
  * @retval 通道ch的times次转换结果平均值
  */
u16 Get_Adc_Average(u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc();
	}
	return temp_val/times;
} 


/**
  * @brief  获取电压值
  * @param  void
  * @retval void
  */
float value = 0;
float Get_Realvoltage(void)
{
	value = Get_Adc_Average(10);		//取10次平均值
	value = (float)value*(3.3/4096);
	return (value*11);
}

/**
  * @brief  电压值百分百
  * @param  void
  * @retval void
  * @attention 22~13表示100%~0%
  */
int Capvoltage_Percent(void)
{
	int percent = 0;
	percent = (Get_Realvoltage() - 13.0f) / (23.5f - 13.0f) * 100.0f;	
	
	if(percent<0)
	{ percent = 0; }
	
	return percent;
}
