/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//----------------------显示数字真�?�表----------------------//
 /*
  *                  A B C D   OUT
  *                  0 0 0 0      3
  *                  1 0 0 0      4
  *                  1 1 0 0      7
  *                  0 1 0 0      6
  *                  0 0 1 0     0
  *                  1 0 1 0      5
  *                  0 1 1 0      9
  *                  1 1 1 0       8
  *                  0 0 0 1      1
  *                  1 0 0 1      2
  *                  输入ADBC
*/

//----------------------数据位宏----------------------//
//1+�?4�?+0000 共九�? 第一�?1为占�?
#define L0  0x110  //100010000
#define L1  0x140  //101000000
#define L2 0x1c0  //111000000
#define L3 0x100  //100000000
#define L4 0x180 //110000000
#define L5 0x190 //110010000
#define L6 0x120 //100100000
#define L7 0x1a0 //110100000
#define L8 0x1b0 //110110000
#define L9 0x130 //100110000

//1+0000+�?4�? 共九�? 第一�?1为占�?
#define R0  0x101  //100000001
#define R1  0x104  //100000001
#define R2 0x10c  //100001100
#define R3 0x100  //100000000
#define R4 0x108 //100001000
#define R5 0x109 //100001001
#define R6 0x102 //100000010
#define R7 0x10a //100001010
#define R8 0x10b //100001011
#define R9 0x103 //100000011

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint32_t Show_Num[4] = {0,0,0,0};
uint32_t n1234[4] = {0};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  DateTime TIME;
  DateTime* time = &TIME;

  DS3231_Init(); //时钟模式初始�???

  //DS3231_setDate(22,3,31);
  //DS3231_setTime(15,45,10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SER2_Pin|STCP_Pin|SHCP_Pin|SER1_Pin
                          |I2C_SCL_Pin|I2C_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SER2_Pin STCP_Pin SHCP_Pin SER1_Pin
                           I2C_SCL_Pin I2C_SDA_Pin */
  GPIO_InitStruct.Pin = SER2_Pin|STCP_Pin|SHCP_Pin|SER1_Pin
                          |I2C_SCL_Pin|I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//-------------------------------------//
//
//-------------------------------------//
void delay_us(int16_t nus)
{
	int32_t temp;
	SysTick->LOAD = nus*9;
	SysTick->VAL = 0X00;
	SysTick->CTRL = 0X10;
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&(!(temp&(1<<16))));
	SysTick->CTRL = 0x00;
	SysTick->VAL = 0x00;
}

//-----------------------------------------------------------//
// 向两�?595写数据，上升沿时序，两片595共用时钟与�?��?�线，但各自用一条数据线
// NUL++/--只是起到�?个延时的作用，虽然实际上可能也没啥用
//
//-----------------------------------------------------------//
void Write_74hc595_Test(uint32_t TxData1,uint32_t TxData2){
	uint8_t i,NUL=0;
	SER1_L;
	STCP_L;
	SHCP_L;
	for(i=0;i<9;i++)
	{
		if(TxData1 & 0x100)	{
			SER1_H;
			if(TxData2 & 0x100) SER2_H;
			else SER2_L;
		}
		else	{
			SER1_L;
			if(TxData2 & 0x100) SER2_H;
			else SER2_L;
		}

		NUL++;
		SHCP_H;//时钟线上升沿，加载两条数据线到移位寄存器

		TxData1 <<= 1;
		TxData2 <<= 1;

		NUL--;
		SHCP_L;
	}

	STCP_H;//选�?�上升沿，锁存器输出

	NUL++;

	STCP_L;

}

//-----------------------------------------------------------//
// �?595要输入的数据合成，共九位
// �?高位�?1占位，剩�?8位高四位为左边管子，低四位为右边管子
// 具体数据详见�?上方真�?�表
//-----------------------------------------------------------//
uint32_t Bytes_Config(uint32_t n1,uint32_t n2){
	uint32_t n3 = 0x0;
	n3 = n1 | n2;
	return n3;
}

//-----------------------------------------------------------//
//0不显示，1显示
//-----------------------------------------------------------//
uint32_t Time_Check(){
	uint16_t now_time = Show_Num[0]*10+Show_Num[1];
	if(now_time==24 || now_time<7) return 0;
	if(now_time>7 && now_time<12) return 0;
	if(now_time>14 && now_time<18) return 0;
	return 1;
}

//-----------------------------------------------------------//
//显示Show_Num数组的四个数字，目前用于显示时间
//-----------------------------------------------------------//
void Write_Num_Time(){
	n1234[0] = Num_Select_L(Show_Num[0]);
	n1234[1] = Num_Select_R(Show_Num[1]);
	n1234[2] = Num_Select_L(Show_Num[2]);
	n1234[3] = Num_Select_R(Show_Num[3]);
	uint32_t n12 = Bytes_Config(n1234[0],n1234[1]);
	uint32_t n34 = Bytes_Config(n1234[2],n1234[3]);
	Write_74hc595_Test(n12,n34);
}

//-----------------------------------------------------------//
//
//-----------------------------------------------------------//
void Write_Num_UP(){
	n1234[0] = Num_Select_L(Show_Num[0]);
	n1234[1] = Num_Select_R(Show_Num[1]);
	n1234[2] = Num_Select_L(Show_Num[2]);
	n1234[3] = Num_Select_R(Show_Num[3]);
	uint32_t n12 = Bytes_Config(n1234[0],n1234[1]);
	uint32_t n34 = Bytes_Config(n1234[2],n1234[3]);
	Write_74hc595_Test(n12,n34);
	Show_Num[0] = (Show_Num[0]+11)%10;
	Show_Num[1] = (Show_Num[0]+12)%10;
	Show_Num[2] = (Show_Num[0]+13)%10;
	Show_Num[3] = (Show_Num[0]+14)%10;
}

//-----------------------------------------------------------//
//
//-----------------------------------------------------------//
uint32_t Num_Select_L(uint16_t num){
	switch(num){
	case 0:return L0; break;
	case 1: return L1; break;
	case 2:return L2; break;
	case 3:return L3; break;
	case 4:return L4; break;
	case 5:return L5; break;
	case 6:return L6; break;
	case 7:return L7; break;
	case 8:return L8; break;
	case 9:return L9; break;
	default:return L0;
	}
}

//-----------------------------------------------------------//
//
//-----------------------------------------------------------//
uint32_t Num_Select_R(uint16_t num){
	switch(num){
	case 0:return R0; break;
	case 1:return R1; break;
	case 2:return R2; break;
	case 3:return R3; break;
	case 4:return R4; break;
	case 5:return R5; break;
	case 6:return R6; break;
	case 7:return R7; break;
	case 8:return R8; break;
	case 9:return R9; break;
	default:return R0;
	}
}

//-----------------------------------------------------------//
//
//-----------------------------------------------------------//
uint16_t Random_Num(){
	uint16_t rnum = 0;
	srand(SysTick->VAL);
	rnum = rand() % 9;    //生成0�?9内的随机整数
	return rnum;

}

/*------------------------------------------------------------------------------------------------
以下为模拟I2C相关函数
------------------------------------------------------------------------------------------------*/


//-------------------------------------//
// 配置SDA为推挽输�??????
//-------------------------------------//
static void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    GPIO_Initure.Pin = I2C_SDA_Pin; // SDA引脚
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出模式
    GPIO_Initure.Pull = GPIO_NOPULL; // �??????
    GPIO_Initure.Speed = GPIO_SPEED_HIGH; // 高�??
    HAL_GPIO_Init(I2C_SDA_GPIOx, &GPIO_Initure); // GPIOF初始�??????
}

//-------------------------------------//
// 配置SDA为上拉输�??????
//-------------------------------------//
static void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    GPIO_Initure.Pin = I2C_SDA_Pin; // SDA引脚
    GPIO_Initure.Mode = GPIO_MODE_INPUT; // 输入模式
    GPIO_Initure.Pull = GPIO_PULLUP; // 上拉
    GPIO_Initure.Speed = GPIO_SPEED_HIGH; // 高�??
    HAL_GPIO_Init(I2C_SDA_GPIOx, &GPIO_Initure); // GPIOF初始�??????
}


//-------------------------------------//
//I2C初始�??????
//本来应该放引脚时钟初始化之类的，但cube ide在main.c中已经自动生成了
//-------------------------------------//
void I2C_Init(void){
	I2C_SDA_H;
	I2C_SCL_H;

}

//-------------------------------------//
//模拟I2C产生起始条件—�?�SCL为高时SDA产生下降�??????
//起始和终止的延时目的都是让电平稳定了再继续，防止误判
//-------------------------------------//
void I2C_Start(void){
	SDA_OUT();//配置成输出模�??????
	I2C_SDA_H;
	I2C_SCL_H;//再次确保都为�??????
	delay_us(2);//4
	I2C_SDA_L;//SDA下降沿，通信�??????�??????
	delay_us(2);
	I2C_SCL_L;//时钟线拉低，准备传输数据
	delay_us(2);//4
}

//-------------------------------------//
//模拟I2C产生终止条件—�?�SCL为高时SDA产生上升�??????
//-------------------------------------//
void I2C_Stop(void){
	SDA_OUT();//配置成输出模�??????
	I2C_SCL_L;//先将时钟线拉低，防止SDA改变影响数据传输
	I2C_SDA_L;//SDA先置�??????
	I2C_SCL_H;//时钟线置�??????
	delay_us(2);//4
	I2C_SDA_H;//SDA拉高，�?�信结束
	delay_us(2);//4
}


//-------------------------------------//
//模拟I2C等待响应
//-------------------------------------//
uint8_t I2C_Wait_Ask(void)
{
    uint16_t tempTime = 0;
    SDA_IN();//配置成输入模�??????
    I2C_SDA_H;   //释放数据总线，交由从机控�??????
    delay_us(2);
    I2C_SCL_H;
    delay_us(1);
    while (HAL_GPIO_ReadPin(I2C_SDA_GPIOx,I2C_SDA_Pin)) //读到 0 ，即接收到ACK，循环跳�??????
    {
        tempTime++;
        if(tempTime > 300)
        {
            I2C_Stop();
            return 1;			//超时返回1
        }
    }
    I2C_SCL_L;
    return 0;							//接收�?????? ACK 返回0
}

//-------------------------------------//
//模拟I2C响应
//-------------------------------------//
void I2C_Ack(void)
{
	I2C_SCL_L;
    SDA_OUT();
    I2C_SDA_L;
    delay_us(2);
    I2C_SCL_H;
    delay_us(3);
    I2C_SCL_L;
}

//-------------------------------------//
//主机不产生应答信号NACK
//-------------------------------------//
void I2C_NAck(void)
{
	I2C_SCL_L;
    SDA_OUT();
    I2C_SDA_H;
    delay_us(2);
    I2C_SCL_H;
    delay_us(3);
    I2C_SCL_L;
}

//-------------------------------------//
//i2c写数�??????
//-------------------------------------//
void I2C_WriteByte(uint8_t data)
{
	uint8_t i;
    SDA_OUT();
    for(i=0;i<8;i++)
    {
        I2C_SCL_L;
        delay_us(2);
        if(data & 0x80)
            I2C_SDA_H;
        else
            I2C_SDA_L;
        I2C_SCL_H;
        delay_us(2);
        I2C_SCL_L;
        data<<=1;
    }
}

//-------------------------------------//
//i2c读数�??????
//�??????1个字节，ack=1时，发�?�ACK，ack=0，发送nACK
//-------------------------------------//
uint8_t I2C_Read_Byte(uint8_t ack)
{
	uint8_t i,receive=0;
	SDA_IN();//SDA设置为输�??????
    for(i=0;i<8;i++ ){
        I2C_SCL_L;
        delay_us(2);
		I2C_SCL_H;
        receive<<=1;
        if(HAL_GPIO_ReadPin(I2C_SDA_GPIOx,I2C_SDA_Pin)){
        	receive++;
        }
		delay_us(1);
	}
    if (!ack)
        I2C_NAck();//发�?�nACK
    else
        I2C_Ack(); //发�?�ACK
    return receive;
}

/*------------------------------------------------------------------------------------------------
以下为DS3231相关函数
------------------------------------------------------------------------------------------------*/

//-------------------------------------//
//初始化，实际上就是模拟I2C初始�??????
//-------------------------------------//
void DS3231_Init(void){
	I2C_Init();
}

//-------------------------------------//
//
//-------------------------------------//
uint8_t I2C_DS3231_ByteWrite(uint8_t WriteAddr , uint8_t date)
{
	I2C_Start();
	//printf("I2C_DS3231_ByteWrite start!");
	I2C_WriteByte(DS3231_ADDRESS_Write);
	//printf("write finish,wait ask!");
	if(I2C_Wait_Ask())
		return 1;
	//printf("add ask ok!");
	I2C_WriteByte(WriteAddr);
	if(I2C_Wait_Ask())
		return 2;
	//printf("date ask ok!");
	I2C_WriteByte(date);
	if(I2C_Wait_Ask())
		return 3;
	I2C_Stop();
	return 0;
}

//-------------------------------------//
//
//-------------------------------------//
uint8_t I2C_DS3231_ByteRead(uint8_t ReadAddr,uint8_t* Receive)
{
	uint8_t data = 0;

	I2C_Start();													//产生起始�??????
	I2C_WriteByte(DS3231_ADDRESS_Write); 	//发�?�从机地�??????（写模式�??????
	if(I2C_Wait_Ask())										//等待响应
		return 1;
	I2C_WriteByte(ReadAddr);							//发�?�寄存器地址
	if(I2C_Wait_Ask())										//等待响应
		return 2;
	I2C_Start();													//重复起始�??????
	I2C_WriteByte(DS3231_ADDRESS_Read);		//发�?�从机地�??????（读模式�??????
	if(I2C_Wait_Ask())										//等待响应
		return 3;
	data = I2C_Read_Byte(0);							//读取数据，参数设�??????0 --- NACK
	*Receive = data;											//将结果赋值给接收�??????
	I2C_Stop();
	return 0;
}

//-------------------------------------//
//
//-------------------------------------//
uint8_t DS3231_setDate(uint8_t year,uint8_t mon,uint8_t day)
{
	uint8_t temp_H , temp_L;
	temp_L = year%10;
	temp_H = year/10;
	year = (temp_H << 4) + temp_L;
	//printf("start DS3231 SETDATE \r\n");
	if(I2C_DS3231_ByteWrite(DS3231_YEAR_REG,year)) //set year
	{
			//printf("set year error\r\n");
			return 1;
	}
	//printf("year ok \r\n");
	temp_L = mon%10;
	temp_H = mon/10;
	mon = (temp_H << 4) + temp_L;
	if(I2C_DS3231_ByteWrite(DS3231_MONTH_REG,mon)) //set mon
	{
		//printf("set month error\r\n");
		return 2;
	}
	//printf("month ok \r\n");
	temp_L = day%10;
	temp_H = day/10;
	day = (temp_H << 4) + temp_L;
	if(I2C_DS3231_ByteWrite(DS3231_MDAY_REG,day)) //set day
	{
		//printf("set day error\r\n");
		return 3;
	}
	//printf("all ok \r\n");
	return 0;
}

//-------------------------------------//
//
//-------------------------------------//
uint8_t DS3231_setTime(uint8_t hour , uint8_t min , uint8_t sec)
{
	uint8_t temp_H , temp_L;
	temp_L = hour%10;
	temp_H = hour/10;
	hour = (temp_H << 4) + temp_L;
	//printf("start DS3231 SETTIME \r\n");
	if(I2C_DS3231_ByteWrite(DS3231_HOUR_REG,hour)){ //set hour
		//printf("set time error \r\n");
		return 1;
	}
	//printf("HOUR OK \r\n");
	temp_L = min%10;
	temp_H = min/10;
	min = (temp_H << 4) + temp_L;
	if(I2C_DS3231_ByteWrite(DS3231_MIN_REG,min)){ //SET min
		//printf("set time error \r\n");
		return 2;
	}
	//printf("MIN OK \r\n");
	temp_L = sec%10;
	temp_H = sec/10;
	sec = (temp_H << 4) + temp_L;
	if(I2C_DS3231_ByteWrite(DS3231_SEC_REG,sec)){
		//printf("set time error \r\n");//SET sec
		return 3;
	}
	//printf("ALL OK \r\n");
	return 0;
}

//-------------------------------------//
//
//-------------------------------------//
static uint8_t bcdToDec(uint8_t byte)
{
	uint8_t temp_H , temp_L;
	temp_L = byte & 0x0f;
	temp_H = (byte & 0xf0) >> 4;
	return ( temp_H * 10 )+ temp_L;
}

//-------------------------------------//
//
//-------------------------------------//
uint8_t DS3231_getTime(DateTime* ans)
{
	uint8_t receive = 0;
	if(I2C_DS3231_ByteRead(DS3231_HOUR_REG,&receive))
		return 1;
	ans->hour = bcdToDec(receive);
	if(I2C_DS3231_ByteRead(DS3231_MIN_REG,&receive))
		return 2;
	ans->minute = bcdToDec(receive);
	if(I2C_DS3231_ByteRead(DS3231_SEC_REG,&receive))
		return 3;
	ans->second = bcdToDec(receive);
	return 0;
}

//-------------------------------------//
//
//-------------------------------------//
uint8_t DS3231_getDate(DateTime* ans)
{
	uint8_t receive = 0;
	if(I2C_DS3231_ByteRead(DS3231_YEAR_REG,&receive))
		return 1;
	ans->year = bcdToDec(receive) + 2000;
	if(I2C_DS3231_ByteRead(DS3231_MONTH_REG,&receive))
		return 2;
	ans->month = bcdToDec(receive);
	if(I2C_DS3231_ByteRead(DS3231_MDAY_REG,&receive))
		return 3;
	ans->dayofmonth = bcdToDec(receive);
	return 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
