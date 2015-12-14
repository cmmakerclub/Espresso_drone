/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

#define beta                        0.1f     
#define ACCELEROMETER_SENSITIVITY   16384.0f  
#define GYROSCOPE_SENSITIVITY       131.07f  
#define M_PI                        3.14159265359f	    
#define sampleFreq                  200.0f     			    // 200 hz sample rate!   
#define limmit_I                    300.0f     

#define roll_offset    0.9f     
#define pitch_offset   -2.7f  

#define gx_diff 		-423
#define gy_diff 		-24
#define gz_diff 		-63

#define Kp_yaw      60.0f
#define Ki_yaw      0.0f
#define Kd_yaw      0.0f

//#define Kp_pitch		120.0f
#define Ki_pitch    0.0f
//#define Kd_pitch    80.0f

//#define Kp_roll	    Kp_pitch
#define Ki_roll  		0.0f
//#define Kd_roll  		Kd_pitch

#include "MPU6050.h"
#include <math.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

float Kp_pitch		= 130.0f;

float Kd_pitch    = 30.0f;

float Ref_yaw=0, Ref_pitch=0, Ref_roll=0 ;
float q_yaw, q_pitch, q_roll;                                            // States value
float q1=0, q2=0, q3=0, q4=1 ;
float T_center =0, yaw_center=0;
float Error_yaw=0, Errer_pitch=0, Error_roll=0; 						//States Error
float Sum_Error_yaw=0, Sum_Error_pitch=0, Sum_Error_roll=0;             // Sum of error
float D_Error_yaw=0, D_Error_pitch=0, D_Error_roll=0; 					// error dot
float Del_yaw=0, Del_pitch=0, Del_roll=0;												// Delta states value for rotate axis
float t_compensate = 0;
float T_center_minus = 0;
float y_roll=0, y_pitch=0, y0_roll=0, y0_pitch=0 ; 
float D_Error_pitch_f ;
float D_Error_roll_f ;


uint16_t watchdog  = 0;

/* USER CODE for SPPM Receiver  */

uint8_t     index =0 ;
uint8_t    	rx_buffer[12]={0} ;
int16_t     ch1=0,ch2=0,ch3=0,ch4=0;                 
int16_t     AccelGyro[6]={0};       // RAW states value
int16_t     motor_A=0, motor_B=0, motor_C=0, motor_D=0 ;// Motors output value 

float a, b, c;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void Initial_MPU6050(void);
void read_mpu6050(void);
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
void MPU6050_GetRawAccelGyro(int16_t* AccelGyro);
volatile void Controller(void);
volatile void PID_controller(void);
volatile void Drive_motor_output(void);
volatile void Interrupt_call(void);
volatile void ahrs(void);
float Smooth_filter(float alfa, float new_data, float prev_data);
void UART_Callback(void);

float Butterworth_filter1(float x_data1);
float Butterworth_filter2(float x_data);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

	Initial_MPU6050();
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
	
	HAL_TIM_Base_Start_IT(&htim17);
		
	HAL_Delay(1000);
	HAL_UART_Receive_IT(&huart1, (uint8_t *)(rx_buffer + index), 1);
		
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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM14 init function */
void MX_TIM14_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 2399;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim14);

  HAL_TIM_PWM_Init(&htim14);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1);

}

/* TIM17 init function */
void MX_TIM17_Init(void)
{

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 479;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 499;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim17);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void Initial_MPU6050(void)
	{
		HAL_Delay(100); // for stability
			//    Reset to defalt 
		MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, ENABLE);
		HAL_Delay(100);
			//	  SetClockSource(MPU6050_CLOCK_PLL_XGYRO)
		MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_ZGYRO);	
			//    SetFullScaleGyroRange(MPU6050_GYRO_FS_250)
		MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);
			//    SetFullScaleAccelRange(MPU6050_ACCEL_FS_2)
		MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
			//    interupt(Enable)
		MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, ENABLE);
		 //    SetSleepModeStatus(DISABLE)
//		MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, DISABLE);
//		//			SetDLPF(MPU6050_DLPF_BW_5)
		
		HAL_Delay(10); // for stability		
		
		MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42);
	
		HAL_Delay(10); // for stability
	
		MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, DISABLE);
			
		HAL_Delay(10); // for stability
		
		MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42);
}

void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp;
		HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    HAL_I2C_Mem_Write(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
}

void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t tmp;
		HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    HAL_I2C_Mem_Write(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
}

void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp;
		HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}

void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t tmp;
		HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    *data = tmp & (1 << bitNum);
}
void MPU6050_GetRawAccelGyro(int16_t* AccelGyro)
{
    uint8_t tmpBuffer[14];
	  HAL_I2C_Mem_Read(&hi2c1,MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,1,tmpBuffer,14,2);
    /* Get acceleration */
    for (int i = 0; i < 3; i++)
        AccelGyro[i] = ((int16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
    /* Get Angular rate */
    for (int i = 4; i < 7; i++)
        AccelGyro[i - 1] = ((int16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);

		ahrs();
}

volatile void PID_controller(void)
{
	static float T_center_buffer;
  float Buf_D_Error_yaw   =Error_yaw;
	float Buf_D_Errer_pitch =Errer_pitch;
	float Buf_D_Error_roll  =Error_roll; 
     
	T_center_buffer    = (float)ch3 *   18.0f;
	
	T_center = Smooth_filter(0.08f, T_center_buffer, T_center);
	
	//Error_yaw 	= (float)ch4 * 3.0f - q_yaw;
	Error_yaw 	= - q_yaw;
	Errer_pitch = (float)ch2 * -0.30f - (q_pitch - pitch_offset)	;
	Error_roll 	= (float)ch1 * 0.30f - (q_roll - roll_offset)	;
	
	
//	Sum_Error_yaw 	+= Error_yaw   /sampleFreq ;
//	Sum_Error_pitch += Errer_pitch /sampleFreq ;
//	Sum_Error_roll 	+= Error_roll  /sampleFreq ;
//	
//    // protect 
//	if(Sum_Error_yaw>limmit_I)Sum_Error_yaw =limmit_I;
//	if(Sum_Error_yaw<-limmit_I)Sum_Error_yaw =-limmit_I;
//	if(Sum_Error_pitch>limmit_I)Sum_Error_pitch =limmit_I;
//	if(Sum_Error_pitch<-limmit_I)Sum_Error_pitch =-limmit_I;
//	if(Sum_Error_roll>limmit_I)Sum_Error_roll =limmit_I;
//	if(Sum_Error_roll<-limmit_I)Sum_Error_roll =-limmit_I;
	
	D_Error_yaw =  (Error_yaw-Buf_D_Error_yaw)    *sampleFreq ;
	D_Error_pitch =(Errer_pitch-Buf_D_Errer_pitch)*sampleFreq ;
	D_Error_roll = Butterworth_filter2(Error_roll-Buf_D_Error_roll)  *sampleFreq ;

	
	D_Error_pitch_f = Smooth_filter(0.8f, D_Error_pitch, D_Error_pitch_f);
	D_Error_roll_f = Smooth_filter(0.8f, D_Error_roll, D_Error_roll_f); 

//	Del_yaw		= (Kp_yaw   * Error_yaw)		+ (Ki_yaw	* Sum_Error_yaw)       	 + (Kd_yaw * D_Error_yaw) ;
//	Del_pitch	= (Kp_pitch * Errer_pitch)	+ (Ki_pitch	* Sum_Error_pitch)     + (Kd_pitch * D_Error_pitch) ;
//	Del_roll	= (Kp_roll  * Error_roll)		+ (Ki_roll	* Sum_Error_roll)      + (Kd_roll * D_Error_roll) ;

	Del_yaw		= (Kp_yaw   * Error_yaw)		+ (Ki_yaw	* Sum_Error_yaw)       	 + (Kd_yaw * D_Error_yaw) ;
	Del_pitch	= (Kp_pitch * Errer_pitch)	+ (Ki_pitch	* Sum_Error_pitch)     + (Kd_pitch * D_Error_pitch) ;
	Del_roll	= (Kp_pitch  * Error_roll)		+ (Ki_pitch	* Sum_Error_roll)      + (Kd_pitch * D_Error_roll) ;

	motor_A = T_center +Del_pitch	-Del_roll -Del_yaw ;
	motor_B = T_center +Del_pitch	+Del_roll +Del_yaw ;
	motor_C = T_center -Del_pitch	+Del_roll -Del_yaw ;
	motor_D = T_center -Del_pitch	-Del_roll +Del_yaw ;
	
	// limmit output max, min
	if(motor_A < 0) motor_A = 0 ;
	if(motor_B < 0) motor_B = 0 ;
	if(motor_C < 0) motor_C = 0 ;
	if(motor_D < 0) motor_D = 0 ;
	
	if(motor_A > 2399) motor_A = 2399 ;
	if(motor_B > 2399) motor_B = 2399 ;
	if(motor_C > 2399) motor_C = 2399 ;
	if(motor_D > 2399) motor_D = 2399 ;
	
    Kd_pitch = Kd_pitch + (float)ch4*0.001f;
}

volatile void Drive_motor_output(void)
{

	TIM3 ->CCR1 = motor_A ;
	TIM3 ->CCR2 = motor_B ;
	TIM3 ->CCR4 = motor_C ;
	TIM14->CCR1 = motor_D ;

}
volatile void Interrupt_call(void)
{

		/* Read data from sensor */
		MPU6050_GetRawAccelGyro(AccelGyro);
	
		/* Controller */
		PID_controller();
	
    if (watchdog > 0) watchdog --;
	
		if((T_center < 50) || (watchdog == 0))		
		{
            
//			Sum_Error_yaw=0;
//			Sum_Error_pitch=0;
//			Sum_Error_roll=0;        
			
			motor_A=0;
			motor_B=0;
			motor_C=0;
			motor_D=0;
		}
        
		Drive_motor_output();
        
}

volatile void ahrs(void)
{
	// quaternion base process 
	float Norm;
	float ax = AccelGyro[0];
	float ay = AccelGyro[1];
	float az = AccelGyro[2];
	float gx =((AccelGyro[3]-gx_diff)/ GYROSCOPE_SENSITIVITY )*M_PI/180 ;
	float gy =((AccelGyro[4]-gy_diff)/ GYROSCOPE_SENSITIVITY )*M_PI/180 ;
	float gz =((AccelGyro[5]-gz_diff)/ GYROSCOPE_SENSITIVITY )*M_PI/180 ;
	
//	a = Smooth_filter(0.001f, AccelGyro[3], a);
//	b = Smooth_filter(0.001f, AccelGyro[4], b);
//	c = Smooth_filter(0.001f, AccelGyro[5], c);
	
	
	float q1_dot = 0.5 * (-q2 * gx - q3 * gy - q4 * gz);
	float q2_dot = 0.5 * ( q1 * gx + q3 * gz - q4 * gy);
	float q3_dot = 0.5 * ( q1 * gy - q2 * gz + q4 * gx);
	float q4_dot = 0.5 * ( q1 * gz + q2 * gy - q3 * gx);

	if(!((ax == 0) && (ay == 0) && (az == 0))) 
	{
		// Normalise 
		Norm = sqrtf(ax * ax + ay * ay + az * az);
		ax /= Norm;
		ay /= Norm;
		az /= Norm;   

		float _2q1 = 2 * q1;
		float _2q2 = 2 * q2;
		float _2q3 = 2 * q3;
		float _2q4 = 2 * q4;
		float _4q1 = 4 * q1;
		float _4q2 = 4 * q2;
		float _4q3 = 4 * q3;
		float _8q2 = 8 * q2;
		float _8q3 = 8 * q3;
		float q1q1 = q1 * q1;
		float q2q2 = q2 * q2;
		float q3q3 = q3 * q3;
		float q4q4 = q4 * q4;
		// Gradient decent 
		float s1 = _4q1 * q3q3 + _2q4 * ax + _4q1 * q2q2 - _2q2 * ay;
		float s2 = _4q2 * q4q4 - _2q4 * ax + 4 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
		float s3 = 4 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
		float s4 = 4 * q2q2 * q4 - _2q2 * ax + 4 * q3q3 * q4 - _2q3 * ay;
		// Normalise 
		Norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
		s1 /= Norm;
		s2 /= Norm;
		s3 /= Norm;
		s4 /= Norm;
		// compensate acc
		q1_dot -= (beta * s1);
		q2_dot -= (beta * s2);
		q3_dot -= (beta * s3);
		q4_dot -= (beta * s4);
	}
	// Integrate 
	q1 += q1_dot / sampleFreq;
	q2 += q2_dot / sampleFreq;
	q3 += q3_dot / sampleFreq;
	q4 += q4_dot / sampleFreq;
	// Normalise
	Norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4 );
	q1 /= Norm;
	q2 /= Norm;
	q3 /= Norm;
	q4 /= Norm;
	// convert to euler
	y_pitch =  2*(q3*q4 + q1*q2);
    
	float x =  2*(0.5 - q2*q2 - q3*q3);

	q_roll = atan2f (y_pitch,x)* -180.0f / M_PI;
	y_roll = 2*(q2*q4 - q1*q3)	;
	q_pitch = asinf(y_roll)* -180.0f / M_PI;
	q_yaw = gz* -180.0f / M_PI;
	
}

float Smooth_filter(float alfa, float new_data, float prev_data)
{
  float output = prev_data + (alfa * (new_data - prev_data));
  return output;
}

void UART_Callback(void)
{
	if (rx_buffer[index] == 0xFE && index >= 4)
	{
		watchdog = 50;
		ch1 = (int8_t)rx_buffer[index - 4];
		ch2 = (int8_t)rx_buffer[index - 3];
		ch3 = (int8_t)rx_buffer[index - 2];
		ch4 = (int8_t)rx_buffer[index - 1];
		index = 0;
	}

  index++;
  /* To avoid buffer overflow */
  if(index == 12) index = 0;

  
  /* Start another reception: provide the buffer pointer with offset and the buffer size */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)(rx_buffer + index), 1);
}

float Butterworth_filter1(float x_data1)
{
	static float x_prev_11, x_prev_21, y_data1, y_prev_11, y_prev_21;
	// lowpass filter butterworth order 2nd fc 20 hz sampling 200hz
	
	x_prev_21 = x_prev_11;
	x_prev_11 = x_data1 ;
	
	y_prev_21 = y_prev_11;
	y_prev_11 = y_data1 ;
	
	float nume = (x_data1+ 2.0f * x_prev_11 +  x_prev_21);
	float denom = (- 1.1429804563522339f * y_prev_11 +  0.412801593542099f * y_prev_21);
	y_data1 = 0.067455276846885681f * nume - denom;
	return y_data1;
}

float Butterworth_filter2(float x_data)
{ 
	static float x_prev_1, x_prev_2, y_data, y_prev_1, y_prev_2;
	// lowpass filter butterworth order 2nd fc 20 hz sampling 200hz
	
	x_prev_2 = x_prev_1;
	x_prev_1 = x_data ;
	
	y_prev_2 = y_prev_1;
	y_prev_1 = y_data ;
	
	float nume = (x_data + 2.0f * x_prev_1 +  x_prev_2);
	float denom = (- 1.1429804563522339f * y_prev_1 +  0.412801593542099f * y_prev_2);
	y_data = 0.067455276846885681f * nume - denom;
	return y_data;
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
