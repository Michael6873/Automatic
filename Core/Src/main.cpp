/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include <RPLidar.h>
#include <cstring>
#include  <cmath>

#define ENC_MAX 1800// максимальное количество тиков за оборот
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float distance = 0;
float angle = 0;
bool  startBit;
uint8_t  quality;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

char str[100];
uint32_t x= 12345;
float y= 1.2345678;
uint8_t z= 0x61;
char str1[]= "РЎС‚СЂРѕРєР°\0";

double errorAngle;
double currentAngle = 0;

volatile int32_t EncoderValue = 0;
uint32_t T1 = 0;
int32_t deltaTime = 100;
int32_t previousPosition = 0;
int32_t speed = 0;
uint32_t PWM = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

int32_t constrain(int32_t value,int32_t num1,int32_t num2){
	if (value>num2) value = num2;
	if (value<num1) value = num1;
	return value;
}


class Motor{

public:

	Motor(TIM_HandleTypeDef *ht): time(0),deltaTime(100),htim(ht),flagEnc(true)
	{}

	int32_t getSpeedMotor(){
		encoderValue = getEncoderValue();
		if (HAL_GetTick()-time>=deltaTime){
			time = HAL_GetTick();
			difPosition = previousPosition-encoderValue;
			speed = ((abs(difPosition))*60000)/(ENC_MAX*deltaTime);
			  if(abs(difPosition)>=1000) speed = helpSpeed;
			  previousPosition = encoderValue;
			  helpSpeed = speed;
		}
		return speed;
	}

	void returnMotor(uint32_t angle){

		if (flagEnc) {
			flagEnc = false;
			currentTic = getEncoderValue();
		}
		uint32_t numberTic  = ENC_MAX * angle/360;
		if (abs(getEncoderValue()-currentTic) <=numberTic) TIM1->CCR4 = 1000;
		else {
			TIM1->CCR4 = 0;
			flagEnc  =true;
		}

	}
	void setEncoderValue(){
		encoderValue = htim->Instance->CNT;
	}

	bool getDirectionMotor(){
		if (difPosition>0) return 1;
		else if (difPosition <0) return 0;
	}

	int32_t getEncoderValue(){
		return encoderValue;
	}
private:

	    void setRightMotor(int32_t rightSpeed){
	    	if (rightSpeed>0){
	    		TIM1->CCR4 = 0;
	    		TIM1->CCR3 = rightSpeed;
	    	}

	    	if (rightSpeed<0){
	    			TIM1->CCR3 = 0;
	    			TIM1->CCR4 = abs(rightSpeed);
	    		}
	    }
	    void setLeftMotor(int32_t leftSpeed){
	    	if (leftSpeed>0)
	    	{
	    		TIM1->CCR2 = 0;
	    		TIM1->CCR1 = leftSpeed;
	    	}

	    	if (leftSpeed<0)
	    		{
	    			TIM1->CCR1 = 0;
	    			TIM1->CCR2 = abs(leftSpeed);
	    		}
	    }
TIM_HandleTypeDef *htim;
uint32_t time;
int32_t deltaTime;
int32_t helpSpeed;
int32_t speed;
int32_t encoderValue;
int32_t previousPosition;
int32_t difPosition;

int32_t currentTic;
bool flagEnc;
};

class PID {
public:
    PID(double kp, double ki, double kd)
        : kp(kp), ki(ki), kd(kd), prevError(0), integral(0) {
    }

    // Метод для расчета управляющего воздействия
    double calculate(double target, double current) {
        double error = target - current;
        integral += error; // Накопление интегральной ошибки
        double derivative = error - prevError; // Разница ошибок
        prevError = error;

        return kp * error + ki * integral + kd * derivative;
    }

private:
    double kp, ki, kd; // Коэффициенты PID
    double prevError; // Предыдущая ошибка
    double integral;   // интегральная ошибка
};


/*class Robot {
public:
    Robot()
        : leftMotorSpeed(0), rightMotorSpeed(0),minBorderSpeed(0),maxBorderSpeed(100),
		  defaultSpeedMotorLeft(50),defaultSpeedMotorRight(50), leftSpeed(0), rightSpeed(0),
        turnPid(1.0, 1.0, 0.5), leftMotorPid(10,0.0005,1), rightMotorPid(10,0.0005,1), enLeftMot(&htim2),enRightMot(&htim4) {

    	lidar.begin();
    	HAL_Delay(100);
    	lidar.startScan();
    	HAL_Delay(100);
    } // �?нициализация v


    void process(){
    	checkErrorLidar();
        int turnSpeed = getErrorAngle();
        setMotorSpeeds(-(int32_t)turnSpeed+defaultSpeedMotorLeft, (int32_t)turnSpeed+defaultSpeedMotorRight); // Устанавливаем скорости моторов
    }

    void makeTurnMotor(uint32_t quantity, bool choiceMotor, bool direction){
        // 1 - левый мотора, 0 - правый, Для dir: 1- вперед, 0 - назад, quantity - количество полных оборотов

    	if(choiceMotor){

        	int32_t encValue;
    		if (direction) setLeftMotor(leftMotorPid.calculate(defaultSpeedMotorLeft, leftSpeed));
    		else if (!direction) setLeftMotor(leftMotorPid.calculate(-defaultSpeedMotorLeft, leftSpeed));
    	}
    	else if (!choiceMotor){

    	}
    }

private:

	void setMotorSpeeds(int32_t leftSpeed, int32_t rightSpeed) {
		setLeftMotor(constrain(leftSpeed,-maxBorderSpeed,maxBorderSpeed));
		setRightMotor(constrain(rightSpeed,-maxBorderSpeed,maxBorderSpeed));
	}



    void setRightMotor(int32_t rightSpeedMotor){
    	rightSpeed = rightSpeedMotor;
    	if (rightSpeed>0){
    		TIM1->CCR4 = 0;
    		TIM1->CCR3 = rightSpeed;
    	}

    	if (rightSpeed<0){
    			TIM1->CCR3 = 0;
    			TIM1->CCR4 = abs(rightSpeed);
    		}
    }
    void setLeftMotor(int32_t leftSpeedMotor){
    	leftSpeed = leftSpeedMotor;
    	if (leftSpeed>0)
    	{
    		TIM1->CCR2 = 0;
    		TIM1->CCR1 = leftSpeed;
    	}

    	if (leftSpeed<0)
    		{
    			TIM1->CCR1 = 0;
    			TIM1->CCR2 = abs(leftSpeed);
    		}
    }


    int getErrorAngle(){
  	minDistance = 1000;
  	  for(int i = 0;i<361;i++){
  		  if(lidar.getDistances(i)<minDistance){
  			  minDistance = lidar.getDistances(i);
  			  if(i>180) currentAngle = -1*(360-i);
  			  else currentAngle = i;
  		  }
  	  }

  	  return turnPid.calculate(0, currentAngle);
    }
    void checkErrorLidar(){
  	  if (!IS_OK(lidar.waitPoint())) {
        // Пытаемся обнаружить RPLidar
        rplidar_response_device_info_t info;
        if (IS_OK(lidar.getDeviceInfo(info, 100))) {
          lidar.startScan(); // Запускаем сканирование
          HAL_Delay(100);
        }
      }
    }


    double leftMotorSpeed;  // Скорость левого двигателя
    double rightMotorSpeed; // Скорость правого двигателя
    double currentAngle;
    float minDistance;

    int32_t minBorderSpeed;
    int32_t maxBorderSpeed;
    int32_t defaultSpeedMotorLeft;
    int32_t defaultSpeedMotorRight;
    int32_t leftSpeed;
    int32_t rightSpeed;

    PID turnPid;   // PID-регулятор для поворота
    PID leftMotorPid;   // PID-регулятор для поворота
    PID rightMotorPid;   // PID-регулятор для поворота

    Encoder enLeftMot; // обьект энкодера на левом моторе
    Encoder enRightMot;// обьект энкодера на правом моторе

    RPLidar lidar;
};




class stateMachine{

public:


void process(){
	switch(state){
	case FIND:
		findState();
		break;
	case ATTACK:
		attackState();
		break;
	}
}
private:
void findState(){

	touchBorder();
}
void attackState(){

	touchBorder();
}
void touchBorder(){

	if (borderFlag){

	}
}

int32_t state;
enum {
	FIND,
	ATTACK,
};
bool borderFlag = false;

};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);


  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  RPLidar lidar;
  PID pid(10,0.0005,1);
  lidar.begin();
  lidar.startScan();
  HAL_Delay(100);


  char str1[100];
  int i = 0;
  float h = 0;
uint32_t T= HAL_GetTick();
uint32_t c= 0;
float minDistance = 1000;
uint32_t helpSpeed = 0;
  PID pidMotor(10,0.0005,1);

  Motor motor(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  motor.setEncoderValue();
	  motor.returnMotor(45);







/*
	  if (IS_OK(lidar.waitPoint())) {
	    distance = lidar.getCurrentPoint().distance; //distance value in mm unit
	    angle    = lidar.getCurrentPoint().angle; //anglue value in degree
	    startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
	    quality  = lidar.getCurrentPoint().quality; //quality of the current measuremen
  	  } else {
      rplidar_response_device_info_t info;
      if (IS_OK(lidar.getDeviceInfo(info, 100))) {
        lidar.startScan(); // Р—Р°РїСѓСЃРєР°РµР�? СЃРєР°РЅРёСЂРѕРІР°РЅРёРµ
        HAL_Delay(100);
      }
    }


	  for(int i = 0;i<361;i++){

		  if(lidar.getDistances(i)<minDistance){
			  minDistance = lidar.getDistances(i);
			  if(i>180) currentAngle = -1*(360-i);
			  else currentAngle = i;
		  }
	  }

	  errorAngle = pid.calculate(0, currentAngle);





//	  if (lidar.getDistances((int)angle)>100&&lidar.getDistances((int)angle)<200){
	  sprintf(str1,"%.1f,%.1f\n", angle, lidar.getDistances((int)angle));
	  //sprintf(str1, "%.1f,%.1f\n", angle, distance);

	  HAL_UART_Transmit(&huart1, (uint8_t*)str1, strlen((char *)str1), 10);
//	  }


	  if (HAL_GetTick() - T>100) {
		  T = HAL_GetTick();
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  }
	  PWM = (uint32_t)constrain(pidMotor.calculate(80, speed),0,10000);
	  TIM1->CCR4 = PWM;
	  EncoderValue = TIM2->CNT;

//	  	if (EncoderValue<=65000-1800){
//	  		TIM1->CCR1 = 0;
//	  	}
		if (HAL_GetTick()-T1>=deltaTime){
			T1 = HAL_GetTick();
			speed = abs(((previousPosition-EncoderValue)*60000)/(1800*deltaTime));
			  if(abs(EncoderValue-previousPosition)>=1000) speed = helpSpeed;
			  previousPosition = EncoderValue;
			  helpSpeed = speed;
		}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
