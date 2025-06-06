/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include "ertc-datalogger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define	HAL_TIMEOUT		1
#define TS	0.01

#define VBATT	12.0
#define V2DUTY	((float)(TIM8_ARR_VALUE+1)/VBATT)
#define DUTY2V	((float)VBATT/(TIM8_ARR_VALUE+1))

#define RPM2RADS	2*M_PI/60

#define DUTY_MAX 400

//light sensor defs
#define HAL_TIMEOUT 100
#define I2C_TIMEOUT 200
#define SX1509_I2C_ADDR1 0x3E 

#define I2C_LIGHT_SENSOR_ADDRESS SX1509_I2C_ADDR1<<1
#define LINE_SENSOR_ADDRESS 0x10

#define TIM3_ARR_VALUE 3840. // max counter value and one revolution in 4x mode.

#define OVERSAMPLING_SIZE 10
#define LINE_SENSOR_BITS 8
#define LINE_SENSOR_ACCURACY 6
#define SLIDING_WINDOW_SIZE 5
#define LINE_SENSOR_PITCH 0.008

#define WHEEL_RADIUS 0.034 // [m]
#define WHEELBASE 0.165 // [m]
#define LINE_SENSOR_DISTANCE 0.085


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
struct ertc_dlog logger;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct datalog {
	float w1, w2;
	float u1, u2;
} data;

typedef struct
{
  float TB_angular_velocity; //derived psi
  float TB_angle; //psi
  float TB_speed_set; // set speed in m/s
  float TB_jaw_error;

  //private
  int8_t TB_speed_set_rad;
} turtle_bot_attributes_t;

turtle_bot_attributes_t turtle_bot_attributers = 
{
  .TB_angular_velocity = 0,
  .TB_angle = 0,
  .TB_speed_set = 0, // rychlost robota nastaven
  .TB_jaw_error = 0,

  .TB_speed_set_rad = 0,
};

typedef struct
{
	uint8_t line_sensor_raw_reading;
  uint16_t line_sensor_accumulator[LINE_SENSOR_BITS];
  bool line_sensor_oversampled_data[LINE_SENSOR_BITS];
  uint8_t samples_counter;

  float line_sensor_distances_array[LINE_SENSOR_BITS];
  float distance_from_center;

  
  // uint16_t line_sensor_left_oversampled_reading;
  // uint16_t line_sensor_right_oversampled_reading;

  // float line_error;
  // float line_error_prev;
  // int16_t line_error_array[SLIDING_WINDOW_SIZE];
} line_sensor_attributes_t;

line_sensor_attributes_t line_folower_attributes =
{
  
  .line_sensor_raw_reading = 0,
  .line_sensor_accumulator = {0},
  .line_sensor_oversampled_data = {false},
  .samples_counter = 0,

  .line_sensor_distances_array = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5},
  .distance_from_center = 0,
  // .line_sensor_left_oversampled_reading = 0,
  // .line_sensor_right_oversampled_reading = 0,

  // .line_error = 0.,
  // .line_error_prev = 0,
  // .line_error_array = {0},
}; 

typedef struct
{
	float wheel_set_speed;
	float wheel_read_speed;
  float wheel_read_speed_ms;
  uint32_t timer1;
  uint32_t timer2;
  TIM_HandleTypeDef* encoder_timer;

  //private
  uint32_t TIM_PreviousCount;
} motor_attributes_t;

motor_attributes_t motor_attributes_left =
 {
    .wheel_set_speed = 0,
    .wheel_read_speed = 0,
    .wheel_read_speed_ms = 0,
    .timer1 = TIM_CHANNEL_1,
    .timer2 = TIM_CHANNEL_2,
    .encoder_timer = &htim3,
 };

 motor_attributes_t motor_attributes_right =
 {
    .wheel_set_speed = 0,
    .wheel_read_speed = 0,
    .wheel_read_speed_ms = 0,
    .timer1 = TIM_CHANNEL_3,
    .timer2 = TIM_CHANNEL_4,
    .encoder_timer = &htim4,
 };

typedef struct
{
	double Td;
  double Ti;
  double Kr;

	double integration;
	double derivation;

	double error;
	double error_previous;

	double output;
} pid_attributes_t;

pid_attributes_t pid_attributes_left =
 {
  .Td = 0,
  .Ti = 0,
  .Kr = 0,

	.integration = 0,
	.derivation = 0,

//private attributes
	.error = 0,
	.error_previous = 0,

	.output = 0
 };

 pid_attributes_t pid_attributes_right =
 {
  .Td = 0,
  .Ti = 0,
  .Kr = 0,

	.integration = 0,
	.derivation = 0,

//private attributes
	.error = 0,
	.error_previous = 0,

	.output = 0
 };

 pid_attributes_t pid_attributes_jaw =
 {
  .Td = 0,
  .Ti = 0,
  .Kr = 0,


	.integration = 0,
	.derivation = 0,

//private attributes
	.error = 0,
	.error_previous = 0,

	.output = 0
 };

void pidRecount(pid_attributes_t * pid_attributes, motor_attributes_t * motor_attributes)
{
	pid_attributes->error_previous = pid_attributes->error;
	pid_attributes->error =  motor_attributes->wheel_set_speed - motor_attributes->wheel_read_speed;

	pid_attributes->derivation = pid_attributes->error - pid_attributes->error_previous;

  if(( pid_attributes->integration +100 < 3000) && (pid_attributes->integration -100 > -3000))
    pid_attributes->integration += pid_attributes->error;

  double integral_part = (TS/pid_attributes->Ti)*pid_attributes->integration;
  
  double derivate_part = (pid_attributes->Td/TS)*pid_attributes->derivation;
  
  pid_attributes->output = pid_attributes->Kr*(pid_attributes->error + derivate_part + integral_part);

}

uint32_t recountDuty(float speed)
{

  if(speed > 120)
	  speed = 120;
  if(speed < -120)
	  speed = -120;

  int speed_2 = (int)speed;


 return (uint32_t)((((float)speed_2)/200.)*DUTY_MAX);


}

void updateMotorSpeedFC(motor_attributes_t * motor_attributes, float speed)
{
	uint32_t duty = recountDuty(speed);

	if(motor_attributes->wheel_set_speed >= 0)
	{
	  __HAL_TIM_SET_COMPARE(&htim8, motor_attributes->timer1, duty);
	  __HAL_TIM_SET_COMPARE(&htim8,  motor_attributes->timer2, 0);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim8,  motor_attributes->timer1, 0);
		__HAL_TIM_SET_COMPARE(&htim8,  motor_attributes->timer2, duty);
	}
}

void updateMotorSpeedFB(motor_attributes_t * motor_attributes, float speed)
{
	uint32_t duty = recountDuty(speed);

	if(motor_attributes->wheel_set_speed >= 0)
	{
	  __HAL_TIM_SET_COMPARE(&htim8, motor_attributes->timer1,  DUTY_MAX);
	  __HAL_TIM_SET_COMPARE(&htim8,  motor_attributes->timer2, DUTY_MAX - duty);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim8,  motor_attributes->timer1, DUTY_MAX - duty);
		__HAL_TIM_SET_COMPARE(&htim8,  motor_attributes->timer2, DUTY_MAX);
	}
}


float speedRecountToRs(int32_t speed_in_pulses)
{
  return ((float)speed_in_pulses/TIM3_ARR_VALUE)*((2.*M_PI)/(TS));
}

float speedRecountToMs(int32_t speed_in_pulses)
{
  return ((float)speed_in_pulses/TIM3_ARR_VALUE)*((2.*M_PI)/(TS)) * WHEEL_RADIUS;
}

void getSpeed(motor_attributes_t * motor_attributes)
{
  uint32_t TIM_CurrentCount;
  int32_t TIM_DiffCount;
  static uint32_t TIM_PreviousCount = 0;
 // float speed = 0; // speed in rpm

  TIM_CurrentCount = __HAL_TIM_GET_COUNTER(motor_attributes->encoder_timer);

  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(motor_attributes->encoder_timer))
  {
    if (TIM_CurrentCount <= motor_attributes->TIM_PreviousCount)
      TIM_DiffCount = TIM_CurrentCount - motor_attributes->TIM_PreviousCount;
    else
      TIM_DiffCount = -((TIM3_ARR_VALUE + 1) - TIM_CurrentCount) - motor_attributes->TIM_PreviousCount;
  }

  else
  {
    if (TIM_CurrentCount >= motor_attributes->TIM_PreviousCount)
      TIM_DiffCount = TIM_CurrentCount - motor_attributes->TIM_PreviousCount;
    else
      TIM_DiffCount = ((TIM3_ARR_VALUE + 1) - motor_attributes->TIM_PreviousCount) + TIM_CurrentCount;
  }

  motor_attributes->TIM_PreviousCount = TIM_CurrentCount;

  motor_attributes->wheel_read_speed = speedRecountToRs(TIM_DiffCount) * 4; // magic constant to get grater swing in int values
  motor_attributes->wheel_read_speed_ms = speedRecountToMs(TIM_DiffCount);
}

void LineSensorGetStatus(line_sensor_attributes_t * lignt_folower_attributes)
{
  HAL_I2C_Mem_Read(&hi2c1, I2C_LIGHT_SENSOR_ADDRESS, LINE_SENSOR_ADDRESS , 1, &lignt_folower_attributes->line_sensor_raw_reading, 1, I2C_TIMEOUT);
}

void LineSensorAccumulate(line_sensor_attributes_t * line_attributes)
{
  uint8_t line_sensor_reading_tmp = 0;
  for(int i = 0; i <LINE_SENSOR_BITS; ++i)
  {
    line_sensor_reading_tmp = line_attributes->line_sensor_raw_reading;
    line_sensor_reading_tmp =  line_sensor_reading_tmp << (LINE_SENSOR_BITS-1-i);
    line_sensor_reading_tmp =  line_sensor_reading_tmp >> LINE_SENSOR_BITS-1;
    
    line_attributes->line_sensor_accumulator[i] += line_sensor_reading_tmp;
  }
  line_attributes->samples_counter++;
}

void LineSensorOversample(line_sensor_attributes_t * line_attributes)
{
  if(line_attributes->samples_counter == OVERSAMPLING_SIZE)
  {
    line_attributes->samples_counter = 0;
    for(int i = 0; i <LINE_SENSOR_BITS; ++i)
    {
      if(line_attributes->line_sensor_accumulator[i] >= LINE_SENSOR_ACCURACY)
        line_attributes->line_sensor_oversampled_data[i] = true;
      else
        line_attributes->line_sensor_oversampled_data[i] = false;
      line_attributes->line_sensor_accumulator[i] = 0;
    }
  }
}

uint16_t GetNumberFromBitArray(uint16_t * array, uint8_t bits)
{
  uint16_t value = 0;
  for(int i = 0; i < bits; ++i)
  {
    value += (array[i] * pow(2,i));
  }
  return value;
}


// void LineSensorSplitLeftAndRight(line_sensor_attributes_t * line_attributes)
// {
//   uint16_t line_sensor_left_oversampled_reading_array[LINE_SENSOR_BITS/2];
//   uint16_t line_sensor_right_oversampled_reading_array[LINE_SENSOR_BITS/2];

//   for(int i = 0; i <LINE_SENSOR_BITS/2; ++i)
//   {
//     //left
//     line_sensor_left_oversampled_reading_array[i] = line_attributes->line_sensor_oversampled_data[(LINE_SENSOR_BITS/2) + i];
  
//     //right
//     line_sensor_right_oversampled_reading_array[i] = line_attributes->line_sensor_oversampled_data[(LINE_SENSOR_BITS/2) - 1 - i];
//   }

//   line_attributes->line_sensor_left_oversampled_reading = GetNumberFromBitArray(line_sensor_left_oversampled_reading_array, LINE_SENSOR_BITS/2);
//   line_attributes->line_sensor_right_oversampled_reading = GetNumberFromBitArray(line_sensor_right_oversampled_reading_array, LINE_SENSOR_BITS/2);
// }

// void LineSensorComputeErrorSlidingWindow(line_sensor_attributes_t * line_attributes)
// {
//   line_attributes->line_error_prev = line_attributes->line_error;
//   line_attributes->line_error = line_attributes->line_sensor_right_oversampled_reading - line_attributes->line_sensor_left_oversampled_reading;
//   int16_t new_err_tmp = line_attributes->line_error;

//   for(int i = 0; i < (SLIDING_WINDOW_SIZE - 1); ++i)
//   {
//     line_attributes->line_error_array[i] = line_attributes->line_error_array[i+1];
//     line_attributes->line_error += line_attributes->line_error_array[i];
//   }

//   line_attributes->line_error_array[SLIDING_WINDOW_SIZE - 1] = new_err_tmp;
//   line_attributes->line_error /= SLIDING_WINDOW_SIZE;
// }

// void StupidMotorControl(line_sensor_attributes_t * line_attributes, motor_attributes_t * motor_attributes_left, motor_attributes_t * motor_attributes_right)
// {
//   // if(line_attributes->line_error_prev > 0)
//   // {
//   //   //motor_attributes_right->wheel_set_speed -= line_attributes->line_error_prev;
//   //   motor_attributes_right->wheel_set_speed -= line_attributes->line_error_prev;
//   // }
//   // else
//   // {
//   //   motor_attributes_left->wheel_set_speed -= line_attributes->line_error_prev;
//   //   //motor_attributes_left->wheel_set_speed -= line_attributes->line_error_prev;
//   // }

//   if(line_attributes->line_error > 0)
//     {
//       motor_attributes_right->wheel_set_speed = 40 + line_attributes->line_error * 0.6;
//       motor_attributes_left->wheel_set_speed = 40;
//       //motor_attributes_left->wheel_set_speed -= line_attributes->line_error;
//     }
//   else
//   {
//     //motor_attributes_right->wheel_set_speed -= line_attributes->line_error;

//     motor_attributes_left->wheel_set_speed = 40 - line_attributes->line_error * 0.6;
//     motor_attributes_right->wheel_set_speed = 40;
//   }
  
// }

void LineSensorComputeDistanceFromCenter(line_sensor_attributes_t * line_attributes)
{
  float sum_dist = 0;
  float sum_bits = 0;

  for(int i = 0; i < LINE_SENSOR_BITS; ++i)
  {
    sum_dist += line_attributes->line_sensor_distances_array[i]*line_attributes->line_sensor_oversampled_data[i];
    sum_bits += line_attributes->line_sensor_oversampled_data[i];
  }

  line_attributes->distance_from_center = (sum_dist/(sum_bits + 0.00001)) * LINE_SENSOR_PITCH; // distance in [cm]
}

void LineSensorComupteAngulerVelocityOfTurtleBot(turtle_bot_attributes_t * trurtle_bot, motor_attributes_t * motor_attributes_left, motor_attributes_t * motor_attributes_right)
{
  trurtle_bot->TB_angular_velocity = (motor_attributes_right->wheel_read_speed_ms - motor_attributes_left->wheel_read_speed_ms)/WHEELBASE;
}

//private
float TBSpeedRecountToRadS(float speed)
{
  return speed / WHEEL_RADIUS;
}

//public
// void TBSpeedRecountLR(turtle_bot_attributes_t * trurtle_bot,  motor_attributes_t * motor_attributes_left, motor_attributes_t * motor_attributes_right)
// {
//   TBSpeedRecountLR(trurtle_bot);
//   motor_attributes_right->wheel_set_speed = TBSpeedRecountToRadS(trurtle_bot->TB_speed_set + trurtle_bot->TB_angular_velocity * WHEELBASE/2);
//   motor_attributes_left->wheel_set_speed = TBSpeedRecountToRadS(trurtle_bot->TB_speed_set - trurtle_bot->TB_angular_velocity * WHEELBASE/2);
// }

void TBComputeJawError(line_sensor_attributes_t * line_attributes, turtle_bot_attributes_t * trurtle_bot)
{
  trurtle_bot->TB_jaw_error = (line_attributes->distance_from_center/LINE_SENSOR_DISTANCE);
}

void TBComputeAngle(turtle_bot_attributes_t * turtle_bot)
{
  turtle_bot->TB_angle += turtle_bot->TB_angular_velocity;
}

void TBComputePID(pid_attributes_t * pid_attributes, turtle_bot_attributes_t * TB)
{
  pid_attributes->output = pid_attributes->Kr*(TB->TB_jaw_error);
}

void TBMapValuesToWheels(pid_attributes_t * pid_attributes, turtle_bot_attributes_t * TB, motor_attributes_t * motor_attributes_left, motor_attributes_t * motor_attributes_right)
{
  motor_attributes_right->wheel_set_speed = (TB->TB_speed_set + pid_attributes->output * WHEELBASE/2);
  motor_attributes_left->wheel_set_speed = (TB->TB_speed_set - pid_attributes->output * WHEELBASE/2);
}

int count = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{




  if(htim->Instance == TIM7)
	{
    LineSensorGetStatus(&line_folower_attributes);
    LineSensorAccumulate(&line_folower_attributes);
    LineSensorOversample(&line_folower_attributes);
    
    getSpeed(&motor_attributes_left);
    getSpeed(&motor_attributes_right);


    if(line_folower_attributes.samples_counter == OVERSAMPLING_SIZE-1) 
    {
      
    	
      //TBSpeedRecountLR(&turtle_bot_attributers, &motor_attributes_left, &motor_attributes_right);
      LineSensorComputeDistanceFromCenter(&line_folower_attributes);

      TBComputeJawError(&line_folower_attributes, &turtle_bot_attributers);

      LineSensorComupteAngulerVelocityOfTurtleBot(&turtle_bot_attributers, &motor_attributes_left, &motor_attributes_right);
      TBComputeAngle(&turtle_bot_attributers);

      TBComputePID(&pid_attributes_jaw, &turtle_bot_attributers);
      
      TBMapValuesToWheels(&pid_attributes_jaw, &turtle_bot_attributers, &motor_attributes_left, &motor_attributes_right);

      count ++;

	  if(count > 20)
    {
		  //turtle_bot_attributers.TB_speed_set = 30;
      //turtle_bot_attributers.TB_speed_set += 0.1 ;
    }

     //StupidMotorControl(&line_folower_attributes,&motor_attributes_left,&motor_attributes_right);
   }
      pidRecount(&pid_attributes_left, &motor_attributes_left);
      updateMotorSpeedFC(&motor_attributes_left, pid_attributes_left.output);
      
      pidRecount(&pid_attributes_right, &motor_attributes_right);
      updateMotorSpeedFC(&motor_attributes_right, pid_attributes_right.output);

	// if(htim->Instance == TIM7)
	// {
  //   LineSensorGetStatus(&line_folower_attributes);
  //   LineSensorAccumulate(&line_folower_attributes);
  //   LineSensorOversample(&line_folower_attributes);
    
  //   if(line_folower_attributes.samples_counter == OVERSAMPLING_SIZE-1) 
  //   {
  //    LineSensorSplitLeftAndRight(&line_folower_attributes);
  //    LineSensorComputeErrorSlidingWindow(&line_folower_attributes);

  //    getSpeed(&motor_attributes_left);
  //

    //  Recount(&pid_attributes_left, &motor_attributes_left);
  //    updateMotorSpeedFC(&motor_attributes_left, pid_attributes_left.output);
     
  //    getSpeed(&motor_attributes_right);
  //    pidRecount(&pid_attributes_right, &motor_attributes_right);
  //    updateMotorSpeedFC(&motor_attributes_right, pid_attributes_right.output);

  //    //StupidMotorControl(&line_folower_attributes,&motor_attributes_left,&motor_attributes_right);
  //  }

  }

  

	static int kLed = 0;

	/* Speed ctrl routine */
	if(htim->Instance == TIM6)
	{
     	/*	Prepare data packet
		data.w1 = motor_attributes_right.wheel_read_speed;
		data.w2 = turtle_bot_attributers.TB_speed_set;
		data.u1 =  motor_attributes_right.wheel_read_speed;;
		data.u2 =pid_attributes_right.output ;
		*/

    //right wheel detailed description

  /*   data.w1 = motor_attributes_right.wheel_read_speed;
		 data.w2 = turtle_bot_attributers.TB_speed_set;
		 data.u1 = pid_attributes_right.error;
		 data.u2 = pid_attributes_right.output;
*/
    //left wheel detailed description
		/*
     data.w1 = motor_attributes_left.wheel_read_speed;
		 data.w2 = turtle_bot_attributers.TB_speed_set;
		 data.u1 = pid_attributes_left.error;
		 data.u2 = pid_attributes_left.output;
		 */

    //jaw controller - change structure
     data.w1 = motor_attributes_right.wheel_set_speed;
		 data.w2 = motor_attributes_left.wheel_set_speed;
		 data.u1 = line_folower_attributes.distance_from_center;
		 data.u2 = pid_attributes_jaw.output;

		ertc_dlog_send(&logger, &data, sizeof(data));

		// Indicate that the program is running
		if(++kLed >= 10)
		{
			kLed = 0;
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
	}
}
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
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM9_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  logger.uart_handle = huart3; // for serial
  //logger.uart_handle = huart2; // for wifi

  /* Reset LCD */
  HAL_GPIO_WritePin(GPIO_OUT_SPI_CS_LCD_GPIO_Port, GPIO_OUT_SPI_CS_LCD_Pin, GPIO_PIN_SET);

  HAL_Delay(1000);

  /* Start encoders timers */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  /* Start servomotors PWM (avoid floating inputs to servomotors) */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  /* Start motor PWM */
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  /* Start speed ctrl ISR */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
/*
    pid_attributes_left.Ti = 33.56;
    pid_attributes_left.Td = 0.59;
    pid_attributes_left.Kr = 10;

    pid_attributes_right.Ti = 31.39;
    pid_attributes_right.Td = 0.54;
    pid_attributes_right.Kr = 9.42;

    pid_attributes_jaw.Kr = 2000;


   */
  /*
  	pid_attributes_left.Ti = 98;
    pid_attributes_left.Td = 0.1;
    pid_attributes_left.Kr = 11.95;

    pid_attributes_right.Ti = 94;
    pid_attributes_right.Td = 0.1;
    pid_attributes_right.Kr = 11.53;

    pid_attributes_jaw.Kr = 323;
    */


         pid_attributes_left.Ti = 0.1;
           pid_attributes_left.Td = 0;
           pid_attributes_left.Kr = 2;

           pid_attributes_right.Ti = 0.1;
           pid_attributes_right.Td = 0;
           pid_attributes_right.Kr = 2;

           pid_attributes_jaw.Kr = 800;


  /*
  	  pid_attributes_left.Ti = 68;
      pid_attributes_left.Td = 0.61;
      pid_attributes_left.Kr = 6;

      pid_attributes_right.Ti = 59.2916;
      pid_attributes_right.Td = 0.55;
      pid_attributes_right.Kr = 5.5;

      pid_attributes_jaw.Kr = 1152;
      */
/*
  pid_attributes_left.Ti = 25;
    pid_attributes_left.Td = 0;
    pid_attributes_left.Kr = 2;

    pid_attributes_right.Ti = 25;
    pid_attributes_right.Td = 0;
    pid_attributes_right.Kr = 2;

    pid_attributes_jaw.Kr = 1400;

*/
/*
  pid_attributes_left.Ti = 2;
  pid_attributes_left.Td = 0;
  pid_attributes_left.Kr = 15;

  pid_attributes_right.Ti = 2;
  pid_attributes_right.Td = 0;
  pid_attributes_right.Kr = 15;

  pid_attributes_jaw.Kr = 800;
  */

  // between 500 and 850 the best run was 700


  turtle_bot_attributers.TB_speed_set = 30;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // @suppress("Float formatting support")
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
	  ertc_dlog_update(&logger);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_FORCED_ACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIM3_ARR_VALUE;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim4.Init.Period = TIM4_ARR_VALUE;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = TIM6_PSC_VALUE;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = TIM6_ARR_VALUE;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9599;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 100;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = TIM8_PSC_VALUE;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = TIM8_ARR_VALUE;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_OUT_SPI_CS_SDCARD_Pin|GPIO_OUT_SPI_CS_LCD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIO_OUT_SPI_CS_SDCARD_Pin GPIO_OUT_SPI_CS_LCD_Pin */
  GPIO_InitStruct.Pin = GPIO_OUT_SPI_CS_SDCARD_Pin|GPIO_OUT_SPI_CS_LCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_EXTI2_PROXY_TOF_SENS_IRQ_Pin GPIO_EXTI3_IMU_IRQ_Pin GPIO_EXTI4_KPAD_IRQ_Pin GPIO_EXTI8_USER_BUT1_IRQ_Pin
                           GPIO_EXTI9_USER_BUT2_IRQ_Pin GPIO_EXTI10_BUMP1_IRQ_Pin GPIO_EXTI11_BUMP2_IRQ_Pin GPIO_EXTI12_BUMP3_IRQ_Pin
                           GPIO_EXTI13_BUMP4_IRQ_Pin */
  GPIO_InitStruct.Pin = GPIO_EXTI2_PROXY_TOF_SENS_IRQ_Pin|GPIO_EXTI3_IMU_IRQ_Pin|GPIO_EXTI4_KPAD_IRQ_Pin|GPIO_EXTI8_USER_BUT1_IRQ_Pin
                          |GPIO_EXTI9_USER_BUT2_IRQ_Pin|GPIO_EXTI10_BUMP1_IRQ_Pin|GPIO_EXTI11_BUMP2_IRQ_Pin|GPIO_EXTI12_BUMP3_IRQ_Pin
                          |GPIO_EXTI13_BUMP4_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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
	  static uint32_t kLed = 0;
	  if(++kLed >= 1000)
	  {
		  kLed = 0;
		  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	  }
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
