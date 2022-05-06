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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define usTIM TIM8
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_ANCHORS 4
#define SBUS_TX_LENGTH 25
#define SBUS_RX_LENGTH2 25*2
#define SBUS_RX_LENGTH 25
#define SBUS_START 0x0f
#define SBUS_END 0x00
#define SBUS_CHANNELS 16
#define SBUS_CHANNEL_MIN 173
#define SBUS_CHANNEL_MAX 1812
#define PID_MAX 500
#define PID_MIN -500
#define RC_MAX 2000
#define RC_MIN 1000
#define UWB_UART_LENGTH 12
#define UWB_PACKET_BUFFER_SIZE 20
#define UWB_MAX_PACKET_LOSS 50
#define LOG_LENGTH 200
#define SPI_RX_DATA_SIZE 90
#define SPI_RX_BUF_SIZE 90
#define SPI_TX_BUF_SIZE 90
#define ULTRASONIC_TIMEOUT_MS 20
#define ULTRASONIC_MIN_DISTANCE_CM 5
#define ULTRASONIC_MAX_DISTANCE_CM 500
//#define ULTRASONIC_FILTER_SIZE 5
#define MAX_FILTER_SIZE 10
#define UWB_FILTER_SIZE 2
//#define TRIG_ANGLE_FILTER_SIZE 5
//#define TRIG_DISTANCE_FILTER_SIZE 2
#define UWB_RX_TO_RX_DISTANCE_CM 29.5
#define pi 3.14159
#define conv_to_deg 180/pi
#define PID_period 0.1 //seconds
//ALTITUDE SETTINGS
#define SETPOINT_ALTITUDE 100 //cm
#define DEADZONE_ALTITUDE 10;
//#define P_altitude 0.1
#define USONIC_FILTER_SIZE 2
#define P_altitude 1
#define I_altitude 0.05
#define D_altitude 0.05
#define PID_MIN_ALT -200
#define PID_MAX_ALT 200
#define PID_MIN_ALT_I -50
#define PID_MAX_ALT_I 50
//ANGLE SETTINGS
#define SETPOINT_ANGLE 0 //degrees
#define DEADZONE_ANGLE 20
#define TRIG_ANGLE_FILTER_SIZE 10
#define P_angle 3
#define I_angle 0.25
#define D_angle 0.3
//DISTANCE SETTINGS
#define SETPOINT_DISTANCE 400 //cm
#define TRIG_DISTANCE_FILTER_SIZE 2
#define P_distance 2
#define I_distance 0.025
#define D_distance 0.03
#define D_FILTER_TIME_CONSTANT 0.0106
//DISPLAY SETTINGS
#define DISPLAY_RX_LENGTH 7
#define DISPLAY_TX_LENGTH 13
#define DISPLAY_START_PERIODIC 0x5a
#define DISPLAY_START_ANGLE_PID 0x5b
#define DISPLAY_START_DISTANCE_PID 0x5c
#define DISPLAY_START_ALTITUDE_PID 0x5d
#define DISPLAY_START_FLIGHT_SETTINGS 0x5e
#define MICROSD_BUFFER_SIZE 1000
#define PIDSUM_MAX 500
#define PIDSUM_MIN -500
#define MAX_DROPPED_SBUS_PACKETS 4
#define U_TURN_DURATION 10//x100ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
uint8_t begin_tracking = 0;
float last_state_rcin6 = 1000;
char microsd_buf[MICROSD_BUFFER_SIZE];
uint16_t microsd_buf_size = 0;
//Timers
uint32_t time_10hz = 0;
uint32_t last_time_10hz = 0;
uint32_t delay = 0;
volatile uint8_t tmr_flag_1s = 0;
volatile uint8_t tmr_flag_9ms = 0;
volatile uint8_t tmr_flag_100ms = 0;
uint8_t arm_flag = 0;
uint8_t uart_error = 0;
uint8_t rxUART7[UWB_UART_LENGTH] = {0};//UWB1 buferis
uint8_t rxUART8[UWB_UART_LENGTH] = {0};//UWB2 buferis
uint8_t rxUART6[SBUS_RX_LENGTH2] = {0};//uart buferis
uint8_t txUART5[SBUS_TX_LENGTH] = {0};
uint8_t txUART7[LOG_LENGTH] = {0};
uint8_t rxUART4[DISPLAY_RX_LENGTH] = {0};
uint8_t txUART4[DISPLAY_TX_LENGTH] = {0};
uint8_t display_flag = 0;
//SBUS variables
uint8_t sbus_receive_length = SBUS_RX_LENGTH2;
uint8_t sbus_frame_start = 0;
uint8_t rc_bad_frame = 0;
uint8_t rc_packets = 0;
uint8_t rc_no_pulses = 0;
uint16_t rc_in_raw[SBUS_CHANNELS] = {0};
uint16_t rc_out_raw[SBUS_CHANNELS] = {0};
float rc_in[SBUS_CHANNELS] = {0};
float rc_out[SBUS_CHANNELS] = {0};
uint8_t rc_frame_lost = 0;
uint8_t rc_failsafe = 0;
//UWB variables
uint8_t uwb1_uart_received = 0;
uint8_t uwb2_uart_received = 0;
struct loc
{
	uint32_t node_pos_x;
	uint32_t node_pos_y;
	uint32_t node_pos_z;
	uint32_t node_pos_qf;//quality factor
	uint8_t distances_amount;//how many distances are being sent
	//From here data should be arrays, if more than one anchor is used
	uint16_t anchor_uwb_address[MAX_ANCHORS];
	uint32_t distance_to_anchor[MAX_ANCHORS];
	float distance_to_anchor_cm[MAX_ANCHORS];
	uint8_t distance_to_anchor_qf[MAX_ANCHORS];//quality factor
	uint32_t anchor_pos_x[MAX_ANCHORS];
	uint32_t anchor_pos_y[MAX_ANCHORS];
	uint32_t anchor_pos_z[MAX_ANCHORS];
	uint32_t anchor_pos_qf[MAX_ANCHORS];//quality factor
	uint32_t last_distance_to_anchor[MAX_ANCHORS];
	uint8_t uwb_ok[MAX_ANCHORS];
	uint16_t packet_sum[MAX_ANCHORS];
	uint16_t packet_loss[MAX_ANCHORS];
};
struct loc dwm_loc_get1;
struct loc dwm_loc_get2;
//Ultrasonic variables
uint8_t IC_flag = 0;
uint8_t IC_index = 0;
uint32_t IC_time1 = 0;
uint32_t IC_time2 = 0;
const float speed_of_sound = 0.0343/2;
float ultrasonic_distance_cm = 0;
uint32_t usonic_distance_cm_uint32 = 0;
struct f_float
{
	float value;
	float moving_sum;
	uint8_t window_index;
	float buf[MAX_FILTER_SIZE];
	uint8_t buf_size;
	uint8_t filter_size;
};
struct f_float f_uwb1_cm;
struct f_float f_uwb2_cm;
struct f_float f_usonic_cm;
struct f_float f_trig_angle_deg;
struct f_float f_trig_distance_cm;
float last_dist_error = 0;//naudojama detektuoti, kada skrenda ne i ta puse
uint8_t wrong_side = 0;
uint8_t good_side = 0;
uint8_t u_turn = 0;
uint8_t u_turn_duration = 0;
//Bilateration variables
float bilat_x_est_cm = 0;
float bilat_y_est_cm = 0;
float bilat_rho_est_cm = 0;
float bilat_theta_est_deg = 0;
//Trigonometrinio variables
float trig_rho_est_cm = 0;
float trig_theta_est_deg = 0;
//struct pid
//{
//	float error;
//	float last_error;
//	float integral;
//	float derivative;
//	float pidsum;
//	float Kp;
//	float Ki;
//	float Kd;
//	float psum;
//	float isum;
//	float dsum;
//	float deadzone;
//};
//struct pid pid_altitude;
//struct pid pid_distance;
//struct pid pid_angle;
struct pid
{
	float plusminus;
	float Kp;
	float Ki;
	float Kd;
	float tau;//D low pass filtro laiko konstanta
	float lim_min;
	float lim_max;
	float lim_min_int;
	float lim_max_int;
	float T;
	float psum;
	float isum;
	float dsum;
	float error;
	float last_error;
	float last_meas;
	float pidsum;
};
struct pid pid_altitude;
struct pid pid_distance;
struct pid pid_angle;
float test_altitude = 101;//cm
float test_distance = 3049;//mm
float test_angle = 91;//degrees
float setpoint_altitude = SETPOINT_ALTITUDE;//cm
float setpoint_distance = SETPOINT_DISTANCE;//cm
float setpoint_angle = SETPOINT_ANGLE;//degrees
uint8_t object_tracking = 0;
float tracking_thr = 1000;
float control_thr = 0;//1000:2000
float control_pitch = 0;//1000:2000
float control_yaw = 0;//1000:2000
uint8_t target_locked = 0;// =1 when angle is within deadzone
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_UART5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_UART7_Init(void);
static void MX_UART8_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void usDelay(uint32_t uSec);
void measure_ultrasonic();
void read_sbus();
void write_sbus();
void sbus_passthrough();
void init_variables();
void moving_average_update_float(float input, struct f_float *s);
void update_trig(float B, float A);
void init_log_microsd();
void update_log_microsd();
void write_periodic_LCD();
float constrain_f(float input, float min, float max);
//void calculate_PID(float measured, float setpoint, struct pid *s);
void parse_uwb_uart(struct loc *loc, uint8_t bfr[]);
void init_PID(struct pid *s);
void calculate_PID(float measurement, float setpoint, struct pid *s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void detect_wrong_side()
{
	if(abs(pid_distance.error)>last_dist_error)
	{
	  wrong_side++;
	}
	else
	{
	  good_side++;
	}
	if(good_side > 5)
	{
	  wrong_side=0;
	  good_side=0;
	}
	if(wrong_side > 20)
	{
		//make a U turn
		u_turn = 1;
	}
	last_dist_error=abs(pid_distance.error);
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
  MX_DMA_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_UART5_Init();
  MX_USART6_UART_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);//periodas 1s
	HAL_TIM_Base_Start_IT(&htim4);//periodas 9ms
	HAL_TIM_Base_Start_IT(&htim5);//periodas 100ms
	sbus_receive_length = SBUS_RX_LENGTH;
//	HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxUART4, DISPLAY_RX_LENGTH);
	init_variables();
	init_log_microsd();
	HAL_Delay(1000);
	//	HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxUART4, DISPLAY_RX_LENGTH);
	HAL_UART_Receive_DMA(&huart6, (uint8_t *)rxUART6, sbus_receive_length);//Receiver
	HAL_UART_Receive_DMA(&huart7, (uint8_t *)rxUART7, UWB_UART_LENGTH);//UWB1
	HAL_UART_Receive_DMA(&huart8, (uint8_t *)rxUART8, UWB_UART_LENGTH);//UWB2
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(uwb1_uart_received == 1 && uwb2_uart_received == 1)
	  {
		  uwb1_uart_received = 0;
		  uwb2_uart_received = 0;
		  HAL_GPIO_TogglePin(GPIOA, LD3_Pin);
		  parse_uwb_uart(&dwm_loc_get1,rxUART7);
		  parse_uwb_uart(&dwm_loc_get2,rxUART8);
		  //test = tmr_flag_100ms;
		  tmr_flag_100ms = 0;
		  measure_ultrasonic();
		  moving_average_update_float(ultrasonic_distance_cm, &f_usonic_cm);
		  if(dwm_loc_get1.packet_loss[0]<=UWB_MAX_PACKET_LOSS && dwm_loc_get2.packet_loss[0]<=UWB_MAX_PACKET_LOSS)
		  {
			  moving_average_update_float(dwm_loc_get1.distance_to_anchor_cm[0], &f_uwb1_cm);
			  moving_average_update_float(dwm_loc_get2.distance_to_anchor_cm[0], &f_uwb2_cm);
			  if (f_uwb1_cm.buf_size >= UWB_FILTER_SIZE && dwm_loc_get1.distance_to_anchor_cm[0] > 0 && dwm_loc_get2.distance_to_anchor_cm[0] > 0)
			  {
				  update_trig(f_uwb1_cm.value, f_uwb2_cm.value);
				  moving_average_update_float(trig_theta_est_deg, &f_trig_angle_deg);
				  moving_average_update_float(trig_rho_est_cm, &f_trig_distance_cm);
				  if (f_trig_angle_deg.buf_size >= TRIG_ANGLE_FILTER_SIZE)
				  {
					  if(rc_in[6] > 1900 && rc_no_pulses < MAX_DROPPED_SBUS_PACKETS && rc_bad_frame == 0 && rc_failsafe == 0)
					  {
						  if(object_tracking==0 && begin_tracking==1)//sekimo ijungimo momentas. Reikia, kad nepagautu kada failsafe ir paskui vel pasileidzia
						  {
							  tracking_thr = rc_in[0];
							  init_PID(&pid_distance);
							  init_PID(&pid_angle);
							  init_PID(&pid_altitude);
						  }
						  object_tracking = 1;
						  calculate_PID(f_trig_distance_cm.value, setpoint_distance, &pid_distance);
						  calculate_PID(f_trig_angle_deg.value, setpoint_angle, &pid_angle);
						  calculate_PID(f_usonic_cm.value, setpoint_altitude, &pid_altitude);
						  if(abs(pid_angle.error) < DEADZONE_ANGLE)
						  {
							  target_locked = 1;
							  //detect_wrong_side();
						  }
						  else
						  {
							  target_locked = 0;
						  }
//						  if(u_turn==1 && u_turn_duration < U_TURN_DURATION)
//						  {
//							  control_yaw = RC_MAX;//make a U turn
//							  u_turn_duration++;
//						  }
//						  else
//						  {
//							  u_turn=0;
//							  u_turn_duration=0;
//							  control_yaw = 1500 + pid_angle.pidsum;
//						  }
						  control_yaw = 1500 + pid_angle.plusminus*pid_angle.pidsum;
						  control_pitch = 1500 + pid_distance.plusminus*pid_distance.pidsum;
						  control_thr = tracking_thr + pid_altitude.plusminus*pid_altitude.pidsum;
						  update_log_microsd();
						  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
					  }
					  else
					  {
						  object_tracking = 0;
						  begin_tracking=0;
						  target_locked = 0;
					  }
					  write_periodic_LCD();
					  HAL_UART_Transmit(&huart4, (uint8_t *)txUART4, DISPLAY_TX_LENGTH, 10);
					  delay = time_10hz - last_time_10hz;
					  last_time_10hz = time_10hz;
				  }
			  }
		  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART6
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_UART8;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart8ClockSelection = RCC_UART8CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 54000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 54000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 18-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 54000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 200-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 216-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 216-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 0xFFFF-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  huart4.Init.BaudRate = 115200;
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
  huart5.Init.BaudRate = 100000;
  huart5.Init.WordLength = UART_WORDLENGTH_9B;
  huart5.Init.StopBits = UART_STOPBITS_2;
  huart5.Init.Parity = UART_PARITY_EVEN;
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
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 100000;
  huart6.Init.WordLength = UART_WORDLENGTH_9B;
  huart6.Init.StopBits = UART_STOPBITS_2;
  huart6.Init.Parity = UART_PARITY_EVEN;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  huart6.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
	if (htim->Instance == TIM3)
	{
		tmr_flag_1s++;
	}
	else if (htim->Instance == TIM4)
	{
		if (rc_packets > 0)
		{
			rc_packets = 0;
			rc_no_pulses = 0;
			if(rc_bad_frame==0 && rc_failsafe==0)
			{
				sbus_passthrough();
				write_sbus();
				HAL_UART_Transmit(&huart5, (uint8_t *)txUART5, SBUS_TX_LENGTH, 10);
			}
		}
		else
		{
			rc_no_pulses++;
		}
	}
	else if (htim->Instance == TIM5)
	{
		tmr_flag_100ms++;
		time_10hz++;
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(IC_index == 0) //First edge
	{
		IC_time1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		IC_index = 1;
	}
	else if(IC_index == 1) //Second edge
	{
		IC_time2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		IC_index = 0;
		IC_flag = 1;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
  if (huart->Instance == USART6)
  {
	  rc_packets++;
	  read_sbus();
	  HAL_UART_Receive_DMA(&huart6, (uint8_t *)rxUART6, sbus_receive_length);
  }
  else if (huart->Instance == UART7)
  {
	  uwb1_uart_received = 1;
	  HAL_UART_Receive_DMA(&huart7, (uint8_t *)rxUART7, UWB_UART_LENGTH);
  }
  else if (huart->Instance == UART8)
  {
	  uwb2_uart_received = 1;
	  HAL_UART_Receive_DMA(&huart8, (uint8_t *)rxUART8, UWB_UART_LENGTH);
  }

//  else if (huart->Instance == UART4)
//  {
//	  HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxUART4, DISPLAY_RX_LENGTH);
//  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  uart_error++;
  if (huart->Instance == USART6)
  {
	  //HAL_UART_DMAStop()
	  HAL_UART_AbortReceive(&huart6);
	  HAL_UART_Receive_DMA(&huart6, (uint8_t *)rxUART6, sbus_receive_length);
  }
  else if (huart->Instance == UART7)
	{
		HAL_UART_AbortReceive(&huart7);
		HAL_UART_Receive_DMA(&huart7, (uint8_t *)rxUART7, UWB_UART_LENGTH);
	}
  else if (huart->Instance == UART8)
  {
	  HAL_UART_AbortReceive(&huart8);
	  HAL_UART_Receive_DMA(&huart8, (uint8_t *)rxUART8, UWB_UART_LENGTH);
  }
//  else if (huart->Instance == UART4)
//  {
//	  HAL_UART_AbortReceive(&huart4);
//	  HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxUART4, DISPLAY_RX_LENGTH);
//  }
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_ErrorCallback can be implemented in the user file.
   */
}
void usDelay(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1;
	usTIM->EGR = 1;
	usTIM->SR &= ~1;
	usTIM->CR1 |= 1;
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}
void measure_ultrasonic()
{
	//Reset TRIG
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	usDelay(3);
	//10us impulse on TRIG
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1);
	uint32_t startTick = HAL_GetTick();
	uint32_t tick_difference = HAL_GetTick() - startTick;
	while (tick_difference < ULTRASONIC_TIMEOUT_MS)
	{
		if (IC_flag == 1)
			break;
		else
			tick_difference = HAL_GetTick() - startTick;
	}
	HAL_TIM_IC_Stop_IT(&htim9, TIM_CHANNEL_1);
	if(IC_flag == 1 && IC_time2 > IC_time1 && tick_difference < ULTRASONIC_TIMEOUT_MS)
	{
		ultrasonic_distance_cm = (IC_time2 - IC_time1) * speed_of_sound;
	}
	if (ultrasonic_distance_cm > ULTRASONIC_MAX_DISTANCE_CM)
		ultrasonic_distance_cm = ULTRASONIC_MAX_DISTANCE_CM;
	else if (ultrasonic_distance_cm < ULTRASONIC_MIN_DISTANCE_CM)
		ultrasonic_distance_cm = ULTRASONIC_MIN_DISTANCE_CM;
	IC_flag = 0;
}
void read_sbus()
{
	uint8_t i = 0;
	if(rxUART6[0] == SBUS_START && rxUART6[24] == SBUS_END)
	{
		//failsafe
		rc_frame_lost = (rxUART6[i+23] & 0x04) >> 2;
		rc_failsafe = (rxUART6[i+23] & 0x08) >> 3;
		//16 channels, 11bit per channel
		rc_in_raw[0] = (rxUART6[i+1] | rxUART6[i+2] << 8) & 0x07ff;
		rc_in_raw[1] = (rxUART6[i+2] >> 3 | rxUART6[i+3] << 5) & 0x07ff;
		rc_in_raw[2] = (rxUART6[i+3] >> 6 | rxUART6[i+4] << 2 | rxUART6[i+5] << 10) & 0x07ff;
		rc_in_raw[3] = (rxUART6[i+5] >> 1 | rxUART6[i+6] << 7) & 0x07ff;
		rc_in_raw[4] = (rxUART6[i+6] >> 4 | rxUART6[i+7] << 4) & 0x07ff;
		rc_in_raw[5] = (rxUART6[i+7] >> 7 | rxUART6[i+8] << 1 | rxUART6[i+9] << 9) & 0x07ff;
		rc_in_raw[6] = (rxUART6[i+9] >> 2 | rxUART6[i+10] << 6) & 0x07ff;
		rc_in_raw[7] = (rxUART6[i+10] >> 5 | rxUART6[i+11] << 3) & 0x07ff;
		rc_in_raw[8] = (rxUART6[i+12] | rxUART6[i+13] << 8) & 0x07ff;
		rc_in_raw[9] = (rxUART6[i+13] >> 3 | rxUART6[i+14] << 5) & 0x07ff;
		rc_in_raw[10] = (rxUART6[i+14] >> 6 | rxUART6[i+15] << 2 | rxUART6[i+16] << 10) & 0x07ff;
		rc_in_raw[11] = (rxUART6[i+16] >> 1 | rxUART6[i+17] << 7) & 0x07ff;
		rc_in_raw[12] = (rxUART6[i+17] >> 4 | rxUART6[i+18] << 4) & 0x07ff;
		rc_in_raw[13] = (rxUART6[i+18] >> 7 | rxUART6[i+19] << 1 | rxUART6[i+20] << 9) & 0x07ff;
		rc_in_raw[14] = (rxUART6[i+20] >> 2 | rxUART6[i+21] << 6) & 0x07ff;
		rc_in_raw[15] = (rxUART6[i+21] >> 5 | rxUART6[i+22] << 3) & 0x07ff;
		for(uint8_t i = 0; i < SBUS_CHANNELS; i++)
		{
			rc_in[i] = (5 * rc_in_raw[i] / 8) + 880;//int to float
		}
		rc_bad_frame = 0;
		if(last_state_rcin6 <= 1900 && rc_in[6] > 1900)
			begin_tracking = 1;
		last_state_rcin6 = rc_in[6];
	}
	else
	{
		rc_bad_frame = 1;
		HAL_UART_AbortReceive(&huart6);
		HAL_UART_Receive_DMA(&huart6, (uint8_t *)rxUART6, sbus_receive_length);
	}
}
void write_sbus()
{
	//float to int
	for(uint8_t i = 0; i < SBUS_CHANNELS; i++)
	{
		//rc_in[i] = (5 * rc_in_raw[i] / 8) + 880;
		rc_out_raw[i] = (rc_out[i] - 880) * 8 / 5;
		if (rc_out_raw[i] < SBUS_CHANNEL_MIN)
			rc_out_raw[i] = SBUS_CHANNEL_MIN;
		if (rc_out_raw[i] > SBUS_CHANNEL_MAX)
			rc_out_raw[i] = SBUS_CHANNEL_MAX;
	}

	txUART5[0] = SBUS_START;
	//16 channels, 11bit per channel
	txUART5[1] = ((rc_out_raw[0] & 0x07ff));
	txUART5[2] = ((rc_out_raw[0] & 0x07ff) >> 8 | (rc_out_raw[1] & 0x07ff) << 3);
	txUART5[3] = ((rc_out_raw[1] & 0x07ff) >> 5 | (rc_out_raw[2] & 0x07ff) << 6);
	txUART5[4] = ((rc_out_raw[2] & 0x07ff) >> 2);
	txUART5[5] = ((rc_out_raw[2] & 0x07ff) >> 10 | (rc_out_raw[3] & 0x07ff) << 1);
	txUART5[6] = ((rc_out_raw[3] & 0x07ff) >> 7 | (rc_out_raw[4] & 0x07ff) << 4);
	txUART5[7] = ((rc_out_raw[4] & 0x07ff) >> 4 | (rc_out_raw[5] & 0x07ff) << 7);
	txUART5[8] = ((rc_out_raw[5] & 0x07ff) >> 1);
	txUART5[9] = ((rc_out_raw[5] & 0x07ff) >> 9 | (rc_out_raw[6] & 0x07ff) << 2);
	txUART5[10] = ((rc_out_raw[6] & 0x07ff) >> 6 | (rc_out_raw[7] & 0x07ff) << 5);
	txUART5[11] = ((rc_out_raw[7] & 0x07ff) >> 3);
	txUART5[12] = ((rc_out_raw[8] & 0x07ff));
	txUART5[13] = ((rc_out_raw[8] & 0x07ff) >> 8 | (rc_out_raw[9] & 0x07ff) << 3);
	txUART5[14] = ((rc_out_raw[9] & 0x07ff) >> 5 | (rc_out_raw[10] & 0x07ff) << 6);
	txUART5[15] = ((rc_out_raw[10] & 0x07ff) >> 2);
	txUART5[16] = ((rc_out_raw[10] & 0x07ff) >> 10 | (rc_out_raw[11] & 0x07ff) << 1);
	txUART5[17] = ((rc_out_raw[11] & 0x07ff) >> 7 | (rc_out_raw[12] & 0x07ff) << 4);
	txUART5[18] = ((rc_out_raw[12] & 0x07ff) >> 4 | (rc_out_raw[13] & 0x07ff) << 7);
	txUART5[19] = ((rc_out_raw[13] & 0x07ff) >> 1);
	txUART5[20] = ((rc_out_raw[13] & 0x07ff) >> 9 | (rc_out_raw[14] & 0x07ff) << 2);
	txUART5[21] = ((rc_out_raw[14] & 0x07ff) >> 6 | (rc_out_raw[15] & 0x07ff) << 5);
	txUART5[22] = ((rc_out_raw[15] & 0x07ff) >> 3);
	txUART5[23] = ((rc_failsafe & 0x01) << 3) | ((rc_frame_lost & 0x01) << 2);
	txUART5[24] = SBUS_END;
}
void sbus_passthrough()
{
	if (object_tracking == 1)
	{
//		rc_out[0] = rc_in[0];//thr
		rc_out[0] = control_thr;//thr
		rc_out[1] = rc_in[1];//roll
//		rc_out[2] = rc_in[2];//pitch
		if(target_locked==1)
			rc_out[2] = control_pitch;
		else
			rc_out[2] = rc_in[2];//pitch
//		rc_out[3] = rc_in[3];//yaw
		rc_out[3] = control_yaw;//yaw
		rc_out[4] = rc_in[4];
		rc_out[5] = rc_in[5];
		rc_out[6] = rc_in[6];
		rc_out[7] = rc_in[7];
		rc_out[8] = rc_in[8];
		rc_out[9] = rc_in[9];
		rc_out[10] = rc_in[10];
		rc_out[11] = rc_in[11];
		rc_out[12] = rc_in[12];
		rc_out[13] = rc_in[13];
		rc_out[14] = rc_in[14];
		rc_out[15] = rc_in[15];
	}
	else
	{
		for (uint8_t i = 0; i < SBUS_CHANNELS; i++)
		{
			rc_out[i] = rc_in[i];
		}
	}
	for(uint8_t i = 0; i < SBUS_CHANNELS; i++)
	{
		if (rc_out_raw[i] < RC_MIN)
			rc_out_raw[i] = RC_MIN;
		if (rc_out_raw[i] > RC_MAX)
			rc_out_raw[i] = RC_MAX;
	}
}
void init_variables()
{
	pid_altitude.plusminus=1;
	pid_altitude.Kp = P_altitude;
	pid_altitude.Ki = I_altitude;
	pid_altitude.Kd = D_altitude;
	pid_altitude.T = 0.1;
	pid_altitude.tau = D_FILTER_TIME_CONSTANT;
	pid_altitude.lim_min = PID_MIN_ALT;
	pid_altitude.lim_max = PID_MAX_ALT;
	pid_altitude.lim_min_int = PID_MIN_ALT_I;
	pid_altitude.lim_max_int = PID_MAX_ALT_I;

	pid_angle.plusminus=-1;
	pid_angle.Kp = P_angle;
	pid_angle.Ki = I_angle;
	pid_angle.Kd = D_angle;
	pid_angle.T = 0.1;
	pid_angle.tau = D_FILTER_TIME_CONSTANT;
	pid_angle.lim_min = PID_MIN;
	pid_angle.lim_max = PID_MAX;
	pid_angle.lim_min_int = PID_MIN;
	pid_angle.lim_max_int = PID_MAX;

	pid_distance.plusminus=-1;
	pid_distance.Kp = P_distance;
	pid_distance.Ki = I_distance;
	pid_distance.Kd = D_distance;
	pid_distance.T = 0.1;
	pid_distance.lim_min = PID_MIN;
	pid_distance.lim_max = PID_MAX;
	pid_distance.lim_min_int = PID_MIN;
	pid_distance.lim_max_int = PID_MAX;

	f_uwb1_cm.filter_size = UWB_FILTER_SIZE;
//	f_uwb1_cm.moving_sum = 0.0f;

	f_uwb2_cm.filter_size = UWB_FILTER_SIZE;
//	f_uwb2_cm.moving_sum = 0.0f;

	f_usonic_cm.filter_size = USONIC_FILTER_SIZE;
//	f_usonic_cm.moving_sum = 0.0f;

	f_trig_angle_deg.filter_size = TRIG_ANGLE_FILTER_SIZE;
//	f_trig_angle_deg.filter_size = 3;
//	f_trig_angle_deg.moving_sum = 0.0f;
//	f_trig_angle_deg.buf_size = 0;
//	f_trig_angle_deg.window_index = 0;

	f_trig_distance_cm.filter_size = TRIG_DISTANCE_FILTER_SIZE;
//	f_trig_distance_cm.moving_sum = 0.0f;
//	f_trig_distance_cm.buf_size = 0;
//	f_trig_distance_cm.window_index = 0;
//	for(uint8_t i=0;i<MAX_FILTER_SIZE;i++)
//	{
//		f_trig_angle_deg.buf[i]=0;
//		f_trig_distance_cm.buf[i]=0;
//	}

	//	float value;
	//	float moving_sum;
	//	uint8_t window_index;
	//	float buf[MAX_FILTER_SIZE];
	//	uint8_t buf_size;
	//	uint8_t filter_size;
}
void moving_average_update_float(float input, struct f_float *s)
{
	s->moving_sum = s->moving_sum - s->buf[s->window_index] + input;
	s->buf[s->window_index] = input;
	s->window_index++;
	if (s->window_index > s->filter_size - 1)
	{
		s->window_index = 0;
		s->buf_size = s->filter_size;
	}
	if (s->buf_size == s->filter_size)
	{
		s->value = s->moving_sum / s->filter_size;
	}
}
void update_trig(float B, float A)
{
	//             degC
	//            * *  *
	//          *    *  *
	//        *      *   *
	//     rx1(B)     *    rx2(A)
	//     *          *      *
	//   *             *       *
	//  *              *degA1    *
	//degA*****rx12(C)*****rx12/2**degB
	if (B > A + UWB_RX_TO_RX_DISTANCE_CM)
	{
		trig_rho_est_cm = (A+B)/2;
		trig_theta_est_deg = -90;
	}
	else if (A > B + UWB_RX_TO_RX_DISTANCE_CM)
	{
		trig_rho_est_cm = (A+B)/2;
		trig_theta_est_deg = 90;
	}
	else
	{
		float C = UWB_RX_TO_RX_DISTANCE_CM;
		float C_div_2 = UWB_RX_TO_RX_DISTANCE_CM/2;
		float C_div_2_pow_2 = pow(C_div_2,2);
		float A_pow_2 = pow(A,2);
		float radA1 = acos((A_pow_2+pow(C,2)-pow(B,2))/(2*A*C));
		trig_rho_est_cm = sqrt(A_pow_2+C_div_2_pow_2-2*A*C_div_2*cos(radA1));
		trig_theta_est_deg = conv_to_deg*acos((pow(trig_rho_est_cm,2)+C_div_2_pow_2-A_pow_2)/(2*trig_rho_est_cm*C_div_2))-90;
	}
}
void init_log_microsd()
{
	//memset(microsd_buf,0,MICROSD_BUFFER_SIZE);
	//sdcard buffer up to 128 bytes. After 120 bytes wait at least 44ms
	microsd_buf[0]=0x0d;
	microsd_buf[1]=0x0a;
	HAL_UART_Transmit(&huart3, (uint8_t *)microsd_buf, 2, 10);
	//HAL_Delay(100);//wait for card
	snprintf(microsd_buf, MICROSD_BUFFER_SIZE, "%s\r\n", "New log begin");
	HAL_UART_Transmit(&huart3, (uint8_t *)microsd_buf, 14, 10);
	HAL_Delay(100);//wait for card
	snprintf(microsd_buf, MICROSD_BUFFER_SIZE, "%s\r\n", "P_angle;I_angle;D_angle;"
			"P_distance;I_distance;D_distance;P_altitude;I_altitude;D_altitude");
	HAL_UART_Transmit(&huart3, (uint8_t *)microsd_buf, 91, 10);
	HAL_Delay(100);//wait for card
	snprintf(microsd_buf, MICROSD_BUFFER_SIZE, "%1.4f;%1.4f;%1.4f;%1.4f;%1.4f;%1.4f;%1.4f;%1.4f;%1.4f;\r\n",
			pid_angle.Kp,pid_angle.Ki,pid_angle.
			Kd,pid_distance.Kp,pid_distance.Ki,pid_distance.Kd,
			pid_altitude.Kp,pid_altitude.Ki,pid_altitude.Kd);
	HAL_UART_Transmit(&huart3, (uint8_t *)microsd_buf, 65, 10);
	HAL_Delay(100);//wait for card
//	snprintf(microsd_buf, MICROSD_BUFFER_SIZE, "%s\r\n", "uwb1;uwb2;usnic;"
//			"angle;dist;"
//			"angleErr;distErr;altErr;"
//			"anglePidsum;distPidsum;altPidsum;"
//			"altPsum,altIsum,altDsum"
//			"trck;tme;pktLss1;pktLss2");
//	snprintf(microsd_buf, MICROSD_BUFFER_SIZE, "%s\r\n", "uwb1;uwb2;usnic;"
//			"angle;dist;"
//			"angleErr;distErr;altErr;"
//			"anglePidsum;distPidsum;altPidsum;"
//			"anglePsum,angleIsum,angleDsum"
//			"trck;tme;pktLss1;pktLss2");
	snprintf(microsd_buf, MICROSD_BUFFER_SIZE, "%s\r\n", "uwb1;uwb2;usnic;"
				"angle;dist;"
				"angleErr;distErr;altErr;"
				"anglePidsum;distPidsum;altPidsum;"
				"distPsum,distIsum,distDsum"
				"trck;tme;pktLss1;pktLss2;trg_l");
	HAL_UART_Transmit(&huart3, (uint8_t *)microsd_buf, 110, 10);
}
void update_log_microsd()
{
	memset(microsd_buf,0,MICROSD_BUFFER_SIZE);
	snprintf(microsd_buf, MICROSD_BUFFER_SIZE, "%4.1f;%4.1f;%3.1f;"
					"%3.1f;%4.1f;"
					"%3.1f;%4.1f;%3.1f;"
					"%3.1f;%3.1f;%3.1f;"
					"%3.1f;%3.1f;%3.1f;"
					"%d;%d;%d;%d;%d\r\n",f_uwb1_cm.value,f_uwb2_cm.value,f_usonic_cm.value,
					f_trig_angle_deg.value,f_trig_distance_cm.value,
					pid_angle.error,pid_distance.error,pid_altitude.error,
					pid_angle.pidsum,pid_distance.pidsum,pid_altitude.pidsum,
					pid_distance.psum,pid_distance.isum,pid_distance.dsum,
					object_tracking,time_10hz,dwm_loc_get1.packet_loss[0],dwm_loc_get2.packet_loss[0],target_locked
					);
//	snprintf(microsd_buf, MICROSD_BUFFER_SIZE, "%4.2f;%4.2f;%3.2f;"
//				"%3.2f;%4.2f;"
//				"%3.2f;%4.2f;%3.2f;"
//				"%3.2f;%3.2f;%3.2f;"
//				"%d;%d;%d;%d\r\n",f_uwb1_cm.value,f_uwb2_cm.value,f_usonic_cm.value,
//				f_trig_angle_deg.value,f_trig_distance_cm.value,
//				pid_angle.error,pid_distance.error,pid_altitude.error,
//				pid_angle.pidsum,pid_distance.pidsum,pid_altitude.pidsum,
//				object_tracking,time_10hz,dwm_loc_get1.packet_loss[0],dwm_loc_get2.packet_loss[0]
//				);
//	snprintf(microsd_buf, MICROSD_BUFFER_SIZE,
//			"0:%4.2f;00:%4.2f;"
//			"1:%3.2f;2:%4.2f;"
//			"3:%3.2f;4:%4.2f;"
//			"5:%d;6:%d;"
//			"7:%6.2f;8:%d;"
//			"9:%3.2f;10:%3.2f;11:%3.2f;\r\n",
//			f_uwb1_cm.value,f_uwb2_cm.value,
//			f_trig_angle_deg.value,f_trig_distance_cm.value,
//			trig_theta_est_deg, trig_rho_est_cm,
//			f_trig_angle_deg.buf_size,f_trig_angle_deg.filter_size,
//			f_trig_angle_deg.moving_sum,f_trig_angle_deg.window_index,
//			f_trig_angle_deg.buf[0],f_trig_angle_deg.buf[1],f_trig_angle_deg.buf[2]
//			);
	for(uint16_t i=0;i<=MICROSD_BUFFER_SIZE-1;i++)
	{
		if(microsd_buf[i] == 0x0A)//ascii \n
		{
			microsd_buf_size = i+1;
			break;
		}
	}
	HAL_UART_Transmit(&huart3, (uint8_t *)microsd_buf, microsd_buf_size, 10);
}
void write_periodic_LCD()
{
	txUART4[0] = DISPLAY_START_PERIODIC;
	uint8_t angle_plus_90=roundf(f_trig_angle_deg.value+90);
//	uint8_t angle_plus_90=roundf(trig_theta_est_deg+90);
	txUART4[1] = (uint8_t)angle_plus_90;
	uint16_t distance = f_trig_distance_cm.value;//conversion from float
	txUART4[2] = distance >> 8;
	txUART4[3] = distance & 0x00FF;
	uint16_t altitude = f_usonic_cm.value;//conversion from float
	txUART4[4] = altitude >> 8;
	txUART4[5] = altitude & 0x00FF;
	uint16_t uwb1 = f_uwb1_cm.value;//conversion from float
	uint16_t uwb2 = f_uwb2_cm.value;//conversion from float
	txUART4[6] = uwb1 >> 8;
	txUART4[7] = uwb1 & 0x00FF;
	txUART4[8] = uwb2 >> 8;
	txUART4[9] = uwb2 & 0x00FF;
	txUART4[10] = 0xFF;//termination byte1
	txUART4[11] = 0xFF;//termination byte2
	txUART4[12] = 0xFF;//termination byte3
}
float constrain_f(float input, float min, float max)
{
	float output;
	if(input > max)
	{
		output = max;
	}
	else if (input < min)
	{
		output = min;
	}
	return output;
}
void init_PID(struct pid *s)
{
	s->isum = 0.0f;
	s->last_error  = 0.0f;
	s->dsum  = 0.0f;
	s->last_meas = 0.0f;
	s->pidsum = 0.0f;
}
void calculate_PID(float measurement, float setpoint, struct pid *s)
{
    //float error = setpoint - measurement;
	s->error = setpoint - measurement;
    s->psum = s->Kp * s->error;
    s->isum = s->isum + 0.5f * s->Ki * s->T * (s->error + s->last_error);
    if (s->isum > s->lim_max_int)
    {
        s->isum = s->lim_max_int;
    }
    else if (s->isum < s->lim_min_int)
    {
        s->isum = s->lim_min_int;
    }
    s->dsum = -(2.0f*s->Kd*(measurement-s->last_meas)+(2.0f*s->tau-s->T)*s->dsum)/(2.0f*s->tau+s->T);
    s->pidsum = s->psum + s->isum + s->dsum;
    if (s->pidsum > s->lim_max)
    {
        s->pidsum = s->lim_max;
    }
    else if (s->pidsum < s->lim_min)
    {
        s->pidsum = s->lim_min;
    }
    s->last_error = s->error;
    s->last_meas = measurement;
}
void parse_uwb_uart(struct loc *loc, uint8_t bfr[])
{
	if (bfr[0]==0x44 && bfr[10]==0x0D && bfr[11]==0x0A && bfr[9]==0x41)
	{
		uint8_t tmp[8];
		for(uint8_t i=0;i<=7;i++)
		{
		  tmp[i]=bfr[i+1];
		}
		sscanf(tmp,"%08lx",&loc->distance_to_anchor[0]);
			loc->distance_to_anchor_cm[0] = (float)loc->distance_to_anchor[0]/10;
			loc->uwb_ok[0] = 1;
			if(loc->packet_sum[0] < UWB_PACKET_BUFFER_SIZE)
				loc->packet_sum[0]++;

		loc->packet_loss[0] = 100-loc->packet_sum[0]*100/UWB_PACKET_BUFFER_SIZE;
	}
	else
	{
		loc->uwb_ok[0] = 0;
		if(loc->packet_sum[0] > 0)
			loc->packet_sum[0]--;

	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
