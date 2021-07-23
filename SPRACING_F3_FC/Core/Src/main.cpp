/* USER CODE BEGIN Header */
/**
 *
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <arming.h>
#include "param_config.h"
#include "mpu6050.h"
#include "pid.h"
#include "angle_calculation.h"
#include "RCrate.h"
#include "IC_ISR.h"
#include "alarms.h"
#include "filter.h"


#include "w25qxx.h"
#include "w25qxxConf.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

Pid Roll_PID (Roll_P, Roll_I, Roll_D, Max_roll);
Pid Pitch_PID (Pitch_P, Pitch_I, Pitch_D, Max_pitch);
Pid Yaw_PID (Yaw_P, Yaw_I, Yaw_D, Max_yaw);

RC_rate ROLL_SP(LEVEL_ROLL_P, MAX_ANGLE, RC_RATE);
RC_rate Pitch_SP(LEVEL_PITCH_P, MAX_ANGLE, RC_RATE);
RC_rate Yaw_SP(LEVEL_PITCH_P, MAX_ANGLE, RC_RATE);

filter filter_X(Gyro_cutoff_freq, 0.002);
filter filter_Y(Gyro_cutoff_freq, 0.002);
filter filter_Z(Gyro_cutoff_freq, 0.002);

filter filter_Dterm_X(Dterm_cutoff_freq, 0.002);
filter filter_Dterm_Y(Dterm_cutoff_freq, 0.002);
filter filter_Dterm_Z(Dterm_cutoff_freq, 0.002);

filter filter_RC_roll(RC_cutoff_freq, 0.002);
filter filter_RC_pitch(RC_cutoff_freq, 0.002);
filter filter_RC_yaw(RC_cutoff_freq, 0.002);


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

//SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//FLASH_EraseInitTypeDef Flash_config;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t cycles, S_cycle;
float Loop_time, Elapsed_time;

uint8_t flight_mode = 1, change_term = 0, change_term_signal = 0;
uint32_t Config_EEPROM [50];


float Roll_PID_out, Pitch_PID_out, Yaw_PID_out;
float Roll_setpoint, Pitch_setpoint, Yaw_setpoint;

float roll_P,  roll_I,  roll_D;
float pitch_P,  pitch_I,  pitch_D;
float yaw_P,  yaw_I;

float roll_P_part,  roll_I_part,  roll_D_part,  roll_gyro,  roll_error,   roll_gyro_f,  roll_Dgyro_f,  roll_Dterm, roll_PID_sum;
float pitch_P_part, pitch_I_part, pitch_D_part, pitch_gyro, pitch_error;
float yaw_P_part,   yaw_I_part,   yaw_D_part,   yaw_gyro,   yaw_error;

float roll_rc,  roll_RC_f,  pitch_rc;
float roll_angle_level, pitch_angle_level;
float Level_P, RC_rate_scale, max_level_angle;


float acc_x_uf, acc_x_pt1, acc_x_bq;  // to the test the filters
float acc_y_uf, acc_y_pt1, acc_y_bq;  // to the test the filters
int32_t acc_x_v[7] ={0, 0, 0, 0, 0, 0, 0};
float acc_x_median;
int32_t acc_y_v[7] ={0, 0, 0, 0, 0, 0, 0};
float acc_y_median;


uint16_t esc_1 = 1000, esc_2 = 1000, esc_3 = 1000, esc_4 = 1000;
uint16_t throttle;

uint16_t Batt_volt, Batt_arr[5], Batt_av;
uint8_t Batt_L_count = 0, esc_counter = 0, flash_read_counter = 0;
int8_t  Batt_av_count = -1;

uint16_t PID_change_counter;

uint8_t Txdata;
uint8_t Rxdata;

uint32_t    Address;


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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */


  RCC->AHBENR |=  ( RCC_AHBENR_ADC12EN );

  HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);


  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // PPM
  HAL_Delay(50);

  while (throttle_ch() < 990  || roll_ch() < 990  || pitch_ch() < 990  || yaw_ch() < 990 ||
	     throttle_ch() > 2010 || roll_ch() > 2010 || pitch_ch() > 2010 || yaw_ch() > 2010)  {
	error_signal(4);
    HAL_Delay(2);
  }


  PWM_ESC_tim_setup();
  HAL_Delay(50);
  HAL_TIM_OC_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim17, TIM_CHANNEL_1);
  HAL_Delay(50);

  if (throttle_ch() < 990  || roll_ch() < 990  || pitch_ch() < 990  || yaw_ch() < 990 ||  // in case of failsafe it should be modified otherwise motors will be off on air
      throttle_ch() > 2010 || roll_ch() > 2010 || pitch_ch() > 2010 || yaw_ch() > 2010) {
  	esc_1 = 1000;
  	esc_2 = 1000;
  	esc_3 = 1000;
  	esc_4 = 1000;
  }


  while (HAL_I2C_IsDeviceReady(&hi2c1, 208, 1 , 10) != HAL_OK) {
	error_signal(2);
	HAL_Delay(2);
  }

  HAL_Delay(2000);


//  MPU6050_confirm();
  MPU6050_Init();
  HAL_Delay(50);
  MPU6050_calib_Gyro();
//  MPU6050_calib_Accel();


  // ----------- save calibration data to EEPROM ------

//  W25qxx_Init();
//  Address = 0x000000;
//  W25qxx_EraseSector(Address);


	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END 2 */
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	S_cycle = DWT->CYCCNT;

// --------------------------------------------------- calibration --------------------------------------------------

//	  if (throttle_ch() > 1980 && yaw_ch() < 1030 && roll_ch() < 1030 && pitch_ch() > 1980){
//
//		    MPU6050_calib_Gyro();
//		    MPU6050_calib_Accel();
//	  }


// --------------------------------------------------- READ BATTERY --------------------------------------------------

//	  Txdata = (uint8_t)(get_roll_angle() + 55 );

	Batt_L_count++;
	if (Batt_L_count == 125){

	HAL_ADC_Start(&hadc2);
	if (HAL_ADC_PollForConversion(&hadc2, 5) == HAL_OK){

	Batt_volt = HAL_ADC_GetValue(&hadc2) / 1.087;
	}
	HAL_ADC_Stop(&hadc2);

	Batt_av_count ++;
	Batt_arr[Batt_av_count] = Batt_volt;
	Batt_av = (Batt_arr[0]+Batt_arr[1]+Batt_arr[2]+Batt_arr[3]+Batt_arr[4])/5;

//      W25qxx_WriteByte(Txdata, Address);
//	    W25qxx_ReadByte(&Rxdata, Address);
//      Address+=8;

	if(Batt_av_count > 3)Batt_av_count = -1;

	Batt_L_count =0;
	  }

// ------------------------------------------- flash  ----------------------------

//for the flash data call the structure w25qxx.feature. the delays in the inti function has to be keept.


// --------------------------------------------------- SET PARAMETERS --------------------------------------------------

	       if (F_mode_ch() >= 1750)                      flight_mode = 1;
	  else if (F_mode_ch() < 1750 && F_mode_ch() >= 1350)flight_mode = 2;
	  else if (F_mode_ch() < 1350)                       flight_mode = 3;


	  PID_change_counter++;
	  if (PID_change_counter == 500){
		  PID_change_counter = 0;

	       if (PID_select_ch() > 1250 && PID_select_ch() <= 1500)change_term = 1;
	  else if (PID_select_ch() > 1500 && PID_select_ch() <= 1830)change_term = 2;
	  else if (PID_select_ch() > 1830)change_term = 3;
	  else change_term = 0;
	   change_term_signal = 0;
		  

	if (PID_change_ch() > 1700){  // increase PID value
	    if (change_term != 0)change_term_signal = 1;

		if (change_term == 1){  // only P value up
			if (flight_mode == 1){  // gyro mode
				Roll_PID.setP(Roll_PID.getP()   + 0.01);    if (Roll_PID.getP() > 5.0)Roll_PID.setP(5.0);
				Pitch_PID.setP(Pitch_PID.getP() + 0.01);    if (Pitch_PID.getP() > 3.0)Pitch_PID.setP(3.0);
				// Yaw_PID.setP(Yaw_PID.getP()     + 0.01);    if (Yaw_PID.getP() > 3.0)Yaw_PID.setP(3.0);
								 }

			if (flight_mode == 2){  // level mode
				ROLL_SP.set_Level_P(ROLL_SP.get_Level_P()   + 0.01);    if (ROLL_SP.get_Level_P() > 2.0)ROLL_SP.set_Level_P(2.0);
				Pitch_SP.set_Level_P(Pitch_SP.get_Level_P() + 0.01);    if (Pitch_SP.get_Level_P() > 2.6)Pitch_SP.set_Level_P(2.6);
		    	     	 	 	 						 }
		                           }
		else if (change_term == 2){ // only I value up
			if (flight_mode == 1){  // gyro mode
				Roll_PID.setI(Roll_PID.getI()   + 0.1); if (Roll_PID.getI()  > 10.0)Roll_PID.setI(10.0);
				Pitch_PID.setI(Pitch_PID.getI() + 0.1); if (Pitch_PID.getI() > 10.0)Pitch_PID.setI(10.0);
				// Yaw_PID.setI(Yaw_PID.getI()     + 0.1); if (Yaw_PID.getI()   > 10.0)Yaw_PID.setI(10.0);
			 	 	 	 	 	 						 }

		if (flight_mode == 2){  // level mode
				ROLL_SP.set_MaxAngle(ROLL_SP.get_MaxAngle()   + 1.0f);    if (ROLL_SP.get_MaxAngle() > 30.0)ROLL_SP.set_MaxAngle(30.0);
				Pitch_SP.set_MaxAngle(Pitch_SP.get_MaxAngle() + 1.0f);    if (Pitch_SP.get_MaxAngle() > 30.0)Pitch_SP.set_MaxAngle(30.0);
			 	 						 	 	 	 	 }
		                           }
		else if (change_term == 3){ // only D value up
			if (flight_mode == 1){  // gyro mode
				Roll_PID.setD(Roll_PID.getD()   + 0.1);    if (Roll_PID.getD() > 10.0)Roll_PID.setD(10.0);
				Pitch_PID.setD(Pitch_PID.getD() + 0.1);    if (Pitch_PID.getD() > 10.0)Pitch_PID.setD(10.0);
			 	 	 	 	 	 						 }

		if (flight_mode == 2){  // level mode
				ROLL_SP.set_RC_rate(ROLL_SP.get_RC_rate()   + 0.1f);    if (ROLL_SP.get_RC_rate()  > 2.0)ROLL_SP.set_RC_rate(2.0);
				Pitch_SP.set_RC_rate(Pitch_SP.get_RC_rate() + 0.1f);    if (Pitch_SP.get_RC_rate() > 2.0)Pitch_SP.set_RC_rate(2.0);
			 	 	 	 	 	 						 }
		                           }
	                             }

	else if (PID_change_ch() < 1300){ // increase PID value
		if (change_term != 0)change_term_signal = 2;

		if (change_term == 1){  // only P value down
			if (flight_mode == 1){  // gyro mode
				Roll_PID.setP(Roll_PID.getP()   - 0.01);    if (Roll_PID.getP() < 0.2)Roll_PID.setP(0.1);
				Pitch_PID.setP(Pitch_PID.getP() - 0.01);    if (Pitch_PID.getP() < 0.2)Pitch_PID.setP(0.1);
				// Yaw_PID.setP(Yaw_PID.getP()     - 0.01);    if (Yaw_PID.getP() < 0.2)Yaw_PID.setP(0.1);
	        	  	  	  	  	    					 }

			if (flight_mode == 2){  // level mode
				ROLL_SP.set_Level_P(ROLL_SP.get_Level_P()   - 0.01);    if (ROLL_SP.get_Level_P()  < 0.2)ROLL_SP.set_Level_P(0.1);
				Pitch_SP.set_Level_P(Pitch_SP.get_Level_P() - 0.01);    if (Pitch_SP.get_Level_P() < 0.2)Pitch_SP.set_Level_P(0.1);
	        	  	  	  	  	    					 }
	                               }
		else if (change_term == 2){  // only I value down
			if (flight_mode == 1){  // gyro mode
				Roll_PID.setI(Roll_PID.getI()   - 0.1); if (Roll_PID.getI()  < 0.2)Roll_PID.setI(0.1);
				Pitch_PID.setI(Pitch_PID.getI() - 0.1); if (Pitch_PID.getI() < 0.2)Pitch_PID.setI(0.1);
				// Yaw_PID.setI(Yaw_PID.getI()     - 0.1); if (Yaw_PID.getI()   < 0.2)Yaw_PID.setI(0.1);
	    	 	 	 	 	 	 						 }

			if (flight_mode == 2){  // level mode
				ROLL_SP.set_MaxAngle(ROLL_SP.get_MaxAngle()   - 1.0f);    if (ROLL_SP.get_MaxAngle() < 6.0)ROLL_SP.set_MaxAngle(5.0);
				Pitch_SP.set_MaxAngle(Pitch_SP.get_MaxAngle() - 1.0f);    if (Pitch_SP.get_MaxAngle() < 6.0)Pitch_SP.set_MaxAngle(5.0);
	    	 	 	 	 	 	 						 }
	                               }
	     else if (change_term == 3){  // only D value down
	    	 	 	 	 	 	 	if (flight_mode == 1){  // gyro mode
				Roll_PID.setD(Roll_PID.getD()   - 0.1);    if (Roll_PID.getD() < 0.2)Roll_PID.setD(0.2);
				Pitch_PID.setD(Pitch_PID.getD() - 0.1);    if (Pitch_PID.getD() < 0.2)Pitch_PID.setD(0.2);
	    	 	 	 	 	 	 						 }

			if (flight_mode == 2){  // level mode
				ROLL_SP.set_RC_rate(ROLL_SP.get_RC_rate()   - 0.1f);    if (ROLL_SP.get_RC_rate()  < 0.5)ROLL_SP.set_RC_rate(0.4);
				Pitch_SP.set_RC_rate(Pitch_SP.get_RC_rate() - 0.1f);    if (Pitch_SP.get_RC_rate() < 0.5)Pitch_SP.set_RC_rate(0.4);
	 	 	 						               	    		 }
	                             }
	  }

	  }


	  if (change_term_signal == 1 ){
		  buzzer_signal(1, 100);
	  }
	  else if (change_term_signal == 2 ){
		  buzzer_signal(2, 100);
	  }


// --------------------------------------------------- FLIGHT CONTROLL --------------------------------------------------

	 	  MPU6050_Read_Gyro();

		  if (flight_mode == 2 || flight_mode == 3) {
		  MPU6050_Read_Accel();
		  calc_angles (Loop_time);
		   }


	 	 if (flight_mode == 1) {

			Roll_setpoint = ROLL_SP.RC_rate_manual(roll_ch())    * ROLL_SP.get_RC_rate();
			Pitch_setpoint = Pitch_SP.RC_rate_manual(pitch_ch()) * Pitch_SP.get_RC_rate();
			if (throttle_ch() > 1050 ){
			  Yaw_setpoint = Yaw_SP.RC_rate_manual(yaw_ch())       * Yaw_SP.get_RC_rate();
			} else Yaw_setpoint = 0;

			 flight_mode_signal(1);
	 	 }


	 	 if (flight_mode == 2) {

			roll_rc = ROLL_SP.RC_rate_manual(roll_ch());
			pitch_rc= Pitch_SP.RC_rate_manual(pitch_ch());

			roll_angle_level  =   get_roll_angle()  * ROLL_SP.get_MaxAngle(); // the higher the max angle value, the lower he final angle during flight
			pitch_angle_level = - get_pitch_angle() * Pitch_SP.get_MaxAngle();

			Roll_setpoint  = (roll_rc  - roll_angle_level)  * ROLL_SP.get_Level_P(); // levle P gain
			  Pitch_setpoint = (pitch_rc - pitch_angle_level) * Pitch_SP.get_Level_P();

			  if (throttle_ch() > 1050 ){
			  Yaw_setpoint = Yaw_SP.RC_rate_manual(yaw_ch());
			  } else Yaw_setpoint = 0;

			flight_mode_signal(2);

	 	 }


	 	 if (flight_mode == 3) {

		      roll_rc = ROLL_SP.RC_rate_manual(roll_ch());
		      pitch_rc= Pitch_SP.RC_rate_manual(pitch_ch());

		      roll_angle_level  =   get_roll_angle_half()   * ROLL_SP.get_MaxAngle();
		      pitch_angle_level = - get_pitch_angle_half()  * Pitch_SP.get_MaxAngle();

		 	  Roll_setpoint  = (roll_rc  - roll_angle_level)  * ROLL_SP.get_Level_P();
		 	  Pitch_setpoint = (pitch_rc - pitch_angle_level) * Pitch_SP.get_Level_P();

		 	  if (throttle_ch() > 1050 ){
		 	  Yaw_setpoint = Yaw_SP.RC_rate_manual(yaw_ch());
		 	  } else Yaw_setpoint = 0;

		     flight_mode_signal(3);
	 	 }


		Roll_PID_out  = Roll_PID.updatePID( filter_RC_roll.Biquad_filter(Roll_setpoint),   filter_X.Pt1_filter(gyro_X()), filter_Dterm_X.Biquad_filter(gyro_X()), Loop_time, flight_mode);
		Pitch_PID_out = Pitch_PID.updatePID(filter_RC_pitch.Biquad_filter(Pitch_setpoint), filter_Y.Pt1_filter(gyro_Y()), filter_Dterm_Y.Biquad_filter(gyro_Y()), Loop_time, flight_mode);
		Yaw_PID_out   = Yaw_PID.updatePID(  filter_RC_yaw.Biquad_filter(Yaw_setpoint),     filter_Z.Pt1_filter(-gyro_Z()),filter_Dterm_Z.Biquad_filter(-gyro_Z()),Loop_time, flight_mode);


// --------------------------------------------------- DEBUG --------------------------------------------------

		roll_P = Roll_PID.getP();
		roll_I = Roll_PID.getI();
		roll_D = Roll_PID.getD();

		pitch_P = Pitch_PID.getP();
		pitch_I = Pitch_PID.getI();
		pitch_D = Pitch_PID.getD();

		yaw_P = Yaw_PID.getP();
		yaw_I = Yaw_PID.getI();

		roll_gyro   =  gyro_X();
		roll_gyro_f =  filter_X.Pt1_filter(gyro_X());
		roll_Dgyro_f = filter_Dterm_X.Biquad_filter(gyro_X());
		roll_RC_f   =  filter_RC_roll.Biquad_filter(Roll_setpoint);

		roll_error  =  Roll_PID.getError();
		roll_P_part =  Roll_PID.getP_part();
		roll_I_part =  Roll_PID.getI_part();
		roll_D_part =  Roll_PID.getD_part();
		roll_Dterm  =  Roll_PID.getDterm();
		roll_PID_sum = Roll_PID.get_PIDsum();

		Level_P         = ROLL_SP.get_Level_P();
		max_level_angle = ROLL_SP.get_MaxAngle();
		RC_rate_scale   = ROLL_SP.get_RC_rate();


// --------------------------------------------------- BATTERY WARRNING --------------------------------------------------

	         if (Batt_av < 1440 && Batt_av > 1200)
	        	 {
	        	 error_signal(1);
	    	 	 buzzer_signal(1, 500);
	        	 }
	         else {
	        	 error_signal(0);
	    	 	 buzzer_signal(0, 500);

//	        	 HAL_GPIO_WritePin(GPIOC, zzub_Pin, GPIO_PIN_RESET);
//	       	     HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
	            }


// --------------------------------------------------- MOTOR and ESC control --------------------------------------------------

	 	     control_motors(idle_speed);

	         if (armed_s() == 1){

	 	        throttle = throttle_ch() + start_speed();

		 	    if (throttle > 1860)throttle = 1860;

		 	      esc_1 = throttle - Pitch_PID_out + Roll_PID_out - Yaw_PID_out;        //L_F
		 	      esc_2 = throttle + Pitch_PID_out + Roll_PID_out + Yaw_PID_out;        //L_B
		 	      esc_3 = throttle + Pitch_PID_out - Roll_PID_out - Yaw_PID_out;        //R_B
		 	      esc_4 = throttle - Pitch_PID_out - Roll_PID_out + Yaw_PID_out;        //R_F

			 	if (Batt_av < 1700 && Batt_av > 1200) {
			 	  esc_1 += (1600 - Batt_av) * battery_compensation;
			 	  esc_2 += (1600 - Batt_av) * battery_compensation;
			 	  esc_3 += (1600 - Batt_av) * battery_compensation;
			 	  esc_4 += (1600 - Batt_av) * battery_compensation;
			 	     }

		 	    if (esc_1 > 2000)esc_1 = 2000;
		 	    if (esc_2 > 2000)esc_2 = 2000;
		 	    if (esc_3 > 2000)esc_3 = 2000;
		 	    if (esc_4 > 2000)esc_4 = 2000;

		 	    if (esc_1 < idle_speed)esc_1 = idle_speed;
		 	    if (esc_2 < idle_speed)esc_2 = idle_speed;
		 	    if (esc_3 < idle_speed)esc_3 = idle_speed;
		 	    if (esc_4 < idle_speed)esc_4 = idle_speed;

	         }
	         else {

	         	Roll_PID.resetPID();
	         	Pitch_PID.resetPID();
	         	Yaw_PID.resetPID();

		 	      esc_1 = 1000;
		 	      esc_2 = 1000;
		 	      esc_3 = 1000;
		 	      esc_4 = 1000;
	         }

//	          esc_counter++;
//	          if (esc_counter == 2){

	           TIM16->CCR1 = esc_1;  // modulating the pulse width
	           TIM17->CCR1 = esc_2;
	           TIM4->CCR1  = esc_3;  // modulating the pulse width
	           TIM4->CCR2  = esc_4;

	           TIM4->CNT  = 5000 - 1; // ARR   // 400 Hz 2 ms pulse high and 0.5ms pulse low*/
	           TIM16->CNT = 5000 - 1;
	           TIM17->CNT = 5000 - 1;

//	           esc_counter = 0;
//	        	  }


	Elapsed_time = (float) (DWT->CYCCNT - S_cycle) / 72000000.0;
	while((DWT->CYCCNT - S_cycle) < 150000); // 150.000  480 Hz   - for 1khz = 1.045ms
	Loop_time = (float) (DWT->CYCCNT - S_cycle ) / 72000000.0;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.Timing = 0x0000020B;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 10;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_1);
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_2);
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_3);
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 72-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 5000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim16, TIM_CHANNEL_1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 72-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 5000-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim17, TIM_CHANNEL_1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  huart2.Init.BaudRate = 57600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|zzub_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|Green_Pin|Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 zzub_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|zzub_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Red_Pin */
  GPIO_InitStruct.Pin = Red_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Red_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB3 Green_Pin Blue_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_3|Green_Pin|Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		__io_putchar(*ptr++);
	}
	return len;
}

	void writeToEEPROM (uint32_t address, uint32_t value)
	   {
		 HAL_StatusTypeDef flash_ok = HAL_ERROR;
		 while (flash_ok != HAL_OK)
		 {
		   flash_ok = HAL_FLASH_Unlock();
		 }
		 flash_ok = HAL_ERROR;
		 while (flash_ok != HAL_OK)
		 {
		 flash_ok = HAL_FLASHEx_Erase(FLASH_TYPEERASE_PAGES, &address);
		 }
		 flash_ok = HAL_ERROR;
		 while (flash_ok != HAL_OK)
		 {
		   flash_ok = HAL_FLASH_Program (FLASH_TYPEPROGRAM_WORD, address, value);
		 }
		 flash_ok = HAL_ERROR;
		 while (flash_ok != HAL_OK)
		 {
		   flash_ok = HAL_FLASH_Lock ();
		 }
	   }

	uint32_t readFromEEPROM (uint32_t address)
	{
	  return (*(__IO uint32_t *)address);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
