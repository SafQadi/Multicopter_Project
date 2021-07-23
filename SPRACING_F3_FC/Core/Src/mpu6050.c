/*
 * mpu6050.c
 *
 *  Created on: Jan 25, 2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
 */

#include "main.h"
#include "mpu6050.h"


I2C_HandleTypeDef hi2c1;


int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;


float Gx, Gy, Gz;
float Ax, Ay, Az;

//int16_t gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0, acc_x_cal = 0, acc_y_cal = 0;
int16_t gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;


int16_t acc_x_cal   = -9;
int16_t acc_y_cal	=  10;
//int16_t gyro_x_cal	= -27;
//int16_t gyro_y_cal	=  32;
//int16_t gyro_z_cal	=  7;

const float GYRO_SCALE = 32.8; // 32.8 for 1000 deg/s, 65,5 for 500 deg/s


void MPU6050_confirm (void){


	  for(uint8_t i = 0; i < 255; i++)
	  {
	      if(HAL_I2C_IsDeviceReady(&hi2c1, i, 1 , 10) == HAL_OK )
	      {
	    	 //printf("Address:  %d\n\r", i);
	         //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

			    HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
		        HAL_Delay(500);
			    HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);

//	        WRITE_REG(GPIOB->BSRR, GPIO_BSRR_BS_8);  // on
//	        HAL_Delay(200);
//	        WRITE_REG(GPIOB->BSRR, GPIO_BSRR_BR_8); //off


	         break;
	      }
	   }
}


void MPU6050_Init (void)
{

	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);
		HAL_Delay(50);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		//Data = 0x07;
		//HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, AFS_SEL=2 -> � 8g
		Data = 0x10; // ACCEL_CONFIG REGISTER = 0001 0000
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=2 -> � 1000 deg/s
		// 0x08 = 500, 0x18 = 2000, 0x10 =1000,
		Data = 0x10; // GYRO_CONFIG REGISTER = 0001 0000
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// Filter 44Hz  GYRO_CONFIG REGISTER = 00000011
		Data = 0x03;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, DLPF_CFG, 1, &Data, 1, 1000);
	}

}


void MPU6050_calib_Gyro (void)
{

	for (int16_t cal_int = 0; cal_int < 500 ; cal_int ++) {
	    if (cal_int % 25 == 0){

	    HAL_GPIO_TogglePin(GPIOB, Green_Pin);



	    	// GPIOA->BSRR = ((GPIOB->ODR ^ GPIO_PIN_8) & GPIO_PIN_8) | GPIO_PIN_8 << 16;
	    }

	    MPU6050_Read_Gyro();

	    gyro_x_cal  += Gyro_X_RAW;
	    gyro_y_cal  += Gyro_Y_RAW;
	    gyro_z_cal  += Gyro_Z_RAW;

		HAL_Delay(2);

	}

    gyro_x_cal  /= 500;
    gyro_y_cal  /= 500;
    gyro_z_cal  /= 500;
}


void MPU6050_calib_Accel (void)
{

	for (int16_t cal_int = 0; cal_int < 500 ; cal_int ++) {
	    if (cal_int % 25 == 0){
		    HAL_GPIO_TogglePin(GPIOB, Blue_Pin);

//	    	 GPIOA->BSRR = ((GPIOB->ODR ^ GPIO_PIN_8 ) & GPIO_PIN_8) | GPIO_PIN_8 << 16;
	    }

	    MPU6050_Read_Accel();

		 acc_x_cal  += Accel_X_RAW;
		 acc_y_cal  += Accel_Y_RAW;

		 HAL_Delay(2);

	}

	 acc_x_cal  /= 500;
	 acc_y_cal  /= 500;
}


void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
//	HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

    Gx = (float)(Gyro_X_RAW - gyro_x_cal) / GYRO_SCALE;
	Gy = (float)(Gyro_Y_RAW - gyro_y_cal) / GYRO_SCALE;
	Gz = (float)(Gyro_Z_RAW - gyro_z_cal) / GYRO_SCALE;

}

void MPU6050_Read_Accel (void)
{

	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
//	HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6);


	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Ax = (Accel_X_RAW - acc_x_cal);// / 4096; // 8g   +9
	Ay = (Accel_Y_RAW - acc_y_cal);// / 4096;     -10
	Az = (Accel_Z_RAW );// / 16384.0;

}


float gyro_X (void) { return Gx; }
float gyro_Y (void) { return Gy; }
float gyro_Z (void) { return Gz; }

float accel_X (void) { return Ax; }
float accel_Y (void) { return Ay; }
float accel_Z (void) { return Az; }





