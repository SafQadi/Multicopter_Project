/*
 * alarms.c
 *
 *  Created on: Feb 11, 2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com

 */


#include "alarms.h"

uint8_t error, error_counter, error_led;
uint8_t flight_mode, flight_mode_counter, flight_mode_led;
uint8_t buzzer_mode, buzzer_mode_counter, buzzer_mode_led;
uint16_t buzzer_last;

uint32_t error_timer, flight_mode_timer, buzzer_mode_timer;



void error_signal(uint8_t e) {
	error= e;
  if (error_timer < HAL_GetTick()) {
    error_timer = HAL_GetTick() + 250;                                                     //Set the next error_timer interval at 250ms.
    if (error > 0 && error_counter > error + 3) error_counter = 0;                         //If there is an error to report and the error_counter > error +3 reset the error.
    if (error_counter < error && error_led == 0 && error > 0) {                            //If the error flash sequence isn't finisched (error_counter < error) and the LED is off.
	  HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_SET);
      error_led = 1;                                                                       //Set the LED flag to indicate that the LED is on.
    }
    else {                                                                                 //If the error flash sequence isn't finisched (error_counter < error) and the LED is on.
	  HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);
      error_counter++;                                                                     //Increment the error_counter variable by 1 to keep trach of the flashes.
      error_led = 0;                                                                       //Set the LED flag to indicate that the LED is off.
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the flight mode LED signal is generated.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void flight_mode_signal(uint8_t m) {
	flight_mode = m;
  if (flight_mode_timer  < HAL_GetTick()) {                                                      //If the error_timer value is smaller that the millis() function.
    flight_mode_timer = HAL_GetTick() + 250;                                                    //Set the next error_timer interval at 250ms.
    if (flight_mode > 0 && flight_mode_counter > flight_mode + 3) flight_mode_counter = 0; //If there is an error to report and the error_counter > error +3 reset the error.
    if (flight_mode_counter < flight_mode && flight_mode_led == 0 && flight_mode > 0) {    //If the error flash sequence isn't finisched (error_counter < error) and the LED is off.
	  HAL_GPIO_WritePin(GPIOB, Green_Pin, GPIO_PIN_SET);
      flight_mode_led = 1;                                                                 //Set the LED flag to indicate that the LED is on.
    }
    else {                                                                                 //If the error flash sequence isn't finisched (error_counter < error) and the LED is on.
	  HAL_GPIO_WritePin(GPIOB, Green_Pin, GPIO_PIN_RESET);
      flight_mode_counter++;                                                               //Increment the error_counter variable by 1 to keep trach of the flashes.
      flight_mode_led = 0;                                                                 //Set the LED flag to indicate that the LED is off.
    }
  }
}



void buzzer_signal(uint8_t b, uint16_t d) {
	buzzer_mode = b; buzzer_last = d;
  if (buzzer_mode_timer  < HAL_GetTick()) {                                                      //If the error_timer value is smaller that the millis() function.
    buzzer_mode_timer = HAL_GetTick() + buzzer_last;                                                    //Set the next error_timer interval at 250ms.
    if (buzzer_mode > 0 && buzzer_mode_counter > buzzer_mode + 3) buzzer_mode_counter = 0; //If there is an error to report and the error_counter > error +3 reset the error.


    if (buzzer_mode_counter < buzzer_mode && buzzer_mode_led == 0 && buzzer_mode > 0) {    //If the error flash sequence isn't finisched (error_counter < error) and the LED is off.
	  HAL_GPIO_WritePin(GPIOC, zzub_Pin, GPIO_PIN_SET);
      buzzer_mode_led = 1;                                                                 //Set the LED flag to indicate that the LED is on.
    }
    else {                                                                                 //If the error flash sequence isn't finisched (error_counter < error) and the LED is on.
	  HAL_GPIO_WritePin(GPIOC, zzub_Pin, GPIO_PIN_RESET);
      buzzer_mode_counter++;                                                               //Increment the error_counter variable by 1 to keep trach of the flashes.
      buzzer_mode_led = 0;                                                                 //Set the LED flag to indicate that the LED is off.
    }
  }
}
