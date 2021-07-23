/*
 * IC_ISR.c
 *
 *  Created on: 28.01.2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
 */

#include "main.h"
#include "IC_ISR.h"

TIM_HandleTypeDef htim2;


uint32_t IC_value_1 = 0;
uint32_t IC_value_2 = 0;
uint16_t difference = 0;
uint16_t receiver_watchdog;
uint8_t Is_first_capture = 0;
uint8_t channel_select_counter;

uint16_t channel_1;
uint16_t channel_2;
uint16_t channel_3;
uint16_t channel_4;
uint16_t channel_5;
uint16_t channel_6;
uint16_t channel_7;
uint16_t channel_8;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt source is channel 1
  {
	  if (Is_first_capture == 0)
	  {
		  IC_value_1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		  Is_first_capture = 1;
		  __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
	  }
	  else if (Is_first_capture == 1)
	  {
		  IC_value_2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		  __HAL_TIM_SET_COUNTER(&htim2, 0);

		  if (IC_value_2 > IC_value_1)
		  {
			  difference = (IC_value_2 - IC_value_1) + 500;
		  }
		  else difference = ((0XFFFF - IC_value_1) + IC_value_2) + 500;  // not necessary since timer is reseted

		  __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

		  Is_first_capture = 0;


		  if (difference > 3000) {
		      channel_select_counter = 0;
		      receiver_watchdog = 0;
		    }
		    else channel_select_counter++;

		    if (channel_select_counter == 1)channel_1 = difference;
		    if (channel_select_counter == 2)channel_2 = difference;
		    if (channel_select_counter == 3)channel_3 = difference;
		    if (channel_select_counter == 4)channel_4 = difference;
		    if (channel_select_counter == 5)channel_5 = difference;
		    if (channel_select_counter == 6)channel_6 = difference;
		    if (channel_select_counter == 7)channel_7 = difference;
		    if (channel_select_counter == 8)channel_8 = difference;
	  }
  }

}


uint16_t throttle_ch (void) { return channel_1; }
uint16_t roll_ch (void) { return channel_2; }
uint16_t pitch_ch (void) { return channel_3; }
uint16_t yaw_ch (void) { return channel_4; }
uint16_t F_mode_ch (void) { return channel_5; }
uint16_t Aux6_ch (void) { return channel_6; }
uint16_t PID_select_ch (void) { return channel_7; }
uint16_t PID_change_ch (void) { return channel_8; }


void PWM_ESC_tim_setup(void){

	// TIM_CR1_ARPE auto reload preload enable: TIM_ARR register is buffered -> transfer to the shadow register.
	  TIM4->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
	  TIM4->CR2 = 0;
	  TIM4->SMCR = 0;
	  TIM4->DIER = 0;
	  TIM4->EGR = 0;
	  //(0b110 << 4) | (0b110 << 12) | setting the compare mode - here PWM mode, upcounting, active as long as TIM4_CNT < TIM4_CCRI, else inactive
	  // TIM_CCMR1_OC1PE preload  enabled
	  // TIM_CCMR1_OC2PE output compare  2 preload enable on CCRx
	  TIM4->CCMR1 = (0b110 << 4) | TIM_CCMR1_OC1PE | (0b110 << 12) | TIM_CCMR1_OC2PE;
	  TIM4->CCMR2 = (0b110 << 4) | TIM_CCMR2_OC3PE | (0b110 << 12) | TIM_CCMR2_OC4PE;
	  //capture output enable from the capture/compare enable register CCER
	  TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

	  //the same as TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	  /*HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
	  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2);
	  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_3);
	  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);*/

		// TIM_CR1_ARPE auto reload preload enable: TIM_ARR register is buffered -> transfer to the shadow register.
		  TIM16->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
		  TIM16->CR2 = 0;
		  TIM16->SMCR = 0;
		  TIM16->DIER = 0;
		  TIM16->EGR = 0;
		  //(0b110 << 4) | (0b110 << 12) | setting the compare mode - here PWM mode, upcounting, active as long as TIM16_CNT < TIM16_CCRI, else inactive
		  // TIM_CCMR1_OC1PE preload  enabled
		  // TIM_CCMR1_OC2PE output compare  2 preload enable on CCRx
		  TIM16->CCMR1 = (0b110 << 4) | TIM_CCMR1_OC1PE;
		  //| (0b110 << 12) | TIM_CCMR1_OC2PE;
		  //capture output enable from the capture/compare enable register CCER
		  TIM16->CCER = TIM_CCER_CC1E;
		  //| TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;


		  // TIM_CR1_ARPE auto reload preload enable: TIM_ARR register is buffered -> transfer to the shadow register.
		  TIM17->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
		  TIM17->CR2 = 0;
		  TIM17->SMCR = 0;
		  TIM17->DIER = 0;
		  TIM17->EGR = 0;
		  //(0b110 << 4) | (0b110 << 12) | setting the compare mode - here PWM mode, upcounting, active as long as TIM17_CNT < TIM17_CCRI, else inactive
		  // TIM_CCMR1_OC1PE preload  enabled
		  // TIM_CCMR1_OC2PE output compare  2 preload enable on CCRx
		  TIM17->CCMR1 = (0b110 << 4) | TIM_CCMR1_OC1PE;
		  //| (0b110 << 12) | TIM_CCMR1_OC2PE;
		  //capture output enable from the capture/compare enable register CCER
		  TIM17->CCER = TIM_CCER_CC1E;
		  //| TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;




		  TIM16->CCR1 = 1000;  // modulating the pulse width
		  TIM17->CCR1 = 1000;
		  TIM4->CCR1  = 1000;  // modulating the pulse width
		  TIM4->CCR2  = 1000;

}
