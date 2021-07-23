/*
 * IC_ISR.h
 *
 *  Created on: 28.01.2020
 *      Author: Dr. Safwan Al-Qadhi
 *      Email: saq.qadi@gmail.com
 */

#ifndef INC_IC_ISR_H_
#define INC_IC_ISR_H_

#ifdef __cplusplus
extern "C" {
#endif


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);


uint16_t throttle_ch (void);
uint16_t roll_ch (void);
uint16_t pitch_ch (void) ;
uint16_t yaw_ch (void) ;
uint16_t F_mode_ch (void) ;
uint16_t Aux6_ch (void) ;
uint16_t PID_select_ch (void) ;
uint16_t PID_change_ch (void) ;


void PWM_ESC_tim_setup(void);


#ifdef __cplusplus
}
#endif

#endif /* INC_IC_ISR_H_ */
