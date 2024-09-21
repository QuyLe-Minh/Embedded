/*
 * software_timer.c
 *
 *  Created on: Sep 24, 2023
 *      Author: HaHuyen
 */

#include "software_timer.h"

#define TIMER_CYCLE_2 1

//software timer variable
uint16_t flag_timer2 = 0;
uint16_t timer2_counter = 0;
uint16_t timer2_MUL = 0;

uint16_t flag_timer3 = 0;
uint16_t timer3_counter = 0;
uint16_t timer3_MUL = 0;

uint16_t flag_timer1 = 0;
uint16_t timer1_counter = 0;
uint16_t timer1_MUL = 0;


/**
  * @brief  Init timer interrupt
  * @param  None
  * @retval None
  */
void timer_init(){
	HAL_TIM_Base_Start_IT(&htim2);
}


/**
  * @brief  Set duration of software timer interrupt
  * @param  duration Duration of software timer interrupt
  * @retval None
  */

void setTimer1(uint16_t duration){
	timer1_MUL = duration/TIMER_CYCLE_2;
	timer1_counter = timer1_MUL;
	flag_timer1 = 0;
}

void setTimer2(uint16_t duration){
	timer2_MUL = duration/TIMER_CYCLE_2;
	timer2_counter = timer2_MUL;
	flag_timer2 = 0;
}

void setTimer3(uint16_t duration){
	timer3_MUL = duration/TIMER_CYCLE_2;
	timer3_counter = timer3_MUL;
	flag_timer3 = 0;
}
/**
  * @brief  Timer interrupt routine
  * @param  htim TIM Base handle
  * @note	This callback function is called by system
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim ){
	if(htim->Instance == TIM2){
		if(timer2_counter > 0){
			timer2_counter--;
			if(timer2_counter == 0) {
				flag_timer2 = 1;
				timer2_counter = timer2_MUL;
			}
		}
		if(timer1_counter > 0){
			timer1_counter--;
			if(timer1_counter == 0) {
				flag_timer1 = 1;
				timer1_counter = timer1_MUL;
				led7_Scan();
			}
		}
		if(timer3_counter > 0){
			timer3_counter--;
			if(timer3_counter == 0) {
				flag_timer3 = 1;
				timer3_counter = timer3_MUL;
			}
		}
		// 1ms interrupt here
	}
}
