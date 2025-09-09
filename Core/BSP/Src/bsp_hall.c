/*
 * bsp_hall.c
 *
 *  Created on: Mar 21, 2024
 *      Author: wx
 */
#include "board_lib.h"
#include "robot_config.h"
//to switch from active high to low whenever
extern motor_data_t g_can_motors[24];

static uint8_t prev_state = HALL_OFF;

uint8_t hall_state = HALL_OFF;


void hall_enable(){
	hall_state = HALL_ON;
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void hall_disable(){
	hall_state = HALL_OFF;
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}


void hall_int(){
	static int32_t prev_tick;
	static uint8_t prev_state;
	if (hall_state == HALL_OFF){
		//irq is enabled by default.... why
		hall_disable();
		return;
	}
	//don't care if rising or falling edge, roughly centered anyways
//	uint8_t curr_state = (GPIOE->IDR & GPIO_PIN_11) ? 1 : 0;
	g_can_motors[YAW_MOTOR_ID-1].angle_data.ticks = 0;
	hall_disable();
	return;
}

