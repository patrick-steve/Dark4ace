/*
 * bsp_micros_timer.c
 *
 *  Created on: Jan 17, 2022
 *      Author: wx
 */
#include "board_lib.h"
#include "robot_config.h"

static uint32_t gv_overflow_times = 0;

void micros_tick(){
	gv_overflow_times ++;
}

void start_micros_timer()
{
	LL_TIM_SetPrescaler(TIM2, 83999999/TIMER_FREQ);
	LL_TIM_DisableCounter(TIM2);
	TIM2->CNT =0;
//	LL_TIM_DisableExternalClock(TIM2);
	LL_TIM_SetPrescaler(TIM2, 83999999/TIMER_FREQ);
//	LL_TIM_EnableUpdateEvent(TIM2);
//	LL_TIM_EnableIT_UPDATE(TIM2);
	TIM2->EGR |= TIM_EGR_UG_Msk;
	LL_TIM_EnableCounter(TIM2);
}

static uint32_t current_cnt;

uint32_t get_microseconds()
{
	current_cnt = TIM2->CNT;
	uint64_t temp_cnt = TIM2->CNT;
	temp_cnt = temp_cnt * (1000000 / TIMER_FREQ);
	return (uint32_t)temp_cnt;
}
