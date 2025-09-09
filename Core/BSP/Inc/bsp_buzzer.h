/*
 * bsp_buzzer.h
 *
 *  Created on: May 23, 2021
 *      Author: wx
 */

#ifndef BSP_INC_BSP_BUZZER_H_
#define BSP_INC_BSP_BUZZER_H_


#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

#define HALL_ON 1
#define HALL_OFF (1-HALL_ON)

void buzzer(uint16_t freq);
void buzzer_init();
void hall_enable();
void hall_disable();

#endif /* BSP_INC_BSP_BUZZER_H_ */
