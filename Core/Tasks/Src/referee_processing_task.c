/*
 * referee_processing_task.c
 *
 *  Created on: Jun 18, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "bsp_queue.h"
#include "bsp_referee.h"
#include "bsp_usart.h"
#include "referee_processing_task.h"
#include "referee_msgs.h"
#include "robot_config.h"
#include "rtos_g_vars.h"

extern int g_spinspin_mode;
extern uint8_t remote_raw_data[18];
extern TaskHandle_t referee_processing_task_handle;
extern DMA_HandleTypeDef hdma_usart6_rx;
referee_limit_t g_referee_limiters;
static ref_msg_t g_ref_msg_buffer;

ref_game_state_t ref_game_state;
uint32_t ref_game_state_txno = 0;

ref_game_robot_HP_t ref_robot_hp;
uint32_t ref_robot_hp_txno = 0;

ref_game_robot_data2_t ref_robot_data;
uint32_t ref_robot_data_txno = 0;

ref_robot_power_data_t ref_power_data;
uint32_t ref_power_data_txno = 0;

ref_game_robot_pos_t ref_robot_pos;
uint32_t ref_robot_pos_txno = 0;

ref_robot_dmg_t ref_dmg_data;
uint32_t ref_dmg_data_txno = 0;

ref_shoot_data_t ref_shoot_data;
uint32_t ref_shoot_data_txno = 0;

ref_magazine_data_t ref_mag_data;
uint32_t ref_mag_data_txno = 0;
uint8_t g_ref_tx_seq = 0;

ref_game_event_data_t ref_game_event;
uint32_t ref_game_event_txno = 0;

uint8_t ref_buffer[2];
queue_t referee_uart_q;

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart){
	if (huart== &DBUS_UART){
		HAL_UART_DMAStop(&DBUS_UART);
		dbus_remote_start();
	} else if (huart == &REFEREE_UART){

	    __HAL_DMA_DISABLE(&hdma_usart6_rx);
		ref_usart_start(&REFEREE_UART, ref_buffer, 2, &referee_uart_q);
	}
}

void referee_processing_task(void *argument) {
	g_referee_limiters.robot_level = 1;
	ref_processing_status_t proc_status;
	g_referee_limiters.feeding_speed = FEEDER_SPEED;
	g_referee_limiters.projectile_speed = PROJECTILE_SPEED;
//	g_referee_limiters.wheel_power_limit = LV1_POWER;
	status_led(7, on_led);
	status_led(8, off_led);
	ref_robot_data.robot_id = 0;
	ref_usart_start(&REFEREE_UART, ref_buffer, 2, &referee_uart_q);
	while (1) {

		uint8_t has_data = ulTaskNotifyTake(pdTRUE, 1000);
		status_led(5, on_led);
		if (queue_get_size(&referee_uart_q) > 7) {
			while (queue_get_size(&referee_uart_q) > 7) {
				proc_status = ref_process_data(&referee_uart_q, &g_ref_msg_buffer);
				if (proc_status == PROCESS_SUCCESS) {
					switch (g_ref_msg_buffer.cmd_id) {
					case REF_ROBOT_SHOOT_DATA_CMD_ID:
						memcpy(&ref_shoot_data, &g_ref_msg_buffer.data,
								sizeof(ref_shoot_data_t));
						ref_shoot_data_txno++;
						break;
					case REF_GAME_STATE_CMD_ID:
							memcpy(&ref_game_state, &g_ref_msg_buffer.data,
									sizeof(ref_game_state_t));
							ref_game_state_txno++;
							break;
					case REF_ROBOT_DATA_CMD_ID:
						memcpy(&ref_robot_data, &g_ref_msg_buffer.data,
								sizeof(ref_game_robot_data2_t));
						ref_robot_data_txno++;
						break;
					case REF_ROBOT_POS_DATA_CMD_ID:
						memcpy(&ref_robot_pos, &g_ref_msg_buffer.data,
								sizeof(ref_game_robot_pos_t));
						ref_robot_pos_txno++;
						break;
					case REF_ROBOT_DMG_DATA_CMD_ID:
						memcpy(&ref_dmg_data, &g_ref_msg_buffer.data,
								sizeof(ref_robot_dmg_t));
						ref_dmg_data_txno++;
						break;
					case REF_ROBOT_HP_CMD_ID:
						memcpy(&ref_robot_hp, &g_ref_msg_buffer.data,
								sizeof(ref_game_robot_HP_t));
						ref_robot_hp_txno++;
						break;
					case REF_ROBOT_MAGAZINE_DATA_CMD_ID:
						memcpy(&ref_mag_data, &g_ref_msg_buffer.data,
								sizeof(ref_magazine_data_t));
						ref_mag_data_txno++;
						break;
					// add your code here !!
					case REF_GAME_EVENT_CMD_ID:
						memcpy(&ref_game_event, &g_ref_msg_buffer.data,
							sizeof(ref_game_event_data_t));
						ref_game_event_txno++;
						break;
					default:
						break;
					/* todo: 6. Get data from the referee system
					 *
					 * We have removed a switch case from above.
					 *
					 * Look at the Referee System Serial Port Protocol Appendix
					 * to find out what Referee System data you can obtain.
					 *
					 * Then, compare with the data used in the code and find what's missing ;)
					 */
					}
				} else if (proc_status == INSUFFICIENT_DATA) {
					break;
				}
			}
		}
		if (!has_data){
		    __HAL_DMA_DISABLE(&hdma_usart6_rx);
			ref_usart_start(&REFEREE_UART, ref_buffer, 2, &referee_uart_q);

		}

		status_led(5, off_led);


		status_led(5, on_led);
		if (ref_robot_data.robot_level != 0) {
			static uint32_t prev_power_tx_no = 0;
			if (prev_power_tx_no != ref_power_data_txno){
				prev_power_tx_no = ref_power_data_txno;
				float temp_buffer = 1;					 // if buffer>30J, multiplier = 1
				if (ref_power_data.buffer_energy < 30){  // if buffer<30J, multiplier = x/34 + BUFFER_MIN
					temp_buffer = ((float)ref_power_data.buffer_energy/34) + BUFFER_MIN;
					temp_buffer = (temp_buffer > 1) ? 1 : temp_buffer;
				}
	#ifdef CHASSIS_POWER_BUFFER_LIMITER
				g_referee_limiters.wheel_buffer_limit = temp_buffer; // wheel_buffer_limit is a multiplier for motor pid output
	#else
				g_referee_limiters.wheel_buffer_limit = 1;
	#endif

				static float prev_chassis_power;
				float max_power = CHASSIS_MAX_POWER;
				float curr_chassis_power = prev_chassis_power * CHASSIS_POWER_LPF + ref_power_data.chassis_power * (1-CHASSIS_POWER_LPF);	//filter on chassis power
				prev_chassis_power = ref_power_data.chassis_power;
				if (ref_robot_data.chassis_power_limit < CHASSIS_MAX_POWER){ //set max_power to referee system max power
					max_power = ref_robot_data.chassis_power_limit;

				}
				float temp_power = (float) ((curr_chassis_power)/(max_power - CHASSIS_POWER_MARGIN)); //instantaneous chassis power divided by in-game max chassis power
	//			temp_power = (temp_power > 1) ? 1 : temp_power;										  //eg if instantaneous is 160W, game limit is 80W, temp_power = 2
				g_referee_limiters.wheel_power_limit = temp_power;
	//			arm_sqrt_f32(temp_power, &referee_limiters.wheel_power_limit);
			}
		} else {
			g_referee_limiters.wheel_buffer_limit = 1;
			g_referee_limiters.wheel_power_limit = 1;

		}
		vTaskDelay(2);
	}
}



