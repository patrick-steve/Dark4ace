/*
 * telemetry_task.c
 *
 *  Created on: Jan 10, 2022
 *      Author: wx
 */
#include "board_lib.h"
#include "robot_config.h"
#include "typedefs.h"
#include "telemetry_task.h"
#include "queue.h"

extern TaskHandle_t telemetry_task_handle;


extern ref_game_robot_data_t ref_robot_data;
extern ref_game_state_t ref_game_state;
extern ref_magazine_data_t ref_mag_data;
extern uint32_t sbc_last_time;


queue_t g_sbc_queue;
sbc_data_t sbc_data;
uint8_t sbc_new_data = 0;
uint8_t sbc_dma_buffer[20];
static uint8_t sbc_error;

void sbc_process_data() {
	if (sbc_dma_buffer[0] == 0xa5 && sbc_dma_buffer[14] == 0x5a){
		sbc_data.cmd_id = sbc_dma_buffer[1];
		memcpy(&sbc_data.data, sbc_dma_buffer+2,12);
		sbc_new_data = 1;
		sbc_last_time = HAL_GetTick();
	} else {
		sbc_error +=1;
	}
//	queue_append_bytes(&g_sbc_queue, sbc_dma_buffer, 15);
//	BaseType_t xHigherPriorityTaskWoken;
//	xHigherPriorityTaskWoken = pdFALSE;
//	vTaskNotifyGiveFromISR(telemetry_task_handle, &xHigherPriorityTaskWoken);
//	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//lol more stuff to add
void telemetry_task(void *argument) {

	uint8_t send_buffer[256];
	uint32_t curr_time = HAL_GetTick();
//	uint32_t game_data_time = curr_time;
//	uint32_t imu_data_time = curr_time;
	sbc_game_data_t sbc_game_tx;
	queue_init(&g_sbc_queue);

	sbc_game_tx.header = 0xA5;
	sbc_game_tx.cmd_id = 0x80;
	sbc_game_tx.team = 1;
	sbc_game_tx.robot_id = 1;
	sbc_game_tx.robot_level = 1;
	sbc_game_tx.remaining_time = 1000;
	sbc_game_tx.ammo = 10;
	sbc_game_tx.end_byte = 0x5A;
	sbc_game_tx.padding[0] = 0;
	sbc_game_tx.padding[1] = 0;
	sbc_game_tx.padding[2] = 0;
	sbc_game_tx.padding[3] = 0;
	sbc_game_tx.padding[4] = 0;

	//insert sbc control here, end byte 0x5B
	sbc_imu_data_t sbc_imu_tx;
	sbc_imu_tx.cmd_id = 0x82;
	sbc_imu_tx.end_byte = 0x5C;
	sbc_imu_tx.header = 0xA5;
//	uint8_t tx_buffer[15];
		init_xvr_usart(sbc_dma_buffer);
	while (1) {
//
//		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//		while (g_sbc_queue.stored_bytes > 15){
//			if (queue_peek(&g_sbc_queue) != 0xA5){
//				queue_pop_element(&g_sbc_queue);
//			} else {
//				queue_pop_elements(&g_sbc_queue, tx_buffer, 15);
//				switch(tx_buffer[1]){
//				case SBC_GIMBAL_TURN_ANG_ID:
//				case SBC_GIMBAL_SET_ANG_ID:
//				case SBC_AIMBOT_NORM_ID:
//					if (tx_buffer[14] == 0x5A){
//						sbc_data.cmd_id = tx_buffer[1];
//						memcpy(&sbc_data.data, tx_buffer+2,12);
//						sbc_new_data = 1;
//					} else {
//
//						sbc_new_data =0 ;
//					}
//					break;
//				default:
//					break;
//				}
//			}
//		}

		if (ref_robot_data.robot_id != 0){
			sbc_game_tx.team = (ref_robot_data.robot_id < 100) ? 1 : 0;
			sbc_game_tx.robot_id = ref_robot_data.robot_id;
			sbc_game_tx.remaining_time = ref_game_state.stage_remain_time;
			sbc_game_tx.ammo = ref_mag_data.magazine_17mm;
			memcpy(send_buffer, &sbc_game_tx, 15);
			while (SBC_UART.gState == HAL_UART_STATE_BUSY_TX){
				vTaskDelay(1);
			}
			HAL_UART_Transmit_IT(&SBC_UART, send_buffer, 15);
		} else {
			sbc_game_tx.team = 1;
			sbc_game_tx.robot_id = 0;
			sbc_game_tx.remaining_time = 999;
			sbc_game_tx.ammo = 999;
			memcpy(send_buffer, &sbc_game_tx, 15);
			while (SBC_UART.gState == HAL_UART_STATE_BUSY_TX){
				vTaskDelay(1);
			}
			HAL_UART_Transmit_IT(&SBC_UART, send_buffer, 15);
		}

		if (sbc_error >= 1){
			HAL_UART_DMAStop(&SBC_UART);
			init_xvr_usart(sbc_dma_buffer);
			sbc_error = 0;
		}
		vTaskDelay(1000);

	}
}

uint8_t telem_data_size(uint8_t data_type) {
		switch (data_type) {
		case (motor_init_data):
			return sizeof(telem_motor_init_t);
		case (ref_init_data):
			return sizeof(telem_ref_init_t);
		case (motor_data):
			return sizeof(telem_motor_data_t);
		case (raw_rc_data):
			return sizeof(remote_cmd_t);
		case (raw_gyro_data):
			return sizeof(gyro_data_t);
		case (raw_accel_data):
			return sizeof(accel_data_t);
		case (raw_mag_data):
			return sizeof(mag_data_t);
		case (orientation_data):
			return sizeof(orientation_data_t);
		case (linear_accel_data):
			return sizeof(linear_accel_t);
		case (ref_bullet_data):
			return sizeof(telem_ref_bullet_data_t);
		case (ref_power_data):
			return sizeof(telem_ref_power_data_t);
		default:
			return 0;
		}
	}
