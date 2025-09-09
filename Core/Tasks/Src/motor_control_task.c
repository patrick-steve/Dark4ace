/*
 * motor_control_task.c
 *
 *  Created on: Sep 26, 2023
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "can_msg_processor.h"
#include "motor_control.h"
#include "motor_control_task.h"
#include "motor_config.h"
#include "bsp_lk_motor.h"

extern motor_data_t g_can_motors[24];
extern motor_map_t dji_motor_map[25];
extern QueueHandle_t g_buzzing_task_msg;
extern remote_cmd_t g_remote_cmd;

extern uint8_t g_safety_toggle;
volatile uint32_t g_motor_control_time;
extern motor_data_t g_pitch_motor;


void empty_tx_mb1(CAN_HandleTypeDef *hcan){
	static uint8_t prev_mailbox;
	prev_mailbox ++;
	if (prev_mailbox > 2){
		prev_mailbox = 0;
	}
	if (prev_mailbox == 0){
		HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0);
	}if (prev_mailbox == 1){
		HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX1);
	}if (prev_mailbox == 2){
		HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX2);
	}
}
void empty_tx_mb2(CAN_HandleTypeDef *hcan){
	static uint8_t prev_mailbox;
	prev_mailbox ++;
	if (prev_mailbox > 2){
		prev_mailbox = 0;
	}
	if (prev_mailbox == 0){
		HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0);
	}if (prev_mailbox == 1){
		HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX1);
	}if (prev_mailbox == 2){
		HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX2);
	}
}


void motor_control_task(void *argument) {
	CAN_TxHeaderTypeDef CAN_tx_message;
	uint8_t CAN_send_data[8];
	uint32_t send_mail_box[3];
	uint8_t curr_send_box;
	int16_t temp_converter;
	CAN_tx_message.IDE = CAN_ID_STD;
	CAN_tx_message.RTR = CAN_RTR_DATA;
	CAN_tx_message.DLC = 0x08;
	uint32_t dji_enabled_motors = 0;

	for (uint8_t i = 0; i < 24; i ++){
		if (dji_motor_map[i+1].motor_data != NULL){
			dji_enabled_motors = dji_enabled_motors | 1 << i;
		}
	}
	TickType_t start_time;
//	uint32_t last_time;
	TickType_t delay = 0;
	while (1) {
		delay = 0;
		start_time = xTaskGetTickCount();
		//if safety is on

		if (g_safety_toggle || g_remote_cmd.right_switch == ge_RSW_SHUTDOWN){

#if PITCH_MOTOR_TYPE > TYPE_GM6020_720	//check if motor is LK or DJI
			lk_motor_kill(&g_pitch_motor);
#endif
			//add lk kill motor
			CAN_send_data[0] = 0;
			CAN_send_data[1] = 0;
			CAN_send_data[2] = 0;
			CAN_send_data[3] = 0;
			CAN_send_data[4] = 0;
			CAN_send_data[5] = 0;
			CAN_send_data[6] = 0;
			CAN_send_data[7] = 0;
			CAN_tx_message.StdId = 0x200;
			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0){
				empty_tx_mb1(&hcan1);
			}
			if (dji_enabled_motors & 0x00000F) {
				HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data,
					send_mail_box);
			}

			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0){
				empty_tx_mb2(&hcan2);
			}
			if (dji_enabled_motors & 0x00F000) {
				HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
						send_mail_box);
			}

			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0){
				empty_tx_mb1(&hcan1);
			}
			CAN_tx_message.StdId = 0x1FF;
			if (dji_enabled_motors & 0x0000F0) {
			HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data,
					send_mail_box);
			}

			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0){
				empty_tx_mb2(&hcan2);
			}
			if (dji_enabled_motors & 0x0F0000) {
				HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
					send_mail_box);
			}

			CAN_tx_message.StdId = 0x2FF;

			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0){
				empty_tx_mb1(&hcan1);
			}


			if (dji_enabled_motors & 0x000F00) {
				HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data,
						send_mail_box);
			}

			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0){
				empty_tx_mb2(&hcan2);
			}
			if (dji_enabled_motors & 0xF00000) {
				HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
						send_mail_box);
			}
			vTaskDelayUntil(&start_time, 10);
			continue;
		}

		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0){
			empty_tx_mb1(&hcan1);
		}
		if (dji_enabled_motors & 0x00000F) {
			CAN_tx_message.StdId = 0x200;
			CAN_send_data[0] = (dji_motor_map[1].motor_data->output) >> 8;
			CAN_send_data[1] = (dji_motor_map[1].motor_data->output);
			CAN_send_data[2] = (dji_motor_map[2].motor_data->output) >> 8;
			CAN_send_data[3] = (dji_motor_map[2].motor_data->output);
			CAN_send_data[4] = (dji_motor_map[3].motor_data->output) >> 8;
			CAN_send_data[5] = (dji_motor_map[3].motor_data->output);
			CAN_send_data[6] = (dji_motor_map[4].motor_data->output) >> 8;
			CAN_send_data[7] = (dji_motor_map[4].motor_data->output);
			HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data,
					send_mail_box);
		}


		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0){
			empty_tx_mb2(&hcan2);
		}
		if (dji_enabled_motors & 0x00F000) {
			CAN_tx_message.StdId = 0x200;
			CAN_send_data[0] = (dji_motor_map[13].motor_data->output) >> 8;
			CAN_send_data[1] = (dji_motor_map[13].motor_data->output);
			CAN_send_data[2] = (dji_motor_map[14].motor_data->output) >> 8;
			CAN_send_data[3] = (dji_motor_map[14].motor_data->output);
			CAN_send_data[4] = (dji_motor_map[15].motor_data->output) >> 8;
			CAN_send_data[5] = (dji_motor_map[15].motor_data->output);
			CAN_send_data[6] = (dji_motor_map[16].motor_data->output) >> 8;
			CAN_send_data[7] = (dji_motor_map[16].motor_data->output);
			HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
					send_mail_box);
		}

		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0){
			empty_tx_mb1(&hcan1);
		}

		if (dji_enabled_motors & 0x0000F0) {
			CAN_tx_message.StdId = 0x1FF;
			CAN_send_data[0] = (dji_motor_map[5].motor_data->output) >> 8;
			CAN_send_data[1] = (dji_motor_map[5].motor_data->output);
			CAN_send_data[2] = (dji_motor_map[6].motor_data->output) >> 8;
			CAN_send_data[3] = (dji_motor_map[6].motor_data->output);
			CAN_send_data[4] = (dji_motor_map[7].motor_data->output) >> 8;
			CAN_send_data[5] = (dji_motor_map[7].motor_data->output);
			CAN_send_data[6] = (dji_motor_map[8].motor_data->output) >> 8;
			CAN_send_data[7] = (dji_motor_map[8].motor_data->output);
			HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data,
					send_mail_box);
		}

		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0){
			empty_tx_mb2(&hcan2);
		}
		if (dji_enabled_motors & 0x0F0000) {
			CAN_tx_message.StdId = 0x1FF;
			CAN_send_data[0] = (dji_motor_map[5+12].motor_data->output) >> 8;
			CAN_send_data[1] = (dji_motor_map[5+12].motor_data->output);
			CAN_send_data[2] = (dji_motor_map[6+12].motor_data->output) >> 8;
			CAN_send_data[3] = (dji_motor_map[6+12].motor_data->output);
			CAN_send_data[4] = (dji_motor_map[7+12].motor_data->output) >> 8;
			CAN_send_data[5] = (dji_motor_map[7+12].motor_data->output);
			CAN_send_data[6] = (dji_motor_map[8+12].motor_data->output) >> 8;
			CAN_send_data[7] = (dji_motor_map[8+12].motor_data->output);
			HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
					send_mail_box);
		}


		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0){
			empty_tx_mb1(&hcan1);
		}
		if (dji_enabled_motors & 0x000F00) {
			CAN_tx_message.StdId = 0x2FF;
			CAN_send_data[0] = (dji_motor_map[9].motor_data->output) >> 8;
			CAN_send_data[1] = (dji_motor_map[9].motor_data->output);
			CAN_send_data[2] = (dji_motor_map[10].motor_data->output) >> 8;
			CAN_send_data[3] = (dji_motor_map[10].motor_data->output);
			CAN_send_data[4] = (dji_motor_map[11].motor_data->output) >> 8;
			CAN_send_data[5] = (dji_motor_map[11].motor_data->output);
			CAN_send_data[6] = (dji_motor_map[12].motor_data->output) >> 8;
			CAN_send_data[7] = (dji_motor_map[12].motor_data->output);
			HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data,
					send_mail_box);
		}

		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0){
			empty_tx_mb2(&hcan2);
		}
		if (dji_enabled_motors & 0xF00000) {
			CAN_tx_message.StdId = 0x2FF;
			CAN_send_data[0] = (dji_motor_map[9+12].motor_data->output) >> 8;
			CAN_send_data[1] = (dji_motor_map[9+12].motor_data->output);
			CAN_send_data[2] = (dji_motor_map[10+12].motor_data->output) >> 8;
			CAN_send_data[3] = (dji_motor_map[10+12].motor_data->output);
			CAN_send_data[4] = (dji_motor_map[11+12].motor_data->output) >> 8;
			CAN_send_data[5] = (dji_motor_map[11+12].motor_data->output);
			CAN_send_data[6] = (dji_motor_map[12+12].motor_data->output) >> 8;
			CAN_send_data[7] = (dji_motor_map[12+12].motor_data->output);
			HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
					send_mail_box);
		}


#if PITCH_MOTOR_TYPE >= TYPE_LK_MG5010E_SPD
		lk_read_motor_sang(&g_pitch_motor);
#endif


		delay = (delay > 2) ? 1 : delay;
//		last_time = get_microseconds();
		vTaskDelayUntil(&start_time, 2-(delay));
//		vTaskDelay(1);



	}
}
