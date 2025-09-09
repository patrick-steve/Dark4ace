/*
 * usb_config_task.c
 *
 *  Created on: Dec 20, 2021
 *      Author: wx
 */
#include "board_lib.h"
#include "robot_config.h"
#include "stdio.h"
#include "strings.h"
#include "motor_control.h"
#include "control_input_task.h"
#include "launcher_control_task.h"
#include "referee_processing_task.h"
#include "usb_task.h"
#include <can_msg_processor.h>
#include "rtos_g_vars.h"


extern remote_cmd_t g_remote_cmd;
extern gimbal_control_t gimbal_ctrl_data;
extern orientation_data_t imu_heading;
extern motor_data_t g_can_motors[24];
extern uint8_t g_safety_toggle;
extern referee_limit_t g_referee_limiters;

#define MAX_CHAR_SIZE 256
#define USB_TIMEOUTS_BEFORE_RESET 20
uint8_t usb_config_mode = 0;
uint8_t gv_usb_connected = 0;
uint8_t usb_input_buffer[MAX_CHAR_SIZE];
uint32_t usb_input_len;
uint8_t usb_waiting = 0;

//#define DATA_OUTPUT_MODE
#define USB_CONFIG_MODE

void usb_vcp_processing(uint8_t *buffer, uint32_t *len) {
	uint8_t blank_buffer[MAX_CHAR_SIZE] = { 0 };
	memcpy(blank_buffer, buffer, *len);
	memcpy(usb_input_buffer, blank_buffer, MAX_CHAR_SIZE);
	usb_input_len = *len;

	//Check if the currently running task needs to yield
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(usb_continue_semaphore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void usb_clear_screen() {
	const char *buffer = "[2J";
	CDC_Transmit_FS((uint8_t*) buffer, 3);
}

const char* return_variable_name(uint8_t variable_enum) {
	switch (variable_enum) {
	case angle_kp:
		return "angle KP";
	case angle_ki:
		return "angle KI";
	case angle_kd:
		return "angle KD";
	case rpm_kp:
		return "RPM KP";
	case rpm_ki:
		return "RPM KI";
	case rpm_kd:
		return "RPM KD";
	case max_torque:
		return "max torque";
	case center_angle:
		return "center angle";
	case max_angle:
		return "max angle";
	case min_angle:
		return "min angle";
	default:
		break;
	}
	return NULL;
}
//setup proper telemetry "conventions" maybe in the form of MAVLink
//then rewrite this to support that


uint8_t motor_config_mode(uint8_t skip) {
	uint32_t received_msg = 0;
	uint8_t send_buffer[MAX_CHAR_SIZE];
	uint16_t send_len;
	if (skip == 0) {
		g_safety_toggle = 1; //triggers safety toggle so if motors were moving it won't move until safety-untoggled
		//let all the control tasks shut off
		vTaskDelay(20);
		vTaskSuspend(movement_control_task_handle); //stops all other tasks from working...to be safe
		vTaskSuspend(gimbal_control_task_handle);
		vTaskSuspend(launcher_control_task_handle);
		kill_can();			//kill motors pls
		send_len = sprintf((char*)send_buffer, "Motor config mode\n"
				"Current RPM: %u\n"
				"Enter e to exit\n"
				"Enter RPM (yeeet):\n", (g_referee_limiters.feeding_speed* FEEDER_SPEED_RATIO));
		CDC_Transmit_FS(send_buffer, send_len);
		vTaskDelay(1);
	}
	vTaskDelay(1);
	received_msg = xSemaphoreTake(usb_continue_semaphore, 30000);
	if (received_msg == pdTRUE) {
		if (*usb_input_buffer == 'e' || *usb_input_buffer == 'E') {
			vTaskDelay(5);
			send_len = sprintf((char*)send_buffer, "Exiting motor config mode\n");
			CDC_Transmit_FS(send_buffer, send_len);
			vTaskResume(movement_control_task_handle); //stops all other tasks from working...to be safe
			vTaskResume(gimbal_control_task_handle);
			vTaskResume(launcher_control_task_handle);
			return 0;
		} else {
			int new_pwm_val;
			sscanf((char*)usb_input_buffer, "%d",&new_pwm_val );
			if (new_pwm_val >= -4000 && new_pwm_val<= 4000){
				send_len = sprintf((char*)send_buffer, "New speed value set to %d\n",new_pwm_val);
				CDC_Transmit_FS(send_buffer, send_len);
				g_referee_limiters.feeding_speed = new_pwm_val/FEEDER_SPEED_RATIO;
				vTaskResume(movement_control_task_handle); //stops all other tasks from working...to be safe
				vTaskResume(gimbal_control_task_handle);
				vTaskResume(launcher_control_task_handle);
				return 0;
			} else {
				send_len = sprintf((char*)send_buffer, "val not within range, enter again pls\n");
				CDC_Transmit_FS(send_buffer, send_len);
				return 1;
			}
		}
		//reset buffer
	} else {
		send_len = sprintf((char*)send_buffer, "No message received, exiting motor config mode\n");
		CDC_Transmit_FS(send_buffer, send_len);
		vTaskResume(movement_control_task_handle);
		vTaskResume(gimbal_control_task_handle);
		vTaskResume(launcher_control_task_handle);
		return 0;
	}
	vTaskDelay(5);
	vTaskResume(movement_control_task_handle);
	vTaskResume(gimbal_control_task_handle);
	vTaskResume(launcher_control_task_handle);
	return 0;

}


void usb_task(void *argument) {


	while (1) {

//		if (curr_time-imu_data_time > 100){
//			CDC_Transmit_FS((uint8_t* sbc_game_tx), 15);
//			imu_data_time = HAL_GetTick();
//		}

//		uint8_t timeout_counter = 0;
//		uint8_t configuring_motors = 1;
//		uint8_t skipping = 0;
//		uint8_t num_line = 0;
//		uint16_t send_len;
//		BaseType_t received_msg;
//		received_msg = xSemaphoreTake(usb_continue_semaphore, 1);
//		if (received_msg == pdTRUE) {
//			configuring_motors = 1;
//			if (usb_config_mode == 0) {
//				send_len = sprintf(send_buffer, 	"Enter 'M' to enter motor config mode \n"
//										"Enter 'e' to exit\n");
//				CDC_Transmit_FS(send_buffer, send_len);
//				usb_config_mode = 1;
//			} else {
//				switch (*usb_input_buffer) {
//				case ('m'):
//				case ('M'):
//					while (configuring_motors) {
//						skipping = motor_config_mode(skipping);
//						configuring_motors = skipping;
//					}
//					usb_config_mode = 0;
//					break;
//				case ('E'):
//				case ('e'):
//					usb_config_mode = 0;
//					break;
//				default:
//					send_len = sprintf(send_buffer, "Invalid command\n");
//					CDC_Transmit_FS(send_buffer, send_len);
//					break;
//				}
//				if (usb_config_mode == 0){
//					send_len = sprintf(send_buffer, "Exiting, sending referee values\n");
//					CDC_Transmit_FS(send_buffer, send_len);
//				}
//			}
//		} else {
//			if (usb_config_mode) {
//				timeout_counter++;
//				if (timeout_counter > USB_TIMEOUTS_BEFORE_RESET) {
//					timeout_counter = 0;
//					usb_config_mode = 0;
//				}
//			} else {
//				timeout_counter = 0;
//				ref_msg_t msg_buffer;
//				BaseType_t avail_msg = xQueueReceive(uart_data_queue, &msg_buffer, 0);
//				if (avail_msg == pdPASS){
//					if (msg_buffer.cmd_id == REF_ROBOT_SHOOT_DATA_CMD_ID){
//						int len = sprintf((char*)usb_input_buffer, "RPM: %d Speed: %.6f\n",
//								msg_buffer.data.shooting_data.bullet_freq,msg_buffer.data.shooting_data.bullet_speed );
//						CDC_Transmit_FS(usb_input_buffer, len);
//					}
//				}
//			}
//		}
		vTaskDelay(100);
	}
}
