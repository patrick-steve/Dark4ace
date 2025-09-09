/*
 * bsp_lk_motor.c
 *
 *  Created on: May 9, 2024
 *      Author: wx
 */
#include "board_lib.h"
#include "bsp_lk_motor.h"
#include "can_msg_processor.h"
#include <can.h>

void process_lk_motor(uint8_t *rx_buffer, motor_data_t *motor_data) {
	motor_data->last_time[1] = motor_data->last_time[0];
	motor_data->last_time[0] = get_microseconds();
	switch (rx_buffer[0]) {
	case 0x30: //read pid
	case 0x31: // write pid to ram
	case 0x32: //write pid to rom
		lk_process_pid(rx_buffer);
		break;
	case 0x33: //read acceleration
	case 0x90: //read encoder
		lk_process_encoder(rx_buffer, motor_data);
		break;
	case 0x91: //write encoder as zero
	case 0x19: //zero motor
	case 0x92: //read multi turn angle
		lk_process_motor_mangle(rx_buffer, motor_data);
		break;
	case 0x94: //read single turn angle
		lk_process_angle_control(rx_buffer, motor_data);
		break;
	case 0x95: //clear motor angles
	case 0x9A: //read motor status 1
	case 0x9B: //clear error status flags
		break;
	case 0x9C: //read motor status 2
		lk_process_status2(rx_buffer, motor_data);
		break;
	case 0x9D: //read motor status 3
		lk_process_status3(rx_buffer, motor_data);
		break;
	case 0x80: //close motor command
	case 0x81: //stop motor command
	case 0x88: //resume motor
		break;
	case 0xA3: //multi turn reply
		lk_process_angle_control(rx_buffer, motor_data);
		break;
	case 0xA4:
	case 0xA5:
	case 0xA6:
		break;
	default:
		break;
	}

}



void lk_write_pid(CAN_HandleTypeDef *can, motor_data_t *motor_data) {
	//0x31
	uint8_t data[8];
	data[0] = 0x31;
	data[1] = 0x00;
	uint8_t temp;
	if (motor_data->angle_pid.kp > 255) {
		temp = 255;
	} else {
		temp = motor_data->angle_pid.kp;
	}
	data[2] = temp;

	if (motor_data->angle_pid.ki > 255) {
		temp = 255;
	} else {
		temp = motor_data->angle_pid.ki;
	}
	data[3] = temp;

	if (motor_data->rpm_pid.kp > 255) {
		temp = 255;
	} else {
		temp = motor_data->rpm_pid.kp;
	}
	data[4] = temp;

	if (motor_data->rpm_pid.ki > 255) {
		temp = 255;
	} else {
		temp = motor_data->rpm_pid.ki;
	}

	data[5] = temp;
	//DEFAULT VALUES FOR MG5010E CHECK FOR MG6010 OR SMTH
	data[6] = 50;
	data[7] = 50;
	uint32_t tx_mailbox;
	while (HAL_CAN_GetTxMailboxesFreeLevel(can) == 0) {
	}
	can_send_msg(motor_data->can, motor_data->id, 8, data);
}

void lk_read_pid(CAN_HandleTypeDef *can, motor_data_t *motor_data) {
	uint8_t data[8];
	//0x30
	data[0] = 0x31;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;
	can_send_msg(motor_data->can, motor_data->id, 8, data);
}
void lk_read_status1(CAN_HandleTypeDef *can, motor_data_t *motor_data) {
	//0x9a
}

void lk_read_status2(motor_data_t *motor_data) {
	uint8_t data[8];
	//0x30
	data[0] = 0x9C;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;
	can_send_msg(motor_data->can, motor_data->id, 8, data);
	//0x9C
}
void lk_read_encoder(motor_data_t *motor_data) {
	uint8_t data[8];
	//0x30
	data[0] = 0x90;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;
	can_send_msg(motor_data->can, motor_data->id, 8, data);

}

void lk_motor_kill(motor_data_t* motor_data){
	uint8_t data[8];
	//0x30
	data[0] = 0x81;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;
	can_send_msg(motor_data->can, motor_data->id, 8, data);

}
void lk_read_motor_mang(motor_data_t *motor_data) {
	uint8_t data[8];
	//0x30
	data[0] = 0x92;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;
	can_send_msg(motor_data->can, motor_data->id, 8, data);
}
void lk_read_motor_sang(motor_data_t *motor_data) {
	uint8_t data[8];
	//0x30
	data[0] = 0x94;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;
	can_send_msg(motor_data->can, motor_data->id, 8, data);
}

void lk_motor_stop(motor_data_t *motor_data) {
	//0x81
}

void lk_motor_singleturn_ang(motor_data_t *motor_data) {
	uint8_t data[8];
	//0x30
	data[0] = 0xA6;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;
	can_send_msg(motor_data->can, motor_data->id, 8, data);
}

void lk_motor_multturn_ang(motor_data_t *motor_data) {
	uint8_t data[8];
	//0x30
	data[0] = 0xA4;
	data[1] = 0x00;
	uint16_t temp = fabs(motor_data->angle_pid.max_out);
	data[2] = (uint8_t) temp;
	data[3] = (uint8_t) (temp >> 8);
	int64_t target_ang = motor_data->output + motor_data->angle_data.center_ang;
//	if (target_ang < 0){
//		target_ang = 360000 + target_ang;
//	}
	data[4] = (uint8_t) (target_ang & 0xFF);
	data[5] = (uint8_t) ((target_ang >> 8) & 0xFF);
	data[6] = (uint8_t) ((target_ang >> 16) & 0xFF);
	data[7] = (uint8_t) ((target_ang >> 24) & 0xFF);
	can_send_msg(motor_data->can, motor_data->id, 8, data);
}

static pid_data_t g_angle_pid;
static pid_data_t g_speed_pid;
static pid_data_t g_torque_pid;

void lk_process_pid(uint8_t *data) {
	//for testing only
	g_angle_pid.kp = data[2];
	g_angle_pid.ki = data[3];
	g_speed_pid.kp = data[4];
	g_speed_pid.ki = data[5];
	g_torque_pid.kp = data[6];
	g_torque_pid.ki = data[7];
}

void lk_process_encoder(uint8_t *data, motor_data_t *motor_data) {
	//check for all encoders or just some encoders
	motor_data->raw_data.angle[0] = data[2] | data[3] << 8;
	motor_data->raw_data.angle[0] = data[6] | (data[7] << 8);
	motor_data->raw_data.raw_encoder = data[4] | data[5] << 8;
	motor_data->raw_data.encoder_offset = data[6] | data[7] << 8;
}

void lk_process_status1(uint8_t *data, motor_data_t *motor_data) {
	motor_data->raw_data.temp = data[1];
	//voltage is data[3] and data[4] but no use
}

void lk_process_status2(uint8_t *data, motor_data_t *motor_data) {
	motor_data->raw_data.temp = data[1];
	motor_data->raw_data.torque = (int16_t) data[2] | ((int16_t) data[3] << 8);
	//raw data is in dps, convert to rpm
	motor_data->raw_data.rpm = ((int16_t) data[4] | ((int16_t) data[5] << 8))
			/ (21600);
	motor_data->raw_data.angle[1] = motor_data->raw_data.angle[0];
	motor_data->raw_data.angle[0] = (uint16_t) data[6]
			| ((uint16_t) data[8] << 8);

}

void lk_process_status3(uint8_t *data, motor_data_t *motor_data) {
	motor_data->raw_data.temp = data[1];
	//we don't do foc math here :<

}

void lk_calc_ang(motor_data_t *motor_data){
	motor_data->angle_data.ticks = motor_data->raw_data.angle[0];
}

void lk_process_angle_control(uint8_t *data, motor_data_t *motor_data) {
	int32_t angle = data[4] + (data[5] << 8) + (data[6] << 16)
			+ (data[7] << 24);
	motor_data->raw_data.angle[1] = motor_data->raw_data.angle[0];
	//range is supposed to be from 0 to 36000... which fits in int16 tho
	motor_data->raw_data.angle[0] = angle;
	lk_calc_ang(motor_data);
//	motor_calc_odometry(&motor_data->raw_data, &motor_data->angle_data,
//			motor_data->last_time);
	angle_offset(&motor_data->raw_data, &motor_data->angle_data);
	//add adj ang calculations
}

void lk_process_motor_mangle(uint8_t *data, motor_data_t *motor_data) {
//	motor_data->raw_data.angle[1] = motor_data->raw_data.angle[0];
	motor_data->raw_data.raw_motor_angle = (((int64_t) data[7]) << 48)
			| ((int64_t) data[6] << 40) | ((int64_t) data[5] << 32)
			| ((int64_t) data[4] << 24) | ((int64_t) data[3] << 16)
			| ((int64_t) data[2] << 8) | (int64_t) data[1];
}

void lk_update_encoder() {

}
