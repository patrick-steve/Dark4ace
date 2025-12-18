/*
 * gimbal_control_task.c
 *
 *  Created on: Jan 1, 2022
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_control.h"
#include "motor_config.h"
#include "can_msg_processor.h"
#include "gimbal_control_task.h"
#include "bsp_lk_motor.h"

extern uint8_t aimbot_mode;

extern EventGroupHandle_t gimbal_event_group;
extern float g_chassis_yaw;
extern motor_data_t g_can_motors[24];
extern motor_data_t g_pitch_motor;
extern gimbal_control_t gimbal_ctrl_data;
extern orientation_data_t imu_heading;
extern QueueHandle_t telem_motor_queue;
extern chassis_control_t chassis_ctrl_data;
extern int32_t chassis_rpm;
static float rel_pitch_angle;
uint8_t g_gimbal_state = 0;

static float prev_pit;
static float prev_yaw;


extern int g_spinspin_mode;
#ifdef YAW_FEEDFORWARD
static pid_data_t g_yaw_ff_pid = {
			.kp = YAW_FF_SPD_KP,
			.ki = YAW_FF_SPD_KI,
			.kd = YAW_FF_SPD_KD,
			.int_max = YAW_FF_INT_MAX,
			.max_out = YAW_FF_MAX_OUTPUT
};
#endif
static float g_chassis_rot;
float curr_rot;

uint8_t check_yaw(){
	if (get_microseconds()- g_can_motors[YAW_MOTOR_ID-1].last_time[0] < 1000){
		return 1;
	} else {
		return 0;
	}
}

/**
 *
 * FreeRTOS task for gimbal controls
 * Has HIGH2 priority
 *
 */
void gimbal_control_task(void *argument) {
	TickType_t start_time;
	while (1) {
		xEventGroupWaitBits(gimbal_event_group, 0b11, pdTRUE, pdFALSE,
		portMAX_DELAY);
		start_time = xTaskGetTickCount();
		if (gimbal_ctrl_data.enabled) {
			if (gimbal_ctrl_data.imu_mode) {
				gimbal_control(&g_pitch_motor,
						g_can_motors + YAW_MOTOR_ID - 1);
			} else {
				gimbal_angle_control(&g_pitch_motor,
						g_can_motors + YAW_MOTOR_ID - 1);
			}
		} else {
			g_pitch_motor.output = 0;
			g_can_motors[YAW_MOTOR_ID - 1].output = 0;
		}
		prev_yaw = imu_heading.yaw;
		status_led(2, off_led);
		xEventGroupClearBits(gimbal_event_group, 0b11);
		vTaskDelayUntil(&start_time, GIMBAL_DELAY);
	}
	//should not run here
}

/**
 * This function controls the gimbals based on IMU reading
 * @param 	pitch_motor		Pointer to pitch motor struct
 * 			yaw_motor		Pointer to yaw motor struct
 * @note both pitch and yaw are currently on CAN2 with ID5 and 6.
 * Need to check if having ID4 (i.e. 0x208) + having the launcher motors (ID 1-3, 0x201 to 0x203)
 * still provides a fast enough response
 */
uint8_t pit_lim;
void gimbal_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor) {
	pit_lim = 0;
	uint8_t yaw_lim = 0;
	if (prev_yaw == imu_heading.yaw || prev_pit == imu_heading.pit) {
		return;}

	rel_pitch_angle = pitch_motor->angle_data.adj_ang
				+ gimbal_ctrl_data.pitch - imu_heading.pit;
	if (rel_pitch_angle > pitch_motor->angle_data.phy_max_ang) {
		rel_pitch_angle = pitch_motor->angle_data.phy_max_ang;
		pit_lim = 1;
	}
	if (rel_pitch_angle < pitch_motor->angle_data.phy_min_ang) {
		rel_pitch_angle = pitch_motor->angle_data.phy_min_ang;
		pit_lim = 1;
	}
	if (pit_lim == 1) {
		gimbal_ctrl_data.pitch = rel_pitch_angle + imu_heading.pit
				- (pitch_motor->angle_data.adj_ang);
	}

	yangle_pid(gimbal_ctrl_data.pitch,imu_heading.pit, pitch_motor,
			imu_heading.pit, &prev_pit,1);
//	angle_pid(gimbal_ctrl_data.pitch,imu_heading.pit, pitch_motor);

	int32_t temp_pit_output = pitch_motor->rpm_pid.output + PITCH_CONST;

	temp_pit_output = (temp_pit_output < -20000) ? -20000 :
						(temp_pit_output > 20000) ? 20000 : temp_pit_output;

	pitch_motor->output = temp_pit_output;

	float rel_yaw_angle = yaw_motor->angle_data.adj_ang + gimbal_ctrl_data.yaw
			- imu_heading.yaw;

	//if yaw has overflowed (i.e. goes to the next round) move it back into pi to -pi range
	if (rel_yaw_angle > PI) {
		rel_yaw_angle -= 2 * PI;
	}
	if (rel_yaw_angle < -PI) {
		rel_yaw_angle += 2 * PI;
	}
	//check limits
	if (rel_yaw_angle > yaw_motor->angle_data.phy_max_ang) {
		rel_yaw_angle = yaw_motor->angle_data.phy_max_ang;
		yaw_lim = 1;
	}
	if (rel_yaw_angle < yaw_motor->angle_data.phy_min_ang) {
		rel_yaw_angle = yaw_motor->angle_data.phy_min_ang;
		yaw_lim = 1;
	}
	if (yaw_lim == 1) {
		gimbal_ctrl_data.yaw = rel_yaw_angle + imu_heading.yaw
				- yaw_motor->angle_data.adj_ang;
	}
	float turn_ang = imu_heading.yaw - prev_yaw;
	if (turn_ang > PI){
		turn_ang -= 2 * PI;
	} else if (turn_ang < -PI){
		turn_ang += 2 * PI;

	}
	xSemaphoreTake(gimbal_ctrl_data.yaw_semaphore,portMAX_DELAY);
	gimbal_ctrl_data.delta_yaw -= turn_ang;
	yangle_pid(gimbal_ctrl_data.delta_yaw, 0, yaw_motor,
			imu_heading.yaw, &prev_yaw,0);
	xSemaphoreGive(gimbal_ctrl_data.yaw_semaphore);

	int32_t temp_output = yaw_motor->rpm_pid.output  + (chassis_ctrl_data.yaw * YAW_SPINSPIN_CONSTANT);

	temp_output = (temp_output > 20000) ? 20000 : (temp_output < -20000) ? -20000 : temp_output;
	yaw_motor->output = temp_output;
#ifdef YAW_FEEDFORWARD
//	speed_pid(yaw_motor->raw_data.rpm + yaw_motor->angle_pid.output, g_chassis_rot, &g_yaw_ff_pid);
//	yaw_motor->output += g_yaw_ff_pid.output;
//	yaw_motor->output = (yaw_motor->output < - 20000) ? -20000 : (yaw_motor->output > 20000) ? 20000:yaw_motor->output;
#endif

}

int test = 0;
void gimbal_angle_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor) {

	if (gimbal_ctrl_data.pitch > pitch_motor->angle_data.max_ang) {
		gimbal_ctrl_data.pitch = pitch_motor->angle_data.max_ang;
	}
	if (gimbal_ctrl_data.pitch < pitch_motor->angle_data.min_ang) {
		gimbal_ctrl_data.pitch = pitch_motor->angle_data.min_ang;
	}

	if (gimbal_ctrl_data.yaw > yaw_motor->angle_data.max_ang) {
		gimbal_ctrl_data.yaw = yaw_motor->angle_data.max_ang;
	}
	if (gimbal_ctrl_data.yaw < yaw_motor->angle_data.min_ang) {
		gimbal_ctrl_data.yaw = yaw_motor->angle_data.min_ang;
	}
	angle_pid(gimbal_ctrl_data.pitch, pitch_motor->angle_data.adj_ang,
			pitch_motor);
	angle_pid(gimbal_ctrl_data.yaw, yaw_motor->angle_data.adj_ang, yaw_motor);

	pitch_motor->output = pitch_motor->rpm_pid.output;
	yaw_motor->output = yaw_motor->rpm_pid.output;
	test = 1;
}
