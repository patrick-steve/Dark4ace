/*
 * movement_control_task.c
 *
 *  Created on: Jan 19, 2021
 *      Author: Hans Kurnia
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "motor_control.h"
#include "arm_math.h"
#include "movement_control_task.h"
#include "bsp_hall.h"

extern EventGroupHandle_t chassis_event_group;

extern chassis_control_t chassis_ctrl_data;

extern remote_cmd_t g_remote_cmd;
extern motor_data_t g_can_motors[24];
extern referee_limit_t g_referee_limiters;
extern ref_game_robot_data_t ref_robot_data;
extern uint32_t ref_power_data_txno;
float g_chassis_yaw = 0;
int32_t chassis_rpm = MAX_SPEED;
extern uint8_t g_gimbal_state;
extern int g_spinspin_mode;

float motor_yaw_mult[4];

extern QueueHandle_t telem_motor_queue;

void movement_control_task(void *argument) {
	TickType_t start_time;
	//initialise in an array so it's possible to for-loop it later
	motor_yaw_mult[0] = FR_YAW_MULT;
	motor_yaw_mult[1] = FL_YAW_MULT;
	motor_yaw_mult[2] = BL_YAW_MULT;
	motor_yaw_mult[3] = BR_YAW_MULT;

	while (1) {

#ifndef CHASSIS_MCU

		EventBits_t motor_bits;
		//wait for all motors to have updated data before PID is allowed to run
		motor_bits = xEventGroupWaitBits(chassis_event_group, 0b1111, pdTRUE,
		pdTRUE,
		portMAX_DELAY);
		if (motor_bits == 0b1111) {
			status_led(3, on_led);
			start_time = xTaskGetTickCount();
			if (chassis_ctrl_data.enabled) {

				chassis_motion_control(g_can_motors + FR_MOTOR_ID - 1,
						g_can_motors + FL_MOTOR_ID - 1,
						g_can_motors + BL_MOTOR_ID - 1,
						g_can_motors + BR_MOTOR_ID - 1);
			} else {
				g_can_motors[FR_MOTOR_ID - 1].output = 0;
				g_can_motors[FL_MOTOR_ID - 1].output = 0;
				g_can_motors[BL_MOTOR_ID - 1].output = 0;
				g_can_motors[BR_MOTOR_ID - 1].output = 0;

			}
#else
		chassis_MCU_send_CAN();
#endif
			status_led(3, off_led);
		} else {
			//motor timed out
			g_can_motors[FR_MOTOR_ID - 1].output = 0;
			g_can_motors[FL_MOTOR_ID - 1].output = 0;
			g_can_motors[BL_MOTOR_ID - 1].output = 0;
			g_can_motors[BR_MOTOR_ID - 1].output = 0;
		}
		//clear bits if it's not already cleared
		xEventGroupClearBits(chassis_event_group, 0b1111);
		//delays task for other tasks to run
		vTaskDelayUntil(&start_time, CHASSIS_DELAY);
	}
	osThreadTerminate(NULL);
}
void chassis_MCU_send_CAN() {

}
static uint32_t chassis_rpm_max = MAX_SPEED; //LV1_MAX_SPEED;

float filtered_rpm_fr;
float filtered_rpm_fl;
float filtered_rpm_bl;
float filtered_rpm_br;
float vforwardrpm;
float vyaw;
float vhorizontal;

void chassis_motion_control(motor_data_t *motorfr, motor_data_t *motorfl,
		motor_data_t *motorbl, motor_data_t *motorbr) {
	//get the angle between the gun and the chassis
	//so that movement is relative to gun, not chassis
	float rel_angle = g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang;
	float translation_rpm[4] = { 0, };
	float yaw_rpm[4] = { 0, };
	float total_power = 0;
	static int32_t chassis_current = CHASSIS_MAX_CURRENT;
	static int32_t prev_drive_mag;

	int32_t curr_avg_rpm = (abs(motorfr->raw_data.rpm)
			+ abs(motorfl->raw_data.rpm) + abs(motorbr->raw_data.rpm)
			+ abs(motorbl->raw_data.rpm)) / 4;

#ifdef REF_POWER_LIM
	static uint32_t prev_tx_no = 0;
	if (ref_power_data_txno != prev_tx_no) {
		prev_tx_no = ref_power_data_txno;
		float avg_power;
		avg_power = (float) curr_avg_rpm / prev_drive_mag; // estimate power based on wheel rpm change?
		avg_power = avg_power * avg_power;		//square the avg power
		float power_diff = fabs(
				avg_power - g_referee_limiters.wheel_power_limit); // difference between robot average power and % of game limit power used
		if (power_diff > 0.2) { // if power difference>0.2, decrease power. if power diff<0.2, increase power
			float power_err = (g_referee_limiters.wheel_power_limit - avg_power);
			power_err = power_err * CHASSIS_POWER_KP;
			if (power_err > CHASSIS_POWER_DELTA_LIM) { // limits max change to CHASSIS_POWER_DELTA_LIM
				power_err = CHASSIS_POWER_DELTA_LIM;
			} else if (power_err < -CHASSIS_POWER_DELTA_LIM) {
				power_err = -CHASSIS_POWER_DELTA_LIM;
			}
			uint32_t lvl_max_speed = LVL_MAX_SPEED;

			chassis_rpm_max = chassis_rpm_max - (chassis_rpm_max * power_err); // increase or decrease chassis_rpm_max by power err

			chassis_rpm_max =
					(chassis_rpm_max < MIN_SPEED) ? MIN_SPEED :
					(chassis_rpm_max > lvl_max_speed) ?
							lvl_max_speed : chassis_rpm_max;

		}
#ifdef CHASSIS_POWER_BUFFER_LIMITER
		if (g_spinspin_mode == 1) {  // for messing with motor current while spinning as needed
			chassis_current = CHASSIS_MAX_CURRENT* g_referee_limiters.wheel_buffer_limit; // multiply max motor pid output (effectively motor current) by multiplier from remaining buffer
		} else {
			chassis_current = CHASSIS_MAX_CURRENT* g_referee_limiters.wheel_buffer_limit;
		}
	#else
		chassis_current = CHASSIS_MAX_CURRENT;
#endif
	}
#endif

	uint32_t chassis_max_curr = chassis_current;

	chassis_rpm = (chassis_rpm_max > MAX_SPEED) ? MAX_SPEED : chassis_rpm_max; // limit chassis_rpm to max rpm of drive motors to avoid motor damage

	//rotate angle of the movement :)
	//MA1513/MA1508E is useful!!

	float rel_forward = ((-chassis_ctrl_data.horizontal * sin(-rel_angle)) //translation and rotation speed of chassis for chassis yaw angle relative to gimbal
	+ (chassis_ctrl_data.forward * cos(-rel_angle)));
	float rel_horizontal = ((-chassis_ctrl_data.horizontal * cos(-rel_angle))
			+ (chassis_ctrl_data.forward * -sin(-rel_angle)));
	float rel_yaw = chassis_ctrl_data.yaw;

	translation_rpm[0] = ((rel_forward * FR_VY_MULT) //calculate theoretical wheel rpm for chassis translation
	+ (rel_horizontal * FR_VX_MULT));
	translation_rpm[1] = ((rel_forward * FL_VY_MULT)
			+ (rel_horizontal * FL_VX_MULT));
	translation_rpm[2] = ((rel_forward * BL_VY_MULT)
			+ (rel_horizontal * BL_VX_MULT));
	translation_rpm[3] = ((rel_forward * BR_VY_MULT)
			+ (rel_horizontal * BR_VX_MULT));

	yaw_rpm[0] = rel_yaw * motor_yaw_mult[0] * CHASSIS_YAW_MAX_RPM; //calculate theoretical wheel rpm for yaw
	yaw_rpm[1] = rel_yaw * motor_yaw_mult[1] * CHASSIS_YAW_MAX_RPM;
	yaw_rpm[2] = rel_yaw * motor_yaw_mult[2] * CHASSIS_YAW_MAX_RPM;
	yaw_rpm[3] = rel_yaw * motor_yaw_mult[3] * CHASSIS_YAW_MAX_RPM;

	float rpm_mult = 1;
	float rpm_sum = 0;
	for (uint8_t i = 0; i < 4; i++) {
		float temp_add = fabs(yaw_rpm[i] + translation_rpm[i]);
		rpm_sum = rpm_sum + temp_add;  //get sum of wheel rpm
		if (temp_add > rpm_mult) {	   //get highest wheel rpm
			rpm_mult = temp_add;
		}
	}

	int32_t avg_trans = 0;
	for (uint8_t j = 0; j < 4; j++) {
		if (g_spinspin_mode == 1) { // if spinning
			translation_rpm[j] = (translation_rpm[j]// sum theoretical wheel rpm for translation and yaw
			+ yaw_rpm[j]) * chassis_rpm / (rpm_sum / 4); // for spinning modulate wheel rpm by dividing by average rpm
			avg_trans += fabs(translation_rpm[j]);
		} else {
			translation_rpm[j] = (translation_rpm[j]// sum theoretical wheel rpm for translation and yaw
			+ yaw_rpm[j]) * chassis_rpm / rpm_mult;	// for no spinning modulate wheel rpm by dividing by highest rpm
			avg_trans += fabs(translation_rpm[j]);
		}

	}
	prev_drive_mag = avg_trans / 4;

	motorfr->rpm_pid.max_out = chassis_max_curr;
	motorfl->rpm_pid.max_out = chassis_max_curr;
	motorbl->rpm_pid.max_out = chassis_max_curr;
	motorbr->rpm_pid.max_out = chassis_max_curr;

	speed_pid(translation_rpm[0], motorfr->raw_data.rpm, &motorfr->rpm_pid);
	total_power += fabs(motorfr->rpm_pid.output);
	speed_pid(translation_rpm[1], motorfl->raw_data.rpm, &motorfl->rpm_pid);
	total_power += fabs(motorfl->rpm_pid.output);
	speed_pid(translation_rpm[2], motorbl->raw_data.rpm, &motorbl->rpm_pid);
	total_power += fabs(motorbl->rpm_pid.output);
	speed_pid(translation_rpm[3], motorbr->raw_data.rpm, &motorbr->rpm_pid);
	total_power += fabs(motorbr->rpm_pid.output);

	motorfr->output = motorfr->rpm_pid.output;
	motorfl->output = motorfl->rpm_pid.output;
	motorbl->output = motorbl->rpm_pid.output;
	motorbr->output = motorbr->rpm_pid.output;
}

