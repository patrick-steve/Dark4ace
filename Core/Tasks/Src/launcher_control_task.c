/*
 * launcher_control_task.c
 *
 *  Created on: Jul 26, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "motor_control.h"
#include "launcher_control_task.h"


extern EventGroupHandle_t launcher_event_group;

extern ref_game_state_t ref_game_state;
extern motor_data_t g_can_motors[24];
extern gun_control_t launcher_ctrl_data;

extern remote_cmd_t g_remote_cmd;
extern referee_limit_t g_referee_limiters;

//static float friction_offset = FRICTION_OFFSET;

extern QueueHandle_t telem_motor_queue;

#define BULLET_17_HEAT 10
#define BULLET_42_HEAT 100

#ifdef BULLET_17
#define BULLET_ACTUAL_HEAT BULLET_17_HEAT
#endif

#ifdef BULLET_42
#define BULLET_ACTUAL_HEAT BULLET_42_HEAT
#endif

extern ref_game_robot_data_t ref_robot_data;
extern ref_robot_power_data_t ref_power_data;

extern uint32_t ref_power_data_txno;
extern ref_magazine_data_t ref_mag_data;
extern uint32_t ref_mag_data_txno;
static uint32_t prev_power_data_no = 0;


enum feeder_state_e feeder_state;

void launcher_control_task(void *argument) {
	TickType_t launcher_ctrl_time;
	while (1) {
		//event flags!
		xEventGroupWaitBits(launcher_event_group, 0b111, pdTRUE, pdTRUE,
		portMAX_DELAY);
		status_led(4, on_led);
		launcher_ctrl_time = xTaskGetTickCount();

		if (launcher_ctrl_data.enabled) {
			flywheel_control(g_can_motors + LFRICTION_MOTOR_ID - 1,
					g_can_motors + RFRICTION_MOTOR_ID - 1);
#ifdef ANGLE_FEEDER
			launcher_angle_control(g_can_motors + LFRICTION_MOTOR_ID - 1,
					g_can_motors + RFRICTION_MOTOR_ID - 1,
					g_can_motors + FEEDER_MOTOR_ID - 1);

#else
			launcher_control(g_can_motors + LFRICTION_MOTOR_ID - 1,
					g_can_motors + RFRICTION_MOTOR_ID - 1,
					g_can_motors + FEEDER_MOTOR_ID - 1);
#endif

		} else {
			g_can_motors[LFRICTION_MOTOR_ID - 1].output = 0;
			g_can_motors[RFRICTION_MOTOR_ID - 1].output = 0;
			g_can_motors[FEEDER_MOTOR_ID - 1].output = 0;
		}
		status_led(4, off_led);
		//vTaskDelay(CHASSIS_DELAY);
		xEventGroupClearBits(launcher_event_group, 0b111);
		vTaskDelayUntil(&launcher_ctrl_time, CHASSIS_DELAY);
	}

}

uint16_t check_overheat() {
#ifdef OVERHEAT_PROTECTION

	uint8_t active_feeder;
	int32_t ammo_remaining;
	static uint32_t last_time;
	if (ref_robot_data.robot_id == 0) {
		//referee system not connected
		return 10;
	}

#ifdef BULLET_17
	//if double barrel launcher, check launcher with more heat only
	if (ref_power_data.shooter_17mm_1_barrel_heat >= ref_power_data.shooter_17mm_2_barrel_heat) {
		active_feeder = 0;
	} else {
		active_feeder = 1;
	}

	if (active_feeder == 0) {
		ammo_remaining = (((ref_robot_data.shooter_barrel_heat_limit
				- ref_power_data.shooter_17mm_1_barrel_heat - OVERHEAT_OFFSET)) / BULLET_17_HEAT);
	} else if (active_feeder == 1) {
		ammo_remaining = (((ref_robot_data.shooter_barrel_heat_limit
				- ref_power_data.shooter_17mm_2_barrel_heat - OVERHEAT_OFFSET)) / BULLET_17_HEAT);
	}

	if (prev_power_data_no != ref_power_data_txno) {
		prev_power_data_no = ref_power_data_txno;
		last_time = get_microseconds();
		if (ammo_remaining < OVERHEAT_MARGIN) {
			return 0;
		} else {
			return ammo_remaining;
		}
	} else {
		//no updated heat information, guessing ammo remaining;
		uint32_t time_diff = get_microseconds() - last_time;
		if (active_feeder == 0) {
			ammo_remaining += (ref_robot_data.shooter_barrel_cooling_value
					* time_diff / TIMER_FREQ);
			ammo_remaining -= FEEDER_SPEED * time_diff
					/ (TIMER_FREQ * 60);
		} else if (active_feeder == 1) {

			ammo_remaining += (ref_robot_data.shooter_barrel_cooling_value
					* time_diff / TIMER_FREQ);
			ammo_remaining -= FEEDER_SPEED * time_diff
					/ (TIMER_FREQ * 60);
		}
		if (ammo_remaining < OVERHEAT_MARGIN) {
			return 0;
		} else {
			return ammo_remaining;
		}
		return ammo_remaining;
	}
#endif

	return ammo_remaining;

#else
	return 100;
#endif
}

void flywheel_control(motor_data_t *l_flywheel, motor_data_t *r_flywheel) {

	enum launcher_state_e {
		WHEEL_FIRING, WHEEL_STANDBY, WHEEL_CLEARING
	};
	static enum launcher_state_e flywheel_state;
	int16_t friction_wheel_speed = g_referee_limiters.projectile_speed
			* PROJECTILE_SPEED_RATIO;
	static uint32_t clear_time = 0;

	/**
	 * Finite state machine for flywheel
	 */
	switch (flywheel_state) {
	case WHEEL_STANDBY:
		if (launcher_ctrl_data.override == 1
				|| launcher_ctrl_data.firing != 0) {
			flywheel_state = WHEEL_FIRING;
		}
		break;

	case WHEEL_FIRING:
		if (launcher_ctrl_data.firing == 0
				&& launcher_ctrl_data.override == 0) {
			flywheel_state = WHEEL_CLEARING;
			clear_time = HAL_GetTick();
		}
		break;

	case WHEEL_CLEARING:
		if (launcher_ctrl_data.firing != 0 ) {
			flywheel_state = WHEEL_FIRING;
		} else if (HAL_GetTick() - clear_time > CLEAR_DELAY && feeder_state == FEEDER_STANDBY) {
			flywheel_state = WHEEL_STANDBY;
		}
		break;

	default:
		flywheel_state = WHEEL_STANDBY;
	}

	switch (flywheel_state) {
	case WHEEL_STANDBY:
		if (FRICTION_SB_SPIN_ON == 2 || (FRICTION_SB_SPIN_ON == 1 && ref_game_state.game_progress == 4)){
			speed_pid( FRICTION_SB_SPIN * FRICTION_INVERT,
					l_flywheel->raw_data.rpm, &l_flywheel->rpm_pid);
			speed_pid(-FRICTION_SB_SPIN * FRICTION_INVERT,
					r_flywheel->raw_data.rpm, &r_flywheel->rpm_pid);
			l_flywheel->output = l_flywheel->rpm_pid.output + FRICTION_OFFSET * FRICTION_INVERT;
			r_flywheel->output = r_flywheel->rpm_pid.output - FRICTION_OFFSET * FRICTION_INVERT;
		} else {
			l_flywheel->output = 0;
			r_flywheel->output = 0;

		}
		break;

	case WHEEL_CLEARING:
	case WHEEL_FIRING:
		speed_pid(friction_wheel_speed * FRICTION_INVERT,
				l_flywheel->raw_data.rpm, &l_flywheel->rpm_pid);
		speed_pid(-friction_wheel_speed * FRICTION_INVERT,
				r_flywheel->raw_data.rpm, &r_flywheel->rpm_pid);
		l_flywheel->output = l_flywheel->rpm_pid.output;
		r_flywheel->output = r_flywheel->rpm_pid.output;
		break;

	default:
		l_flywheel->output = 0;
		r_flywheel->output = 0;
		break;
	}

}

#ifndef ANGLE_FEEDER

void launcher_control(motor_data_t *l_flywheel, motor_data_t *r_flywheel,
		motor_data_t *feeder) {

	static uint32_t jam_start_time = 0;

	int16_t feeder_speed = launcher_ctrl_data.firing
			* g_referee_limiters.feeding_speed * FEEDER_INVERT
			/ FEEDER_SPEED_RATIO;
	int16_t friction_wheel_speed = g_referee_limiters.projectile_speed
			* PROJECTILE_SPEED_RATIO;

	int16_t rpm_diff = abs(l_flywheel->raw_data.rpm + r_flywheel->raw_data.rpm);
	int16_t avg_rpm = abs(l_flywheel->raw_data.rpm - r_flywheel->raw_data.rpm)
			/ 2;

	/**
	 * Finite state machine for feeder
	 */
	switch (feeder_state) {
	case FEEDER_STANDBY:
		if (launcher_ctrl_data.firing != 0) {
			feeder_state = FEEDER_SPINUP;
		}
		break;
	case FEEDER_SPINUP:
		if (abs(avg_rpm - friction_wheel_speed) < LAUNCHER_MARGIN) {
			if (rpm_diff < LAUNCHER_DIFF_MARGIN) {
				feeder_state = FEEDER_FIRING;
			}
		}
		break;

	case FEEDER_FIRING:
		//check for feeder jam first, prioritise unjamming
		if ((feeder->raw_data.torque)
				> (FEEDER_JAM_TORQUE * FEEDER_INVERT) && (abs(feeder->raw_data.rpm) < FEEDER_JAM_RPM)) {
			jam_start_time = HAL_GetTick();
			feeder->rpm_pid.integral = 0;
			feeder_state = FEEDER_JAM;
			break;
		}

		if (launcher_ctrl_data.firing == 0) {
			feeder_state = FEEDER_STANDBY;
			break;
		}

		if (check_overheat() == 0) {
			feeder_state = FEEDER_OVERHEAT;
			break;
		}
		//one side will always be negative

		if (abs(avg_rpm - friction_wheel_speed) > LAUNCHER_MARGIN) {
			if (rpm_diff > LAUNCHER_DIFF_MARGIN) {
				feeder_state = FEEDER_SPINUP;
				break;
			}
		}
		//else stay firing
		break;

	case FEEDER_JAM:
		//check if either after unjam time
		if ((HAL_GetTick() - jam_start_time) > FEEDER_UNJAM_TIME) {
			feeder_state = FEEDER_SPINUP;
		}

		if ((feeder->raw_data.torque * FEEDER_INVERT)
				< -(FEEDER_JAM_TORQUE * FEEDER_INVERT)) {
			feeder_state = FEEDER_SPINUP;
		}
		break;

	case FEEDER_OVERHEAT:
		if (check_overheat() > OVERHEAT_EXCESS) {
			if (launcher_ctrl_data.firing != 0) {
				feeder_state = FEEDER_SPINUP;
			} else {
				feeder_state = FEEDER_STANDBY;
			}
		}
		break;

	default:
		feeder_state = FEEDER_STANDBY;
	}

	switch (feeder_state) {
	case FEEDER_STANDBY:
	case FEEDER_SPINUP:
	case FEEDER_OVERHEAT:
		speed_pid(0, feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
//		feeder->output = 0;
		break;

	case FEEDER_FIRING:
		speed_pid(feeder_speed * feeder->angle_data.gearbox_ratio,
				feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
		break;

	case FEEDER_JAM:
		speed_pid(
				FEEDER_UNJAM_SPD * feeder->angle_data.gearbox_ratio
						* FEEDER_INVERT, feeder->raw_data.rpm,
				&feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
		break;

	default:
		feeder->output = 0;
	}

}
#else

void launcher_angle_control(motor_data_t *l_flywheel, motor_data_t *r_flywheel,
		motor_data_t *feeder) {
	static uint32_t jam_start_time = 0;
	int16_t friction_wheel_speed = g_referee_limiters.projectile_speed
			* PROJECTILE_SPEED_RATIO;
	static uint32_t last_fire;
	static float target_ang = 0;

	int16_t rpm_diff = abs(l_flywheel->raw_data.rpm + r_flywheel->raw_data.rpm);
	int16_t avg_rpm = abs(l_flywheel->raw_data.rpm - r_flywheel->raw_data.rpm)
			/ 2;
	uint32_t curr_time = HAL_GetTick();
	float ang_diff;
	static uint8_t fired = 0;
	ang_diff = target_ang - feeder->angle_data.adj_ang;
	/**
	 * Finite state machine for feeder
	 */
	switch (feeder_state) {
	case FEEDER_STANDBY:
		if (launcher_ctrl_data.firing != 0) {
			if (check_overheat() == 0) {
				feeder_state = FEEDER_OVERHEAT;
				break;
			}

			if (curr_time - last_fire < ANGLE_FEEDER_DELAY) {
				last_fire = curr_time;
				break;
			}
			last_fire = curr_time;
			feeder_state = FEEDER_SPINUP;
		}
		break;
	case FEEDER_SPINUP:
		if (abs(avg_rpm - friction_wheel_speed) < LAUNCHER_MARGIN) {
			if (rpm_diff < LAUNCHER_DIFF_MARGIN) {
				if (fired == 0){
					target_ang = feeder->angle_data.adj_ang
							+ ((2 * PI / FEEDER_SPEED_RATIO) * FEEDER_INVERT);
					if (target_ang > PI) {
						target_ang -= 2 * PI;
					} else if (target_ang < -PI) {
						target_ang += 2 * PI;
					}
					fired = 1;
				}
				feeder_state = FEEDER_STEP;
			}
		}
		break;

	case FEEDER_STEP:
		//check for feeder jam first, prioritise unjamming
		if ((feeder->raw_data.torque) > (FEEDER_JAM_TORQUE * FEEDER_INVERT)) {
			jam_start_time = HAL_GetTick();
			feeder->angle_pid.integral = 0;
			feeder->rpm_pid.integral = 0;
			feeder_state = FEEDER_JAM;
			break;
		}
		ang_diff = target_ang - feeder->angle_data.adj_ang;
		if (ang_diff > PI) {
			ang_diff -= 2 * PI;
		} else if (ang_diff < -PI) {
			ang_diff += 2 * PI;
		}

		if (((fabs(ang_diff) < ANGLE_FEEDER_MARGIN
				&& fabs(feeder->raw_data.rpm / feeder->angle_data.gearbox_ratio)
						< ANGLE_FEEDER_SPD_MARGIN))
				|| (curr_time - last_fire) > ANGLE_FEEDER_TIMEOUT) {
			fired = 0;
			feeder->angle_pid.integral = 0;
			feeder->rpm_pid.integral = 0;
			feeder_state = FEEDER_STANDBY;
			last_fire = curr_time;
			break;
		}

		//don't need to check for flywheel, it'll just mess it up
//			if (abs(avg_rpm - friction_wheel_speed) > LAUNCHER_MARGIN) {
//				if (rpm_diff > LAUNCHER_DIFF_MARGIN) {
//					feeder_state = FEEDER_SPINUP;
//					break;
//				}
//			}
		//else stay firing
		break;

	case FEEDER_JAM:
		//check if either after unjam time
		if ((HAL_GetTick() - jam_start_time) > FEEDER_UNJAM_TIME) {
			feeder_state = FEEDER_SPINUP;
		}

		if ((feeder->raw_data.torque * FEEDER_INVERT)
				< -(FEEDER_JAM_TORQUE * FEEDER_INVERT)) {
			feeder_state = FEEDER_SPINUP;
		}
		break;

	case FEEDER_OVERHEAT:
		if (check_overheat() >= OVERHEAT_EXCESS) {
			feeder->angle_pid.integral = 0;
			feeder->rpm_pid.integral = 0;
			feeder_state = FEEDER_STANDBY;
		}

		break;

	default:
		feeder->angle_pid.integral = 0;
		feeder->rpm_pid.integral = 0;
		feeder_state = FEEDER_STANDBY;
		break;
	}


	ang_diff = target_ang - feeder->angle_data.adj_ang;
	if (ang_diff > PI) {
		ang_diff -= 2 * PI;
	} else if (ang_diff < -PI) {
		ang_diff += 2 * PI;
	}

	switch (feeder_state) {
	case FEEDER_STANDBY:
	case FEEDER_SPINUP:
	case FEEDER_OVERHEAT:

				speed_pid(0, feeder->raw_data.rpm, &feeder->rpm_pid);
				feeder->output = feeder->rpm_pid.output;
//		speed_pid(ang_diff, 0, &feeder->angle_pid);
//		speed_pid(feeder->angle_pid.output * feeder->angle_data.gearbox_ratio,
//				feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
//		feeder->output = 0;
		break;

	case FEEDER_STEP:
		speed_pid(ang_diff, 0, &feeder->angle_pid);
		speed_pid(feeder->angle_pid.output * feeder->angle_data.gearbox_ratio,
				feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
		break;

	case FEEDER_JAM:
		speed_pid(FEEDER_UNJAM_SPD * feeder->angle_data.gearbox_ratio,
				feeder->raw_data.rpm, &feeder->rpm_pid);
		feeder->output = feeder->rpm_pid.output;
		break;

	default:
		feeder->output = 0;
	}
}
#endif
