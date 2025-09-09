/*
 * motor_config.h
 *
 *  Created on: 21 Dec 2021
 *      Author: wx
 */

#ifndef TASKS_INC_MOTOR_CONFIG_H_
#define TASKS_INC_MOTOR_CONFIG_H_


void config_motors();
void motor_calib_task(void* argument);


#define GM6020_MAX_OUTPUT 	20000
#define GM6020_MAX_RPM		400

#define M2006_MAX_RPM		15000
#define M2006_MAX_OUTPUT 	16384
#define M2006_GEARBOX_RATIO	36

#define M3508_MAX_OUTPUT 	16384
#define M3508_MAX_RPM		9000
#define M3508_GEARBOX_RATIO	(3591.0f / 187.0f)

#define LK_MG5010E_MAX_RPM 	3200


#define	TYPE_GM6020 		1
#define	TYPE_M2006 			2
#define	TYPE_M3508 			3
#define	TYPE_M3508_NGEARBOX 4
#define	TYPE_M3508_STEPS 	5
#define	TYPE_M2006_STEPS 	6
#define	TYPE_M2006_ANGLE 	7
#define	TYPE_M3508_ANGLE 	8
#define	TYPE_GM6020_720		9
#define	TYPE_LK_MG5010E_SPD		10
#define	TYPE_LK_MG5010E_ANG		11
#define	TYPE_LK_MG5010E_MULTI_ANG		12

void motor_calib_task(void *argument);
void set_motor_config(motor_data_t *motor);
void config_motors();
void bz_buzzer(uint8_t high, uint8_t low);
void motor_temp_bz(uint8_t hi, uint8_t low);
uint16_t check_motors();
uint8_t lk_set_pid(motor_data_t *motor, uint32_t timeout);

#endif /* TASKS_INC_MOTOR_CONFIG_H_ */
