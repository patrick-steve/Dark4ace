/*
 * control_sbc.h
 *
 *  Created on: 6 Jul 2023
 *      Author: wx
 */

#ifndef TASKS_INC_CONTROL_SBC_H_
#define TASKS_INC_CONTROL_SBC_H_

void sbc_control_input();
void aimbot_pid_init();
void sbc_gimbal_control_input(uint8_t timeout);
void sbc_gimbal_process_yolo();
void sbc_gimbal_process_norm();
void sbc_gimbal_process_set_ang();
void sbc_gimbal_process_turn_ang();
void sbc_chassis_control_input(uint8_t sbc_timeout);
void sbc_launcher_control_input(uint8_t timeout);

#endif /* TASKS_INC_CONTROL_SBC_H_ */
