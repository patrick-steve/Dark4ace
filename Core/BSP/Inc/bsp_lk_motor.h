/*
 * bsp_lk_motor.h
 *
 *  Created on: May 10, 2024
 *      Author: wx
 */

#ifndef TASKS_INC_BSP_LK_MOTOR_H_
#define TASKS_INC_BSP_LK_MOTOR_H_

void process_lk_motor(uint8_t* rx_buffer, motor_data_t *motor_data);
void lk_write_pid(CAN_HandleTypeDef *can, motor_data_t *motor_data);
void lk_read_pid(CAN_HandleTypeDef *can, motor_data_t* motor_data);
void lk_read_status1(CAN_HandleTypeDef *can, motor_data_t* motor_data);
void lk_read_status2(motor_data_t* motor_data);
void lk_read_encoder(motor_data_t* motor_data);
void lk_read_motor_mang(motor_data_t* motor_data);
void lk_read_motor_sang(motor_data_t* motor_data);
void lk_motor_stop(motor_data_t* motor_data);
void lk_motor_kill(motor_data_t* motor_data);

void lk_motor_singleturn_ang(motor_data_t* motor_data);
void lk_motor_multturn_ang(motor_data_t* motor_data);

void lk_process_pid(uint8_t* data);
void lk_process_encoder(uint8_t* data, motor_data_t* motor_data);

void lk_process_status1(uint8_t* data, motor_data_t* motor_data);

void lk_process_status2(uint8_t* data, motor_data_t* motor_data);
void lk_process_status3(uint8_t* data, motor_data_t* motor_data);
void lk_process_encoder(uint8_t* data, motor_data_t* motor_data);
void lk_process_angle_control(uint8_t* data, motor_data_t* motor_data);
void lk_process_motor_mangle(uint8_t* data, motor_data_t* motor_data);
void lk_calc_ang(motor_data_t *motor_data);
void lk_update_encoder();



#endif /* TASKS_INC_BSP_LK_MOTOR_H_ */
