/*
 * launcher_control_task.h
 *
 *  Created on: Jul 26, 2021
 *      Author: wx
 */

#ifndef TASKS_INC_LAUNCHER_CONTROL_TASK_H_
#define TASKS_INC_LAUNCHER_CONTROL_TASK_H_


uint16_t check_overheat();

void launcher_control_task(void *argument);
void flywheel_control(motor_data_t *l_flywheel, motor_data_t *r_flywheel);
void launcher_control(motor_data_t *l_flywheel, motor_data_t *r_flywheel,motor_data_t *feeder);
void launcher_angle_control(motor_data_t *l_flywheel, motor_data_t *r_flywheel,motor_data_t *feeder);

#endif /* TASKS_INC_LAUNCHER_CONTROL_TASK_H_ */
