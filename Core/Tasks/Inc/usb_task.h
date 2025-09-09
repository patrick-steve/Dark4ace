/*
 * usb_config_task.h
 *
 *  Created on: Dec 20, 2021
 *      Author: wx
 */

#ifndef TASKS_INC_USB_TASK_H_
#define TASKS_INC_USB_TASK_H_

void usb_vcp_processing(uint8_t* buffer, uint32_t *len);
void usb_task(void *argument);


#endif /* TASKS_INC_USB_TASK_H_ */
