/*
 * hud_new.h
 *
 *  Created on: Dec 2, 2024
 *      Author: cw
 */

#ifndef TASKS_INC_HUD_NEW_H_
#define TASKS_INC_HUD_NEW_H_

void map_robot_id(uint16_t robot_id);
void hud_task(void *argument);
void set_top_coordinates();

void draw_dynamic(uint8_t modify);
void draw_char(uint8_t modify);
void draw_static();
void clear_hud();

uint16_t draw_graphic_header(uint8_t* tx_buffer, uint8_t num_graphics);
uint16_t draw_char_header(uint8_t* tx_buffer, uint8_t char_len);
void ref_send(uint8_t* tx_buffer, uint16_t tx_len);

uint16_t draw_empty(uint8_t* tx_buffer); // Packet with no operation
void draw_spin_char(uint8_t modify, uint32_t x_coords); // Char for spinspin
uint16_t draw_spin_border(uint8_t* tx_buffer, uint8_t modify, uint32_t x_coords); // Border for spinspin
void draw_gearing(uint8_t modify, uint32_t x_coords);
uint16_t draw_supercap(uint8_t* tx_buffer, uint8_t modify);
void draw_aimbot(uint8_t modify, uint32_t x_coords);
void draw_crosshair(uint8_t modify);

void draw_pitch_graphics(uint8_t modify); // Function to call all static pitch graphics
void draw_major_ticks(uint8_t modify);
void draw_minor_ticks(uint8_t modify);
void draw_pitch_labels(uint8_t modify);
void draw_pitch_limits(uint8_t modify); // Draw the max / min angle allowed by software
uint16_t draw_curr_pitch(uint8_t* tx_buffer, uint8_t modify);

void motor_fault();
void delete_motor_fault();
void draw_motor_fault(uint8_t modify); // Char to list all disconnected motors

void draw_feeder_state(uint8_t modify);
void delete_feeder_state();
void dfeeder_state();

uint16_t draw_balancing_status(uint8_t* tx_buffer, uint8_t modify);
#endif /* TASKS_INC_HUD_NEW_H_ */
