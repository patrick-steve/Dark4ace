/*
 * hud_new.c
 *
 *  Created on: Dec 2, 2024
 *      Author: cw
 */


#include "board_lib.h"
#include "bsp_queue.h"
#include "bsp_referee.h"
#include "bsp_usart.h"
#include "hud_task.h"
#include "referee_msgs.h"
#include "robot_config.h"
#include "rtos_g_vars.h"
#include "hud_constants.h"
#include "typedefs.h"
#include "arm_math.h"

static uint16_t g_client_id = 0;
extern ref_game_robot_data_t ref_robot_data;
extern uint8_t g_ref_tx_seq;

extern motor_data_t g_can_motors[24];

extern int g_spinspin_mode;
int prev_spinspin = 0;

static uint32_t spin_coords = 0;

extern enum feeder_state_e feeder_state;
int prev_feeder_state;
extern float rel_pitch_angle;
int feeder_state_enabled = 0;

extern gimbal_control_t gimbal_ctrl_data;
extern float rel_pitch_angle;
extern motor_data_t g_pitch_motor;

int top_graphics = 0;
int dynamic_graphics = 0;
float graphic_edge = 0; // Max/min angle in radians

extern ref_game_state_t ref_game_state;
int motor_fault_enabled = 0;
extern uint16_t g_motor_fault;
int prev_motor_error = 0;

extern remote_cmd_t g_remote_cmd;
extern uint8_t charging_state;

void map_robot_id(uint16_t robot_id){
	switch (robot_id) {
		case 1: // Red Hero
			g_client_id = 0x101;
			break;
		case 2: // Red Engineer
			g_client_id = 0x102;
			break;
		case 3: // Red Standard 3
			g_client_id = 0x103;
			break;
		case 4: // Red Standard 4
			g_client_id = 0x104;
			break;
		case 5: // Red Standard 5
			g_client_id = 0x105;
			break;
		case 6: // Red Aerial
			g_client_id = 0x106;
			break;

		case 101: // Blue Hero
			g_client_id = 0x165;
			break;
		case 102: // Blue Engineer
			g_client_id = 0x166;
			break;
		case 103: // Blue Standard 3
			g_client_id = 0x167;
			break;
		case 104: // Blue Standard 4
			g_client_id = 0x168;
			break;
		case 105: // Blue Standard 5
			g_client_id = 0x169;
			break;
		case 106: // Blue Aerial
			g_client_id = 0x16A;
			break;
		default:
			g_client_id = 0;
			break;
		}
}

void hud_task(void *argument) {
	// Wait for connection to server
	while (ref_robot_data.robot_id == 0) {
		vTaskDelay(10);
	}
	// Find robot client ID
	map_robot_id(ref_robot_data.robot_id);

	// Set position for graphics at the top
	set_top_coordinates();
	// For calculating the angle for the edge of the pitch HUD
	graphic_edge = TICK_INTERVALS * 4 * 0.0174533;
	// Add all graphics
	clear_hud();
	draw_dynamic(0);
	draw_char(0);
	draw_static();

	//	uint32_t refresh_timer = HAL_GetTick();
	while (1) {
		draw_dynamic(1);
		draw_char(1);

		// Only drawn outside of competition
		// To make sure that impt info gets drawn asap
#ifdef MOTOR_FAULT
		motor_fault();
#endif
#ifdef FEEDER_STATE
		dfeeder_state();
#endif
	}
}

void set_top_coordinates() {
	uint32_t x_coordinates[3];

#ifdef SPINSPIN
	top_graphics++;
#endif
#ifdef GEARING
	top_graphics++;
#endif
#ifdef AIMBOT
	top_graphics++;
#endif
#ifdef SUPERCAP
	top_graphics++;
#endif

	switch (top_graphics) {
		case 1:
			x_coordinates[0] = HUD_MAX_X / 2;
			break;
		case 2:
			x_coordinates[0] = HUD_MAX_X / 2 - TOP_GAP / 2;
			x_coordinates[1] = HUD_MAX_X / 2 + TOP_GAP / 2;
			break;
		case 3:
		default:
			x_coordinates[0] = HUD_MAX_X / 2 - TOP_GAP;
			x_coordinates[1] = HUD_MAX_X / 2;
			x_coordinates[2] = HUD_MAX_X / 2 + TOP_GAP;
			break;
	}

	int index = 0;
#ifdef SPINSPIN
	spin_coords = x_coordinates[index++];
#endif
#ifdef GEARING
	gear_coords = x_coordinates[index++];
#endif
#ifdef AIMBOT
	aimbot_coords = x_coordinates[index++];
#endif
#ifdef SUPERCAP
	aimbot_coords = x_coordinates[index++];
#endif

#ifdef SPINSPIN
	dynamic_graphics++;
#endif
#ifdef SUPERCAP
	dynamic_graphics++;
#endif
#ifdef PITCH_ANG
	dynamic_graphics++;
#endif
#ifdef BALANCE_STATUS
	dynamic_graphics += 2;
#endif
}

// For graphics that need to be modified frequently
// To be updated together within one packet for SPEED
void draw_dynamic(uint8_t modify) {
	// Supercap, spinspin, pitch angle (3 graphics max))
	uint8_t tx_buffer[256];
	uint8_t curr_pos = 0;
	// Count how many graphics to be sent in the packet
	switch (dynamic_graphics) {
	case 0:
		break;
	case 1:
		curr_pos = draw_graphic_header(tx_buffer, 1);
		break;
	case 2:
		curr_pos = draw_graphic_header(tx_buffer, 2);
		break;
	// Draw 5 graphic header, fill the rest with empty graphics
	case 3:
	case 4:
	case 5:
		curr_pos = draw_graphic_header(tx_buffer, 5);
		for (int i = 0; i < 5 - dynamic_graphics; i++) {
			curr_pos += draw_empty(tx_buffer + curr_pos);
		}
		break;
	case 6:
	case 7:
		curr_pos = draw_graphic_header(tx_buffer, 7);
		for (int i = 0; i < 7 - dynamic_graphics; i++) {
			curr_pos += draw_empty(tx_buffer + curr_pos);
		}
		break;
	}


#ifdef SPINSPIN
	curr_pos += draw_spin_border(tx_buffer + curr_pos, modify, spin_coords);
#endif
#ifdef SUPERCAP
	curr_pos += draw_supercap(tx_buffer + curr_pos, modify);
#endif
#ifdef PITCH_ANG
	curr_pos += draw_curr_pitch(tx_buffer + curr_pos, modify);
#endif
#ifdef BALANCE_STATUS
	curr_pos += draw_balancing_status(tx_buffer + curr_pos, modify);
#endif

	if (curr_pos) {
		ref_send(tx_buffer, curr_pos);
		vTaskDelay(REF_DELAY);
	}
}

// Characters to draw: spinspin, gear, aimbot
// Each character has to be sent in their own packet
void draw_char(uint8_t modify) {
	// Draw if adding (initializing), check for change if modifying
	if (modify) {
#ifdef SPINSPIN
		if (prev_spinspin != g_spinspin_mode) {
			prev_spinspin = g_spinspin_mode;
			draw_spin_char(modify, spin_coords);
		}
#endif
#ifdef AIMBOT
		if (prev_aimbot != aimbot_mode) {
			prev_aimbot = aimbot_mode;
			draw_aimbot(modify, aimbot_coords);
		}
#endif
#ifdef SUPERCAP
		if (prev_supercap_dash != supercap_dash) {
			prev_supercap_dash = supercap_dash;
			draw_aimbot(modify, aimbot_coords);
		}
#endif
#ifdef GEARING
		if (prev_gear != gear_speed.curr_gear) {
			prev_gear = gear_speed.curr_gear;
			draw_gearing(modify, gear_coords);
		}
#endif
	} else {
#ifdef SPINSPIN
		prev_spinspin = g_spinspin_mode;
		draw_spin_char(modify, spin_coords);
#endif
#ifdef AIMBOT
		prev_aimbot = aimbot_mode;
		draw_aimbot(modify, aimbot_coords);
#endif
#ifdef SUPERCAP
		prev_supercap_dash = supercap_dash;
		draw_aimbot(modify, aimbot_coords);
#endif
#ifdef GEARING
		prev_gear = gear_speed.curr_gear;
		draw_gearing(modify, gear_coords);
#endif
	}
}

// Graphics that only needs to be drawn once at the start
void draw_static() {
#ifdef CROSSHAIR
	draw_crosshair(0);
#endif
#ifdef PITCH_ANG
	draw_pitch_graphics(0);
#endif
}

void clear_hud() {
	uint8_t tx_buffer[256];
	ref_frame_header_t *send_header = (ref_frame_header_t*)tx_buffer;
	send_header->start_frame = 0xA5;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_delete_graphic_t);
	append_CRC8_check_sum(tx_buffer, 5);
	send_header->cmd_id = REF_ROBOT_COMMS_CMD_ID;
	ref_delete_graphic_t *ref_delete = (ref_delete_graphic_t*)(tx_buffer + 7);
	ref_delete->cmd_ID = 0x100;
	ref_delete->graphic_layer = 9;
	ref_delete->graphic_operation = 2; // delete all
	ref_delete->receiver_ID = g_client_id;
	ref_delete->send_ID = ref_robot_data.robot_id;
	uint16_t tx_len = 7 + sizeof(ref_delete_graphic_t);
	ref_send(tx_buffer,tx_len);
	vTaskDelay(REF_DELAY);
}

uint16_t draw_graphic_header(uint8_t* tx_buffer, uint8_t num_graphics){
	ref_frame_header_t *send_header = (ref_frame_header_t*)tx_buffer;
	send_header->start_frame = 0xA5;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_inter_robot_data_t) +
			sizeof(graphic_data_struct_t) * num_graphics;
	append_CRC8_check_sum(tx_buffer, 5);
	send_header->cmd_id = REF_ROBOT_COMMS_CMD_ID;
	ref_inter_robot_data_t* graphic_header = (ref_inter_robot_data_t*)(tx_buffer + 7);
	if (num_graphics == 1){
		graphic_header->cmd_ID = 0x101;
	}
	if (num_graphics == 2){
		graphic_header->cmd_ID = 0x102;
	}
	if (num_graphics == 5){
		graphic_header->cmd_ID = 0x103;
	}
	if (num_graphics == 7){
		graphic_header->cmd_ID = 0x104;
	}
	//send to self
	graphic_header->send_ID = ref_robot_data.robot_id;
	graphic_header->receiver_ID = g_client_id;

	return sizeof(ref_frame_header_t)+sizeof(ref_inter_robot_data_t);
}

uint16_t draw_char_header(uint8_t* tx_buffer, uint8_t char_len){
	ref_frame_header_t *send_header = (ref_frame_header_t*)tx_buffer;
	send_header->start_frame = 0xA5;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_inter_robot_data_t)
			+ sizeof(graphic_data_struct_t) + char_len;
//	send_header->seq = g_ref_tx_seq++;
	append_CRC8_check_sum(tx_buffer, 5);
	send_header->cmd_id = REF_ROBOT_COMMS_CMD_ID;
	ref_inter_robot_data_t* graphic_header = (ref_inter_robot_data_t*)(tx_buffer + sizeof(ref_frame_header_t));

	graphic_header->cmd_ID = 0x110;
	graphic_header->send_ID = ref_robot_data.robot_id;
	graphic_header->receiver_ID = g_client_id;

	return sizeof(ref_frame_header_t)+sizeof(ref_inter_robot_data_t);
}

void ref_send(uint8_t* tx_buffer, uint16_t tx_len){
	append_CRC16_check_sum(tx_buffer, tx_len + 2);
	while (huart6.gState != HAL_UART_STATE_READY) {
		vTaskDelay(1);
	}
	static uint8_t tx_dma_buffer[256];
//	separate buffer to prevent touching dma buffer while tx is ongoing
	memcpy(tx_dma_buffer, tx_buffer, tx_len+2);
	HAL_UART_Transmit_DMA(&huart6, tx_buffer, tx_len + 2);
}

uint16_t draw_empty(uint8_t* tx_buffer) {
	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer);
	graphic_data->operation_type = GRAPHIC_NO_OP;
	return sizeof(graphic_data_struct_t);
}

void draw_spin_char(uint8_t modify, uint32_t x_coords) {
	uint8_t tx_buffer[256];
	uint32_t curr_pos = 0;
	uint8_t char_len = 0;
	char char_buffer[30];
	char_len = g_spinspin_mode ?
			snprintf((char*) char_buffer, 30, "ON") :
			snprintf((char*) char_buffer, 30, "OFF");

	curr_pos = draw_char_header(tx_buffer, char_len);

	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);

	graphic_data->color = g_spinspin_mode ? GRAPHIC_COLOUR_GREEN : GRAPHIC_COLOUR_ORANGE;
	graphic_data->graphic_name[0] = 'C';
	graphic_data->graphic_name[1] = 'H';
	graphic_data->graphic_name[2] = 'A';
	graphic_data->layer = 2;
	//draw number
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;

	graphic_data->graphic_type = GRAPHIC_TYPE_CHAR; // char
	graphic_data->details_a = FONT_SIZE; // font size
	graphic_data->details_b = char_len; //number of decimal places
	graphic_data->width = CHAR_WIDTH; //line width
	graphic_data->layer = 0;
	//assuming 1920x1080? need check
	graphic_data->start_x = x_coords - CHAR_X_OFFSET * char_len;
	graphic_data->start_y = TOP_Y_POS + CHAR_Y_OFFSET;
	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, char_len);
	curr_pos += char_len;

	ref_send(tx_buffer, curr_pos);
	vTaskDelay(REF_DELAY);
}

uint16_t draw_spin_border(uint8_t* tx_buffer, uint8_t modify, uint32_t x_coords) {
	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer);
	graphic_data->color = g_spinspin_mode ? GRAPHIC_COLOUR_GREEN : GRAPHIC_COLOUR_ORANGE;
	//self set number for identification purposes only
	graphic_data->graphic_name[0] = 'B';
	graphic_data->graphic_name[1] = 'O';
	graphic_data->graphic_name[2] = 'R';
	graphic_data->layer = 2;

	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
	// Checks where the gimbal is facing relative to the chassis
	// Convert from radian to degree, map to 0 to 360
	float chassis_dir = g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang * 57.2958;
	graphic_data->graphic_type = GRAPHIC_TYPE_ARC;
	if (chassis_dir < 0) {
		graphic_data->details_a = 360 + chassis_dir + BORDER_GAP_SIZE; // Start angle
		graphic_data->details_b = 360 + chassis_dir - BORDER_GAP_SIZE; // End angle
	} else {
		graphic_data->details_a = chassis_dir + BORDER_GAP_SIZE; // Start angle
		graphic_data->details_b = chassis_dir - BORDER_GAP_SIZE; // End angle
	}

	graphic_data->width = 7; //line width
	graphic_data->start_x = x_coords;//x_coords;
	graphic_data->start_y = TOP_Y_POS;//TOP_Y_POS;
	graphic_data->details_d = 50; // Length of x axis
	graphic_data->details_e = 50; // Length of y axis

	return sizeof(graphic_data_struct_t);
}

void draw_crosshair(uint8_t modify) {
	uint8_t tx_buffer[256];
	int num_graphics = 2;
	uint32_t curr_pos = 0;

#ifdef CROSSHAIR_TWO_COLOUR
	num_graphics += 3;
#endif

	curr_pos = draw_graphic_header(tx_buffer, num_graphics);

#ifdef CROSSHAIR_TWO_COLOUR
	// Horizontal line
	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
	//self set number for identification purposes only
	graphic_data->graphic_name[0] = 'C';
	graphic_data->graphic_name[1] = 'B';
	graphic_data->graphic_name[2] = 'X';
	graphic_data->layer = 0;
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;

	graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
	graphic_data->color = CROSSHAIR_SHADOW_COLOUR;
	graphic_data->width = CROSSHAIR_SHADOW_THICKNESS;
	graphic_data->start_x = CROSSHAIR_START_X - (CROSSHAIR_SHADOW_THICKNESS/2);
	graphic_data->start_y = CROSSHAIR_CENTER_Y;
	graphic_data->details_d = CROSSHAIR_END_X + (CROSSHAIR_SHADOW_THICKNESS/2);
	graphic_data->details_e = CROSSHAIR_CENTER_Y;
	curr_pos += sizeof(graphic_data_struct_t);

	// Vertical line
	graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
	//self set number for identification purposes only
	graphic_data->graphic_name[0] = 'C';
	graphic_data->graphic_name[1] = 'B';
	graphic_data->graphic_name[2] = 'Y';
	graphic_data->layer = 0;
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;

	graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
	graphic_data->color = CROSSHAIR_SHADOW_COLOUR;
	graphic_data->width = CROSSHAIR_SHADOW_THICKNESS;
	graphic_data->start_x = CROSSHAIR_CENTER_X;
	graphic_data->start_y = CROSSHAIR_START_Y - (CROSSHAIR_SHADOW_THICKNESS/2);
	graphic_data->details_d = CROSSHAIR_CENTER_X;
	graphic_data->details_e = CROSSHAIR_END_Y + (CROSSHAIR_SHADOW_THICKNESS/2);
	curr_pos += sizeof(graphic_data_struct_t);
#endif
	// Horizontal line
	graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
	//self set number for identification purposes only
	graphic_data->graphic_name[0] = 'C';
	graphic_data->graphic_name[1] = 'X';
	graphic_data->graphic_name[2] = 'X';
	graphic_data->layer = 0;
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;

	graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
	graphic_data->color = CROSSHAIR_COLOUR;
	graphic_data->width = CROSSHAIR_THICKNESS;
	graphic_data->start_x = CROSSHAIR_START_X;
	graphic_data->start_y = CROSSHAIR_CENTER_Y;
	graphic_data->details_d = CROSSHAIR_END_X;
	graphic_data->details_e = CROSSHAIR_CENTER_Y;
	curr_pos += sizeof(graphic_data_struct_t);

	// Vertical line
	graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
	//self set number for identification purposes only
	graphic_data->graphic_name[0] = 'C';
	graphic_data->graphic_name[1] = 'X';
	graphic_data->graphic_name[2] = 'Y';
	graphic_data->layer = 0;
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;

	graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
	graphic_data->color = CROSSHAIR_COLOUR;
	graphic_data->width = CROSSHAIR_THICKNESS;
	graphic_data->start_x = CROSSHAIR_CENTER_X;
	graphic_data->start_y = CROSSHAIR_START_Y;
	graphic_data->details_d = CROSSHAIR_CENTER_X;
	graphic_data->details_e = CROSSHAIR_END_Y;
	curr_pos += sizeof(graphic_data_struct_t);

#ifdef CROSSHAIR_TWO_COLOUR
	// Dot
	graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
	//self set number for identification purposes only
	graphic_data->graphic_name[0] = 'C';
	graphic_data->graphic_name[1] = 'B';
	graphic_data->graphic_name[2] = 'D';
	graphic_data->layer = 0;
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;

	graphic_data->graphic_type = GRAPHIC_TYPE_CIRCLE;
	graphic_data->color = CROSSHAIR_DOT_COLOUR;
	graphic_data->width = CROSSHAIR_DOT_WIDTH;
	graphic_data->start_x = CROSSHAIR_CENTER_X;
	graphic_data->start_y = CROSSHAIR_CENTER_Y;
	graphic_data->details_c = CROSSHAIR_DOT_WIDTH/2;
	curr_pos += sizeof(graphic_data_struct_t);
#endif

	ref_send(tx_buffer, curr_pos);
	vTaskDelay(REF_DELAY);
}

void draw_pitch_graphics(uint8_t modify) {
	draw_major_ticks(modify);
	draw_minor_ticks(modify);
	draw_pitch_limits(modify);
	draw_pitch_labels(modify);
}

void draw_major_ticks(uint8_t modify) {
	uint8_t tx_buffer[256];
	uint32_t curr_pos = 0;
	graphic_data_struct_t* graphic_data;

	curr_pos = draw_graphic_header(tx_buffer, 5);

	uint32_t xpos[5] = {
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(ANGLE_LIMIT * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_X/2 + RADIAL_DIAMETER,
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(-ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(-ANGLE_LIMIT * 0.0174533))
	};
	uint32_t ypos[5] = {
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(ANGLE_LIMIT * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_Y/2,
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(-ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(-ANGLE_LIMIT * 0.0174533))
	};

	// Drawing the 5 major ticks
	for (int i = 0; i < 5; i++) {
		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->graphic_name[0] = 'M';
		graphic_data->graphic_name[1] = 'A';
		graphic_data->graphic_name[2] = i + 1;
		graphic_data->layer = 1;
		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
		graphic_data->color = TICK_COLOUR;
		graphic_data->width = MAJOR_TICK_WIDTH;
		graphic_data->start_x = xpos[i] - MAJOR_TICK_LENGTH/2;
		graphic_data->start_y = ypos[i];
		graphic_data->details_d = xpos[i] + MAJOR_TICK_LENGTH/2;
		graphic_data->details_e = ypos[i];
		curr_pos += sizeof(graphic_data_struct_t);
	}

	ref_send(tx_buffer, curr_pos);
	vTaskDelay(REF_DELAY);
}

void draw_minor_ticks(uint8_t modify) {
	uint8_t tx_buffer[256];
	uint32_t curr_pos = 0;
	graphic_data_struct_t* graphic_data;

	curr_pos = draw_graphic_header(tx_buffer, 5);

	float gap = ANGLE_LIMIT / 4;
	uint32_t xpos[4] = {
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(gap * 3 * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(gap * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(-gap * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(-gap * 3 * 0.0174533))
	};
	uint32_t ypos[4] = {
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(gap * 3 * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(gap * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(-gap * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(-gap * 3 * 0.0174533))
	};

	// Drawing the 4 minor ticks
	for (int i = 0; i < 4; i++) {
		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->graphic_name[0] = 'M';
		graphic_data->graphic_name[1] = 'I';
		graphic_data->graphic_name[2] = i + 1;
		graphic_data->layer = 1;
		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
		graphic_data->color = TICK_COLOUR;
		graphic_data->width = MINOR_TICK_WIDTH;
		graphic_data->start_x = xpos[i] - MINOR_TICK_LENGTH/2;
		graphic_data->start_y = ypos[i];
		graphic_data->details_d = xpos[i] + MINOR_TICK_LENGTH/2;
		graphic_data->details_e = ypos[i];
		curr_pos += sizeof(graphic_data_struct_t);
	}
	// Placeholder to meet the 5 graphics limit
	curr_pos += draw_empty(tx_buffer + curr_pos);

	ref_send(tx_buffer, curr_pos);
	vTaskDelay(REF_DELAY);
}

void draw_pitch_labels(uint8_t modify) {
	uint8_t tx_buffer[256];
	uint32_t curr_pos = 0;
	graphic_data_struct_t* graphic_data;

	uint8_t char_len = 0;
	char char_buffer[30];

	int curr_tick_num = 2;

	uint32_t xpos[5] = {
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(ANGLE_LIMIT * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_X/2 + RADIAL_DIAMETER,
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(-ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(-ANGLE_LIMIT * 0.0174533))
	};
	uint32_t ypos[5] = {
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(ANGLE_LIMIT * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_Y/2,
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(-ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(-ANGLE_LIMIT * 0.0174533))
	};

	// Drawing the 5 labels for the major ticks
	for (int i = 0; i < 5; i++) {
		curr_pos = 0;
		char_len = snprintf((char*) char_buffer, 30, "%d", curr_tick_num * TICK_INTERVALS * 2);
		curr_tick_num--;
		curr_pos = draw_char_header(tx_buffer, char_len);

		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->graphic_name[0] = 'L';
		graphic_data->graphic_name[1] = 'A';
		graphic_data->graphic_name[2] =  i + 1;
		graphic_data->layer = 1;
		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_CHAR;
		graphic_data->color = TICK_COLOUR;
		graphic_data->width = CHAR_WIDTH_SMALL;
		graphic_data->start_x = xpos[i] + PITCH_LABEL_DIST;
		graphic_data->start_y = ypos[i] + CHAR_Y_OFFSET;
		graphic_data->details_a = FONT_SIZE_SMALL; // font size
		graphic_data->details_b = char_len; // character length
		curr_pos += sizeof(graphic_data_struct_t);

		memcpy(tx_buffer + curr_pos, char_buffer, char_len);
		curr_pos += char_len;

		ref_send(tx_buffer, curr_pos);
		vTaskDelay(REF_DELAY);
	}
}

void draw_pitch_limits(uint8_t modify) {
	uint8_t tx_buffer[256];
	uint32_t curr_pos = 0;
	graphic_data_struct_t* graphic_data;

	// Map the max and min angle depending on the HUD boundaries
	float max_ang_pos = -PITCH_INVERT * g_pitch_motor.angle_data.phy_max_ang * ANGLE_LIMIT / graphic_edge;
	float min_ang_pos = -PITCH_INVERT * g_pitch_motor.angle_data.phy_min_ang * ANGLE_LIMIT / graphic_edge;

	uint32_t xpos[2] = {
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(max_ang_pos * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(min_ang_pos * 0.0174533))
	};
	uint32_t ypos[2] = {
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(max_ang_pos * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(min_ang_pos * 0.0174533))
	};

	curr_pos = draw_graphic_header(tx_buffer, 2);

	for (int i = 0; i < 2; i++) {
		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->graphic_name[0] = 'L';
		graphic_data->graphic_name[1] = 'I';
		graphic_data->graphic_name[2] = i + 1;
		graphic_data->layer = 1;
		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
		graphic_data->color = PITCH_BOUNDARY_COLOUR;
		graphic_data->width = PITCH_BOUNDARY_WIDTH;
		graphic_data->start_x = xpos[i] - MAJOR_TICK_LENGTH/2;
		graphic_data->start_y = ypos[i];
		graphic_data->details_d = xpos[i] + MAJOR_TICK_LENGTH/2;
		graphic_data->details_e = ypos[i];
		curr_pos += sizeof(graphic_data_struct_t);
	}

	ref_send(tx_buffer, curr_pos);
	vTaskDelay(REF_DELAY);
}

// Takes the center angle that you set as the "true cebter"
uint16_t draw_curr_pitch(uint8_t* tx_buffer, uint8_t modify) {
	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer);

	float curr_ang_pos = -PITCH_INVERT * g_pitch_motor.angle_data.adj_ang * ANGLE_LIMIT / graphic_edge;
	uint32_t xpos = HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(curr_ang_pos * 0.0174533));
	uint32_t ypos = HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(curr_ang_pos * 0.0174533));

	graphic_data->graphic_name[0] = 'P';
	graphic_data->graphic_name[1] = 'I';
	graphic_data->graphic_name[2] = 'T';
	graphic_data->layer = 1;
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
	graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
	graphic_data->color = PITCH_ANG_COLOUR;
	graphic_data->width = PITCH_ANG_WIDTH;
	graphic_data->start_x = xpos - MAJOR_TICK_LENGTH/2;
	graphic_data->start_y = ypos;
	graphic_data->details_d = xpos + MAJOR_TICK_LENGTH/2;
	graphic_data->details_e = ypos;

	return sizeof(graphic_data_struct_t);
}

// Draw motor fault debugging strings outside of competition
void motor_fault() {
	// If currently in competition and motor fault is drawn, delete the layer
	if (ref_game_state.game_progress == 4 || ref_game_state.game_progress == 3) {
		if (motor_fault_enabled) {
			motor_fault_enabled = 0;
			delete_motor_fault();
		}
	} else if (!motor_fault_enabled) {
		motor_fault_enabled = 1;
		draw_motor_fault(0);
		prev_motor_error = g_motor_fault;
	} else if (prev_motor_error != g_motor_fault) {
		delete_motor_fault();
		draw_motor_fault(0);
		prev_motor_error = g_motor_fault;
	}
}

void delete_motor_fault() {
	uint8_t tx_buffer[256];
	ref_frame_header_t *send_header = (ref_frame_header_t*)tx_buffer;
	send_header->start_frame = 0xA5;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_delete_graphic_t);
	append_CRC8_check_sum(tx_buffer, 5);
	send_header->cmd_id = REF_ROBOT_COMMS_CMD_ID;
	ref_delete_graphic_t *ref_delete = (ref_delete_graphic_t*)(tx_buffer + 7);
	ref_delete->cmd_ID = 0x100;
	ref_delete->graphic_layer = 7;
	ref_delete->graphic_operation = 1; // delete a graphic layer
	ref_delete->receiver_ID = g_client_id;
	ref_delete->send_ID = ref_robot_data.robot_id;
	uint16_t tx_len = 7 + sizeof(ref_delete_graphic_t);
	ref_send(tx_buffer,tx_len);
	vTaskDelay(REF_DELAY);
}

void draw_motor_fault(uint8_t modify) {
	uint8_t tx_buffer[256];
	uint8_t curr_pos = 0;
	uint8_t char_len = 0;
	char char_buffer[30] = {0};
	uint8_t char_pos = 0;
	graphic_data_struct_t* graphic_data;

	// Check the 4 chassis motors
	for (uint8_t i = 0; i < 4; i++) {
		if (g_motor_fault & (1 << (i))) {
			// If there are already characters in the buffer
			if (strlen(char_buffer)) {
				snprintf(char_buffer + strlen(char_buffer), 30, ", ");
			}

			switch (i) {
			case 0:
				snprintf(char_buffer + strlen(char_buffer), 30, "FR");
				break;
			case 1:
				snprintf(char_buffer + strlen(char_buffer), 30, "FL");
				break;
			case 2:
				snprintf(char_buffer + strlen(char_buffer), 30, "BL");
				break;
			case 3:
				snprintf(char_buffer + strlen(char_buffer), 30, "BR");
				break;
			}
		}
	}
	char_len = strlen(char_buffer);
	if (char_len) {
		curr_pos = draw_char_header(tx_buffer, char_len);
		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->color = GRAPHIC_COLOUR_PURPLISH_RED;

		//self set number for identification purposes only
		graphic_data->graphic_name[0] = 'C';
		graphic_data->graphic_name[1] = 'H';
		graphic_data->graphic_name[2] = 'S';
		graphic_data->layer = 7;

		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_CHAR; // char
		graphic_data->details_a = FONT_SIZE; // font size
		graphic_data->details_b = char_len; // character length
		graphic_data->width = CHAR_WIDTH; //line width

		graphic_data->start_x = 50;
		graphic_data->start_y = MOTOR_FAULT_START - MOTOR_FAULT_GAP * char_pos;
		char_pos++;

		curr_pos += sizeof(graphic_data_struct_t);
		memcpy(tx_buffer + curr_pos, char_buffer, char_len);
		curr_pos += char_len;

		ref_send(tx_buffer, curr_pos);
		vTaskDelay(REF_DELAY);

		memset(char_buffer, 0, sizeof(char_buffer)); // clear buffer
	}

	// Check launcher motors
	for (uint8_t i = 4; i < 7; i++) {
		if (g_motor_fault & (1 << (i))) {
			// If there are already characters in the buffer
			if (strlen(char_buffer)) {
				snprintf(char_buffer + strlen(char_buffer), 30, ", ");
			}

			switch (i) {
			case 4:
				snprintf(char_buffer + strlen(char_buffer), 30, "LF");
				break;
			case 5:
				snprintf(char_buffer + strlen(char_buffer), 30, "RF");
				break;
			case 6:
				snprintf(char_buffer + strlen(char_buffer), 30, "FEED");
				break;
			}
		}
	}
#ifdef ACTIVE_GUIDANCE
	for (uint8_t i = 9; i < 11; i++) {
		if (g_motor_fault & (1 << (i))) {
			// If there are already characters in the buffer
			if (strlen(char_buffer)) {
				snprintf(char_buffer + strlen(char_buffer), 30, ", ");
			}

			switch (i) {
			case 9:
				snprintf(char_buffer + strlen(char_buffer), 30, "BF");
				break;
			case 10:
				snprintf(char_buffer + strlen(char_buffer), 30, "AG");
				break;
			}
		}
	}
#endif

	char_len = strlen(char_buffer);
	if (char_len) {
		curr_pos = draw_char_header(tx_buffer, char_len);
		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->color = GRAPHIC_COLOUR_PURPLISH_RED;

		//self set number for identification purposes only
		graphic_data->graphic_name[0] = 'L';
		graphic_data->graphic_name[1] = 'A';
		graphic_data->graphic_name[2] = 'U';
		graphic_data->layer = 7;

		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_CHAR; // char
		graphic_data->details_a = FONT_SIZE; // font size
		graphic_data->details_b = char_len; // character length
		graphic_data->width = CHAR_WIDTH; //line width

		graphic_data->start_x = 50;
		graphic_data->start_y = MOTOR_FAULT_START - MOTOR_FAULT_GAP * char_pos;
		char_pos++;

		curr_pos += sizeof(graphic_data_struct_t);
		memcpy(tx_buffer + curr_pos, char_buffer, char_len);
		curr_pos += char_len;

		ref_send(tx_buffer, curr_pos);
		vTaskDelay(REF_DELAY);

		memset(char_buffer, 0, sizeof(char_buffer)); // clear buffer
	}

	// Check pitch and yaw
	for (uint8_t i = 7; i < 9; i++) {
		if (g_motor_fault & (1 << (i))) {
			// If there are already characters in the buffer
			if (strlen(char_buffer)) {
				snprintf(char_buffer + strlen(char_buffer), 30, ", ");
			}

			switch (i) {
			case 7:
				snprintf(char_buffer + strlen(char_buffer), 30, "PITCH");
				break;
			case 8:
				snprintf(char_buffer + strlen(char_buffer), 30, "YAW");
				break;
			}
		}
	}

	char_len = strlen(char_buffer);
	if (char_len) {
		curr_pos = draw_char_header(tx_buffer, char_len);
		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->color = GRAPHIC_COLOUR_PURPLISH_RED;

		//self set number for identification purposes only
		graphic_data->graphic_name[0] = 'P';
		graphic_data->graphic_name[1] = 'A';
		graphic_data->graphic_name[2] = 'Y';
		graphic_data->layer = 7;

		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_CHAR; // char
		graphic_data->details_a = FONT_SIZE; // font size
		graphic_data->details_b = char_len; // character length
		graphic_data->width = CHAR_WIDTH; //line width

		graphic_data->start_x = 50;
		graphic_data->start_y = MOTOR_FAULT_START - MOTOR_FAULT_GAP * char_pos;
		char_pos++;

		curr_pos += sizeof(graphic_data_struct_t);
		memcpy(tx_buffer + curr_pos, char_buffer, char_len);
		curr_pos += char_len;

		ref_send(tx_buffer, curr_pos);
		vTaskDelay(REF_DELAY);
	}
}

void draw_feeder_state(uint8_t modify) {
	uint8_t tx_buffer[256];
	uint8_t curr_pos = 0;
	uint8_t char_len = 0;
	char char_buffer[30];
	uint32_t colour = 0;

	switch (feeder_state) {
	case FEEDER_STANDBY:
	case FEEDER_SPINUP:
		colour = GRAPHIC_COLOUR_GREEN;
		char_len = snprintf((char*) char_buffer, 30, "STANDBY");
		break;
	case FEEDER_JAM:
		colour = GRAPHIC_COLOUR_YELLOW;
		char_len = snprintf((char*) char_buffer, 30, "JAMMED");
		break;
	case FEEDER_OVERHEAT:
		colour = GRAPHIC_COLOUR_PURPLISH_RED;
		char_len = snprintf((char*) char_buffer, 30, "OVERHEAT");
		break;
	case FEEDER_STEP:
	case FEEDER_FIRING:
	default:
		colour = GRAPHIC_COLOUR_GREEN;
		char_len = snprintf((char*) char_buffer, 30, "FIRING");
		break;
	}

	curr_pos = draw_char_header(tx_buffer, char_len);

	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
	graphic_data->color = colour;

	//self set number for identification purposes only
	graphic_data->graphic_name[0] = 'F';
	graphic_data->graphic_name[1] = 'D';
	graphic_data->graphic_name[2] = 'R';
	graphic_data->layer = 6;

	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
	graphic_data->graphic_type = GRAPHIC_TYPE_CHAR; // char
	graphic_data->details_a = FONT_SIZE; // font size
	graphic_data->details_b = char_len; // character length
	graphic_data->width = CHAR_WIDTH; //line width

	graphic_data->start_x = 50;
	graphic_data->start_y = 800;

	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, char_len);
	curr_pos += char_len;

	ref_send(tx_buffer, curr_pos);
	vTaskDelay(REF_DELAY);
}

void delete_feeder_state() {
	uint8_t tx_buffer[256];
	ref_frame_header_t *send_header = (ref_frame_header_t*)tx_buffer;
	send_header->start_frame = 0xA5;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_delete_graphic_t);
	append_CRC8_check_sum(tx_buffer, 5);
	send_header->cmd_id = REF_ROBOT_COMMS_CMD_ID;
	ref_delete_graphic_t *ref_delete = (ref_delete_graphic_t*)(tx_buffer + 7);
	ref_delete->cmd_ID = 0x100;
	ref_delete->graphic_layer = 6;
	ref_delete->graphic_operation = 1; // delete a graphic layer
	ref_delete->receiver_ID = g_client_id;
	ref_delete->send_ID = ref_robot_data.robot_id;
	uint16_t tx_len = 7 + sizeof(ref_delete_graphic_t);
	ref_send(tx_buffer,tx_len);
	vTaskDelay(REF_DELAY);
}

void dfeeder_state() {
	// If currently in competition and feeder state is drawn, delete the layer
	if (ref_game_state.game_progress == 4 || ref_game_state.game_progress == 3) {
		if (feeder_state_enabled) {
			feeder_state_enabled = 0;
			delete_feeder_state();
		}
	} else if (!feeder_state_enabled) {
		feeder_state_enabled = 1;
		draw_feeder_state(0);
		prev_feeder_state = feeder_state;
	} else if (prev_feeder_state != feeder_state) {
		draw_feeder_state(1);
		prev_feeder_state = feeder_state;
	}
}

//this function is meant to visualize the current balancing status of the robot
uint16_t draw_balancing_status(uint8_t* tx_buffer, uint8_t modify){
	//stateVar.phi = angle between horizontal axis and chassis
	//stateVar.theta = angle between normal of ground and the leg
	uint8_t curr_pos = 0;
	//center the point at point of contact with the floor

	//draw the "floor"
	//length of floor is 210 pixels
//	graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
//	graphic_data->graphic_name[0] = 'B';
//	graphic_data->graphic_name[1] = 'S';
//	graphic_data->graphic_name[2] = 'F';
//	graphic_data->layer = 3;
//	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
//	graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
//	graphic_data->color = 6; //cyan
//	graphic_data->width = PITCH_BOUNDARY_WIDTH;
//	//assuming 1920x1080
//	graphic_data->start_x = 1605;
//	graphic_data->start_y = 540;
//	graphic_data->details_d = 1815;
//	graphic_data->details_e = 540;
//	curr_pos += sizeof(graphic_data_struct_t);

	//center the point at point of contact with the floor
	//draw the "leg"
	//length of leg put as 180 pixels
	float pitch_remote = ((float) g_remote_cmd.right_y / 660) * PITCH_INVERT
	      * REMOTE_PITCH_SPEED * PI;
	  float yaw_remote = ((float) g_remote_cmd.left_y / 660) * YAW_INVERT
	      * REMOTE_YAW_SPEED * PI;
	uint32_t pivot_chassis_x = sin( pitch_remote ) * 180 + 1710; //replace pi/6 with stateVar.theta
	uint32_t pivot_chassis_y = cos( pitch_remote ) * 180 + 540;

	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer);
	graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
	graphic_data->graphic_name[0] = 'B';
	graphic_data->graphic_name[1] = 'S';
	graphic_data->graphic_name[2] = 'L';
	graphic_data->layer = 3;
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
	graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
	graphic_data->color = 1; //yellow
	graphic_data->width = PITCH_BOUNDARY_WIDTH;
	graphic_data->start_x = 1710; //anchor wheel to midpoint of floor
	graphic_data->start_y = 540;
	graphic_data->details_d = pivot_chassis_x;
	graphic_data->details_e = pivot_chassis_y;
	curr_pos += sizeof(graphic_data_struct_t);
//draw the chassis
	uint32_t chassis_left_x = pivot_chassis_x - cos( yaw_remote ) * 105; //replace pi/6 with stateVar.phi
	uint32_t chassis_left_y = pivot_chassis_y - sin( yaw_remote ) * 105;
	uint32_t chassis_right_x = pivot_chassis_x + cos( yaw_remote) * 105;
	uint32_t chassis_right_y = pivot_chassis_y + sin( yaw_remote ) * 105;
	graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
	graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
	graphic_data->graphic_name[0] = 'B';
	graphic_data->graphic_name[1] = 'S';
	graphic_data->graphic_name[2] = 'C';
	graphic_data->layer = 3;
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
	graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
	graphic_data->color = 1; //yellow
	graphic_data->width = PITCH_BOUNDARY_WIDTH;
	graphic_data->start_x = chassis_left_x;
	graphic_data->start_y = chassis_left_y;
	graphic_data->details_d = chassis_right_x;
	graphic_data->details_e = chassis_right_y;
	curr_pos += sizeof(graphic_data_struct_t); //technically unnecessary

	return (sizeof(graphic_data_struct_t)*2);
}
