/*
 * telemetry_task.h
 *
 *  Created on: 17 Feb 2022
 *      Author: wx
 */

#ifndef TASKS_INC_TELEMETRY_TASK_H_
#define TASKS_INC_TELEMETRY_TASK_H_

void telemetry_task(void *argument);

typedef struct{
	uint8_t frame_header;
	uint8_t packet_id;
	uint8_t data_length;
}telem_header_t;

//8 bytes
typedef struct{
	float integral;
	float output;
}telem_pid_t;

//9 bytes
typedef struct{
	float adj_ang;
	int16_t rpm;
	int16_t torque;
	uint8_t temp;
}telem_motor_sense_data_t;

//30 bytes
typedef struct{
	uint8_t motor_id;
	telem_motor_sense_data_t motor_sense_data;
	telem_pid_t rpm_pid_data;
	telem_pid_t ang_pid_data;
	int32_t ticks;
}telem_motor_data_t;

//20 bytes
typedef struct{
	float kp;
	float ki;
	float kd;
	float int_max;
	float max_out;
}telem_pid_init_t;

//50 bytes
typedef struct{
	uint8_t motor_id;
	uint8_t motor_type;
	int32_t center_ang;
	float gearbox_ratio;
	telem_pid_init_t rpm_pid_var;
	telem_pid_init_t ang_pid_var;
}telem_motor_init_t;


//25 bytes
typedef struct{
	float pitch;
	float yaw;
	float forward;
	float horizontal;
	uint16_t projectile_speed;
	uint16_t gun_feeding_speed;
	uint8_t aimbot_mode;
}telem_input_data_t;

//7 bytes
typedef struct{
    uint16_t magazine_17mm;
    uint8_t bullet_freq;
    float bullet_speed;
    uint16_t shooter_heat0;
    uint16_t shooter_heat1;
}telem_ref_bullet_data_t;

//10 bytes
typedef struct{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
}telem_ref_power_data_t;


//12 bytes
typedef struct{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t shooter_heat0_cooling_rate;
    uint16_t shooter_heat0_cooling_limit;
    uint16_t shooter_heat1_cooling_rate;
    uint16_t shooter_heat1_cooling_limit;
    uint16_t chassis_power_limit;
}telem_ref_init_t;

typedef enum {
	motor_init_data = 0,
	ref_init_data	= 1,

	motor_data		= 2,
	control_data	= 3,
	raw_rc_data		= 4,
	raw_gyro_data	= 5,
	raw_accel_data	= 6,
	raw_mag_data	= 7,
	orientation_data= 8,
	linear_accel_data= 9,

	ref_bullet_data	= 10,
	ref_power_data	= 11
}data_enum_t;

typedef union {
	//init data
	telem_motor_init_t motor_init_data; //50 bytes
	telem_ref_init_t ref_init_data;		//12 bytes

	telem_motor_data_t motor_data; 		//30 bytes
	telem_input_data_t control_data; 	//25 bytes
	remote_cmd_t raw_rc_data; 			//30 bytes
	gyro_data_t raw_gyro_data;			//16 bytes
	accel_data_t raw_accel_data;		//16 bytes
	mag_data_t raw_mag_data;			//10 bytes
	orientation_data_t orientation_data;//12 bytes
	linear_accel_t linear_acc_data;		//12 bytes MODULE NOT DONE YET

	telem_ref_bullet_data_t ref_bullet_data; 	//7 bytes
	telem_ref_power_data_t ref_power_data; 		//10 bytes

}telem_data_union_t;

typedef struct{
	uint8_t data_type;
	telem_data_union_t telem_data;
}telem_data_struct_t;

#define TELEM_MAX_DATA_SIZE 50

typedef struct {
	telem_header_t telem_packet_header;
	uint8_t packet_data[TELEM_MAX_DATA_SIZE];
	//uint16_t CRC16;
	uint8_t packet_size;
}telem_send_data;

#endif /* TASKS_INC_TELEMETRY_TASK_H_ */
