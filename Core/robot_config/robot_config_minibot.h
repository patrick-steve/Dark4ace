/*
 * robot_config_minibot.h
 *
 *  Created on: Sep 1, 2024
 *      Author: cw
 */

#ifndef ROBOT_CONFIG_ROBOT_CONFIG_MINIBOT_H_
#define ROBOT_CONFIG_ROBOT_CONFIG_MINIBOT_H_

#include "motor_config.h"
#include "hud_minibot.h"
#define BULLET_17

/********************* DEV C IMU CONFIGURATION ***********/
//0 for SWDIO port to be roll, 1 for SWDIO port to be pitch, 2 for vertical mount SWDIO port to the right
#define IMU_ORIENTATION	1

//flip until motor angle and yaw angle matches
#define IMU_YAW_INVERT		-1
#define IMU_PITCH_INVERT	1
//nothing uses roll.....yet
#define IMU_ROLL_INVERT		1

#define G_X_OFFSET		-7
#define G_Y_OFFSET 		-16
#define G_Z_OFFSET	 	5
#define ZERO_ROLL


/********************* CONTROL SENSITIVITIES ***********/
#define REMOTE_YAW_SPEED 	 			0.1 			//Speed of gimbal yaw turning
#define REMOTE_PITCH_SPEED 	 			-0.1//0.005		//Speed of gimbal pitch turning

#define MOUSE_X_SENSITIVITY		(300 * REMOTE_YAW_SPEED)				//Speed of yaw turning with mouse, dependent on above speed
#define MOUSE_Y_SENSITIVITY 	(200 * REMOTE_PITCH_SPEED)				//Speed of pitch turning with mouse,  dependent on above speed


/*********************** MANUAL CONTROL CONFIGURATION *******************/
//Inverts for both keyboard and mouse controls
#define YAW_INVERT  			-1				//1 to invert control -1 to disable
#define PITCH_INVERT  			1				//1 to invert control -1 to disable

#define MOUSE_X_INVERT			1				//Set to -1 if it needs to be inverted
#define	MOUSE_Y_INVERT			-1				//Set to -1 if it needs to be inverted

#define MOUSE_LIMIT 			200

#define KEYBD_MAX_SPD 			1				//% of max speed	//% of max speed

#define GIMBAL_MODE 			1				//1 for IMU control, 0 for absolute angle based control


/* PID TUNING GUIDE
 * For all motors, there are 2 different PID values, angle PID and speed PID
 * For motors that require position control, both values have to be set
 *
 * Speed PID is the main control loop which determines how much current
 * to send to the motors. i.e. it sets the target speed for the motors
 * Angle PID calculates the RPM the motor should be running at, then runs the
 * target values through the speed PID loop
 *
 * Generally, the speed control loop should be PID,
 * while the angle control loop can just be a P control
 *
 * TO TUNE PID
 * Tune speed loop FIRST, if not the angle loop might resonate and cause it to oscillate instead
 */

/*********************** LAUNCHER CONFIGURATION ***********************/
#define FEEDER_SPEED			1200
#define	PROJECTILE_SPEED		20.5

#define PROJECTILE_SPEED_RATIO	360	//rpm per m/s of the friction wheels ish don't think this will work well lmao
#define FEEDER_SPEED_RATIO		8	//projectiles per round of the feeder

// prevents pilots from overheating when firing
#define OVERHEAT_PROTECTION
#define OVERHEAT_MARGIN 		2
#define OVERHEAT_EXCESS 		2
#define OVERHEAT_OFFSET			20

// FEEDER PID VALUES
#define FEEDER_KP 				5
#define FEEDER_KI  				0.02
#define FEEDER_KD  				3
#define FEEDER_MAX_INT			10000

#define FEEDER_ANGLE_KP 		1000
#define FEEDER_ANGLE_KD  		0
#define FEEDER_ANGLE_KI  		0
#define FEEDER_ANGLE_INT_MAX  	0
#define FEEDER_MAX_RPM			100

#define FEEDER_JAM_TORQUE  		9800		//Before feeder deemed to be jammed
#define FEEDER_JAM_RPM			100			// if feeeder is below this rpm, it is jammed
#define FEEDER_UNJAM_SPD  		-500		//Reverse unjam
#define FEEDER_UNJAM_TIME		50
#define FEEDER_MAX_CURRENT		10000
#define FEEDER_INVERT			1

// FRICTION WHEELS PID VALUES
#define STEPPER_ANGLE			1.8
#define FRICTION_SB_SPIN		0//LV1_PROJECTILE * PROJECTILE_SPEED_RATIO//6000
#define FRICTION_KP  			5
#define FRICTION_KI  			0
#define FRICTION_KD  			0
#define FRICTION_MAX_CURRENT 	16384
#define FRICTION_MAX_INT		10000
#define FRICTION_INVERT			-1
#define LAUNCHER_MARGIN			300
#define LAUNCHER_DIFF_MARGIN	300
#define FRICTION_OFFSET			0

#define CLEAR_DELAY				1000

/*********************** CHASSIS CONFIGURATION ***********************/
// CHASSIS WHEELS PID VALUES
#define CHASSIS_KP  			4
#define CHASSIS_KI  			0.1
#define CHASSIS_KD  			1
#define CHASSIS_INT_MAX  		1000
#define CHASSIS_MAX_CURRENT 	6000
#define CHASSIS_MIN_CURRENT 	0
#define BUFFER_MIN				0.22		// power buffer minimum, at zero buffer left, motors will draw CHASSIS_MAX_CURRENT * BUFFER_MIN
											//tune this by seeing if pilot likes the speed
#define CHASSIS_CAN_SPINSPIN
#define CHASSIS_YAW_MAX_RPM		0.75		//max RPM for chassis centering
#define CHASSIS_YAW_KP 			3.5
#define CHASSIS_YAW_KI			0
#define CHASSIS_YAW_KD 			0.2
#define CHASSIS_YAW_MIN			0.1

#define REF_POWER_LIM
#define MIN_SPEED				2000
#define MAX_SPEED 				9000		//M3508_MAX_RPM  //Max speed of robot
#define LVL_MAX_SPEED			10
#define CHASSIS_POWER_KP		0.2

#define CHASSIS_MAX_POWER 		400
#define CHASSIS_POWER_MARGIN 	0
//#define CHASSIS_POWER_BUFFER_LIMITER
#define CHASSIS_POWER_LPF 		0.1         // filter for chassis power readout from referee system
#define CHASSIS_POWER_DELTA_LIM 0.3   		// limit change for the automatic rpm adjusting code


/* To configure centers, start the boards in debug mode with all motors
 * powered *but in safe mode* (i.e. remotes off)
 * Physically push the motors to the desired centers
 * and put a breakpoint/live expression on their respective real_ang variables
 * from their raw_data structs
 * The centers should be from 0 to 8192, it should be the value directly from
 * the motors
 */
/*********************** GIMBAL CONFIGURATION ***********************/
#define PITCH_ANGLE_KP	  		0.05
#define PITCH_ANGLE_KI  		0.01
#define PITCH_ANGLE_KD  		0
#define PITCH_ANGLE_INT_MAX		0.1
#define PITCH_MAX_RPM			60

#define PITCHRPM_KP				1
#define PITCHRPM_KI				1
#define PITCHRPM_KD				0
#define PITCHRPM_INT_MAX		50
#define PITCH_MAX_CURRENT		20000

#define PITCH_MOTOR_TYPE		TYPE_GM6020
#define PITCH_CENTER			2071
#define PITCH_MAX_ANG			0.50
#define PITCH_MIN_ANG			-0.50
#define PITCH_CONST 			0

#define YAW_ANGLE_KP			0.1
#define YAW_ANGLE_KI			0.1
#define YAW_ANGLE_KD			0
#define YAW_ANGLE_INT_MAX		0.5
#define YAW_MAX_RPM				85
#define YAW_SPINSPIN_CONSTANT	5000

#define YAWRPM_KP				1000
#define YAWRPM_KI				10
#define YAWRPM_KD				15
#define YAWRPM_INT_MAX			200
#define YAW_MAX_CURRENT			20000

#define YAW_CENTER 				-2.5
#define YAW_MAX_ANG				5*PI
#define YAW_MIN_ANG				5*-PI

/*********************** MOTOR CONFIGURATION *******************/
//CAN ids for the motors, for motors on the CAN2 bus, add 12
//ADD 4 TO GM6020 IDS i.e. flashing 5 times = ID 9
//#define CHASSIS_MCU
#ifndef CHASSIS_MCU
#define FR_MOTOR_ID 			13
#define FR_MOTOR_CAN_PTR		&hcan2
#define FL_MOTOR_ID 			14
#define FL_MOTOR_CAN_PTR		&hcan2
#define BL_MOTOR_ID 			15
#define BL_MOTOR_CAN_PTR		&hcan2
#define BR_MOTOR_ID 			16
#define BR_MOTOR_CAN_PTR		&hcan2
#endif
#define FEEDER_MOTOR_ID			3 // set to 3 on the esc
#define FEEDER_MOTOR_CAN_PTR	&hcan1
#define LFRICTION_MOTOR_ID		1 // either 2 / 3
#define LFRICTION_MOTOR_CAN_PTR	&hcan1
#define RFRICTION_MOTOR_ID		2 // either 3 / 2
#define RFRICTION_MOTOR_CAN_PTR	&hcan1

//NOTE: two motors CANNOT have the same __flashing__ number (i.e. GM6020 id 9 cannot be used
//with any id 6 motors
#define PITCH_MOTOR_ID 			10 // blinking 6
#define PITCH_MOTOR_CAN_PTR		&hcan1
#ifndef CHASSIS_MCU
#define YAW_MOTOR_ID 			23 // blinking 7
#define YAW_MOTOR_CAN_PTR		&hcan2

#endif

// /* MECANUM WHEEL PROPERTIES */
// #define WHEEL_CIRC			7.625	//in CM

// #define FR_VX_MULT			-1		//-cos(FR_ANG_Y - FR_ANG_PASSIVE)/sin(FR_ANG_PASSIVE)
// #define FR_VY_MULT			-1		//-sin(FR_ANG_Y - FR_ANG_PASSIVE)/sin(FR_ANG_PASSIVE)
// #define FR_YAW_MULT			1		//((-FR_DIST * sin(FR_ANG_Y - FR_ANG_PASSIVE - FR_ANG_X)) / (sin(FR_ANG_PASSIVE) * WHEEL_CIRC))

// #define FL_VX_MULT			-1 		//-cos(FL_ANG_Y - FL_ANG_PASSIVE)/sin(FL_ANG_PASSIVE)
// #define FL_VY_MULT			1		//-sin(FL_ANG_Y - FL_ANG_PASSIVE)/sin(FL_ANG_PASSIVE)
// #define FL_YAW_MULT			1		//((-FL_DIST * sin(FL_ANG_Y - FL_ANG_PASSIVE - FL_ANG_X)) / (sin(FL_ANG_PASSIVE) * WHEEL_CIRC))

// #define BL_VX_MULT			1		//-cos(BL_ANG_Y - BL_ANG_PASSIVE)/sin(BL_ANG_PASSIVE)
// #define BL_VY_MULT			1		//-sin(BL_ANG_Y - BL_ANG_PASSIVE)/sin(BL_ANG_PASSIVE)
// #define BL_YAW_MULT			1		//((-BL_DIST * sin(BL_ANG_Y - BL_ANG_PASSIVE - BL_ANG_X)) / (sin(BL_ANG_PASSIVE) * WHEEL_CIRC))

// #define BR_VX_MULT			1		//-cos(BR_ANG_Y - BR_ANG_PASSIVE)/sin(BR_ANG_PASSIVE)
// #define BR_VY_MULT			-1		//-sin(BR_ANG_Y - BR_ANG_PASSIVE)/sin(BR_ANG_PASSIVE)
// #define BR_YAW_MULT			1		//((-BR_DIST * sin(BR_ANG_Y - BR_ANG_PASSIVE - BR_ANG_X)) / (sin(BR_ANG_PASSIVE) * WHEEL_CIRC))

/* edited wheel constants */
#define WHEEL_CIRC			7.625	//in CM

#define FR_VX_MULT			-1		//-cos(FR_ANG_Y - FR_ANG_PASSIVE)/sin(FR_ANG_PASSIVE)
#define FR_VY_MULT			-1		//-sin(FR_ANG_Y - FR_ANG_PASSIVE)/sin(FR_ANG_PASSIVE)
#define FR_YAW_MULT			1		//((-FR_DIST * sin(FR_ANG_Y - FR_ANG_PASSIVE - FR_ANG_X)) / (sin(FR_ANG_PASSIVE) * WHEEL_CIRC))

#define FL_VX_MULT			-1 		//-cos(FL_ANG_Y - FL_ANG_PASSIVE)/sin(FL_ANG_PASSIVE)
#define FL_VY_MULT			1		//-sin(FL_ANG_Y - FL_ANG_PASSIVE)/sin(FL_ANG_PASSIVE)
#define FL_YAW_MULT			1		//((-FL_DIST * sin(FL_ANG_Y - FL_ANG_PASSIVE - FL_ANG_X)) / (sin(FL_ANG_PASSIVE) * WHEEL_CIRC))

#define BL_VX_MULT			1		//-cos(BL_ANG_Y - BL_ANG_PASSIVE)/sin(BL_ANG_PASSIVE)
#define BL_VY_MULT			1		//-sin(BL_ANG_Y - BL_ANG_PASSIVE)/sin(BL_ANG_PASSIVE)
#define BL_YAW_MULT			1		//((-BL_DIST * sin(BL_ANG_Y - BL_ANG_PASSIVE - BL_ANG_X)) / (sin(BL_ANG_PASSIVE) * WHEEL_CIRC))

#define BR_VX_MULT			1		//-cos(BR_ANG_Y - BR_ANG_PASSIVE)/sin(BR_ANG_PASSIVE)
#define BR_VY_MULT			-1		//-sin(BR_ANG_Y - BR_ANG_PASSIVE)/sin(BR_ANG_PASSIVE)
#define BR_YAW_MULT			1		//((-BR_DIST * sin(BR_ANG_Y - BR_ANG_PASSIVE - BR_ANG_X)) / (sin(BR_ANG_PASSIVE) * WHEEL_CIRC))

/*********************** OTHERS ***********************/
#define CONTROL_DELAY 			5
#define GIMBAL_DELAY			2
#define CHASSIS_DELAY 			5

//microsecond timer used for PIDs
#define TIMER_FREQ			1000000 //Cannot be too high if not the ISRs overload the CPU
#define TIMER_FREQ_MULT		10 //1000000/100000
#endif /* TASKS_INC_ROBOT_CONFIG_H_ */
