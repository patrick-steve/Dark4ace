

#ifndef TASKS_INC_ROBOT_CONFIG_H_
#define TASKS_INC_ROBOT_CONFIG_H_

#include "typedefs.h"
#include "robot_config_minibot.h"

// COMMON CONFIGURATION

#define SPINSPIN_RANDOM_DELAY	50
#define MOTOR_ONLINE_CHECK 		-1
//1 for annoying beep sound, 0 for some error beeps every 3s, -1 for absolute peace and tranquility (with pancik
#define ARM_SWITCH 				0			//set to 1 to enable remote up and down arming switch
#define FIRING_DISABLE			0			//set to 1 to stop firing
#define LAUNCHER_SAFETY			0			//set to 1 to enable launcher arm safety
#define FRICTION_SB_SPIN_ON		1
//(between mode switch, remote off, power on, left switch MUST be down)


//if no overrides in the respective configs
#ifndef CONTROL_DEFAULT
//#define CONTROL_DEFAULT 		KEYBOARD_CTRL_MODE
#define CONTROL_DEFAULT			REMOTE_CTRL_MODE
//#define CONTROL_DEFAULT			SBC_CTRL_MODE

#endif

#define CHASSIS_CTL_RAMP 0.000001

/*********************** OTHERS ***********************/
#define MOTOR_TIMEOUT_MAX	1000000

#define REMOTE_TIMEOUT 		200
#define MAX_RC_VALUE 		1320 //660 * 2
#define RC_LIMITS			660
#define HITEMP_WARNING  	70

#define KEY_OFFSET_W        ((uint16_t)0x01<<0)
#define KEY_OFFSET_S        ((uint16_t)0x01<<1)
#define KEY_OFFSET_A 		((uint16_t)0x01<<2)
#define KEY_OFFSET_D        ((uint16_t)0x01<<3)
#define KEY_OFFSET_Q        ((uint16_t)0x01<<6)
#define KEY_OFFSET_E        ((uint16_t)0x01<<7)
#define KEY_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define KEY_OFFSET_CTRL     ((uint16_t)0x01<<5)

#endif /* TASKS_INC_ROBOT_CONFIG_H_ */

