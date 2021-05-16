
/********************************** INCLUDE ***********************************/

#include <ch.h>
#include <hal.h>
#include <main.h>
#include "motors.h"
#include "leds.h"
#include "sensors/proximity.h"
#include "chprintf.h"
#include "i2c_bus.h"
#include "usbcfg.h"
#include "audio_processing.h"
#include "sensors/VL53L0X/VL53L0X.h"

/********************************** STATIC GLOBALE VARIABLES ***********************************/

static bool no_goal = true;

/********************************** CONSTANTS ***********************************/

#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    // in [cm]
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define MOTOR_TIMER_FREQ 100000 // in [Hz]
#define THRESV 650 // This is the speed under which the power save feature is active
#define WHEEL_PERIMETER 130 // in [mm]
#define NSTEP_ONE_TURN 1000 // Number of steps for 1 turn of the motor
#define SEARCH_SPEED 350 // Constant speed in step/s of the robot when turning to find the ball
#define ATTACK_SPEED 800 // Constant speed in step/s of the robot when moving towards the ball
#define STOP_SPEED 0 // Halt speed
#define CELEB_SPEED_LEFT 750 // Constant left motor speed in step/s of the robot when turning to celebrate the goal
#define CELEB_SPEED_RIGHT -750 // Constant right motor speed in step/s of the robot when turning to celebrate the goal
#define SLEEP_THD_SEARCH 100 // Sleep time of the search thread
#define BALL_IN_THE_AREA 500 // Ball distance in [mm] from which the robot starts to move towards it
#define BALL_BLOCKED 300 // Threshold value from the IR sensors from which the robot analyses if it's blocked
#define BALL_BLOCKED_CONFIRMED 10 // Threshold value to confirm if the ball is blocked or not

/********************************** UNBLOCK BALL FUNCTION ***********************************/

/* Function used to detect if the ball is blocked in a corner. IR1 and IR8 sensors are used.
* params :
* uint8_t* counter : a counter value that is incremented when the IR1 and IR8 sensors are so close to the ball.
* When this value reaches the threshold BALL_BLOCKED_CONFIRMED (in this case we chose it 10), the ball is blocked and the robot
* starts turning in order to generate the rotation of the ball.
* Thus, the ball is unblocked and the game continues
*/

void unblock_ball(uint8_t* counter ){
	// We chose the left motor as a reference.
	int32_t blocked_pos = left_motor_get_pos(); // Return the last position of the left motor
	// If IR1 or IR8 detects the ball very close to the robot
	if(get_calibrated_prox(0)> BALL_BLOCKED || get_calibrated_prox(7)> BALL_BLOCKED){
		*counter+= 1;
		if(*counter>=BALL_BLOCKED_CONFIRMED){
			left_motor_set_pos(PERIMETER_EPUCK * NSTEP_ONE_TURN / WHEEL_PERIMETER); // Left motor makes one turn
			right_motor_set_pos(PERIMETER_EPUCK * NSTEP_ONE_TURN / WHEEL_PERIMETER); // Right motor makes one turn
			left_motor_set_speed(SEARCH_SPEED);
			right_motor_set_speed(-SEARCH_SPEED);
			while(left_motor_get_pos()-blocked_pos<=PERIMETER_EPUCK * NSTEP_ONE_TURN / WHEEL_PERIMETER){
				; // Nothing to do until the robot finishes a complete turn
			}
		}
	}
	else{
		*counter=0; // The ball is not blocked
	}
}

/********************************** SEARCH THREAD ***********************************/

static THD_WORKING_AREA(waSEARCHThd, 512);
static THD_FUNCTION(SEARCHThd, arg){
	(void)arg;
	chRegSetThreadName("SEARCHThd");
	bool search=false;
	static uint8_t counter = 0;
	uint16_t dist=0;
	// Thread loop while there's no termination request and the game duration didn't exceed 30s
	while (chThdShouldTerminateX() == false){
		while (get_fail_to_score()== false){
			// If the sound of the whistle has been detected
			if(get_start_detected()==true){
				if (no_goal==false){
					set_start_detected(false);
					search=false;
				}
				else{
					left_motor_set_speed(SEARCH_SPEED);
					right_motor_set_speed(-SEARCH_SPEED);
					unblock_ball(&counter); // Unblock the ball if it's stuck in a corner
					search=true;
				}
			}
			dist= VL53L0X_get_dist_mm(); // Get the distance detected by the Time-of-Flight in mm
			// If the ball is at a certain distance or less from the robot and there's no goal detected and is not blocked
			if(dist<BALL_IN_THE_AREA && search==true && counter < 10){
				left_motor_set_pos(dist * NSTEP_ONE_TURN / WHEEL_PERIMETER); // Left motor stops after reaching the ball
				right_motor_set_pos(dist * NSTEP_ONE_TURN / WHEEL_PERIMETER); // Right motor stops after reaching the ball
				left_motor_set_speed(ATTACK_SPEED);
				right_motor_set_speed(ATTACK_SPEED);
			}
			// If the sound of the ball in the goal has been detected
			if (get_start_celeb() == true){
				left_motor_set_speed(CELEB_SPEED_LEFT);
				right_motor_set_speed(CELEB_SPEED_RIGHT);
				no_goal=false;
			}
			chThdSleepMilliseconds(SLEEP_THD_SEARCH);
		}
	}
}

/********************************** THREAD CALL FUNCTION ***********************************/

void start_search(void){
	chThdCreateStatic(waSEARCHThd,
	                     sizeof(waSEARCHThd),
	                     NORMALPRIO,
	                     SEARCHThd,
	                     NULL);
}

// Return the no_goal value

bool get_no_goal(void){
	return no_goal;
}

/*
*  Function to change the value of the static variable no_goal
*  params :
*  bool val : Tells the new value that we desire to set for the variable no_goal
*/

void set_no_goal(bool val){
	no_goal = val;
}

