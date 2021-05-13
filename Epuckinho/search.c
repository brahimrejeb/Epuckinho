
/********************************** INCLUDES ***********************************/

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

/********************************** MAGIC NUMBERS ***********************************/

#define MOTOR_TIMER_FREQ 100000 // in [Hz]
#define THRESV 650 // This is the speed under which the power save feature is active
#define WHEEL_PERIMETER 130 // in [mm]
#define NSTEP_ONE_TURN 1000 // Number of steps for 1 turn of the motor
#define SEARCH_SPEED 500 // Constant speed in step/s of the robot when turning to find the ball
#define ATTACK_SPEED 1000 // Constant speed in step/s of the robot when moving towards the ball
#define STOP_SPEED 0 // Halt speed
#define CELEB_SPEED_LEFT 750 // Constant left speed in step/s of the robot when turning to celebrate the goal
#define CELEB_SPEED_RIGHT -750 // Constant right speed in step/s of the robot when turning to celebrate the goal
#define SLEEP_THD_SEARCH 100 // Sleep time of the search thread
#define BALL_IN_THE_AREA 500 // Ball distance in [mm] from which the robot starts to move towards it
#define BALL_PROX 100 // Ball distance in [mm] from the IR sensors from which the robot calibrates its sense of rotation

/********************************** SEARCH CONTROL FUNCTION ***********************************/

void search_control(void){
	// If IR2 or IR3 or IR4 detects the ball then turn right
	if (get_prox(1)> BALL_PROX || get_prox(2)> BALL_PROX || get_prox(3)> BALL_PROX){
			left_motor_set_speed(SEARCH_SPEED);
			right_motor_set_speed(-SEARCH_SPEED);
	}
	// If IR5 or IR6 or IR7 detects the ball then turn left
	else if (get_prox(4)> BALL_PROX|| get_prox(5)> BALL_PROX || get_prox(6)> BALL_PROX){
		left_motor_set_speed(-SEARCH_SPEED);
		right_motor_set_speed(SEARCH_SPEED);
	}
	// turn right by default
	else {
		left_motor_set_speed(SEARCH_SPEED);
		right_motor_set_speed(-SEARCH_SPEED);
	}
}

/********************************** SEARCH THREAD ***********************************/

static THD_WORKING_AREA(waSEARCHThd, 512);
static THD_FUNCTION(SEARCHThd, arg){
	(void)arg;
	chRegSetThreadName("SEARCHThd");
	bool search=false;
	bool no_goal=true;
	uint16_t dist=0;
	// Thread loop while there's no termination request and the game duration didn't exceed 15s
    while (chThdShouldTerminateX() == false && get_fail_to_score()== false){
    	// If the sound of the whistle has been detected
    	if(get_start_detected()==true){
    		if (no_goal==false){
    			set_start_detected(false);
				search=false;
			}
			else{
				search_control();
				search=true;
			}
		}
		dist= VL53L0X_get_dist_mm(); // Get the distance detected by the Time-of-Flight in mm
		// If the ball is at a certain distance or less from the robot and there's no goal detected
		if(dist<BALL_IN_THE_AREA && search==true ){
			left_motor_set_pos(dist * NSTEP_ONE_TURN / WHEEL_PERIMETER); // Left motor stops after reaching the ball
			right_motor_set_pos(dist * NSTEP_ONE_TURN / WHEEL_PERIMETER); // Right motor stops after reaching the ball
			left_motor_set_speed(ATTACK_SPEED); // Motor speed when moving towards the ball
			right_motor_set_speed(ATTACK_SPEED); // Motor speed when moving towards the ball
			//search=false;
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

/********************************** THREAD CALL FUNCTION ***********************************/

void start_search(void){
	chThdCreateStatic(waSEARCHThd,
	                     sizeof(waSEARCHThd),
	                     NORMALPRIO,
	                     SEARCHThd,
	                     NULL);
}



