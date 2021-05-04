#include <ch.h>
#include <hal.h>
#include <main.h>

#include "motors.h"
#include "leds.h"
#include "chprintf.h"
#include "i2c_bus.h"
#include "usbcfg.h"
#include "audio_processing.h"
#include "sensors/VL53L0X/VL53L0X.h"



#define MOTOR_TIMER_FREQ 100000 // [Hz]
#define THRESV 650 // This is the speed under which the power save feature is active.
#define WHEEL_PERIMETER     130 // [mm]
#define NSTEP_ONE_TURN      1000 // number of steps for 1 turn of the motor
#define THREASHOLD_STOP 35
#define SEARCH_SPEED_LEFT 500
#define SEARCH_SPEED_RIGHT -500
#define ATTACK_SPEED 1000
#define STOP_SPEED 0
#define CELEB_SPEED_LEFT 750
#define CELEB_SPEED_RIGHT -750
#define SLEEP_THD_SEARCH 100
#define BALL_IN_THE_AREA 500

/************************SEARCH THREAD*********************/

static THD_WORKING_AREA(waSEARCHThd, 512);
static THD_FUNCTION(SEARCHThd, arg) {
	bool search=false;
	bool no_goal=true;
	uint16_t dist=0;
	chRegSetThreadName("SEARCHThd");
    /* Reader thread loop.*/
    while (chThdShouldTerminateX() == false && get_fail_to_score()== false){
			if(get_start_detected()==true){
				if (no_goal==false){
					set_start_detected(false);
					search=false;
				}
				else{
				left_motor_set_speed(SEARCH_SPEED_LEFT);
				right_motor_set_speed(SEARCH_SPEED_RIGHT);
				search=true;
				}
			}
			 dist= VL53L0X_get_dist_mm();
			if(dist<BALL_IN_THE_AREA && search==true ){
				left_motor_set_pos(dist * NSTEP_ONE_TURN / WHEEL_PERIMETER);
				right_motor_set_pos(dist * NSTEP_ONE_TURN / WHEEL_PERIMETER);
				left_motor_set_speed(ATTACK_SPEED);
				right_motor_set_speed(ATTACK_SPEED);
				search=false;
			}
			if (get_start_celeb() == true){
				left_motor_set_speed(CELEB_SPEED_LEFT);
				right_motor_set_speed(CELEB_SPEED_RIGHT);
				no_goal=false;
			}
			chThdSleepMilliseconds(SLEEP_THD_SEARCH);
    	}
    }


/***********THREAD CALL FUNCTION***********************/

void start_search(void){
	chThdCreateStatic(waSEARCHThd,
	                     sizeof(waSEARCHThd),
	                     NORMALPRIO,
	                     SEARCHThd,
	                     NULL);
}


