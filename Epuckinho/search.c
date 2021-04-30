#include <ch.h>
#include <hal.h>
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

/************************THREAD*********************/

static THD_WORKING_AREA(waSEARCHThd, 512);
static THD_FUNCTION(SEARCHThd, arg) {
	bool search=false;
	bool no_goal=true;
	uint16_t dist=0;
	chRegSetThreadName("SEARCHThd");
    /* Reader thread loop.*/
    while (chThdShouldTerminateX() == false ) {
			if(get_start_detected()==true){
				if (no_goal==false){
					set_start_detected(false);
					search=false;
				}
				else{
				left_motor_set_speed(500);
				right_motor_set_speed(-500);
				search=true;
				}
			}
			 dist= VL53L0X_get_dist_mm();
			if(dist<200 && search==true ){
				left_motor_set_speed(1000);
				right_motor_set_speed(1000);
				chThdSleepMilliseconds(dist * NSTEP_ONE_TURN / WHEEL_PERIMETER);
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				search=false;
				}
			if (get_start_celeb() == true){
										left_motor_set_speed(750);
										right_motor_set_speed(-750);
										no_goal=false;
										}
			chThdSleepMilliseconds(50);
    	}
    }


/***********THREAD CALL FUNCTION***********************/

void start_search(void){
	chThdCreateStatic(waSEARCHThd,
	                     sizeof(waSEARCHThd),
	                     NORMALPRIO+2,
	                     SEARCHThd,
	                     NULL);
}


