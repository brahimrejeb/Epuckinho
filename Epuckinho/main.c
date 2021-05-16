
/********************************** INCLUDES ***********************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "spi_comm.h"

#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <chvt.h>
#include <motors.h>
#include <audio/microphone.h>
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/proximity.h"
#include "leds.h"
#include "search.h"
#include "audio_processing.h"
#include <fft.h>
#include <communications.h>
#include <arm_math.h>

/********************************** STATIC GLOBAL VARIABLES ***********************************/

static bool fail_to_score = false; //verify if the simulation has passed 15 seconds

/********************************** MAGIC NUMBERS ***********************************/

#define LED2_RGB 0
#define LED4_RGB 1
#define LED6_RGB 2
#define LED8_RGB 3
#define COLOR_INTENSITY 10	// Intensity of the RGB color
#define LED_GREEN_R 0 //Intensity of red in RGB led for green color
#define LED_GREEN_G 10 //Intensity of green in RGB led for green color
#define LED_GREEN_B 0 //Intensity of blue in RGB led for green color
#define SLEEP_THD 1000	// Sleep duration for the main thread in ms
#define BLINK_MODE 2	// Blink mode
#define GAME_OVER 20000 // in ms = 20s to score , converted to ticks with MS2ST (the match ends and Epuckinho loses)
#define STACK_CHK_GUARD 0xe2dee396

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Start the UART3 communication
static void serial_start(void){
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};
	sdStart(&SD3, &ser_cfg);
}


/* Function used to celebrate when the ball enters the goal and show the score with RGB leds
* params :
* uint8_t* nb_goals : a counter value that is incremented when a goal is scored. We display the score of the game
* by RGB leds. If two goals has been scored, led8 and led6 will be set on.
* Final celebration includes playing melody and blinky mode of the body led.
*/
void celebrate(uint8_t *nb_goals){
	set_start_celeb(false);
	*nb_goals+=1;
	//add a goal to the score
	//show the score with leds and prepare to another shot if goals<4
	//the score is the number of RGB leds on .
	//the final celebration includes playing the melody WE_ARE_THE_CHAMPIONS and blinking body_led
	switch(*nb_goals){
		case 1 :
			set_rgb_led(LED8_RGB,LED_GREEN_R,LED_GREEN_G,LED_GREEN_B);
			set_start_detected(false);
			set_fail_to_score(false);
			set_no_goal(true);
			break;
		case 2 :
			set_rgb_led(LED6_RGB, LED_GREEN_R,LED_GREEN_G,LED_GREEN_B);
			set_start_detected(false);
			set_fail_to_score(false);
			set_no_goal(true);
			break;
		case 3 :
			set_rgb_led(LED4_RGB, LED_GREEN_R,LED_GREEN_G,LED_GREEN_B);
			set_start_detected(false);
			set_fail_to_score(false);
			set_no_goal(true);
			break;
		default :
			set_rgb_led(LED2_RGB, LED_GREEN_R,LED_GREEN_G,LED_GREEN_B);
			set_body_led(BLINK_MODE); // Blinky mode for the body led in green
			playMelody(WE_ARE_THE_CHAMPIONS, ML_SIMPLE_PLAY, NULL); // Play celebration melody
			break;
	}
}

// Failure when the simulation time exceeds 20 seconds
void game_over(void){
	 fail_to_score = true; // Report the failure of the robot and end of game
	 left_motor_set_speed(STOP_SPEED);
	 right_motor_set_speed(STOP_SPEED);
	 toggle_rgb_led(LED8_RGB, BLUE_LED, COLOR_INTENSITY);
	 toggle_rgb_led(LED6_RGB, BLUE_LED, COLOR_INTENSITY);
	 toggle_rgb_led(LED4_RGB, BLUE_LED, COLOR_INTENSITY);
	 toggle_rgb_led(LED2_RGB, BLUE_LED, COLOR_INTENSITY);
	 set_front_led(BLINK_MODE); // Blinky mode for the front led in red
	 playMelody(MARIO_DEATH, ML_SIMPLE_PLAY, NULL); // Play failure melody
}

/********************************** MAIN FUNCTION ***********************************/

int main(void){
	halInit();
    chSysInit(); // ChibiOS initialization
    serial_start(); // Start the serial communication
    usb_start(); // Start the USB communication
    spi_comm_start(); // Start SPI
    motors_init(); // Motors initialization
    messagebus_init(&bus, &bus_lock, &bus_condvar);  // Messagebus initialization
    proximity_start();  // Start the IR detection
    calibrate_ir(); // Calibration of the IR sensors
    VL53L0X_start();  // Start the Time-of-Flight thread
    mic_start(&processAudioData); // Start the microphones processing thread
    start_search(); // Start the search thread
    // Start the melody processing thread
    //Starts the digital-to-analog converter
    dac_start();
    // Start the melody processing thread
    playMelodyStart();
    static uint8_t nb_goals=0;
    /* Infinite loop. */
    while (1) {
    	if (get_start_celeb()==true && fail_to_score ==false ){
    		celebrate(&nb_goals);//celebrates,shows score and prepares for another shot if nb_goals <4
    	}
    	if (chVTGetSystemTime()-get_time_start() > MS2ST(GAME_OVER) && get_start_detected()==true){
    		game_over(); // End of the game if the simulation time has exceeded 30 seconds
    	}
    	chThdSleepMilliseconds(SLEEP_THD); // Sleep of the main thread
    }
}

// return the fail_to_score value
bool get_fail_to_score(void){
	return fail_to_score;
}

// Set the fail_to_score value
void set_fail_to_score(bool val){
	fail_to_score=val;
}
/********************************** HALT CHIBIOS IN CASE OF ERRORS ***********************************/

uintptr_t __stack_chk_guard = STACK_CHK_GUARD;
void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}




/*Github switch*/
