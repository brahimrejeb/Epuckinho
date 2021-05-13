
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
#define COLOR_LED_R 10	// Intensity of the red color
#define COLOR_LED_G 5	// Intensity of the green color
#define COLOR_LED_B 8	// Intensity of the blue color
#define SLEEP_THD 1000	// Sleep duration for the main thread in ms
#define BLINK_MODE 2	// Blink mode
#define GAME_OVER 15000 // in ms = 15s to score (the match ends and Epuckinho loses)
#define STACK_CHK_GUARD 0xe2dee396

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Start the SD3 communication
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};
	sdStart(&SD3, &ser_cfg);
}

// Celebration when the ball enters the goal
void celebrate(void)
{
	set_rgb_led(LED8_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B); // Display L8
	set_rgb_led(LED6_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B); // Display L6
	set_rgb_led(LED4_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B); // Display L4
	set_rgb_led(LED2_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B); // Display L2
	set_body_led(BLINK_MODE); // Blinky mode for the body led in green
	playMelody(WE_ARE_THE_CHAMPIONS, ML_SIMPLE_PLAY, NULL); // Play celebration melody
}

// Failure when the simulation time exceeds 15 seconds
void game_over(void)
{
	 fail_to_score = true; // Report the failure of the robot and end of game
	 left_motor_set_speed(STOP_SPEED); // Halt speed for the left motor
	 right_motor_set_speed(STOP_SPEED); // Halt speed for the right motor
	 set_rgb_led(LED8_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B); // Display L8
	 set_rgb_led(LED6_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B); // Display L6
	 set_rgb_led(LED4_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B); // Display L4
	 set_rgb_led(LED2_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B); // Display L2
	 set_front_led(BLINK_MODE); // Blinky mode for the front led in red
	 playMelody(MARIO_DEATH, ML_SIMPLE_PLAY, NULL); // Play failure melody
}

/********************************** MAIN FUNCTION ***********************************/

int main(void)
{
	halInit();
    chSysInit(); // ChibiOS initialization
    serial_start(); // Start the serial communication
    usb_start(); // Start the USB communication
    spi_comm_start(); // Start SPI
    motors_init(); // Motors initialization
    messagebus_init(&bus, &bus_lock, &bus_condvar);  // Messagebus initialization
    proximity_start();  // Start the IR detection
    calibrate_ir(); // Calibration of the IR sensors
    VL53L0X_start();  // Start the Time-of-Flight processing thread
    mic_start(&processAudioData); // Start the microphones processing thread
    start_search(); // Start the search processing thread
    // Start the melody processing thread
    dac_start();
    playMelodyStart();
    /* Infinite loop. */
    while (1) {
    	if (get_start_celeb()==true && fail_to_score ==false ){
    		celebrate();
    	}
    	if (chVTGetSystemTime()-get_time_start() > GAME_OVER && get_start_detected()==true){
    		game_over(); // End of the game if the simulation time has exceeded 15 seconds
    	}
    	chThdSleepMilliseconds(SLEEP_THD); // Sleep of the main thread
    }
}

// return the fail_to_score value
bool get_fail_to_score(void){
	return fail_to_score;
}

/********************************** HALT CHIBIOS IN CASE OF ERRORS ***********************************/

uintptr_t __stack_chk_guard = STACK_CHK_GUARD;
void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}




/*Github switch*/
