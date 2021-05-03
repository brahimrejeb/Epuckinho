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
#include "leds.h"
#include "search.h"
#include "audio_processing.h"
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
static bool fail_to_score = false;

#define LED_RGB_0 0
#define LED_RGB_1 1
#define LED_RGB_2 2
#define LED_RGB_3 3
#define COLOR_LED_R 10
#define COLOR_LED_G 5
#define COLOR_LED_B 8
#define SLEEP_THD 1000
#define BLINK_MODE 2
#define GAME_OVER 10000 //in ms = 10s to score (the match ends and Epuckinho loses)

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

void celebrate(void)
{
	set_rgb_led(LED_RGB_3 ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	set_rgb_led(LED_RGB_2 ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	set_rgb_led(LED_RGB_1 ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	set_rgb_led(LED_RGB_0 ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	set_body_led(BLINK_MODE);
	playMelody(WE_ARE_THE_CHAMPIONS, ML_SIMPLE_PLAY, NULL);
}
void game_over(void)
{
	 fail_to_score = true;
	 playMelody(MARIO_DEATH, ML_SIMPLE_PLAY, NULL);
	 left_motor_set_speed(STOP_SPEED);
	 right_motor_set_speed(STOP_SPEED);
	 set_rgb_led(LED_RGB_3 ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	 set_rgb_led(LED_RGB_2 ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	 set_rgb_led(LED_RGB_1 ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	 set_rgb_led(LED_RGB_0 ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	 set_front_led(BLINK_MODE);
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //start SPI
    dac_start(); //à verifier
    spi_comm_start();
    //inits the motors
    motors_init();
    //starts ToF
    VL53L0X_start();
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
    //motors and ToF activated once we detect start sound to search the ball
    start_search();
    //starts the melody
    playMelodyStart();


    /* Infinite loop. */
    while (1) {
    	if (get_start_celeb()==true && fail_to_score ==false ){
    		celebrate();
    	}
    	if (chVTGetSystemTime()-get_time_start() > GAME_OVER && get_start_detected()==true){
    		game_over();
    	}
    	chThdSleepMilliseconds(SLEEP_THD);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
bool get_fail_to_score(void){
	return fail_to_score;
}


/*Github switch*/
