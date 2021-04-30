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
    dac_start();
    spi_comm_start();
    //inits the motors
    motors_init();
    //starts ToF
    VL53L0X_start();
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
    //motors and tof activated once we detect start sound to search the ball
    start_search();
    //starts the melody
    playMelodyStart();


    /* Infinite loop. */
    while (1) {
    	if (get_start_celeb()==true && fail_to_score ==false ){
    		//set_front_led(1);
    		set_rgb_led(3,10,5,8);
    		set_rgb_led(2,10,5,8);
    		set_rgb_led(1,10,5,8);
    		set_rgb_led(0,10,5,8);
    		set_body_led(2);
    		playMelody(WE_ARE_THE_CHAMPIONS, ML_SIMPLE_PLAY, NULL);
    	}
    	if (chVTGetSystemTime()-get_time_start() > 10000 && get_start_detected()==true){
    		 fail_to_score = true;
    		 playMelody(MARIO_DEATH, ML_SIMPLE_PLAY, NULL);
    		 left_motor_set_speed(0);
    		 right_motor_set_speed(0);
    		 set_rgb_led(3,10,5,8);
    		 set_rgb_led(2,10,5,8);
    		 set_rgb_led(1,10,5,8);
    		 set_rgb_led(0,10,5,8);
    		 set_front_led(2);
    	}
    	chThdSleepMilliseconds(1000);
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
