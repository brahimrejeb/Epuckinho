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
static bool fail_to_score = false;

#define LED2_RGB 0
#define LED4_RGB 1
#define LED6_RGB 2
#define LED8_RGB 3
#define COLOR_LED_R 10
#define COLOR_LED_G 5
#define COLOR_LED_B 8
#define SLEEP_THD 1000
#define BLINK_MODE 2
#define GAME_OVER 15000 //in ms = 10s to score (the match ends and Epuckinho loses)

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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
	set_rgb_led(LED8_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	set_rgb_led(LED6_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	set_rgb_led(LED4_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	set_rgb_led(LED2_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	set_body_led(BLINK_MODE);
	playMelody(WE_ARE_THE_CHAMPIONS, ML_SIMPLE_PLAY, NULL);
}
void game_over(void)
{
	 fail_to_score = true;
	 playMelody(MARIO_DEATH, ML_SIMPLE_PLAY, NULL);
	 left_motor_set_speed(STOP_SPEED);
	 right_motor_set_speed(STOP_SPEED);
	 set_rgb_led(LED8_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	 set_rgb_led(LED6_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	 set_rgb_led(LED4_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
	 set_rgb_led(LED2_RGB ,COLOR_LED_R,COLOR_LED_G,COLOR_LED_B);
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
    spi_comm_start();
    //inits the motors
    motors_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start();
    calibrate_ir();

    //starts ToF
    VL53L0X_start();
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
    //motors and ToF activated once we detect start sound to search the ball
    start_search();
    //starts the melody
    dac_start();
    playMelodyStart();


    /* Infinite loop. */
    while (1) {
       // chprintf((BaseSequentialStream *)&SD3, "%4d", get_prox(1));
       // chprintf((BaseSequentialStream *)&SD3, "%4d", get_prox(6));
    	if (get_start_celeb()==true && fail_to_score ==false ){
    		celebrate();
    	}
    	//if (chVTGetSystemTime()-get_time_start() > GAME_OVER && get_start_detected()==true){
    		//game_over();
    	//}
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
