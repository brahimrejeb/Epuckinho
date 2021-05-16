
/********************************** INCLUDE ***********************************/

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <search.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

/********************************** STATIC GLOBAL VARIABLES ***********************************/

static float micLeft_cmplx_input[2 * FFT_SIZE]; // 2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_output[FFT_SIZE]; // Arrays containing the computed magnitude of the complex numbers
static bool start_detected = false; // Start ball detection flag
static bool start_celeb = false; // Start celebration flag
static uint32_t time_start=0; // The system time when the robot starts searching for the ball in ticks

/********************************** CONSTANTS ***********************************/

#define MIN_VALUE_THRESHOLD	10000 
#define MIN_FREQ 20 // We don't analyze before this index to not use resources for nothing
#define FREQ_START_L 143.2  // The minimum frequency that the robot should detect to start the ball search: 2200Hz
#define FREQ_START_H 149.72 // The maximum frequency that the robot should detect to start the ball search: 2300Hz
#define MAX_FREQ 160 // We don't analyze after this index to not use resources for nothing
#define FREQ_CELEB_L 35 // The minimum frequency that the robot should detect to celebrate: 540Hz
#define FREQ_CELEB_H 40 // The maximum frequency that the robot should detect to celebrate: 620Hz

/* Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/

void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 
	// Search for the highest peak of frequency
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	// Start to play if a whistle sound is detected
	if(max_norm_index >= FREQ_START_L && max_norm_index <= FREQ_START_H){
		start_detected = true; // The start signal is detected and the game begins
		time_start=chVTGetSystemTime(); // Return the system time in ticks when game begins
	}
	// A goal is detected : celebrate it !
	if(max_norm_index >= FREQ_CELEB_L && max_norm_index <= FREQ_CELEB_H){
		start_celeb = true;
	}
}

/*
*	Callback called when the demodulation of the left microphone is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/

void processAudioData(int16_t *data, uint16_t num_samples){
	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/
	static uint16_t nb_samples = 0;
	// Loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		// Construct an array of complex numbers. Put 0 to the imaginary part
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		nb_samples++;
		micLeft_cmplx_input[nb_samples] = 0;
		nb_samples++;
		// Stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}
	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT processing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		nb_samples = 0;
		sound_remote(micLeft_output);
	}
}
float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if (name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else{
		return NULL;
	}
}

/*
*  Function to return the value of the static variable start_detected
*/

bool get_start_detected(void){
	return start_detected ;
}

/*
*  Function to change the value of the static variable start_detected
*  params :
*  bool state : Tells the new value that we desire to set for the variable start_detected
*/

void set_start_detected(bool state){
	start_detected=state;
}

/*
*  Function to return the value of the static variable start_celeb
*/

bool get_start_celeb (void){
	return start_celeb ;
}

/*
*  Function to change the value of the static variable start_celeb
*  params :
*  bool state : Tells the new value that we desire to set for the variable start_celeb
*/

void set_start_celeb (bool state){
	start_celeb =state;
}

/*
*  Function to return the value of static variable time_start of type uint32_t
*/

uint32_t get_time_start(void){
	return time_start;
}

