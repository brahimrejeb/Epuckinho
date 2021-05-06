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



//semaphore
//static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
//static float micRight_cmplx_input[2 * FFT_SIZE];
//static float micFront_cmplx_input[2 * FFT_SIZE];
//static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
//static float micRight_output[FFT_SIZE];
//static float micFront_output[FFT_SIZE];
//static float micBack_output[FFT_SIZE];
//start_detection flag
static bool start_detected = false;
static bool start_celeb = false;
static uint32_t time_start=0;

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ	 5 //we don't analyze before this index to not use resources for nothing
#define FREQ_START_L  143.2  //2200Hz
#define FREQ_START_H  149.72 //2300Hz
#define MAX_FREQ	 160	//we don't analyze after this index to not use resources for nothing
#define FREQ_CELEB_L 5.5 // 90Hz
#define FREQ_CELEB_H 6.5 //100Hz
/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 
	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	//START TO PLAY
	if(max_norm_index >= FREQ_START_L && max_norm_index <= FREQ_START_H){
		start_detected = true;
		time_start=chVTGetSystemTime();
	}
	//A GOAL IS DETECTED: CELEBRATE IT !
	if(max_norm_index >= FREQ_CELEB_L && max_norm_index <= FREQ_CELEB_H){
		start_celeb = true;
	}
}

/*
*	Callback called when the demodulation of the four microphones is done.
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
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		//micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		//micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		//micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		//micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		//micBack_cmplx_input[nb_samples] = 0;
		//micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		//doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		//doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		//doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		//arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			//chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;
		sound_remote(micLeft_output);
	}
}

/*void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}*/

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	/*else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}*/
	else{
		return NULL;
	}
}

bool get_start_detected(void){
	return start_detected ;
}

void set_start_detected(bool state){
	start_detected=state;
}
bool get_start_celeb (void){
	return start_celeb ;
}

void set_start_celeb (bool state){
	start_celeb =state;
}
uint32_t get_time_start(void){
	return time_start;
}
