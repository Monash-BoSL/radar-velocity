#include "src\arduinoFFTfix.h"
#include "src\RadarService.h"


#define PEAK_THRESHOLD 0.5
#define SWEEP_RATE	1E6		//(mHZ)
#define DISTANCE_RES 60	   //(mm)
#define SWEEP_LENGTH  560 //(mm)
#define SWEEPS 64

arduinoFFTfix FFTfix = arduinoFFTfix();

RadarService Radar(PEAK_THRESHOLD, SWEEP_RATE, DISTANCE_RES, SWEEP_LENGTH, SWEEPS); 

int16_t data[SWEEPS*(SWEEP_LENGTH/DISTANCE_RES)];//will need to subtract 2<<15 from data


float distance;
float velocity;
float amplitude;


void setup(){}

void loop(){
	get_velocity();
}

void get_velocity(){
	Radar.measure_sparce(240);
	Radar.read_buf(data);
	eval_data();
}

void eval_data(){
	float scales[Radar.distance_bins];
	
	//remove dc component
	for(int i = 0; i<Radar.distance_bins; i++){
		uint32_t accumulator = 0;
		for(int j = 0; j<Radar.sweeps; j++){
			accumulator += data[j*Radar.sweeps+i];
		}
		uint32_t average = accumulator/Radar.sweeps;
		for(int j = 0; j<Radar.sweeps; j++){
			data[j*Radar.sweeps+i] -= average;
		}
	}
	
	//do fft on each row
	for(int i = 0; i<Radar.distance_bins; i++){
		//note that this method of coping data to another array to do the FFT becomes less relatively memory efficient the fewer distance bins you have.
		int16_t real[Radar.sweeps];
		int16_t imag[Radar.sweeps];
		//set real componet of FFT to data
		for (int j = 0; j < Radar.sweeps; j++) {
		  real[j] = data[j*Radar.sweeps+i];
		}
		//set imaginary componet of FFT to zero
		for (int j = 0; j < Radar.sweeps; j++) {
		  imag[j] = 0;
		}
		
		//Compute FFT
		scales[i] = FFTfix.RangeScaling(real, Radar.sweeps);
		FFTfix.Windowing(real, Radar.sweeps, FFT_FORWARD);
		FFTfix.Compute(real, imag, Radar.sweeps, FFT_FORWARD);
		FFTfix.ComplexToMagnitude(real, imag, Radar.sweeps);
		
		//load data back in to data array
		//set real componet of FFT to data
		for (int j = 0; j < Radar.sweeps/2; j++) {
		  data[j*Radar.sweeps+i] = real[j];
		}
		for (int j = Radar.sweeps/2; j < Radar.sweeps; j++) {
		  data[j*Radar.sweeps+i] = 0;
		}
	}
	//the data has been normalised in each row but to make meaningful comparisons we need in normalised across the entire array, we shall do this now
	
	//we will normalise by scaling everything realtive to the smallest scaling
	float min_scale = 0;
	for(int i =0; i<Radar.distance_bins; i++){
		if (scales[i] < min_scale){
			min_scale = scales[i];
		}
	}
	
	//now just multiply each row by the relevant scaling factor
	for(int i =0; i<Radar.distance_bins; i++){
		float scaling_factor = min_scale/scales[i]; 
		for(int j = 0; i<Radar.distance_bins/2; i++){
			data[j*Radar.distance_bins+i] *= scaling_factor;
		}
	}
	
	
	//now we find the maximum value in the data
	uint16_t apex = 0;
	for(int i =0; i< Radar.distance_bins*Radar.sweeps/2; i++){
		if (data[i] > apex){
			apex = data[i];
		}
	}
	
	//now we set all the data below the threshold to zero
	for(int i =0; i< Radar.distance_bins*Radar.sweeps/2; i++){
		if (data[i] < apex*Radar.peak_threshold){
			data[i] = 0;
		}
	}
	
	//the center of mass of the image need to be computed
	float mass = 0;
	float weights_d = 0; 
	float weights_s = 0;
	
	for(int i = 0; i<Radar.distance_bins*Radar.sweeps/2; i++){
		mass += data[i];
		weights_d += data[i]* (i%Radar.distance_bins);
		weights_s += data[i]* (i/Radar.distance_bins);	
	}
	
	weights_d /= mass;
	weights_s /= mass;
	
	distance = Radar.distance_res*weights_d + Radar.start_distance;	//converts data to distance (mm hopefully)
	velocity = Radar.sweep_rate/Radar.sweeps * 2.445E-6; //converts data to velocity (check units)
	amplitude = apex*min_scale;
	
	
}