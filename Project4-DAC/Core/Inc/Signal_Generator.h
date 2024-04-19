/*
 * Signal_Generator.h
 *
 *  Created on: Nov 6, 2023
 *      Author: tcber
 */

#ifndef INC_SIGNAL_GENERATOR_H_
#define INC_SIGNAL_GENERATOR_H_

#include "main.h"
#include "math.h"

#define SAMPLES (256)

enum waveform {
	RECTANGLE,
	TRIANGLE,
	SINE,
	ARBITRARY
};

struct user_command{
	int channel;
	float frequency;
	float minv;
	float maxv;
	int noise;
	char wave;
};

uint16_t get_noise(int noise);

void sig_gen(struct user_command *cmd, RNG_HandleTypeDef *hrng,DAC_HandleTypeDef *dac );

void init_generator(float maxv, float minv, int noise, RNG_HandleTypeDef *hrng,int channel);

void init_DC(float maxv, float minv, int noise, RNG_HandleTypeDef *hrng,int channel);

void init_triangle(float maxv, float minv, int noise, RNG_HandleTypeDef *hrng, int channel);

void init_rectangle(float maxv, float minv, int noise, RNG_HandleTypeDef *hrng, int channel);

void init_ekg(int channel);

#endif /* INC_SIGNAL_GENERATOR_H_ */
