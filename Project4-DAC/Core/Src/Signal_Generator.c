/*
 * Signal_Generator.c
 *
 *  Created on: Nov 6, 2023
 *      Author: tcber
 */


#include "Signal_Generator.h"


int NUM_SAMPLES = SAMPLES;

uint32_t waveform[SAMPLES];

uint32_t waveform2[SAMPLES];


static uint32_t ekg[256] = {
    1690, 1680, 1680, 1669, 1648, 1648, 1648, 1680, 1680, 1690, 1680, 1680, 1680, 1680, 1680, 1658,
    1690, 1690, 1712, 1690, 1690, 1680, 1669, 1669, 1680, 1690, 1690, 1680, 1669, 1669, 1669, 1680,
    1680, 1680, 1669, 1669, 1680, 1690, 1690, 1680, 1690, 1690, 1680, 1690, 1690, 1712, 1680, 1680,
    1658, 1648, 1648, 1648, 1669, 1669, 1680, 1690, 1690, 1701, 1680, 1680, 1669, 1680, 1680, 1680,
    1701, 1701, 1701, 1690, 1690, 1701, 1712, 1712, 1722, 1712, 1712, 1690, 1669, 1669, 1680, 1690,
    1690, 1690, 1733, 1733, 1765, 1776, 1861, 1882, 1936, 1936, 1968, 1989, 1989, 2032, 2053, 2053,
    2085, 2149, 2069, 2080, 2058, 2058, 1930, 1930, 1845, 1824, 1792, 1872, 1840, 1754, 1754, 1722,
    1680, 1680, 1680, 1637, 1637, 1637, 1637, 1637, 1626, 1648, 1648, 1637, 1605, 1605, 1616, 1680,
    1680, 1765, 1776, 1861, 2042, 2106, 2021, 1776, 2480, 2400, 2176, 1632, 1637, 1360, 933, 928,
    1962, 1962, 2042, 2149, 3141, 3141, 2320, 1200, 1200, 1392, 1669, 1669, 1658, 1701, 1701, 1701,
    1701, 1701, 1722, 1690, 1690, 1690, 1680, 1680, 1690, 1690, 1690, 1669, 1669, 1669, 1701, 1733,
    1733, 1754, 1744, 1744, 1733, 1733, 1733, 1722, 1765, 1765, 1765, 1733, 1733, 1733, 1722, 1722,
    1701, 1690, 1690, 1701, 1690, 1690, 1701, 1701, 1701, 1701, 1722, 1722, 1712, 1722, 1722, 1733,
    1733, 1733, 1733, 1712, 1712, 1712, 1733, 1733, 1733, 1733, 1733, 1733, 1744, 1744, 1744, 1744,
    1744, 1744, 1733, 1733, 1722, 1722, 1722, 1722, 1722, 1722, 1733, 1722, 1722, 1722, 1722, 1722,
    1701, 1669, 1669, 1680, 1690, 1690, 1690, 1701, 1701, 1712, 1712, 1712, 1690, 1669, 1669, 1680
};
uint16_t get_noise(int noise){

	switch(noise){
		case(0):
			return (uint16_t)0x00;
			break;
		case(1):
			return (uint16_t)0x01;
			break;
		case(2):
			return (uint16_t)0x03;
			break;
		case(3):
			return (uint16_t)0x07;
			break;
		case(4):
			return (uint16_t)0x0F;
			break;
		case(5):
			return (uint16_t)0x1F;
			break;
		case(6):
			return (uint16_t)0x3F;
			break;
		case(7):
			return (uint16_t)0x7F;
			break;
		case(8):
			return (uint16_t)0xFF;
			break;
		case(9):
			return (uint16_t)0x1FF;
			break;
		case(10):
			return (uint16_t)0x3FF;
			break;
		case(11):
			return (uint16_t)0x7FF;
			break;
		case(12):
			return (uint16_t)0xFFF;
			break;
		default:
			return (uint16_t) 0;

	}

}
void init_DC(float maxv, float minv, int noise, RNG_HandleTypeDef *hrng,int channel){
	uint32_t random;
	uint16_t bits = get_noise(noise);
	if(channel == 1){
	for(int i = 0; i < NUM_SAMPLES; i++){
		HAL_RNG_GenerateRandomNumber(hrng, &random);
		uint16_t new_noise = (uint16_t)(((uint16_t)random) & bits);
		waveform[i] = (uint16_t) (minv*4096/3.3);
		waveform[i] |= new_noise;
	    }
	}
	else{
		for(int i = 0; i < NUM_SAMPLES; i++){
			HAL_RNG_GenerateRandomNumber(hrng, &random);
			uint16_t new_noise = (uint16_t)(random & bits);
			waveform2[i] = (uint16_t) (minv*4096/3.3);
			waveform2[i] |= new_noise;
		    }
	}

}

void init_generator(float maxv, float minv,int noise, RNG_HandleTypeDef *hrng, int channel) {
	uint32_t random;
	uint16_t bits = get_noise(noise);
	if(channel == 1){
	for(int i = 0; i < NUM_SAMPLES; i++){
		HAL_RNG_GenerateRandomNumber(hrng, &random);
		uint16_t new_noise = (uint16_t)(((uint16_t)random) & bits);
		waveform[i] = (uint16_t) (((sin(2.0*M_PI*((double) i)/(double)(NUM_SAMPLES-1)) * ((maxv-minv)/2)) + ((maxv+minv)/2))*4096/3.3);
		//waveform[i] = round(((sin(i*2*M_PI / 256) + 1)*((4096 / 3.3*(maxv - minv)) / 2)) + 4096 / 3.3 * minv);
		waveform[i] |= new_noise;
	    }
	}
	else{
		for(int i = 0; i < NUM_SAMPLES; i++){
			HAL_RNG_GenerateRandomNumber(hrng, &random);
			uint16_t new_noise = (uint16_t)(random & bits);
			waveform2[i] = (uint16_t) (((sin(2.0*M_PI*((double) i)/(double)(NUM_SAMPLES-1)) * ((maxv-minv)/2)) + ((maxv+minv)/2))*4096/3.3);
			waveform2[i] |= new_noise;
		    }
	}
}

void init_triangle(float maxv, float minv,int noise, RNG_HandleTypeDef *hrng, int channel){
	uint32_t random;
	uint16_t bits = get_noise(noise);
	if(channel == 1){
    for(int i = 0; i < NUM_SAMPLES; i++){
		HAL_RNG_GenerateRandomNumber(hrng, &random);
		uint16_t noise = (uint16_t)(random & bits);
        if(i < NUM_SAMPLES/2){
        	waveform[i] = (uint16_t) ((((float)(2*i)/(float)(NUM_SAMPLES-1)) * (maxv-minv) + minv)*4096/3.3);
        }else{
        	waveform[i] = (uint16_t) ((((float)(NUM_SAMPLES-1-(2*i))/(float)(NUM_SAMPLES-1)) * (maxv-minv) + maxv)*4096/3.3);
        }
    	waveform[i] |= noise;
    }
	}
	else{
	    for(int i = 0; i < NUM_SAMPLES; i++){
			HAL_RNG_GenerateRandomNumber(hrng, &random);
			uint16_t noise = (uint16_t)(random & bits);
	        if(i < NUM_SAMPLES/2){
	        	waveform2[i] = (uint16_t) ((((float)(2*i)/(float)(NUM_SAMPLES-1)) * (maxv-minv) + minv)*4096/3.3);
	        }else{
	        	waveform2[i] = (uint16_t) ((((float)(NUM_SAMPLES-1-(2*i))/(float)(NUM_SAMPLES-1)) * (maxv-minv) + maxv)*4096/3.3);
	        }
	    	waveform2[i] |= noise;
	    }
	}
}


void init_rectangle(float maxv, float minv,int noise, RNG_HandleTypeDef *hrng,int channel){
	uint32_t random;
	uint16_t bits = get_noise(noise);
	if(channel == 1){
    for(int i = 0; i < NUM_SAMPLES; i++){
		HAL_RNG_GenerateRandomNumber(hrng, &random);
		uint16_t noise = (uint16_t)(random & bits);
    	waveform[i] = (uint16_t) (((i < (NUM_SAMPLES/2))? minv : maxv)*4096/3.3);
    	waveform[i] |= noise;
    }
	}
	else{
	    for(int i = 0; i < NUM_SAMPLES; i++){
			HAL_RNG_GenerateRandomNumber(hrng, &random);
			uint16_t noise = (uint16_t)(random & bits);
	    	waveform2[i] = (uint16_t) (((i < (NUM_SAMPLES/2))? minv : maxv)*4096/3.3);
	    	waveform2[i] |= noise;
	    }
	}
}

void init_ekg(int channel){
	if(channel == 1){
		for(int i = 0; i < NUM_SAMPLES; i++){
			waveform[i] = ekg[i];
		}
	}
	else{
	    for(int i = 0; i < NUM_SAMPLES; i++){
	        waveform2[i] = ekg[i];
	    }
	}
}

void sig_gen(struct user_command *cmd, RNG_HandleTypeDef *hrng,DAC_HandleTypeDef *dac ){
	//Process user input
	if(cmd->channel == 1){
		HAL_DAC_Stop_DMA(dac, DAC_CHANNEL_1);
	}
	else{
		HAL_DAC_Stop_DMA(dac, DAC_CHANNEL_2);
	}
	switch(cmd->wave){
	case('R'):
		//Change to Rectangle wave
		init_rectangle(cmd->maxv, cmd->minv,cmd->noise, hrng, cmd->channel);
		break;
	case('S'):
		// Change to Sine wave

		if(cmd->frequency != 0.0){
			init_generator(cmd->maxv, cmd->minv,cmd->noise, hrng, cmd->channel);
		}
		else{
			init_DC(cmd->maxv, cmd->minv,cmd->noise, hrng, cmd->channel);
		}
		break;
	case('T'):
		//Change to Triangle
		init_triangle(cmd->maxv, cmd->minv,cmd->noise, hrng, cmd->channel);
		break;
	case('A'):
		//Change to ekg
		init_ekg(cmd->channel);
		break;
	default:
		break;
	}

	if(cmd->channel == 1){
		TIM2->ARR = round(80000000.0 / (float)(NUM_SAMPLES * (float)cmd->frequency));
		TIM2->EGR = TIM_EGR_UG;
		HAL_DAC_Start_DMA(dac, DAC_CHANNEL_1, (uint32_t*)waveform, SAMPLES, DAC_ALIGN_12B_R);
	}
	else{
		TIM5->ARR = (80000000.0 / (float)(NUM_SAMPLES * (float)cmd->frequency));
		TIM5->EGR = TIM_EGR_UG;
		HAL_DAC_Start_DMA(dac, DAC_CHANNEL_2, (uint32_t*)waveform2, SAMPLES, DAC_ALIGN_12B_R);
	}



}


