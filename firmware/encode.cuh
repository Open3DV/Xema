#ifndef ENCODE_CUDA_CUH
#define ENCODE_CUDA_CUH
#pragma once
// #include "memory_management.cuh"
#include <device_launch_parameters.h>
#include <device_functions.h>
#include <cuda_runtime.h>  
#include <cuda_texture_types.h>
#include <texture_types.h>  
#include <iostream>
#include <stdint.h>
#include <vector>
#include "system_config_settings.h"
#include <opencv2/core.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "protocol.h"
#include "easylogging++.h"

//kernel
__global__ void kernel_four_step_phase_shift(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
float * const d_out, float * const confidence); 

__global__ void kernel_six_step_phase_shift(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
unsigned char* const d_in_4,unsigned char* const d_in_5, float * const d_out, float * const confidence);

__global__ void kernel_six_step_phase_shift_with_average(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
unsigned char* const d_in_4,unsigned char* const d_in_5, float * const d_out, float * const confidence,unsigned char* const average,unsigned char* const brightness);

__global__ void kernel_computre_global_light(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
unsigned char* const d_in_4,unsigned char* const d_in_5, float b,unsigned char * const direct_out,unsigned char * const global_out,unsigned char * const uncertain_out);

__global__ void kernel_six_step_phase_shift_global(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
unsigned char* const d_in_4,unsigned char* const d_in_5, float * const d_out, float * const confidence,float b,unsigned char * const direct_out,unsigned char * const global_out);
 
__global__ void kernel_unwrap_variable_phase(int width,int height,float * const d_in_wrap_abs, float * const d_in_wrap_high,float const rate,float threshold, float * const d_out);

__global__ void kernel_unwrap_variable_phase_base_confidence(int width,int height,float * const d_in_wrap_abs, float * const d_in_wrap_high,float const rate,float threshold,float fisher_rate, float* const d_fisher_confidece_mask, float * const d_out);
 
__global__ void kernel_normalize_phase(int width,int height,float * const d_in_unwrap_map, float rate,  float * const d_out_normal_map);

__global__ void kernel_merge_six_step_phase_shift(unsigned short * const d_in_0, unsigned short * const d_in_1, unsigned short * const d_in_2, 
	unsigned short * const d_in_3,unsigned short* const d_in_4,unsigned short* const d_in_5,int repetition_count,
	uint32_t img_height, uint32_t img_width,float * const d_out, float * const confidence);

__global__ void kernel_merge_computre_global_light(int width,int height,unsigned short * const d_in_0, unsigned short * const d_in_1, unsigned short *  d_in_2, unsigned short * const d_in_3,
unsigned short* const d_in_4,unsigned short* const d_in_5, int repetition_count,float b,unsigned char * const direct_out,unsigned char * const global_out,unsigned char * const uncertain_out);
    
__global__ void kernel_merge_four_step_phase_shift(unsigned short * const d_in_0, unsigned short * const d_in_1, unsigned short * const d_in_2, 
	unsigned short * const d_in_3,int repetition_count,uint32_t img_height, uint32_t img_width,float * const d_out, float * const confidence);

__global__ void kernal_convolution_2D(int width,int height, unsigned char *input, unsigned char *output, float *mask, int masksize);

__global__ void kernal_convolution_2D_short(int width,int height, unsigned short *input, unsigned short *output, float *mask, int masksize);

__global__ void kernel_six_step_phase_rectify(int width,int height,float* computeWrapedPhase_good, float* computeWrapedPhase_bad, float* computeWrapedPhase_original);

__global__ void kernel_four_step_phase_shift(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
float * const d_out, float * const confidence); 

#endif