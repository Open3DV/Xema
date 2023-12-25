#ifndef MEMORY_MANAGEMENT_CUDA_CUH
#define MEMORY_MANAGEMENT_CUDA_CUH
#pragma once
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
#include "encode.cuh"
#include "reconstruct.cuh"
#include "merge_hdr.cuh"
#include "merge_repetition.cuh"
#include "filter_module.cuh"

#define SIZE_OF_CONVOLUTION_KERNAL 9
#define MAX_PATTERNS_NUMBER 36
#define MAX_WRAP_NUMBER 8
#define MAX_UNWRAP_NUMBER 2
#define LAST_STEPS_NUM 6
#define D_HDR_MAX_NUM 6 
#define D_REPETITIONB_MAX_NUM 10
#define FISHER_RATE_1 -6.61284856e-06
#define FISHER_RATE_2 4.52035763e-06
#define FISHER_RATE_3 -1.16182132e-05
#define FISHER_RATE_4 -2.89004663e-05
#define FISHER_CENTER_LOW -3.118464936149469e-05
#define FISHER_CENTER_RATE 2.0478921488348731e-07


__device__ int d_image_width_ = 0;
__device__ int d_image_height_ = 0;
 


/**********************************************************************/
//basic memory
__device__ unsigned char* d_patterns_list_[MAX_PATTERNS_NUMBER];
__device__ unsigned char* d_six_step_patterns_list_[LAST_STEPS_NUM];
__device__ unsigned char* d_six_step_patterns_convolved_list_[LAST_STEPS_NUM];
__device__ unsigned short* d_repetition_02_merge_patterns_convolved_list_[LAST_STEPS_NUM];  
__device__ float* d_wrap_map_list_[MAX_WRAP_NUMBER];
__device__ float* d_confidence_map_list_[MAX_WRAP_NUMBER];
__device__ float* d_unwrap_map_list_[MAX_UNWRAP_NUMBER];
__device__ float* d_six_step_pattern_convolution_phase_list_[2];

__device__ float* d_fisher_confidence_map;
__device__ float* d_convolution_kernal_map;
__device__ unsigned char* d_fisher_mask_;
__device__ unsigned char* d_mask_map_;
__device__ unsigned char* d_brightness_map_;
__device__ unsigned short* d_brightness_short_map_;
__device__ unsigned char* d_darkness_map_;
__device__ float* d_point_cloud_map_;
__device__ float* d_depth_map_; 
__device__ float* d_depth_map_temp_; 
__device__ float* d_triangulation_error_map_;
__device__ unsigned char* d_global_light_map_;
__device__ unsigned char* d_direct_light_map_;
__device__ unsigned char* d_uncertain_map_;
 
__device__ int load_calib_data_flag_ = 0;
//calib data
__device__ float* d_camera_intrinsic_= NULL;
__device__ float* d_project_intrinsic_= NULL;
__device__ float* d_camera_distortion_= NULL;
__device__ float* d_projector_distortion_= NULL;
__device__ float* d_rotation_matrix_= NULL;
__device__ float* d_translation_matrix_= NULL;


__device__ __constant__ unsigned char* d_minsw8_table_= NULL;
 
   
__device__ float d_baseline_ = 0; 
// 因为有多个查找表，在上传查找表前，先释放内存，防止内存泄漏
__device__ float* d_single_pattern_mapping_ = NULL;
__device__ float* d_single_pattern_minimapping_ = NULL;
__device__ float* d_xL_rotate_x_ = NULL;
__device__ float* d_xL_rotate_y_ = NULL; 
__device__ float* d_R_1_ = NULL; 

__device__ float* d_undistort_map_x_ = NULL;
__device__ float* d_undistort_map_y_ = NULL; 
/**********************************************************************/
//hdr memory

__device__ float* d_hdr_depth_map_list_[D_HDR_MAX_NUM];
__device__ unsigned char* d_hdr_brightness_list_[D_HDR_MAX_NUM]; 
__device__ float* d_hdr_bright_pixel_sum_list_[D_HDR_MAX_NUM];
   
/**********************************************************************/
//repetition memory
__device__ unsigned char* d_repetition_patterns_list_[6*D_REPETITIONB_MAX_NUM]; 
__device__ unsigned short* d_repetition_merge_patterns_list_[6];  

#define D_REPETITION_02_MAX_NUM 37
__device__ unsigned short* d_repetition_02_merge_patterns_list_[D_REPETITION_02_MAX_NUM];  
__device__ unsigned short* d_merge_brightness_map_;
/**********************************************************************/
void cuda_set_param_system_config(SystemConfigDataStruct param);
    
bool cuda_set_projector_version(int version);

bool cuda_set_camera_resolution(int width,int height);

//分配basic内存
bool cuda_malloc_basic_memory();

bool cuda_free_basic_memory();

//分配hdr内存
bool cuda_malloc_hdr_memory();

bool cuda_free_hdr_memory();

//分配repetition内存
bool cuda_malloc_repetition_memory();

bool cuda_free_repetition_memory();


/************************************************************************************/
//copy
void cuda_copy_calib_data(float* camera_intrinsic, float* project_intrinsic, float* camera_distortion,
	float* projector_distortion, float* rotation_matrix, float* translation_matrix); 

void cuda_copy_talbe_to_memory(float* mapping,float* mini_mapping,float* rotate_x,float* rotate_y,float* r_1,float base_line);

void coud_copy_undistort_table_to_memory(float* undistort_x_map,float* undistort_y_map);
 
bool cuda_copy_pattern_to_memory(unsigned char* pattern_ptr,int serial_flag);


void cuda_copy_pointcloud_from_memory(float* pointcloud);

void cuda_copy_depth_from_memory(float* depth);

void cuda_copy_brightness_from_memory(unsigned char* brightness);

void cuda_copy_convolution_kernal_to_memory(float* convolution_kernal, int kernal_diameter);

void cuda_copy_brightness_to_memory(unsigned char* brightness);

void cuda_clear_reconstruct_cache();


/***********************************************************************************/

bool cuda_compute_phase_shift(int serial_flag); 

bool cuda_compute_convolved_image_phase_shift(int serial_flag);

bool cuda_unwrap_phase_shift(int serial_flag);

bool cuda_unwrap_phase_shift_base_fisher_confidence(int serial_flag);

bool cuda_rectify_six_step_pattern_phase(int mode, int kernal_diameter);

bool cuda_normalize_phase(int serial_flag);

void fisher_filter(float fisher_confidence_val);

void phase_monotonicity_filter(float monotonicity_val);

void depth_filter(float depth_threshold_val);

  
/****************************************************************************************/

int cuda_copy_minsw8_pattern_to_memory(unsigned char* pattern_ptr,int serial_flag);


int cuda_handle_minsw8(int flag);

int cuda_handle_repetition_model06(int repetition_count);

int cuda_handle_repetition_model06_16(int repetition_count);


/*********************************************************************************/
//mono

void cuda_copy_brightness_16_to_memory(unsigned short* brightness);

int cuda_handle_model06_16();

int cuda_handle_minsw8_16(int flag);

int cuda_copy_minsw8_pattern_to_memory_16(unsigned short* pattern_ptr,int serial_flag);

// bool cuda_copy_result_to_hdr_16(int serial_flag,int brigntness_serial);

bool cuda_merge_hdr_data_16(int hdr_num,float* depth_map, unsigned char* brightness);
/***********************************************************************************/
//reconstruct
 
bool cuda_generate_pointcloud_base_table();
 
bool cuda_generate_pointcloud_base_minitable();


/***********************************************************************************/
//hdr 


bool cuda_copy_result_to_hdr_color(int serial_flag,int brigntness_serial,cv::Mat brightness); 

bool cuda_copy_result_to_hdr(int serial_flag,int brigntness_serial);
 
bool cuda_merge_hdr_data(int hdr_num,float* depth_map, unsigned char* brightness);


bool cuda_merge_brigntness(int hdr_num, unsigned char* brightness);
/***********************************************************************************/


bool cuda_copy_repetition_pattern_to_memory(unsigned char* patterns_ptr,int serial_flag);


bool cuda_merge_repetition_patterns(int repetition_serial);


bool cuda_compute_merge_phase(int repetition_count);


bool cuda_clear_repetition_02_patterns();

bool cuda_merge_repetition_02_patterns(int repetition_serial);

bool cuda_merge_repetition_02_patterns_16(unsigned short * const d_in_pattern,int repetition_serial);

bool cuda_compute_merge_repetition_02_phase(int repetition_count,int phase_num);
/***********************************************************************************/
//filter
void cuda_remove_points_base_radius_filter(float dot_spacing,float radius,int threshold_num);

void cuda_filter_reflect_noise(float * const unwrap_map);


//repetition
 
void cuda_copy_phase_from_cuda_memory(float* phase_x,float* phase_y);

/************************************************************************************************/


#endif