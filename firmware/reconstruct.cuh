#ifndef RECONSTRUCT_CUDA_CUH
#define RECONSTRUCT_CUDA_CUH
#pragma once 
#include <opencv2/core.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
 


bool cuda_set_param_dlp_resolution(int width,int height);

bool cuda_set_param_confidence(float val);

bool cuda_set_param_z_range(float min,float max);
 

__global__ void kernel_reconstruct_pointcloud_base_table(int width,int height,float * const xL_rotate_x,float * const xL_rotate_y,float * const single_pattern_mapping,float * const R_1,float b,
float * const confidence_map,float * const phase_x , float * const pointcloud,float * const depth);
 
__global__ void kernel_remove_mask_result(int width,int height, unsigned char * const mask,uchar threshold,float* const depth,float * const pointcloud);

__device__ float bilinear_interpolation(float x, float y, int map_width, float *mapping);

__global__ void kernel_reconstruct_pointcloud_base_minitable(uint32_t img_height, uint32_t img_width, float* const xL_rotate_x, float* const xL_rotate_y, float* const single_pattern_minimapping, float* const R_1, float b, 
 float* const confidence_map, float* const phase_x,float* const pointcloud, float* const depth);
 
__device__ float mini_bilinear_interpolation(float x, float y, int map_width, float *mapping);

__global__ void kernel_reconstruct_pointcloud_base_depth(int width,int height,float * const xL_rotate_x,float * const xL_rotate_y,
float* const  camera_intrinsic,float* const  camera_distortion, float * const depth, float * const pointcloud);





#endif