#include "minsw.cuh" 
#include <opencv2/core.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


__global__ void kernel_generate_merge_threshold_map(int width,int height,unsigned short * const d_in_white, unsigned short * const d_in_black,unsigned short * const d_out_threshold)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
    const unsigned int offset = idy * width + idx;

    if (idx < width && idy < height)
    { 
        unsigned short val = 0.5 + (d_in_white[offset] - d_in_black[offset])/2;
        d_out_threshold[offset] = d_in_black[offset] + val;
    }
}
 
__global__ void kernel_generate_threshold_map(int width,int height,unsigned char * const d_in_white, unsigned char * const d_in_black,unsigned char * d_out_threshold)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
    const unsigned int offset = idy * width + idx;

    if (idx < width && idy < height)
    { 
        unsigned char val = 0.5 + (d_in_white[offset] - d_in_black[offset])/2;
        d_out_threshold[offset] = d_in_black[offset] + val;
    }
}

__global__ void kernel_threshold_merge_patterns(int width,int height,unsigned short * const d_in_pattern, unsigned short * const d_in_threshold,int places,unsigned char* const d_out_bin)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
    const unsigned int offset = idy * width + idx;

    if (idx < width && idy < height)
    { 
        unsigned char val = d_out_bin[offset];

        unsigned char mv_i = (7- places);

        unsigned char mask = 1 << mv_i;
        val = val & (~mask);
        unsigned char set_bit = 0;

        if(d_in_pattern[offset] > d_in_threshold[offset] )
        { 
            set_bit = 1 << mv_i;
        }
 

        val = val | set_bit;
        d_out_bin[offset] = val;
    }
}

__global__ void kernel_threshold_merge_patterns_with_uncertain(int width,int height,unsigned short * const d_in_pattern, unsigned short * const d_in_threshold,int places,
unsigned char* const d_out_bin,unsigned char * const d_in_direct,unsigned char * const d_in_global,unsigned char * const d_out_uncertain)
{

   const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
    const unsigned int offset = idy * width + idx;

    if (idx < width && idy < height)
    { 
        unsigned char val = d_out_bin[offset];

        unsigned char mv_i = (7- places);

        unsigned char mask = 1 << mv_i;
        val = val & (~mask);
        unsigned char set_bit = 0;

        if (d_in_direct[offset] < 2)
        {
            d_out_uncertain[offset] += 4;
        }

        if (d_in_direct[offset] > d_in_global[offset])
        {
            if (d_in_pattern[offset] > d_in_direct[offset])
            { 
                set_bit = 1 << mv_i;
            }
            // else if (d_in_pattern[offset] < d_in_global[offset])
            // {
            //     // ptr_bin[c] = 0;
            // }
            else
            {
                if (d_in_pattern[offset]> d_in_threshold[offset])
                { 
                    set_bit = 1 << mv_i;
                }
                // else
                // { 
                // }

                d_out_uncertain[offset] += 2;
            }
        }
        else
        {
            // if (d_in_pattern[offset] < d_in_direct[offset])
            // { 
            // }
            if (d_in_pattern[offset] > d_in_global[offset])
            { 
                set_bit = 1 << mv_i;
            }
            else
            {
                d_out_uncertain[offset] += 20;
            }
        }

        // if(d_in_pattern[offset] > d_in_threshold[offset] )
        // { 
        //     set_bit = 1 << mv_i;
        // }
 

        val = val | set_bit;
        d_out_bin[offset] = val;
    }

    
}



__global__ void kernel_threshold_patterns_with_uncertain(int width,int height,unsigned char * const d_in_pattern, unsigned char * const d_in_threshold,int places,
unsigned char* const d_out_bin,unsigned char * const d_in_direct,unsigned char * const d_in_global,unsigned char * const d_out_uncertain)
{
     const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
    const unsigned int offset = idy * width + idx;

    if (idx < width && idy < height)
    { 
        unsigned char val = d_out_bin[offset];

        unsigned char mv_i = (7- places);

        unsigned char mask = 1 << mv_i;
        val = val & (~mask);
        unsigned char set_bit = 0;

        if (d_in_direct[offset] < 2)
        {
            d_out_uncertain[offset] += 4;
        }

        if (d_in_direct[offset] > d_in_global[offset])
        {
            if (d_in_pattern[offset] > d_in_direct[offset])
            { 
                set_bit = 1 << mv_i;
            }
            // else if (d_in_pattern[offset] < d_in_global[offset])
            // {
            //     // ptr_bin[c] = 0;
            // }
            else
            {
                if (d_in_pattern[offset]> d_in_threshold[offset])
                { 
                    set_bit = 1 << mv_i;
                }
                // else
                // { 
                // }

                d_out_uncertain[offset] += 2;
            }
        }
        else
        {
            // if (d_in_pattern[offset] < d_in_direct[offset])
            // { 
            // }
            if (d_in_pattern[offset] > d_in_global[offset])
            { 
                set_bit = 1 << mv_i;
            }
            else
            {
                d_out_uncertain[offset] += 20;
            }
        }

        // if(d_in_pattern[offset] > d_in_threshold[offset] )
        // { 
        //     set_bit = 1 << mv_i;
        // }
 

        val = val | set_bit;
        d_out_bin[offset] = val;
    }


}
 
__global__ void kernel_threshold_patterns(int width,int height,unsigned char * const d_in_pattern, unsigned char * const d_in_threshold,int places ,unsigned char * const d_out_bin)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
    const unsigned int offset = idy * width + idx;

    if (idx < width && idy < height)
    { 
        unsigned char val = d_out_bin[offset];

        unsigned char mv_i = (7- places);

        unsigned char mask = 1 << mv_i;
        val = val & (~mask);
        unsigned char set_bit = 0;

        if(d_in_pattern[offset] > d_in_threshold[offset] )
        { 
            set_bit = 1 << mv_i;
        }
 

        val = val | set_bit;
        d_out_bin[offset] = val;
    }
}
 

__global__ void kernel_minsw8_to_bin(int width,int height,unsigned char * const minsw8_code,unsigned char * const d_in_minsw8, unsigned char * const d_out_bin)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
    const unsigned int offset = idy * width + idx;

    if (idx < width && idy < height)
    { 
        // unsigned char array[256] = { 0, 1, 105, 2, 155, 154, 156, 3, 19, 172, 106, 107, 70, 69, 157, 4, 169,
		// 66, 192, 193, 220, 67, 91, 90, 170, 171, 211, 108, 221, 68, 6, 5, 255, 152, 126, 23, 50, 
		// 153, 177, 176, 20, 173, 21, 22, 71, 174, 72, 175, 190, 87, 191, 88, 241, 240, 242, 89, 85,
		// 86, 212, 109, 136, 239, 7, 110, 233, 130, 128, 129, 28, 131, 27, 26, 234, 235, 147, 44, 29,
		// 132, 198, 197, 64, 65, 41, 194, 219, 218, 92, 195, 83, 236, 42, 43, 134, 133, 93, 196, 254,
		// 151, 127, 24, 49, 48, 178, 25, 149, 150, 148, 45, 200, 47, 199, 46, 63, 216, 62, 215, 114, 
		// 217, 113, 112, 84, 237, 213, 214, 135, 238, 8, 111, 103, 206, 104, 207, 52, 205, 53, 54, 
		// 18, 121, 209, 208, 223, 120, 158, 55, 168, 15, 39, 142, 117, 118, 244, 141, 17, 16, 210, 57,
		// 222, 119, 159, 56, 102, 101, 125, 228, 51, 204, 74, 75, 123, 122, 124, 227, 224, 225, 73, 
		// 226, 189, 36, 38, 37, 138, 139, 243, 140, 188, 35, 59, 58, 137, 34, 160, 161, 232, 79, 231,
		// 78, 181, 182, 180, 77, 81, 80, 146, 249, 30, 183, 95, 248, 167, 14, 40, 143, 116, 13, 245,
		// 246, 82, 185, 145, 144, 31, 184, 94, 247, 253, 100, 230, 229, 202, 203, 179, 76, 252, 99, 
		// 251, 250, 201, 98, 96, 97, 166, 165, 61, 164, 115, 12, 10, 11, 187, 186, 60, 163, 32, 33, 9, 162 };

        d_out_bin[offset] = minsw8_code[d_in_minsw8[offset]];
    }
}


__global__ void kernel_bin_unwrap(int width,int height,unsigned char * const d_in_bin, float * const d_in_wrap,float * const d_out_unwrap)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
    const unsigned int offset = idy * width + idx;

    if (idx < width && idy < height)
    { 
            int k2 = (d_in_bin[offset] + 1) / 2;
            int k1 = d_in_bin[offset]/2;

			if (d_in_wrap[offset] < CV_PI / 2.0)
			{
				d_out_unwrap[offset] =d_in_wrap[offset] + 2 * CV_PI * k2;
			}
			else if (d_in_wrap[offset] < 3 * CV_PI / 2.0)
			{

				d_out_unwrap[offset] = d_in_wrap[offset] + 2 * CV_PI * k1;
			}
			else
			{
				d_out_unwrap[offset] = d_in_wrap[offset] + 2 * CV_PI * (k2 - 1);
			}
    }
}
