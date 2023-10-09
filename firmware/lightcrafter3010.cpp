#include "lightcrafter3010.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "easylogging++.h"
#include "math.h"
#include "protocol.h"

LightCrafter3010::LightCrafter3010()
{
	int fd;
	if((fd = i2c_open("/dev/i2c-1")) == -1)
	{
		perror("Open i2c bus error");
		return;
	}

	memset(&_device, 0, sizeof(_device));
	i2c_init_device(&_device);

	_device.bus = fd;
    _device.addr = 0x1b & 0x3ff;        //0x1b is LC3010 device address
    _device.page_bytes = 256;//32;
    _device.iaddr_bytes = 1;

    camera_exposure_ = 12000;

    // ----- MCP3221 init
	memset(&_MCP3221, 0, sizeof(_MCP3221));
	i2c_init_device(&_MCP3221);
    _MCP3221.bus = fd;
    _MCP3221.addr = 0x4f;               //MCP3221A7T-E/OT 
    _MCP3221.page_bytes = 256;
    _MCP3221.iaddr_bytes = 1;  

    dlp_min_exposure_ = 1700;
    camera_min_exposure_ = 6000;
}
 
size_t LightCrafter3010::read_with_param(char inner_addr,unsigned char param, void* buffer, size_t buffer_size)
{
    return	i2c_read_with_param(&_device, inner_addr,param, buffer, buffer_size);
}

size_t LightCrafter3010::read(char inner_addr, void* buffer, size_t buffer_size)
{
	return	i2c_read(&_device, inner_addr, buffer, buffer_size);
}

size_t LightCrafter3010::write(char inner_addr, void* buffer, size_t buffer_size)
{
	return	i2c_write(&_device, inner_addr, buffer, buffer_size);
}

LightCrafter3010::~LightCrafter3010()
{
	i2c_close(_device.bus);
}


int LightCrafter3010::init()
{
	char buffer[6];
    memset(buffer,0,6*sizeof(char));


    int ret = 0;
    char ver_buffer[6];
    memset(buffer,0,6*sizeof(char));


	//internal test pattern mode
	buffer[0] = 0x04;
	write(Write_Operating_Mode_Select, buffer, 1);

    read(Read_Operating_Mode_Select,ver_buffer,1);
    ret = memcmp(buffer,ver_buffer,1);
    if(0 != ret)
    { 
        return DF_ERROR_LIGHTCRAFTER_SET_MODEL;
    }

	//LED set to brightest
	// SetLedCurrent(1023, 1023, 1023);

	//enable tigger out 1
	buffer[4] = 0x00;
	buffer[3] = 0x00;
	buffer[2] = 0x00;
	buffer[1] = 0x00;
	buffer[0] = 0x02;
	write(Write_Trigger_Out, buffer, 5);

	read(Read_Trigger_Out, ver_buffer, 5);
    ret = memcmp(buffer,ver_buffer,5);
    if(!ret)
    {
        return DF_ERROR_LIGHTCRAFTER_SET_TRIGGEROUT;
    }

	//enable trigger out 2
	//delay = -500us (0xfffffe0c)
	buffer[4] = 0xff;
	buffer[3] = 0xff;
	buffer[2] = 0xfe;
	buffer[1] = 0x0c;
	buffer[0] = 0x03;
	write(Write_Trigger_Out, buffer, 5);

	read(Read_Trigger_Out, ver_buffer, 5);
    ret = memcmp(buffer,ver_buffer,5);
    if(!ret)
    {
        return DF_ERROR_LIGHTCRAFTER_SET_TRIGGEROUT;
    }

    // set_trigger_out_delay(-500);

    return DF_SUCCESS;

}


int LightCrafter3010::read_patterns_sets_num(int &num)
{

    num = 0;
    for(int i= 0;i< 64;i++)
    {
        unsigned char ver_buffer[25];
        memset(ver_buffer,0,25*sizeof(char));

        ver_buffer[0] = 2;
        write(Write_Pattern_Order, ver_buffer, 25);
 
        memset(ver_buffer,0,25*sizeof(char));
        read_with_param(Read_Pattern_Order,i, ver_buffer, 25);
 

        if(255 == ver_buffer[0])
        {
            break;
        }

        num = i+1; 

    }

    return 0;
}
  
void LightCrafter3010::set_trigger_out_delay(int delay_time)
{
     
    int delay = abs(delay_time); 

    std::vector<unsigned char> remainder_list;
    std::vector<unsigned char> inv_remainder_list;

    for (int i = 0; i < 4; i++)
    {
        int remainder = delay % 256;
        // LOG(INFO) << remainder;

        unsigned char inv_c = ~(remainder);
        inv_remainder_list.push_back(inv_c); 
        remainder_list.push_back(remainder);
        delay /= 256;
    }
 
    inv_remainder_list[0]++;
    if (0 == inv_remainder_list[0])
    {
        inv_remainder_list[1]++;
        if (0 == inv_remainder_list[1])
        {
            inv_remainder_list[2]++;
            if (0 == inv_remainder_list[2])
            {
                inv_remainder_list[3]++;
                if (0 == inv_remainder_list[3])
                {
                    return;
                }
            }
        }
    }

    
	char buffer[6];

    if (delay_time > 0)
    {
        // enable trigger out 2
        // delay = -500us (0xfffffe0c)
        buffer[4] = remainder_list[3];
        buffer[3] = remainder_list[2];
        buffer[2] = remainder_list[1];
        buffer[1] = remainder_list[0];
        buffer[0] = 0x03;
        write(0x92, buffer, 5);
    }
    else
    {
        // enable trigger out 2
        // delay = -500us (0xfffffe0c)
        buffer[4] = inv_remainder_list[3];
        buffer[3] = inv_remainder_list[2];
        buffer[2] = inv_remainder_list[1];
        buffer[1] = inv_remainder_list[0];
        buffer[0] = 0x03;
        write(0x92, buffer, 5);
    }
 
       std::cout << std::hex << (int)buffer[4] << std::endl;
       std::cout << std::hex << (int)buffer[3] << std::endl;
       std::cout << std::hex << (int)buffer[2] << std::endl;
       std::cout << std::hex << (int)buffer[1] << std::endl; 
       std::cout << std::hex << (int)buffer[0] << std::endl; 

  
 
}

int LightCrafter3010::SetLedCurrent(unsigned short R, unsigned short G, unsigned short B)
{
    if (R>1023) R=1023;
    if (G>1023) G=1023;
    if (B>1023) B=1023;
    
    char buffer[6];
    memset(buffer,0,6*sizeof(char));

    int ret = 0;
    char ver_buffer[6];
    memset(ver_buffer,0,6*sizeof(char));



    buffer[5] = ((B>>8)&0x03);
    buffer[4] = (B&0xff);
    buffer[3] = ((G>>8)&0x03);
    buffer[2] = (G&0xff);
    buffer[1] = ((R>>8)&0x03);
    buffer[0] = (R & 0xff);

    write(0x5C, buffer, 6);
    write(Write_Led_Current, buffer, 6);

    // for(int i= 0;i< 6;i+=2)
    // {
    //     int val = buffer[i]+buffer[i+1]*256;
    //     LOG(INFO)<<" Write_Led_Current: "<<i<<" "<<val;
    // }
     
    read(Read_Led_Current, ver_buffer, 6);

    // for(int i= 0;i< 6;i+=2)
    // {
    //     int val = ver_buffer[i]+ver_buffer[i+1]*256;
    //     LOG(INFO)<<" Read_Led_Current: "<<i<<" "<<val;
    // }

    ret = memcmp(buffer,ver_buffer,6*sizeof(char));

    if (0 != ret)
    {
        return DF_ERROR_LIGHTCRAFTER_SET_CURRENT; 
    }

    return DF_SUCCESS;
} 


void LightCrafter3010::read_dmd_device_id(int& version)
{
    
    unsigned char TxBuffer[8];

    read(Read_Device_ID,TxBuffer,1);

    int value = int(TxBuffer[0]);

    // LOG(INFO)<<"DMD: "<<value;

    if(11 == value)
    {
        version = 3010;
    }
    else if(12 == value)
    {
        version = 4710;
    }
}

void LightCrafter3010::set_camera_min_exposure(float min)
{
    camera_min_exposure_ = min;
}

void LightCrafter3010::enable_solid_field()
{
    unsigned char TxBuffer[8];
 
    TxBuffer[0] = 0x01;
    //1Ah
    write(Write_Image_Freeze, TxBuffer, 1);

    TxBuffer[0] = 0x00;
    //52h
    write(Write_Rgb_Led_Enable, TxBuffer, 1);

    TxBuffer[0] = 0x01;
    //05h
    write(Write_Operating_Mode_Select, TxBuffer, 1);
 
    TxBuffer[0] = 0x00;
    TxBuffer[1] = 0x70;
    TxBuffer[2] = 0x0F;
    TxBuffer[3] = 0x00;
    TxBuffer[4] = 0x0F;
    TxBuffer[5] = 0x00;
    //0Bh
    write(Write_Checkerboard, TxBuffer, 6);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
 
    TxBuffer[0] = 0x07;
    //52h
    write(Write_Rgb_Led_Enable, TxBuffer, 1);

    TxBuffer[0] = 0x00;
    //1Ah
    write(Write_Image_Freeze, TxBuffer, 1);

}

void LightCrafter3010::disable_solid_field()
{
    disable_checkerboard();
    init();
}

void LightCrafter3010::enable_checkerboard()
{
    unsigned char TxBuffer[8];
    
    TxBuffer[0] = 0x01;
    write(Write_Image_Freeze, TxBuffer, 1);

    TxBuffer[0] = 0x01;
    write(Write_Operating_Mode_Select, TxBuffer, 1);
/*
    TxBuffer[0] = 0x00; 
    TxBuffer[1] = 0x05; 
    TxBuffer[2] = 0xD0; 
    TxBuffer[3] = 0x02;
    write(Write_Input_Image_Size, TxBuffer, 4);

    memset(TxBuffer, 0x00, 8);
    TxBuffer[5] = 0x05;
    TxBuffer[6] = 0xD0;
    TxBuffer[7] = 0x02;
    write(Write_Image_Crop, TxBuffer, 8);
*/
    TxBuffer[0] = 0x87;
    TxBuffer[1] = 0x70;
    TxBuffer[2] = 0x0F;
    TxBuffer[3] = 0x00;
    TxBuffer[4] = 0x0F;
    TxBuffer[5] = 0x00;
    write(Write_Checkerboard, TxBuffer, 6);
/*
    memset(TxBuffer, 0x00, 8);
    TxBuffer[5] = 0x05;
    TxBuffer[6] = 0xD0;
    TxBuffer[7] = 0x02;
    write(Write_Display_Size, TxBuffer, 8);
*/
    TxBuffer[0] = 0x07;
    write(Write_Rgb_Led_Enable, TxBuffer, 1);

    TxBuffer[0] = 0x00;
    write(Write_Image_Freeze, TxBuffer, 1);
} 

void LightCrafter3010::disable_checkerboard()
{
    unsigned char TxBuffer[8];
    
    TxBuffer[0] = 0x01;
    write(Write_Image_Freeze, TxBuffer, 1);

    TxBuffer[0] = 0x00;
    TxBuffer[1] = 0x00;
    TxBuffer[2] = 0x0F;
    TxBuffer[3] = 0x00;
    TxBuffer[4] = 0x0F;
    TxBuffer[5] = 0x00;
    write(Write_Checkerboard, TxBuffer, 6);

    TxBuffer[0] = 0x00;
    write(Write_Image_Freeze, TxBuffer, 1);
} 

void LightCrafter3010::set_internal_pattern_stop()
{
    unsigned char TxBuffer[8];
    
    TxBuffer[0] = 0x01;
    TxBuffer[1] = 0x00;
    write(Write_Internal_Pattern_Control, TxBuffer, 2);
}

void LightCrafter3010::set_flash_data_type()
{
    unsigned char TxBuffer[8];
    
    TxBuffer[0] = 0xD0;
    write(Write_Flash_Data_Type_Select, TxBuffer, 1);
}

bool LightCrafter3010::set_flash_build_data_size(unsigned int data_size)
{
    bool ack = true;
    unsigned char MxBuffer[8];
    
    MxBuffer[0] = (data_size >> 0) & 0xff;
    MxBuffer[1] = (data_size >> 8) & 0xff;
    MxBuffer[2] = (data_size >> 16) & 0xff;
    MxBuffer[3] = (data_size >> 24) & 0xff;
    write(Read_Flash_Update_Precheck, MxBuffer, 4);

    return ack;
}

void LightCrafter3010::set_erase_flash()
{
    unsigned char TxBuffer[8];
    
    TxBuffer[0] = 0xAA;
    TxBuffer[1] = 0xBB;
    TxBuffer[2] = 0xCC;
    TxBuffer[3] = 0xDD;
    write(Write_Flash_Erase, TxBuffer, 4);
}

bool LightCrafter3010::check_erase_flash_status()
{
    bool ack = true;
    unsigned char RxBuffer[8];
    int i;
    
    for (i = 0; i < 30; i++)
    {
        read(Read_Short_Status, RxBuffer, 1);

        char string[50] = {'\0'};
        sprintf(string, "Read_Short_Status: %d- 0x%X", i, RxBuffer[0]);
        LOG(INFO)<<string;

        if ((RxBuffer[0] & 0x7E) != 0) {
            ack = false;
        } else {
            ack = true;
            break;
        }

        usleep(10000);
    }

    return ack;
}

void LightCrafter3010::set_flash_data_length(unsigned short dataLen)
{
    unsigned char TxBuffer[8];
    
    TxBuffer[0] = (dataLen >> 0) & 0xff;
    TxBuffer[1] = (dataLen >> 8) & 0xff;
    write(Write_Flash_Data_Length, TxBuffer, 2);
}

int LightCrafter3010::write_data_into_the_flash(unsigned char writeFlashCmd, char *TxBuffer, unsigned short dataLen)
{
    return write(writeFlashCmd, TxBuffer, dataLen);
}

void LightCrafter3010::read_data_from_the_flash(unsigned char readFlashCmd, char *RxBuffer, unsigned short dataLen)
{
    read(readFlashCmd, RxBuffer, dataLen);
}

void LightCrafter3010::reload_pattern_order_table_from_flash()
{
    unsigned char TxBuffer[24];
    
    memset(TxBuffer, 0, sizeof(TxBuffer));
    TxBuffer[0] = 0x02;
    write(Write_Pattern_Order_Table_Entry, TxBuffer, 24);
}

int LightCrafter3010::write_pattern_table(unsigned char* pattern_index, unsigned char* pattern_nums,unsigned char* pattern_invert, int len,float camera_exposure)
{
     unsigned char buffer[24];
    memset(buffer,0,24*sizeof(char));

    int ret = 0;
    unsigned char ver_buffer[25];
    memset(ver_buffer,0,25*sizeof(char));

    buffer[0] = 0x01; // Start
    buffer[1] = 0x00; // Pattern Set Index

    buffer[2] = 0x06; // Number of pattern to display
    // buffer[3] = 0x07;  //RGB
    buffer[3] = 0x04; // BLUE

    // Pattern Invert
    buffer[4] = 0;
    buffer[5] = 0;
    buffer[6] = 0;
    buffer[7] = 0;
    buffer[8] = 0;
    buffer[9] = 0;
    buffer[10] = 0;
    buffer[11] = 0;

    int pre_illumination_dark_time = 8500;
    int pose_illumination_dark_time = 1500;

    int exposure_rate = camera_exposure_ / 2000;

    pre_illumination_dark_time = 1000 + 170 * (exposure_rate); //根据经验的值
    // pose_illumination_dark_time = 500+ 30*(exposure_rate);
    // pose_illumination_dark_time = 1000+ 30*(exposure_rate);

    // Illumination Time = 11000us
    int illumination_time = camera_exposure - 1000;

    if (camera_exposure < camera_min_exposure_)
    {
        pre_illumination_dark_time = camera_min_exposure_ - camera_exposure + 1000;
        illumination_time = camera_exposure;

        if(pre_illumination_dark_time > 12000)
        {
            pre_illumination_dark_time= 12000;  
            //此行代码涉及到两个issue：
            //1. https://gitee.com/open3dv/xema/issues/I66IGV
            //2. https://gitee.com/open3dv/xema/issues/I7JVG0
        }
    }

    LOG(INFO) << "pre_illumination_dark_time: " << pre_illumination_dark_time;
    LOG(INFO) << "illumination_time: " << illumination_time;
    LOG(INFO) << "pose_illumination_dark_time: " << pose_illumination_dark_time;

    std::vector<unsigned char> remainder_list;
    for (int i = 0; i < 4; i++)
    {
        int remainder = illumination_time % 256;
        remainder_list.push_back(remainder);
        illumination_time /= 256;
    }

    buffer[12] = remainder_list[0];
    buffer[13] = remainder_list[1];
    buffer[14] = remainder_list[2];
    buffer[15] = remainder_list[3];

    std::vector<unsigned char> dark_remainder_list;
    for (int i = 0; i < 4; i++)
    {
        int remainder = pre_illumination_dark_time % 256;
        dark_remainder_list.push_back(remainder);
        pre_illumination_dark_time /= 256;
    }

    // Pre-illumination Dark Time = 500us

    buffer[16] = dark_remainder_list[0];
    buffer[17] = dark_remainder_list[1];
    buffer[18] = dark_remainder_list[2];
    buffer[19] = dark_remainder_list[3];

    std::vector<unsigned char> pose_dark_remainder_list;
    for (int i = 0; i < 4; i++)
    {
        int remainder = pose_illumination_dark_time % 256;
        pose_dark_remainder_list.push_back(remainder);
        pose_illumination_dark_time /= 256;
    }
    buffer[20] = pose_dark_remainder_list[0];
    buffer[21] = pose_dark_remainder_list[1];
    buffer[22] = pose_dark_remainder_list[2];
    buffer[23] = pose_dark_remainder_list[3];



    for (int i = 0; i < len; i++)
    {
        buffer[1] = pattern_index[i];
	    buffer[2] = pattern_nums[i];
       

        // Pattern Invert
        buffer[4] =  pattern_invert[i];
        buffer[5] =  pattern_invert[i];
        buffer[6] =  pattern_invert[i];
        buffer[7] =  pattern_invert[i];
        buffer[8] =  pattern_invert[i];
        buffer[9] =  pattern_invert[i];
        buffer[10] =  pattern_invert[i];
        buffer[11] =  pattern_invert[i];

        write(Write_Pattern_Order, buffer, 24);

        // usleep(100);
        ver_buffer[0] = pattern_index[i];
        read(Read_Pattern_Order, ver_buffer, 25);

        ret = memcmp(buffer, ver_buffer+1, 24 * sizeof(char)); 
        if (0 != ret)
        {
            for(int i = 0;i< 24;i++)
            {
                LOG(INFO)<<(int)buffer[i] <<" , "<<(int)ver_buffer[i];
            }
            return DF_ERROR_LIGHTCRAFTER_SET_PATTERN_ORDER;
        }

        buffer[0] = 0x00;
    }





    return DF_SUCCESS;
}


int LightCrafter3010::write_pattern_table(unsigned char* pattern_index, unsigned char* pattern_nums, int len,float camera_exposure)
{
    unsigned char buffer[24];
    memset(buffer,0,24*sizeof(char));

    int ret = 0;
    unsigned char ver_buffer[25];
    memset(ver_buffer,0,25*sizeof(char));

    buffer[0] = 0x01; // Start
    buffer[1] = 0x00; // Pattern Set Index

    buffer[2] = 0x06; // Number of pattern to display
    // buffer[3] = 0x07;  //RGB
    buffer[3] = 0x04; // BLUE

    // Pattern Invert
    buffer[4] = 0;
    buffer[5] = 0;
    buffer[6] = 0;
    buffer[7] = 0;
    buffer[8] = 0;
    buffer[9] = 0;
    buffer[10] = 0;
    buffer[11] = 0;

    int pre_illumination_dark_time = 8500;
    int pose_illumination_dark_time = 1500;

    int exposure_rate = camera_exposure_ / 2000;

    pre_illumination_dark_time = 1000 + 170 * (exposure_rate); //根据经验的值
    // pose_illumination_dark_time = 500+ 30*(exposure_rate);
    // pose_illumination_dark_time = 1000+ 30*(exposure_rate);

    // Illumination Time = 11000us
    int illumination_time = camera_exposure - 1000;

    if (camera_exposure < camera_min_exposure_)
    {
        pre_illumination_dark_time = camera_min_exposure_ - camera_exposure + 1000;
        illumination_time = camera_exposure;

        if(pre_illumination_dark_time > 12000)
        {
            pre_illumination_dark_time= 12000;  
            //此行代码涉及到两个issue：
            //1. https://gitee.com/open3dv/xema/issues/I66IGV
            //2. https://gitee.com/open3dv/xema/issues/I7JVG0
        }
    }

    LOG(INFO) << "pre_illumination_dark_time: " << pre_illumination_dark_time;
    LOG(INFO) << "illumination_time: " << illumination_time;
    LOG(INFO) << "pose_illumination_dark_time: " << pose_illumination_dark_time;

    std::vector<unsigned char> remainder_list;
    for (int i = 0; i < 4; i++)
    {
        int remainder = illumination_time % 256;
        remainder_list.push_back(remainder);
        illumination_time /= 256;
    }

    buffer[12] = remainder_list[0];
    buffer[13] = remainder_list[1];
    buffer[14] = remainder_list[2];
    buffer[15] = remainder_list[3];

    std::vector<unsigned char> dark_remainder_list;
    for (int i = 0; i < 4; i++)
    {
        int remainder = pre_illumination_dark_time % 256;
        dark_remainder_list.push_back(remainder);
        pre_illumination_dark_time /= 256;
    }

    // Pre-illumination Dark Time = 500us

    buffer[16] = dark_remainder_list[0];
    buffer[17] = dark_remainder_list[1];
    buffer[18] = dark_remainder_list[2];
    buffer[19] = dark_remainder_list[3];

    std::vector<unsigned char> pose_dark_remainder_list;
    for (int i = 0; i < 4; i++)
    {
        int remainder = pose_illumination_dark_time % 256;
        pose_dark_remainder_list.push_back(remainder);
        pose_illumination_dark_time /= 256;
    }
    buffer[20] = pose_dark_remainder_list[0];
    buffer[21] = pose_dark_remainder_list[1];
    buffer[22] = pose_dark_remainder_list[2];
    buffer[23] = pose_dark_remainder_list[3];



    for (int i = 0; i < len; i++)
    {
        buffer[1] = pattern_index[i];
	    buffer[2] = pattern_nums[i];
        write(Write_Pattern_Order, buffer, 24);

        // usleep(100);
        ver_buffer[0] = pattern_index[i];
        read(Read_Pattern_Order, ver_buffer, 25);

        ret = memcmp(buffer, ver_buffer+1, 24 * sizeof(char)); 
        if (0 != ret)
        {
            for(int i = 0;i< 24;i++)
            {
                LOG(INFO)<<(int)buffer[i] <<" , "<<(int)ver_buffer[i];
            }
            return DF_ERROR_LIGHTCRAFTER_SET_PATTERN_ORDER;
        }

        buffer[0] = 0x00;
    }





    return DF_SUCCESS;
}

void LightCrafter3010::write_pattern_table(unsigned char* pattern_index, int len)
{
    unsigned char buffer[24];
    buffer[0] = 0x01;  //Start
    buffer[1] = 0x00;  //Pattern Set Index

    buffer[2] = 0x06;  //Number of pattern to display
    //buffer[3] = 0x07;  //RGB
    buffer[3] = 0x04;  //RGB

    // Pattern Invert
    buffer[4] = 0; 
    buffer[5] = 0;
    buffer[6] = 0;
    buffer[7] = 0;
    buffer[8] = 0;
    buffer[9] = 0;
    buffer[10] = 0;
    buffer[11] = 0;

    // Illumination Time = 11000us
    buffer[12] = 0xf8;
    buffer[13] = 0x2a;
    buffer[14] = 0x00;
    buffer[15] = 0x00;

    // Pre-illumination Dark Time = 500us
    buffer[16] = 0xf4;
    buffer[17] = 0x01;
    buffer[18] = 0x00;
    buffer[19] = 0x00;

    // Post-illumination Dark Time = 1000us
    buffer[20] = 0xb8;
    buffer[21] = 0x0b;
    buffer[22] = 0x00;
    buffer[23] = 0x00;

    for(int i=0; i<len; i++)
    {
	    buffer[1] = pattern_index[i];
        write(0x98, buffer, 24);
	    buffer[0] = 0x00;
    }
}

void LightCrafter3010::pattern_mode01()
{
    // unsigned char pattern_index[] = {12,13,15,16};
    unsigned char pattern_index[] = {0,1,3,4};
    unsigned char pattern_nums[] = {6,6,6,6};
    write_pattern_table(pattern_index, pattern_nums, 4, camera_exposure_);
}

void LightCrafter3010::pattern_mode02()
{
    unsigned char pattern_index[] = {0,1,2,3,4,5,6};
    unsigned char pattern_nums[] = {6,6,6,6,6,6,6};
    write_pattern_table(pattern_index, pattern_nums, 7, camera_exposure_);
}

void LightCrafter3010::pattern_mode_brightness()
{
    unsigned char pattern_index[] = {6};
    unsigned char pattern_nums[] = {6};
    write_pattern_table(pattern_index, pattern_nums, 1, camera_exposure_);
}

void LightCrafter3010::pattern_mode03()
{
    unsigned char pattern_index[] = {0,1,2,3,4,6};
    unsigned char pattern_nums[] = {6,6,6,6,6,6};
    write_pattern_table(pattern_index, pattern_nums, 6, camera_exposure_);
}


int LightCrafter3010::pattern_mode04_repetition(int repetition_count)
{
    if(repetition_count< 1)
    {
        repetition_count = 1;
    }

    int group_count = 3+repetition_count;

    unsigned char pattern_index[group_count];
    unsigned char pattern_nums[group_count];
    for(int i=0; i<group_count; i++)
    {
	pattern_nums[i] = 6;
    }
    pattern_index[0] = 0;
    pattern_index[1] = 1;

    for(int i= 0;i< repetition_count;i++)
    {
        pattern_index[2+i] = 2;
    }
  
    pattern_index[group_count-1] = 6;
    return write_pattern_table(pattern_index, pattern_nums, group_count, camera_exposure_);
}

void LightCrafter3010::pattern_mode03_repetition(int repetition_count)
{
    if(repetition_count< 1)
    {
        repetition_count = 1;
    }

    int group_count = 5+repetition_count;

    unsigned char pattern_index[group_count];
    unsigned char pattern_nums[group_count];
    for(int i=0; i<group_count; i++)
    {
	pattern_nums[i] = 6;
    }

    pattern_index[0] = 0;
    pattern_index[1] = 1;

    for(int i= 0;i< repetition_count;i++)
    {
        pattern_index[2+i] = 2;
    }
 
    pattern_index[2+repetition_count] = 3;
    pattern_index[3+repetition_count] = 4;
    pattern_index[4+repetition_count] = 6;
    write_pattern_table(pattern_index, pattern_nums, group_count, camera_exposure_);

}	

int LightCrafter3010::pattern_mode04()
{
    unsigned char pattern_index[] = {0,1,2,8};
    unsigned char pattern_nums[] = {6,6,6,1};
    return write_pattern_table(pattern_index, pattern_nums, 4, camera_exposure_);
}

int LightCrafter3010::pattern_mode05()
{
    unsigned char pattern_index[] = {2,7,9};
    unsigned char pattern_nums[] = {6,8,2};
    return write_pattern_table(pattern_index, pattern_nums, 3, camera_exposure_);
}

int LightCrafter3010::pattern_mode06()
{
    unsigned char pattern_index[] = {8,2,7};
    unsigned char pattern_nums[] = {2,6,8};
    return write_pattern_table(pattern_index, pattern_nums, 3, camera_exposure_);
}

int LightCrafter3010::pattern_mode08() 
{
    unsigned char pattern_index[] = {8,2,7,7};
    unsigned char pattern_nums[] = {2,6,8,8};
    unsigned char pattern_invert[] = {0,0,0,0xFF};
    return write_pattern_table(pattern_index, pattern_nums, pattern_invert,4, camera_exposure_);
}

void LightCrafter3010::read_pattern_status()
{
    char buffer[7];

    read(0x9f, buffer, 7);
    printf("read pattern status\n");
    for(int i=0; i<7; i++)
    {
	printf("%d   %x\n", i, buffer[i]);
    }
}

void LightCrafter3010::read_pattern_table(int i)
{
    //0x99
    char buffer[24];

    buffer[0] = i;
    write(0x99, buffer, 1);
    read(0x99, buffer, 24);
    for(int i=0; i<24; i++)
    {
	printf("%x ", buffer[i]);
    }
    printf("\n");
}

		
void LightCrafter3010::start_pattern_sequence()
{
	char buffer[2] = {0x00, 0x00};
	write(0x9e, buffer, 2);
}


void LightCrafter3010::stop_pattern_sequence()
{
	char buffer[2] = {0x01, 0x00};
	write(0x9e, buffer, 2);
     
}

float LightCrafter3010::get_temperature()
{
    char buffer[2];
    read(0xd6, buffer, 2);
    bool negative = buffer[1] & 0x80;
    float temperature = ((*((unsigned short*)buffer)) & 0x7FFF)/10.0;
    if(negative)
    {
	temperature = -temperature;
    }
    return temperature;
}

size_t LightCrafter3010::read_mcp3221(void* buffer, size_t buffer_size)
{
	return	i2c_read(&_MCP3221, 0, buffer, buffer_size);
}

float LightCrafter3010::lookup_table(float fRntc)
{
    float temperature = 30.0;
    float last = 0, current = 0;
    int i = 0, number = 0;

    last = abs(fRntc - R_table[0]);

    for (i = 0; i < R_TABLE_NUM; i++)
    {
        current = abs(fRntc - R_table[i]);

        if (current < last) 
        {
            last = current;
            number = i;
        }
    }

    temperature = number - 40.0;

    printf("temperature = %f, R_table[%d] = %f\n", temperature, number, R_table[number]);

    return temperature;
}

float LightCrafter3010::get_projector_temperature()
{
    unsigned char buffer[2];
    int size = read_mcp3221(buffer, 2);

    short OutputCode;
    float temperature;
    if (size == 2) 
    {
        OutputCode = ((buffer[0] << 8) & 0xff00) | buffer[1];
        printf("The AD data = 0x%x = %d\n", OutputCode, OutputCode);

        // Rntc = 10 * (4096 - AD) / AD, unit=KO
        float fAD = OutputCode;
        float fRntc = 10.0 * (4096.0 - fAD) / fAD;
        printf("R = %f\n", fRntc);

        temperature = lookup_table(fRntc);
        temperature = (temperature >= 125.0) ? -125.0 : temperature;
    } 
    else
    {
        temperature = -100;
    }

    return temperature;
}
