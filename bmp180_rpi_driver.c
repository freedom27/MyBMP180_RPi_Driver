#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>

#include "bmp180.h"
#include "bmp180_rpi_driver.h"

// These 3 functions are the delegate functions for the Bosch driver
s8 BMP180_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMP180_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BMP180_delay_msek(u32 msek);

// Function to initialize the connection with the I2C device
int I2C_Setup(int dev_addr);

// File descriptor of the I2C device, is preinitialized to one to know when is 
// yet to be initialized
int i2c_fd = -1;

// Implementation of the delegate functions for the Bosch driver
s8 BMP180_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    s32 iError = BMP180_INIT_VALUE;
	u8 stringpos = BMP180_INIT_VALUE;
	for (stringpos = BMP180_INIT_VALUE; stringpos < cnt; stringpos++) {
        iError = i2c_smbus_write_word_data(i2c_fd, reg_addr + stringpos, *(reg_data + stringpos));
	}

	return (s8)iError;
}

s8 BMP180_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) { 
    s32 iError = BMP180_INIT_VALUE;
	u8 stringpos = BMP180_INIT_VALUE;
    
	for (stringpos = BMP180_INIT_VALUE; stringpos < cnt; stringpos++) {
        *(reg_data + stringpos) = i2c_smbus_read_word_data(i2c_fd, reg_addr + stringpos);
	}
    
	return (s8)iError;
}

void BMP180_delay_msek(u32 msek) {
	usleep(msek * 1000);
}

// Inplementation of the function to initialize the connection with the I2C device
int I2C_Setup(int dev_addr) {
    if (i2c_fd <= 0) {
        //Temporarely hardcoded to 1 for RPi2
        int adapter_nr = 1;
        char filename[20];
    
        snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
    
        i2c_fd = open(filename, O_RDWR);
        if(i2c_fd < 0) {
            return -1;
        }
        if(ioctl(i2c_fd, I2C_SLAVE, dev_addr) < 0){
            return -1;
        }
    }
    
    return 0;
}

int bmp180_read(struct bmp180_sensor_data *sensor_data) {
    if (I2C_Setup(BMP180_I2C_ADDR) < 0) {
        return BMP180_I2C_INIT_ERROR;
    }
    
    struct bmp180_t bmp180;

    u16 v_uncomp_temp_u16 = BMP180_INIT_VALUE;
    u32 v_uncomp_press_u32 = BMP180_INIT_VALUE;

    bmp180.bus_write = BMP180_I2C_bus_write;
    bmp180.bus_read = BMP180_I2C_bus_read;
    bmp180.dev_addr = BMP180_I2C_ADDR;
    bmp180.delay_msec = BMP180_delay_msek;

    if (bmp180_init(&bmp180) < 0) {
        return BMP180_DRIVER_INIT_ERROR;
    }
    
    // Increasing sensor precision
    bmp180.sw_oversamp = BMP180_SW_OVERSAMP_U8X;
    bmp180.oversamp_setting = BMP180_OVERSAMP_SETTING_U8X;
    
    v_uncomp_temp_u16 = bmp180_get_uncomp_temperature();
    v_uncomp_press_u32 = bmp180_get_uncomp_pressure();
    
    sensor_data->temperature = bmp180_get_temperature(v_uncomp_temp_u16) / 10.0;
    sensor_data->pressure = bmp180_get_pressure(v_uncomp_press_u32) / 100.0;
    
    return BMP180_OK;
}