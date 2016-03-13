#include<stdio.h>
#include <unistd.h>
#include "bmp180_rpi_driver.h"

int main(void)
{
    printf("Sensor test:\n");
    struct bmp180_sensor_data data;
	
    while(1) {
        int ret_val = bmp180_read(&data);
        if(ret_val < 0) {
            switch(ret_val) {
                case -1:
                    printf("Error: I2C init failed!\n");
                    break;
                case -2:
                    printf("Error: driver init failed!\n");
                    break;
                case -3:
                    printf("Error: sensor reading error!\n");
                    break;
            }
        } else {
            printf("Pressure %.2f mbar and Temperature %.2f C \n", data.pressure, data.temperature);
        }
		
        sleep(2);
    }
}