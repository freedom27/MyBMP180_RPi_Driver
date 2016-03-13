#ifndef BMP180_RPI_DRIVER_H
#define BMP180_RPI_DRIVER_H

#define BMP180_OK                    0
#define BMP180_I2C_INIT_ERROR       -1
#define BMP180_DRIVER_INIT_ERROR    -2
#define BMP180_SENSOR_ERROR         -3

struct bmp180_sensor_data {
    float temperature;
    float pressure;
};

int bmp180_read(struct bmp180_sensor_data *sensor_data);

#endif
