#ifndef RPI_UTILS_H
#define RPI_UTILS_H

typedef enum {
    RPI_UNKNOWN,
    RPI_1B_1,
    RPI_1B_2,
    RPI_1A_2,
    RPI_1BPLUS_1,
    RPI_1APLUS_1,
    RPI_1BPLUS_12,
    RPI_2B_11,
    RPI_ZERO_12,
    RPI_3B_12
} rpi_model;

rpi_model get_rpi_model(void);

#endif