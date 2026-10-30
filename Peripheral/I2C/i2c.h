#ifndef __I2C_H
#define __I2C_H

#include "sys.h"

/* I2C0 自身地址 (7位) */
#define I2C0_OWN_ADDRESS7      0x30
/* 默认从机地址 (MMC5983MA) */
#define I2C0_SLAVE_ADDRESS7    0x30

void i2c0_config(void);

#endif
