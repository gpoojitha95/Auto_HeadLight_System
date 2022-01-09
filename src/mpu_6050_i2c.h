/* 
 * File:   mpu_6050_i2c.h
 * Author: geeth
 *
 * Created on December 16, 2021, 5:13 PM
 */

#ifndef MPU_6050_I2C_H
#define	MPU_6050_I2C_H
#include "i2c_mpu.h"
#ifdef	__cplusplus
extern "C" {
#endif

void delay_ms(int ms);
void MPU6050_Init();
void MPU6050_Init();
void MPU_Start_Loc(unsigned char *value);
void MPU6050_read();


#ifdef	__cplusplus
}
#endif

#endif	/* MPU_6050_I2C_H */

