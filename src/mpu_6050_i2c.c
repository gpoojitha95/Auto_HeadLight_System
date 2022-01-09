#include "mpu_6050_i2c.h"
#include <config/pic32mz/peripheral/i2c/plib_i2c_master.h>
#include <config/pic32mz/peripheral/i2c/plib_i2c1.h>
#include <config/pic32mz/peripheral/p32mz2048efm144.h>
#include "definitions.h"
#include "MPU6050_res_define.h"
void delay_us(unsigned int us)
{
    // Convert microseconds us into how many clock ticks it will take
    us *= (SYS_FREQ / 1000000) / 2; // Core Timer updates every 2 ticks
                      
    _CP0_SET_COUNT(0); // Set Core Timer count to 0

    while (us > _CP0_GET_COUNT()); // Wait until Core Timer count reaches the number we calculated earlier
}

void delay_ms(int ms)
{
    delay_us(ms * 1000);
}

void MPU6050_Init()		/* Gyro initialization function */
{
	delay_ms(150);		/* Power up time >100ms */
    I2C_start();
	I2C_write(0xD0,1);	/* Start with device write address */
	I2C_write(SMPLRT_DIV,1);	/* Write to sample rate register */
	I2C_write(0x07,1);	/* 1KHz sample rate */
	I2C_stop();

	I2C_start();
    I2C_write(0xD0,1);
	I2C_write(PWR_MGMT_1,1);	/* Write to power management register */
	I2C_write(0x01,1);	/* X axis gyroscope reference frequency */
	I2C_stop();

	I2C_start();
    I2C_write(0xD0,1);
	I2C_write(CONFIG,1);	/* Write to Configuration register */
	I2C_write(0x00,1);	/* Fs = 8KHz */
	I2C_stop();

	I2C_start();
    I2C_write(0xD0,1);
	I2C_write(GYRO_CONFIG,1);	/* Write to Gyro configuration register */
	I2C_write(0x18,1);	/* Full scale range +/- 2000 degree/C */
	I2C_stop();

	I2C_start();
    I2C_write(0xD0,1);
	I2C_write(INT_ENABLE,1);	/* Write to interrupt enable register */
	I2C_write(0x01,1);
	I2C_stop();
}

void MPU_Start_Loc(unsigned char *value)

{
    //unsigned char value;
    I2C_start();
	I2C_write(0xD0,1);	/* I2C start with device write address */
	I2C_write(ACCEL_XOUT_H,1);/* Write start location address to read */ 
    I2C_restart();	
	I2C_write(0xD1,1);/* I2C start with device read address */
    I2C_read(value, 1);
    I2C_stop();
}

void MPU6050_read()
{
    //char buff;
    unsigned char reg_address = GYRO_XOUT_H;   // address for Gyro X register
    I2C_start();						/* Send start condition */  
    I2C_write(0XD0, 1);	/* Send MPU9250's address, read/write bit not set (AD + R) */  
    I2C_write(reg_address, 1);			/* Send the register address (RA) */  
    I2C_restart();						/* Send repeated start condition */  
    I2C_write(0xD1, 1);	/* Send MPU9250's address, read/write bit set (AD + W) */  
    //buff = I2C_read(value, 1);					/* Read value from the I2C bus */  
    //I2C_stop();  
    //return buff;    /* Send stop condition */  
}
