/* 
 * File:   i2c_mpu.h
 * Author: geeth
 *
 * Created on December 16, 2021, 5:27 PM
 */

#ifndef I2C_MPU_H
#define	I2C_MPU_H
#define SYS_FREQ 200000000                          // Define SYS CLK Frequency

#ifdef	__cplusplus
extern "C" {
#endif
void I2C_wait_for_idle(void);
void I2C_start();
void I2C_stop();
void I2C_restart();
void I2C_ack(void);
void I2C_nack(void);
void I2C_write(unsigned char address, char wait_ack);
char I2C_read(unsigned char *value, char ack_nack);
char I2C22_read(char ack_nack);
void I2C_init(double frequency);




#ifdef	__cplusplus
}
#endif

#endif	/* I2C_MPU_H */

