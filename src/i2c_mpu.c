#include <config/pic32mz/peripheral/i2c/plib_i2c_master.h>
#include "i2c_mpu.h"
#include <config/pic32mz/peripheral/p32mz2048efm144.h>

void I2C_wait_for_idle(void)
{
    while(I2C1CON & 0x1F); // Acknowledge sequence not in progress
                                // Receive sequence not in progress
                                // Stop condition not in progress
                                // Repeated Start condition not in progress
                                // Start condition not in progress
    while(I2C1STATbits.TRSTAT); // Bit = 0 ? Master transmit is not in progress
}

// I2C_start() sends a start condition  
void I2C_start()
{
    I2C_wait_for_idle();
    I2C1CONbits.SEN = 1;
    while (I2C1CONbits.SEN == 1);
}

// I2C_stop() sends a stop condition  
void I2C_stop()
{
    I2C_wait_for_idle();
    I2C1CONbits.PEN = 1;
}

// I2C_restart() sends a repeated start/restart condition
void I2C_restart()
{
    I2C_wait_for_idle();
    I2C1CONbits.RSEN = 1;
    while (I2C1CONbits.RSEN == 1);
}
// I2C_ack() sends an ACK condition

void I2C_ack(void)
{
    I2C_wait_for_idle();
    I2C1CONbits.ACKDT = 0; // Set hardware to send ACK bit
    I2C1CONbits.ACKEN = 1; // Send ACK bit, will be automatically cleared by hardware when sent  
    while(I2C1CONbits.ACKEN); // Wait until ACKEN bit is cleared, meaning ACK bit has been sent
}

// I2C_nack() sends a NACK condition
void I2C_nack(void) // Acknowledge Data bit
{
    I2C_wait_for_idle();
    I2C1CONbits.ACKDT = 1; // Set hardware to send NACK bit
    I2C1CONbits.ACKEN = 1; // Send NACK bit, will be automatically cleared by hardware when sent  
    while(I2C1CONbits.ACKEN); // Wait until ACKEN bit is cleared, meaning NACK bit has been sent
}

// address is I2C slave address, set wait_ack to 1 to wait for ACK bit or anything else to skip ACK checking  
void I2C_write(unsigned char address, char wait_ack)
{
    I2C1TRN = address;				// Send slave address with Read/Write bit cleared
    while (I2C1STATbits.TBF == 1);		// Wait until transmit buffer is empty
    I2C_wait_for_idle();				// Wait until I2C bus is idle
    if (wait_ack)                       // Wait until ACK is received  
    {
        while (I2C1STATbits.ACKSTAT == 1)
        {
         ;   
        }
        
    } 
}

// value is the value of the data we want to send, set ack_nack to 0 to send an ACK or anything else to send a NACK  
char I2C_read(unsigned char *value, char ack_nack)
{
    char buffer;
    I2C1CONbits.RCEN = 1;				// Receive enable
    while (I2C1CONbits.RCEN);			// Wait until RCEN is cleared (automatic)  
    while (!I2C1STATbits.RBF);    		// Wait until Receive Buffer is Full (RBF flag)  
    *value = I2C1RCV;    				// Retrieve value from I2C1RCV
    buffer = I2C1RCV;
    if (!ack_nack)						// Do we need to send an ACK or a NACK?  
        I2C_ack();						// Send ACK  
    else
        I2C_nack();						// Send NACK 
    return buffer;
}

char I2C22_read(char ack_nack)
{
    char buffer;
    I2C1CONbits.RCEN = 1;				// Receive enable
    while (I2C1CONbits.RCEN);			// Wait until RCEN is cleared (automatic)  
    while (!I2C1STATbits.RBF);    		// Wait until Receive Buffer is Full (RBF flag)  
    //*value = I2C1RCV;    				// Retrieve value from I2C1RCV
    buffer = I2C1RCV;
    if (!ack_nack)						// Do we need to send an ACK or a NACK?  
        I2C_ack();						// Send ACK  
    else
        I2C_nack();						// Send NACK 
    return buffer;
}// I2C_init() initialises I2C1 at at frequency of [frequency]Hz
void I2C_init(double frequency)
{
    double BRG;
    
    I2C1CON = 0;			// Turn off I2C1 module
    I2C1CONbits.DISSLW = 1; // Disable slew rate for 100kHz
    
    BRG = (1 / (2 * frequency)) - 0.000000104;
    BRG *= (SYS_FREQ / 2) - 2;    

    I2C1BRG = (int)BRG;		// Set baud rate
    I2C1CONbits.ON = 1;		// Turn on I2C1 module
}
