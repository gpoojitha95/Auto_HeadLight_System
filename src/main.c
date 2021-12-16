/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include <math.h>
#include "MPU6050_res_define.h"

#define PI 3.14159265
#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define SYS_FREQ 200000000                          // Define SYS CLK Frequency
#define SWITCH_PRESSED_STATE                    0   // Active LOW switch
// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************
static bool HL_SW = 0;
static bool HL_AUTO = 0;

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

void PWM_init(void)
{
	TMR2_Start();		// Turn on timer 2
    OCMP1_Enable();		// Turn on Output Compare module 1 (OC1)
    OCMP2_Enable();     // Turn on Output Compare module 2 (OC2)
    
    //Set all servos to Initial position
    OCMP1_CompareSecondaryValueSet(4687);
    OCMP2_CompareSecondaryValueSet(4687);
    HL_SW =0;
    HL_AUTO =0;
}

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

// I2C_init() initialises I2C1 at at frequency of [frequency]Hz
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

static void SW1_User_Handler(GPIO_PIN pin, uintptr_t context)
{
    if(SW1_Get() == SWITCH_PRESSED_STATE)
    {
        if (HL_SW == 0)
        {
            HL_SW = 1;
            HL_Relay_Clear();
            LED_GREEN_Clear();
        }
            
        else
        {
            HL_SW = 0;
            LED_GREEN_Set();
            HL_Relay_Set();
        }
            
    }
}

static void SW2_User_Handler(GPIO_PIN pin, uintptr_t context)
{
    if(SW2_Get() == SWITCH_PRESSED_STATE)
    {
        
        if (HL_AUTO == 0)
        {
            HL_AUTO = 1;  
            LED_ORANGE_Clear();
        }
        else
        {
            HL_AUTO = 0;
            LED_ORANGE_Set();
        }
            
    }
}


int main ( void )
{
    unsigned char reg_address = GYRO_XOUT_H; 
    int Gx,i;
    int HL_pos_write = 4687; //HL Horz position write value(Initialized to center)
    int HL_Vpos_write = 4687; //HL Vert position write value(Initialized to center)
    int prev_left = 0;  //Has the HL turned left 0-NO, 1- YES
    int prev_right = 0; //Has the HL turned right 0-NO, 1- YES
    int HL_pos = 4687; // HL initial position (center pos = 4687)
    //Set the sensitivity for the HL system to turn on ( 0[Daylight] to 4096[night])
    int Sensitivity_f = 2800; //Sensitivity factor for ambient light sensor
    int Xg;
    float AN5_in = 0; //Initialize Steering input 
    float AN2_in = 0; //Initialize Ambient light sensor input 
    LED_RED_Clear();
    I2C_init(100000);
    MPU6050_Init();
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    GPIO_PinInterruptCallbackRegister(SW1_PIN, SW1_User_Handler, 0);
    GPIO_PinInterruptEnable(SW1_PIN);
    GPIO_PinInterruptCallbackRegister(SW2_PIN, SW2_User_Handler, 0);
    GPIO_PinInterruptEnable(SW2_PIN);
    PWM_init();
    LED_RED_Set();
    LED_ORANGE_Set(); //ORANGE
    LED_GREEN_Set(); // GREEN
    HL_Relay_Set(); //Turn Off HL
    while ( true )
    {
        LE_RED_Set();
        delay_ms(10);
        ADCHS_ChannelConversionStart(5U); //Start ADC conversion for Analog Input 5
        ADCCON3bits.GSWTRG = 1;  // Trigger the conversion
        while(ADCDSTAT1bits.ARDY5 == 0); //Wait for the conversion to complete
        AN5_in = ADCDATA5; 
        ADCHS_ChannelConversionStart(2U); //Start ADC conversion for Analog Input 5
        ADCCON3bits.GSWTRG = 1;  // Trigger the conversion
        while(ADCDSTAT1bits.ARDY2 == 0); //Wait for the conversion to complete
        AN2_in = ADCDATA2; 
        if((AN2_in > Sensitivity_f)&(HL_AUTO == 1)){
            HL_SW = 1;  //turn HL_SW status to 1
            HL_Relay_Clear(); //turn HL On
            LED_GREEN_Clear();
        }
        else if((AN2_in < Sensitivity_f)&(HL_AUTO == 1)) {
            HL_SW = 0;  //turn HL_SW status to 1
            HL_Relay_Set(); //turn HL Off
            LED_GREEN_Set();
        }
        if ((HL_SW == 1) & (HL_AUTO == 1))
        {
            if (AN5_in >=1800 && AN5_in < 2200) // If steering is at center
            {
            if (HL_pos == 4687){
                if(HL_pos_write == 5200){       // If steering is at left bring it to center
                    for (i=HL_pos_write;i>=4687;i--){
                        OCMP2_CompareSecondaryValueSet(i);
                        delay_ms(1);
                    }
                }
                else if(HL_pos_write == 4000){     // If steering is at right bring it to center
                    for (i=HL_pos_write;i<=4687;i++){
                        OCMP2_CompareSecondaryValueSet(i); 
                        delay_ms(1);
                    } 
                }   
                HL_pos_write = 4687;
                OCMP2_CompareSecondaryValueSet(HL_pos); //persistant write 
                }
                else if (HL_pos == 5200) {
                   for (i=5200;i>=4687;i--){
                   OCMP2_CompareSecondaryValueSet(i); 

                   delay_ms(1);
                    }   
                }
                else if (HL_pos == 4000) {
                   for (i=4000;i<=4687;i++){
                   OCMP2_CompareSecondaryValueSet(i); 

                   delay_ms(1);
                }   
            }   
            prev_left = 0;
            prev_right = 0;
            HL_pos = 4687;
            OCMP2_CompareSecondaryValueSet(HL_pos); //persistant write 
            }
            else if ((AN5_in > 3200) && (prev_right == 0))
            {
                if (prev_right == 1){
                    HL_pos_write = 5200;
                }
                else{
                    HL_pos_write = 4687;
                }
                for (i=HL_pos_write;i>=4000;i--){
                   OCMP2_CompareSecondaryValueSet(i); 

                   delay_ms(1);
                }   
                prev_right = 1;
                HL_pos = 4000;
                OCMP2_CompareSecondaryValueSet(HL_pos); //persistant write
            }
            else if ((AN5_in < 1000 && AN5_in > 0 ) && (prev_left == 0))
            {
                if (prev_left == 1){
                    HL_pos_write = 4000;
                }
                else{
                    HL_pos_write = 4687;
                }
                for (i=HL_pos_write;i<=5200;i++){
                   OCMP2_CompareSecondaryValueSet(i); 
                   delay_ms(1);
                }   
                prev_left = 1;
                HL_pos = 5200;
                OCMP2_CompareSecondaryValueSet(HL_pos); //persistant write 
            }
            I2C_start();						/* Send start condition */  
            I2C_write(0XD0, 1);	/* Send MPU9250's address, read/write bit not set (AD + R) */  
            I2C_write(reg_address, 1);			/* Send the register address (RA) */  
            I2C_restart();						/* Send repeated start condition */  
            I2C_write(0xD1, 1);
            Gx = (((int)I2C22_read(0)<<8) | (int)I2C22_read(1));
            I2C_stop();
            Xg = Gx/131.0;
            if ((Xg > -0.31) & (Xg <= 0))
            {
                Xg = 0;
            }
            if(Xg > 3)
            {
                LE_RED_Clear();
                LE_GREEN_Set();
                LE_BLUE_Set();
                for (i=HL_Vpos_write;i>=4000;i--){
                    OCMP1_CompareSecondaryValueSet(i);
                    delay_ms(1.5);
                }
                HL_Vpos_write = 4000;
                //OCMP1_CompareSecondaryValueSet(HL_Vpos_write);
                Xg = 0;
                delay_ms(20);
                MPU6050_Init();
            }
            else if(Xg < -3)
            {
                LE_GREEN_Clear();
                LE_BLUE_Set();
                LE_RED_Set();
                for (i=HL_Vpos_write;i<=5149;i++){
                    OCMP1_CompareSecondaryValueSet(i);
                    delay_ms(1.5);
                }
                HL_Vpos_write = 5149;
                //OCMP1_CompareSecondaryValueSet(HL_Vpos_write);
                Xg = 0;
                delay_ms(20);
                MPU6050_Init();
                
            }
            LE_BLUE_Clear();
            LE_RED_Set();
            LE_GREEN_Set();
            if(HL_Vpos_write == 4000){
                for (i=HL_Vpos_write;i<=4687;i++){
                    OCMP1_CompareSecondaryValueSet(i);
                    delay_ms(1.5);
                }
                HL_Vpos_write = 4687;
                //OCMP1_CompareSecondaryValueSet(HL_Vpos_write);
                Xg = 0;
                delay_ms(20);
            }
            else if(HL_Vpos_write == 5149){
                for (i=HL_Vpos_write;i>=4687;i--){
                    OCMP1_CompareSecondaryValueSet(i);
                    delay_ms(1.5);
                }
                HL_Vpos_write = 4687;
                //OCMP1_CompareSecondaryValueSet(HL_Vpos_write);
                Xg = 0;
                delay_ms(20);

            }
            HL_Vpos_write = 4687;
            Xg = 0;
            delay_ms(20);
        }
        else
        {
            if (HL_AUTO == 0)
            {
                LED_ORANGE_Set();  
            }
            if (HL_SW == 0)
            {
                LED_GREEN_Set();
                LE_BLUE_Set();
            }    
                  
        }
        delay_ms(10);
        MPU6050_Init();
        LED_RED_Toggle();
        //OCMP1_CompareSecondaryValueSet(4687);
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
    }
    /* Execution should not come here during normal operation */
    return ( EXIT_FAILURE );
}
/*******************************************************************************
 End of File
*/

