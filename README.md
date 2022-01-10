# Auto_HeadLight_System
## This project was developed and evaluated for ECE-5731 Course at Oakland Univeristy under Professor Dr Ramesh Sethu
A dynamic leveling and cornering headlight system using PIC32MZ2048EFM144 Microcontroller

### Problem Definition:
To design an automatic headlamp leveling system that controls the angle of headlamp such that
proper road illumination is achieved. The main objectives of this project are:
1. To dynamically control the pitch angle of the headlamp using gyroscope and provide 
proper illumination angle of the headlamp.
2. To control the headlamp angle during cornering and automate the headlamp leveling 
system.

### Approach:
![image](https://user-images.githubusercontent.com/96264299/148716868-ef81e7f8-d3f8-4e08-b9bd-302a5610e01f.png)

### Hardware Requirements:
The hardware components used for this project are as listed below:
1. PIC32 board - For the current project **PIC32MZ2048EFM144** microcontroller was used. 
2. Sensors:
a. Gyroscope/Accelerometer - **MPU6050**
Chip was used which includes two sensors - gyroscope that detects the change in 
the angle and accelerometer that detects the speed of any moving object. For this project, 
the gyroscope alone was used.
b. Photoresistor: 
Detects the intensity of the light and accordingly allows the flow of voltage.
3. Actuators - Stepper Motor 
Two **SG90** stepper motors were used. one for vertical movement and the other for 
horizontal movement.
4. Potentiometer 
Used as a steer to indicate the direction of the vehicle.ECE 5731 Embedded Computing in Mechatronics Fall 2021
7
Final Project Report
5. Relay- Used as a bridge the power difference between the headlamp and the board to avoid
damages.
6. Headlamp
7. 12V power adapter 
Used to power the headlamp via a relay
8. Bench setup
a. 3D-printed structures for supporting the headlamp system bench setup
![image](https://user-images.githubusercontent.com/96264299/148716933-8d1cb613-0b9c-42a3-8409-26c07f2a25b1.png)
![image](https://user-images.githubusercontent.com/96264299/148717022-1297fe7c-1733-4d4e-adce-4a45dc7f10ac.png)

### Software Requirements:
##### MPLAB IDE along with MPlAB Harmony Configurator -
MPLAB is proprietary free software that enables users to design, configure and debug embedded
systems. MPLAB is an integrated development environment that helps to communicate with
microcontrollers especially the PIC 32 and dsPIC (digital signal PICs) family line of 
microcontrollers. 

#### Demo video
[![Demo Video](https://user-images.githubusercontent.com/96264299/148718339-3cb18c7a-c83f-407c-aa16-dfd2606446aa.png)](https://youtu.be/t87c2LjkgIc)

#### References:
1. [How Adaptive Headlights Work | HowStuffWorks](https://auto.howstuffworks.com/adaptive-headlight.htm)
2. [User manual of PIC322048EFM144](http://ww1.microchip.com/downloads/en/DeviceDoc/Curiosity_PIC32MZEF2.0_Development_Board_Users_Guide_DS70005400A.pdf)
3. [autoENG3: Automatic Headlamp Levelling (euromotor.org)](https://www.euromotor.org/mod/resource/view.php?id=21787)
4. [PIC32MZ series I2C Tutorial](https://www.aidanmocke.com/blog/2018/11/27/i2c/)
5. [PIC32MZ series PWM tutorial](https://www.aidanmocke.com/blog/2018/11/16/pwm/)

Please feel to contact me on my email geethapoojithan@oakland.edu for any questions/concerns.


