# ProiectSTM32

3D real-time simulation of an object using C/C++, Python, Blender

Abstract

- Implementation of a visual platform to observe real-time movements of an object.                                    
- The position data is provided by MPU6050/9250 sensors.                                                             
- ARM-Cortex M7 microcontroller will be used to process the received data.                                          

Implementation

● Receiving the sensor data using I2C communication protocol                                          
● Processing the data & reducing the noise                                              
● Representing the accelerometer's 3 axis and gyroscope's 3 axis as quaternions using Madgwick algorithm               
● Serial transmission of quaternions using UART                                                     
● Development of the visual platform of the 3D object and real-time movements of the object.                                
![image](https://user-images.githubusercontent.com/53474954/108990750-f52a4d80-769f-11eb-94c3-a465c76b3ab0.png)

Setup

● MPU6050 Accelerometer and Gyroscope Module                                               
● Microcontroller ARM Cortex-M7                                                      
● F232R USB UART                                                                      
![image](https://user-images.githubusercontent.com/53474954/108990939-26a31900-76a0-11eb-8a05-5432726f2088.png)

Tools

- PuTTy                                                                                             
- STM32 Cube & Open STM32 tools                                                                      
- Blender                                                                                        
- Pycharm                                                                                       


Blender object
![image](https://user-images.githubusercontent.com/53474954/108992045-7f26e600-76a1-11eb-90fb-a4812dfeeabf.png)

Results 
                                                                                                               
![image](https://user-images.githubusercontent.com/53474954/108992953-95817180-76a2-11eb-9f3f-9969f60eacb6.png)
![image](https://user-images.githubusercontent.com/53474954/108992992-a0d49d00-76a2-11eb-959d-b81a69078369.png)
![image](https://user-images.githubusercontent.com/53474954/108993074-ba75e480-76a2-11eb-815a-2b695cff4a42.png)

