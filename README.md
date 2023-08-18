# Avionics - Flight Software

## Initialisation_Test.ino
This contains code for initialisation of the flight computer. 

In the setup section, it checks the connection and readiness of the inbuilt SD Card, BMP280, MPU9250, SX1278 (LoRa library), with their begin() methods. Servos are also initialised and attached. 

In the loop, with the help of the millis function, we get consecutive readings from each sensor respectively, for specified durations. These readings are writtten to the SD Card, serial monitor and transmitted via the telemetry module, in the form of strings.

In the case of the MPU, the data that the sensor can have large deviations. In order to smoothen the data that is received, we use a Kalman Filter - the kalman() function. The signals() function, meanwhile, gives us the acceleration, angular velocity, and Euler angle data from the MPU. 
