//Only kalman output readings, no pid

/*
 * Run an example of Kalman filter.
 * This example simulates a sinusoidal position of an object.
 * The 'SIMULATOR_' functions below simulate the physical process and its measurement with sensors. 
 * Results are printed on Serial port. You can use 'kalman_full.py' to analyse them with Python.
 * 
 * Author:
 *  R.JL. Fétick
 *  
 * Revision:
 *  31 Aug 2019 - Creation
 * 
 */

#include <Kalman.h>

#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <Adafruit_BMP280.h>
#include <PID_v1.h>
#include <Servo.h>

 
#include <MPU6500_WE.h>
#include <Wire.h>
#define MPU6500_ADDR 0x68
#define BMP280_ADDRESS 0x77

using namespace BLA;


const int numReadings = 10; // Number of readings to average
const float processNoise_bmp = 0.01; // Process noise covariance
const float measurementNoise_bmp = 1.0; // Measurement noise covariance
float readings[numReadings]; // Array to store readings for averaging
int readIndex = 0; // Index to keep track of the current reading
float total = 0; // Running total of readings for averaging
 
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
xyzFloat AccValue;
xyzFloat gyr;
xyzFloat angle;
 
float roll=0,pitch=0,yaw=0;
 
float w1=0,w2=0,w3=0;
 
int checkFlag = 0;



struct Quaternion {
  float w, x, y, z;
  // Quaternion(float _w, float _x, float _y, float _z) : w(_w), x(_x), y(_y), z(_z) {}
};
 

//Define PID gains  
// double kp = 3;
// double ki = 3;
// double kd = 5;

// double setpoint = 0.0;
// double input_x = 0.0;
// double input_z = 0.0;
// double output_x = 0.0;
// double output_z = 0.0;

Quaternion q;
//------------------------------------
/****  MODELIZATION PARAMETERS  ****/
//------------------------------------

#define Nstate 13 // position, speed, acceleration
#define Nobs 8   // position, acceleration

// // measurement std of the noise
// #define n_p 0.3 // position measurement noise
// #define n_a 5.0 // acceleration measurement noise

// // model std (1/inertia)
// #define m_p 0.1
// #define m_s 0.1
// #define m_a 0.8

BLA::Matrix<Nobs> obs; // observation vector
KALMAN<Nstate,Nobs> K; // your Kalman filter
unsigned long T; // current time
float DT; // delay between two updates of the filter

// Note: I made 'obs' a global variable so memory is allocated before the loop.
//       This might provide slightly better speed efficiency in loop.


//------------------------------------
/****    SIMULATOR PARAMETERS   ****/
//------------------------------------

// These variables simulate a physical process to be measured
// In real life, the SIMULATOR is replaced by your operational system

BLA::Matrix<Nstate> state; // true state vector

#define SIMUL_PERIOD 0.3 // oscillating period [s]
#define SIMUL_AMP 1.0    // oscillation amplitude
#define LOOP_DELAY 10    // add delay in the measurement loop [ms]


//------------------------------------
/****        SETUP & LOOP       ****/
//------------------------------------

void setup() {

  Serial.begin(115200);
  
  Wire.setClock(400000);
  Wire.begin();
  // time evolution matrix (whatever... it will be updated inloop)
  K.F = {1, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
                0, 0, 1, 0, 0, 0, 0, 0, 0,0,0,0,0,
                0, 0, 0, 1, 0, 0, 0, 0, 0,0,0,0,0,
                0, 0, 0, 0, 1, 0, 0, 0 , 0,0,0,0,0,
                0, 0, 0, 0, 0, 1, 0, 0, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 1, 0, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 1, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 1,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,1,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,1,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,1,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,1};

  // measurement matrix n the position (e.g. GPS) and acceleration (e.g. accelerometer)
  K.H = {0,0,1,0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,1,0,0,0,0,0,0,
                0,0,0,0,0,0,0,1,0,0,0,0,0,
                0,0,0,0,0,0,0,0,1,0,0,0,0,
                0,0,0,0,0,0,0,0,0,1,0,0,0,
                0,0,0,0,0,0,0,0,0,0,1,0,0,
                0,0,0,0,0,0,0,0,0,0,0,1,0,
                0,0,0,0,0,0,0,0,0,0,0,0,1};
  // measurement covariance matrix
  K.R = {4,0,0,0,0,0,0,0,
               0,1,0,0,0,0,0,0,
               0,0,1,0,0,0,0,0,
               0,0,0,1,0,0,0,0,
               0,0,0,0,0.05,0,0,0,
               0,0,0,0,0,0.05,0,0,
               0,0,0,0,0,0,0.05,0,
               0,0,0,0,0,0,0,0.05};
  // model covariance matrix
  K.Q = {1,0,0,0,0,0,0,0,0,0,0,0,0,
                  0,1,0,0,0,0,0,0,0,0,0,0,0,
                  0,0,1,0,0,0,0,0,0,0,0,0,0,
                  0,0,0,1,0,0,0,0,0,0,0,0,0,
                  0,0,0,0,1,0,0,0,0,0,0,0,0,
                  0,0,0,0,0,1,0,0,0,0,0,0,0,
                  0,0,0,0,0,0,1,0,0,0,0,0,0,
                  0,0,0,0,0,0,0,1,0,0,0,0,0,
                  0,0,0,0,0,0,0,0,1,0,0,0,0,
                  0,0,0,0,0,0,0,0,0,0.1,0,0,0,
                  0,0,0,0,0,0,0,0,0,0,0.1,0,0,
                  0,0,0,0,0,0,0,0,0,0,0,0.1,0,
                  0,0,0,0,0,0,0,0,0,0,0,0,0.1};
  
  
  
  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6500.autoOffsets();
  Serial.println("Done!");
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);

  Serial.println("Please enter 1 when ready");
  while (!Serial.available()) {
          ; // Wait for user input
        }

  T = millis();
  // INITIALIZE SIMULATION
  
}

void loop() {
	
  // TIME COMPUTATION
  DT = (millis()-T)/1000.0;
  T = millis();

  // UPDATE STATE EQUATION
  // Here we make use of the Taylor expansion on the (position,speed,acceleration)
  // position_{k+1} = position_{k} + DT*speed_{k} + (DT*DT/2)*acceleration_{k}
  // speed_{k+1}    = speed_{k} + DT*acceleration_{k}
  // acceleration_{k+1} = acceleration_{k}
  K.F = {1, 0, 0, DT, 0, 0, 0.5*DT*DT, 0, 0,0,0,0,0,
                0, 1, 0, 0, DT, 0, 0, 0.5*DT*DT, 0, 0,0,0,0,
                0, 0, 1, 0, 0, DT, 0, 0, 0.5*DT*DT,0,0,0,0,
                0, 0, 0, 1, 0, 0, DT, 0, 0,0,0,0,0,
                0, 0, 0, 0, 1, 0, 0, DT, 0,0,0,0,0,
                0, 0, 0, 0, 0, 1, 0, 0, DT,0,0,0,0,
                0, 0, 0, 0, 0, 0, 1, 0, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 1, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 1,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,-0.5*DT*w1,-0.5*DT*w2,-0.5*DT*w3,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0.5*DT*w1,0,0.5*DT*w3,-0.5*DT*w2,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0.5*DT*w2,-0.5*DT*w3,0,0.5*DT*w1,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0.5*DT*w3,0.5*DT*w2,-0.5*DT*w1,0};;

  // UPDATE THE SIMULATED PHYSICAL PROCESS
  
  // SIMULATE A NOISY MEASUREMENT WITH A SENSOR
  // Result of the measurement is written into 'obs'
  SYSTEM_MEASURE();
  
  // APPLY KALMAN FILTER
  K.update(obs);

  // PRINT RESULTS: true state, measurements, estimated state, posterior covariance
  // The most important variable for you might be the estimated state 'K.x'
  Serial << state << ' ' << obs << ' ' << K.x << ' ' << K.P << '\n';
}

//------------------------------------
/****     SIMULATOR FUNCTIONS   ****/
//------------------------------------

void SIMULATOR_INIT(){
  // Initialize stuff for the simulator
  randomSeed(analogRead(0));
  state.Fill(0.0);
  obs.Fill(0.0);
}



void SYSTEM_MEASURE(){
  // Simulate a noisy measurement of the physical process
  gyr = myMPU6500.getGyrValues();
  AccValue = myMPU6500.getGValues();
  w1 = gyr.x;
  w2 = gyr.y;
  w3 = gyr.z;
  obs(0) = 0;
  obs(1) = 9.81*AccValue.x;
  obs(2) = 9.81*AccValue.y;
  obs(3) = 9.81*AccValue.z;
  roll = 180*atan(obs(2)/sqrt(obs(1)*obs(1)+obs(3)*obs(3)))/(3.14159);
  pitch = -180*atan(obs(1)/sqrt(obs(2)*obs(2)+obs(3)*obs(3)))/(3.14159);
  yaw = yaw + w3*DT;
 Quaternion q = eulerToQuaternion(roll, pitch, yaw);
 normalizeQuaternion(q);
 
  obs(3) = obs(3) - 9.81;
  obs(4) = q.w;
  obs(5) = q.x;
  obs(6) = q.y;
  obs(7) = q.z;


   //simulate a delay in the measurement
}

double SIMULATOR_GAUSS_NOISE(){
  // Generate centered reduced Gaussian random number with Box-Muller algorithm
  double u1 = random(1,10000)/10000.0;
  double u2 = random(1,10000)/10000.0;
  return sqrt(-2*log(u1))*cos(2*M_PI*u2);
}

float constrainTo180(float angle) {
    angle = fmod(angle + 180, 360);
    if (angle < 0)
        angle += 360;
    return angle - 180;
}

float getYaw(const Quaternion &q) {
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    float yaw = atan2(siny_cosp, cosy_cosp);
    return constrainTo180(yaw * 180.0 / M_PI);
}

float getPitch(const Quaternion &q) {
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    float pitch;
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);
    return constrainTo180(pitch * 180.0 / M_PI);
}

float getRoll(const Quaternion &q) {
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    float roll = atan2(sinr_cosp, cosr_cosp);
    return constrainTo180(roll * 180.0 / M_PI);
}

void normalizeQuaternion(Quaternion &q) {
    float norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
}

Quaternion eulerToQuaternion(float roll, float pitch, float yaw) {
  Quaternion q;
  
  // Convert degrees to radians
  roll = radians(roll);
  pitch = radians(pitch);
  yaw = radians(yaw);
 
  // Calculate half angles
  float cosRoll = cos(roll / 2);
  float sinRoll = sin(roll / 2);
  float cosPitch = cos(pitch / 2);
  float sinPitch = sin(pitch / 2);
  float cosYaw = cos(yaw / 2);
  float sinYaw = sin(yaw / 2);
 
  // Calculate quaternion components
  q.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
  q.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  q.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  q.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
 
  return q;
}