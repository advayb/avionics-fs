#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <Adafruit_BMP280.h>

#include <MPU6500_WE.h>
#include <Wire.h>
#define MPU6500_ADDR 0x68
#define BMP280_ADDRESS 0x77

using namespace BLA;

// Adafruit_BMP280 bmp;
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
float dt = 0.03;

unsigned long time_now;
unsigned long time_last;
// Define Kalman filter parameters
Matrix<8,1> z;
Matrix<13,1> x = {0,0,0,0,0,0,0,0,0,1,0,0,0};
  // State vector (position, velocity, acceleration)
Matrix<13,13> A;
Matrix<13,13> A_id = {1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0, 0,0,0,0,0,
                0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0, 0,0,0,0,
                0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt,0,0,0,0,
                0, 0, 0, 1, 0, 0, dt, 0, 0,0,0,0,0,
                0, 0, 0, 0, 1, 0, 0, dt, 0,0,0,0,0,
                0, 0, 0, 0, 0, 1, 0, 0, dt,0,0,0,0,
                0, 0, 0, 0, 0, 0, 1, 0, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 1, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 1,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,1,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,1,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,1,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,1};
                 // State transition matrix
Matrix<13,13> P = {1,0,0,0,0,0,0,0,0,0,0,0,0,
                0,1,0,0,0,0,0,0,0,0,0,0,0,
                0,0,1,0,0,0,0,0,0,0,0,0,0,
                0,0,0,1,0,0,0,0,0,0,0,0,0,
                0,0,0,0,1,0,0,0,0,0,0,0,0, // State covariance matrix
                0,0,0,0,0,1,0,0,0,0,0,0,0,
                0,0,0,0,0,0,1,0,0,0,0,0,0,
                0,0,0,0,0,0,0,1,0,0,0,0,0,
                0,0,0,0,0,0,0,0,1,0,0,0,0,
                0,0,0,0,0,0,0,0,0,1,0,0,0,
                0,0,0,0,0,0,0,0,0,0,1,0,0,
                0,0,0,0,0,0,0,0,0,0,0,1,0,
                0,0,0,0,0,0,0,0,0,0,0,0,1};

Matrix<13,13> Q = {1,0,0,0,0,0,0,0,0,0,0,0,0,
                  0,1,0,0,0,0,0,0,0,0,0,0,0,
                  0,0,1,0,0,0,0,0,0,0,0,0,0,
                  0,0,0,1,0,0,0,0,0,0,0,0,0,
                  0,0,0,0,1,0,0,0,0,0,0,0,0,
                  0,0,0,0,0,1,0,0,0,0,0,0,0,
                  0,0,0,0,0,0,1,0,0,0,0,0,0,
                  0,0,0,0,0,0,0,1,0,0,0,0,0,
                  0,0,0,0,0,0,0,0,1,0,0,0,0,
                  0,0,0,0,0,0,0,0,0,0.001,0,0,0,
                  0,0,0,0,0,0,0,0,0,0,0.001,0,0,
                  0,0,0,0,0,0,0,0,0,0,0,0.001,0,
                  0,0,0,0,0,0,0,0,0,0,0,0,0.001,};
                  
                   // Process noise covariance matrix
Matrix<8,13> H = {0,0,1,0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,1,0,0,0,0,0,0,
                0,0,0,0,0,0,0,1,0,0,0,0,0,
                0,0,0,0,0,0,0,0,1,0,0,0,0,
                0,0,0,0,0,0,0,0,0,1,0,0,0,
                0,0,0,0,0,0,0,0,0,0,1,0,0,
                0,0,0,0,0,0,0,0,0,0,0,1,0,
                0,0,0,0,0,0,0,0,0,0,0,0,1,};// Measurement matrix
Matrix<8,8> R= {4,0,0,0,0,0,0,0,
               0,1,0,0,0,0,0,0,
               0,0,1,0,0,0,0,0,
               0,0,0,1,0,0,0,0,
               0,0,0,0,5,0,0,0,
               0,0,0,0,0,5,0,0,
               0,0,0,0,0,0,5,0,
               0,0,0,0,0,0,0,5}; // Measurement noise covariance matrix

Matrix<13,8> K;

struct Quaternion {
  float w, x, y, z;
};

void setup() {
  // Initialize the Kalman filter parameters
  Serial.begin(115200);
  
  Wire.setClock(400000);
  Wire.begin();
  // if(!bmp.begin(BMP280_ADDRESS)){
  //   Serial.println("Cannot find BMP280 sensor - check wiring!");
  //   while(1);
  // }
  // bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);

  // for (int i = 0; i < numReadings; i++) {//initialize array
  //   total += bmp.readAltitude();
  //   readings[i] = bmp.readAltitude();
  // }

  // if(!myMPU6500.init()){
  //   Serial.println("MPU6500 does not respond");
  //   while(1);
  // }
  // // else{
  //   Serial.println("MPU6500 is connected");
  // }
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

 //figure out timing of kalman, first dt, and launch start

  checkFlag = Serial.parseInt();
  if(checkFlag ==1){
    AccValue = myMPU6500.getGValues();
    z(0) = 0; //bmp.ReadAltitude
    z(1) = 9.81*AccValue.x;
    z(2) = 9.81*AccValue.y;
    z(3) = 9.81*(AccValue.z-1);

    roll = 180*atan(AccValue.y/sqrt(AccValue.x*AccValue.x + AccValue.z*AccValue.z))/(3.14159);
    pitch = -180*atan(AccValue.x/sqrt(AccValue.y*AccValue.y+AccValue.z*AccValue.z))/(3.14159);
  //initial orientation measurement



    Serial << z;
  }
  else{
    Serial.println("Incorrect setup");
  }


  initKalmanFilter();
  printMatrix(~R);

}
void loop() {
  time_now = millis();
  if(time_now - time_last >= 50){
  // Simulate sensor measurements (replace with actual sensor readings)
  // VectorXd z(3); // Simulated 3D position measurement
  // Fill z with actual measurements here
  
  gyr = myMPU6500.getGyrValues();
  w1 = gyr.x;
  w2 = gyr.y;
  w3 = gyr.z;

  AccValue = myMPU6500.getGValues();
  
  z(0) = 0;
  z(1) = 9.81*AccValue.x;
  z(2) = 9.81*AccValue.y;
  z(3) = 9.81*AccValue.z;




  //loop orientation measurement;
  roll = 180*atan(z(2)/sqrt(z(1)*z(1)+z(3)*z(3)))/(3.14159);
  pitch = -180*atan(z(1)/sqrt(z(2)*z(2)+z(3)*z(3)))/(3.14159);
  yaw = yaw + w3*dt;

  Quaternion q = eulerToQuaternion(roll, pitch, yaw);

  z(3) = z(3) - 9.81;
  z(4) = q.w;
  z(5) = q.x;
  z(6) = q.y;
  z(7) = q.z;

  Serial.println("ini");
  Serial << z;

  // // Prediction step
  predict();
  
  // // Update step
  update(z);
  Serial << x;
  Serial.println("aft");
  // Output the estimated state (position, velocity, acceleration)
  // Serial.print("Estimated State (x): ");
  // for (int i = 0; i < x.size(); i++) {
  //   Serial.print(x(i));
  //   Serial.print("\t");
  // }
  // Serial.println();
  time_last = millis();
  }

}
void printMatrix(Matrix<8,8> A){
  Serial << A;
}
void initKalmanFilter() {
  // Initialize state vector, A, P, Q, H, R, and other parameters as needed
}
void predict() {
  // Predict the next state (x) and covariance (P) here
   Matrix<13,13> del_A = {0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0,-0.5*dt*w1,-0.5*dt*w2,-0.5*dt*w3,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0.5*dt*w1,0,0.5*dt*w3,-0.5*dt*w2,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0.5*dt*w2,-0.5*dt*w3,0,0.5*dt*w1,
                0, 0, 0, 0, 0, 0, 0, 0, 0,0.5*dt*w3,0.5*dt*w2,-0.5*dt*w1,0};
  
  A = A_id + del_A;
  x = A * x;
  P = A*P*(~A) + Q;
}
void update(Matrix<8,1> z) {
  // Update the state (x) and covariance (P) based on measurement (z) here
  K = P*(~H)*(Inverse(H*P*(~H) + R));
  x = x + K*(z-(H*x));
  P = P - K*H*P;
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