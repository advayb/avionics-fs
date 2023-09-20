#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <Adafruit_BMP280.h>
#include <MPU6500_WE.h>
#include <Wire.h>
#define MPU6500_ADDR 0x68
#define BMP280_ADDRESS 0x77
using namespace BLA;

float dt;
Adafruit_BMP280 bmp;
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

int checkFlag = 0;
float dt = 0.03;

unsigned long time_now;
unsigned long time_last;
// Define Kalman filter parameters
Matrix<4,1> z;
Matrix<9,1> x = {0,0,0,0,0,0,0,0,0};
  // State vector (position, velocity, acceleration)
Matrix<9,9> A = {1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0, 0,
                0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0,
                0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt,
                0, 0, 0, 1, 0, 0, dt, 0, 0,
                0, 0, 0, 0, 1, 0, 0, dt, 0,
                0, 0, 0, 0, 0, 1, 0, 0, dt,
                0, 0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 1}; // State transition matrix

Matrix<9,9> P = {1,0,0,0,0,0,0,0,0,
                0,1,0,0,0,0,0,0,0,
                0,0,1,0,0,0,0,0,0,
                0,0,0,1,0,0,0,0,0,
                0,0,0,0,1,0,0,0,0, // State covariance matrix
                0,0,0,0,0,1,0,0,0,
                0,0,0,0,0,0,1,0,0,
                0,0,0,0,0,0,0,1,0,
                0,0,0,0,0,0,0,0,1};

Matrix<9,9> Q = {1,0,0,0,0,0,0,0,0,
                  0,1,0,0,0,0,0,0,0,
                  0,0,1,0,0,0,0,0,0,
                  0,0,0,1,0,0,0,0,0,
                  0,0,0,0,1,0,0,0,0,
                  0,0,0,0,0,1,0,0,0,
                  0,0,0,0,0,0,1,0,0,
                  0,0,0,0,0,0,0,1,0,
                  0,0,0,0,0,0,0,0,1};
                   // Process noise covariance matrix
Matrix<4,9> H = {0,0,1,0,0,0,0,0,0,
                0,0,0,0,0,0,1,0,0,
                0,0,0,0,0,0,0,1,0,
                0,0,0,0,0,0,0,0,1};// Measurement matrix
Matrix<4,4> R= {4,0,0,0,
               0,1,0,0,
               0,0,1,0,
               0,0,0,1}; // Measurement noise covariance matrix

Matrix<9,4> K;

void setup() {
  // Initialize the Kalman filter parameters
  Serial.begin(115200);
  
  Wire.setClock(400000);
  Wire.begin();
  if(!bmp.begin(BMP280_ADDRESS)){
    Serial.println("Cannot find BMP280 sensor - check wiring!");
    while(1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);

  for (int i = 0; i < numReadings; i++) {//initialize array
    total += bmp.readAltitude();
    readings[i] = bmp.readAltitude();
  }

  if(!myMPU6500.init()){
    Serial.println("MPU6500 does not respond");
    while(1);
  }
  else{
    Serial.println("MPU6500 is connected");
  }
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
  checkFlag = Serial.parseInt();
  if(checkFlag ==1){
    AccValue = myMPU6500.getGValues();
    z(0) = bmp.readAltitude();
    z(1) = AccValue.x;
    z(2) = AccValue.y;
    z(3) = AccValue.z;
    Serial << z;
  }
  else{
    Serial.println("Incorrect setup");
  }


  initKalmanFilter();


}
void loop() {
  time_now = millis();
  if(time_now - time_last >= 50){
  // Simulate sensor measurements (replace with actual sensor readings)
  // VectorXd z(3); // Simulated 3D position measurement
  // Fill z with actual measurements here
  AccValue = myMPU6500.getGValues();
  
  z(1) = AccValue.x;
  z(2) = AccValue.y;
  z(3) = AccValue.z;
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
void printMatrix(Matrix<9,9> A){
  Serial << A;
}
void initKalmanFilter() {
  // Initialize state vector, A, P, Q, H, R, and other parameters as needed
}
void predict() {
  // Predict the next state (x) and covariance (P) here
  x = A * x;
  P = A*P*(~A) + Q;
}
void update(const Matrix<4,1> z) {
  // Update the state (x) and covariance (P) based on measurement (z) here
  K = P*(~H)*(Inverse(H*P*(~H) + R));
  x = x + K*(z-(H*x));
  P = P - K*H*P;
}
// int main() {
//   setup();
//   while (true) {
//     loop();
//   }
//   return 0;
// }
