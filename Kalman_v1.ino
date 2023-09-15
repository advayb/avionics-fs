#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

using namespace BLA;
float dt = 4;

// Define Kalman filter parameters
// VectorXd x(9); // State vector (position, velocity, acceleration)
// MatrixXd A(9, 9); // State transition matrix
Matrix<9, 9> A = {1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0, 0,
     0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0,
     0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt,
     0, 0, 0, 1, 0, 0, dt, 0, 0,
     0, 0, 0, 0, 1, 0, 0, dt, 0,
     0, 0, 0, 0, 0, 1, 0, 0, dt,
     0, 0, 0, 0, 0, 0, 1, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 1, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 1};



// MatrixXd P(9, 9); // State covariance matrix
// MatrixXd Q(9, 9); // Process noise covariance matrix
// MatrixXd H(3, 9); // Measurement matrix
// MatrixXd R(3, 3); // Measurement noise covariance matrix
void setup() {
  // Initialize the Kalman filter parameters
  Serial.begin(9600);

  // initKalmanFilter();
  Serial << A;
}
void loop() {
  // Simulate sensor measurements (replace with actual sensor readings)
  // VectorXd z(3); // Simulated 3D position measurement
  // Fill z with actual measurements here
  
  // // Prediction step
  // predict();
  
  // // Update step
  // update(z);
  
  // Output the estimated state (position, velocity, acceleration)
  // Serial.print("Estimated State (x): ");
  // for (int i = 0; i < x.size(); i++) {
  //   Serial.print(x(i));
  //   Serial.print("\t");
  // }
  // Serial.println();
  
  delay(100);
}
// void initKalmanFilter() {
//   // Initialize state vector, A, P, Q, H, R, and other parameters as needed
// }
// void predict() {
//   // Predict the next state (x) and covariance (P) here
// }
// void update(const VectorXd &z) {
//   // Update the state (x) and covariance (P) based on measurement (z) here
// }
// int main() {
//   setup();
//   while (true) {
//     loop();
//   }
//   return 0;
// } 