#include <PID_v1.h>
#include <Servo.h>

// Define PID gains
double kp = 1.0;
double ki = 0.1;
double kd = 0.2;

double setpoint = 0.0; // Desired trajectory value
double input_x = 0.0;
double input_y = 0.0;

void loop()
{
    double angle_x = readAngleX;
    double angle_y = readAngleY;

    input_x = angle_x;
    input_y = angle_y;

    myPID_x.Compute();
    myPID_y.Compute();

    adjustServoX(output_x);
    adjustServoY(output_y);

    delay(10);
}
PID myPID_x(&input_x, &output_x, &setpoint, kp, ki, kd, DIRECT);
PID myPID_y(&input_y, &output_y, &setpoint, kp, ki, kd, DIRECT);

void adjustServoX(double control_x){
    double servoAngle = map(control_x, -255, 255, 0, 180);
    servoX.write(servoAngle);
}
void adjustServoY(double control_y){
    double servoAngle = map(control_y, -255, 255, 0, 180);
    servoX.write(servoAngle);
}
