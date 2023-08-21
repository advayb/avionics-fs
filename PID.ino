#include <PID_v1.h>
#include <Servo.h>

// Define PID gains
double kp = 1.0;
double ki = 0.1;
double kd = 0.2;

double setpoint = 0.0; // Desired trajectory value
double input_x = 0.0;
double input_z = 0.0;

void loop()
{
    double angle_x = readAngleX;
    double angle_z = readAngleZ;

    input_x = angle_x;
    input_z = angle_z;

    myPID_x.Compute();
    myPID_z.Compute();

    adjustServoX(output_x);
    adjustServoZ(output_z);

    delay(10);
}
PID myPID_x(&input_x, &output_x, &setpoint, kp, ki, kd, DIRECT);
PID myPID_z(&input_z, &output_z, &setpoint, kp, ki, kd, DIRECT);

void adjustServoX(double control_x){
    // double servoAngle = map(control_x, -255, 255, 0, 180);
    servoX.write(servoAngle);
}
void adjustServoY(double control_z){
    // double servoAngle = map(control_z, -255, 255, 0, 180);
    servoZ.write(servoAngle);
}
