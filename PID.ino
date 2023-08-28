#include <PID_v1.h>
#include <Servo.h>

//Define PID gains  
double kp = 1.0;
double ki = 0.1;
double kd = 0.2;

double setpoint = 0.0;
double input_x = 0.0;
double input_z = 0.0;
double output_x = 0.0;
double output_z = 0.0;

Servo servoX;
Servo servoZ;

void setup() {
    servoX.attach(/*X servo pin*/);
    servoZ.attach(/*Z servo pin*/);

    // Set PID modes and tuning parameters
    myPID_x.SetMode(AUTOMATIC);
    myPID_z.SetMode(AUTOMATIC);
}

PID myPID_x(&input_x, &output_x, &setpoint, kp, ki, kd, DIRECT);
PID myPID_z(&input_z, &output_z, &setpoint, kp, ki, kd, DIRECT);


void loop() {
    double angle_x = readAngleX(); // Replace with actual sensor reading
    double angle_z = readAngleZ(); // Replace with actual sensor reading

    input_x = angle_x;
    input_z = angle_z;

    myPID_x.Compute();
    myPID_z.Compute();

    adjustServoX(output_x);
    adjustServoZ(output_z);

    delay(10);
}

void adjustServoX(double control_x) {
    // double servoAngle = map(control_x, -255, 255, 0, 180);
    servoX.write(servoAngle);
}

void adjustServoZ(double control_z) {
    // double servoAngle = map(control_z, -255, 255, 0, 180);
    servoZ.write(servoAngle);
}
