#include <MPU6500_WE.h>
#include <quaternionFilters.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <SD.h>
#include <LoRa.h>

#define MPU6500_ADDR 0x68
#define BMP280_ADDRESS 0x76
Adafruit_BMP280 bmp;

int counter = 0;
//servo
const int servoPin1 = 9;
const int servoPin2 = 10;
Servo servo1;
Servo servo2;
int pos = 0;
//pyro
// const int pyro1 = 11;
// const int pyro2 = 12;

const int numReadings = 10; // Number of readings to average
const float processNoise_bmp = 0.01; // Process noise covariance
const float measurementNoise_bmp = 1.0; // Measurement noise covariance

// Variables for Kalman filter
float x_pred = 0; // Predicted state
float P_pred = 1; // Predicted covariance
float K; // Kalman gain
float x_est = 0; // Estimated state

float readings[numReadings]; // Array to store readings for averaging
int readIndex = 0; // Index to keep track of the current reading
float total = 0; // Running total of readings for averaging


MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
xyzFloat AccValue;
xyzFloat gyr;
xyzFloat angle;

float accX, accY, accZ; //acceleration in axes
float angleR, angleP, angleY; //angles of roll pitch blah

// float k_angleR, k_angleP, k_angleY; //kalman angles
// float kU_angleR = 4, kU_angleP = 4, kU_angleY = 4; //kalman uncertainties, assumed to be 4 in the start
// float kalman1D[2] = {0,0}; //array to store kalman angle and uncertainty for each roll, pitch and yaw


void setup() {
  // put your setup code here, to run once:
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  delay(2000);
  Serial.begin(9600);

  Wire.setClock(400000);
  Wire.begin();
  Serial.println("flight software initialisation"); 

  // if(!SD.begin(BUILTIN_SDCARD)){
  //   Serial.println("SD card initialisation failed!");
  // }

  if(!bmp.begin(BMP280_ADDRESS)){
    LoRa.write("Cannot find BMP280 sensor - check wiring!"); // correct this later, add lora to all status updates
    while(1);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);
 
  for (int i = 0; i < numReadings; i++) {//initialize array
    total + = bmp.readAltitude();
    readings[i] = bmp.readAltitude();
  }

  /*Servo setup*/
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  


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
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t prev_ms = 0;
  int sd_t, bmp_t, servo_t, mpu_t;
  //sd test

  if(millis() < sd_t){
  Serial.println("SD Card initialisation start");
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if(dataFile){
    dataFile.println("SD card works!");
    dataFile.close();
    String message = "Data written to SD Card";
    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket(true);
  }
  else{
    String message = "Error writing to SD Card";
    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket(true);
  }
  delay(2000);
  }
  prev_ms = millis();

  //bmp test
  if(millis() < bmp_t && millis() > sd_t){
  Serial.println("BMP initialisation start")
  float temp = bmp.readTemperature();
  float press = bmp.readPressure()/100;
  
  float rawValue = bmp.readAltitide();
  total = total + rawValue - readings[readIndex];
  readings[++readIndex] = rawValue;
  readIndex = readIndex % numReadings;

  // Calculate the average reading
  float averageValue = total / numReadings;

  // Kalman filter
  x_pred = x_est;
  P_pred = P_pred + processNoise;

  K = P_pred / (P_pred + measurementNoise);
  x_est = x_pred + K * (averageValue - x_pred);
  P_pred = (1 - K) * P_pred;

  Serial.println("BMP280 Test start");
  Serial.println("Temperature = ");
  Serial.println(temp);
  Serial.println("*C");

  Serial.println("Pressure = ");
  Serial.println(press);
  Serial.println("hPa");

  Serial.println("Approx altitude = ");
  Serial.println(x_est);
  Serial.println(" m");

  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile){
    String data = "Temp - " + String(temp);
    dataFile.println(data);
    String data = "Pressure - " + String(press);
    dataFile.println(data);
    String data = "Alt - " + String(x_est);
    dataFile.println(data);
    dataFile.close();
  }


  LoRa.beginPacket();
  String message = "Temp - " + String(temp) + " Pressure - " + String(press) + " Alt - " + String(x_est);
  LoRa.print(message);
  LoRa.endPacket(true);

  delay(50);

  }
  //servo test

  if(millis() < servo_t && millis() > sd_t){
  Serial.println("Servo initialisation start");
  testServos();
  delay(2000);

  }
  
  //mpu test
  if(millis() < mpu_t && millis() > servo_t){
      prev_ms = millis();
      if (millis() > prev_ms + 25) {
        angles();
        Serial.print("Roll, pitch: ");
        Serial.print(angleR, 4);
        Serial.print(", ");
        Serial.print(angleP, 4);
        // kalman(k_angleR, kU_angleR, rollR, angleR); //to get smoother and more accurate roll. same below for pitch and yaw
        // k_angleR = kalman1D[0];
        // kU_angleR = kalman1D[1];

        // kalman(k_angleP, kU_angleP, pitchR, angleP);
        // k_angleP = kalman1D[0];
        // kU_angleP = kalman1D[1];

        // kalman(k_angleY, kU_angleY, yawR, angleY);
        // k_angleY = kalman1D[0];
        // kU_angleY = kalman1D[1];

        // Serial.print("roll, pitch, yaw: ");
        // Serial.print(k_angleR, 4);
        // Serial.print(", ");
        // Serial.print(k_angleP, 4);
        // Serial.print(", ");
        // Serial.println(k_angleY, 4);

        File dataFile = SD.open("data.txt", FILE_WRITE);
        if (dataFile){
          String data = "Roll - " + String(angleR);
          dataFile.println(data);
          String data = " Pitch - " + String(angleP);
          dataFile.println(data);
          dataFile.close();
        }

        LoRa.beginPacket();
        String message = "Roll - " + String(angleR) + " Pitch - " + String(angleP);
        LoRa.print(message);
        LoRa.endPacket(true);



        /* if magnetic field values are needed
        Serial.print("Magnetic field in x,y,z: ");
        Serial.print(mpu.getMagX(), 4);
        Serial.print(", ");
        Serial.print(mpu.getMagY(), 4);
        Serial.print(", ");
        Serial.println(mpu.getMagZ(), 4);
        Serial.println("");
        */

        prev_ms = millis();
      }
    }
  delay(5000);
}

void testServos(){
    for (pos = 0; pos <= 180; pos += 1) {
    servo1.write(pos);
    servo2.write(pos);
   delay(15);
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    servo1.write(pos);
    servo2.write(pos);
    delay(15);
  }
}

// char* charArray(float data) {
//   String data_send = "Temp - " + String(data);
  
//   int data_len = data_send.length() + 1;
//   uint8_t total[data_len];
//   data_send.toCharArray((char*)total, data_len);
//   return (char*)total;
// }

void angles(){

  angleR = myMPU6500.getRoll();
  angleP = myMPU6500.getPitch();
  // rollR = mpu.getGyroX();
  // pitchR = mpu.getGyroY();
  // yawR = mpu.getGyroZ();

  // accX = mpu.getAccX();
  // accY = mpu.getAccY();
  // accZ = mpu.getAccZ();

  // angleR = 180*atan(accY/sqrt(accX*accX+accZ*accZ))/(3.14159);
  // angleP = -180*atan(accX/sqrt(accY*accY+accZ*accZ))/(3.14159);
  // angleY = 180*atan(accZ/sqrt(accX*accX+accZ*accZ))/(3.14159);
}
