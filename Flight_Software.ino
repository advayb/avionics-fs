#include <MPU9250.h>
#include <quaternionFilters.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <Servo.h>
#include <SD.h>
#include <LoRa.h>

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
const int pyro1 = 11;
const int pyro2 = 12;


MPU9250 mpu;
int t_rate; //transmission rate of data from serial
float rollR, pitchR, yawR; //rates of change of roll blah blah in rad/s

float accX, accY, accZ; //acceleration in axes
float angleR, angleP, angleY; //angles of roll pitch blah

float k_angleR, k_angleP, k_angleY; //kalman angles
float kU_angleR = 4, kU_angleP = 4, kU_angleY = 4; //kalman uncertainties, assumed to be 4 in the start
float kalman1D[2] = {0,0}; //array to store kalman angle and uncertainty for each roll, pitch and yaw


void setup() {
  // put your setup code here, to run once:
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  delay(2000);
  Serial.begin(9600);
<<<<<<< Updated upstream
  Serial.println("Flight software initialisation");
=======

  Wire.setClock(400000);
  Wire.begin();

  Serial.println("flight software test");
>>>>>>> Stashed changes

  // if(!SD.begin(BUILTIN_SDCARD)){
  //   Serial.println("SD card initialisation failed!");
  // }

  if(!bmp.begin(BMP280_ADDRESS)){
    LoRa.write("Cannot find BMP280 sensor - check wiring!");
    while(1);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);
 
  /*Servo setup*/
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
<<<<<<< Updated upstream
  
=======

  if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed");
            delay(1000);
        }
    }
>>>>>>> Stashed changes
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t prev_ms = 0;
  int sd_t, bmp_t, servo_t, mpu_t;
  //sd test

  if(millis() < sd_t){
  Serial.println("SD Card Test start");
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if(dataFile){
    dataFile.println("SD card works!");
    dataFile.close();
    Serial.println("Data written to SD Card");
  }
  else{
    Serial.println("Error opening SD card");
  }
  delay(2000);
  }
  prev_ms = millis();

  //bmp test
  if(millis() < bmp_t && millis() > sd_t){
  Serial.println("BMP280 Test start");
  Serial.println("Temperature = ");
  Serial.println(bmp.readTemperature());
  Serial.println("*C");

  float temp = bmp.readTemperature();
  String temp_send = "Temp - " + String(temp);
  
  int temp_len = temp_send.length(); temp_len++;
  unit8_t total[temp_len];
  temp_send.toCharArray(total, temp_len);
  
  LoRa.beginPacket();
  LoRa.print(temp_send);
  LoRa.endPacket(true);

  Serial.println("Pressure = ");
  Serial.println(bmp.readPressure()/100);
  Serial.println("hPa");

  Serial.println("Approx altitude = ");
  Serial.println(bmp.readAltitude(1019.66));
  Serial.println(" m");

  Serial.println();
  delay(50);

  }
  //servo test

  if(millis() < servo_t && millis() > sd_t){
  Serial.println("Servo Test start");
  testServos();
  delay(2000);
<<<<<<< Updated upstream
  
  Serial.println("MPU 9250 Test starts");
=======

  }
>>>>>>> Stashed changes
  
  //mpu test
  if(millis() < mpu_t && millis() > servo_t){

  if (mpu.update()) {
      prev_ms = millis();
      if (millis() > prev_ms + 25) {
        signals();

        kalman(k_angleR, kU_angleR, rollR, angleR); //to get smoother and more accurate roll. same below for pitch and yaw
        k_angleR = kalman1D[0];
        kU_angleR = kalman1D[1];

        kalman(k_angleP, kU_angleP, pitchR, angleP);
        k_angleP = kalman1D[0];
        kU_angleP = kalman1D[1];

        kalman(k_angleY, kU_angleY, yawR, angleY);
        k_angleY = kalman1D[0];
        kU_angleY = kalman1D[1];

        Serial.print("roll, pitch, yaw: ");
        Serial.print(k_angleR, 4);
        Serial.print(", ");
        Serial.print(k_angleP, 4);
        Serial.print(", ");
        Serial.println(k_angleY, 4);

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
  }
  delay(5000);

  while (LoRa.beginPacket() == 0) {
    Serial.print("waiting for radio ... ");
    delay(100);
  }

  Serial.print("Sending packet non-blocking: ");
  Serial.println(counter);

  // send in async / non-blocking mode
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket(true); // true = async / non-blocking mode

  counter++;

  testServos();

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

<<<<<<< Updated upstream
char* charArray(float data) {
  String data_send = "Temp - " + String(data);
  
  int data_len = data_send.length() + 1;
  uint8_t total[data_len];
  data_send.toCharArray((char*)total, data_len);
  return (char*)total;
}
=======
void signals(){
  rollR = mpu.getGyroX();
  pitchR = mpu.getGyroY();
  yawR = mpu.getGyroZ();

  accX = mpu.getAccX();
  accY = mpu.getAccY();
  accZ = mpu.getAccZ();

  angleR = 180*atan(accY/sqrt(accX*accX+accZ*accZ))/(3.14159);
  angleP = -180*atan(accX/sqrt(accY*accY+accZ*accZ))/(3.14159);
  angleY = 180*atan(accZ/sqrt(accX*accX+accZ*accZ))/(3.14159);
}

void kalman(float kState, float kU, float kInput, float kM){
  kState = kState + 0.004*kInput;
  kU = kU + 0.004*0.004*4*4;
  float kGain = kU/(kU + 3*3);
  kState = kState + kGain*(kM - kState);
  kU = (1 - kGain)*kU;

  kalman1D[0] = kState;
  kalman1D[1] = kU;
}
>>>>>>> Stashed changes
