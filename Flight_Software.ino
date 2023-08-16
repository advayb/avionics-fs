#include <MPU9250.h>
#include <quaternionFilters.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <Servo.h>
#include <SD.h>
#include <LoRa.h>
#include <RH_RF95.h>

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

void setup() {
  // put your setup code here, to run once:
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  delay(2000);
  Serial.begin(9600);
  Serial.println("Flight software initialisation");

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
  testServos();

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
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
  delay(2000);

  Serial.println("Servo Test start");
  testServos();
  delay(2000);

  Serial.println("Testing pyro channels");
  Serial.println("Pyro 1 high");
  digitalWrite(pyro1, HIGH);
  delay(2000);
  Serial.println("Pyro 1 low");
  digitalWrite(pyro1, LOW);
  delay(2000);
  Serial.println("Pyro 2 high");
  digitalWrite(pyro2, HIGH);
  delay(2000);
  Serial.println("Pyro 2 low");
  digitalWrite(pyro2, LOW);
  
  Serial.println("MPU 9250 Test starts");
  

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

char* charArray(float data) {
  String data_send = "Temp - " + String(data);
  
  int data_len = data_send.length() + 1;
  uint8_t total[data_len];
  data_send.toCharArray((char*)total, data_len);
  return (char*)total;
}
