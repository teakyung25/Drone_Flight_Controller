#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
void setup_mpu6050(void);

Adafruit_BMP280 bmp; // I2C
 
void setup() {
  Wire.begin();
  Serial.begin(115200);
    setup_mpu6050();

    bool status = status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID); //56,57,58

    if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
    }
}
 
void setup_mpu6050(void) {
  // 400 KHz from MPU6050 specifications 
  Wire.begin(21,22);
  Wire.setClock(400000);
  delay(250); //Give mpu6050 time to start
  //Power Mode Activation
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Serial.println("SETUP---Complete---");

}

void loop() {
  uint8_t error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);          
}