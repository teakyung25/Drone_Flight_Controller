#include <Arduino.h>
#include <Wire.h>
#include "esp32-rmt-pwm-reader.h"

void setup_mpu6050(void);
void gyro_signals(void);
void chanInterrupt(void);
void setup_receiver(void);
void loop_receiver(void);

float RollRate, PitchRate, YawRate;
float CalibrationRollRate, CalibrationPitchRate, CalibrationYawRate;
float CalibrationRateNumber;
int32_t impulseChannel_1;
int32_t impulseChannel_2;
int32_t impulseChannel_3;
int32_t impulseChannel_4;
int32_t impulseChannel_5;
int32_t impulseChannel_6;
float InputThrottle;



uint8_t receiver_pins[] = {36,39,34,35,32,33};  // desired input pins
uint8_t motor_pins[] = {13,9,10,11}; //FR, FL, RR, RL
uint8_t numberOfChannels = sizeof(receiver_pins) / sizeof(uint8_t);


// PID VALS
float kp_roll = 0.0;
float ki_roll = 0.0;
float kd_roll = 0.0;
float kp_pitch = 0.0;
float ki_pitch = 0.0;
float kd_pitch = 0.0;
float kp_yaw = 0.0;
float ki_yaw = 0.0;
float kd_yaw = 0.0;

// FC settings 
float cycle_target = 250.0;
float throttle_idle = 0.0; //setup value through test
float throttle_cap = 0.22;
float roll_rate_max = 30.0;
float pitch_rate_max = 30.0;
float yaw_rate_max = 50.0;

float cycle_s = 1/cycle_target;
int32_t cycle_us = (int32_t) cycle_s * 1000000, 0;
float throttle_range = throttle_cap - throttle_idle;
float limit_integral = 150.0;
bool prev_flight_state = False; // False (Stand by)

float prev_roll_integral = 0.0;
float prev_roll_error = 0.0;
float prev_pitch_integral = 0.0;
float prev_pitch_error = 0.0;
float prev_yaw_integral = 0.0;
float prev_yaw_error = 0.0;

// SETUP -------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println("SETUP---start---");

  // Blue light turn on
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  // Setup MPU6050
  setup_mpu6050();
  

  // Setup Receiver
  setup_receiver();
  analogWriteFrequency(250);
  analogWriteResolution(12);
  delay(250);

  // while(impulseChannel_2 < 1020 || impulseChannel_2 > 1050) {
  //   loop_receiver();
  //   delay(4);
  // }
}

// MAIN LOOP -------------------------------------------
void loop() {
  int32_t start_time = millis();
  Serial.println("Loop---Start---");
  gyro_signals();
  RollRate -= CalibrationRollRate;
  PitchRate -= CalibrationPitchRate;
  YawRate -= CalibrationYawRate;
  // Serial.print("Roll Rate: ");
  // Serial.print(RollRate);
  // Serial.print(" Pitch Rate: ");
  // Serial.print(PitchRate);
  // Serial.print(" Yaw Rate: ");
  // Serial.println(YawRate);
  loop_receiver(); 
  analogWrite(13,1.024*impulseChannel_4);    
  if (impulseChannel_5 == 1000) { // IDLE Mode
  
    
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

  for(CalibrationRateNumber = 0; CalibrationRateNumber<2000; CalibrationRateNumber++) {
    gyro_signals();
    CalibrationRollRate += RollRate;
    CalibrationPitchRate = PitchRate;
    CalibrationYawRate = YawRate;
  }
  CalibrationRollRate/=2000;
  CalibrationPitchRate/=2000;
  CalibrationYawRate/=2000;
}

void setup_receiver(void) {
// init channels
    pwm_reader_init(receiver_pins, numberOfChannels);

    // here you can change channel defaults values before reading (if needed)
    // e.g. pwm_set_channel_pulse_min() /max/neutral
    // e.g. set auto_zero/auto_min_max for channel 0-2
    for (int ch = 0; ch < 5; ch++) {
        pwm_set_auto_zero(ch, true);     // set channel to auto zero
        pwm_set_auto_min_max(ch, true);  // set channel to auto min/max calibration
    }

    // begin reading 
    esp_err_t err = pwm_reader_begin();
    if (err != ESP_OK) {
        Serial.printf("begin() err: %i", err);
    }
}

void loop_receiver(void) {
    // Reading the actual pulse width of channel 1 
    impulseChannel_1 = pwm_get_rawPwm(1); 
    impulseChannel_2 = pwm_get_rawPwm(2); 
    impulseChannel_3 = pwm_get_rawPwm(3); 
    impulseChannel_4 = pwm_get_rawPwm(4); 
    impulseChannel_5 = pwm_get_rawPwm(5); 
    impulseChannel_6 = pwm_get_rawPwm(6); 
    Serial.printf("Pitch (c1): %d Throttle (c2): %d Yaw (c3): %d C4: %d C5: %d C6: %d\n",impulseChannel_1,impulseChannel_2,impulseChannel_3,impulseChannel_4,impulseChannel_5,impulseChannel_6);
    // Do something with the pulse width...

    // The lib is not blocking. Therefore the read cycle can be 
    // quite high. In esp32_pwm_read.cpp I use 100ms. However, this 
    // should certainly not be necessary with a PWM frequency of 50Hz. 
    // delay() is just as an example. In fact you should use your own 
    // non-blocking task. e.g. like in esp32_pwm_read.cpp
    delay(200); 
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Serial.println("mpu6050---verified---");

  // Built Digit Low Pass Filter (DLPF)
  Wire.write(0x1A); //configures register
  Wire.write(0x05); // DLPF Value of 5 == 10Hz Bandwidth
  Wire.endTransmission();

  //Sensitivity scale factor 
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8); //500 deg/s or why is 1 == 0x8?
  Wire.endTransmission();
  //
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  int16_t GyroX = Wire.read()<<8 | Wire.read();
  int16_t GyroY = Wire.read()<<8 | Wire.read();
  int16_t GyroZ = Wire.read()<<8 | Wire.read();

  RollRate = (float) GyroX / 65.5;
  PitchRate = (float) GyroY / 65.5;
  YawRate = (float) GyroZ / 65.5;

}






