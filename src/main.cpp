/////////////////////////////////////////////////////////////////////////////////////////
//                                                                                     //
// This is a robust and high accurate earhquake detection system using MPU6050
// accelerometer and gyroscope.    
// I have used a buzzer to alert the user. The main idea is to wake up the user when
// the earthquake is detected. The system using some custom smoothing algorithm to
// prevent false alarms like when the user is walking or car, bus, train is moving.
// Aspecially the gyroscope is very sensitive to the movement of the car, bus, train.
// You are welcome to collaborate and improve the code.
// Berkay Eraydin - 2023

// TODO - Use weighted average for smoothing algorithm. Some axis are more sensitive as I have seen.

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define BUZZER 12
#define LED_CALIBRATION 4

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float acc_x, acc_y, acc_z, acc_avg;
float gyro_x, gyro_y, gyro_z, gyro_avg;

unsigned int alarmDuration = 200; // milliseconds
unsigned int alarmDelay = 50; // milliseconds

// Buffer for sensor data
const int sensorListSize = 80;
const int ST_ListSize = 6;

float accList[sensorListSize];
float gyroList[sensorListSize];

float accListSum = 0;
float gyroListSum = 0;

int accListIndex = 0;
int gyroListIndex = 0;

float accThresholdMult = 1.0012;
float gyroThresholdMult = 1.5; 

float gyroThreshold = 1000;
float accThreshold = 1000;

float ST_AccList[ST_ListSize];
float ST_GyroList[ST_ListSize];

float ST_AccListSum = 0;
float ST_GyroListSum = 0;

float ST_AccAvg = 0;
float ST_GyroAvg = 0;

int ST_accListIndex = 0;
int ST_gyroListIndex = 0;

long alarmStartTime = 0;
bool alarmActive = false;

void calibrateMPU() {
  // Calibrate gyro and accelerometers, load biases in bias registers
  digitalWrite(LED_CALIBRATION, HIGH);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  // mpu.PrintActiveOffsets();
}

void testBuzzer() {
  tone(BUZZER, 2000);
  delay(100);
  tone(BUZZER, 1000);
  delay(100);
  noTone(BUZZER);
}


void updateST_AccList(float value) {
  ST_AccListSum -= ST_AccList[ST_accListIndex];
  ST_AccList[ST_accListIndex] = value;
  ST_AccListSum += ST_AccList[ST_accListIndex];
  ST_accListIndex = (ST_accListIndex + 1) % ST_ListSize;
}

void updateST_GyroList(float value) {
  ST_GyroListSum -= ST_GyroList[ST_gyroListIndex];
  ST_GyroList[ST_gyroListIndex] = value;
  ST_GyroListSum += ST_GyroList[ST_gyroListIndex];
  ST_gyroListIndex = (ST_gyroListIndex + 1) % ST_ListSize;
}


void updateAccList(float value) {
  accListSum -= accList[accListIndex];
  accList[accListIndex] = value;
  accListSum += accList[accListIndex];
  accListIndex = (accListIndex + 1) % sensorListSize;
}

void updateGyroList(float value) {
  gyroListSum -= gyroList[gyroListIndex];
  gyroList[gyroListIndex] = value;
  gyroListSum += gyroList[gyroListIndex];
  gyroListIndex = (gyroListIndex + 1) % sensorListSize;
}

float getST_AccAvg() {
  // if len of list less than ListSize then return a big number to avoid false alarm
  if (ST_accListIndex < ST_ListSize-1 && ST_AccAvg == 0) {
    return 0;
  }
  return ST_AccListSum / ST_ListSize;
}

float getST_GyroAvg() {
  // if len of list less than ListSize then return a big number to avoid false alarm
  if (ST_gyroListIndex < ST_ListSize-1 && ST_GyroAvg == 0) {
    return 0;
  }
  return ST_GyroListSum / ST_ListSize;
}


float getAccThreshold() {
  // if len of list less than ListSize then return a big number to avoid false alarm
  if (accListIndex < sensorListSize-1 && accThreshold == 1000) {
    // waiting for list to fill up, calibrating. So led on
    digitalWrite(LED_CALIBRATION, HIGH);
    return 1000;
  }
  digitalWrite(LED_CALIBRATION, LOW);
  accThreshold = accListSum / sensorListSize * accThresholdMult;
  return accThreshold;
}

float getGyroThreshold() {
  // if len of list less than ListSize then return a big number to avoid false alarm
  if (gyroListIndex < sensorListSize-1 && gyroThreshold == 1000) {
    return 1000;
  }
  gyroThreshold = gyroListSum / sensorListSize * gyroThresholdMult;
  return gyroThreshold;
}

float ACC_DIV = 10.0;
float GYRO_DIV = 10.0;
void readSensor()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  acc_x = ax / ACC_DIV;
  acc_y = ay / ACC_DIV;
  acc_z = az / ACC_DIV;
  acc_avg = (abs(acc_x) + abs(acc_y) + abs(acc_z)) / 3; // TODO find better way instead of using avg
  
  gyro_x = gx / GYRO_DIV;
  gyro_y = gy / GYRO_DIV;
  gyro_z = gz / GYRO_DIV;
  gyro_avg = (abs(gyro_x) + abs(gyro_y) + abs(gyro_z)) / 3; // TODO find better way instead of using avg
}

void startAlarm() {
  unsigned long tone1StartTime = 0;
  unsigned long tone2StartTime = 0;
  // get alarm start time 
  alarmStartTime = millis();  
  while (millis() - alarmStartTime < alarmDuration) 
  {
    // keep updating the values to get rid of the false alarm when the alarm is ended
    readSensor();
    // updateAccList(acc_avg);
    // updateGyroList(gyro_avg);
    updateST_AccList(acc_avg);
    updateST_GyroList(gyro_avg);

    if (tone1StartTime == 0) {
      tone1StartTime = millis();
      tone(BUZZER, 1000);
    }
    else if (millis() - tone1StartTime > alarmDelay) {

      if (tone2StartTime == 0) {
        tone2StartTime = millis();
        tone(BUZZER, 2000);
      }
      else if (millis() - tone2StartTime > alarmDelay) { 
        tone1StartTime = 0;
        tone2StartTime = 0;
      }
    }
  }
  noTone(BUZZER);
  return;
}

void setup() {
  // Serial.begin(19200);
  Wire.begin();
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_CALIBRATION, OUTPUT);
  mpu.initialize();
  mpu.setSleepEnabled(false);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  calibrateMPU();

  testBuzzer();
  digitalWrite(LED_CALIBRATION, LOW);
 

  mpu.setDHPFMode(MPU6050_DHPF_5);
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);

  // Filling up the threshold lists
  for (int i = 0; i < sensorListSize; i++) {
    delay(100);
    readSensor();
    updateAccList(acc_avg);
    updateGyroList(gyro_avg);
    accThreshold = getAccThreshold();
    gyroThreshold = getGyroThreshold();
  }
}

void loop() {
  // Filling up Smoothed Threshold lists
  readSensor();
  updateST_AccList(acc_avg);
  updateST_GyroList(gyro_avg);
  ST_AccAvg = getST_AccAvg();
  ST_GyroAvg = getST_GyroAvg();
  
  if (ST_AccAvg > accThreshold || ST_GyroAvg > gyroThreshold)
  // if (ST_AccAvg > accThreshold)
  // if (ST_GyroAvg > gyroThreshold)
  {
    if (alarmStartTime == 0 || millis() - alarmStartTime > alarmDuration) {
      // Serial.println("ALARM STARTED");
      startAlarm();
    }
  }

  // Serial.print(" - Acc: ");
  // Serial.print(acc_avg, 4);
  // Serial.print(" - ST: ");
  // Serial.print(ST_AccAvg, 4);
  // Serial.print(" - Threshold: ");
  // Serial.println(accThreshold, 4);
  // Serial.print(" ---- ");
  // Serial.print(" - Gyro: ");
  // Serial.print(gyro_avg , 4);
  // Serial.print(" - ST: ");
  // Serial.print(ST_GyroAvg , 4);
  // Serial.print(" - Threshold: ");
  // Serial.println(gyroThreshold , 4);


  delay(100);
  
}