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
#define LED_CALIBRATION_R 4
#define LED_CALIBRATION_B 5
#define LED_OK 3
#define POT_PIN A3

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float acc_x, acc_y, acc_z, acc_avg;
float gyro_x, gyro_y, gyro_z, gyro_avg;

unsigned int alarmDuration = 2000; // milliseconds
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

volatile float accThresholdMult; //1.0012;
volatile float gyroThresholdMult;// 1.5; 

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
  digitalWrite(LED_CALIBRATION_R, HIGH);
  delay(5000); // wait for 5 seconds to settle
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
    digitalWrite(LED_CALIBRATION_R, HIGH);
    return 1000;
  }
  digitalWrite(LED_CALIBRATION_R, LOW);
  // remap pot value to 1.0 - 1.015
  accThresholdMult = map(analogRead(POT_PIN), 5, 1018, 10010, 10150) / 10000.0;
  accThreshold = accListSum / sensorListSize * accThresholdMult;
  return accThreshold;
}

float getGyroThreshold() {
  // if len of list less than ListSize then return a big number to avoid false alarm
  if (gyroListIndex < sensorListSize-1 && gyroThreshold == 1000) {
    return 1000;
  }
  // remap pot value to 1.4 - 2.0
  gyroThresholdMult = map(analogRead(POT_PIN), 5, 1018, 15000, 35000) / 10000.0;
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
int getPotRemapped() {
  return map(analogRead(POT_PIN), 6, 1017, 1, 1000);
}
void startAlarm() {
  unsigned long tone1StartTime = 0;
  unsigned long tone2StartTime = 0;
  int potValueStart = getPotRemapped();
  int potValue = potValueStart;
  unsigned long potValueStartTime = 0;
  // get alarm start time 
  alarmStartTime = millis();
  alarmActive = true;

  while (millis() - alarmStartTime < alarmDuration) 
  {


    // CHECK current pot value every sec without blocking
    // if (millis() - potValueStartTime > 500) {
    //   potValueStartTime = millis();
    //   potValue = getPotRemapped();
    //   Serial.print("potValue: ");
    //   Serial.print(potValue);
    //   Serial.print(" -- potValueStart: ");
    //   Serial.println(potValueStart);
    //   if (abs(potValue - potValueStart) > 50 && potValue != 0) {
    //     // pot value changed, so end alarm
    //     alarmActive = false;
    //     break;
    //   }
    // }

    // keep updating the values to get rid of the false alarm when the alarm is ended
    readSensor();
    // updateAccList(acc_avg);
    // updateGyroList(gyro_avg);
    updateST_AccList(acc_avg);
    updateST_GyroList(gyro_avg);
    accThreshold = getAccThreshold();
    gyroThreshold = getGyroThreshold();

  // blocking tone code
    tone(BUZZER, 1000);
    delay(alarmDelay);
    tone(BUZZER, 2000);
    delay(alarmDelay);
    noTone(BUZZER);
    if (abs(getPotRemapped() - potValueStart) > 50 && getPotRemapped() != 0) {
      // pot value changed, so end alarm
      alarmActive = false;
      break;
    }

// non blocking tone code
    // if (tone1StartTime == 0) {
    //   tone1StartTime = millis();
    //   tone(BUZZER, 1000);
    // }
    // else if (millis() - tone1StartTime > alarmDelay) {

    //   if (tone2StartTime == 0) {
    //     tone2StartTime = millis();
    //     tone(BUZZER, 2000);
    //   }
    //   else if (millis() - tone2StartTime > alarmDelay) { 
    //     tone1StartTime = 0;
    //     tone2StartTime = 0;
    //   }
    // }
  }
  alarmActive = false;
  noTone(BUZZER);
  return;
}

void testLeds()
{
  digitalWrite(LED_CALIBRATION_R, HIGH);
  delay(250);
  digitalWrite(LED_CALIBRATION_R, LOW);
  digitalWrite(LED_CALIBRATION_B, HIGH);
  delay(250);
  digitalWrite(LED_CALIBRATION_B, LOW);
  digitalWrite(LED_OK, HIGH);
  delay(250);
  digitalWrite(LED_OK, LOW);
}

void setup() {
  // Serial.begin(19200);
  Wire.begin();
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_CALIBRATION_R, OUTPUT);
  pinMode(LED_CALIBRATION_B, OUTPUT);
  pinMode(LED_OK, OUTPUT);
  pinMode(POT_PIN, INPUT);

  testLeds();

  mpu.initialize();
  mpu.setSleepEnabled(false);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  calibrateMPU();

  testBuzzer();
  digitalWrite(LED_CALIBRATION_R, LOW);
 

  mpu.setDHPFMode(MPU6050_DHPF_5);
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);

  // Filling up the threshold lists
  for (int i = 0; i < sensorListSize; i++) {
    readSensor();
    updateAccList(acc_avg);
    updateGyroList(gyro_avg);
    accThreshold = getAccThreshold();
    gyroThreshold = getGyroThreshold();
    delay(100);
  }

}

void loop() {

  accThreshold = getAccThreshold();
  gyroThreshold = getGyroThreshold();

  // Filling up Smoothed Threshold lists
  readSensor();
  updateST_AccList(acc_avg);
  updateST_GyroList(gyro_avg);
  ST_AccAvg = getST_AccAvg();
  ST_GyroAvg = getST_GyroAvg();
  
  if (ST_AccAvg > accThreshold || ST_GyroAvg > gyroThreshold)
  {
    // if (ST_AccAvg > accThreshold) Serial.print("ALARM ACC");
    // if (ST_GyroAvg > gyroThreshold) Serial.print("ALARM GYRO");
    if (alarmStartTime == 0 || alarmActive == false) {
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


  delay(50);
  
}