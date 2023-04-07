// This is a robust and high accurate earhquake detection system using MPU6050
// accelerometer and gyroscope.    
// I have used a buzzer to alert the user. The main idea is to wake up the user when
// the earthquake is detected. The system using some custom smoothing algorithm to
// prevent false alarms like when the user is walking or car, bus, train is moving.
// Aspecially the gyroscope is very sensitive to the movement of the car, bus, train.
// You are welcome to collaborate and improve the code.

// TODO - We can dynamically change the threshold by updating the threshold lists for each n seconds or minutes.

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define BUZZER 12
#define LED_B 5
#define LED_R 3
#define LED_G 4
#define POT_PIN A3

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float acc_x, acc_y, acc_z, acc_avg;
float gyro_x, gyro_y, gyro_z, gyro_avg;

unsigned long LTA_uptdateTime = millis();
float LTA_updateInterval = 5000;

unsigned int alarmDuration = 10000;
unsigned int alarmDelay = 150;

// Buffer for sensor data
const int sensorListSize = 70;
const int ST_ListSize = 7;

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
  digitalWrite(LED_B, HIGH);
  delay(5000); // wait for 5 seconds to settle
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
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
    digitalWrite(LED_B, HIGH);
    return 1000;
  }
  digitalWrite(LED_B, LOW);
  // remap pot value to magic numbers that works for me - TODO - find a better way
  accThresholdMult = map(analogRead(POT_PIN), 5, 1018, 10020, 10080) / 10000.0;
  accThreshold = accListSum / sensorListSize * accThresholdMult;
  return accThreshold;
}

float getGyroThreshold() {
  // if len of list less than ListSize then return a big number to avoid false alarm
  if (gyroListIndex < sensorListSize-1 && gyroThreshold == 1000) {
    return 1000;
  }
  // remap pot value to magic numbers that works for me - TODO - find a better way
  gyroThresholdMult = map(analogRead(POT_PIN), 5, 1018, 25000, 35000) / 10000.0;
  gyroThreshold = gyroListSum / sensorListSize * gyroThresholdMult;
  return gyroThreshold;
}

float ACC_DIV = 16384.0;
float GYRO_DIV = 131.0;
void readSensor()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  acc_x = ax / ACC_DIV;
  acc_y = ay / ACC_DIV;
  acc_z = az / ACC_DIV;

  acc_avg = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
  
  gyro_x = gx / GYRO_DIV;
  gyro_y = gy / GYRO_DIV;
  gyro_z = gz / GYRO_DIV;

  gyro_avg = sqrt(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);
}
int getPotRemapped() {
  return map(analogRead(POT_PIN), 6, 1017, 1, 1000);
}
void startAlarm() {
  int potValueStart = getPotRemapped();
  // get alarm start time 
  alarmStartTime = millis();
  alarmActive = true;
  // Reset the AVGs
  ST_AccAvg = 0;
  ST_GyroAvg = 0;
  ST_gyroListIndex = 0;
  ST_accListIndex = 0;
  ST_AccListSum = 0;
  ST_GyroListSum = 0;
  memset(ST_AccList, 0, sizeof(ST_AccList));
  memset(ST_GyroList, 0, sizeof(ST_GyroList));


  while (millis() - alarmStartTime < alarmDuration) 
  {

    // keep updating the values to get rid of the false alarm when the alarm is ended
    // readSensor();
    // updateST_AccList(acc_avg);
    // updateST_GyroList(gyro_avg);
    // accThreshold = getAccThreshold();
    // gyroThreshold = getGyroThreshold();

    if (abs(getPotRemapped() - potValueStart) > 50 && getPotRemapped() != 0) {
      // pot value changed, so end alarm
      alarmActive = false;
      break;
    }
  // blocking tone code
    tone(BUZZER, 1000);
    delay(alarmDelay);
    tone(BUZZER, 2000);
    delay(alarmDelay);
    noTone(BUZZER);

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
  digitalWrite(LED_B, HIGH);
  delay(250);
  digitalWrite(LED_B, LOW);
  digitalWrite(LED_R, HIGH);
  delay(250);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, HIGH);
  delay(250);
  digitalWrite(LED_G, LOW);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(POT_PIN, INPUT);

  testLeds();

  mpu.initialize();
  // mpu.dmpInitialize();

  if (mpu.testConnection()) {
      // mpu.setDMPEnabled(true);
      digitalWrite(LED_G, HIGH);
    } else {
      digitalWrite(LED_B, HIGH);
      while (1);
    }

  calibrateMPU();
  mpu.setRate(9); // 1khz / (1 + 9) = 100 Hz
  mpu.setSleepEnabled(false);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);



  testBuzzer();
  digitalWrite(LED_B, LOW);
 

  mpu.setDHPFMode(MPU6050_DHPF_5);
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);

  // Filling up the threshold lists
  for (int i = 0; i < sensorListSize; i++) {
    readSensor();
    updateAccList(acc_avg);
    updateGyroList(gyro_avg);
    accThreshold = getAccThreshold();
    gyroThreshold = getGyroThreshold();
    delay(70);
  }

}


void loop() {
  // digitalWrite(LED_G, HIGH);
  
  accThreshold = getAccThreshold();
  gyroThreshold = getGyroThreshold();

  // Filling up Smoothed Threshold lists
  readSensor();
  updateST_AccList(acc_avg);
  updateST_GyroList(gyro_avg);
  ST_AccAvg = getST_AccAvg();
  ST_GyroAvg = getST_GyroAvg();

  if(millis() - LTA_uptdateTime > LTA_updateInterval) {
    LTA_uptdateTime = millis();
    updateAccList(acc_avg);
    updateGyroList(gyro_avg);
    accThreshold = getAccThreshold();
    gyroThreshold = getGyroThreshold();
  }
  
  if (ST_AccAvg > accThreshold || ST_GyroAvg > gyroThreshold)
  {
    if (alarmStartTime == 0 || alarmActive == false) {

      startAlarm();
    }
  }

  // Serial.print("acc_avg:");
  // Serial.print(ST_AccAvg,6);
  // Serial.print(",");
  // Serial.print("accThreshold:");
  // Serial.print(accThreshold,6);
  // Serial.print(",");
  // Serial.print("gyro_avg:");
  // Serial.print(ST_GyroAvg,6);
  // Serial.print(",");
  // Serial.print("gyroThreshold:");
  // Serial.println(gyroThreshold,6);
  delay(20);


}