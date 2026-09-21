#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// MOTORS PINS
int M1pin = 5;
int M2pin = 6;
int M3pin = 9;
int M4pin = 3;
int M1;
int M2;
int M3;
int M4;

// VOLTMETER
int Vpin = A3;

struct Volts {
  float Vin;
  float Vout;
  float Vp;
};

Volts Vdata;

float readBattery();

// setting up MPU
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

int RateCalibrationNum;
float RateCalibrationRoll = 0;
float RateCalibrationYaw = 0;
float RateCalibrationPitch = 0;
float RateRoll;
float RatePitch;
float RateYaw;
float DesiredRoll = 0;
float DesiredPitch = 0;
float DesiredYaw = 0;
float Throttle = 0;
float AngleRoll;
float AnglePitch;
float ErrorRoll;
float ErrorPitch;
float ErrorYaw;
float AccAngleRoll;
float AccAnglePitch;
float AccX;
float AccY;
float AccZ;
float Kp_Roll = 0.8;
float Kp_Pitch = 0.8;
float Kp_Yaw = 0.6;
float Kd_Roll = 0.25;
float Kd_Pitch = 0.25;
float Kd_Yaw = 0.15;
float RollCorrection;
float PitchCorrection;
float YawCorrection;
float PreviousErrorRoll = 0;
float PreviousErrorPitch = 0;
float PreviousErrorYaw = 0;
float D_Roll;
float D_Pitch;
float D_Yaw;

// data to receive
struct ControllerData {
  float ThXval;
  float ThYval;
  float RoXval;
  float RoYval;
};

ControllerData Cdata;

// TIME STEP
float dt = 0.004;
unsigned long previousTime = 0;

// FAILSAFE
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 500;  
bool failsafe = false;

// setting up RADIO
int CE = 8;
int CSN = 7;

const byte addresses[][6] = {"00001", "00002"}; //00001 to receive Cdata and 00002 to transmit Vdata

RF24 radio(CE,CSN);

void setup() {
  //MOTOR PINS
  Serial.begin(9600);
  pinMode(M1pin,OUTPUT);
  pinMode(M2pin,OUTPUT);
  pinMode(M3pin,OUTPUT);
  pinMode(M4pin,OUTPUT); 

  //VOLTMETER 
  pinMode(Vpin,INPUT);

  //RADIO
  radio.begin();
  radio.openWritingPipe(addresses[1]); //transmitter
  radio.openReadingPipe(1,addresses[0]); //receiver
  radio.setPALevel(RF24_PA_MAX);
  
  //MPU
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //MPU CALIBRATION
  for (RateCalibrationNum=0; RateCalibrationNum<3000; RateCalibrationNum++) {
    mpu.getEvent(&a, &g, &temp);
    RateCalibrationRoll += (g.gyro.x*180.)/M_PI;
    RateCalibrationPitch += (g.gyro.y*180.)/M_PI;
    RateCalibrationYaw += (g.gyro.z*180.)/M_PI;
    delay(1);

  }
  RateCalibrationRoll /= 3000.;
  RateCalibrationPitch /= 3000.;
  RateCalibrationYaw /= 3000.;

}

void loop() {
  delay(1);
  
// VOLTMETER
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(Vpin);
    delayMicroseconds(200);
  }
  Vdata.Vin = (sum / 10.0) * (5.0 / 1023.0);
  Vdata.Vp = (1000.0 / 9.0) * Vdata.Vin - (1100.0 / 3.0);

// RADIO
radio.stopListening();
radio.write(&Vdata, sizeof(Vdata));

radio.startListening();

bool gotPacket = false;
unsigned long started = millis();

  while (millis() - started < 5) {
    if (radio.available()) {
      radio.read(&Cdata, sizeof(Cdata));
      gotPacket = true;
      lastPacketTime = millis();          //update last successful receive
      break;
    }
  }

  //failsafe check
  if (millis() - lastPacketTime > FAILSAFE_TIMEOUT) {
    failsafe = true;
  }   
  else {
    failsafe = false;
  }

// MAPPING VALUES
  if (failsafe) {
    Throttle     = 0;
    DesiredRoll  = 0;
    DesiredPitch = 0;
    DesiredYaw   = 0;
  } 
  else {
  Throttle = map(Cdata.ThXval, 0, 1023, 0, 255);
  DesiredRoll = map(Cdata.RoYval, 0, 1023, -30, 30);
  DesiredPitch = map(Cdata.RoXval, 0, 1023, -30, 30);
  DesiredYaw = map(Cdata.ThYval, 0, 1023, -30, 30);
  }

// MPU
  mpu.getEvent(&a, &g, &temp);
  
  RateRoll = (g.gyro.x * 180.0)/M_PI - RateCalibrationRoll;
  RatePitch = (g.gyro.y * 180.0)/M_PI - RateCalibrationPitch;
  RateYaw = (g.gyro.z * 180.0)/M_PI - RateCalibrationYaw;

  AccX = a.acceleration.x;
  AccY = a.acceleration.y;
  AccZ = a.acceleration.z;

  //calculate dt (time step in seconds)
  //needed because gyro gives me angular velocity and I need just the angle
  unsigned long currentTime = micros();
  dt = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;
  if (dt > 0.1) dt = 0.004;
  
  AccAngleRoll = atan2(AccY, AccZ) * 180.0 / M_PI;
  AccAnglePitch = atan2(-AccX, sqrt(AccY*AccY + AccZ*AccZ))*180.0/M_PI;

  //complementary filter 
  //gyro drifts with time so accelerometer is better in long term
  // Angle = # * (Angle + GyroRate * dt) + (1-#)*AccelAngle
  AngleRoll = 0.98*(AngleRoll + RateRoll *dt) + 0.02*AccAngleRoll; //98% gyro and 2% acc because I won't fly long
  AnglePitch = 0.98*(AnglePitch + RatePitch * dt) + 0.02*AccAnglePitch;

  //errors 
  ErrorRoll = DesiredRoll - AngleRoll;
  ErrorPitch = DesiredPitch - AnglePitch;
  ErrorYaw = DesiredYaw - RateYaw;

  //PD controller
  D_Roll = (ErrorRoll - PreviousErrorRoll) / dt;
  D_Pitch = (ErrorPitch - PreviousErrorPitch) / dt;
  D_Yaw = (ErrorYaw - PreviousErrorYaw) / dt;
  
  RollCorrection = (ErrorRoll * Kp_Roll) + (D_Roll * Kd_Roll);
  PitchCorrection = (ErrorPitch * Kp_Pitch) + (D_Pitch * Kd_Pitch);
  YawCorrection = (ErrorYaw * Kp_Yaw) + (D_Yaw * Kd_Yaw);

  //saving errors for the next loop
  PreviousErrorRoll = ErrorRoll;
  PreviousErrorPitch = ErrorPitch;
  PreviousErrorYaw = ErrorYaw;

  M1 = Throttle - PitchCorrection + RollCorrection - YawCorrection;
  M2 = Throttle - PitchCorrection - RollCorrection + YawCorrection;
  M3 = Throttle + PitchCorrection - RollCorrection - YawCorrection;
  M4 = Throttle + PitchCorrection + RollCorrection + YawCorrection;

  M1 = constrain(M1, 0, 255); //limits the M values to 0-255 as this are the values that the motor driver reads
  M2 = constrain(M2, 0, 255);
  M3 = constrain(M3, 0, 255);
  M4 = constrain(M4, 0, 255);

  if (Throttle < 15) {  
    M1 = 0;
    M2 = 0;
    M3 = 0;
    M4 = 0;
}

  //sending to the motor 
  analogWrite(M1pin, M1);
  analogWrite(M2pin, M2);
  analogWrite(M3pin, M3);
  analogWrite(M4pin, M4);
}
