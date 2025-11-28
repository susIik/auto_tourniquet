#ifndef MAIN_H_
#define MAIN_H_

#include <Arduino.h>
#include <U8g2lib.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "MyButton.hpp"

#define RXD2 4
#define TXD2 5
#define SDA_PIN 6
#define SCL_PIN 7
#define SPO_BUFFER 100
#define BUTTON_POW 9
#define BUTTON_PLUSS 10
#define BUTTON_MINUS 11
#define BUTTON_SCREEN 8
#define N_SLEEP 21
#define N_FAULT 20
#define IPROPI 3
#define DRIVE_PH 23
#define DRIVE_EN 22
#define SOLENOID 18
#define VIBRATE 19 

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL_PIN, /* data=*/ SDA_PIN);  // High speed I2C

byte get_data[] = { 0xFD, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte calib[] = { 0xFE, 0x7A, 0x47, 0x46, 0x00, 0x00 };
int hrData[] = { 255, 255, 255 }; // High, Low, HR

unsigned long startMillis; 
unsigned long currentMillis;
unsigned long activateMillis;
unsigned long motorMillis;

float k_t = 0.44 / 2.7; // Torque constant
int activation = 0; // 0 - off, 1 - tightening in progress, 2 - tightened

// particlesensor test stuff
MAX30105 particleSensor;

uint32_t irBuffer[SPO_BUFFER]; //infrared LED sensor data
uint32_t redBuffer[SPO_BUFFER];  //red LED sensor data
int32_t bufferLength = SPO_BUFFER; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

HardwareSerial senSerial(0);

MyButton powerButton(BUTTON_POW);
MyButton stopButton(BUTTON_SCREEN);
MyButton plussButton(BUTTON_PLUSS);
MyButton minusButton(BUTTON_MINUS);


// Functions
void readHrSensor();
bool calibrateHrSequence();
void calibrateHrSensor();
void showTime();
void checkButtons();
void checkHealth();
void readSpSensor(int32_t n);
void driveMotor(int direction);
void motorWakeUp();
void motorOff();
void tightenStrap();
float strapTorque();
void triggerSolenoid(int a);
void vibrate();

#endif