#include <Arduino.h>
#include <U8g2lib.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#define RXD2 4
#define TXD2 5
#define SDA_PIN 6
#define SCL_PIN 7
#define SPO_BUFFER 100
#define BUTTON_POW 9
#define BUTTON_PLUSS 10
#define BUTTON_MINUS 11
#define BUTTON_SCREEN 8

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL_PIN, /* data=*/ SDA_PIN);  // High speed I2C

byte get_data[] = { 0xFD, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte calib[] = { 0xFE, 0x7A, 0x47, 0x46, 0x00, 0x00 };
int hrData[] = { 255, 255, 255 };

unsigned long startMillis; 
unsigned long currentMillis;
unsigned long activateMillis;

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

// Functions
void readHrSensor();
bool calibrateSensor();
void updateScreen();
void checkButtons();
void checkHealth();
void readSpSensor(int32_t n);
void driveMotor(int direction);


void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);

  // Screen setup
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.clearBuffer();

  Serial.begin(115200);
  senSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // PWM setup for motor
  ledcAttach(10, 20000, 11);
  ledcWrite(0, 2000);

  // Sensor calibration
  /* if(calibrateSensor()) {
    u8g2.drawStr(0, 40, "Calibrate Success");
  } else {
    u8g2.drawStr(0, 40, "Calibrate Failed");
  }
  u8g2.sendBuffer();
  delay(3000);
  u8g2.setPowerSave(1); */

  // Setup counting
  startMillis = millis();
  activateMillis = millis(); // Remove from here!!!!!*/


  // Setup SpO2 sensor
  particleSensor.begin(Wire, I2C_SPEED_FAST); //Use default I2C port, 400kHz speed

  byte ledBrightness = 100; //Options: 0=Off to 255=50mA
  byte sampleAverage = 2; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 215; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
  
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  readSpSensor(bufferLength);
  
}


void loop() {
  //u8g2.clearBuffer(); // clear the internal memory

  currentMillis = millis();

  /* if (currentMillis - startMillis >= 1000) {
    readHrSensor();
    String mes1 = "Heart rate: " + (String)hrData[2];
    String mes2 = "Blood p H/L: " + (String)hrData[0] + "/" + (String)hrData[1];
    Serial.println(mes1);
    Serial.println(mes2);
    Serial.println("-------------------------------");
    startMillis = currentMillis;  
  }
 */

  checkButtons();
  //Serial.println("noice");



  //u8g2.setPowerSave(0);
  //updateScreen();

  //readSpSensor(25);
  
  //u8g2.drawStr(0, 20, mes1.c_str());
  //u8g2.drawStr(0, 40, mes2.c_str());	// write something to the internal memory
  //u8g2.sendBuffer();


  // delay(1000);
  // u8g2.setPowerSave(1);
  // delay(2000);
  // u8g2.setPowerSave(0);
}


// Read sensor data
void readHrSensor() {
  senSerial.write(get_data, sizeof(get_data)); // Start data reading sequence
  while (!senSerial.available()); // Wait until sesor available

  while (senSerial.available()) { // Read bit values
    byte a = senSerial.read();
    if (int(a) == 253) {
      for (size_t i = 0; i < 3; i++) {
        hrData[i] = senSerial.read();
      }
    }
  }
}


// Calibrate sensor
bool calibrateSensor() {
  senSerial.write(calib, sizeof(calib)); // Start calibration sequence

  for (size_t i = 0; i < 50; i++) { // Try to calibrate for 50 cycles
    while (!senSerial.available()); // Wait until sesor available

    while (senSerial.available()) { // Read bit values
      byte a = senSerial.read();
      if (int(a) == 254) {
        byte b;
        for (size_t i = 0; i < 3; i++) {
          b = senSerial.read();
        }
        if (b == 0x00) { // Success
          return true;
        } else if (b == 0x02) { // Failed calibration
          return false;
        }
      }
    }
  }
  return false;
}


// Update screen
void updateScreen() {
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.clearBuffer();
  String minutes = (String)((millis() - activateMillis) / 1000 / 60) + " min"; // Calculate minutes passed

  u8g2.drawStr(0, 20, "Time since activated");
  u8g2.drawStr(30, 40, minutes.c_str());
  u8g2.sendBuffer();
}


// Check buttons
void checkButtons() {
  if (!digitalRead(BUTTON_POW)) {
    // Stop motors
    //rgbLedWrite(8, 255, 0, 0);
  } else {
    //rgbLedWrite(8, 0, 0, 255);
  }
  
  if (!digitalRead(BUTTON_SCREEN)) {
    u8g2.setPowerSave(0); // Wake up screen
    updateScreen();
  } else {
    u8g2.setPowerSave(1); // Screen to sleep mode
  }

  if (!digitalRead(BUTTON_PLUSS)) {
    // Motor +
  } else if (!digitalRead(BUTTON_MINUS)) {
    // Motor -
  }
  
}


// Check health
void checkHealth() {

}


// Read SpO2 sensor
void readSpSensor(int32_t n) {
  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = n; i < SPO_BUFFER; i++)
  {
    redBuffer[i - n] = redBuffer[i];
    irBuffer[i - n] = irBuffer[i];
  }

  //take 25 sets of samples before calculating the heart rate.
  for (byte i = SPO_BUFFER - n; i < SPO_BUFFER; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    //send samples and calculation result to terminal program through UART
    // Serial.print(F("red="));
    // Serial.print(redBuffer[i], DEC);
    // Serial.print(F(", ir="));
    // Serial.print(irBuffer[i], DEC);

    // Serial.print(F(", HR="));
    // Serial.print(heartRate, DEC);

    // Serial.print(F(", HRvalid="));
    // Serial.print(validHeartRate, DEC);

    // Serial.print(F(", SPO2="));
    // Serial.print(spo2, DEC);

    // Serial.print(F(", SPO2Valid="));
    // Serial.println(validSPO2, DEC);
  }

  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}


// Drive motor
void driveMotor(int direction) {
  if (direction == 1) {
    // Set direction positive
  } else {
    // Set direction negative
  }
  // Set pwm value
}
