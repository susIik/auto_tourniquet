#include "MAX30105.h"
#include "MyButton.hpp"
#include "spo2_algorithm.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <U8g2lib.h>
#include <Wire.h>

static constexpr uint8_t RXD2 = 17;
static constexpr uint8_t TXD2 = 16;
static constexpr uint8_t SDA_PIN = 6;
static constexpr uint8_t SCL_PIN = 7;
static constexpr uint8_t BUTTON_POW = 9;
static constexpr uint8_t BUTTON_PLUSS = 10;
static constexpr uint8_t BUTTON_MINUS = 11;
static constexpr uint8_t BUTTON_SCREEN = 8;
static constexpr uint8_t N_SLEEP = 21;
static constexpr uint8_t N_FAULT = 20;
static constexpr uint8_t IPROPI = 3;
static constexpr uint8_t DRIVE_PH = 23;
static constexpr uint8_t DRIVE_EN = 22;
static constexpr uint8_t SOLENOID = 18;
static constexpr uint8_t VIBRATE = 19;
static constexpr uint8_t DRVOFF = 2;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL_PIN, /* data=*/SDA_PIN); // High speed I2C

byte dataRequestMsg[] = {0xFD, 0x00, 0x00, 0x00, 0x00, 0x00};
byte calibRequestMsg[] = {0xFE, 0x7A, 0x47, 0x46, 0x00, 0x00};
int hrData[] = {255, 255, 255}; // High, Low, HR

unsigned long startMillis;
unsigned long currentMillis;
unsigned long activateMillis;
unsigned long motorMillis;
unsigned long screenWakeMillis = 0;

float k_t = 0.44 / 2.7; // Torque constant
int activation = 0;     // 0 - off, 1 - tightening in progress, 2 - tightened
int adjusting = 0;

// particlesensor test stuff
MAX30105 particleSensor;

static constexpr uint32_t bufferLength = 100;
uint32_t irBuffer[bufferLength];     // infrared LED sensor data
uint32_t redBuffer[bufferLength];    // red LED sensor data
int32_t spo2;                      // SPO2 value
int8_t validSPO2;                  // indicator to show if the SPO2 calculation is valid
int32_t heartRate;                 // heart rate value
int8_t validHeartRate;             // indicator to show if the heart rate calculation is valid

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
void optimizePower();

void setup() {
    // Hr sensor and Serial Monitor setup
    Serial.begin(115200);
    senSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

    // I2C setup
    Wire.begin(SDA_PIN, SCL_PIN);

    // Screen setup
    u8g2.begin();
    u8g2.setFont(u8g2_font_6x13_tf);
    u8g2.clearBuffer();

    // PWM setup for motor
    ledcAttach(DRIVE_EN, 16000, 11);
    ledcWrite(DRIVE_EN, 0); // MAX 2048 (2^11)

    // Sensor calibration
    // calibrateHrSensor();

    // Setup counting
    startMillis = millis();
    activateMillis = millis(); // Remove from here!!!!!*/

    // Setup SpO2 sensor
    // particleSensor.begin(Wire, I2C_SPEED_FAST); //Use default I2C port, 400kHz speed

    byte ledBrightness = 100; // Options: 0=Off to 255=50mA
    byte sampleAverage = 2;   // Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 2;         // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    byte sampleRate = 100;    // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 215;     // Options: 69, 118, 215, 411
    int adcRange = 16384;     // Options: 2048, 4096, 8192, 16384

    // particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
    // readSpSensor(bufferLength);

    // Output pins setup
    pinMode(N_SLEEP, OUTPUT);
    pinMode(DRIVE_PH, OUTPUT);
    pinMode(SOLENOID, OUTPUT);
    pinMode(VIBRATE, OUTPUT);
    pinMode(DRVOFF, OUTPUT);
}

void loop() {
    currentMillis = millis();

    checkButtons();
    optimizePower();

}

// Read sensor data
void readHrSensor() {
    senSerial.write(dataRequestMsg, sizeof(dataRequestMsg)); // Start data reading sequence
    while (!senSerial.available())
        ; // Wait until sesor available

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
bool calibrateHrSequence() {
    senSerial.write(calibRequestMsg, sizeof(calibRequestMsg)); // Start calibration sequence

    for (size_t i = 0; i < 50; i++) { // Try to calibrate for 50 cycles
        while (!senSerial.available())
            ; // Wait until sesor available

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

// Show time
void showTime() {
    u8g2.setFont(u8g2_font_6x13_tf); // Set up screen
    u8g2.clearBuffer();
    screenWakeMillis = millis();
    String minutes = (String)((screenWakeMillis - activateMillis) / 1000 / 60) + " min"; // Calculate minutes passed

    u8g2.drawStr(0, 20, "Time since activated"); 
    u8g2.drawStr(30, 40, minutes.c_str()); // Display minutes 
    u8g2.sendBuffer();
}

// Check buttons
void checkButtons() {

    switch (stopButton.checkButton()) {
    case 1: // Click stop button
        Serial.println("Normal click STOP");
        motorOff(); // Stop tightening process
        activation = 0;
        break;

    case 3: // Long hold stop button
        Serial.println("Long click STOP");
        u8g2.setPowerSave(0); // Wake up screen
        showTime();
        break;   
    }

    switch (powerButton.checkButton()) {
    case 1:
        Serial.println("Normal click POWER");
        break;

    case 3: // Long hold power button
        Serial.println("Long click POWER");
        vibrate(); // Manual tighten strap
        tightenStrap();
        break;
    }

    switch (minusButton.checkButton()) {
    case 1:
        Serial.println("Normal click MINUS");
        break;

    case 4: // Hold minus button
        //Serial.println("Continius MINUS");
        if (adjusting != 2) { // If pluss button is not being held down drive motor
            driveMotor(0);
            adjusting = 1;
        }
        break;

    default:
        if (adjusting != 2) { // If pluss button is not being held down switch motor off
            motorOff();
            adjusting = 0;
        }
    }

    switch (plussButton.checkButton()) {
    case 1:
        Serial.println("Normal click PLUS");
        break;

    case 4: // Hold pluss button
        //Serial.println("Continius PLUS");
        if (adjusting != 1) { // If minus button is not being held down drive motor
            driveMotor(1);
            adjusting = 2;
        }
        break;

    default:
        if (adjusting != 1) { // If minus button is not being held down switch motor off
            motorOff();
            adjusting = 0;
        }
    }
}

// Check health
void checkHealth() {
    // Logic to check data from senors and make calculations
    readHrSensor();
    if (activation == 0 && hrData[2] == 255) { // First check if Hr data is valid
        return;
    } else if (activation == 0 && hrData[2] > 180) { // If data is valid chack if pulse is very high
        readSpSensor(25);
        if (hrData[0] < 100 && spo2 < 90 && validSPO2) { // Last check if blood pressure and spo2 are low
            activation = 1;
            vibrate(); // Start tightening process
            tightenStrap(); 
        }
    }
    
    if (activation == 1) { // Continue tightening
        tightenStrap();
    }
}

// Read SpO2 sensor
void readSpSensor(int32_t n) {
    // dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = n; i < bufferLength; i++) {
        redBuffer[i - n] = redBuffer[i];
        irBuffer[i - n] = irBuffer[i];
    }

    // take 25 sets of samples before calculating the heart rate.
    for (byte i = bufferLength - n; i < bufferLength; i++) {
        while (particleSensor.available() == false) // do we have new data?
            particleSensor.check();                 // Check the sensor for new data

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); // We're finished with this sample so move to next sample
    }

    // After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}

// Drive motor
void driveMotor(int direction) {
    motorWakeUp();
    triggerSolenoid(1);
    digitalWrite(DRIVE_PH, direction == 1 ? HIGH : LOW);
    // Set pwm value
    ledcWrite(DRIVE_EN, 2000); // Use half of the speed
}

// Wake up motor
void motorWakeUp() {
    if (digitalRead(N_SLEEP) == LOW) { // If not awake, start with wake up sequence
        digitalWrite(N_SLEEP, HIGH);
        delay(1);
        digitalWrite(N_SLEEP, LOW);
        delayMicroseconds(25);
        digitalWrite(N_SLEEP, HIGH);
        digitalWrite(DRVOFF, LOW);
    }
}

// Turn motor off
void motorOff() {
    triggerSolenoid(0); // Lock in position of the strap
    ledcWrite(DRIVE_EN, 0);
    digitalWrite(DRVOFF, HIGH);
    digitalWrite(N_SLEEP, LOW);
}

// CalibrateHrSensor
void calibrateHrSensor() {
    u8g2.setFont(u8g2_font_6x13_tf); // Set up screen
    u8g2.clearBuffer();
    u8g2.setPowerSave(0);
    const auto msg  = calibrateHrSequence() ? "Calibrate Success" : "Calibrate Failed";
    u8g2.drawStr(0, 40, msg);
    u8g2.sendBuffer();
    delay(3000);
    u8g2.setPowerSave(1);
}

// Tighten Strap
void tightenStrap() {
    driveMotor(1);
    if (strapTorque() > 0.38) { // If torque is reaches set value stop motors
        motorOff();
        activation = 2;
    }
}

// Calculate strap torque
float strapTorque() {
    return analogRead(IPROPI) * 16 / 3760 * k_t; // must multiply by something ADC value != Current value
}

// Activate solenoid
void triggerSolenoid(int a) {
    digitalWrite(SOLENOID, a == 1 ? HIGH : LOW);
}

// Vibrate
void vibrate() {
    digitalWrite(VIBRATE, HIGH);
    delay(500);
    digitalWrite(VIBRATE, LOW);
}

// Optimize Power
void optimizePower() {
    if (millis() - screenWakeMillis > 10000) { // Turn off screen when 10 sec passed
        u8g2.setPowerSave(1);
    }
}
