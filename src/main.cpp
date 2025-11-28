#include <main.hpp>

void setup() {

  //I2C setup
  Wire.begin(SDA_PIN, SCL_PIN);

  // Screen setup
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.clearBuffer();


  // Hr sensor and Serial Monitor setup
  Serial.begin(115200);
  senSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // PWM setup for motor
  ledcAttach(DRIVE_EN, 20000, 11);
  ledcWrite(DRIVE_EN, 0); // MAX 2048 (2^11)

  // Sensor calibration
  // calibrateHrSensor();

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


  // Pins setup
  pinMode(N_SLEEP, OUTPUT);
  pinMode(DRIVE_PH, OUTPUT);
  pinMode(SOLENOID, OUTPUT);
  pinMode(VIBRATE, OUTPUT);
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
  //checkHealth();



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
bool calibrateHrSequence() {
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


// Show time
void showTime() {
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.clearBuffer();
  String minutes = (String)((millis() - activateMillis) / 1000 / 60) + " min"; // Calculate minutes passed

  u8g2.drawStr(0, 20, "Time since activated");
  u8g2.drawStr(30, 40, minutes.c_str());
  u8g2.sendBuffer();
}


// Check buttons
void checkButtons() {

  switch (stopButton.checkButton()) {
    case 1:
      Serial.println("Normal click");
      break;

    case 3:
      Serial.println("Long click");
      break;
  }

  if (!digitalRead(BUTTON_POW)) {
    // Stop motors
    //rgbLedWrite(8, 255, 0, 0);
  } else {
    //rgbLedWrite(8, 0, 0, 255);
  }
  
  if (!digitalRead(BUTTON_SCREEN)) {
    u8g2.setPowerSave(0); // Wake up screen
    showTime();
  } else {
    u8g2.setPowerSave(1); // Screen to sleep mode
  }

  if (!digitalRead(BUTTON_PLUSS)) {
    driveMotor(1); // Motor +
  } else if (!digitalRead(BUTTON_MINUS)) {
    driveMotor(0); // Motor -
  } else {
    motorOff();
  }
}


// Check health
void checkHealth() {
  // Logic to check data from senors and make calculations

  if (false && activation == 0) {
    vibrate();
    tightenStrap();
    activation = 1;
  } else if (activation == 1) {
    tightenStrap();
  }
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
  motorWakeUp();
  triggerSolenoid(1);
  if (direction == 1) {
    digitalWrite(DRIVE_PH, HIGH); // Set direction +
  } else {
    digitalWrite(DRIVE_PH, LOW); // Set direction -
  }
  // Set pwm value
  ledcWrite(DRIVE_EN, 1000); // Use half of the speed
}


// Wake up motors
void motorWakeUp() {
  if (digitalRead(N_SLEEP) == LOW) {
    digitalWrite(N_SLEEP, HIGH);
    delay(1);
    digitalWrite(N_SLEEP, LOW);
    delayMicroseconds(25);
    digitalWrite(N_SLEEP, HIGH);
  }
}


// Turn motor off
void motorOff() {
  ledcWrite(DRIVE_EN, 0);
  triggerSolenoid(0);
  digitalWrite(N_SLEEP, LOW);
}


// CalibrateHrSensor
void calibrateHrSensor() {
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.clearBuffer();
  u8g2.setPowerSave(0);
  if(calibrateHrSequence()) {
    u8g2.drawStr(0, 40, "Calibrate Success");
  } else {
    u8g2.drawStr(0, 40, "Calibrate Failed");
  }
  u8g2.sendBuffer();
  delay(3000);
  u8g2.setPowerSave(1);
}


// Tighten Strap
void tightenStrap() {
  driveMotor(1);
  if (strapTorque() > 0.38) {
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
  if (a == 1) {
    digitalWrite(SOLENOID, HIGH);
  } else {
    digitalWrite(SOLENOID, LOW);
  }
}


// Vibrate
void vibrate() {
  digitalWrite(VIBRATE, HIGH);
  delay(500);
  digitalWrite(VIBRATE, LOW);
}
