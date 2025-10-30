#include <Arduino.h>
#include <U8g2lib.h>
#include <HardwareSerial.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);  // High speed I2C

#define RXD2 5
#define TXD2 6

byte get_data[] = { 0xFD, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte calib[] = { 0xFE, 0x7A, 0x47, 0x46, 0x00, 0x00 }; 

unsigned long startMillis; 
unsigned long currentMillis;

HardwareSerial senSerial(0);

void readHrSensor(int *hrData);
bool calibrateSensor();

void setup() {
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.clearBuffer();
  
  Serial.begin(115200);
  senSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);


  if(calibrateSensor()) {
    u8g2.drawStr(0, 40, "Calibrate Success");
  } else {
    u8g2.drawStr(0, 40, "Calibrate Failed");
  }
  u8g2.sendBuffer();
  delay(3000);
  u8g2.setPowerSave(1);

  startMillis = millis();
}

void loop() {
  u8g2.clearBuffer(); // clear the internal memory

  int hrData[] = {255, 255, 255};

  currentMillis = millis();
  if (currentMillis - startMillis >= 1000) {
    readHrSensor(hrData);
    String mes1 = "Heart rate: " + (String)hrData[2];
    String mes2 = "Blood p H/L: " + (String)hrData[0] + "/" + (String)hrData[1];
    Serial.println(mes1);
    Serial.println(mes2);
    Serial.println("-------------------------------");
    startMillis = currentMillis;  
  }
 

  //u8g2.drawStr(0, 20, mes1.c_str());
  //u8g2.drawStr(0, 40, mes2.c_str());	// write something to the internal memory
  //u8g2.sendBuffer();


  // delay(1000);
  // u8g2.setPowerSave(1);
  // delay(2000);
  // u8g2.setPowerSave(0);
}


// Read sensor data
void readHrSensor(int *hrData) {
  senSerial.write(get_data, sizeof(get_data));
  while (!senSerial.available());

  while (senSerial.available()) {
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
  senSerial.write(calib, sizeof(calib));

  for (size_t i = 0; i < 50; i++) {
    while (!senSerial.available());

    while (senSerial.available()) {
      byte a = senSerial.read();
      if (int(a) == 254) {
        byte b;
        for (size_t i = 0; i < 3; i++) {
          b = senSerial.read();
        }
        if (b == 0x00) {
          return true;
        } else if (b == 0x02) {
          return false;
        }
      }
    }
  }
  return false;
}