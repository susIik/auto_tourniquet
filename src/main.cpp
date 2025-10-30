#include <Arduino.h>
#include <U8g2lib.h>
#include <HardwareSerial.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);  // High speed I2C

#define RXD2 5
#define TXD2 6

byte data[] = { 0xFD, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte calib[] = { 0xFE, 0x7A, 0x47, 0x46, 0x00, 0x00 }; 

HardwareSerial senSerial(0);

void readHrSensor(int *hrData);

void setup() {
  // put your setup code here, to run once:
  //int result = myFunction(2, 3);
  u8g2.begin();
  Serial.begin(115200);
  senSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial 2 started at 115200 baud rate");
  senSerial.write(calib, sizeof(calib));
  delay(10000);
}

void loop() {
  // put your main code here, to run repeatedly:
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_6x13_tf);	// choose a suitable font



  int hrData[] = {255, 255, 255};
  readHrSensor(hrData);

  String mes1 = "Heart rate: " + (String)hrData[2];
  String mes2 = "Blood p H/L: " + (String)hrData[0] + "/" + (String)hrData[1];
  Serial.println(mes1);
  Serial.println(mes2);
 

  u8g2.drawStr(0, 20, mes1.c_str());
  u8g2.drawStr(0, 40, mes2.c_str());	// write something to the internal memory
  u8g2.sendBuffer();


  delay(1000);
  Serial.println("-------------------------------");
}


// Read sensor data
void readHrSensor(int *hrData) {
  senSerial.write(data, sizeof(data));
  while (!senSerial.available());

  while (senSerial.available()) {
    byte a = senSerial.read();
    delay(10);
    if (int(a) == 253) {
      for (size_t i = 0; i < 3; i++) {
        hrData[i] = senSerial.read();
        delay(10);
      }
    }
  }
}