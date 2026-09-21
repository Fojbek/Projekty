#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//setting up OLED screen
int screenWidth = 128;
int screenHeight = 64;
int oledReset = -1;

#define SCREEN_ADDRESS 0x3D

Adafruit_SSD1306 oled(screenWidth, screenHeight, &Wire, oledReset);

// joysticks pins
int ThXpin = A1;
int ThYpin = A0;
int RoXpin = A2;
int RoYpin = A3;

// data to send
struct ControllerData {
  float ThXval;
  float ThYval;
  float RoXval;
  float RoYval;
};

ControllerData Cdata;

// data to receive
struct Volts {
  float Vin;
  float Vout;
  float Vp;
};

Volts Vdata;

//setting up radio
int CE = 8;
int CSN = 7;

RF24 radio(CE,CSN);

const byte addresses[][6] = {"00001", "00002"}; //00001 to transmit Cdata and 00002 to receive Vdata

void setup() {
  Serial.begin(9600);
  pinMode(ThXpin,INPUT);
  pinMode(ThYpin,INPUT);
  pinMode(RoXpin,INPUT);
  pinMode(RoYpin,INPUT);
  radio.begin();
  radio.openWritingPipe(addresses[0]); //transmitter
  radio.openReadingPipe(1,addresses[1]); //receiver
  radio.setPALevel(RF24_PA_MAX);
  oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
}

void loop() {
  //reading data
  Cdata.ThXval = analogRead(ThXpin);
  Cdata.ThYval = analogRead(ThYpin);
  Cdata.RoXval = analogRead(RoXpin);
  Cdata.RoYval = analogRead(RoYpin);
  
  radio.stopListening(); //transmitter
  radio.write(&Cdata, sizeof(Cdata));

  //receiving data
  radio.startListening(); //receiver
  bool gotVoltage = false;
  unsigned long started = millis();

  while (millis() - started < 8) {  //waiting for the Vdata
    if (radio.available()) {
      radio.read(&Vdata, sizeof(Vdata));
      gotVoltage = true;
      break;
    }
  }

  //OLED
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.setTextColor(WHITE);
  oled.setTextSize(2);
  oled.println("Battery:");

  if (gotVoltage) {
    oled.print(Vdata.Vp, 1);
    oled.print("%");
  } else {
    oled.print("---"); //lost signal
  }

  oled.display();

  delay(5);
}
  
