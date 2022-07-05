#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Servo.h>
#include <Arduino_LSM6DS3.h>

long lastTime = millis();

byte localAddress = 0x2f;
byte targetAddress = 0x4d;
int counterID = 0;


// TinyGPS++
TinyGPSPlus gps;
typedef struct {
  uint32_t rawLat;
  uint32_t rawLng;
  double rawSpeed;
  double rawAltitude;
  
} GPS_DATA;

GPS_DATA gD;

typedef struct {
  byte cmd;
  byte ack;
} COMMAND;

COMMAND cmd;

void setup() {
  Serial.begin(9600);
  while(!Serial);

  Serial1.begin(9600);
  

  // Starting LoRa module
  if(!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while(1);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  readGPS();
  delay(15);
  
  receiveDataStream();
}

// READ GPS DATA
void readGPS() {
  if(Serial1.available()) {
    gps.encode(Serial1.read());
    if(gps.location.isUpdated()) {
      gD.rawLat = gps.location.rawLat().billionths;
      gD.rawLng = gps.location.rawLng().billionths;
      gD.rawSpeed = gps.speed.mps();
      gD.rawAltitude = gps.altitude.meters();
      
    }
  }
}

void sendDataStream(byte* message, unsigned int length) {
  LoRa.beginPacket();
  //LoRa.print(message);
  LoRa.write(targetAddress);
  LoRa.write(localAddress);
  counterID++;
  LoRa.write(byte(counterID));
  LoRa.write(length);
  LoRa.write((uint8_t*)message, length);
  LoRa.write(gD.rawLat);
  LoRa.write(gD.rawLng);
  LoRa.write(gD.rawSpeed);
  LoRa.write(gD.rawAltitude);
  LoRa.endPacket();
}

void receiveDataStream() {
  int packetSize = LoRa.parsePacket();
  if(packetSize == 0) {
    return;
  }
  String text = "";
  while(LoRa.available()) {
    int inChar = LoRa.read();
    text += (char) inChar;
  }
  LoRa.packetRssi();
  Serial.println(text);
  int r = 0, t = 0;
  String commands[2];
  for(int i = 0; i < text.length(); i++) {
    if(text.charAt(i) == '#') {
      commands[t] = text.substring(r, i);
      r = (i + 1);
      t++;
    }
  }
  cmd.cmd = (byte) commands[0].toInt();
  cmd.ack = (byte) commands[1].toInt();
}
