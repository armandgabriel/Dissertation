#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Servo.h>
#include <Arduino_LSM6DS3.h>

long lastTime = millis();
bool isCKCmd = false;

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
  //readGPS();
  //smartDelay(5000);
  //delay(1500);


  long elapsedTime = millis() - lastTime;
  lastTime = lastTime + elapsedTime;
  
  receiveDataStream();
  //String t = "Hello";
  //int lengT = t.length(); lengT++;
  //Serial.println("Serial Length");
  //Serial.println(lengT);
  //byte plain[lengT];
  //t.getBytes(plain, lengT);
  //sendDataStream(plain, lengT);

 // Verificare comenzi la 1 secunda
 checkCommands(elapsedTime);
  
}

void checkCommands(long elapsedTime) {

  long ckCMTime = 0;

  ckCMTime = ckCMTime + elapsedTime;

  if(ckCMTime >= 1000) {
    isCKCmd = !isCKCmd;
    if(isCKCmd) {
      if(cmd.cmd == 1) {
          // Start Drone
          Serial.println("Drone is starting ");
          bool isStart = startDrone();
      }
      if (cmd.cmd == 2) {
          Serial.println("Drone is stoping ");
          bool isStop = stopDrone();
      } 
      if (cmd.cmd == 3) {
        Serial.println("Disconnecting...");
        bool isDisconnected = disconnectDrone();
      }
      if (cmd.cmd == 4) {
        Serial.println("Checking drone status...");
        checkStatus();
      }
      if (cmd.cmd == 5) {
        Serial.println("Assigning task to drone...");
        assignTask();
      }
      if (cmd.cmd == 6) {
        Serial.println("Starting the fly procedure...");
        bool isFlight = isFlying();
      }
      if (cmd.cmd == 7) {
        Serial.println("Calling the drone home...");
        callDrone();
      }
      if (cmd.cmd == 8) {
        Serial.println("Landing the drone home...");
        landDrone();
      }
      ckCMTime = ckCMTime - 1000;
    }
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    //while (gpsSerial.available())
    //  gps.encode(gpsSerial.read());
    readGPS();
  } while (millis() - start < ms);
}

// READ GPS DATA
void readGPS() {
  
  if(Serial1.available()) {
    gps.encode(Serial1.read());
    if(gps.location.isUpdated()) {
      Serial.println("Reading gps Data");
      gD.rawLat = gps.location.rawLat().billionths;
      gD.rawLng = gps.location.rawLng().billionths;
      gD.rawSpeed = gps.speed.mps();
      gD.rawAltitude = gps.altitude.meters();
      Serial.println("GPS DATA");
      Serial.println(gD.rawLat);
      Serial.println(gD.rawLng);
      Serial.println(gD.rawSpeed);
      Serial.println(gD.rawAltitude);
      
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

  //String gpsData = String(gD.rawLat) + "-" + String(gD.rawLng) + "-" + String(gD.rawSpeed) + "-" + String(gD.rawAltitude);
  //int dataLength = gpsData.length(); dataLength++;
  //uint8_t total[dataLength];
  //gpsData.StringToCharArray(total, dataLength);
  //char* total = new char[dataLength];
  //strcpy(total, gpsData.c_str());
  //LoRa.write(dataLength);
  //LoRa.write((uint8_t*)total, dataLength);
  //free(total);
  //LoRa.write(gD.rawLat);
  //LoRa.write(gD.rawLng);
  //LoRa.write(gD.rawSpeed);
  //LoRa.write(gD.rawAltitude);
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

  int index_T = 0;

  int textLength = text.length();
  char text_array[textLength+1];
  strcpy(text_array, text.c_str());
  
  char * token = strtok(text_array, "#");
  String commands[2];
  while(token != NULL) {
    Serial.println(token);
    commands[index_T] = token;
    token = strtok(NULL, "#");
    index_T++;
  }
  Serial.println("PRINTING COMMANDS:");
  Serial.println(commands[0]);
  Serial.println(commands[1]);
  cmd.cmd = (byte) commands[0].toInt();
  cmd.ack = (byte) commands[1].toInt();
  Serial.println("Command stack display:");
  Serial.println(cmd.cmd);
  Serial.println(cmd.ack);
}

bool startDrone() {

  return false;
}

bool stopDrone() {

  return false;
}

bool disconnectDrone() {

  return false;
}

void checkStatus() {
  
}

void assignTask() {
  
}

bool isFlying() {

  return false;
}

void callDrone() {
  
}

void landDrone() {
  
}
