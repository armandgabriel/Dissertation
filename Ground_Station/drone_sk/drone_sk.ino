#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Servo.h>
#include <Arduino_LSM6DS3.h>
#include <pt.h>

static struct pt pt1; // command proto
static struct pt pt2;

static int protothreadCheckCommands(struct pt *pt)
{
  static unsigned long lastTimeCheckedCommand = 0;
  PT_BEGIN(pt);
  while(1) {
    lastTimeCheckedCommand = millis();
    PT_WAIT_UNTIL(pt, millis() - lastTimeCheckedCommand > 2000);
    checkCommands();
    PT_WAIT_UNTIL(pt, millis() - lastTimeCheckedCommand > 2000);
  }
  PT_END(pt);
}

static int protothreadReadDPS(struct pt *pt) 
{
  static unsigned long lastTimeCheckedCommand = 0;
  PT_BEGIN(pt);
  while(1) {
    lastTimeCheckedCommand = millis();
    PT_WAIT_UNTIL(pt, millis() - lastTimeCheckedCommand > 10);
    //Serial.println("Checking GPS");
    readGPS();
    PT_WAIT_UNTIL(pt, millis() - lastTimeCheckedCommand > 10);
  }
  PT_END(pt);
}

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
  uint32_t rawSpeed;
  uint32_t rawAltitude;
  
} GPS_DATA;

GPS_DATA gD;

typedef struct {
  byte cmd;
  byte ack;
} COMMAND;

COMMAND cmd;

// SERVO 
Servo motor1;
#define MOTOR_PIN_1 7
Servo motor2;
#define MOTOR_PIN_2 6
Servo motor3;
#define MOTOR_PIN_3 5
Servo motor4;
#define MOTOR_PIN_4 4

static int motorSpeed = 0;

bool testRunMotors = false;

bool isINITMotorPhase = true;

void setup() {
  Serial.begin(9600);
  while(!Serial);

  Serial1.begin(9600);
  
  
  // Starting LoRa module
  if(!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while(1);
  }

//  // Setup Motor
//  motor1.attach(MOTOR_PIN_1);
//  motor2.attach(MOTOR_PIN_2);
//  motor3.attach(MOTOR_PIN_3);
//  motor4.attach(MOTOR_PIN_4);

PT_INIT(&pt1);
PT_INIT(&pt2);

  // SET Motors pins
//  motor1.attach(MOTOR_PIN_1);
//  motor2.attach(MOTOR_PIN_2);
//  motor3.attach(MOTOR_PIN_3);
//  motor4.attach(MOTOR_PIN_4);
}

void loop() {
  // put your main code here, to run repeatedly:
  //readGPS();
  //smartDelay(5000);
  //delay(1500);


  //long elapsedTime = millis() - lastTime;
  //lastTime = lastTime + elapsedTime;
  
  receiveDataStream();
  //String t = "Hello";
  //int lengT = t.length(); lengT++;
  //Serial.println("Serial Length");
  //Serial.println(lengT);
  //byte plain[lengT];
  //t.getBytes(plain, lengT);
  //sendDataStream(plain, lengT);

 // Verificare comenzi la 1 secunda
 //checkCommands(elapsedTime);
 protothreadCheckCommands(&pt1);
  //testSpinMotors();
 protothreadReadDPS(&pt2);
}

void checkCommands() {
  Serial.println("Checking commands...");
  Serial.println("COMMAND: ");
  Serial.println(cmd.cmd);
  if(cmd.cmd == 1) {
      // Start Drone
      Serial.println("Drone is starting ");
      bool isStart = startDrone();
      if(isStart) {
        // Send response ack back
        Serial.println("Sending ACK Back!");
        int messageLength = 2;
        byte message[messageLength]  = { 1, 2 };
        sendDataStream(message, messageLength);
        isStart = false;
      }
      cmd.cmd = 0;
  }
  if (cmd.cmd == 2) {
      Serial.println("Drone is stoping ");
      //bool isStop = stopDrone();
      cmd.cmd = 0;
  } 
  if (cmd.cmd == 3) {
    Serial.println("Disconnecting...");
    bool isDisconnected = disconnectDrone();
    cmd.cmd = 0;
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
  
  while(Serial1.available() > 0) {
    gps.encode(Serial1.read());
    if(gps.location.isUpdated()) {
      //Serial.println("*************************************");
      //Serial.println("Reading gps Data");
      //gD.rawLat = gps.location.rawLat().billionths;
      //gD.rawLng = gps.location.rawLng().billionths;
      gD.rawLat = gps.location.lat();
      gD.rawLng = gps.location.lng();
      gD.rawSpeed = gps.speed.mps();
      gD.rawAltitude = gps.altitude.meters();
//      Serial.println("GPS DATA");
//      Serial.println(gD.rawLat);
//      Serial.println(gD.rawLng);
//      Serial.println(gD.rawSpeed);
//      Serial.println(gD.rawAltitude);
//      Serial.println("*************************************");
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
  //LoRa.waitPacketSent();

  LoRa.write(gD.rawLat);
  LoRa.write(gD.rawLng);
  LoRa.write(gD.rawSpeed);
  LoRa.write(gD.rawAltitude);

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
  if(isINITMotorPhase) {
    
    testRunMotors = true;
     
//    int test_motor = 250;
//    int speed_1 = map(test_motor, 0, 1023, 0, 180);
//    int speed_2 = map(test_motor, 0, 1023, 0, 180);
//    int speed_3 = map(test_motor, 0, 1023, 0, 180);
//    int speed_4 = map(test_motor, 0, 1023, 0, 180);
//    motor1.write(speed_1);
//    motor2.write(speed_2);
//    motor3.write(speed_3);
//    motor4.write(speed_4);
    isINITMotorPhase = false;
    motorSpeed = 250;
  }
  
  return true;
}

bool stopDrone() {
  if(!isINITMotorPhase) {
    motor1.detach();
    motor2.detach();
    motor3.detach();
    motor4.detach();
    isINITMotorPhase = true;
    motorSpeed = 0;
  }
  testRunMotors = false;
  return false;
}

bool disconnectDrone() {

  return false;
}

void checkStatus() {
//  gD.rawLat = gps.location.rawLat().billionths;
//      gD.rawLng = gps.location.rawLng().billionths;
//      gD.rawSpeed = gps.speed.mps();
//      gD.rawAltitude = gps.altitude.meters();
//  String message = String(gD.rawLat) + ""
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

void testSpinMotors() {
  if(testRunMotors) {
    //Serial.println(motorSpeed);
    int test_motor = motorSpeed;
    int speed_1 = map(test_motor, 0, 1023, 0, 180);
    int speed_2 = map(test_motor, 0, 1023, 0, 180);
    int speed_3 = map(test_motor, 0, 1023, 0, 180);
    int speed_4 = map(test_motor, 0, 1023, 0, 180);
    motor1.write(speed_1);
    motor2.write(speed_2);
    motor3.write(speed_3);
    motor4.write(speed_4);
  }
}
