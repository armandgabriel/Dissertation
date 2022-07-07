#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Servo.h>
#include <Arduino_LSM6DS3.h>
#include <pt.h>

static struct pt pt1; // command proto
static struct pt pt2;
//static struct pt pt3; // IMU Thread

bool sendGPSData = false;

static bool isCheckCommands = false;

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

static int protothreadReadGPS(struct pt *pt) 
{
  static unsigned long lastTimeReadGPS = 0;
  PT_BEGIN(pt);
  while(1) {
    lastTimeReadGPS = millis();
    PT_WAIT_UNTIL(pt, millis() - lastTimeReadGPS > 10);
    //Serial.println("Checking GPS");
    readGPS();
    PT_WAIT_UNTIL(pt, millis() - lastTimeReadGPS > 10);
  }
  PT_END(pt);
}

typedef struct {
  long x;
  long y;
  long z;
} GYRO;

GYRO gyroData;

float x, y, z;

static int protothreadReadIMU(struct pt *pt) {
  static unsigned long lastTimeCheckedIMU = 0;
  
  PT_BEGIN(pt);
  while(1) {
    
    lastTimeCheckedIMU = millis();
    PT_WAIT_UNTIL(pt, millis() - lastTimeCheckedIMU > 500);
    readGyroDataIMU();
    PT_WAIT_UNTIL(pt, millis() - lastTimeCheckedIMU > 500);
  }
  PT_END(pt);
}

bool isGyroUp = false;

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
  uint32_t targetLat;
  uint32_t targetLng;
  char mode = 'N';
  uint32_t targetAltitude;
} COMMAND;

COMMAND cmd;


bool executeTask = false;
bool patrolTask = false;
bool surveyTask = false;
bool protectTask = false;


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
//PT_INIT(&pt3);

  // SET Motors pins
//  motor1.attach(MOTOR_PIN_1);
//  motor2.attach(MOTOR_PIN_2);
//  motor3.attach(MOTOR_PIN_3);
//  motor4.attach(MOTOR_PIN_4);


// SETUP IMU
  if(!IMU.begin()) {
    Serial.println("Failed to init IMU");
    while (1);
  }
  Serial.println();
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" HZ");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
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
 //checkCommands(elapsedTime);
 protothreadCheckCommands(&pt1);
  //testSpinMotors();
 protothreadReadGPS(&pt2);

 sendGPSDataF(elapsedTime);

 //protothreadReadIMU(&pt3);
 readGyroDataIMU();
 //displayGyroStats(elapsedTime);

 _executeTask();
 
}

void readGyroDataIMU() {
  
  if(IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(x, y, z);
      
      gyroData.x = x;
      //Serial.print(x);
      
      //Serial.print("\t");
      
      gyroData.y = y;
      //Serial.print(y);
      
      //Serial.print("\t");
      
      gyroData.z = z;
      //Serial.println(z);
    }
}

void displayGyroStats(long elapsedTime) {
  static long readGyro = 0;
  readGyro = readGyro + elapsedTime;
  if(readGyro >= 3000) {
    isGyroUp = !isGyroUp;
    if(isGyroUp) {
      
      if((gyroData.x > 0 || gyroData.x < 0) || 
          (gyroData.y < 0 || gyroData.y > 0) || 
          (gyroData.z < 0 || gyroData.z > 0) ) {
        Serial.print(gyroData.x);
        Serial.print("\t");
        Serial.print(gyroData.y);
        Serial.print("\t");
        Serial.println(gyroData.z);
      }
      readGyro = readGyro - 3000;
    }
  }
}

void sendGPSDataF(long elapsedTime) {
  static long readGD = 0;
  readGD = readGD + elapsedTime;
  if(readGD >= 5000) {
    sendGPSData = !sendGPSData;
    if(sendGPSData) {
      if(gD.rawLat > 0) {
        int gpsPayloadSize = 2;
        byte* gpsPayload = (byte*)malloc(gpsPayloadSize);
        gpsPayload[0] = 4;
        gpsPayload[1] = 2;
        sendDataStream(gpsPayload, gpsPayloadSize);
      }
      readGD = readGD - 5000;
    }
  }
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
    cmd.cmd = 0;
  }
  if (cmd.cmd == 5) {
    Serial.println("Assigning task to drone...");
    assignTask();
    cmd.cmd = 0;
  }
  if (cmd.cmd == 6) {
    Serial.println("Starting the fly procedure...");
    bool isFlight = isFlying();
    cmd.cmd = 0;
  }
  if (cmd.cmd == 7) {
    Serial.println("Calling the drone home...");
    callDrone();
    cmd.cmd = 0;
  }
  if (cmd.cmd == 8) {
    Serial.println("Landing the drone home...");
    landDrone();
    cmd.cmd = 0;
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
  int byteArrayLength = LoRa.read();
  byte* responseMessage = (byte*) malloc (byteArrayLength);
  for(int i = 0; i < byteArrayLength; i++) {
    responseMessage[i] = (byte) LoRa.read();
    Serial.print((int) responseMessage[i]);
    Serial.print("\t");
  }
  int commandNo = (uint8_t)LoRa.read();
  LoRa.packetRssi();
  Serial.println();
  Serial.println("LENGTH: ");
  Serial.print(byteArrayLength);
  Serial.println();
  if(commandNo == 2) {
     Serial.println("Just 2 commands !:O");
     cmd.cmd = (byte) responseMessage[0];
     cmd.ack = (byte) responseMessage[1];
  } else if(commandNo == 6) {
     Serial.println("We have lat lng and altitude + command ");
     cmd.cmd = (byte) responseMessage[0];
     cmd.ack = (byte) responseMessage[1];
     cmd.targetLat = (uint32_t)responseMessage[2];
     cmd.targetLng = (uint32_t)responseMessage[3];
     cmd.mode = (char)responseMessage[4];
     cmd.targetAltitude = (uint32_t)responseMessage[5];
     Serial.println((int)cmd.cmd);
     Serial.println((int)cmd.cmd);
     Serial.println(cmd.targetLat);
     Serial.println(cmd.targetLng);
     Serial.println(cmd.mode);
     Serial.println(cmd.targetAltitude);
     Serial.println("****************************");
  }
 
 
  /*
  String text = "";
  while(LoRa.available()) {
    int inChar = LoRa.read();
    text += (char) inChar;
  }
  Serial.println(text);
  int commandNo = (uint8_t)LoRa.read();
  LoRa.packetRssi();
  Serial.println(text);

  int index_T = 0;

  int textLength = text.length();
  char text_array[textLength+1];
  strcpy(text_array, text.c_str());
  
  char * token = strtok(text_array, "#");
  String commands[commandNo];
  while(token != NULL) {
    Serial.println(token);
    commands[index_T] = token;
    token = strtok(NULL, "#");
    index_T++;
  }
  Serial.println("PRINTING COMMANDS:");
  Serial.println(commandNo);
  Serial.println(commands[0]);
  Serial.println(commands[1]);
  if(commandNo == 6) {
    cmd.targetLat = (uint32_t)LoRa.read();
    cmd.targetLng = (uint32_t)LoRa.read();
    cmd.mode = (char)LoRa.read();
    cmd.targetAltitude = (uint32_t)LoRa.read();
  }
  cmd.cmd = (byte) commands[0].toInt();
  cmd.ack = (byte) commands[1].toInt();
  Serial.println("Command stack display:");
  Serial.println(cmd.cmd);
  Serial.println(cmd.ack);
  */
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
  if(cmd.mode == 'p') {
    patrolTask = true;
  }
  if(cmd.mode == 's') {
    surveyTask = true;
  }
  if(cmd.mode == 'r') {
    protectTask = true;
  }
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

void _executeTask() {
  if(executeTask) {
    if(patrolTask) {
      runningTaskP();
    }
    if(surveyTask) {
      runningTaskS();
    }
    if(protectTask) {
      runningTaskR();
    }
  }
}

void runningTaskP() {
  // Starting Patrol task
  
}

void runningTaskS() {
  // Starting Survey task
  
}

void runningTaskR() {
  // Starting Protect task
  
}
