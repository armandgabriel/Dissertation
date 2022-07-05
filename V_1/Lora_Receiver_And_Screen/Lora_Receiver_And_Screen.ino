#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
//BMP180
#include <Adafruit_BMP085.h>
#include <MechaQMC5883.h>
#include <Arduino_LSM6DS3.h>
#include <TinyGPS++.h>
//#include <SparkFun_ATECCX08a_Arduino_Library.h>
#define seaLevelPressure_hPa 1013.25
bool isReadBMP = false;
// Accelerometer
bool gyroUP = false;
#define address 0x0D
MechaQMC5883 qmc;
bool internalGyroUP = false;

// Protothread
//#include <pt.h>
long lastTime = millis();


// BMP180
Adafruit_BMP085 bmp;


// GPS MODULE
TinyGPSPlus gps;
bool gpsUP = false;

// ----------------------------------------------------------------------------- //
// define screen width and height
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// DEFINE OLDE
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ----------------------------------------------------------------------------- //
// LORA 
byte localAddress = 0x4D;
byte targetAddress = 0x2F;

// ----------------------------------------------------------------------------- //
// Read Data from I2C Network 
//int i2CAddress = 0x6a;

// CRYPTO
//ATECCX08A atecc;

// Struct for data transmission
struct DATA_OBJ {
  byte akn = 0x0E;
  byte sender = 0x4D;
  byte receiver = 0x2F;
  String lg;
  String lt;
  String speed;
};

DATA_OBJ _lOraObj;


void setup() {
  // DISPLAY IN SERIAL MONITOR ;;
  Serial.begin(9600);
  Serial1.begin(9600);
  while(!Serial);
  
  // SETUP - OLED SCREEN //
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);

  // LORA SETUP
  if(!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa module failed on address 433E6");
    while(1);
  }
  
  pinMode(2, INPUT);
  // Setup I2C 
  //Wire.begin(i2CAddress);
  //Wire.onReceive(receiveEvent);
  //Wire.onRequest(sendData);
  
  // BMP180;
  if (!bmp.begin(0x77)) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }

  // SETUP Accelerometer
  // lower I2C clock http://www.gammon.com.au/forum/?id=10896
  Wire.begin();
  Init_QMC5803L();
  Init_InternalGyro();

 
}


void Init_InternalGyro() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");  
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
}

void Init_QMC5803L(){
  
  /* Set the module to 8x averaging and 15Hz measurement rate */
  qmc.init();
  //qmc.setMode(Mode_Standby,ODR_200Hz,RNG_8G,OSR_512);
}

int counter = 0;
int buttonState = 0;

void loop() {
  // get time elapsed
  long elapsedTime = millis() - lastTime;
  lastTime = lastTime + elapsedTime;

  // READ GPS
  readGPS(elapsedTime); 
  
  // LORA RECEIVE
  readLoRa();
  // PRESS BUTTON
  pressButtonFunctionTest();
  // READ BMP180 
  readBMP(elapsedTime);

  // GYROS
  readGyro(elapsedTime);
  //readInternalGyro(elapsedTime);

  
}

void pressButtonFunctionTest() {
  int buttonChanged = digitalRead(2);
  if(buttonChanged != buttonState) {
    Serial.println(buttonChanged);
    sendLoRa();
  }
}

void sendLoRa() {
  LoRa.beginPacket();
  //LoRa.print("Salut inapoi!");
  LoRa.print(_lOraObj.akn);
  LoRa.print(_lOraObj.sender);
  LoRa.print(_lOraObj.receiver);
  LoRa.print(_lOraObj.lg);
  LoRa.print(_lOraObj.lt);
  LoRa.endPacket();
}

void readLoRa() {
int packetSize = LoRa.parsePacket();
  if(packetSize) {
    Serial.println("working...");
    Serial.println();
    Serial.print("Packet Size... ");
    Serial.print(packetSize);
    Serial.println();
    String text = receiveLoRaEvent();
    // DISPLAY SHOW
    Serial.println(text);
    displayOLED(text);
  }
}

void displayOLED(String text) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setCursor(1, 4);
  display.println();
  display.print(text);
  display.println();
  display.display();
}

String receiveLoRaEvent() {
  String text = "";
  while(LoRa.available()) {
    int inChar = LoRa.read();
    text += (char)inChar;
    
  }
  LoRa.packetRssi();
  return text;
}

//void receiveEvent(int bytes) {
//  
//}

//void sendData() {
//  
//}

void readBMP(long elapsedTime) {
  // protothreading
  static long readBMPTime = 0;

  // update total elapsed time 
  readBMPTime = readBMPTime + elapsedTime;

  if(readBMPTime >= 5000) {
    isReadBMP = !isReadBMP;
  
    if(isReadBMP) {
      Serial.println("READ BMP DATA");
      Serial.println("*************************************************************");
      Serial.print("Temperature = ");
      Serial.print(bmp.readTemperature());
      Serial.println(" *C");
      
      Serial.print("Pressure = ");
      Serial.print(bmp.readPressure());
      Serial.println(" Pa");
    
      Serial.print("Altitude = ");
      Serial.print(bmp.readAltitude());
      Serial.println(" meters");
    
      Serial.print("Pressure at sealevel (calculated) = ");
      Serial.print(bmp.readSealevelPressure());
      Serial.println(" Pa");
    
      Serial.print("Real altitude = ");
      Serial.print(bmp.readAltitude(seaLevelPressure_hPa * 100));
      Serial.println(" meters");
      
      Serial.println();
      readBMPTime = readBMPTime - 5000;
    }
  }
}

int x,y,z; //triple axis data

char bufferX [20];
char bufferY [20];
char bufferZ [20];

#define X 3
#define Y 7
#define Z 5

void readGyro(long elapsedTime) {

  x = 0;
  y = 0;
  z = 0;
  long a;  
  static long readGyro = 0;

  readGyro = readGyro + elapsedTime;
  if(readGyro >= 1000) {
    gyroUP = !gyroUP;
    if(gyroUP) {
      qmc.read(&x,&y,&z);
      a = qmc.azimuth(&x, &y);
      Serial.print("x: ");
      Serial.print(x);
      Serial.print(" y: ");
      Serial.print(y);
      Serial.print(" z: ");
      Serial.print(z);
      Serial.print(" AZIMUTH: ");
      Serial.print(a);
      Serial.println();
      readGyro = readGyro - 1000;
    }
  }  
}

void readInternalGyro(long elapsedTime) {
  float _x, _y, _z;
  int plusThreshold = 30;
  int minusThreshold = -30;
  
  static long internalGyro = 0;
  internalGyro = internalGyro + elapsedTime;
  if(internalGyro >= 3000) {
    internalGyroUP = !internalGyroUP;
    if(internalGyroUP) {
      Serial.println("READ INTERNAL GYRO DATA");
      Serial.println("*************************************************************");
      Serial.println("Internal Gyro:");
      Serial.println();
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(_x, _y, _z);
      }
      if(_y > plusThreshold)
      {
        Serial.println("Collision front");
        delay(25);
      }
      if(_y < minusThreshold)
      {
        Serial.println("Collision back");
        delay(25);
      }
      if(_x < minusThreshold)
      {
        Serial.println("Collision right");
        delay(25);
      }
      if(_x > plusThreshold)
      {
        Serial.println("Collision left");
        delay(25);
      }
      internalGyro = internalGyro - 3000;
    }
  }
}
  
void readGPS(long elapsedTime) {
  static long readGPS = 0;
  readGPS = readGPS + elapsedTime;
  //if(readGPS >= 50) {
    //gpsUP = !gpsUP;
   // if(gpsUP) {
        if(Serial1.available()) {
          gps.encode(Serial1.read());
          if(gps.location.isUpdated()) {
            Serial.println("READ GPS DATA");
            Serial.println("*************************************************************");
            Serial.println("GPS UPDATED");
//            Serial.print("Latittude=");
//            Serial.print(gps.location.lat(), 6);
//            Serial.print(" Longitude=");
//            Serial.println(gps.location.lng(), 6);
            _lOraObj.lg = String(gps.location.lng(),6);
            _lOraObj.lt = String(gps.location.lat(),6);
            _lOraObj.speed = gps.speed.mps();

            // Raw latitude in whole degrees
            Serial.print("Raw latitude = "); 
            Serial.print(gps.location.rawLat().negative ? "-" : "+");
            Serial.println(gps.location.rawLat().deg); 
            // ... and billionths (u16/u32)
            Serial.println(gps.location.rawLat().billionths);
            
            // Raw longitude in whole degrees
            Serial.print("Raw longitude = "); 
            Serial.print(gps.location.rawLng().negative ? "-" : "+");
            Serial.println(gps.location.rawLng().deg); 
            // ... and billionths (u16/u32)
            Serial.println(gps.location.rawLng().billionths);
          } 
        } 
     // readGPS = readGPS - 50;
    //}
  //}
}
