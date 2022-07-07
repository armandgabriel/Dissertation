//#include <ArduinoMqttClient.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LoRa.h>

// counter display gps data
long lastTime = millis();
bool isDisplayGPSData = false;

// WIFI Connection
char ssid[] = "DIGI-9jB5";
char pass[] = "zw6rxkaa";
WiFiClient wifiClient;
// MQTT Client Object
//MqttClient mqttClient(wifiClient);
PubSubClient mqttClient(wifiClient);
const char broker[] = "test.mosquitto.org";
int port = 1883;
char topic0[] = "drone/0/connect";
char topic1[] = "drone/1/start";
char topic2[] = "drone/2/stop";
char topic3[] = "drone/3/disconnect";
char topic4[] = "drone/4/checkDrone";
char topic5[] = "drone/5/assignTask";
char topic6[] = "drone/6/fly";
char topic7[] = "drone/7/callDrone";
char topic8[] = "drone/8/landDrone";
char topic9[] = "drone/9/status";

typedef struct { 
  uint32_t rawLat = 0;
  uint32_t rawLng = 0;
  uint32_t rawSpeed = 0;
  uint32_t rawAltitude = 0;
} GPS_DATA;

GPS_DATA gD;

// Screen Setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// LORA Setup
byte localAddress = 0x4d;
byte targetAddress = 0x2f;



void setup() {
  // Serial Communication Init
  Serial.begin(9600);
  // Serial Communication send data on USB
  // On Standalone comment this line
  //while(!Serial) { ; }

  //OLED SETUP
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);

  displayOLED("Setup phase...", "Attempting to start modules", "", "");
  
  // WIFI Setup
  Serial.println("Connecting to Wifi...");
  displayOLED("Setup phase...", "Connecting to WIFI", "", "");
  delay(500);
  while(WiFi.begin(ssid, pass) != WL_CONNECTED) { 
    Serial.println("Failed to connect to WiFi...");
    Serial.println("Attempting 1 more try in 2 sec...");
    displayOLED("Setup phase...", "Failed to connect", "Attempting 1 more try", "in 2 seconds");
    delay(2000); 
  }
  delay(500);
  Serial.println("You are now connected to WiFi network");
  displayOLED("Setup phase...", "Connected to WIFI", "", "");
  // Connecting to MQTT Broker
  delay(500);
  Serial.println("Attempting to connect to MQTT Broker...");
  displayOLED("Setup phase...", "Attempting to connect", "to MQTT Broker...", "");
//  if(!mqttClient.connect(broker, port)) {
//    Serial.print("MQTT connection failed! Error code = ");
//    Serial.println(mqttClient.connectError());
//    while(1);
//  }
  mqttClient.setServer(broker, port);
  mqttClient.setCallback(callback);
  if(mqttClient.connect("drone_test_armand")) {
    displayOLED("Setup phase...", "Connected to MQTT Broker", "", "");
  } else {
    displayOLED("Setup phase...", "Could not connect to MQTT Broker", "", "");
  }
  delay(750);
  Serial.println("You are connected to the MQTT broker!");
  displayOLED("Setup phase...", "You are connected", "to the MQTT Broker!", "");
  Serial.println();
  delay(2500);
  Serial.println("Attempting to send data to mqtt broker on topic 1");
  displayOLED("Setup phase...", "Publishing a test sample ", "to be displayed on ", "mobile app.");
  //mqttClient.beginMessage(topic);
  char test[] = "Hello World";
  //mqttClient.print(test);
  //mqttClient.endMessage();
  delay(500);
  boolean rc = mqttClient.publish(topic9, test);
  if(rc) {
    Serial.println("Sent message");
    displayOLED("Setup phase...", "Message published", "Check your mobile application", "");
  } else {
    Serial.println("Error sending message");
    displayOLED("Setup phase...", "Could not publish the message", "", "");
  }

  delay(1000);
  displayOLED("Setup phase...", "Subscribing to topics", "", "");
  subscribeTopics();
  
  delay(1000);
  displayOLED("Setup phase...", "Succesfully subscribed to topics", "", "");
  delay(500);
  displayOLED("Setup phase...", "Attempting to connect to the LoRa module", "", "");
  // Attempting to connect to LoRa Drone module
  if(!LoRa.begin(433E6)){
    Serial.println("Starting LoRa module failed");
    delay(10);
    displayOLED("Setup phase...", "Connection failed...", "Attempting one more time...", "");
    while(1);
  }

  Serial.println("Connected to LoRa module!");
  displayOLED("Setup phase...", "Connected to the LoRa module!", "", "");
  Serial.println("Starting ground station...");
  
}

void subscribeTopics() {
  Serial.println("Subscribing to Topics");
  boolean sub_0 = mqttClient.subscribe(topic0, 1);
  if(sub_0) {
    Serial.println("Subscribed to topic 0");
  } else {
    Serial.println("Could not subscribe to topic 0");
  }
  delay(85);
  boolean sub_1 = mqttClient.subscribe(topic1, 1);
  if(sub_1) {
    Serial.println("Subscribed to topic 1");
  } else {
    Serial.println("Could not subscribe to topic 1");
  }
  delay(85);
  boolean sub_2 = mqttClient.subscribe(topic2, 1);
  if(sub_2) {
    Serial.println("Subscribed to topic 2");
  } else {
    Serial.println("Could not subscribe to topic 2");
  }
  delay(85);
  boolean sub_3 = mqttClient.subscribe(topic3, 1);
  if(sub_3) {
    Serial.println("Subscribed to topic 3");
  } else {
    Serial.println("Could not subscribe to topic 3");
  }
  delay(85);
  boolean sub_4 = mqttClient.subscribe(topic4, 1);
  if(sub_4) {
    Serial.println("Subscribed to topic 4");
  } else {
    Serial.println("Could not subscribe to topic 4");
  }
  delay(85);
  boolean sub_5 = mqttClient.subscribe(topic5, 1);
  if(sub_5) {
    Serial.println("Subscribed to topic 5");
  } else {
    Serial.println("Could not subscribe to topic 5");
  }
  delay(85);
  boolean sub_6 = mqttClient.subscribe(topic6, 1);
  if(sub_6) {
    Serial.println("Subscribed to topic 6");
  } else {
    Serial.println("Could not subscribe to topic 6");
  }
  boolean sub_7 = mqttClient.subscribe(topic7, 1);
  if(sub_7) {
    Serial.println("Subscribed to topic 7");
  } else {
    Serial.println("Could not subscribe to topic 7");
  }
  delay(85);
  boolean sub_8 = mqttClient.subscribe(topic8, 1);
  if(sub_8) {
    Serial.println("Subscribed to topic 8");
  } else {
    Serial.println("Could not subscribe to topic 8");
  }
}

void loop() {
  //mqttClient.poll();
  long elapsedTime = millis() - lastTime;
  lastTime = lastTime + elapsedTime;

  
  if(!mqttClient.connected()) {
    //mqttClient.loop();
    reconnect();
  }
  mqttClient.loop();
  receiveDataStream();

  displayGPSData(elapsedTime);
}

void displayGPSData(long elapsedTime) {

  static long readGD = 0;
  readGD = readGD + elapsedTime;
  if(readGD >= 5000) {
    isDisplayGPSData = !isDisplayGPSData;
      if(isDisplayGPSData) {
        if(gD.rawLat > 0) {
        Serial.println("GPS DATA: ");
        Serial.println(gD.rawLat);
        Serial.println(gD.rawLng);
        Serial.println(gD.rawSpeed);
        Serial.println(gD.rawAltitude);
      } else {
        Serial.println("No GPS Data!");
      }
      readGD = readGD - 5000;
    }
    
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

void callback(char* topic, byte* payload, unsigned int length) {

  if(strcmp(topic, topic0) == 0) {
    Serial.println("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for(int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
    Serial.println("Length: " );
    Serial.print(length);
    Serial.println();
    Serial.println(length);
    sendDataStream(payload, length, 0);
    Serial.println();
  }
  
  if(strcmp(topic, topic1) == 0) {
    Serial.println("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
//    for(int i = 0; i < length; i++) {
//      Serial.print((char)payload[i]);
//    }
    Serial.println((char*)payload);
    Serial.println("Length: " );
    Serial.print(length);
    Serial.println();
    Serial.println(length);
    sendDataStream(payload, length, 0);
    Serial.println();
  }

  if(strcmp(topic, topic2) == 0) {
    Serial.println("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for(int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    sendDataStream(payload, length, 0);
    Serial.println();
  }

  if(strcmp(topic, topic3) == 0) {
    Serial.println("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for(int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    sendDataStream(payload, length, 0);
    Serial.println();
  }

  if(strcmp(topic, topic4) == 0) {
    Serial.println("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for(int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    sendDataStream(payload, length, 0);

    if(gD.rawLat > 0) {
      int gDSize = 6;
      byte* gpsPayload = (byte*) malloc(gDSize);
      for(int i = 0; i < gDSize; i++) {
        if(i == 0) { 
          gpsPayload[i] = 4;
        }
        if(i == 1) {
          gpsPayload[i] = 2;
        }
        if(i == 2) {
          gpsPayload[i] = gD.rawLat;
        }
        if(i == 3) {
          gpsPayload[i] = gD.rawLng;
        }
        if(i == 4) {
          gpsPayload[i] = gD.rawSpeed;
        }
        if(i == 5) {
          gpsPayload[i] = gD.rawAltitude;
        }
      }
      pub(topic9, gpsPayload, gDSize);
      
    }
    
    Serial.println();
  }

  if(strcmp(topic, topic5) == 0) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    Serial.println();
    Serial.println("LENGTH: " );
    Serial.print(length);
    Serial.println();
    for(int i = 0; i < length; i++) {
      Serial.print((int)payload[i]);
      Serial.print("\t");
    }
    sendDataStream(payload, length, 4);
    Serial.println();
  }

  if(strcmp(topic, topic6) == 0) {
    Serial.println("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for(int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    sendDataStream(payload, length, 0);
    Serial.println();
  }


  if(strcmp(topic, topic7) == 0) {
    Serial.println("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for(int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    sendDataStream(payload, length, 0);
    Serial.println();
  }

  if(strcmp(topic, topic8) == 0) {
    Serial.println("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for(int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    sendDataStream(payload, length, 0);
    Serial.println();
  }
  
  
}

void pub(char* topic, byte* payload, unsigned int length) {
  byte* p =(byte*)malloc(length);
  memcpy(p, payload, length);
  mqttClient.publish(topic, p, length, false);
  free(p);
}

void reconnect() {
  // Loop until we're reconnected
  char test[] = "Hello World Reconnect";
  while (!mqttClient.connected()) {
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("drone_test_armand")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttClient.publish(topic9,test);
      // ... and resubscribe
      //mqttClient.subscribe(topic2);
      subscribeTopics();
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 2.5 seconds before retrying
      delay(2500);
    }
  }
}

// LoRa
void sendDataStream(byte* message, unsigned int length, unsigned int commandNo) {
  displayOLED("SENDING LoRa message...", "Content Length", length + "", "");
  LoRa.beginPacket();
  //LoRa.print(message);
  LoRa.write(length);
  LoRa.write((uint8_t*)message, length);
  Serial.println(commandNo);
  if(commandNo == 4) {
    LoRa.write((uint8_t)6);
  } else {
    LoRa.write((uint8_t)2);
  }
  LoRa.endPacket();
}

void receiveDataStream() {
  int packetSize = LoRa.parsePacket();
  if(packetSize) {

    byte recep = LoRa.read();
    byte sender = LoRa.read();
    byte msgID = LoRa.read();
    byte incLength = LoRa.read();
    
    //String text = "";
    //while(LoRa.available()) {
    //  int inChar = LoRa.read();
    //   text += (char)inChar;
    //}
    byte* responseMessage = (byte*)malloc(incLength);//[incLength];
    for(int i = 0; i < incLength; i++) {
      responseMessage[i] = (byte) LoRa.read();
    }
    //Serial.println("Size of text");
    //Serial.println(incLength);
    //Serial.println(text);
    //...
//    gD.rawLat = LoRa.read();
//    gD.rawLng = LoRa.read();
//    gD.rawSpeed = LoRa.read();
//    gD.rawAltitude = LoRa.read();
    //int gpsDataLength = LoRa.read();
    //String gpsData = "";
    //while(LoRa.available()) {
    //  int inChar = LoRa.read();
    //  gpsData += (char)inChar;
    //}
    gD.rawLat = (uint32_t)LoRa.read();
    gD.rawLng = (uint32_t)LoRa.read();
    gD.rawSpeed = (uint32_t)LoRa.read();
    gD.rawAltitude = (uint32_t)LoRa.read();
    
    LoRa.packetRssi();
    //int length = sizeof(text);
    
    //byte* p = (byte*)malloc(incLength);
    //memcpy(p, text, length);
    //text.getBytes(p, incLength);
    displayOLED("RECEIVED LoRa message...", "Content Length", incLength + "", "");
    //Serial.println(text);
    //pub(topic9, responseMessage, incLength); 
    //free(p);
  }
}

// Display Oled
void displayOLED(String text, String text2, String text3, String text4) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setCursor(0, 1);
  display.println();
  display.print(text);
  display.println();
  display.print(text2);
  display.println();
  display.print(text3);
  display.println();
  display.print(text4);
  display.println();
  display.display();
}
