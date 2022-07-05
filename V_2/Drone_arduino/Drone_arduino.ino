/**
 * Author: Camner Armand-Gabriel
 * Version: 1.0
 * Disertatie
 * Drone component for controll and command
 *          2022
 */


/**
 * STEPS:
 * 1. Communication between the arduino and the jetson nano
 *      -- DETECTION OF I2C network
 *      -- 0x60  -> Crypto Chip
 *      -- 0x6A  -> Jetson Nano
 *      -> Send commands to the Jetson Nano 
 *      -> Receives commands from the Jetson Nano
 *      
 *      
 *      
 *      
 */
#include <Wire.h>               // Library used for i2c communication


// *********************************************** //
// type           FIELD NAME                 VAL   //
int               i2cAddress         =       0x6a;
typedef struct {
  int32_t         command;
  int32_t         ack_Signature;                   // 0x1C                 
} JETSON_COMMANDS;

JETSON_COMMANDS SEND_CMD;
JETSON_COMMANDS RECEIVE_RESPONSE_CMD;
/**
 * JETSON COMMANDS
 * 1.         0x01          
 * 2.         0x02
 * 3.         0x03
 * 4.         0x04
 * 5.         0x05
 * 6.         0x06
 * 7.         0x07
 * 8.         0x08
 * 9.         0x10
 */



void setup() {
  // Setup the Serial
  Serial.begin(9600);
  // Caution, on headless mode there is no Serial
  while(!Serial) { ; }
  
  // *********************************************** //
  // Serial communication setup
  Wire.begin(i2cAddress);
  Wire.onReceive(receiveJetsonEvent);                // The Jetson will send data.
  Wire.onRequest(sendJetsonEvent);                   // The Arduino will send data.
}

void loop() {
  // put your main code here, to run repeatedly:
  SEND_CMD.command = 3;
  SEND_CMD.ack_Signature = 12;
  sendJetsonEvent();
  Serial.println("SEND COMMAND");
  delay(1000);
}


// *********************************************** //
//              FUNCTIONS                          //
//            1. receiveJetsonEvent()              //
void receiveJetsonEvent(int bytes) {
  Serial.println("Received event from Jetson Nano");
  Serial.println("Request size: ");
  Serial.print(bytes);
  Serial.println();
  Wire.readBytes((byte*) &RECEIVE_RESPONSE_CMD, sizeof RECEIVE_RESPONSE_CMD);
  Serial.println(RECEIVE_RESPONSE_CMD.command);
  Serial.println(RECEIVE_RESPONSE_CMD.ack_Signature);
  Serial.println("*******************************");
}
//            2. sendJetsonEvent()                 //
void sendJetsonEvent() {
  Serial.println("SENDING TO JETSON DATA:");
  Serial.println(SEND_CMD.command);
  Serial.println(SEND_CMD.ack_Signature);
  Serial.println("SIZE OF SEND_CMD: ");
  Serial.print(sizeof SEND_CMD);
  Serial.println();
  byte data[] = {0x00, 0x03};
  int count = Wire.write((byte*)& data, sizeof(data));
  Serial.println("Received Sent Count: ");
  Serial.print(count);
  Serial.println();
}
