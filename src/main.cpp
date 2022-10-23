#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include "utils.h"
#include <SparkFun_TB6612.h>
#include "ServoEasing.hpp"

// BLE SECTION
BLEServer *pServer = NULL;

BLECharacteristic *message_characteristic = NULL;
BLECharacteristic *box_characteristic = NULL;

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define MESSAGE_CHARACTERISTIC_UUID "6d68efe5-04b6-4a85-abc4-c2670b7bf7fd"

// motor driver pins
#define AIN1 25
#define BIN1 27

#define AIN2 33
#define BIN2 14
#define PWMA 32
#define PWMB 13
#define STBY 26

// motor speed
#define turningSpeed 200

// servo pins
#define base 23
#define shoulder 22
#define elbow 21
#define wrist_pitch 19
#define wrist_roll 18
#define gripper 5
#define sonar_servo 4

// esp32 to arduino serial communication pin
#define rx 16
#define tx 17

int sonarServoPosition = 60;
#define servoDelayPerDegRotation 15

Motor motor1 = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, 1, STBY);
Servo SonarServo;

ServoEasing BASE;
ServoEasing SHOULDER;
ServoEasing ELBOW;
ServoEasing WRIST_PITCH;
ServoEasing WRIST_ROLL;
ServoEasing GRIPPER;

void handle_command(String cmd)
{
  char start = cmd.charAt(0);
  if (start == 'm')
  {
    // getting direction, motor, pwm from cmd
    char dir = cmd.charAt(1);
    char motor = cmd.charAt(2);

    if (dir == 'f')
    { // direction == f then forward
      forward(motor1, motor2);
      Serial.println("forwarding...");
      Serial2.println("forwarding...");
    }
    else if (dir == 'b')
    { // direction == b then backward
      back(motor1, motor2);
      Serial.println("backwarding...");
      Serial2.println("backwarding...");
    }
    else if (dir == 'r')
    { // direction == r then right
      right(motor1, motor2, turningSpeed);
      Serial.println("turning right...");
      Serial2.println("turning right...");
    }
    else if (dir == 'l')
    { // direction == l then left
      left(motor1, motor2, turningSpeed);
      Serial.println("turning left...");
      Serial2.println("turning left...");
    }
    else if (dir == 's')
    { // direction == s then stop
      brake(motor1, motor2);
      Serial.println("stopping...");
      Serial2.println("stopping...");
    }
    else if (dir == 'c')
    {                                     // direction == c then custom pwm
      int pwm = cmd.substring(3).toInt(); // getting pwm
      Serial.println(pwm);
      if (motor == '1')
      {
        motor1.drive(pwm);
      }
      else
      {
        motor2.drive(pwm);
      }
    }
  }
  else if (start == 's')
  {
    int servoNum = cmd.charAt(1) - '0'; // converting char to int
    int pos = cmd.substring(2).toInt();
    switch (servoNum)
    {
    case 1:
      BASE.startEaseTo(pos, 40);
      Serial.println("moving base servo");
      Serial2.println("moving base servo");
      break;
    case 2:
      SHOULDER.startEaseTo(pos, 40);
      Serial.println("moving shoulder servo");
      Serial2.println("moving shoulder servo");
      break;
    case 3:
      ELBOW.startEaseTo(pos, 60);
      Serial.println("moving elbow servo");
      Serial2.println("moving elbow servo");
      break;
    case 4:
      WRIST_PITCH.startEaseTo(pos, 60);
      Serial.println("moving wrist pitch servo");
      Serial2.println("moving wrist pitch servo");
      break;
    case 5:
      WRIST_ROLL.startEaseTo(pos, 60);
      Serial.println("moving wrist roll servo");
      Serial2.println("moving wrist roll servo");
      break;
    case 6:
      GRIPPER.startEaseTo(pos, 80);
      Serial.println("moving gripper servo");
      Serial2.println("moving gripper servo");
      break;
    default:
      break;
    }
  }
  else
  {
    Serial.println(cmd);
  }
}

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("Connected");
    Serial2.println("Connected");
  };

  void onDisconnect(BLEServer *pServer)
  {
    Serial.println("Disconnected");
    Serial2.println("Disconnected");
  }
};

class CharacteristicsCallbacks : public BLECharacteristicCallbacks
{
  // on onWrite function, we're handling the command
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    String message = pCharacteristic->getValue().c_str();
    Serial.println(message);
    Serial2.println(message);
    handle_command(message);
  }
};

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, rx, tx);

  // Create the BLE Device
  BLEDevice::init("Durbar");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  delay(100);

  // Create a BLE Characteristic
  message_characteristic = pService->createCharacteristic(
      MESSAGE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);

  // Start the BLE service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();

  message_characteristic->setValue("Connection Established");
  message_characteristic->setCallbacks(new CharacteristicsCallbacks());

  SonarServo.attach(sonar_servo);
  BASE.attach(base, 90);
  SHOULDER.attach(shoulder, 0);
  ELBOW.attach(elbow, 0);
  WRIST_PITCH.attach(wrist_pitch, 90);
  WRIST_ROLL.attach(wrist_roll, 90);
  GRIPPER.attach(gripper, 0);

  BASE.setEasingType(EASE_QUADRATIC_IN_OUT);
  SHOULDER.setEasingType(EASE_QUADRATIC_IN_OUT);

  Serial.println("Initiated connection successfully :)");
}
unsigned long previousTime = 0;

void loop()
{
  if (Serial2.available())
  {
    String reading = Serial2.readString(); // reading from serial2
    char buf[50];
    reading.toCharArray(buf, reading.length()); // converting string to char array
    message_characteristic->setValue(buf);      // sending over ble
  }
  message_characteristic->notify(); // its to receive message

  // float dis = distance();
  // String distance = "d" + String(dis) + "s" + String();
  // char distanceBuf[6];
  // distance.toCharArray(distanceBuf, distance.length());
  // message_characteristic->setValue(distanceBuf);
  // delay(100);

  unsigned long currentTime = millis();
  if (currentTime - previousTime >= servoDelayPerDegRotation)
  {
    SonarServo.write(sonarServoPosition);
    if (sonarServoPosition < 120)
    {
      sonarServoPosition++;
    }
    else if (sonarServoPosition > 60)
    {
      sonarServoPosition--;
    }
    previousTime = currentTime;
    float dis = distance();
    String distance = "d" + String(dis) + "s" + String(sonarServoPosition);
    char distanceBuf[20];
    distance.toCharArray(distanceBuf, distance.length());
    message_characteristic->setValue(distanceBuf);
    Serial2.println(distance);
  }
}