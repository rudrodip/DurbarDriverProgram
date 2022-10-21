#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
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
#define sonar_servo 23

// sonar sensor pin
#define sonar_trig 4
#define sonar_echo 2

// esp32 to arduino serial communication pin
#define rx 16
#define tx 17

Motor motor1 = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, 1, STBY);

ServoEasing SonarServo;
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
    }
    else if (dir == 'b')
    { // direction == b then backward
      back(motor1, motor2);
      Serial.println("backwarding...");
    }
    else if (dir == 'r')
    { // direction == r then right
      right(motor1, motor2, turningSpeed);
      Serial.println("turning right...");
    }
    else if (dir == 'l')
    { // direction == l then left
      left(motor1, motor2, turningSpeed);
      Serial.println("turning left...");
    }
    else if (dir == 's')
    { // direction == s then stop
      brake(motor1, motor2);
      Serial.println("stopping...");
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
      Serial.println("moving base servo to" + pos);
      break;
    case 2:
      SHOULDER.startEaseTo(pos, 40);
      Serial.println("moving shoulder servo to" + pos);
      break;
    case 3:
      ELBOW.startEaseTo(pos, 40);
      Serial.println("moving elbow servo to" + pos);
      break;
    case 4:
      WRIST_PITCH.startEaseTo(pos, 40);
      Serial.println("moving wrist pitch servo to" + pos);
      break;
    case 5:
      WRIST_ROLL.startEaseTo(pos, 40);
      Serial.println("moving wrist roll servo to" + pos);
      break;
    case 6:
      GRIPPER.startEaseTo(pos, 40);
      Serial.println("moving gripper servo to" + pos);
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
  };

  void onDisconnect(BLEServer *pServer)
  {
    Serial.println("Disconnected");
  }
};

class CharacteristicsCallbacks : public BLECharacteristicCallbacks
{
  // on onWrite function, we're handling the command
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    String message = pCharacteristic->getValue().c_str();
    Serial.println(message);
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

  Serial.println("Initiated connection successfully :)");

  BASE.attach(base, 90);
  SHOULDER.attach(shoulder, 180);
  ELBOW.attach(elbow, 170);
  WRIST_PITCH.attach(wrist_pitch, 90);
  WRIST_ROLL.attach(wrist_roll, 90);
  GRIPPER.attach(gripper, 0);
  SonarServo.attach(sonar_servo, 90);

  BASE.setEasingType(EASE_QUADRATIC_IN_OUT);
  SHOULDER.setEasingType(EASE_QUADRATIC_IN_OUT);
  WRIST_PITCH.setEasingType(EASE_QUADRATIC_IN_OUT);
  WRIST_ROLL.setEasingType(EASE_QUADRATIC_IN_OUT);
}

void loop()
{
  if (Serial2.available()){
    String reading = Serial2.readString(); // reading from serial2
    char buf[50];
    reading.toCharArray(buf, reading.length());  // converting string to char array
    message_characteristic->setValue(buf); // sending over ble
  }
  if (Serial1.available()){
    String reading = Serial2.readString(); // reading from serial2
    char buf[50];
    reading.toCharArray(buf, reading.length());  // converting string to char array
    message_characteristic->setValue(buf); // sending over ble
  }
  message_characteristic->notify(); // its to receive message
  delay(100);
}