/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"
#include <CmdMessenger.h> // CmdMessenger


#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"


#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

const int LED_FRONT = A2;
const int LED_RIGHT = A3;
const int LED_BACK = A4;
const int LED_LEFT = A5;
const int SWITCH = A1;

enum
{
  // Commands
  kAcknowledge = 0       , // Command to acknowledge that cmd was received
  kError = 1              , // Command to report errors
  kLed = 2      , // Command to request add two floats
};

CmdMessenger cmdMessengerBLE = CmdMessenger(Serial); // serial only for init
CmdMessenger cmdMessengerSerial = CmdMessenger(Serial);
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

bool waitForBleConnection = false;
bool bleConnected = false;


void error(const __FlashStringHelper*err)
{
  Serial.println(err);
  while (1);
}

void OnUnknownCommandBLE()
{
  Serial.println( F("BLE: unknown Cmd") );
  // cmdMessengerBLE.sendCmd(kError,"Command without attached callback");
}

void OnUnknownCommandSerial()
{
  Serial.println( F("Serial: unknown Cmd") );
  // cmdMessengerBLE.sendCmd(kError,"Command without attached callback");
}


void OnLedCommand(CmdMessenger& cmdMessenger)
{
  Serial.println( F("Led Cmd") );
  if (cmdMessenger.readBoolArg()) {
      digitalWrite(LED_FRONT, LOW);
  } else {
      digitalWrite(LED_FRONT, HIGH);
  }
  if (cmdMessenger.readBoolArg()) {
      digitalWrite(LED_RIGHT, LOW);
  } else {
      digitalWrite(LED_RIGHT, HIGH);
  }
  if (cmdMessenger.readBoolArg()) {
      digitalWrite(LED_BACK, LOW);
  } else {
      digitalWrite(LED_BACK, HIGH);
  }
  if (cmdMessenger.readBoolArg()) {
      digitalWrite(LED_LEFT, LOW);
  } else {
      digitalWrite(LED_LEFT, HIGH);
  }
}

void OnLedCommandBLE() { OnLedCommand(cmdMessengerBLE); }
void OnLedCommandSerial() { OnLedCommand(cmdMessengerSerial); }


void attachCommandCallbacksBLE()
{
  // Attach callback methods
  cmdMessengerBLE.attach(OnUnknownCommandBLE);
  cmdMessengerBLE.attach(kLed, OnLedCommandBLE);
}

void attachCommandCallbacksSerial()
{
  // Attach callback methods
  cmdMessengerSerial.attach(OnUnknownCommandSerial);
  cmdMessengerSerial.attach(kLed, OnLedCommandSerial);
}

void initBluetooth() {
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  // Perform a factory reset to make sure everything is in a known state
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
    error(F("Couldn't factory reset"));
  }

  // Disable command echo from Bluefruit
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  // Print Bluefruit information
  ble.info();

  ble.verbose(false);  // debug info is a little annoying after this point!

  waitForBleConnection = true;
}

void bluetoothLoop() {
  if (waitForBleConnection) {
    /* Wait for connection */
    if (! ble.isConnected()) {
        return;
    };

    // Set module to DATA mode
    Serial.println( F("Got Connection. Switching to DATA mode!") );
    ble.setMode(BLUEFRUIT_MODE_DATA);

    cmdMessengerBLE = CmdMessenger(ble);
    attachCommandCallbacksBLE();

    bleConnected = true;
    waitForBleConnection = false;
  }

  if (bleConnected) {
    size_t bytesAvailable = min(ble.available(), 64);
    if (bytesAvailable > 0) {
      Serial.print("A:");
      Serial.println(bytesAvailable);
    }
    cmdMessengerBLE.feedinSerialData();
  }

  // test
  //size_t bytesAvailable = min(ble.available(), 100-1);
  //ble.readBytes(streamBuffer, bytesAvailable);
  //Serial.write(streamBuffer, bytesAvailable);
  //Serial.flush();


}


void setup()
{
  Serial.begin(9600);

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(LED_FRONT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_BACK, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(SWITCH, INPUT_PULLUP);
  analogWrite(5, 0);
  analogWrite(6, 0);
  analogWrite(11, 0);
  analogWrite(10, 0);
  digitalWrite(LED_FRONT, LOW);
  digitalWrite(LED_RIGHT, LOW);
  digitalWrite(LED_BACK, LOW);
  digitalWrite(LED_LEFT, LOW);

  delay(2000);
  Serial.println("Start.");

  initBluetooth();

  attachCommandCallbacksSerial();
}


  char streamBuffer[100]; // Buffer that holds the data
void loop()
{
  bluetoothLoop();

  cmdMessengerSerial.feedinSerialData();
}