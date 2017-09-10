#include "Arduino.h"

#include <CmdMessenger.h> // CmdMessenger

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>

#include "BluefruitConfig.h"
#include "Constants.h"
#include "Vibration.h"

// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// COMMUNICATION STUFF

enum
{
  // Commands
  kAcknowledge = 0       , // Command to acknowledge that cmd was received
  kError = 1              , // Command to report errors
  kLed = 2,
  kShowCalib = 3,
  kShowCalibResult = 30,
  kSetVibrationEnable = 4,
  kSetMaxIntensity = 5,
  kSetThresholdForVibrate = 6,
  kSetVibrationLoopTime = 7,
  kLlSetVibration = 8,
};

CmdMessenger cmdMessengerBLE = CmdMessenger(Serial); // serial only for init
CmdMessenger cmdMessengerSerial = CmdMessenger(Serial);
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

bool waitForBleConnection = false;
bool bleConnected = false;

// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// SENSOR STUFF
Measurement sensorData;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
unsigned long time;

// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// FATAL ERROR HANDLING

inline void error(const __FlashStringHelper*err)
{
  Serial.println(err);
  while (1);
}

// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// COMMAND HANDLER

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

uint8_t bool2level(bool b) {
  return b ? LOW : HIGH;
}

void OnLedCommand(CmdMessenger& cmdMessenger)
{
  digitalWrite(LED_FRONT, bool2level(cmdMessenger.readBoolArg()));
  digitalWrite(LED_RIGHT, bool2level(cmdMessenger.readBoolArg()));
  digitalWrite(LED_BACK, bool2level(cmdMessenger.readBoolArg()));
  digitalWrite(LED_LEFT, bool2level(cmdMessenger.readBoolArg()));
}

void OnShowCalib(CmdMessenger& cmdMessenger) {
  cmdMessenger.sendCmdStart((uint8_t) kShowCalibResult);
  cmdMessenger.sendCmdArg(sensorData.winkel);
  cmdMessenger.sendCmdArg(sensorData.system);
  cmdMessenger.sendCmdArg(sensorData.gyro);
  cmdMessenger.sendCmdArg(sensorData.accel);
  cmdMessenger.sendCmdArg(sensorData.magy);
  cmdMessenger.sendCmdEnd();
}
void OnSetVibrationEnable(CmdMessenger& cmdMessenger) {
  bt_set_vibration_active(cmdMessenger.readBoolArg());
}
void OnSetMaxIntensity(CmdMessenger& cmdMessenger) {
  bt_set_maxintensity(cmdMessenger.readInt32Arg());
}
void OnSetThresholdForVibrate(CmdMessenger& cmdMessenger) {
  bt_set_treshhold_forVibrate(cmdMessenger.readFloatArg());
}
void OnSetVibrationLoopTime(CmdMessenger& cmdMessenger) {
  bt_set_realbigdelay(cmdMessenger.readInt32Arg());
}
void OnLlSetVibration(CmdMessenger& cmdMessenger) {
  ll_set_vibration(cmdMessenger.readFloatArg(), efront);
  ll_set_vibration(cmdMessenger.readFloatArg(), eright);
  ll_set_vibration(cmdMessenger.readFloatArg(), eback);
  ll_set_vibration(cmdMessenger.readFloatArg(), eleft);
}

void OnLedCommandBLE() { OnLedCommand(cmdMessengerBLE); }
void OnLedCommandSerial() { OnLedCommand(cmdMessengerSerial); }
void OnShowCalibBLE() { OnShowCalib(cmdMessengerBLE); }
void OnShowCalibSerial() { OnShowCalib(cmdMessengerSerial); }
void OnSetVibrationEnableBLE() { OnSetVibrationEnable(cmdMessengerBLE); }
void OnSetVibrationEnableSerial() { OnSetVibrationEnable(cmdMessengerSerial); }
void OnSetMaxIntensityBLE() { OnSetMaxIntensity(cmdMessengerBLE); }
void OnSetMaxIntensitySerial() { OnSetMaxIntensity(cmdMessengerSerial); }
void OnSetThresholdForVibrateBLE() { OnSetThresholdForVibrate(cmdMessengerBLE); }
void OnSetThresholdForVibrateSerial() { OnSetThresholdForVibrate(cmdMessengerSerial); }
void OnSetVibrationLoopTimeBLE() { OnSetVibrationLoopTime(cmdMessengerBLE); }
void OnSetVibrationLoopTimeSerial() { OnSetVibrationLoopTime(cmdMessengerSerial); }
void OnLlSetVibrationBLE() { OnLlSetVibration(cmdMessengerBLE); }
void OnLlSetVibrationSerial() { OnLlSetVibration(cmdMessengerSerial); }

void attachCommandCallbacksBLE()
{
  // Attach callback methods
  cmdMessengerBLE.attach(OnUnknownCommandBLE);
  cmdMessengerBLE.attach(kLed, OnLedCommandBLE);
  cmdMessengerBLE.attach(kShowCalib, OnShowCalibBLE);
  cmdMessengerBLE.attach(kSetVibrationEnable, OnSetVibrationEnableBLE);
  cmdMessengerBLE.attach(kSetMaxIntensity, OnSetMaxIntensityBLE);
  cmdMessengerBLE.attach(kSetThresholdForVibrate, OnSetThresholdForVibrateBLE);
  cmdMessengerBLE.attach(kSetVibrationLoopTime, OnSetVibrationLoopTimeBLE);
  cmdMessengerBLE.attach(kLlSetVibration, OnLlSetVibrationBLE);
}

void attachCommandCallbacksSerial()
{
  // Attach callback methods
  cmdMessengerSerial.attach(OnUnknownCommandSerial);
  cmdMessengerSerial.attach(kLed, OnLedCommandSerial);
  cmdMessengerSerial.attach(kShowCalib, OnShowCalibSerial);
  cmdMessengerSerial.attach(kSetVibrationEnable, OnSetVibrationEnableSerial);
  cmdMessengerSerial.attach(kSetMaxIntensity, OnSetMaxIntensitySerial);
  cmdMessengerSerial.attach(kSetThresholdForVibrate, OnSetThresholdForVibrateSerial);
  cmdMessengerSerial.attach(kSetVibrationLoopTime, OnSetVibrationLoopTimeSerial);
  cmdMessengerSerial.attach(kLlSetVibration, OnLlSetVibrationSerial);
}

// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// BLUETOOTH COMMUNICATION

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

  // For testing
  // static char streamBuffer[100]; // Buffer that holds the data
  //size_t bytesAvailable = min(ble.available(), 100-1);
  //ble.readBytes(streamBuffer, bytesAvailable);
  //Serial.write(streamBuffer, bytesAvailable);
  //Serial.flush();
}

// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// SENSOR / MEASUREMENTS

void initSensor() {
  if (!bno.begin((Adafruit_BNO055::adafruit_bno055_opmode_t)0X09)) //Compass=0X09 //NDOF=0X0C
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
  Serial.println("Sensor initialized.");
}



unsigned long next_measure_time = 0;
void measureLoop() {
  if (next_measure_time > time) {
    return;
  }
  next_measure_time = time + BNO055_SAMPLERATE_DELAY_MS;

  sensors_event_t event;
  bno.getEvent(&event);
  sensorData.winkel = event.orientation.x;

  bno.getCalibration(&sensorData.system, &sensorData.gyro, &sensorData.accel, &sensorData.magy);
}

int show_calibration(bool print_it)
{
  uint8_t system, gyro, accel, magy;
  system = gyro = accel = magy = 0;
  bno.getCalibration(&system, &gyro, &accel, &magy);

  if (print_it && debug_out)
  {
    Serial.print("\tCalibration: G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(magy, DEC);
    Serial.print("");
  }
  return gyro + accel + magy;
}


// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// SWITCH

unsigned long next_checkswitch_time = 0;
void checkSwitch() {
  if (next_checkswitch_time > time) {
    return;
  }
  bool val = digitalRead(SWITCH);
  if (!val) {
    bt_vibration_active_toggle();
    Serial.println("Switch pressed!");
    next_checkswitch_time = time + 500; // denoise
  }
}

// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// SETUP AND LOOP

void setup()
{
  Serial.begin(9600);

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

  initSensor();

  initBluetooth();

  attachCommandCallbacksSerial();
}


void loop()
{
  time = millis();
  measureLoop();
  vibrateLoop(sensorData);
  checkSwitch();

  bluetoothLoop();
  cmdMessengerSerial.feedinSerialData();
}