
#include "Vibration.h"

float treshhold_forVibrate = 0.5; // 20% power is min for brrrt.
uint8_t MAXINTENSITY = 90; // max max is 255


uint8_t soll_front = 0;
uint8_t soll_left = 0;
uint8_t soll_right = 0;
uint8_t soll_back = 0;

uint8_t shaped_front = 0;
uint8_t shaped_left = 0;
uint8_t shaped_right = 0;
uint8_t shaped_back = 0;

bool debug_out = false;
bool debug_prot = true;
bool vibration_active = false;
int multiplikator = 0;
int realbigdelay = 10;

/*!
    @brief  A small helper function for error messages
*/
/**************************/
inline void error(const __FlashStringHelper*err)
{
  Serial.println(err);
  while (1);
}

unsigned long next_vibrate_time = 0;
void vibrateLoop(Measurement& sensorData) {
  unsigned long time = millis();
  if (next_vibrate_time > time) {
    return;
  }
  next_vibrate_time = time + realbigdelay;

  float winkel = sensorData.winkel;
  winkel = winkel + 180; //ausgleich der einbauorientierung
  winkel = winkel > 360 ? winkel - 360 : winkel;
  int quality = 1;
  if (quality >= 1) //wird immer erfüllt sein, da quality erst danach berechnet wird
  {
    letItVibrate(winkel, eWorking);
  }
  else
  {
    soll_front = 0;
    soll_left = 0;
    soll_right = 0;
    soll_back = 0;
  }
  quality = sensorData.gyro + sensorData.accel + sensorData.magy;
  // ehemals quality = show_calibration(1);

  if (debug_out)
  {
    // Serial.print("\tQuality: ");
    // Serial.print(quality, DEC);
    Serial.print("\tWinkel: ");
    Serial.print(winkel, 4);
    Serial.println("\n");
  }

  if (quality == 0 && winkel == 0)
  {
    Serial.print("\nFehler: ");
    //letItVibrate(0,eERROR);
  }

  multiplikator += 10;
  if (multiplikator > 1000)
  {
    multiplikator = 0;
  }

  vibration_shaper(0, multiplikator);

  commitVibrations();

}

void commitVibrations() {
  if (vibration_active)
  {
    analogWrite(FRONT, shaped_front);
    analogWrite(RECHTS, shaped_right);
    analogWrite(BACK, shaped_back);
    analogWrite(LEFT, shaped_left);

    digitalWrite(LED_FRONT, shaped_front > 0 ? LOW : HIGH);
    digitalWrite(LED_RIGHT, shaped_right > 0 ? LOW : HIGH);
    digitalWrite(LED_BACK, shaped_back > 0 ? LOW : HIGH);
    digitalWrite(LED_LEFT, shaped_left > 0 ? LOW : HIGH);
  } else {
    analogWrite(FRONT, 0);
    analogWrite(RECHTS, 0);
    analogWrite(BACK, 0);
    analogWrite(LEFT, 0);
  }
}



void theOnlyVibrate(float intensity, e_vibraMotorWhich wo) // intensity between 0 and 1 and e_vibraMotorWhich
{
  // umrechnen von 0-1 auf 0-255
  uint8_t newintensity = intensity * MAXINTENSITY;
  switch (wo)
  {
    case efront:
      soll_front = intensity > treshhold_forVibrate ? newintensity : 0;
      if (debug_out)
      {
        Serial.print("\tFront auf ");
        Serial.print(newintensity, DEC);
      }
      break;
    case eleft:
      soll_left = intensity > treshhold_forVibrate ? newintensity : 0;
      if (debug_out)
      {
        Serial.print("\tLeft auf ");
        Serial.print(newintensity, DEC);
      }
      break;
    case eright:
      soll_right = intensity > treshhold_forVibrate ? newintensity : 0;
      if (debug_out)
      {
        Serial.print("\tRight auf ");
        Serial.print(newintensity, DEC);
      }
      break;
    case eback:
      soll_back = intensity > treshhold_forVibrate ? newintensity : 0;
      if (debug_out)
      {
        Serial.print("\tBack auf ");
        Serial.print(newintensity, DEC);
      }
      break;
    default:
      break;
  }
}

void letItVibrate(float angle, e_status stat) // 0 bis 360 grad.
{
  if (stat == eWorking)
  {
    // calc power of front
    if (angle < 90)
    {
      theOnlyVibrate(1 - (angle / 90.0), efront);
      theOnlyVibrate(angle / 90.0, eright);
      theOnlyVibrate(0, eback);
      theOnlyVibrate(0, eleft);
    }
    if (angle > 90 && angle <= 180)
    {
      theOnlyVibrate(0, efront);
      theOnlyVibrate(1 - ((angle - 90) / 90.0), eright);
      theOnlyVibrate(((angle - 90.0) / 90.0), eback);
      theOnlyVibrate(0, eleft);
    }
    if (angle > 180 && angle <= 270)
    {
      theOnlyVibrate(0, efront);
      theOnlyVibrate(0, eright);
      theOnlyVibrate(1 - ((angle - 180 ) / 90.0), eback);
      theOnlyVibrate(((angle - 180) / 90.0), eleft);
    }
    if (angle > 270 && angle <= 360)
    {
      theOnlyVibrate(((angle - 270) / 90), efront);
      theOnlyVibrate(0, eright);
      theOnlyVibrate(0, eback);
      theOnlyVibrate(1 - ((angle - 270) / 90), eleft);
    }
  }
  else if (stat == eERROR)
  {
    theOnlyVibrate(0.5, efront);
    theOnlyVibrate(0.5, eback);
    theOnlyVibrate(0.5, eleft);
    theOnlyVibrate(0.5, eright);
  }
}

void ll_set_vibration(int intensity, e_vibraMotorWhich wo) //0...255
{
  switch (wo)
  {
    case efront:
      analogWrite(FRONT, intensity);
      break;
    case eleft:
      analogWrite(LEFT, intensity);
      break;
    case eright:
      analogWrite(RECHTS, intensity);
      break;
    case eback:
      analogWrite(BACK, intensity);
      break;
    default:
      break;
  }
}

void vibration_shaper(int mode, int t_step)
{
  switch (mode)
  {
    default: //lässt die sollwerte sinus-förmig schwanken. t_step wird in Grad umgerechnet
      shaped_front = soll_front * sin(t_step / 57) + soll_front / 2;
      shaped_back = soll_back * sin(t_step / 57) + soll_back / 2;
      shaped_right = soll_right * sin(t_step / 57) + soll_right / 2;
      shaped_left = soll_left * sin(t_step / 57) + soll_left / 2;
      break;
  }
}