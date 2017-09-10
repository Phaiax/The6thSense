#pragma once

#include "Arduino.h"
#include "Constants.h"

extern float treshhold_forVibrate; // 20% power is min for brrrt.
extern uint8_t MAXINTENSITY; // max max is 255


extern uint8_t soll_front;
extern uint8_t soll_left;
extern uint8_t soll_right;
extern uint8_t soll_back;

extern uint8_t shaped_front;
extern uint8_t shaped_left;
extern uint8_t shaped_right;
extern uint8_t shaped_back;

extern bool debug_out;
extern bool debug_prot;
extern bool vibration_active;
extern int multiplikator;
extern int realbigdelay;

enum e_vibraMotorWhich
{
  efront, eleft, eright, eback
};
enum e_status
{
  eERROR, eWorking
};

struct Measurement {
    uint8_t system, gyro, accel, magy;
    float winkel;
    float w2, w3;
};

inline void bt_set_vibration_active(bool enable)
{
  vibration_active = enable;
}
inline void bt_vibration_active_toggle()
{
  vibration_active = !vibration_active;
}
inline void bt_set_maxintensity(int tomax255)
{
  if (tomax255 > 255) {
    tomax255 = 0;
  }
  MAXINTENSITY = tomax255;
}
inline void bt_set_treshhold_forVibrate(float tomax1)
{
  if (tomax1 > 1.0) {
    tomax1 = 0;
  }
  treshhold_forVibrate = tomax1;
}
inline void bt_set_realbigdelay(int one_sec_is1k)
{
  realbigdelay = one_sec_is1k;
}

void vibrateLoop(Measurement& sensorData);
void commitVibrations();

int show_calibration(bool print_it);

void ll_set_vibration(int intensity, e_vibraMotorWhich wo); //0...255
void letItVibrate(float angle, e_status stat); // 0 bis 360 grad.
void theOnlyVibrate(float intensity, e_vibraMotorWhich wo); // intensity between 0 and 1 and e_vibraMotorWhich
void vibration_shaper(int mode, int t_step);
