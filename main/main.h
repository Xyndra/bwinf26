#pragma once

#include <Display.h>
#include <GxEPD2_BW.h>

extern GxEPD2_BW<WatchyDisplay, WatchyDisplay::HEIGHT> display;

extern bool motor_on;  // true while driving is active

extern int car_x;  // car position in map coords (0-100, origin = top-left)
extern int car_y;
extern int car_angle;
