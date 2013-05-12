// -*- C++ -*-
/*
 * LeafCAN Firmware
 *
 * Copyright (c) 2012-2013 Sam C. Lin <lincomatic@gmail.com>
 * Maintainer: SCL

 * This file is part of LeafCAN

 * LeafCAN is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.

 * LeafCAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with LeafCAN; see the file COPYING.  If not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
#ifndef _LEAFCAN_H_
#define _LEAFCAN_H_

#define VER_STR "v2.0B0"

//
// configuration
//

// support Adafruit 16x2 OLED display http://www.adafruit.com/products/823
// requires Adafruit's library: https://github.com/ladyada/Adafruit_CharacterOLED
// due to a bug in Arduino, you must uncomment
//  //#include <Adafruit_CharacterOLED.h>
// in LeafCAN.pde if ADA_OLED is defined
#define ADA_OLED

// Barbouri's V2oled3 board with RGB LED encoder
// comment this out for V2 hardware
#define V2O3
//#define BREADBOARD // don't use - lincomatic's test rig

#ifdef BREADBOARD
#define I2CLCD // adafruit I2C LCD backpack
#endif

//#define SERIAL // serial logging

#ifndef ADA_OLED
#define BACKLIGHT_PIN 15 // PB7
#define CONTRAST_PIN 36 // PE4
#endif 

#ifdef V2O3
#define INVERT_ENC_BTN
#define GLED_PIN 19 // PC3
#define BLED_PIN 20 // PC4
#define RLED_PIN 21 // PC5
#endif

#define BACKLIGHT_TIMEOUT 5000 // turn off backlight after CAN bus idle for (ms)
#define SERIAL_BAUD 115200
#define LCD_UPDATE_MS 250 // update interval for LCD in ms

// the Leaf's fuel bar display isn't tied to SOC, which I find very confusing,
// because this means that as the battery ages or temperature changes, the bars
// correspond to less
// stored KWh in the battery.  When FIXED_FUEL_BARS is defined, instead of
// displaying the same fuel bars as on the dash, the fuel display is directly
// tied to Gids. My formula arbitrarily assigns 13 = 100% charge = 281 gids,
// and the transition from 1 -> 0 is at 24 gids, which is the transition to
// Very Low Battery.  When the battery gets this low, it's kind of useless to
// use something as coarse as fuel bars, so when it displays 0, it's time to
// look directly at the raw Gid/SOC values.
// the fuel bars are displayed w/ 1 decimal place (e.g 9.2) so you always know
// how "full" the current bar is
//n.b. FIXED_FUEL_BARS is no longer a #define. It's always enabled
//     but m_GOMFuelBars is available for use in Screen() functions

/*
 rotary encoder configuration.
 This code only works if ENC_A and ENC_B are connected to pins 0,1 of the selected port
 if they are not pins 0,1 the bits need to be shifted to bits 0 and 1 in the
 line
   old_AB |= ( ENC_PIN & 0x03 );  //add current state
 */
#define ENC_BIT_A   (1 << 0)
#define ENC_BIT_B   (1 << 1)
#define ENC_BIT_BTN (1 << 2)
#if defined(BREADBOARD)||defined(V2O3)
#define ENC_DDR DDRC
#define ENC_PIN PINC
#define ENC_PORT PORTC
#else
#define ENC_DDR DDRA
#define ENC_PIN PINA
#define ENC_PORT PORTA
#endif

// for DTE screen
#define DEF_DPKWH10_LOW 20 // default lowest dist/kwh*10 for DTE
#define DEF_DPKWH10_INCR 10 // default dist/kwh*10 increment for DTE

#define SCNIDX_INFO 0
#define SCNIDX_DTE  1
#define SCNIDX_VA   2
#define SCNIDX_BATT_TEMP 3
#define SCNIDX_CP_VOLT 4
#define SCNIDX_SOC_CAP 5
#define SCREEN_CNT 6

#define CAN_REQ_INTERVAL 1000 // # ms between active CAN requests
#define REQ_INTERVAL_7BB 20 // # ms between 7BB requests

#define CAN_TIMEOUT 500 // ms - assume CAN bus is off when exceeded

#define DEFAULT_TEMP_UNIT 'C'
#define DEFAULT_DTE_TYPE  'V'

//
// constants
//
#define KW_FACTOR 7473L // 80 is from ingineer, surfingslovak prefers 74.73
#define GIDS_LB  49 // low batt
#define GIDS_VLB 24 // very low batt
#define GIDS_TURTLE 6 // really varies from 4-6 ...

// EEPROM offsets
#define EOFS_TEMP_UNIT 0 // C/F
#define EOFS_DTE_TYPE  1 // L/V/T



//
// includes
//

#if ARDUINO >= 100 
   #include "Arduino.h"
#else
   #include "WProgram.h"
#endif

#include "RotaryEncoder.h"
#include "CanBusInterface.h"
#include "LeafCanData.h"

//
// globals
//
extern LeafCanData g_LeafCanData;
extern CanBusInterface g_CanBus;

#ifdef BACKLIGHT_PIN
extern uint8_t g_Brightness;

//0=off 255=max bright
void setBackLight(uint8_t brightness);
#endif // BACKLIGHT_PIN



#endif // _LEAFCAN_H_
