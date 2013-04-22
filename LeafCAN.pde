// -*- C++ -*-
/*
2.0 changes
 -> reduce usage of floats as much as possible, since no FPU

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
#include "LeafCAN.h"

//
// globals
//

LeafCanData g_LeafCanData;
CanBusInterface g_CanBus;

#ifdef ADA_OLED
// rs/rw/enable/d4/d5/d6/d7
//#include <Adafruit_CharacterOLED.h>
Adafruit_CharacterOLED lcd(37,39,38,11,12,13,14); // PE5/PE7/PE6/PB3/PB4/PB5/PB6
#else
#include <LiquidCrystal.h>
#ifdef V2
LiquidCrystal g_Lcd(37,38,11,12,13,14); // PE5/PE6/PB3/PB4/PB5/PB6
#else 
LiquidCrystal g_Lcd(21,20,16,17,18,19); // PC5/PC4/PC0/PC1/PC2/PC3
#endif
//LiquidCrystal g_Lcd(37,38,11,12,13,14); // PE5/PE6/PB3/PB4/PB5/PB6
#endif // ADA_OLED

uint8_t g_LogEnabled = 0;
uint8_t g_LogOnly = 0;
uint8_t g_LcdEnabled = 1;

#ifdef BACKLIGHT_PIN
uint8_t g_Brightness = 0;
#endif // BACKLIGHT_PIN

unsigned long g_LastScreenUpdateMs;

#include "RotaryEncoder.h"
RotaryEncoder g_RotEnc;
//encoder stuff
int8_t lastbtnstate = ENC_BIT_BTN; // 0=pressed
int8_t lastcount = 0;


// print 2-digit number 100 = A0, 113 = B3, etc
char *SPrintDist(int dist,char *buf)
{
  int i=dist/10;
  if (i == 0) {
    buf[0] = ' ';
  }
  else if (i > 9) {
    buf[0] = 'A' + (i-10);
  }
  else {
    buf[0] = i + '0';
  }
  buf[1] = (dist % 10) + '0';
  buf[2] = 0;
  return buf;
}

// distPerKwhx10 = dist/KWh * 10
// returns distance to whMin
int DistRem(int32_t whRem,int32_t whMin,int distPerKwhx10)
{
  return (int)((((whRem-whMin)*((int32_t)distPerKwhx10))+5000L)/10000);
}


// KWh gids fixedbars
// volts soc kw
uint8_t Screen0()
{
  uint8_t rc = 1;
  char sf1[10],sf2[10],sf3[10];
  char line[17];
  int w,f;

  // top line: KWh gids fixedbars
  if (g_LeafCanData.DirtyBitsSet(DBF_WH_REMAINING |DBF_GIDS|DBF_FIXED_FUEL_BARS)) {
    rc = 0;
    g_LeafCanData.ClearDirtyBits(DBF_WH_REMAINING |DBF_GIDS|DBF_FIXED_FUEL_BARS);

    int32_t wh = g_LeafCanData.m_Wh;
    if (wh >= 10000) {
      // nn.n
      wh += 50;
      sprintf(sf1,"%2d.%d",(int)(wh/1000),(int)((wh%1000)/100));
    }
    else if (wh >= 1000) {
      // n.nn
      wh += 5;
      sprintf(sf1,"%d.%02d",(int)(wh/1000),(int)((wh%1000)/10));
    }
    else {
      // .nnn
      sprintf(sf1,".%03d",(int)(wh % 1000));
    }

    // nn.n
    sprintf(sf2,"%2d.%d",g_LeafCanData.m_FixedFuelBars/10,g_LeafCanData.m_FixedFuelBars%10);
    sprintf(line,"%s  %3d   %s",sf1,g_LeafCanData.m_Gids,sf2);

    g_Lcd.setCursor(0,0);
    g_Lcd.print(line);
  }
  // bottom line: volts soc kw
  if (g_LeafCanData.DirtyBitsSet(DBF_PACK_VOLTS|DBF_SOC|DBF_WATTS)) {
    rc = 0;
    g_LeafCanData.ClearDirtyBits(DBF_PACK_VOLTS|DBF_SOC|DBF_WATTS);

    // nnn.n
    sprintf(sf1,"%3d.%c",g_LeafCanData.m_PackVolts/2,(g_LeafCanData.m_PackVolts & 1) ? '5' : '0');
    // nnn.n
    sprintf(sf2,"%3d.%d",g_LeafCanData.m_SOC/10,g_LeafCanData.m_SOC%10);

    int32_t w = g_LeafCanData.m_W;
    if (w >= 10000) {
      // nn.n
      w += 50;
      sprintf(sf3,"%2d.%d",(int)(w/1000),(int)((w%1000)/100));
    }
    else if (w <= -10000) {
      w -= 500;
      // b-nn
      sprintf(sf3,"%4d",(int)(w/1000));
    }
    else if (w < 0) {
      // -n.n
      w -= 50;
      int wd1000 = w/1000;
      if (wd1000 == 0) {
	sprintf(sf3,"-0.%d",-(int)((w%1000)/100));
      }
      else {
	sprintf(sf3,"%2d.%d",wd1000,-(int)((w%1000)/100));
      }
    }
    else {
      // n.nn
      w += 5;
      sprintf(sf3,"%d.%02d",(int)(w/1000),(int)((w%1000)/10));
    }


    sprintf(line,"%s %s %s",sf1,sf2,sf3);
    g_Lcd.setCursor(0,1);
    g_Lcd.print(line);
  }

  return rc;
}

// DTE screen
// DTEtype dist/kwh dist/kwh dist/kwh dist/kwh dist/kwh
// units   dist     dist     dist     dist     dist
//
// DTEtype: = 'L' = low batt, 'V' = very low batt, 'T' = turtle
// units: 'M' = mi, 'K' = km
uint8_t Screen1()
{
  if (g_LeafCanData.DirtyBitsSet(DBF_WH_REMAINING)) {
    g_LeafCanData.ClearDirtyBits(DBF_WH_REMAINING);

    int32_t whmin;
    if (g_LeafCanData.m_CurDteType == 'L') whmin = g_LeafCanData.m_WhL;
    else if (g_LeafCanData.m_CurDteType == 'V') whmin = g_LeafCanData.m_WhV;
    else whmin = g_LeafCanData.m_WhT; 
    int32_t wh = g_LeafCanData.m_Wh;
    int32_t dpkw100 = g_LeafCanData.m_DpKWh10_Low;
    int32_t dpkw101 = dpkw100 + g_LeafCanData.m_DpKWh10_Incr;
    int32_t dpkw102 = dpkw101 + g_LeafCanData.m_DpKWh10_Incr;
    int32_t dpkw103 = dpkw102 + g_LeafCanData.m_DpKWh10_Incr;
    int32_t dpkw104 = dpkw103 + g_LeafCanData.m_DpKWh10_Incr;

    char distrem0[3],distrem1[3],distrem2[3],distrem3[3],distrem4[3];
    SPrintDist(DistRem(wh,whmin,dpkw100),distrem0);
    SPrintDist(DistRem(wh,whmin,dpkw101),distrem1);
    SPrintDist(DistRem(wh,whmin,dpkw102),distrem2);
    SPrintDist(DistRem(wh,whmin,dpkw103),distrem3);
    SPrintDist(DistRem(wh,whmin,dpkw104),distrem4);

    char line[17];
    sprintf(line,"%c %02d %02d %02d %02d %02d\n",g_LeafCanData.m_CurDteType,dpkw100,dpkw101,dpkw102,dpkw103,dpkw104);
    g_Lcd.setCursor(0,0);
    g_Lcd.print(line);

    sprintf(line,"%c %s %s %s %s %s\n",g_LeafCanData.m_CurDteUnits,distrem0,distrem1,distrem2,distrem3,distrem4);
    g_Lcd.setCursor(0,1);
    g_Lcd.print(line);
  }
}


#ifdef BACKLIGHT_PIN
//0=off 255=max bright
void setBackLight(uint8_t brightness)
{
  //  analogWrite(BACKLIGHT_PIN,brightness);
  digitalWrite(BACKLIGHT_PIN,brightness ? HIGH : LOW);

  g_Brightness = brightness;
}

#endif // BACKLIGHT_PIN

#ifdef CONTRAST_PIN
//0=low 255=max 
void setContrast(uint8_t contrast)
{
  analogWrite(CONTRAST_PIN,contrast);
}
#endif // CONTRAST_PIN


void setup()   
{    
  /* Setup encoder pins */
  g_RotEnc.Setup();

#ifdef BACKLIGHT_PIN
  pinMode(BACKLIGHT_PIN,OUTPUT);
  setBackLight(255);
#endif
#ifdef CONTRAST_PIN
  pinMode(CONTRAST_PIN,OUTPUT);
  setContrast(255);
//digitalWrite( CONTRAST_PIN, LOW);
//pinMode( CONTRAST_PIN, INPUT ); // now we're tri-stated
#endif // CONTRAST_PIN


  g_Lcd.begin(16,2);
  g_Lcd.setCursor(0,0);
  g_Lcd.print("  LeafCAN ");
  g_Lcd.print(VER_STR);
  g_Lcd.setCursor(0,1);
  g_Lcd.print("Waiting for CAN");

  g_CanBus.Init();
  g_Lcd.setCursor(0,1);
  g_Lcd.print("Serial init    ");
  Serial.begin(SERIAL_BAUD);
  g_Lcd.setCursor(0,1);
  g_Lcd.print("Init Complete  ");
   
}


void ServiceEncoder()
{
  int8_t count,btnstate;

  count = g_RotEnc.Read(&btnstate);
  if (count != lastcount) {
    Serial.println(count, DEC);
    lastcount = count;
  }

  /* encoder button */
  if (btnstate != lastbtnstate) {
    g_RotEnc.ResetCount();
    lastcount = 0;
    //    Serial.println(btnstate ? "btnrelease" : "btnpress");
  }
  lastbtnstate = btnstate;
}

void loop()                     
{
  ServiceEncoder();

  if (!g_CanBus.Read()) {
    if (!g_Brightness) {
      setBackLight(255);
    }

    if (g_LogEnabled) {
      st_cmd_t *rxmsg = g_CanBus.GetMsgRx();
      uint8_t msg[11];
      msg[1] = (uint8_t) rxmsg->id.std;
      msg[2] = (uint8_t) (rxmsg->id.std >> 8);
      msg[2] |= (rxmsg->dlc << 4);
      msg[0] = msg[1] ^ msg[2] ^ 0x53;
      memcpy(msg+3,rxmsg->pt_data,rxmsg->dlc);
      for (int8_t i=rxmsg->dlc;i < 8;i++)
	msg[3+i] = 0xff;
      Serial.write(msg,sizeof(msg));
    }

    uint8_t prc = g_LeafCanData.ProcessRxMsg(g_CanBus.GetMsgRx());
    if (!prc && ((millis()-g_LastScreenUpdateMs) > LCD_UPDATE_MS)) { // processed a message
      //uint8_t src = Screen0();
      uint8_t src = Screen1();
      if (!src) {
	g_LastScreenUpdateMs = millis();
      }
    }
  }
}


