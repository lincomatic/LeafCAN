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
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include "LeafCAN.h"

//
// globals
//

LeafCanData g_LeafCanData;
CanBusInterface g_CanBus;

#ifdef ADA_OLED
// rs/rw/enable/d4/d5/d6/d7
#include <Adafruit_CharacterOLED.h>
Adafruit_CharacterOLED g_Lcd(37,39,38,11,12,13,14); // PE5/PE7/PE6/PB3/PB4/PB5/PB6
#elif defined(BREADBOARD)
#include <Wire.h>
#include <LiquidTWI2.h>
LiquidTWI2 g_Lcd(0x20,1);
#else
#include <LiquidCrystal.h>
LiquidCrystal g_Lcd(37,38,11,12,13,14); // PE5/PE6/PB3/PB4/PB5/PB6
//LiquidCrystal g_Lcd(21,20,16,17,18,19); // PC5/PC4/PC0/PC1/PC2/PC3
//LiquidCrystal g_Lcd(37,38,11,12,13,14); // PE5/PE6/PB3/PB4/PB5/PB6
#endif // ADA_OLED

uint8_t g_LogEnabled = 0;
uint8_t g_LogOnly = 0;
uint8_t g_LcdEnabled = 1;

uint8_t g_Brightness = 0;

unsigned long g_LastScreenUpdateMs=0;
int8_t g_CurScreenIdx = SCNIDX_INFO;

#include "RotaryEncoder.h"
RotaryEncoder g_RotEnc;
//encoder stuff
int8_t g_LastBtnState = ENC_BIT_BTN; // 0=pressed
int8_t g_LastCount = 0;

void LcdPrint_P(const prog_char *s)
{
  char buf[17];
  strcpy_P(buf,s);
  g_Lcd.print(buf);
}


// print 2-digit number 100 = A0, 113 = B3, etc
char *SPrintDist(int dist,char *buf)
{
  int i;
  if (dist < 0) {
    buf[0] = '.';
    buf[1] = '0' - dist;
  }
  else {
    i=dist/10;
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
  }
  buf[2] = 0;
  return buf;
}

// distPerKwhx10 = dist/KWh * 10
// returns distance to whMin
// if return value is negative, it's multipied by 10 (meaning it's < 1)
int DistRem(int32_t whRem,int32_t whMin,int distPerKwhx10)
{
  int32_t t = (whRem-whMin)*((int32_t)distPerKwhx10);
  int32_t drem = (t+5000L)/10000;
  if (drem <= 1) {
    int32_t drem10 = (t+500L)/1000; /// distrem * 10
    if (drem10 < 10) {
      drem = -drem10;
    }
  }
  return (int) drem;
}


// KWh gids fixedbars
// volts soc kw
uint8_t InfoScreen(uint8_t force)
{
  uint8_t rc = 1;
  char sf1[10],sf2[10],sf3[10];
  char line[17];
  int w,f;

  // top line: KWh gids fixedbars
  if (force || g_LeafCanData.DirtyBitsSet(DBF_WH_REMAINING |DBF_GIDS|DBF_FIXED_FUEL_BARS)) {
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
 if (force || g_LeafCanData.DirtyBitsSet(DBF_PACK_VOLTS|DBF_SOC|DBF_WATTS)) {
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
// dist/kwh dist/kwh dist/kwh dist/kwh dist/kwh DTEtype
// dist     dist     dist     dist     dist
//
// DTEtype: = 'L' = low batt, 'V' = very low batt, 'T' = turtle
uint8_t DTEScreen(uint8_t force)
{
  if (force || g_LeafCanData.DirtyBitsSet(DBF_WH_REMAINING)) {
    g_LeafCanData.ClearDirtyBits(DBF_WH_REMAINING);

    int32_t whmin;
    if (g_LeafCanData.m_CurDteType == DTE_TYPE_LB) whmin = g_LeafCanData.m_WhL;
    else if (g_LeafCanData.m_CurDteType == DTE_TYPE_VLB) whmin = g_LeafCanData.m_WhV;
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
    sprintf(line,"%2d %2d %2d %2d %2d %c",(int)dpkw100,(int)dpkw101,(int)dpkw102,(int)dpkw103,(int)dpkw104,g_LeafCanData.m_CurDteType);
    g_Lcd.setCursor(0,0);
    g_Lcd.print(line);

    sprintf(line,"%s %s %s %s %s",distrem0,distrem1,distrem2,distrem3,distrem4);
    g_Lcd.setCursor(0,1);
    g_Lcd.print(line);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t VAScreen(uint8_t force)
{
  char line[17];
  if (force || g_LeafCanData.DirtyBitsSet(DBF_PACK_AMPS|DBF_PACK_VOLTS)) {
    g_LeafCanData.ClearDirtyBits(DBF_PACK_AMPS|DBF_PACK_VOLTS);
    // nnn.n
    sprintf(line,"%3d.%cV",(int)(g_LeafCanData.m_PackVolts/2),(g_LeafCanData.m_PackVolts & 1) ? '5' : '0');
    g_Lcd.setCursor(0,0);
    g_Lcd.print(line);
    int16_t amps = -g_LeafCanData.m_PackAmps;
    sprintf(line,"%3d.%cA",(int)(amps/2),(amps % 2) ? '5' : '0');
    g_Lcd.setCursor(0,1);
    g_Lcd.print(line);

    return 0;
  }
  else {
    return 1;
  }
}

int CtoF(int c)
{
  return (c * 9)/5 + 32;
}

uint8_t BattTempScreen(uint8_t force)
{
  if (force || g_LeafCanData.DirtyBitsSet(DBF_BATT_TEMP)) {
    g_LeafCanData.ClearDirtyBits(DBF_BATT_TEMP);

  char line[17];
  g_Lcd.setCursor(0,0);
  sprintf(line,"T1  T2  T3  T4 %c",g_LeafCanData.m_TempUnit);
  g_Lcd.print(line);
  if (g_LeafCanData.m_TempUnit == 'C') {
  sprintf(line,"%2d  %2d  %2d  %2d",(int)g_LeafCanData.m_BattTemp1,(int)g_LeafCanData.m_BattTemp2,(int)g_LeafCanData.m_BattTemp3,(int)g_LeafCanData.m_BattTemp4);
  }
  else {
    sprintf(line,"%-3d %-3d %-3d %-3d",CtoF(g_LeafCanData.m_BattTemp1),CtoF(g_LeafCanData.m_BattTemp2),CtoF(g_LeafCanData.m_BattTemp3),CtoF(g_LeafCanData.m_BattTemp4));
  }
  g_Lcd.setCursor(0,1);
  g_Lcd.print(line);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t CellPairVoltScreen(uint8_t force)
{
  if (force || g_LeafCanData.DirtyBitsSet(DBF_CP_VOLTS)) {
    g_LeafCanData.ClearDirtyBits(DBF_CP_VOLTS);

  char line[17];
  g_Lcd.setCursor(0,0);
  g_Lcd.print("Min  Max  Diff m");
  sprintf(line,"%4d %4d %4d V",(int)g_LeafCanData.m_CPVmin,(int)g_LeafCanData.m_CPVmax,(int)(g_LeafCanData.m_CPVmax-g_LeafCanData.m_CPVmin));
  g_Lcd.setCursor(0,1);
  g_Lcd.print(line);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t SocCapScreen(uint8_t force)
{
  if (force || g_LeafCanData.DirtyBitsSet(DBF_SOC_CAP)) {
    g_LeafCanData.ClearDirtyBits(DBF_SOC_CAP);
    
    char line[17];
    
    sprintf(line,"SOC %2d.%04d%%",(int)(g_LeafCanData.m_SOC32/10000),(int)(g_LeafCanData.m_SOC32 % 10000));
    g_Lcd.setCursor(0,0);
    g_Lcd.print(line);
    sprintf(line,"%2d.%04dAh %2d.%02d%%",(int)(g_LeafCanData.m_PackCap/10000),(int)(g_LeafCanData.m_PackCap % 10000),
                (int)(g_LeafCanData.m_PackHealth/100),(int)(g_LeafCanData.m_PackHealth % 100));
    g_Lcd.setCursor(0,1);
    g_Lcd.print(line);
    return 0;
  }
  else {
    return 1;
  }
}

// return 0 if screen redrawn
uint8_t DrawScreen(uint8_t force=0)
{
  uint8_t rc;
  if (g_CurScreenIdx == SCNIDX_INFO) {
    rc = InfoScreen(force);
  }
  else if (g_CurScreenIdx == SCNIDX_DTE) {
    rc = DTEScreen(force);
  }
  else if (g_CurScreenIdx == SCNIDX_VA) {
    rc = VAScreen(force);
  }
  else if (g_CurScreenIdx == SCNIDX_BATT_TEMP) {
    rc = BattTempScreen(force);
  }
  else if (g_CurScreenIdx == SCNIDX_CP_VOLT) {
    rc = CellPairVoltScreen(force);
  }
  else { // SCNIDX_SOC_CAP
    rc = SocCapScreen(force);
  }

  return rc;
}


//0=off 255=max bright
void setBackLight(uint8_t brightness)
{
#ifdef BACKLIGHT_PIN
  //  analogWrite(BACKLIGHT_PIN,brightness);
  digitalWrite(BACKLIGHT_PIN,brightness ? HIGH : LOW);
#else
  if (brightness) g_Lcd.display();
  else g_Lcd.noDisplay();
#endif

  g_Brightness = brightness;
}

#ifdef CONTRAST_PIN
//0=low 255=max 
void setContrast(uint8_t contrast)
{
  analogWrite(CONTRAST_PIN,contrast);
}
#endif // CONTRAST_PIN

void setup()   
{
#ifdef RLED_PIN
  pinMode(RLED_PIN,OUTPUT);
  digitalWrite(RLED_PIN,HIGH); // turn off
#endif
#ifdef GLED_PIN
  pinMode(GLED_PIN,OUTPUT);
  digitalWrite(GLED_PIN,HIGH);
#endif
#ifdef BLED_PIN
  pinMode(BLED_PIN,OUTPUT);
  digitalWrite(BLED_PIN,HIGH);
#endif

  g_LastScreenUpdateMs = millis();
  /* Setup encoder pins */
  g_RotEnc.Setup();


#ifdef BACKLIGHT_PIN
  pinMode(BACKLIGHT_PIN,OUTPUT);
#endif
  setBackLight(255);
#ifdef CONTRAST_PIN
  pinMode(CONTRAST_PIN,OUTPUT);
  setContrast(255);
//digitalWrite( CONTRAST_PIN, LOW);
//pinMode( CONTRAST_PIN, INPUT ); // now we're tri-stated
#endif // CONTRAST_PIN


  g_Lcd.begin(16,2);
  g_Lcd.setCursor(0,0);
  LcdPrint_P(PSTR("LeafCAN "));
  g_Lcd.print(VER_STR);
  g_Lcd.setCursor(0,1);
  LcdPrint_P(PSTR("by Lincomatic"));
//  delay(2000);
//  g_Lcd.setCursor(0,1);
//  LcdPrint_P(PSTR("Awaiting CAN...."));

  g_CanBus.Init();
#ifdef SERIAL
  Serial.begin(SERIAL_BAUD);
#endif
}

void ServiceEncoder()
{
  int8_t count,btnstate;

  count = g_RotEnc.Read(&btnstate);
  if (count != g_LastCount) {
    g_CurScreenIdx += count;
    while (g_CurScreenIdx > SCREEN_CNT-1) g_CurScreenIdx -= SCREEN_CNT;
    while (g_CurScreenIdx < 0) g_CurScreenIdx += SCREEN_CNT;
	g_LastScreenUpdateMs = millis()-1000;
    g_Lcd.clear();
    DrawScreen(1);
    g_RotEnc.ResetCount();
    g_LastCount = 0;
  }

  /* encoder button */
  if (btnstate != g_LastBtnState) {
    if (!g_Brightness && ((millis()-g_CanBus.m_LastCanMsgRxMs) >= BACKLIGHT_TIMEOUT)) {
      // can bus idle and backlight off
      // turn it on for a few sec
      g_CanBus.m_LastCanMsgRxMs = millis();
      setBackLight(255);
    }
    else if (
#ifdef INVERT_ENC_BTN
	!
#endif
	btnstate
	) { // released

      if (g_CurScreenIdx == SCNIDX_DTE) {
	char dt;
        if (g_LeafCanData.m_CurDteType == DTE_TYPE_LB) dt = DTE_TYPE_VLB;
        else if (g_LeafCanData.m_CurDteType == DTE_TYPE_VLB) dt = DTE_TYPE_TURTLE;
        else dt = DTE_TYPE_LB;
	g_LeafCanData.SetDteType(dt);
        DrawScreen(1);
      }
      else if (g_CurScreenIdx == SCNIDX_BATT_TEMP) {
	g_LeafCanData.SetTempUnit((g_LeafCanData.m_TempUnit == TEMP_UNIT_C) ? TEMP_UNIT_F : TEMP_UNIT_C);
	DrawScreen(1);
      }
    }
    //    Serial.println(btnstate ? "btnrelease" : "btnpress");
  }
  g_LastBtnState = btnstate;
}

void loop()                     
{
  ServiceEncoder();

  uint8_t cbrc;
  if (!(cbrc=g_CanBus.Read())) {
    if (!g_Brightness) {
      setBackLight(255);
    }

#ifdef SERIAL
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
#endif // SERIAL

    uint8_t prc = g_LeafCanData.ProcessRxMsg(g_CanBus.GetMsgRx());
#ifdef RLED_PIN
    if (!prc && g_LeafCanData.DirtyBitsSet(DBF_PACK_AMPS)) {
      if (g_LeafCanData.m_PackAmps < -6) { // consumption
	// n.b. need to rewrite using DDRx instead
	digitalWrite(RLED_PIN,LOW);
	digitalWrite(GLED_PIN,HIGH);
	digitalWrite(BLED_PIN,HIGH);
      }
      else if (g_LeafCanData.m_PackAmps > 1) { // regen
	digitalWrite(RLED_PIN,HIGH);
	digitalWrite(GLED_PIN,LOW);
	digitalWrite(BLED_PIN,HIGH);
      }
      else { // idle
	digitalWrite(RLED_PIN,HIGH);
	digitalWrite(GLED_PIN,HIGH);
	digitalWrite(BLED_PIN,LOW);
      }
    }
#endif  
    unsigned long ms = millis();
    if (!prc && ((ms - g_LastScreenUpdateMs) > LCD_UPDATE_MS)) { // processed a message
      uint8_t src = DrawScreen();
      if (!src) {
	g_LastScreenUpdateMs = ms;
      }
    }

    if ((ms-g_LeafCanData.m_Last7BBreqTime) > CAN_REQ_INTERVAL) {
      uint8_t group;
      switch(g_CurScreenIdx) {
      case SCNIDX_SOC_CAP:
	group = 1;
	break;
      case SCNIDX_CP_VOLT:
	group = 3;
	break;
      case SCNIDX_BATT_TEMP:
	group = 4;
	break;
      default:
	group = REQ_GROUP_INVALID;
      }
      
      if (group != REQ_GROUP_INVALID) {
	g_LeafCanData.Req79B(group);
      }
    }

    g_LeafCanData.ReqNext7BBFrame();
  }
  else {
    if (cbrc > 1) { // timed out CAN read
      if (g_Brightness && ((millis()-g_CanBus.m_LastCanMsgRxMs) >= BACKLIGHT_TIMEOUT)) {
	g_LeafCanData.SaveEEPROM();
	setBackLight(0);
#ifdef RLED_PIN
	digitalWrite(RLED_PIN,HIGH); // turn off
	digitalWrite(GLED_PIN,HIGH); // turn off
	digitalWrite(BLED_PIN,HIGH); // turn off
#endif
      }
    }
  }
}


