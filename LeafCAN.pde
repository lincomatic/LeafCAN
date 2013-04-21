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

#ifdef V2
//#ifdef ADA_OLED
//#include <Adafruit_CharacterOLED.h>
// rs/rw/enable/d4/d5/d6/d7
//Adafruit_CharacterOLED lcd(37,39,38,11,12,13,14); // PE5/PE7/PE6/PB3/PB4/PB5/PB6
//#else
#include <LiquidCrystal.h>
LiquidCrystal g_Lcd(37,38,11,12,13,14); // PE5/PE6/PB3/PB4/PB5/PB6
//#endif // ADA_OLED
#else // V1
LiquidCrystal g_Lcd(21,20,16,17,18,19); // PC5/PC4/PC0/PC1/PC2/PC3
#endif // V2
//LiquidCrystal g_Lcd(37,38,11,12,13,14); // PE5/PE6/PB3/PB4/PB5/PB6

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
    if (wh >= 10.0F) {
      // nn.n
      wh += 50;
      sprintf(sf1,"%d.%d",(int)(wh/1000),(int)((wh%1000)/100));
    }
    else if (wh >= 1.0F) {
      // n.nn
      wh += 5;
      sprintf(sf1,"%d.%d",(int)(wh/1000),(int)((wh%1000)/10));
    }
    else {
      // .nnn
      sprintf(sf1,".%d",(int)(wh % 1000));
    }

    // nn.n
    sprintf(sf2,"%d.%d",g_LeafCanData.m_FixedFuelBars/10,g_LeafCanData.m_FixedFuelBars%10);
    sprintf(line,"%s  %3d   %s",sf1,g_LeafCanData.m_Gids,sf2);

    g_Lcd.setCursor(0,0);
    g_Lcd.print(line);
  }
  // bottom line: volts soc kw
  if (g_LeafCanData.DirtyBitsSet(DBF_PACK_VOLTS|DBF_SOC|DBF_WATTS)) {
    rc = 0;
    g_LeafCanData.ClearDirtyBits(DBF_PACK_VOLTS|DBF_SOC|DBF_WATTS);

    // nnn.n
    sprintf(sf1,"%d.%c",g_LeafCanData.m_PackVolts/2,(g_LeafCanData.m_PackVolts & 1) ? '5' : '0');
    // nn.n
    sprintf(sf2,"%d.%d",g_LeafCanData.m_SOC/10,g_LeafCanData.m_SOC%10);

    int32_t w = g_LeafCanData.m_W;
    if (w >= 10000.0F) {
      // nn.n
      w += 50;
      sprintf(sf3,"%d.%d",(int)(w/1000),(int)((w%1000)/100));
    }
    else if (w <= -10000.0F) {
      // -nn
      sprintf(sf3,"%d",(int)(w/1000));
    }
    else if (w < 0.0F) {
      // -n.n
      w += 50;
      sprintf(sf3,"%d.%d",(int)(w/1000),-(int)((w%1000)/100));
    }
    else {
      // n.nn
      w += 5;
      sprintf(sf3,"%d.%d",(int)(w/1000),(int)((w%1000)/10));
    }

    sprintf(line,"%s %s %s",sf1,sf2,sf3);
    g_Lcd.setCursor(0,1);
    g_Lcd.print(line);
  }

  return rc;
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
      uint8_t src = Screen0();
      if (!src) {
	g_LastScreenUpdateMs = millis();
      }
    }
  }
}


