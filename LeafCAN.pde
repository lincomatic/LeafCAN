// -*- C++ -*-
/*
 * LeafCAN Firmware
 *
 * Copyright (c) 2012 Sam C. Lin <lincomatic@gmail.com>
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
#include <LiquidCrystal.h>
#include <can_lib.h>

#define V2 // v2 hardware
// support Adafruit 16x2 OLED display http://www.adafruit.com/products/823
// requires Adafruit's library: https://github.com/ladyada/Adafruit_CharacterOLED
//#define ADA_OLED

#ifdef ADA_OLED
#define VER_STR "v1.3o"
#else
#define VER_STR "v1.3"
#endif

#define BACKLIGHT_PIN 15 // PB7
#define CONTRAST_PIN 36 // PE4

#define SERIAL_BAUD 115200
#define MAX_SOC 281.0F
#define KW_FACTOR 74.73F // 80 is from ingineer, surfingslovak prefers 74.73
#define LCD_UPDATE_MS 250 // update interval for LCD in ms
#define SOCPCT_55B
#define SHOW_KWH // show remaining pack KWh in line 1
#define SHOW_KW // show KW usage on line 2






// the Leaf's fuel bar display isn't tied to SOC, which I find very confusing,
// because this means that as the battery ages or temperature changes, the bars
// correspond to less
// stored KWh in the battery.  When FIXED_FUEL_BARS is defined, instead of
// displaying the same fuel bars as on the dash, the fuel display is directly
// tied to SOC. My formula arbitrarily assigns 13 = 100% charge = 281 gids,
// and the transition from 1 -> 0 is at 24 gids, which is the transition to
// Very Low Battery.  When the battery gets this low, it's kind of useless to
// use something as coarse as fuel bars, so when it displays 0, it's time to
// look directly at the raw SOC values.
// the fuel bars are displayed w/ 1 decimal place (e.g 9.2) so you always know
// how "full" the current bar is
//#define FIXED_FUEL_BARS

typedef struct ev_data {
  uint16_t m_Soc;
  float m_SocPct;
#ifdef SOCPCT_55B
  char m_SocPct_55b[5]; // soc pct from msg 55b
#endif
  float m_PackVolts;
  float m_PackAmps; // A
#ifdef FIXED_FUEL_BARS
  float m_FuelBars;
#else
  uint8_t m_FuelBars; // <= 12
#endif // FIXED_FUEL_BARS
  int16_t m_RPM;
  int16_t m_MotorPower;//kW ??
} EV_DATA,*PEV_DATA;

EV_DATA g_EvData;

st_cmd_t g_CanMsg;
uint8_t g_CanData[8];
#ifdef V2
#ifdef ADA_OLED
#include <Adafruit_CharacterOLED.h>
// rs/rw/enable/d4/d5/d6/d7
Adafruit_CharacterOLED lcd(37,39,38,11,12,13,14); // PE5/PE7/PE6/PB3/PB4/PB5/PB6
#else
// rs/enable/d4/d5/d6/d7
LiquidCrystal lcd(37,38,11,12,13,14); // PE5/PE6/PB3/PB4/PB5/PB6

#endif
#else // V1
LiquidCrystal lcd(21,20,16,17,18,19); // PC5/PC4/PC0/PC1/PC2/PC3
#endif // V2
//LiquidCrystal lcd(37,38,11,12,13,14); // PE5/PE6/PB3/PB4/PB5/PB6

uint8_t g_LogEnabled = 0;
uint8_t g_LcdEnabled = 1;

#ifdef V2
#define BACKLIGHT_TIMEOUT 5000 // turn off backlight after CAN bus idle for (ms)
uint8_t g_BacklightOn = 0;
unsigned long g_LastCanMsgMs;
#endif // V2

void CANinit()
{
  // Clock prescaler Reset SCL:don't need this unless CLKDIV8 is set in the low fuse
  // CLKPR = 0x80;  CLKPR = 0x00;

  //- Pull-up on TxCAN & RxCAN one by one to use bit-addressing
  CAN_PORT_DIR &= ~(1<<CAN_INPUT_PIN );
  CAN_PORT_DIR &= ~(1<<CAN_OUTPUT_PIN);
  CAN_PORT_OUT |=  (1<<CAN_INPUT_PIN );
  CAN_PORT_OUT |=  (1<<CAN_OUTPUT_PIN);
  
  //- Wait until activity on RxCAN
  while ((CAN_PORT_IN & (1<<CAN_INPUT_PIN)) != 0);
  //- Reset CAN peripheral
  Can_reset();
  
  CAN.set_baudrate(CAN_AUTOBAUD);
  CAN.init(0); 
  
  //- Set CAN Timer Prescaler
  // SCL: dont think we need this CANTCON = CANBT1;                   // Why not !
}

#ifdef V2
//0=off 255=max bright
void setBackLight(uint8_t brightness)
{
  //  analogWrite(BACKLIGHT_PIN,brightness);
  if (brightness) {
    digitalWrite(BACKLIGHT_PIN,HIGH);
    g_BacklightOn = 1;
  }
  else {
    digitalWrite(BACKLIGHT_PIN,LOW);
    g_BacklightOn = 0;
  }
}

//0=low 255=max 
void setContrast(uint8_t contrast)
{
  analogWrite(CONTRAST_PIN,contrast);
}
#endif // V2

void setup()   
{  
#ifdef V2
  pinMode(BACKLIGHT_PIN,OUTPUT);
  pinMode(CONTRAST_PIN,OUTPUT);

  setBackLight(255);
  setContrast(255);
//digitalWrite( CONTRAST_PIN, LOW);
//pinMode( CONTRAST_PIN, INPUT ); // now we're tri-stated
#endif // V2
  
  g_CanMsg.cmd = CMD_RX;
  g_CanMsg.pt_data = g_CanData;

  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("  LeafCAN ");
  lcd.print(VER_STR);
  lcd.setCursor(0,1);
  lcd.print("Waiting for CAN");

  CANinit();
  lcd.setCursor(0,1);
  lcd.print("Serial init    ");
  Serial.begin(SERIAL_BAUD);
  lcd.setCursor(0,1);
  lcd.print("Init Complete  ");
   
}

uint8_t ReadCAN()
{
  uint8_t canstat;
  // --- Enable Rx
  while(CAN.cmd(&g_CanMsg) != CAN_CMD_ACCEPTED) {
#ifdef V2
    if (g_BacklightOn && ((millis()-g_LastCanMsgMs) >= BACKLIGHT_TIMEOUT)) {
      setBackLight(0);
    }
#endif // V2
  }
  // --- Wait for Rx completed
  while((canstat=CAN.get_status(&g_CanMsg)) == CAN_STATUS_NOT_COMPLETED) {
#ifdef V2
    if (g_BacklightOn && ((millis()-g_LastCanMsgMs) >= BACKLIGHT_TIMEOUT)) {
      setBackLight(0);
    }
#endif // V2
  }

  return (canstat == CAN_STATUS_ERROR) ? 1 : 0;
}

unsigned long lastpack,lastsoc;
void loop()                     
{
  int16_t i16;
  char sf1[10],sf2[10],sf3[10];
  char line[17];
  int8_t i;

  if (!ReadCAN()) {
#ifdef V2
    g_LastCanMsgMs = millis();
    if (!g_BacklightOn) {
      setBackLight(255);
    }
#endif //V2
    if (g_LogEnabled) {
      uint8_t msg[11];
      msg[1] = (uint8_t) g_CanMsg.id.std;
      msg[2] = (uint8_t) (g_CanMsg.id.std >> 8);
      msg[2] |= (g_CanMsg.dlc << 4);
      msg[0] = msg[1] ^ msg[2] ^ 0x53;
      memcpy(msg+3,g_CanData,g_CanMsg.dlc);
      for (i=g_CanMsg.dlc;i < 8;i++)
	msg[3+i] = 0xff;
      Serial.write(msg,sizeof(msg));
    }
    if (g_LcdEnabled) {
      unsigned long ms = millis();
      if (g_CanMsg.id.std == 0x1db) {
        if ((ms-lastpack) > LCD_UPDATE_MS) {
	g_EvData.m_PackVolts = (g_CanData[2] << 2) | (g_CanData[3] >> 6);
	g_EvData.m_PackVolts /= 2.0F;


	i16 = (g_CanData[0] << 3) | (g_CanData[1] >> 5);
	if (i16 & 0x0400) { // negative - cvt extend sign bit
	  i16 |= 0xf800;
	}

	g_EvData.m_PackAmps = -(i16 / (2.0F));
	dtostrf(g_EvData.m_PackVolts,5,1,sf1);
        char *skw = sf3;
	float kw = (g_EvData.m_PackAmps * g_EvData.m_PackVolts)/1000.0F;
#ifdef SHOW_KW
#ifdef SOCPCT_55B
	strcpy(sf2,g_EvData.m_SocPct_55b);
	dtostrf(kw,5,((kw < 10.0) && (kw > -10.0)) ? 2 : 1,skw);
#else
	dtostrf(g_EvData.m_PackAmps,5,1,sf2);

	if (kw >= 10.0F) {
	  dtostrf(kw,4,1,skw);
	}
        else if (kw <= -10.0F) {
          dtostrf(kw,4,0,skw);
        }
        else if (kw < 0.0F) {
          dtostrf(kw,4,1,skw);
        }
	else {
	  dtostrf(kw,4,2,skw);
	}
#endif

	sprintf(line,"%s %s %s",sf1,sf2,skw);
#else
	dtostrf(g_EvData.m_PackAmps,6,1,sf2);
        sprintf(line,"B %sV %sA",sf1,sf2);
#endif // SHOW_KW
	lcd.setCursor(0,1);
	lcd.print(line);
	lastpack = ms;
        }
      }
#ifdef SOCPCT_55B
      else if (g_CanMsg.id.std == 0x55b) {
	// http://www.mynissanleaf.com/viewtopic.php?f=44&t=4131&p=229040#p229040
	// ticktock says
	//I am fairly certain that 55b is the actual SOC in D1&D2:
	//55b:D1[7:0]<<2+D2[7:6]>>6 = SOC % X 10.
	//I get exactly 80.0 on an 80% charge and 95.1-95.6 on a 100% charge. This suggests that this is actually what is used to dtermine the stop point on an 80% charge. I was wondering if anyone who hasn't seen the southwest loss impact could take a look. I'm curious if it shows > 95% for a new battery or if 95% is the target. Since 80% scales with the battery capacity degradation I would expect the 95% too as well but maybe less precisely since they use pack volts to stop instead of SOC.
	uint16_t socpctx10 = (g_CanData[0] << 2) | (g_CanData[1] >> 6);
	uint8_t socpct = socpctx10 / 10;
	uint8_t socpctfrac = socpctx10 % 10;
	sprintf(g_EvData.m_SocPct_55b,"%d.%d",socpct,socpctfrac);
      }
#endif //SOCPCT_55B
#ifndef FIXED_FUEL_BARS
      else if (g_CanMsg.id.std == 0x5b9) {
	g_EvData.m_FuelBars = g_CanData[0] >> 3;
      }
#endif // !FIXED_FUEL_BARS
      else if (g_CanMsg.id.std == 0x5bc)  {
        if ((ms-lastsoc) > LCD_UPDATE_MS) {
	g_EvData.m_Soc = (g_CanData[0] << 2) | (g_CanData[1] >> 6);
	g_EvData.m_SocPct = (g_EvData.m_Soc / MAX_SOC) * 100.0F;
#ifdef FIXED_FUEL_BARS
	g_EvData.m_FuelBars = ((float)(g_EvData.m_Soc - 24))/257 * 13.0F;
#endif // FIXED_FUEL_BARS
#ifdef SHOW_KWH
	float kwh = (((float)g_EvData.m_Soc) * KW_FACTOR) / 1000.0F;
        char *skwh = sf2;
        if (kwh >= 10.0F) {
          dtostrf(kwh,4,1,sf2);
        }
        else if (kwh >= 1.0F) {
          dtostrf(kwh,4,2,sf2);
        }
        else {
          dtostrf(kwh,4,3,sf2);
          skwh = sf2 + 1;
        }
#ifdef FIXED_FUEL_BARS
        float ffb = g_EvData.m_FuelBars - .049999;
	dtostrf(ffb,4,1,sf3);
	sprintf(line,"%s  %3d   %s",skwh,g_EvData.m_Soc,sf3);
#else
	dtostrf(g_EvData.m_SocPct,4,g_EvData.m_SocPct < 100.0F ? 1 : 0,sf1);

	sprintf(line,"%s %3d %s %2d",skwh,g_EvData.m_Soc,sf1,g_EvData.m_FuelBars);
#endif // FIXED_FUEL_BARS
#else
	dtostrf(g_EvData.m_SocPct,5,1,sf1);
	sprintf(line,"SOC%s%% %3d %2d",sf1,g_EvData.m_Soc,g_EvData.m_FuelBars);
#endif // SHOW_KWH

	lcd.setCursor(0,0);
	lcd.print(line);
	lastsoc = ms;
        }
      }
      /*not yet
      else if (g_CanMsg.id.std == 0x1da) {
	// RPM
	i16 = (g_CanData[4] << 8) | g_CanData[5];
	g_EvData.RPM = i16;

	i16 = ((g_CanData[2] & 0x03) << 8) | g_CanData[3];
	if (i16 & 0x0200) { // negative - extend sign bit
	  i16 |= 0xfc00;
	}
	g_EvData.m_MotorPower = i16;
      }
      */

    }
  }
}

