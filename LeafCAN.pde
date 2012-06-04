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

#define VER_STR "v1.2"

#define SERIAL_BAUD 115200
#define MAX_SOC 281.0F
#define KW_FACTOR 80.0F // 80 is from phil, tonywilliams prefers 75
#define LCD_UPDATE_MS 250 // update interval for LCD in ms
#define SHOW_KWH // show remaining pack KWh in line 1
#define SHOW_KW // show KW usage on line 2

typedef struct ev_data {
  uint16_t m_Soc;
  float m_SocPct;
  float m_PackVolts;
  float m_PackAmps; // A
  uint8_t m_FuelBars; // <= 12
  int16_t m_RPM;
  int16_t m_MotorPower;//kW ??
} EV_DATA,*PEV_DATA;

EV_DATA g_EvData;

st_cmd_t g_CanMsg;
uint8_t g_CanData[8];
LiquidCrystal lcd(21,20,16,17,18,19); // PC5/PC4/PC0/PC1/PC2/PC3
//LiquidCrystal lcd(37,38,11,12,13,14); // PE5/PE6/PB3/PB4/PB5/PB6

uint8_t g_LogEnabled = 0;
uint8_t g_LcdEnabled = 1;

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

void setup()   
{  
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
  while(CAN.cmd(&g_CanMsg) != CAN_CMD_ACCEPTED);
  // --- Wait for Rx completed
  while((canstat=CAN.get_status(&g_CanMsg)) == CAN_STATUS_NOT_COMPLETED);

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
#ifdef SHOW_KW
	dtostrf(g_EvData.m_PackAmps,5,1,sf2);
	float kw = (g_EvData.m_PackAmps * g_EvData.m_PackVolts)/1000.0F;
        char *skw = sf3;

	if (kw >= 10.0F) {
	  dtostrf(kw,4,1,sf3);
	}
        else if (kw <= -10.0F) {
          dtostrf(kw,4,0,sf3);
        }
        else if (kw < 0.0F) {
          dtostrf(kw,4,1,sf3);
        }
	else {
	  dtostrf(kw,4,2,sf3);
	}

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
      else if (g_CanMsg.id.std == 0x5b9) {
	g_EvData.m_FuelBars = g_CanData[0] >> 3;
      }
      else if (g_CanMsg.id.std == 0x5bc)  {
        if ((ms-lastsoc) > LCD_UPDATE_MS) {
	g_EvData.m_Soc = (g_CanData[0] << 2) | (g_CanData[1] >> 6);
	g_EvData.m_SocPct = (g_EvData.m_Soc / MAX_SOC) * 100.0F;
#ifdef SHOW_KWH
	dtostrf(g_EvData.m_SocPct,4,g_EvData.m_SocPct < 100.0F ? 1 : 0,sf1);

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
	sprintf(line,"%s %3d %s %2d",skwh,g_EvData.m_Soc,sf1,g_EvData.m_FuelBars);
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

