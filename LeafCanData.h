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
#ifndef _LEAFCANDATA_H_
#define _LEAFCANDATA_H_

// dirty bits
#define DBF_PACK_VOLTS 0x0001
#define DBF_PACK_AMPS  0x0002
#define DBF_WATTS      0x0004
#define DBF_SOC        0x0008
#define DBF_GIDS       0x0010
#define DBF_GOM_FUEL_BARS   0x0020
#define DBF_FIXED_FUEL_BARS 0x0040
#define DBF_WH_REMAINING    0x0080
#define DBF_BATT_TEMP  0x0100
#define DBF_SOC_CAP    0x0200
#define DBF_CP_VOLTS   0x0400

#define DBF_EEPROM 0x8000 // special dirty bit to force EEPROM save

// m_CurDteType
#define DTE_TYPE_LB 'L'
#define DTE_TYPE_VLB 'V'
#define DTE_TYPE_TURTLE 'T'

inline uint8_t IsValidDteType(char t)
{
  if ((t == DTE_TYPE_LB) || (t == DTE_TYPE_VLB) || (t == DTE_TYPE_TURTLE)) return 1;
  else return 0;
}

// m_TempUnit
#define TEMP_UNIT_C 'C'
#define TEMP_UNIT_F 'F'
inline uint8_t IsValidTempUnit(char u)
{
  if ((u == TEMP_UNIT_C) || (u == TEMP_UNIT_F)) return 1;
  else return 0;
}

class LeafCanData {
  uint16_t m_DirtyBits;
public:
  // 79B msg response is 7BB
  unsigned long m_Last7BBreqTime;
  uint8_t m_Cur79BGroup;
  uint8_t m_Cur7BBReqFrameIdx;
  uint8_t m_Cur7BBRcvFrameIdx;
  uint8_t m_Cur7BBFrameCnt;
public:
  uint16_t m_PackVolts; // Pack Volts * 2
  int16_t m_PackAmps;  // Pack Amps * 2 (positive = charging)
  uint16_t m_SOC;   // = SOC (% * 10)
  uint16_t m_Gids;
  uint8_t  m_GOMFuelBars;  // fuel bars displayed in dash GOM
  uint8_t  m_FixedFuelBars; // fixed fuel bars * 10
  uint8_t m_BattTemp1,m_BattTemp2,m_BattTemp3,m_BattTemp4; // battery temperature (C)
  uint16_t m_CPVmin,m_CPVmax; // cell pair min/max voltage (mV)
  int32_t m_SOC32; // high precision SOC (% * 10000)
  int32_t m_PackCap; // pack capacity (Ah * 10000)
  int32_t m_PackHealth; // health (% * 100)

  // calculated values
  int32_t m_W; // Watts (negative = charging)
  int32_t m_Wh; // energy remaining in pack = gids * KW_FACTOR

  // calculated constants
  int32_t m_WhL; // Wh left @ LB
  int32_t m_WhV; // Wh left @ VLB
  int32_t m_WhT; // Wh left @ Turtle

  // settings
  int32_t m_DpKWh10_Low; // lowest dist/KWh * 10 for DTEt
  int32_t m_DpKWh10_Incr; // DpKWh10 increment for DTE
  char m_CurDteType; // DTE_TYPE_xxx
  char m_TempUnit; // TEMP_UNIT_x

  
  LeafCanData();
  uint16_t SetDirtyBits(uint16_t bits) {
    m_DirtyBits |= bits;
  }
  uint16_t ClearDirtyBits(uint16_t bits) {
    m_DirtyBits &= ~bits;
  }
  uint16_t DirtyBitsSet(uint16_t bits) {
    return m_DirtyBits & bits;
  }
  void SetTempUnit(char t) { m_TempUnit = t;SetDirtyBits(DBF_EEPROM); }
  void SetDteType(char t) { m_CurDteType = t;SetDirtyBits(DBF_EEPROM); }
  void SaveEEPROM();

  uint8_t ProcessRxMsg(st_cmd_t *rxmsg);
  uint8_t Process7BBFrame(uint8_t *candata);
  uint8_t Req79B(uint8_t group);
  uint8_t ReqNext7BBFrame();
};

#endif // _LEAFCANDATA_H_
