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

#define DBF_PACK_VOLTS 0x0001
#define DBF_PACK_AMPS  0x0002
#define DBF_WATTS      0x0004
#define DBF_SOC        0x0008
#define DBF_GIDS       0x0010
#define DBF_GOM_FUEL_BARS   0x0020
#define DBF_FIXED_FUEL_BARS 0x0040
#define DBF_WH_REMAINING    0x0080
class LeafCanData {
  uint16_t m_DirtyBits;
public:
  uint16_t m_PackVolts; // Pack Volts * 2
  int16_t m_PackAmps;  // Pack Amps * 2 (positive = charging)
  uint16_t m_SOC;   // = SOC% * 10
  uint16_t m_Gids;
  uint8_t  m_GOMFuelBars;  // fuel bars displayed in dash GOM
  uint8_t  m_FixedFuelBars; // fixed fuel bars * 10

  // calculated values
  int32_t m_W; // Watts (negative = charging)
  int32_t m_Wh; // energy remaining in pack = gids * KW_FACTOR

  // calculated constants
  int32_t m_WhL; // Wh left @ LB
  int32_t m_WhV; // Wh left @ VLB
  int32_t m_WhT; // Wh left @ Turtle

  // settings
  int32_t m_DpKWh10_Low; // lowest dist/KWh * 10 for DTE
  int32_t m_DpKWh10_Incr; // DpKWh10 increment for DTE
  char m_CurDteType; // 'L'=lb,'V'=vlb,'T'=turtle
  char m_CurDteUnits; // 'M' = mi, 'K' = km

  
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

  uint8_t ProcessRxMsg(st_cmd_t *rxmsg);
};

#endif // _LEAFCANDATA_H_
