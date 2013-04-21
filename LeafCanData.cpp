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
#include <can_lib.h>
#include "LeafCAN.h"

LeafCanData::LeafCanData()
{
  m_DirtyBits = 0;
  m_PackVolts = 0;
  m_PackAmps = 0;
  m_W = 0;
  m_SOC = 0;
  m_GOMFuelBars = 0;
  m_Gids = 0;
}

// return = 0 = processed a CAN msg
//        = 1 = got an unprocessed CAN msg
uint8_t LeafCanData::ProcessRxMsg(st_cmd_t *rxmsg)
{
  uint8_t rc = 1;
  uint8_t *candata = rxmsg->pt_data;

  if (rxmsg->id.std == 0x1db) {
    char calcKW = 0;
    uint16_t rpv = (candata[2] << 2) | (candata[3] >> 6);
    if (rpv != m_PackVolts) {
      SetDirtyBits(DBF_PACK_VOLTS);
      m_PackVolts = rpv;
      calcKW = 1;
    }
    
    int16_t rpa = (candata[0] << 3) | (candata[1] >> 5);
    if (rpa & 0x0400) { // negative - cvt extend sign bit
      rpa |= 0xf800;
    }
    if (rpa != m_PackAmps) {
      SetDirtyBits(DBF_PACK_AMPS);
      m_PackAmps = rpa;
      calcKW = 1;
    }
    
    if (calcKW) {
      SetDirtyBits(DBF_WATTS);
      int32_t v10 = rpv*10 + ((rpv & 1) ? 5 : 0); // volts * 10
      int32_t a10 = rpa*10 + ((rpa & 1) ? 5 : 0); // amps * 10
      m_W = ((v10 * -a10) + 50L)/100L;
    }
    rc = 0;
  }
  else if (rxmsg->id.std == 0x55b) {
    SetDirtyBits(DBF_SOC);
    
    uint16_t soc10 = (candata[0] << 2) | (candata[1] >> 6);
    if (soc10 != m_SOC) {
      m_SOC = soc10;
    }
    rc = 0;
  }
  else if (rxmsg->id.std == 0x5b9) {
    SetDirtyBits(DBF_GOM_FUEL_BARS);
    m_GOMFuelBars = candata[0] >> 3;
    rc = 0;
  }
  else if (rxmsg->id.std == 0x5bc)  {
    SetDirtyBits(DBF_GIDS|DBF_WH_REMAINING|DBF_FIXED_FUEL_BARS);
    m_Gids = (candata[0] << 2) | (candata[1] >> 6);
    m_Wh = ((((int32_t)m_Gids) * KW_FACTOR)+50L) / 100L;
    m_FixedFuelBars = (uint8_t)(((((int32_t)m_Gids) - 24L)*130L)/257);
    rc = 0;
  }
      /*not yet
      else if (rxmsg->id.std == 0x1da) {
	// RPM
	i16 = (candata[4] << 8) | candata[5];
	g_EvData.RPM = i16;

	i16 = ((candata[2] & 0x03) << 8) | candata[3];
	if (i16 & 0x0200) { // negative - extend sign bit
	  i16 |= 0xfc00;
	}
	g_EvData.m_MotorPower = i16;
      }
      */

  return rc;
}


