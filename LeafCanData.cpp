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

  // calculated constants
  m_WhL = ((((int32_t)GIDS_LB) * KW_FACTOR)+50L) / 100L;
  m_WhV = ((((int32_t)GIDS_VLB) * KW_FACTOR)+50L) / 100L;
  m_WhT = ((((int32_t)GIDS_TURTLE) * KW_FACTOR)+50L) / 100L;

  // startup settings
  m_DpKWh10_Low = DEF_DPKWH10_LOW; // lowest miles/KWh * 10;
  m_DpKWh10_Incr = DEF_DPKWH10_INCR; // MpKWh10 increment 
  m_CurDteType = 'V';
  m_CurDteUnits = 'M';
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
      int32_t v10 = (((int32_t)rpv)/2L)*10L + ((rpv & 1) ? 5L : 0L); // volts * 10
      int32_t a10 = (((int32_t)rpa)/2L)*10L + ((rpa & 1) ? 5L : 0L); // amps * 10
      int32_t w = ((v10 * -a10) + ((a10 >= 0) ? -50L:50L))/100L;
      if (w != m_W) {
	SetDirtyBits(DBF_WATTS);
	m_W = w;
      }

    }
    rc = 0;
  }
  else if (rxmsg->id.std == 0x55b) {
    uint16_t soc10 = (candata[0] << 2) | (candata[1] >> 6);
    if (soc10 != m_SOC) {
      SetDirtyBits(DBF_SOC);
      m_SOC = soc10;
    }
    rc = 0;
  }
  else if (rxmsg->id.std == 0x5b9) {
    uint8_t gfb = candata[0] >> 3;
    if (gfb != m_GOMFuelBars) {
      SetDirtyBits(DBF_GOM_FUEL_BARS);
      m_GOMFuelBars = gfb;
    }
    rc = 0;
  }
  else if (rxmsg->id.std == 0x5bc)  {
    uint16_t gids = (candata[0] << 2) | (candata[1] >> 6);
    if (gids != m_Gids) {
      SetDirtyBits(DBF_GIDS|DBF_WH_REMAINING|DBF_FIXED_FUEL_BARS);
      m_Gids = gids;
      m_Wh = ((((int32_t)m_Gids) * KW_FACTOR)+50L) / 100L;
      m_FixedFuelBars = (uint8_t)(((((int32_t)m_Gids) - 24L)*130L)/257);
    }
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

#ifdef notyet

void sendReq() {
  uint8_t *candata = rxmsg->pt_data;
  txmsg->dlc = 8;

  candata[0] = 0x02;
  candata[1] = 0x21;
  candata[2] = 0x01;
  candata[3] = 0xff;
  candata[4] = 0xff;
  candata[5] = 0xff;
  candata[6] = 0xff;
  candata[7] = 0xff;

    static char data[8] = {0x02, 0x21, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff};
    if(reqMsgCnt<99){
        switch (reqMsgCnt){
            case 0:
                can1.monitor(false); // set to active mode
                can1SleepMode = 0; // enable TX
                data[0]=0x02; //change to request group 1
                data[1]=0x21;
                data[2]=0x01;
                break;
            case 6: // group 1 has 6 frames
                can1.monitor(false); // set to active mode
                can1SleepMode = 0; // enable TX
                data[0]=0x02; //change to request group 2 (cp data)
                data[1]=0x21;
                data[2]=0x02;
                break;
            case 35: // group 2 has 29 frames
                data[0]=0x02; //change to request group 3
                data[1]=0x21;
                data[2]=0x03;
                break;
            case 40: // group 3 has 5 frames
                data[0]=0x02; //change to request group 4 (temperature)
                data[1]=0x21;
                data[2]=0x04;
                break;
            case 43: // group 4 has 3 frames
                data[0]=0x02; //change to request group 5
                data[1]=0x21;
                data[2]=0x05;
                break;
            case 54: // group 5 has 11 frames
                reqMsgCnt = 99;
                can1SleepMode = 1; // disable TX
                can1.monitor(true); // set to snoop mode
                msgReq.detach(); // stop ticker
            default:
                data[0]=0x30; //change to request next line message
                data[1]=0x01;
                data[2]=0x00;
        }
        can1.write(CANMessage(0x79b, data, 8));
        reqMsgCnt++;
    }
}
#endif


