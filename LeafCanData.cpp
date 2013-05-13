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
#include <EEPROM.h>
#include "LeafCAN.h"

LeafCanData::LeafCanData()
{
  m_DirtyBits = 0;
  /*
  m_PackVolts = 0;
  m_PackAmps = 0;
  m_W = 0;
  m_SOC = 0;
  m_GOMFuelBars = 0;
  m_Gids = 0;
  m_BatTemp1 = 0;
  m_BatTemp2 = 0;
  m_BatTemp3 = 0;
  m_BatTemp4 = 0;
  m_CPVmin = 0;
  m_CPVmax = 0;
  m_CPVavg = 0;
  m_SOC32 = 0;
  m_PackCap = 0;
  */

  m_Cur7BBFrameCnt = 0;
  m_Cur7BBReqFrameIdx = 255;

  // calculated constants
  m_WhL = ((((int32_t)GIDS_LB) * KW_FACTOR)+50L) / 100L;
  m_WhV = ((((int32_t)GIDS_VLB) * KW_FACTOR)+50L) / 100L;
  m_WhT = ((((int32_t)GIDS_TURTLE) * KW_FACTOR)+50L) / 100L;

  // startup settings
  m_DpKWh10_Low = DEF_DPKWH10_LOW; // lowest dist/KWh * 10;
  m_DpKWh10_Incr = DEF_DPKWH10_INCR; // DpKWh10 increment 
  m_CurDteType = EEPROM.read(EOFS_DTE_TYPE);
  if (!IsValidDteType(m_CurDteType)) m_CurDteType = DEFAULT_DTE_TYPE;
  m_TempUnit = EEPROM.read(EOFS_TEMP_UNIT);
  if (!IsValidTempUnit(m_TempUnit)) m_TempUnit = DEFAULT_TEMP_UNIT;
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
    if ((gids < 1023) && (gids != m_Gids)) {
      SetDirtyBits(DBF_GIDS|DBF_WH_REMAINING|DBF_FIXED_FUEL_BARS);
      m_Gids = gids;
      m_Wh = ((((int32_t)m_Gids) * KW_FACTOR)+50L) / 100L;
      m_FixedFuelBars = (uint8_t)(((((int32_t)m_Gids) - 24L)*130L)/257);
    }
    rc = 0;
  }
  else if (rxmsg->id.std == 0x7bb) {
    rc = Process7BBFrame(candata);
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

// group = 255 requests next msg in current group
uint8_t LeafCanData::Req79B(uint8_t group)
{
  st_cmd_t *txmsg = g_CanBus.GetMsgTx();
  uint8_t *candata = txmsg->pt_data;

  txmsg->cmd = CMD_TX_DATA;
  txmsg->ctrl.ide = 0; // CAN 2.0A
  txmsg->id.std = 0x79b;
  txmsg->dlc = 8;
  if (group != REQ_GROUP_NEXT_FRAME) {
    candata[0] = 0x02;
    candata[1] = 0x21;
    candata[2] = group;
    if (group == 1) {
      m_Cur7BBFrameCnt = 6;
    }
    else if (group == 2) {
      m_Cur7BBFrameCnt = 29;
    }
    else if (group == 3) {
      //      m_Cur7BBFrameCnt = 5;
      m_Cur7BBFrameCnt = 3; // we only need the 1st 3 lines
    }
    else if (group == 4) {
     m_Cur7BBFrameCnt = 3;
    }
    else if (group == 5) {
      m_Cur7BBFrameCnt = 11;
    }
    else if (group == 6) {
      m_Cur7BBFrameCnt = 4;
    }

    m_Cur79BGroup = group;
    m_Cur7BBRcvFrameIdx = 0;
    m_Cur7BBReqFrameIdx = 1;
  }
  else { // REQ_GROUP_NEXT_FRAME - req next msg
    candata[0] = 0x30;
    candata[1] = 0x01;
    candata[2] = 0x00;
  }
  candata[3] = 0xff;
  candata[4] = 0xff;
  candata[5] = 0xff;
  candata[6] = 0xff;
  candata[7] = 0xff;

#ifdef SDBG
  Serial.print("group: ");Serial.print(group,DEC);Serial.print(" framecnt: ");Serial.print(m_Cur7BBFrameCnt,DEC);
#endif

  if (!g_CanBus.Write()) {
    m_Last7BBreqTime = millis();
    return 0;
  }
  else {
    return 1;
  }
}

uint8_t LeafCanData::ReqNext7BBFrame()
{
  if (m_Cur7BBRcvFrameIdx && (m_Cur7BBReqFrameIdx < m_Cur7BBFrameCnt) &&
      ((millis()-m_Last7BBreqTime) >= REQ_INTERVAL_7BB)) {
    if (!Req79B(REQ_GROUP_NEXT_FRAME)) {// req next 7BB msg
      m_Cur7BBReqFrameIdx++;
    }
  }
}

uint8_t LeafCanData::Process7BBFrame(uint8_t *candata)
{
#ifdef SDBG
  Serial.print(" 7BB:");Serial.print(candata[0],HEX);
#endif
  if (m_Cur79BGroup == 1) {
    if (candata[0] == 0x10) {
      m_Cur7BBRcvFrameIdx = 1;
    }
    else if (candata[0] == 0x21) {
      m_Cur7BBRcvFrameIdx = 2;
    }
    else if (candata[0] == 0x22) {
      m_Cur7BBRcvFrameIdx = 3;
    }
    else if (candata[0] == 0x23) {
      m_Cur7BBRcvFrameIdx = 4;
    }
    else if ((m_Cur7BBRcvFrameIdx == 4) && (candata[0] == 0x24)) {
      m_PackHealth = candata[2];
      m_PackHealth <<= 8;
      m_PackHealth |= candata[3];
      m_SOC32 = candata[5];
      m_SOC32 <<= 8;
      m_SOC32 |= candata[6];
      m_SOC32 <<= 8;
      m_SOC32 |= candata[7];
      m_Cur7BBRcvFrameIdx = 5;
    }
    else if ((m_Cur7BBRcvFrameIdx == 5) && (candata[0] == 0x25)) {
      m_PackCap = candata[2];
      m_PackCap <<= 8;
      m_PackCap |= candata[3];
      m_PackCap <<= 8;
      m_PackCap |= candata[4];
      
      m_Cur7BBRcvFrameIdx = 6;
      m_Cur7BBFrameCnt = 0;
      //      Serial.print(" soc:");Serial.print(m_SOC32);Serial.print(" c:");Serial.print(m_PackCap);Serial.print(" h:");Serial.print(m_PackHealth);
      SetDirtyBits(DBF_SOC_CAP);
    }
  }
  else if (m_Cur79BGroup == 3) {
    if (candata[0] == 0x10) {
      m_Cur7BBRcvFrameIdx = 1;
    }
    else if ((m_Cur7BBRcvFrameIdx == 1) && (candata[0] == 0x21)) {
      m_CPVmax = candata[7];
      m_Cur7BBRcvFrameIdx = 2;
    }
    else if ((m_Cur7BBRcvFrameIdx == 2) && (candata[0] == 0x22)) {
      m_CPVmax <<= 8;
      m_CPVmax |= candata[1];
      m_CPVmin = candata[2];
      m_CPVmin <<= 8;
      m_CPVmin |= candata[3];

      m_Cur7BBRcvFrameIdx = 3;
      m_Cur7BBFrameCnt = 0;
      SetDirtyBits(DBF_CP_VOLTS);
    }
  }
  else if (m_Cur79BGroup == 4) {
    if (candata[0] == 0x10) {
      m_BattTemp1 = candata[6];
      m_Cur7BBRcvFrameIdx = 1;
    }
    else if ((m_Cur7BBRcvFrameIdx == 1) && (candata[0] == 0x21)) {
      m_BattTemp2 = candata[2];
      m_BattTemp3 = candata[5];
      m_Cur7BBRcvFrameIdx = 2;
    }
    else if ((m_Cur7BBRcvFrameIdx == 2) && (candata[0] == 0x22)) {
      m_BattTemp4 = candata[1];
      m_Cur7BBRcvFrameIdx = 3;
      m_Cur7BBFrameCnt = 0;
      SetDirtyBits(DBF_BATT_TEMP);
    }
  }

  return 0;
}

void LeafCanData::SaveEEPROM()
{
  if (DirtyBitsSet(DBF_EEPROM)) {
    ClearDirtyBits(DBF_EEPROM);
    EEPROM.write(EOFS_DTE_TYPE,m_CurDteType);
    EEPROM.write(EOFS_TEMP_UNIT,m_TempUnit);
  }
}
