// -*- C++ -*-
/*
 * LeafCAN Firmware
 *
 * Copyright (c) 2012-2014 Sam C. Lin <lincomatic@hotmail.com>
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
#ifndef _CANBUSINTERFACE_H_
#define _CANBUSINTERFACE_H_
#include <can_lib.h>

// for m_LastReadStat/m_LastWriteStat
#define CAN_STAT_OK 0
#define CAN_STAT_ERROR 1
#define CAN_STAT_ACCEPT_TIMEOUT 2
#define CAN_STAT_COMPLETION_TIMEOUT 3

class CanBusInterface {
  st_cmd_t m_CanMsgRx; // rx message
  st_cmd_t m_CanMsgTx; // tx message
  uint8_t m_CanDataRx[8]; // rx data buffer
  uint8_t m_CanDataTx[8]; // tx data buffer

  uint8_t m_LastCanReadStat; // CAN_STAT_XX
  uint8_t m_LastCanWriteStat; // CAN_STAT_XX
public:
  unsigned long m_LastCanMsgRxMs; // millis() when the last Rx msg was received
  unsigned long m_LastCanMsgTxMs; // millis() when the last Rx msg was sent


  CanBusInterface();

  void Init();

  uint8_t Read();
  uint8_t Write();

  st_cmd_t *GetMsgRx() { return &m_CanMsgRx; }
  st_cmd_t *GetMsgTx() { return &m_CanMsgTx; }

  uint8_t GetLastReadStat() { return m_LastCanReadStat; }
  uint8_t GetLastWriteStat() { return m_LastCanWriteStat; }

  uint8_t IsActive() { return (((millis()-m_LastCanMsgRxMs) <= CAN_TIMEOUT) ? 1 : 0); }
};

#endif // _CANBUSINTERFACE_H_
