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
#include "LeafCAN.h"

CanBusInterface::CanBusInterface()
{
  m_LastCanMsgRxMs = 0;
  m_LastCanMsgTxMs = 0;

  m_CanMsgRx.cmd = CMD_RX;
  m_CanMsgRx.pt_data = m_CanDataRx;
  m_CanMsgTx.cmd = CMD_TX_DATA;
  m_CanMsgTx.pt_data = m_CanDataTx;
}

void CanBusInterface::Init()
{
  m_LastCanReadStat = CAN_STAT_OK;
  m_LastCanWriteStat = CAN_STAT_OK;

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


uint8_t CanBusInterface::Read()
{
  uint8_t canstat;
  unsigned long start = millis();
  uint8_t dumpit;
  if (m_LastCanReadStat == CAN_STAT_COMPLETION_TIMEOUT) {
    dumpit = 1;
  }
  else {
    dumpit = 0;
    // --- Enable Rx
    while(CAN.cmd(&m_CanMsgRx) != CAN_CMD_ACCEPTED) {
      if ((millis()-start) > CAN_TIMEOUT) {
	m_LastCanReadStat = CAN_STAT_ACCEPT_TIMEOUT;
	return m_LastCanReadStat;
      }
    }
  }

  // --- Wait for Rx completed
  while((canstat=CAN.get_status(&m_CanMsgRx)) == CAN_STATUS_NOT_COMPLETED) {
    if ((m_LastCanReadStat == 3) || ((millis()-start) > CAN_TIMEOUT)) { // don't loop here if already timed out so we don't block encoder
      m_LastCanReadStat = CAN_STAT_COMPLETION_TIMEOUT;
      return m_LastCanReadStat;
    }
  }

  if (dumpit || (canstat == CAN_STATUS_ERROR)) {
    m_LastCanReadStat = CAN_STAT_ERROR;
  }
  else {
    m_LastCanReadStat = CAN_STAT_OK;
    m_LastCanMsgRxMs = millis();
  }
  return m_LastCanReadStat;
}


uint8_t CanBusInterface::Write()
{
  uint8_t canstat;
  unsigned long start = millis();

  while (CAN.cmd(&m_CanMsgTx) != CAN_CMD_ACCEPTED) {
    if ((millis()-start) > CAN_TIMEOUT) {
      m_LastCanWriteStat = CAN_STAT_ACCEPT_TIMEOUT;
      return m_LastCanWriteStat;
    }
  }

  // --- Wait for Tx completed
  while ((canstat=CAN.get_status(&m_CanMsgRx)) == CAN_STATUS_NOT_COMPLETED) {
    if ((millis()-start) > CAN_TIMEOUT) {
      m_LastCanWriteStat = CAN_STAT_COMPLETION_TIMEOUT;      
      return m_LastCanWriteStat;
    }
  }

  if (canstat == CAN_STATUS_ERROR) {
    m_LastCanWriteStat = CAN_STAT_ERROR;
  }
  else {
    m_LastCanWriteStat = CAN_STAT_OK;
    m_LastCanMsgTxMs = millis();
  }

  return m_LastCanWriteStat;
}

