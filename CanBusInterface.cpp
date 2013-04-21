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

CanBusInterface::CanBusInterface()
{
  m_LastCanMsgRxMs = 0;

  m_CanMsgRx.cmd = CMD_RX;
  m_CanMsgRx.pt_data = m_CanDataRx;
  m_CanMsgRx.cmd = CMD_TX;
  m_CanMsgRx.pt_data = m_CanDataTx;
}

void CanBusInterface::Init()
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


uint8_t CanBusInterface::Read()
{
  uint8_t canstat;
  // --- Enable Rx
  while(CAN.cmd(&m_CanMsgRx) != CAN_CMD_ACCEPTED) {
#ifdef BACKLIGHT_PIN
    if (g_Brightness && ((millis()-m_LastCanMsgRxMs) >= BACKLIGHT_TIMEOUT)) {
      setBackLight(0);
    }
#endif // BACKLIGHT_PIN
  }
  // --- Wait for Rx completed
  while((canstat=CAN.get_status(&m_CanMsgRx)) == CAN_STATUS_NOT_COMPLETED) {
#ifdef BACKLIGHT_PIN
    if (g_Brightness && ((millis()-m_LastCanMsgRxMs) >= BACKLIGHT_TIMEOUT)) {
      setBackLight(0);
    }
#endif // BACKLIGHT_PIN
  }

  if (canstat == CAN_STATUS_ERROR) {
    return 1;
  }
  else {
    m_LastCanMsgRxMs = millis();
    return 0;
  }
}

