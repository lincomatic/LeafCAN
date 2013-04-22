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
#ifndef _ROTARY_ENCODER_H_
#define _ROTARY_ENCODER_H_

/*
 Rotary encoder with polling reads.
 for 2 channel gray code (quadrature) encoder ... 4 transitions/click
 code adapted from http://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino
 by Sam C. Lin 20121125
 20121012 SCL - changed to RotaryEncoder class
 
 For interrupt-driven see http://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros
 
 This code only works if ENC_A and ENC_B are connected to pins 0,1 of the selected port
 switch may be any other bit on the same port
 if they are not pins 0,1 the bits need to be shifted there in the line
 old_AB |= ( ENC_PIN & 0x03 );  //add current state
 */

#define ENC_BIT_A (1<<0)
#define ENC_BIT_B (1<<1)
#define ENC_BIT_BTN (1<<2)
#define ENC_DDR DDRC
#define ENC_PIN PINC
#define ENC_PORT PORTC
#define DEBOUNCE_DELAY 50 // ms

class RotaryEncoder {
  static int8_t enc_states[16];
  int8_t m_tcnt;
  uint8_t m_old_AB;
  int8_t m_count;
  int8_t m_lastbtnval;
  int8_t m_lastbtnstate;
  long m_lastDebounceTime;

  int8_t read_encoder(int8_t *btnval);
public:
  RotaryEncoder();
  void Setup();
  int8_t Read(int8_t *btnState);
  void ResetCount() { 
    m_count = 0; 
  }
};

#endif // _ROTARY_ENCODER_H_
