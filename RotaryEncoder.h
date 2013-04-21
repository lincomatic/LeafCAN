#ifndef _ROTARY_ENCODER_H_
#define _ROTARY_ENCODER_H_
#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

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
