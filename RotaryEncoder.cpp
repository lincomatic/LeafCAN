#include "RotaryEncoder.h"

int8_t RotaryEncoder::enc_states[16] = {
  0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};



RotaryEncoder::RotaryEncoder()
{
}

void RotaryEncoder::Setup()
{
  /* Setup encoder pins as inputs */
  // set up as inputs
  ENC_DDR &= ~(ENC_BIT_A|ENC_BIT_B|ENC_BIT_BTN);
  // turn on pullups
  ENC_PORT |= ENC_BIT_A|ENC_BIT_B|ENC_BIT_BTN;

  m_tcnt = 0;
  m_old_AB = 0;

  m_lastDebounceTime = 0;
  m_lastbtnval = ENC_BIT_BTN;
  m_lastbtnstate = ENC_BIT_BTN; // 0=pressed

  ResetCount();
}

/*
 returns change in encoder state (-1,0,1) and btn bit
 */
int8_t RotaryEncoder::read_encoder(int8_t *btnval)
{
  int8_t encpin;

  m_old_AB <<= 2;                   //remember previous state
  encpin = ENC_PIN;
  *btnval = encpin & ENC_BIT_BTN;  
  m_old_AB |= ( encpin & 0x03 );  //add current state
  int8_t tval = enc_states[( m_old_AB & 0x0f )];

  // return nonzero only when we get 4 consecutive vals in the same direction
  if (tval) {
    m_tcnt += tval;
    if ((m_tcnt == 4) || (m_tcnt == -4)) {
      m_tcnt = 0;
      return tval;
    }
  }
  return 0;
}

// btnState = 0 = pressed,=1=released
// return value = encoder count 
int8_t RotaryEncoder::Read(int8_t *btnState=NULL)
{
  int8_t encval,btnval;

  /*encoder*/
  encval = read_encoder(&btnval);
  if( encval ) {
    m_count += encval;
  }

  /* encoder button */
  if (btnval != m_lastbtnval) {
    m_lastDebounceTime = millis();
    *btnState = m_lastbtnstate;
  }
  else if ((millis()-m_lastDebounceTime) > DEBOUNCE_DELAY) {
    *btnState = btnval;
    m_lastbtnstate = btnval;
  }
  else {
    *btnState = m_lastbtnstate;
  }

  m_lastbtnval = btnval;

  return m_count;
}

