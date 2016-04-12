/*
  tftSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Chentao Zhao.  All right reserved.

  This library is Elite Robot; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  

  Modified 8 July 2015 by Chentao Zhao
*/


#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <Arduino.h>

#include "Marlin.h"
//#include "tftSerial.h"

  void serialEvent2() __attribute__((weak));
  bool Serial2_available() __attribute__((weak));
  void serialEventRun(void)
  {
  if (Serial2_available && serialEvent2 && Serial2_available()) serialEvent2();
  }

// from here
// Constructors ////////////////////////////////////////////////////////////////
tftSerial::tftSerial(
  volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
  volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
  volatile uint8_t *ucsrc, volatile uint8_t *udr):
    _ubrrh(ubrrh), _ubrrl(ubrrl),
    _ucsra(ucsra), _ucsrb(ucsrb), _ucsrc(ucsrc),
    _udr(udr),
    _rx_buffer_head(0), _rx_buffer_tail(0),
    _tx_buffer_head(0), _tx_buffer_tail(0)
{
}

// Public Methods //////////////////////////////////////////////////////////////

void tftSerial::begin(unsigned long baud, byte config)
{
  // Try u2x mode first
  uint16_t baud_setting = (F_CPU / 4 / baud - 1) / 2;
  *_ucsra = 1 << U2X2;

  // hardcoded exception for 57600 for compatibility with the bootloader
  // shipped with the Duemilanove and previous boards and the firmware
  // on the 8U2 on the Uno and Mega 2560. Also, The baud_setting cannot
  // be > 4095, so switch back to non-u2x mode if the baud rate is too
  // low.
  if (((F_CPU == 16000000UL) && (baud == 57600)) || (baud_setting >4095))
  {
    *_ucsra = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  // assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register)
  *_ubrrh = baud_setting >> 8;
  *_ubrrl = baud_setting;

  _written = false;

  //set the data bits, parity, and stop bits
  *_ucsrc = config;
  
  sbi(*_ucsrb, RXEN2);
  sbi(*_ucsrb, TXEN2);
  sbi(*_ucsrb, RXCIE2);
  cbi(*_ucsrb, UDRIE2);
}
void tftSerial::end()
{
  // wait for transmission of outgoing data
  while (_tx_buffer_head != _tx_buffer_tail)
    ;

  cbi(*_ucsrb, RXEN2);
  cbi(*_ucsrb, TXEN2);
  cbi(*_ucsrb, RXCIE2);
  cbi(*_ucsrb, UDRIE2);
  
  // clear any received data
  _rx_buffer_head = _rx_buffer_tail;
}

int tftSerial::available(void)
{
  return ((unsigned int)(SERIAL_RX_BUFFER_SIZE_2 + _rx_buffer_head - _rx_buffer_tail)) % SERIAL_RX_BUFFER_SIZE_2;
}

int tftSerial::peek(void)
{
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  } else {
    return _rx_buffer[_rx_buffer_tail];
  }
}

int tftSerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  } else {
    unsigned char c = _rx_buffer[_rx_buffer_tail];
    _rx_buffer_tail = (rx_buffer_index_t)(_rx_buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE_2;
    return c;
  }
}






int tftSerial::availableForWrite(void)
{
#if (SERIAL_TX_BUFFER_SIZE_2>256)
  uint8_t oldSREG = SREG;
  cli();
#endif
  tx_buffer_index_t head = _tx_buffer_head;
  tx_buffer_index_t tail = _tx_buffer_tail;
#if (SERIAL_TX_BUFFER_SIZE_2>256)
  SREG = oldSREG;
#endif
  if (head >= tail) return SERIAL_TX_BUFFER_SIZE_2 - 1 - head + tail;
  return tail - head - 1;
}

void tftSerial::flush()
{
  // If we have never written a byte, no need to flush. This special
  // case is needed since there is no way to force the TXC (transmit
  // complete) bit to 1 during initialization
  //if (!_written)
  //  return;

  while (bit_is_set(*_ucsrb, UDRIE2) || bit_is_clear(*_ucsra, TXC2)) {
    if (bit_is_clear(SREG, SREG_I) && bit_is_set(*_ucsrb, UDRIE2))
	// Interrupts are globally disabled, but the DR empty
	// interrupt should be enabled, so poll the DR empty flag to
	// prevent deadlock
	if (bit_is_set(*_ucsra, UDRE2)){
		MYSERIAL.println("sending");
	  _tx_udr_empty_irq();}
  }
  // If we get here, nothing is queued anymore (DRIE is disabled) and
  // the hardware finished tranmission (TXC is set).
}

size_t tftSerial::write(uint8_t c)
{
  _written = true;
  // If the buffer and the data register is empty, just write the byte
  // to the data register and be done. This shortcut helps
  // significantly improve the effective datarate at high (>
  // 500kbit/s) bitrates, where interrupt overhead becomes a slowdown.
  if (_tx_buffer_head == _tx_buffer_tail && bit_is_set(*_ucsra, UDRE2)) {
    *_udr = c;
    sbi(*_ucsra, TXC2);
	//MYSERIAL.println("soon");
    return 1;
  }
  tx_buffer_index_t i = (_tx_buffer_head + 1) % SERIAL_TX_BUFFER_SIZE_2;
	
  // If the output buffer is full, there's nothing for it other than to 
  // wait for the interrupt handler to empty it a bit
  while (i == _tx_buffer_tail) {
    if (bit_is_clear(SREG, SREG_I)) {
      // Interrupts are disabled, so we'll have to poll the data
      // register empty flag ourselves. If it is set, pretend an
      // interrupt has happened and call the handler to free up
      // space for us.
      if(bit_is_set(*_ucsra, UDRE2))
		{MYSERIAL.println("leaking");
	_tx_udr_empty_irq();}
    } else {
      // nop, the interrupt handler will free up space for us
    }
  }

  _tx_buffer[_tx_buffer_head] = c;
  _tx_buffer_head = i;
	
  sbi(*_ucsrb, UDRIE2);
  
  return 1;
}

void tftSerial::writenum(uint16_t n) {

     write('0' + n) ;
}
// Interrupt Response as below

void tftSerial::_tx_udr_empty_irq(void)   
{
  // If interrupts are enabled, there must be more data in the output
  // buffer. Send the next byte
  unsigned char c = _tx_buffer[_tx_buffer_tail];
  _tx_buffer_tail = (_tx_buffer_tail + 1) % SERIAL_TX_BUFFER_SIZE_2;

  *_udr = c;

  // clear the TXC bit -- "can be cleared by writing a one to its bit
  // location". This makes sure flush() won't return until the bytes
  // actually got written
  sbi(*_ucsra, TXC2);

  if (_tx_buffer_head == _tx_buffer_tail) {
    // Buffer empty, so disable interrupts
    cbi(*_ucsrb, UDRIE2);
  }
}

// Actual interrupt handlers //////////////////////////////////////////////////////////////

void tftSerial::_rx_complete_irq(void)
{
  if (bit_is_clear(*_ucsra, UPE2)) {
    // No Parity error, read byte and store it in the buffer if there is
    // room
    unsigned char c = *_udr;
    rx_buffer_index_t i = (unsigned int)(_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE_2;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != _rx_buffer_tail) {
      _rx_buffer[_rx_buffer_head] = c;
      _rx_buffer_head = i;
    }
  } else {
    // Parity error, read byte but discard it
    *_udr;
  };
}

String tftSerial::hearfromtft()
{
	char* ctp_end;
	strcpy(ctp_end,"\r");
		return readStringUntil(*ctp_end);


}


tftSerial GSERIAL(&UBRR2H, &UBRR2L, &UCSR2A, &UCSR2B, &UCSR2C, &UDR2);

// Function that can be weakly referenced by serialEventRun to prevent
// pulling in this file if it's not otherwise used.

ISR(USART2_RX_vect)
{
  GSERIAL._rx_complete_irq();
}

ISR(USART2_UDRE_vect)
{
  GSERIAL._tx_udr_empty_irq();
}









bool GSERIAL_available() {
  return GSERIAL.available();
}





