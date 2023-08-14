/*!
 *  @file Fast_TLC5947.h
 *
 * 	Fast 24-channel PWM/LED driver
 *
 * 	This is a faster library for the Adafruit 24-channel PWM/LED driver:
 * 	http://www.adafruit.com/products/1429
 *
 *  These drivers uses SPI to communicate, 3 pins are required to
 *  interface: Data, Clock and Latch.
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  Written by Limor Fried/Ladyada for Adafruit Industries.
 *  Modified by Gordy French
 *
 *	BSD license (see license)
 */
 */

#ifndef _Fast_TLC5947_H
#define _Fast_TLC5947_H

#include <Arduino.h>
#include <FastLED.h>

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            TLC5947 24-channel PWM/LED driver
 */
template <uint8_t DATA_PIN, uint8_t CLOCK_PIN, uint8_t LATCH_PIN>
class Fast_TLC5947
{
public:
  Fast_TLC5947()
  {
    pwmbuffer = (uint16_t *)malloc(2 * 24);
    memset(pwmbuffer, 0, 2 * 24);
  }
  ~Fast_TLC5947()
  {
    free(pwmbuffer);
  }

  boolean begin(void)
  {
    if (!pwmbuffer)
      return false;

    _data.setOutput();
    _clock.setOutput();
    _latch.setOutput();
    _latch.lo();

    return true;
  }

  void setPWM(uint16_t chan, uint16_t pwm)
  {
    if (pwm > 4095)
      pwm = 4095;
    if (chan >= 24)
      return;
    pwmbuffer[chan] = pwm;
  }

  void write()
  {
    _latch.lo();
    // 24 channels per TLC5974
    for (int16_t c = 24 - 1; c >= 0; c--)
    {
      // 12 bits per channel, send MSB first
      for (int8_t b = 11; b >= 0; b--)
      {
        _clock.lo();

        if (pwmbuffer[c] & (1 << b))
          _data.hi();
        else
          _data.lo();

        _clock.hi();
      }
    }
    _clock.lo();
    _latch.hi();
    _latch.lo();
  }

private:
  uint16_t *pwmbuffer;
  FastPin<DATA_PIN> _data;
  FastPin<CLOCK_PIN> _clock;
  FastPin<LATCH_PIN> _latch;
};

#endif
