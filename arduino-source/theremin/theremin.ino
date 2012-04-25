/**
 * @file      theremin.ino
 * @brief     Arduino Based Theremin - Utilizes a 4.11 MHz LC Oscillator
 * @copyright Copyright (C) 2012  Brian McLaughlin GNU Public License (GPL) v3.
 *
 * @par
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * @par
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * @par
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/ .
 *
 * @date      2012
 * @author    Brian McLaughlin
 * @version   0.1.1
 * 
 * An implementation of a Theremin in the Arduino environment. Actually,
 * you might not recognize a lot of the code if you are used to the Arduino
 * programming environment only. I make a lot of call to the ATMega registers
 * on the chip and there is no work done in the @a loop() function as
 * all the work is done via timer interrupts!
 *
 * Original code came from Martin Nawrath for KHM in 2008. This is also where
 * the oscillator schematic originated. The code has been changed to the point
 * where there is nothing left of the original code, however the concept and
 * the schematic are greatly appreciated. <a href=" http://goo.gl/kCJp6"> Original
 * Project Website</a>
 *
 * @note
 * This code relies heavily on an understanding of the ATmega design, 
 * specifically the ATmega328 for this version. I have attempted to comment low-level 
 * functionality in code for better understanding but it is always best to refer
 * the datasheet to really get your best understanding. The datasheet for the
 * ATmega328, the chip used in the Arduino Uno, can be found at:
 * http://www.atmel.com/devices/atmega328.aspx
 *
 * @warning
 * Systems such as this, where timing is critical, are very sensitive to changes
 * in physical setup and code changes. If you make some code changes you may find
 * it necessary to re-tune the system. It will probably work but the output frequencies
 * may be a little different than exepected.
 * 
 * @warning
 * This program uses the Timer/Counter 0 which is utilized in the Arduino environment for
 * the \c delay functions. Keep this in mind if you adapt or use this code and then try to 
 * utilizes these functions as their use will be impacted.
 *
 * The ATmega328, the microcontroller used in the Arduino Uno, has a set of three on-board Timer/Counters
 * that are available for use. Timer/Counter 0 and Timer/Counter 2 are both 8-bit and Timer/Counter 1
 * is 16-bit. Configuration of the Timer/Counters is achieved here via direct manipulation of their
 * control registers. In this code, I use the following registers:
 * 
 * | Register | Description                                            |
 * | -------- | ------------------------------------------------------ |
 * | TCNTn    | Register that stores the actual Timer/Counter n value. |
 * | TCCRnA   | Timer/Counter n configuration register A.              |
 * | TCCRnB   | Timer/Counter n configuration register B.              |
 * | TIMSKn   | Timer/Counter n interrupt control mask.                |
 * | OCRnA    | Timer/Counter n Compare Register.                      |
 * 
 * @see [ATmega238 Datasheet](http://www.atmel.com/devices/atmega328.aspx?tab=documents)
 * @see [Arduino 101: Timers and Interrupts](http://letsmakerobots.com/node/28278)
 *
 * @todo Add a physical button interface to trigger an auto-calibration feature.
 */
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

/** @defgroup globals Defines global variables for the code. */

/** @defgroup globals-waveform Define the available waveforms.
 *  @ingroup globals
 *
 */
 
/**
 *  @brief Table of 256 sine values, one sine period, stored in flash memory.
 *  @hideinitializer
 */
PROGMEM  prog_uchar sinewave[]  = {
  127, 130, 133, 136, 139, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173,
  176, 178, 181, 184, 187, 190, 192, 195, 198, 200, 203, 205, 208, 210, 212, 215,
  217, 219, 221, 223, 225, 227, 229, 231, 233, 234, 236, 238, 239, 240, 242, 243,
  244, 245, 247, 248, 249, 249, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254,
  254, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 249, 249, 248, 247, 245,
  244, 243, 242, 240, 239, 238, 236, 234, 233, 231, 229, 227, 225, 223, 221, 219,
  217, 215, 212, 210, 208, 205, 203, 200, 198, 195, 192, 190, 187, 184, 181, 178,
  176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 143, 139, 136, 133, 130,
  127, 124, 121, 118, 115, 111, 108, 105, 102,  99,  96,  93,  90,  87,  84,  81,
   78,  76,  73,  70,  67,  64,  62,  59,  56,  54,  51,  49,  46,  44,  42,  39,
   37,  35,  33,  31,  29,  27,  25,  23,  21,  20,  18,  16,  15,  14,  12,  11,
   10,   9,   7,   6,   5,   5,   4,   3,   2,   2,   1,   1,   1,   0,   0,   0,
    0,   0,   0,   0,   1,   1,   1,   2,   2,   3,   4,   5,   5,   6,   7,   9,
   10,  11,  12,  14,  15,  16,  18,  20,  21,  23,  25,  27,  29,  31,  33,  35,
   37,  39,  42,  44,  46,  49,  51,  54,  56,  59,  62,  64,  67,  70,  73,  76,
   78,  81,  84,  87,  90,  93,  96,  99, 102, 105, 108, 111, 115, 118, 121, 124};
/** @} */

/** @defgroup globals-pins Define the pin assignments for the Arduino board.
 *  @ingroup globals
 *  @{
 */
const int PIN_RF_FREQ    = 5; /**< RF Frequency input from oscillator (RESERVED FOR TIMER 1 INPUT) */
const int PIN_AUDIO_OUT = 11; /**< PWM Based Audio Output from Arduino                             */
/** @} */

/** @defgroup globals-constants
 *  @ingroup globals
 *  @{
 */
const unsigned long long  DDS_MODULUS     = 0x100000000LL; /**< Size of an unsigned integer (16 bits)                                            */
const unsigned int  DDS_REFERENCE_CLOCK   = 80000;         /**< The frequency setting for Timer/Counter 0                           */
const unsigned int  DDS_FINAL_SHIFT       = 24;            /**< Number of bits to drop from the DDS phase accumulator at calculation.            */
const unsigned long OSCILLATOR_FREQUENCY  = 4110000L;      /**< The hardware resonator is tuned to 4.11 MHz                                      */
const unsigned int  GATE_SAMPLE_FREQUENCY = 10;            /**< Frequency in Hz to measure the input frequency                                   */
const unsigned int  AUDIO_FREQUENCY_SCALE = 10;            /**< Amount to bit-shift right the RF frequency to get to the audio range.            */
/** @} */

/** @defgroup globals-volatiles
 *  @ingroup globals
 *  @{
 */
volatile unsigned long timer1_overflow_counter = 0; /**< Timer/Counter 0 overflow counter. Volatile due to use in 2 ISR routines. */
/** @} */

/** @brief The standard Arduino `setup()` function.
 *
 *  The `setup()` function for this program does the following:
 *  - Configure the Arduino General Purpose I/O pins for `INPUT` and `OUTPUT`.
 *  - Configure the ATmega Timer/Counters.
 */
void setup() {

  // Configure the Arduino IO Pins
  pinMode(PIN_RF_FREQ,   INPUT);  // Set the RF pin for input
  pinMode(PIN_AUDIO_OUT, OUTPUT); // Set the audio out pin for output
  Serial.begin(9600);
  // Configure the timers
  cli();            // Disable interrups before we configure the timer interrupts

  // Configure Timer/Counter 0 to trigger an interrupt at the desired DDS clock rate
  // to act as the gate for frequency measurement and take the time to update the output
  // frequency of the DDS.
  TCCR0A  = (1<<WGM01);                   // Configure Timer/Counter 0 Control Register A
  TCCR0B  = (1<<CS02);                   // Configure Timer/Counter 0 Control Register B 
  TIMSK0  = (1<<OCIE0A);                  // Enable Timer/Counter 0 to trigger an interrupt on CTC A
  OCR0A   = F_CPU/DDS_REFERENCE_CLOCK;   //
  TCNT0   = 0x0000;                      // Reset timer/counter 0 value
  
  // Configure Timer/Counter 1 to to count the incoming rising edges from the external oscillator
  // for the frequency measurement.
  TCCR1A  = B00000000;                                      // Configure Timer/Counter 1 Control Register A
  TCCR1B  = (1<<CS12) | (1<<CS11) | (1<<CS10); // Configure Timer/Counter 1 Control Register B
  TIMSK1  = (1<<TOIE1);                                     // Enable Timer/Counter 1 to trigger an interrupt on overflow
  TCNT1   = 0x0000;                                         // Reset Timer/Counter 1 value
  
  // Configure Timer/Counter 2 to output a Phase Correct PWM signal.
  TCCR2A = (1<<COM2A1) | (1<<WGM20); // Configure Timer/Counter 2 Control Register A
  TCCR2B = (1<<CS20);                // Configure Timer/Counter 2 Control Register B
  //OCR2A  = 0;                        // Set the Timer/Counter 0 CTC Match Value
  
  sei();            // Enable interrupts
}

/** @brief The standard Arduino `loop()` function.
 *
 *  @details
 *  This is the normal Arduino `loop()` function where the main code execution
 *  in Arduino software normally occurs. In the case of this software, the 
 *  loop is left empty as the work is all accomplished in timers and a timer
 *  interrupt.
 */
void loop() {}

/** @brief     Timer/Counter 0 Interrupt Service Routine
  * @param[in] TIMER0_COMPA_vect Timer/Counter 0 COMPA Interrupt Vector
  * @result
  * Makes a frequency measurement based on the number of rising edges seen since the last
  * call and updates the DDS algorithim for any output frequency changes.
  * 
  */  
ISR(TIMER0_COMPA_vect) {
  // calculate new frequency value
  static unsigned long dds_phase_accumulator    = 0;
  static unsigned long dds_tuning_word          = 0;
  static unsigned int  timer0_interrupt_counter = 0;
  static unsigned int  output_frequency         = 0;
  static unsigned int  frequency_gate_count     = DDS_REFERENCE_CLOCK / GATE_SAMPLE_FREQUENCY;
  unsigned long long   dds_modulus_temp         = 0;
  unsigned long        frequency_input          = 0;
 
  
  if (timer0_interrupt_counter == frequency_gate_count) {
    frequency_input  = (TCNT1 + timer1_overflow_counter) * GATE_SAMPLE_FREQUENCY;  // convert to cycles/second
    output_frequency = frequency_input >> AUDIO_FREQUENCY_SCALE;
    dds_modulus_temp = output_frequency * DDS_MODULUS;
    dds_tuning_word =  dds_modulus_temp / DDS_REFERENCE_CLOCK;
    timer0_interrupt_counter = 0;
    timer1_overflow_counter = 0L;
    TCNT1 = 0;
  }
  
  dds_phase_accumulator += dds_tuning_word;
  OCR2A = pgm_read_byte_near(sinewave + (dds_phase_accumulator >> DDS_FINAL_SHIFT));
  timer0_interrupt_counter++;
  
}

/** @brief     Timer/Counter 1 Overflow Interrupt Service Routine
  * @param[in] TIMER1_OVF_vect Timer/Counter 1 Overflow Interrupt Vector
  * @result
  * Overflow counter is incremented and the overflow flag is cleared.
  */
ISR(TIMER1_OVF_vect) {
  timer1_overflow_counter += 0xFFFFL;
  TIFR1 |= (1<<TOV1);
}
