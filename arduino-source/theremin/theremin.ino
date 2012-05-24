/*
 * Arduino Based Theremin - Utilizes a 4.11 MHz LC Oscillator
 * Copyright (C) 2012  Brian McLaughlin GNU Public License (GPL) v3.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/ .
 *
 * Brian McLaughlin May 2012
 * Version 0.1
 * 
 * An implementation of a Theremin in the Arduino environment. Actually,
 * you might not recognize a lot of the code if you are used to the Arduino
 * programming environment only. I make a lot of call to the ATMega registers
 * on the chip and there is no work done in the @a loop() function as
 * all the work is done via timer interrupts!
 *
 * Original code came from Martin Nawrath for KHM in 2008. This is also where
 * the oscillator schematic originated. The code has been changed to the point
 * where there is almostnothing left of the original code, however the concept
 * and the schematic are greatly appreciated. http://goo.gl/kCJp6
 *
 * This code relies heavily on an understanding of the ATmega design, 
 * specifically the ATmega328 for this version. I have attempted to comment 
 * low-level functionality in code for better understanding but it is always 
 * best to refer to the datasheet to really get your best understanding. The
 * datasheet for the ATmega328, the chip used in the Arduino Uno, can be found
 * at: http://www.atmel.com/devices/atmega328.aspx
 *
 * WARNING:
 * Systems such as this, where timing is critical, are very sensitive to changes
 * in physical setup and code changes. If you make some code changes you may 
 * find it necessary to re-tune the system. It will probably work but the output
 * frequencies may be a little different than exepected.
 *
 * The ATmega328, the microcontroller used in the Arduino Uno, has a set of three
 * on-board Timer/Counters that are available for use. Timer/Counter 0 and
 * Timer/Counter 2 are both 8-bit and Timer/Counter 1 is 16-bit. Configuration of
 * the Timer/Counters is achieved here via direct manipulation of their control
 * registers. In this code, I use the following registers:
 * 
 * | Register | Description                                            |
 * | -------- | ------------------------------------------------------ |
 * | TCNTn    | Register that stores the actual Timer/Counter n value. |
 * | TCCRnA   | Timer/Counter n configuration register A.              |
 * | TCCRnB   | Timer/Counter n configuration register B.              |
 * | TIMSKn   | Timer/Counter n interrupt control mask.                |
 * | OCRnA    | Timer/Counter n Compare Register.                      |
 * 
 * See: ATmega238 Datasheet at http://www.atmel.com/devices/atmega328.aspx?tab=documents
 * See: Arduino 101: Timers and Interrupts http://letsmakerobots.com/node/28278
 */
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// Table of 256 sine values, one sine period, stored in flash memory.
PROGMEM  prog_uchar SINEWAVE[]  = {
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

// Define the pin assignments for the Arduino board.
const int PIN_RF_FREQ    = 5; /**< RF Frequency input from oscillator (RESERVED FOR TIMER 1 INPUT) */
const int PIN_AUDIO_OUT = 11; /**< PWM Based Audio Output from Arduino                             */

// Define other global constants
const unsigned long DDS_MODULUS         = 0x10000L;                // 2 ^ Size of an unsigned long (16 bits)                                */
const unsigned int  DDS_REFERENCE_CLOCK = F_CPU/510;               // The frequency setting for Timer/Counter 2                             */
const unsigned int  DDS_UPDATE_CLOCK    = DDS_REFERENCE_CLOCK/100; // Update frequency of the output tone.                                  */
const unsigned int  DDS_FINAL_SHIFT     = 8;                       // Number of bits to drop from the DDS phase accumulator at calculation. */
const double        AUDIO_OUTPUT_FILTER = 0.7;                     // Filter value on the audio output for smoothing of the data.           */
const unsigned int  AUDIO_MAX_OUTPUT_F  = 1000;                    // Maximum audio output frequency. Pretty high for pure tones.           */
const unsigned int  AUDIO_MIN_OUTPUT_F  = 60;                      // Minimum audio output frequency.                                       */

// Define volatile vairables passed across ISRs
// **TODO** Clean-Up - These don't need to be volatiles anymore
volatile unsigned long highest_freq = 0x0;
volatile unsigned long lowest_freq  = 0xFFFFFFFF;

/* Standard Arduino `setup()` function.
 *
 *  The `setup()` function for this program does the following:
 *  - Configure the Arduino General Purpose I/O pins for `INPUT` and `OUTPUT`.
 *  - Configure the ATmega Timer/Counters.
 */
void setup() {

  // Configure the Arduino I/O Pins
  pinMode(PIN_RF_FREQ,   INPUT);  // Set the RF pin for input
  pinMode(PIN_AUDIO_OUT, OUTPUT); // Set the audio out pin for output

  // Configure the timers
  cli(); // Disable interrupts before we configure the timers
  
  // Configure Timer/Counter 1 to count the input frequency on external port T0 (Arduino
  // pin 5) and to increment on a rising edge (CS12:CS11:CS10 to 1:1:1)
  TCCR1A = 0;                                 // Configure TCCR1A - Timer/Counter 1 Control Register A
  TCCR1B = (1<<CS12) | (1<<CS11) | (1<<CS10); // Configure TCCR1B - Timer/Counter 1 Control Register B
  TCNT1  = 0;                                 // Clear Timer/Counter 1
  
  // Configure Timer/Counter 2 to output a Phase Correct PWM signal. Bit settings are:
  // | Register | Bit ID | Description                                                                                 |
  // | -------- | ------ | ------------------------------------------------------------------------------------------- |
  // | TCCR2A   | COM2A1 | Clear OC2A on Compare Match when up-counting. Set OC2A on Compare Match when down-counting. |
  // | TCCR2A   | WGM20  | Pulse Width Modulation, Phase Correct                                                       |
  // | TCCR2B   | CS21   | Without CS20 & CS22 set, this configures the timer clock to the IO Clock/256                |
  TCCR2A = (1<<COM2A1) | (1<<WGM20); // Configure TCCR2A - Timer/Counter 2 Control Register A
  TCCR2B = (1<<CS21);                // Configure TCCR2B - Timer/Counter 2 Control Register B
  TIMSK2 |= (1<<TOIE2);              // Enable the Timer/Counter 2 Overflow Interrupt
  
  sei(); // Enable interrupts

}

/* The standard Arduino `loop()` function.
 *
 *  This is the normal Arduino `loop()` function where the main code execution
 *  in Arduino software normally occurs. In the case of this software, the 
 *  loop is left empty as the work is all accomplished in timers and a timer
 *  interrupt.
 */
void loop() {}

/* Timer/Counter 2 Overflow Interrupt Service Routine
 *
 *  This Interrupt Service Routine (ISR) will trigger once per PWM cycle. When it triggers,
 *  the routine updates the DDS phase accumulator and updates the output based on the next
 *  next jump value of the sine wave. This is configured for a frequency that is 1/256 as
 *  fast as the system clock, ~32 kHz on the Arduino Uno with a 16 MHz system clock. Once
 *  every 100th time through the cycle (about 320 Hz) the input frequency is checked and
 *  the output frequency is updated accordingly.
 */
ISR(TIMER2_OVF_vect) {
  static unsigned int  dds_phase_accumulator;
  static unsigned int  dds_tuning_word;
  static unsigned int  timer2_interrupt_counter;
  static unsigned int  last_output_frequency;
  static unsigned long frequency_input;
  static unsigned int  output_frequency;
  
  if(timer2_interrupt_counter == DDS_UPDATE_CLOCK) {
    // First check out the input frequency
    frequency_input  = TCNT1;                          // Get the current Timer/Counter 1 value
    highest_freq = max(highest_freq, frequency_input); // Store it as the highest if it is the highest seen
    lowest_freq  = min(lowest_freq, frequency_input);  // Store it as the lowest if it is the lowest value seen
    
    // Map the input frequency into a reasonable audio frequency range and smooth the output.
    output_frequency = map(frequency_input, lowest_freq, highest_freq, AUDIO_MIN_OUTPUT_F, AUDIO_MAX_OUTPUT_F);
    output_frequency = (output_frequency * (1-AUDIO_OUTPUT_FILTER)) + (last_output_frequency * AUDIO_OUTPUT_FILTER);
    last_output_frequency = output_frequency;
    
    // Calculate the new DDS tuning word and start counting again.
    dds_tuning_word =  (output_frequency * DDS_MODULUS) / DDS_REFERENCE_CLOCK;
    timer2_interrupt_counter = 0;
    TCNT1 = 0;

  } 
  
  // Update the tuning word.
  dds_phase_accumulator += dds_tuning_word;
  
  // Get the next value in the wave-form.
  OCR2A = pgm_read_byte_near(SINEWAVE + (dds_phase_accumulator >> DDS_FINAL_SHIFT));
  
  // Update the loop counter.
  timer2_interrupt_counter++;
  
}

