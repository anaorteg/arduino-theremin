/* Arduino Theremin
 *
 * Therremin with TTL Oscillator 4.1 MHz
 * Timer1 for freauency measurement
 * Timer2 for gate time
 *
 * Author - Brian McLaughlin
 * Rev-   - Baseline from original source by Martin Nawrath
 *          for KHM 2008. http://goo.gl/kCJp6
 */

// Define Macros to set and clear bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Define Macro that clears all Timer/Counter1 interrupt flags.
#define CLEAR_ALL_TIMER1_INT_FLAGS    (TIFR1 = TIFR1)

// Arduino Pin Assignments
const int PIN_RF_FREQ       = 5; // RF Frequency input from oscillator
const int PIN_AUDIO_PWM_OUT = 8; // PWM Based Audio Output from Arduino

// Arduino setup function
void setup() {
  pinMode(PIN_RF_FREQ, INPUT);        // Set the RF input Pin for input
  pinMode(PIN_AUDIO_PWM_OUT, OUTPUT); // Set the audio output pin for outptu

  // hardware counter setup ( refer atmega168.pdf chapter 16-bit counter1)
  TCCR1A=0;                   // reset timer/counter1 control register A
  TCCR1B=0;                   // reset timer/counter1 control register A
  TCNT1=0;                    // counter value = 0

  // set timer/counter1 hardware as counter , counts events on pin T1 ( arduino pin 5)
  // normal mode, wgm10 .. wgm13 = 0
  sbi (TCCR1B, CS10);         // External clock source on T1 pin. Clock on rising edge.
  sbi (TCCR1B, CS11);
  sbi (TCCR1B, CS12);

  // timer2 setup / is used for frequency measurement gatetime generation
  // timer 2 presaler set to 256 / timer 2 clock = 16Mhz / 256 = 62500 Hz
  cbi (TCCR2B, CS20);
  sbi (TCCR2B, CS21);
  sbi (TCCR2B, CS22);

  //set timer2 to CTC Mode
  cbi (TCCR2A, WGM20);
  sbi (TCCR2A, WGM21);
  cbi (TCCR2B, WGM22);
  OCR2A = 124;         // CTC at top of OCR2A / timer2 interrupt when coun value reaches OCR2A value

  // interrupt control
  sbi (TIMSK2,OCIE2A);          // enable Timer2 Interrupt

}

volatile byte i_tics;
volatile byte f_ready ;
volatile byte mlt ;
unsigned int ww;

int cal;
int cal_max;

char st1[32];
long freq_in;
long freq_zero;
long freq_cal;

unsigned int dds;
int tune;

int cnt=0;

void loop() {
  cnt++;

  f_meter_start();

  tune = tune + 1;
  
  while (f_ready == 0) {          // wait for period length end (100ms) by interrupt
    PORTB=((dds+=tune) >> 15);    // kind of DDS tonegenerator / connect speaker to portb.0 = arduino pin8
  }
  
  tune = freq_in-freq_zero;

  if (cnt==10) {
    freq_zero = freq_in;
    freq_cal  = freq_in;
    cal_max   = 0;
  }

  // autocalibration
  if ((cnt % 20) == 0) {   // try autocalibrate after n cycles
    
    if (cal_max <= 2) {
      freq_zero = freq_in;
    }
    
    freq_cal = freq_in;
    cal_max  = 0;
  }
  
  cal = freq_in-freq_cal;
  cal = abs(cal);
  cal = max(cal, cal_max); 

  tune = abs(tune);
}


//******************************************************************
void f_meter_start() {
  f_ready = 0;          // reset period measure flag
  i_tics  = 0;          // reset interrupt counter
  sbi (GTCCR,PSRASY);   // reset presacler counting
  TCNT2 = 0;            // Reset Timer
  TCNT1 = 0;            // Reset Counter
  cbi (TIMSK0,TOIE0);   // Disable Timer0 again 
  sbi (TIMSK2,OCIE2A);  // Enable Timer2 Interrupt
  TCCR1B = TCCR1B | 7;  // Counter Clock source = pin T1 , start counting now
}

//******************************************************************
// Timer2 Interrupt Service is invoked by hardware Timer2 every 2ms = 500 Hz
//  16Mhz / 256 / 125 / 500 Hz
// here the gatetime generation for freq. measurement takes place: 
ISR(TIMER2_COMPA_vect) {

  if (i_tics == 50) {       // multiple 2ms = gate time = 100 ms
                            // end of gate time, measurement ready
    
    TCCR1B = TCCR1B & ~7;   // Gate Off  / Counter T1 stopped
    cbi (TIMSK2,OCIE2A);    // disable Timer2 Interrupt
    sbi (TIMSK0,TOIE0);     // ensable Timer0 again // millis and delay
    f_ready=1;              // set global flag for end count period

    // calculate now frequeny value
    freq_in = 0x10000 * mlt;// mukt #ovverflows by 65636
    freq_in += TCNT1;       // add counter1 value
    mlt = 0;

  }
  
  i_tics++;                 // count number of interrupt events
  if (TIFR1 & 1) {          // if Timer/Counter 1 overflow flag
    mlt++;                  // count number of Counter1 overflows
    sbi(TIFR1,TOV1);        // clear Timer/Counter 1 overflow flag
  }

}
