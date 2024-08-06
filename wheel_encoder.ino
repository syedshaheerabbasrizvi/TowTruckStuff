// Timer and Counter example
// Author: Nick Gammon
// Date: 17th January 2012


// Input: Pin D5
// Note from Shaheer: I do not understand a good portion of this code, especially the work with interrupts, since I borrowed this from online to save time and effort. Perhaps I will set aside some time to understand and explain this, but for now, consider this as a black box that takes a typical digital signal (where 0-1 V is low and 4-5 V is high) and figures out its frequency. The larger the sampling time, the more accurate our frequency... but the less frequently we will be updated. This is not to say that low sampling time kills the accuracy, just that we will have to consider the average of the output data (for example, we are sending 50 Hz and the output constantly alternates between 0 and 100 Hz equally, averaging to 50 Hz). Trial and error is a good idea to find the sweet spot.

// these are checked for in the main program
volatile unsigned long timerCounts; // apparently the volatile keyword is to let the compiler know this can have very sharp changes? other than that, this is self explanatory
volatile boolean counterReady; // time to calculate after counting


// internal to counting routine
unsigned long overflowCount; // keeps track of overflows for Timer 1. will make more sense later
unsigned int timerTicks;
unsigned int timerPeriod;
double pulses = 0;


void startCounting (unsigned int ms)
{
  counterReady = false;         // time not up yet
  timerPeriod = ms;             // how many 1 ms counts to do
  timerTicks = 0;               // reset interrupt counter
  overflowCount = 0;            // no overflows yet


  // reset Timer 1 and Timer 2
  TCCR1A = 0;            
  TCCR1B = 0;              
  TCCR2A = 0;
  TCCR2B = 0;


  // Timer 1 - counts events on pin D5
  TIMSK1 = bit (TOIE1);   // interrupt on Timer 1 overflow


  // Timer 2 - gives us our 1 ms counting interval
  // 16 MHz clock (62.5 ns per tick) - prescaled by 128
  //  counter increments every 8 µs.
  // So we count 125 of them, giving exactly 1000 µs (1 ms)
  TCCR2A = bit (WGM21) ;   // CTC mode
  OCR2A  = 124;            // count up to 125  (zero relative!!!!)


  // Timer 2 - interrupt on match (ie. every 1 ms)
  TIMSK2 = bit (OCIE2A);   // enable Timer2 Interrupt


  TCNT1 = 0;      // Both counters to zero
  TCNT2 = 0;    


  // Reset prescalers
  GTCCR = bit (PSRASY);        // reset prescaler now
  // start Timer 2
  TCCR2B =  bit (CS20) | bit (CS22) ;  // prescaler of 128
  // start Timer 1
  // External clock source on T1 pin (D5). Clock on rising edge.
  TCCR1B =  bit (CS10) | bit (CS11) | bit (CS12);
}  // end of startCounting


ISR (TIMER1_OVF_vect)
{
  ++overflowCount;               // count number of Counter1 overflows  
}  // end of TIMER1_OVF_vect


//******************************************************************
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz


ISR (TIMER2_COMPA_vect)
{
  // grab counter value before it changes any more
  unsigned int timer1CounterValue;
  timer1CounterValue = TCNT1;  // see datasheet, page 117 (accessing 16-bit registers)
  unsigned long overflowCopy = overflowCount;


  // see if we have reached timing period
  if (++timerTicks < timerPeriod)
    return;  // not yet


  // if just missed an overflow
  if ((TIFR1 & bit (TOV1)) && timer1CounterValue < 256)
    overflowCopy++;


  // end of gate time, measurement ready


  TCCR1A = 0;    // stop timer 1
  TCCR1B = 0;    


  TCCR2A = 0;    // stop timer 2
  TCCR2B = 0;    


  TIMSK1 = 0;    // disable Timer1 Interrupt
  TIMSK2 = 0;    // disable Timer2 Interrupt
   
  // calculate total count
  timerCounts = (overflowCopy << 16) + timer1CounterValue;  // each overflow is 65536 more
  counterReady = true;              // set global flag for end count period
}  // end of TIMER2_COMPA_vect


void setup ()
{
  Serial.begin(115200); // serial is at baud rate 115200, which should be noted wherever else you use serial
  Serial.println("Frequency Counter"); // to show that we have initialized successfully
  startCounting(10); // Start counting for 10 ms (or another period)
} // end of setup


void loop ()
{
  if (counterReady) {
    // adjust counts by counting interval to give frequency in Hz
    float frq = (timerCounts *  1000.0) / timerPeriod; // frequency is obtained by dividing the number of detected pulses by the period, then multiplying by 1000 because the period is in ms
    pulses += timerCounts; // we may use this to keep track of total distance travelled
    // Send pulses and frequency to serial, separated by a comma
    Serial.println(frq, 2);   // 2 decimal places for float
    
    // restart counting
    startCounting(10); // Continue counting for 10 ms (or another period)
  }
}
