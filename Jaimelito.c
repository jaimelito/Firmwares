#define F_CPU 4800000UL
 
// PWM Mode
#define PHASE 0b00000001
#define FAST  0b00000011
 
/*
 * =========================================================================
 * Settings to modify per driver
 */
 
#define BLINK_ON_POWER      // blink once when power is received
                            // (helpful on e-switch lights, annoying on dual-switch lights)
#define VOLTAGE_MON         // Comment out to disable - ramp down and eventual shutoff when battery is low
#define MODES           0,4,10,120,255      // Must be low to high, and must start with 0
#define MODES2          0,2,32,125,255      // Must be low to high, and must start with 0, the defines the level for the secondary output. Comment out if no secondary output
#define MODE_PWM        0,PHASE,FAST,FAST,PHASE     // Define one per mode above. 0 tells the light to go to sleep
                             
#define ADC_LOW         130 // When do we start ramping
#define ADC_CRIT        120 // When do we shut the light off
#define ADC_DELAY       188 // Delay in ticks between low-bat rampdowns (188 ~= 3s)
 
#define MOM_EXIT_DUR    128 // .16ms each
 
#define FULL_MODE  -1  // A mode of "-1" will be interpreted as Full mode
 
/*
 * =========================================================================
 */
 
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>  
#include <avr/eeprom.h>
#include <avr/sleep.h>
//#include <avr/power.h>
 
 
#define SWITCH_PIN  PB3     // what pin the switch is connected to, which is Star 4
#define SWITCH_PIN2 PB4     // what pin the switch is connected to, which is Star 4
#define PWM_PIN     PB1
#define PWM_PIN2     PB0
#define VOLTAGE_PIN PB2
#define ADC_CHANNEL 0x01    // MUX 01 corresponds with PB2
#define ADC_DIDR    ADC1D   // Digital input disable bit corresponding with PB2
#define ADC_PRSCL   0x06    // clk/64
 
#define PWM_LVL1     OCR0B   // OCR0B is the output compare register for PB1
#define PWM_LVL2     OCR0A   // OCR0B is the output compare register for PB0
 
#define DB_PRES_DUR 0b00000001 // time before we consider the switch pressed (after first realizing it was pressed)
#define DB_REL_DUR  0b00001111 // time before we consider the switch released
                               // each bit of 1 from the right equals 16ms, so 0x0f = 64ms
 
// Switch handling
#define LONG_PRESS_DUR   128 // How many WDT ticks until we consider a press a long press                        // 32 is roughly .5 s
#define LOCK_PRESS_DUR   600
/*
 * The actual program
 * =========================================================================
 */
 
/*
 * global variables
 */
const uint8_t modes[]     = { MODES    };
const uint8_t modes2[]     = { MODES2};
const uint8_t mode_pwm[] = { MODE_PWM };
volatile int8_t mode_idx = 0;
volatile int8_t mode_idx2 = 0;
volatile uint8_t press_duration = 0;
volatile uint8_t press_duration2 = 0;
volatile uint8_t low_to_high = 1;
volatile uint8_t in_momentary = 0;
 
// Debounce switch press value
 
int is_pressed()
{
    // Keep track of last switch values polled
    static uint8_t buffer = 0x00;
    // Shift over and tack on the latest value, 0 being low for pressed, 1 for pulled-up for released
    buffer = (buffer << 1) | ((PINB & (1 << SWITCH_PIN)) == 0);
     
    return (buffer & DB_REL_DUR);
}
 
 int is_pressed2()
 {
	 // Keep track of last switch values polled
	 static uint8_t buffer2 = 0x00;
	 // Shift over and tack on the latest value, 0 being low for pressed, 1 for pulled-up for released
	 buffer2 = (buffer2 << 1) | ((PINB & (1 << SWITCH_PIN2)) == 0);
	 
	 return (buffer2 & DB_REL_DUR);
 }
 
 inline void next_mode() {
	 if (++mode_idx >= sizeof(modes)) {
		 // Wrap around
		 mode_idx = 1;
	 }
 }
 
 inline void prev_mode() {
	 if (mode_idx == 0) {
		 // Wrap around
		 mode_idx = sizeof(modes) - 1;
		 } else {
		 --mode_idx;
	 }
 }
inline void next_mode2() {
	if (++mode_idx2 >= sizeof(modes2)) {
		// Wrap around
		mode_idx2= 1;
	}
}

inline void prev_mode2() {
	if (mode_idx2 == 0) {
		// Wrap around
		mode_idx2 = sizeof(modes2) - 1;
		} else {
		--mode_idx2;
	}
}
 
inline void PCINT_on() {
    // Enable pin change interrupts
    GIMSK |= (1 << PCIE);
}
 
inline void PCINT_off() {
    // Disable pin change interrupts
    GIMSK &= ~(1 << PCIE);
}
 
// Need an interrupt for when pin change is enabled to ONLY wake us from sleep.
// All logic of what to do when we wake up will be handled in the main loop.
EMPTY_INTERRUPT(PCINT0_vect);
 
inline void WDT_on() {
    // Setup watchdog timer to only interrupt, not reset, every 16ms.
    cli();                          // Disable interrupts
    wdt_reset();                    // Reset the WDT
    WDTCR |= (1<<WDCE) | (1<<WDE);  // Start timed sequence
    WDTCR = (1<<WDTIE);               // Enable interrupt every 16ms
    sei();                          // Enable interrupts
}
 
inline void WDT_off()
{
    cli();                          // Disable interrupts
    wdt_reset();                    // Reset the WDT
    MCUSR &= ~(1<<WDRF);          // Clear Watchdog reset flag
    WDTCR |= (1<<WDCE) | (1<<WDE);  // Start timed sequence
    WDTCR = 0x00;                   // Disable WDT
    sei();                          // Enable interrupts
}
 
inline void ADC_on() {
    ADMUX  = (1 << REFS0) | (1 << ADLAR) | ADC_CHANNEL; // 1.1v reference, left-adjust, ADC1/PB2
    DIDR0 |= (1 << ADC_DIDR);                         // disable digital input on ADC pin to reduce power consumption
    ADCSRA = (1 << ADEN ) | (1 << ADSC ) | ADC_PRSCL;   // enable, start, prescale
}
 
inline void ADC_off() {
    ADCSRA &= ~(1<<7); //ADC off
}
 
void sleep_until_switch_press()
{
    // This routine takes up a lot of program memory :(
    // Turn the WDT off so it doesn't wake us from sleep
    // Will also ensure interrupts are on or we will never wake up
    WDT_off();
    // Need to reset press duration since a button release wasn't recorded
    press_duration = 0;
    // Enable a pin change interrupt to wake us up
    // However, we have to make sure the switch is released otherwise we will wake when the user releases the switch
    while (is_pressed()||is_pressed2()) {
        _delay_ms(16);
    }
    PCINT_on();
    // Enable sleep mode set to Power Down that will be triggered by the sleep_mode() command.
    //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    // Now go to sleep
    sleep_mode();
    // Hey, someone must have pressed the switch!!
    // Disable pin change interrupt because it's only used to wake us up
    PCINT_off();
    // Turn the WDT back on to check for switch presses
    WDT_on();
    // Go back to main program
}
 
// The watchdog timer is called every 16ms
ISR(WDT_vect) {
 
    static uint8_t  adc_ticks = ADC_DELAY;
    static uint8_t  lowbatt_cnt = 0;
 
    if (is_pressed()) {
        if (press_duration < 255) {
            press_duration++;
        }
     if (is_pressed2()) {
	     if (press_duration2 < 255) {
		     press_duration2++;
	     }
         
 
        if (press_duration == LONG_PRESS_DUR) {
            // Long press
            if (mode_idx == 0) {
                // TODO: if we were off, enter strobe mode
                mode_idx = 4;
            } else {
                // if we were on, turn off
                mode_idx = 0;
            }
        }
         
		if (press_duration2 == LONG_PRESS_DUR) {
			// Long press
			if (mode_idx2 == 0) {
				// TODO: if we were off, enter strobe mode
				mode_idx2 = 4;
				} else {
				// if we were on, turn off
				mode_idx2 = 0;
			}
		}
        // Just always reset turbo timer whenever the button is pressed
        //turbo_ticks = 0;
        // Same with the ramp down delay
        adc_ticks = ADC_DELAY;
     
    } else {
         
         
         
        // Not pressed
        if (press_duration > 0 && press_duration < LONG_PRESS_DUR) {
            // Short press
            if (mode_idx == 4) {
                // short-press from strobe goes to second-highest mode
                mode_idx = sizeof(modes)-4;
            }
            else { // regular short press
                if (low_to_high) {
                    next_mode();
                } else {
                    prev_mode();
                }   
            }
			 if (press_duration2 > 0 && press_duration2 < LONG_PRESS_DUR) {
				 // Short press
				 if (mode_idx2 == 4) {
					 // short-press from strobe goes to second-highest mode
					 mode_idx2 = sizeof(modes2)-4;
				 }
				 else { // regular short press
					 if (low_to_high) {
						 next_mode2();
						 } else {
						 prev_mode2();
					 }
				 }
        } else {
             
            // Only do voltage monitoring when the switch isn't pressed
        #ifdef VOLTAGE_MON
            if (adc_ticks > 0) {
                --adc_ticks;
            }
            if (adc_ticks == 0) {
                // See if conversion is done
                if (ADCSRA & (1 << ADIF)) {
                    // See if voltage is lower than what we were looking for
                    if (ADCH < ((mode_idx == 1) ? ADC_CRIT : ADC_LOW)) {
                        ++lowbatt_cnt;
                    } else {
                        lowbatt_cnt = 0;
                    }
                }
                 
                // See if it's been low for a while
                if (lowbatt_cnt >= 4) {
                    prev_mode();
                    lowbatt_cnt = 0;
                    // If we reach 0 here, main loop will go into sleep mode
                    // Restart the counter to when we step down again
                    adc_ticks = ADC_DELAY;
                }
                 
                // Make sure conversion is running for next time through
                ADCSRA |= (1 << ADSC);
            }
        #endif
        }
        press_duration = 0;
    }
}
 
int main(void)
{   
    // Set all ports to input, and turn pull-up resistors on for the inputs we are using
    DDRB = 0x00;
    PORTB = (1 << SWITCH_PIN)| (1 << SWITCH_PIN2);
 
    // Set the switch as an interrupt for when we turn pin change interrupts on
    PCMSK = (1 << SWITCH_PIN) | (1 << SWITCH_PIN2);
     
    // Set PWM pin to output
 
    DDRB = (1 << PWM_PIN) | (1 << PWM_PIN2);
 
    // Set timer to do PWM for correct output pin and set prescaler timing
    TCCR0A = 0x23; // phase corrected PWM is 0x21 for PB1, fast-PWM is 0x23
    TCCR0B = 0x01; // pre-scaler for timer (1 => 1, 2 => 8, 3 => 64...)
     
    // Turn features on or off as needed
    #ifdef VOLTAGE_MON
    ADC_on();
    #else
    ADC_off();
    #endif
    ACSR   |=  (1<<7); //AC off
     
 
     
#ifdef BLINK_ON_POWER
    // blink once to let the user know we have power
    TCCR0A = PHASE | 0b00100000;  // Only use the normal output
    PWM_LVL1 =60;
    PWM_LVL2 = 60;
    _delay_ms(3);
    PWM_LVL1 = 0;
	PWM_LVL2 = 0;
    _delay_ms(1);
#endif
 
    // Enable sleep mode set to Power Down that will be triggered by the sleep_mode() command.
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_until_switch_press();
     
    uint8_t last_mode_idx = 0;
    int8_t real_mode_idx = 0;
	 uint8_t last_mode_idx2 = 0;
	 int8_t real_mode_idx2 = 0;
     
    while(1) {
        // We will never leave this loop.  The WDT will interrupt to check for switch presses and 
        // will change the mode if needed.  If this loop detects that the mode has changed, run the
        // logic for that mode while continuing to check for a mode change.
        if (mode_idx != last_mode_idx) {
            real_mode_idx = mode_idx;
            // The WDT changed the mode.
            if (mode_idx == FULL_MODE) {
                // strobe should use second-highest mode
                real_mode_idx = sizeof(modes) - 4;
            } else if (mode_idx > 0) {
                // No need to change the mode if we are just turning the light off
                // Check if the PWM mode is different
                if (mode_pwm[last_mode_idx] != mode_pwm[mode_idx]) {
                    TCCR0A = mode_pwm[mode_idx] | 0b10100000;  // Use both outputs      
                }
            }
            PWM_LVL1     = modes[real_mode_idx];
            last_mode_idx = mode_idx;
            // Handle strobe mode
            if (mode_idx == FULL_MODE) {
                PWM_LVL1=255;
                TCCR0A=PHASE | 0b00100000;
             
            }
			if (mode_idx2 != last_mode_idx2) {
				real_mode_idx2 = mode_idx2;
				// The WDT changed the mode.
				if (mode_idx2 == FULL_MODE) {
					// strobe should use second-highest mode
					real_mode_idx2 = sizeof(modes2) - 4;
					} else if (mode_idx2 > 0) {
					// No need to change the mode if we are just turning the light off
					// Check if the PWM mode is different
					if (mode_pwm[last_mode_idx2] != mode_pwm[mode_idx2]) {
						TCCR0A = mode_pwm[mode_idx2] | 0b10100000;  // Use both outputs
					}
				}
				PWM_LVL2     = modes2[real_mode_idx2];
				last_mode_idx2 = mode_idx2;
				// Handle strobe mode
				if (mode_idx2 == FULL_MODE) {
					PWM_LVL2=255;
					TCCR0A=PHASE | 0b00100000;
					
				}

			if (real_mode_idx && real_mode_idx2 == 0){
                _delay_ms(1); // Need this here, maybe instructions for PWM output not getting executed before shutdown?
                // Go to sleep
                sleep_until_switch_press();
            }
        }
    }
}
		
	
 
    return 0; // Standard Return Code
        }
	}
}
