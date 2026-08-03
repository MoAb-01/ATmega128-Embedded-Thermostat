#ifndef F_CPU
#define F_CPU 8000000UL // Set to 1000000UL if you haven't changed factory fuses
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

// =================================================
//     ATmega128 Pin Definitions //
// =================================================

// TM1637 Display on PORTD
#define TM1637_PORT     PORTD
#define TM1637_DDR      DDRD
#define TM1637_PIN      PIND
#define TM1637_CLK_PIN  PD2
#define TM1637_DIO_PIN  PD3

// Buttons on PORTB
#define BTN_PORT        PORTB
#define BTN_DDR         DDRB
#define BTN_PIN_REG     PINB
#define BTN_UP_PIN      PB0
#define BTN_DOWN_PIN    PB1
#define BTN_CONFIRM_PIN PB2

// MOSFET PWM on PORTB: PB4 (OC0 / Timer0 PWM Pin)
#define MOSFET_PIN      PB4

// NTC Thermistor on PORTF: PF0 (ADC0)
#define THERMISTOR_ADC_CH 0

// =================================================
//     NTC3950 & Thermostat State Variables
// =================================================

#define NUMSAMPLES 20
const float SERIES_RESISTOR     = 10000.0f;
const float NOMINAL_RESISTANCE  = 10000.0f;
const float NOMINAL_TEMPERATURE = 25.0f;
const float B_COEFFICIENT       = 3950.0f;

float targetTemp = 22.0f;
float currentTemp = 0.0f;
int timeDuration = 10; // Default 10 seconds

// --- PID Controller Variables ---
float Kp = 25.0f; // Proportional Gain
float Ki = 1.0f;  // Integral Gain
float Kd = 5.0f;  // Derivative Gain
float integral = 0.0f;
float previousError = 0.0f;

// --- Timers & Inputs ---
unsigned long previousMillis = 0;
unsigned long endTime = 0;
const long sensorInterval = 1000;

volatile unsigned long timer1_millis = 0;

ISR(TIMER1_COMPA_vect) {
	timer1_millis++;
}

unsigned long millis(void) {
	unsigned long m;
	cli();
	m = timer1_millis;
	sei();
	return m;
}

void delayWithWatchdog(unsigned long ms) {
	unsigned long start = millis();
	while (millis() - start < ms) {
		wdt_reset();
		_delay_ms(1);
	}
}

// =================================================
//      DYNAMIC TM1637 BIT-BANG DRIVER
// =================================================

static inline void tm1637_clk_low(void)  { TM1637_PORT &= ~(1 << TM1637_CLK_PIN); }
static inline void tm1637_clk_high(void) { TM1637_PORT |= (1 << TM1637_CLK_PIN); }
static inline void tm1637_dio_low(void)  { TM1637_PORT &= ~(1 << TM1637_DIO_PIN); }
static inline void tm1637_dio_high(void) { TM1637_PORT |= (1 << TM1637_DIO_PIN); }

void tm1637_start(void) {
	tm1637_clk_high();
	tm1637_dio_high();
	_delay_us(10);
	tm1637_dio_low();
	_delay_us(10);
	tm1637_clk_low();
	_delay_us(10);
}

void tm1637_stop(void) {
	tm1637_clk_low();
	tm1637_dio_low();
	_delay_us(10);
	tm1637_clk_high();
	_delay_us(10);
	tm1637_dio_high();
	_delay_us(10);
}

void tm1637_write_byte(uint8_t b) {
	for (uint8_t i = 0; i < 8; i++) {
		tm1637_clk_low();
		if (b & 0x01) tm1637_dio_high();
		else tm1637_dio_low();
		_delay_us(10);
		tm1637_clk_high();
		_delay_us(10);
		b >>= 1;
	}
	
	// Send 9th Clock for ACK
	tm1637_clk_low();
	tm1637_dio_high();
	_delay_us(10);
	tm1637_clk_high();
	_delay_us(10);
	tm1637_clk_low();
}

const uint8_t digitMAP[] = {
	0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f
};

const uint8_t SEG_MENU[]  = { 0x37, 0x79, 0x54, 0x3e };
const uint8_t SEG_CTR[]   = { 0x00, 0x39, 0x78, 0x50, 0x00 };
const uint8_t SEG_TIME[]  = { 0x78, 0x3e, 0x54, 0x79 };
const uint8_t SEG_SET[]   = { 0x6d, 0x79, 0x78, 0x00 };
const uint8_t SEG_TEMP[]  = { 0x78, 0x79, 0x54, 0x73 };
const uint8_t SEG_LINES[] = { 0x48, 0x48, 0x48, 0x48 };
const uint8_t SEG_ERROR[] = { 0x79, 0x50, 0x50, 0x50 };
const uint8_t SEG_BLANK[] = { 0x00, 0x00, 0x00, 0x00 };
const uint8_t SEG_DASH[]  = { 0x40, 0x40, 0x40, 0x40 };
const uint8_t SEG_DONE[]  = { 0x5e, 0x5c, 0x37, 0x79 }; // d, o, n, E

void tm1637_set_segments(const uint8_t segments[], uint8_t length, uint8_t pos) {
	tm1637_start();
	tm1637_write_byte(0x40); // Command: Write data, auto increment
	tm1637_stop();

	tm1637_start();
	tm1637_write_byte(0xC0 | pos); // Command: Address
	for (uint8_t k = 0; k < length; k++) {
		tm1637_write_byte(segments[k]);
	}
	tm1637_stop();

	tm1637_start();
	tm1637_write_byte(0x88 | 0x07); // Command: Display ON, Max Brightness
	tm1637_stop();
}

void tm1637_clear(void) {
	tm1637_set_segments(SEG_BLANK, 4, 0);
}

void tm1637_show_number_dec_ex(int num, uint8_t dots) {
	uint8_t digits[4];
	int temp = num;
	for (int i = 3; i >= 0; i--) {
		uint8_t d = temp % 10;
		digits[i] = digitMAP[d];
		if (i == 1 && (dots & 0x40)) {
			digits[i] |= 0x80; // Turn on colon
		}
		temp /= 10;
	}
	tm1637_set_segments(digits, 4, 0);
}

// =================================================
//        ADC & PWM PERIPHERALS
// =================================================

void adc_init(void) {
	ADMUX = 0;
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

void pwm_init(void) {
	DDRB |= (1 << MOSFET_PIN);
	TCCR0 = (1 << WGM00) | (1 << WGM01) | (1 << COM01) | (1 << CS02);
	OCR0 = 0;
}

void timer1_init(void) {
	TCCR1A = 0;
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
	OCR1A = (uint16_t)((F_CPU / (64UL * 1000UL)) - 1);
	TIMSK |= (1 << OCIE1A);
}

// =================================================
//                SCREEN DRAWER
// =================================================

void DrawScreen(char state) {
	tm1637_clear();
	switch (state) {
		case 'W': tm1637_set_segments(SEG_CTR, 4, 0);   break;
		case 'M': tm1637_set_segments(SEG_MENU, 4, 0);  break;
		case 'T': tm1637_set_segments(SEG_TIME, 4, 0);  break;
		case 'S': tm1637_set_segments(SEG_SET, 4, 0);   break;
		case 'P': tm1637_set_segments(SEG_TEMP, 4, 0);  break;
		case 'N': tm1637_set_segments(SEG_LINES, 4, 0); break;
		case 'E': tm1637_set_segments(SEG_BLANK, 4, 0); break;
		case 'e': tm1637_set_segments(SEG_ERROR, 4, 0); break;
		case 'A': tm1637_set_segments(SEG_DASH, 4, 0);  break;
		case 'D': tm1637_set_segments(SEG_DONE, 4, 0);  break;
	}
	delayWithWatchdog(1000);
}

float getTemperature(uint8_t pin);

// =================================================
//          INPUT & CONFIRMATION HANDLER
// =================================================

int IO(int startValue) {
	int localTarget = startValue;
	bool input = true;
	
	tm1637_show_number_dec_ex(localTarget, 0);

	while (input) {
		wdt_reset();

		if (!(BTN_PIN_REG & (1 << BTN_UP_PIN))) {
			localTarget += 1;
			tm1637_show_number_dec_ex(localTarget, 0);
			delayWithWatchdog(250);
		}
		else if (!(BTN_PIN_REG & (1 << BTN_DOWN_PIN))) {
			localTarget -= 1;
			if (localTarget < 0) localTarget = 0;
			tm1637_show_number_dec_ex(localTarget, 0);
			delayWithWatchdog(250);
		}
		else if (!(BTN_PIN_REG & (1 << BTN_CONFIRM_PIN))) {
			tm1637_set_segments(SEG_LINES, 4, 0);
			delayWithWatchdog(500);
			while (!(BTN_PIN_REG & (1 << BTN_CONFIRM_PIN))) { delayWithWatchdog(10); }
			input = false;
		}
	}
	return localTarget;
}

// =================================================
//               SETUP & INITIALIZATION
// =================================================

void setup(void) {
	adc_init();
	pwm_init();
	timer1_init();

	// Set buttons as Inputs with Internal Pullups
	BTN_DDR &= ~((1 << BTN_UP_PIN) | (1 << BTN_DOWN_PIN) | (1 << BTN_CONFIRM_PIN));
	BTN_PORT |= (1 << BTN_UP_PIN) | (1 << BTN_DOWN_PIN) | (1 << BTN_CONFIRM_PIN);

	// Set TM1637 pins as Outputs
	TM1637_DDR |= (1 << TM1637_CLK_PIN) | (1 << TM1637_DIO_PIN);

	// Read saved EEPROM target
	eeprom_read_block(&targetTemp, (const void*)0, sizeof(float));
	if (isnan(targetTemp) || targetTemp < 0.0f || targetTemp > 50.0f) {
		targetTemp = 22.0f;
	}

	wdt_enable(WDTO_2S);
	sei();

	OCR0 = 0; // PWM OFF

	DrawScreen('A'); // Show Dashes
	DrawScreen('W'); // Show Ctr
}

// =================================================
//                    MAIN LOOP
// =================================================

int main(void) {
	setup();
	int step = 0;

	while (1) {
		wdt_reset();

		if (step == 0) {
			// Display 'MENU'
			DrawScreen('M');
			step = 1;
		}
		else if (step == 1) {
			// Display 'tInE' then get Time Input
			DrawScreen('T');
			timeDuration = IO(timeDuration);
			if (timeDuration <= 0) timeDuration = 1; // Safety check
			step = 2;
		}
		else if (step == 2) {
			// Display 'tEnP' then get Temp Input
			DrawScreen('P');
			int tempInput = IO((int)targetTemp);
			targetTemp = (float)tempInput;
			
			// Save to EEPROM
			eeprom_update_block(&targetTemp, (void*)0, sizeof(float));
			
			DrawScreen('E'); // Blank screen
			
			// Calculate when the timer should end
			endTime = millis() + ((unsigned long)timeDuration * 1000UL);
			
			// Reset PID variables on start
			integral = 0.0f;
			previousError = 0.0f;
			
			step = 3;
		}
		else if (step == 3) {
			unsigned long currentMillis = millis();
			
			// 1. Check if Timer is still active
			if (currentMillis < endTime) {
				
				if (currentMillis - previousMillis >= sensorInterval) {
					previousMillis = currentMillis;
					
					currentTemp = getTemperature(THERMISTOR_ADC_CH);

					if (currentTemp == -999.0f) {
						OCR0 = 0;
						continue;
					}

					// --- PID Closed-Loop Control Calculation ---
					float error = targetTemp - currentTemp;
					
					// Accumulate integral (with anti-windup clamping)
					integral += error;
					if (integral > 100.0f) integral = 100.0f;
					if (integral < -100.0f) integral = -100.0f;
					
					float derivative = error - previousError;
					previousError = error;
					
					float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
					
					// Constrain PID output to valid PWM range (0 - 255)
					if (pidOutput > 255.0f) {
						pidOutput = 255.0f;
						} else if (pidOutput < 0.0f) {
						pidOutput = 0.0f;
					}
					
					OCR0 = (uint8_t)pidOutput; // Apply smooth PWM duty cycle

					// --- Display Formatting (Temp : TimeLeft) ---
					int timeLeft = (endTime - currentMillis) / 1000;
					if (timeLeft > 99) timeLeft = 99; // Cap at 99 so it fits on 2 digits
					
					int tempInt = (int)currentTemp;
					if (tempInt > 99) tempInt = 99;

					int combinedData = (tempInt * 100) + timeLeft;
					tm1637_show_number_dec_ex(combinedData, 0b01000000); // Colon turned ON
				}
			}
			// 2. Timer Expired
			else {
				OCR0 = 0; // Turn OFF MOSFET
				DrawScreen('D'); // Display 'donE'
				
				// Halt the system until restarted
				while (1) {
					delayWithWatchdog(1000);
				}
			}
		}
	}
	return 0;
}

// =================================================
//             THERMISTOR & FILTER LOGIC
// =================================================

float getTemperature(uint8_t pin) {
	float average_adc = 0;

	for (uint8_t i = 0; i < NUMSAMPLES; i++) {
		average_adc += (1023.0f - (float)adc_read(pin));
		delayWithWatchdog(1);
	}
	average_adc /= NUMSAMPLES;
	
	if (average_adc <= 0 || average_adc >= 1023) return -999.0f;

	float resistance = 1023.0f / average_adc - 1.0f;
	resistance = SERIES_RESISTOR / resistance;
	
	float temp = resistance / NOMINAL_RESISTANCE;
	temp = logf(temp);
	temp /= B_COEFFICIENT;
	temp += 1.0f / (NOMINAL_TEMPERATURE + 273.15f);
	temp = 1.0f / temp;
	temp -= 273.15f;

	static float xn1 = 25.0f;
	static float yn1 = 25.0f;
	static bool firstRun = true;

	if (firstRun) {
		xn1 = temp;
		yn1 = temp;
		firstRun = false;
	}

	float filteredTemp = (0.969f * yn1) + (0.0155f * temp) + (0.0155f * xn1);
	xn1 = temp;
	yn1 = filteredTemp;

	if (filteredTemp < 0.0f || filteredTemp > 100.0f) {
		OCR0 = 0; // Turn OFF MOSFET
		DrawScreen('e');
		return -999.0f;
	}
	
	return filteredTemp;
}
