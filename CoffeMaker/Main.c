#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

typedef struct {
 volatile uint8_t seconds;
 volatile uint8_t minutes;
 volatile uint8_t hours;
} time_s;

volatile uint8_t heaterOn = 0;
volatile uint8_t autoOn = 0;

time_s time = {0,23,23};
time_s alarm = {0,0,22};
uint8_t blinkingDotsValue = 0;

ISR(TIMER2_OVF_vect) {
	time.seconds++;
	if (time.seconds == 60) {
		time.seconds = 0;
		time.minutes++;
		if (time.minutes == 60) {
			time.minutes = 0;
			time.hours++;
			if (time.hours == 24) {
				time.hours = 0;
			}
		}
	}
	if(autoOn){
		if(alarm.hours == time.hours     &&
		   alarm.minutes == time.minutes &&
		   alarm.seconds == time.seconds)
			heaterOn = 1;
	}
}

/*Buttons definition*/
#define BUTTON_DDR DDRB
#define BUTTON_PIN PINB
#define BUTTON_PORT PORTB
#define BUTTON_DOWN(button)(!(BUTTON_PIN & (1<<button)))

#define AUTO  0
#define HOUR  1
#define MIN   2
#define PROG  3
#define ONOFF 4

static inline void initButtons(){
	BUTTON_DDR &= ~((1<<AUTO) | (1<<HOUR) | (1<<MIN) | (1<<PROG) | (1<<ONOFF));
	BUTTON_PORT |= ((1<<AUTO) | (1<<HOUR) | (1<<MIN) | (1<<PROG) | (1<<ONOFF));
}

/*heater pin*/
#define HEATER_COIL_PIN  5
#define HEATER_COIL_PORT PORTB
#define HEATER_COIL_DDR DDRB


static inline void initHeater(){
	HEATER_COIL_DDR |= (1<<HEATER_COIL_PIN);
	HEATER_COIL_PORT &= ~(1<<HEATER_COIL_PIN);
}

/*status diodes*/
#define DIODE_PORT PORTC
#define DIODE_DDR DDRC
#define BLUE_AUTO_DIODE 5
#define RED_RUN_DIODE 4

static inline void initDiodes(){
	DIODE_DDR  |= (1<<BLUE_AUTO_DIODE);
	DIODE_PORT |= (1<<BLUE_AUTO_DIODE);
	DIODE_DDR  |= (1<<RED_RUN_DIODE);
	DIODE_PORT |= (1<<RED_RUN_DIODE);

}

/*LED display*/
#define DISPLAY_CATODE_PORT PORTD
#define DISPLAY_CATODE_DDR DDRD
#define DISPLAY_ANODE_DDR DDRC
#define DISPLAY_ANODE_PORT PORTC

#define AN1 (1<<0)
#define AN2 (1<<1)
#define AN3 (1<<2)
#define AN4 (1<<3)

#define SEG_A (1<<0)
#define SEG_B (1<<1)
#define SEG_C (1<<2)
#define SEG_D (1<<3)
#define SEG_E (1<<4)
#define SEG_F (1<<5)
#define SEG_G (1<<6)
#define SEG_DP (1<<7)

uint8_t digits[15]  = {
    ~(SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F),           // 0
    ~(SEG_B|SEG_C),                                   // 1
    ~(SEG_A|SEG_B|SEG_D|SEG_E|SEG_G),                 // 2
    ~(SEG_A|SEG_B|SEG_C|SEG_D|SEG_G),                 // 3
    ~(SEG_B|SEG_C|SEG_F|SEG_G),                       // 4
    ~(SEG_A|SEG_C|SEG_D|SEG_F|SEG_G),                 // 5
    ~(SEG_A|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G),           // 6
    ~(SEG_A|SEG_B|SEG_C|SEG_F),                       // 7
    ~(SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G),     // 8
    ~(SEG_A|SEG_B|SEG_C|SEG_D|SEG_F|SEG_G),           // 9
};

#define TIMER0_START_VALUE 6

static inline void initDisplay() {
	DISPLAY_CATODE_DDR = 0xff;
	DISPLAY_CATODE_PORT = 0x00;
	DISPLAY_ANODE_DDR |= AN1 | AN2 | AN3 | AN4;
	DISPLAY_ANODE_PORT |= AN1 | AN2 | AN3 | AN4;
	/*timer 0 - blinkig dots and anode switching*/
	TIMSK |= (1 << TOIE0);            //overflow
	TCCR0 |= (1 << CS01) | (1 << CS00); //clk, 1024 prescaler
	TCNT0 = TIMER0_START_VALUE;
}


inline uint8_t digitToLed(uint8_t digit){
	return digits[digit];
}

volatile uint8_t activeDigit = 0;
volatile uint8_t timer0Count = 0;

ISR(TIMER0_OVF_vect){
	TCNT0 = TIMER0_START_VALUE;
	/*anode switching*/
	timer0Count++;
	if(timer0Count > 249){
		timer0Count=0;
		blinkingDotsValue = !blinkingDotsValue;
	}

	time_s *shownTime = &time;
	if(BUTTON_DOWN(AUTO)) shownTime = &alarm;

	/*eliminate ghosting*/
	DISPLAY_CATODE_PORT = 0xff;
	/*show next digit*/
	switch(activeDigit){
	case 0:
		DISPLAY_ANODE_PORT |= AN1;
		DISPLAY_ANODE_PORT &= ~(AN2 | AN3 | AN4);
		DISPLAY_CATODE_PORT = digitToLed(shownTime->minutes % 10);
		break;
	case 1:
		DISPLAY_ANODE_PORT |= AN2;
		DISPLAY_ANODE_PORT &= ~(AN1 | AN3 | AN4);
		DISPLAY_CATODE_PORT = digitToLed((shownTime->minutes - (shownTime->minutes % 10))/10);
		break;
	case 2:
		DISPLAY_ANODE_PORT |= AN3;
		DISPLAY_ANODE_PORT &= ~(AN1 | AN2 | AN4);
		DISPLAY_CATODE_PORT = digitToLed(shownTime->hours % 10);
		//blinking
		if(blinkingDotsValue) DISPLAY_CATODE_PORT &= ~SEG_DP;
		else DISPLAY_CATODE_PORT |= SEG_DP;
		break;
	case 3:
		DISPLAY_ANODE_PORT |= AN4;
		DISPLAY_ANODE_PORT &= ~(AN1| AN2 | AN3);
		uint8_t h = (shownTime->hours - (shownTime->hours % 10))/10;
		DISPLAY_CATODE_PORT = h ? digitToLed(h) : 0xff;
		break;

	default:
		break;
	}

	activeDigit++;
	activeDigit = activeDigit % 4;
}


inline static uint8_t debounceCheckButton(uint8_t button){
	if(BUTTON_DOWN(button)){
		_delay_ms(100);
		if(BUTTON_DOWN(button)) return 1;
	}
	return 0;
}

/*Adjust time by pressing and holding hour or minute button.
 * Incrementation delay gradually decreases with time       */
void adjustTime(time_s* target){
	uint16_t delayLoops = 200;
	while (debounceCheckButton(HOUR)) {
		target->hours++;
		if(target->hours > 23) target->hours = 0;
		if (delayLoops > 10) delayLoops -= 10;
		for (uint8_t i = 0; i < delayLoops; ++i) {
			_delay_ms(5);
		}
	}
	delayLoops = 200;
	while (debounceCheckButton(MIN)) {
		target->minutes++;
		if(target->minutes > 59) target->minutes= 0;
		if (delayLoops > 10) delayLoops -= 10;
		for (uint8_t i = 0; i < delayLoops; ++i) {
			_delay_ms(5);
		}
	}
}

static void initRTC() {
	DIODE_DDR |= (1 << BLUE_AUTO_DIODE);
	DIODE_PORT &= ~(1 << BLUE_AUTO_DIODE);
	/* Wait for external clock crystal to stabilize */
	for (uint8_t i = 0; i < 0x40; i++) {

		for (unsigned int j = 0; j < 0xFFFF; j++) {
			/* Do a nop instruction to keep the empty
			 * loop from being optimized away */
			asm volatile("nop");
		}
	}

	/* Make sure all TC2 interrupts are disabled */
	TIMSK &= ~((1 << TOIE2) | (1 << OCIE2));
	/* set Timer/counter2 to be asynchronous from the CPU clock.
	 * This will clock TC2 from the external 32,768 kHz crystal. */
	ASSR |= (1 << AS2);

	/* Reset timer */
	TCNT2 = 0;

	/* Prescale the timer to be clock source/128 to make */
	/* TC2 overflow precisely once every second. */
	TCCR2 = (1 << CS00) | (1 << CS02);

	//DIODE_PORT |= (1 <<BLUE_AUTO_DIODE);

	/* Wait until TC2 is updated */
	while (ASSR & ((1 << TCN2UB) | (1 << OCR2UB) | (1 << TCR2UB))) {}
	//TCN2UB, TCR2UB never

	DIODE_PORT |= (1 <<BLUE_AUTO_DIODE);


	/* Enable Timer/Counter2 Overflow Interrupts */
	TIMSK |= (1 << TOIE2);
}
int main(void) {

	cli();
	initButtons();
	initDiodes();
	initHeater();
	initDisplay();
	initRTC();
    sei();

	while (1){

		/*enable heater if needed */
		if(debounceCheckButton(ONOFF)) {
			heaterOn = !heaterOn;
			DIODE_PORT &= ~(1<<RED_RUN_DIODE); //doubled for faster diode switching
			_delay_ms(200);
		}
		if(heaterOn){
			HEATER_COIL_PORT |= (1<<HEATER_COIL_PIN);
			DIODE_PORT &= ~(1<<RED_RUN_DIODE);
		}
		else {
			HEATER_COIL_PORT &= ~(1<<HEATER_COIL_PIN);
			DIODE_PORT |= (1<<RED_RUN_DIODE);
		}


		/*enable auto if needed */
		if(debounceCheckButton(AUTO)){
			autoOn = !autoOn;
			DIODE_PORT &= ~(1<<BLUE_AUTO_DIODE);
			_delay_ms(200);
		}
		/*status diodes*/
		if(autoOn) DIODE_PORT &= ~(1<<BLUE_AUTO_DIODE);
		else DIODE_PORT |= (1<<BLUE_AUTO_DIODE);

		/*setting time*/
		while(debounceCheckButton(PROG)){
			adjustTime(&time);
		}
		/*setting alarm*/
		while(debounceCheckButton(AUTO)){
			adjustTime(&alarm);
		}
	}

}
