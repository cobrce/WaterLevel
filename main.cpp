// #define TEST // uncomment to enable test mode for simulation
#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "MAX7219_attiny.h"
#include "font.h"
#include "errors.h"
#include "ShiftRegister.h"
#include "calibrate.h"
#include "sevenSegmentsFSM.h"

// Pins definition
#define HB_LED PD3 // heartbeat LED
#define PIN_UART_TX PD1
#define PIN_I2C_SDA PC4
#define PIN_I2C_SCL PC5
// VL53L0X sensor library
#include "vl53l0x-non-arduino/VL53L0X.h"
#include "vl53l0x-non-arduino/util/i2cmaster.h"
#include "vl53l0x-non-arduino/util/debugPrint.h"
#include "vl53l0x-non-arduino/util/millis.h"

#define __COMPILING_AVR_LIBC__ 1 // just to avoid some intellisence annoying messages

#ifndef FALSE
#define FALSE 0
#define TRUE 1
#endif // !FALSE

// some constants
#define SENSOR_HEIGHT 30
#define FULL_WATER 190
#define TOO_FAR_HEIGHT (FULL_WATER + SENSOR_HEIGHT + 10)
#define TOO_CLOSE_HEIGHT 10
#define HYSTERESIS 5

uint16_t EEMEM EE_FullHeight = FULL_WATER + SENSOR_HEIGHT; // eeprom variable in case we calibrate and save the actual values
volatile uint16_t FullHeight = 0;
volatile uint16_t distance;       // decalred as global for debug
volatile uint16_t levelInPercent; // same as above, but this is a claculated percentage of water level

// calculate a bargraph representation of a value then shift it to fit the shift register
// the bargraph contains 10 leds, the percentage is rounded up to 10s, example : 51 is displayed as 6leds, 0 is 1 led, 34 is 4 leds..etc
uint32_t calculateBarGraph(uint16_t valueInPercent, uint8_t shift)
{

    if (valueInPercent > 100) // check limits
        valueInPercent = 100;
    uint32_t result = 0;

    uint8_t valueInTens = ((valueInPercent / 10) + 1) % 11; // round up the percents to tens

    while (valueInTens--) // for each "1" in tens (i.e for each 10 in percents) light an led
        result = (result << 1) | 1;

    return result << shift; // shift the final result to the desired bit position (see the call lcoation for more info)
}

// a stack of detected errors to be displayed in in the seven segment
uint8_t errors[4] = {0};


// are we displaying errors or level percentage?
enum
{
    SevenSegDisplayingError,
    SevenSegDisplayingWaterLevel
} StateOfSevenSeg = SevenSegDisplayingWaterLevel;

// swap the order of bits, due to the way the bargraph is wired in the PCB
uint8_t swapBits(uint8_t input)
{
	uint8_t result = 0;
	for (int i = 0; i < 8; i++)
		result = (result << 1) | ((input>> i) & 1);
	return result;
}

// the seven segment display system is based on a finite state machine
// first either the character "P" (for percents) or "F" (for faults) is displayed
// then follwed by each number of the percentage of water the error code resp.
// each character is displayed for 500ms then a 100ms of blank to mark the transition between characters,
// for example 33% is displayed as : 'P' [short_blank] '0' [short_blank] '3' [short_blank] '3' [long_blank], this way the two identical numbers can be told apart
//
// the use of a finite state machine makes easy to keep track of what should be dispalyed next since the updated is called by
// a timer interrupt function, and no loop should block the interrupt thread
SevenSegmentsFSM levelDisplayFSM(3, SevenSegP); // initialize the level display finite state machine as 3 character display
SevenSegmentsFSM errorDisplayFSM(2, SevenSegF); // ...as a 2 character dispaly


// event handler executed when the error display fsm finishes executing the last phase
void errorDisplayFSM_OnLastPhaseDoneHandler()
{
    StateOfSevenSeg = SevenSegDisplayingWaterLevel; // switch to display level, if an other error is still in the stack the display update function will automatically switch back to displaying errors the next time it's called
    levelDisplayFSM.Reinit();                       // reinit the water level display fsm (to display the message from the beginning)
}

// even handler executed when the level display fms starts executing the first phase
void levelDisplayFSM_OnFirstPhaseStarted()
{
    levelDisplayFSM.value = levelInPercent; // set the value to be displayed
}


volatile uint32_t lastSevenSegUpdate = 0; // the last time when the seven segment value was updated
volatile uint8_t previous7SegValue = 0;   // since the seven segment is updated at slower pace, its previous state is being buffered until next update
// this function calculates the value of 7seg display whether it's an error code or percentage
uint8_t calculate7Seg()
{
    // code to select the FSM currently being executed
    SevenSegmentsFSM *activeFSM = StateOfSevenSeg == SevenSegDisplayingWaterLevel ? &levelDisplayFSM : &errorDisplayFSM;
#ifndef TEST // if test mode is enabled this timing system is bypassed
    // update the seven segment value every 500ms (or 100ms in case of flicker), during that time the prvious value is returned
    uint32_t now = millis();
    if (lastSevenSegUpdate && (now - lastSevenSegUpdate) < (activeFSM->isFlickerDone(1) ? 100 : 500))
        return previous7SegValue;
    lastSevenSegUpdate = now;
#endif

    if (StateOfSevenSeg == SevenSegDisplayingWaterLevel) // if we're displaying level we check for the presence of error codes, if we're already displaying an error we let it finish first, and when it's done it will switch back to display levels, then we can proced to display the next error (if it exists)
    {
        if (errors[0]) // we have an error? then reset the display state to prepare for a new display of error code
        {
            StateOfSevenSeg = SevenSegDisplayingError;
            errorDisplayFSM.value = errors[0];
            activeFSM = &errorDisplayFSM;

            for (uint8_t i = 0; i < sizeof(errors) - 1; i++) // pop the error being displayed from the stack
                errors[i] = errors[i + 1];
            errors[sizeof(errors) - 1] = 0;
        }
    }

    activeFSM->Execute();
    return (previous7SegValue = activeFSM->LastResult());
}

// macro to set/clear the 10th bit of the bargraph
#define setA9 PORTD |= _BV(PD4)
#define clearA9 PORTD &= ~_BV(PD4)

//    Atmega328    ][  seconds shift rehister ][   first shift register    ]
//            PD4 ][ Q0 Q1 Q2 Q3 Q4 Q5 Q6 Q7  ][ Q7   Q6 Q5 Q4 Q3 Q2 Q1 Q0 ]
//              |    |  |  |  |  |  |  |  |     |     |  |  |  |  |  |  |
//           [ A9   A8 A7 A6 A5 A4 A3 A2 A1    A0 ][ G  F  E  D  C  B  A ]
//           [           bargraph                 ][  seven segments     ]

// the first shift register has the data of the seven segment display and the the first bit of the bargraph
// the second shift register has 8 bits of the data of the bargraph swapped (because of schematic)
// the 10 bit of the bargraph is conencted to pin PD4 of the micro controller
void displayInShiftRegister()
{

    uint32_t bargraphValue = calculateBarGraph(levelInPercent, 7);
    uint8_t sevenSegValue = calculate7Seg();

    uint32_t shiftRegisterValue = bargraphValue | sevenSegValue;

    WriteRegister(swapBits(((uint8_t *)&shiftRegisterValue)[1])); // write the data of the bargraph
    WriteRegister(((uint8_t *)&shiftRegisterValue)[0]);           // write the data to be displayed by the seven seg

    if ((shiftRegisterValue >> 16) & 1) // complete the data of the bargraph
        setA9;
    else
        clearA9;

    UpdateRegister(); // latch
}

uint16_t measureDistance() // in cm
{
    statInfo_t xTraStats;

    distance = readRangeSingleMillimeters(&xTraStats) / 10;

    debug_dec(distance);
    debug_str("cm ");
    return distance;
}

// display integer value in MAX7219 display

volatile uint8_t AutoRefreshCounter = 0; // for some reason my display shows weird character that I had to reinit it regularly
void displayInt(uint16_t value, uint8_t isPercent)
{
    if (++AutoRefreshCounter == 20) // reinit the display every 20 call
    {
        AutoRefreshCounter = 0;
        Max7219_Init();
    }
    uint8_t oldSreg = SREG;
    cli(); // disable all interrupts

    uint8_t *data[5];

    // the display has 4 screens, the characters are sent in reverse (last one sent first)
    // the number is coded as a percent charater followed by 4 digits
    // if the dispalyed number is a percentage the percent character is the first one and only three digits are displayed,
    // otherwise its probably an error code so 4 digits are displayed
    data[0] = (uint8_t *)(percent);
    data[1] = (uint8_t *)(numbers[(value) % 10]);
    data[2] = (uint8_t *)(numbers[(value / 10) % 10]);
    data[3] = (uint8_t *)(numbers[(value / 100) % 10]);
    data[4] = (uint8_t *)(numbers[(value / 1000) % 10]);

    uint8_t **first_data = &data[isPercent ? 0 : 1]; // if it's percentage start from the percent character, otherwsise skip it

    for (int col = 0; col < 8; col++)
    {
        Write_Max7219(col + 1,
                      get_line_from_8x8_matrix(first_data[0], col),
                      get_line_from_8x8_matrix(first_data[1], col),
                      get_line_from_8x8_matrix(first_data[2], col),
                      get_line_from_8x8_matrix(first_data[3], col));
    }
    SREG = oldSreg;
}

// display a number (not in percent) for one second
void displayError(uint16_t error_code)
{
    while (TRUE)
    {
        displayInt(error_code, FALSE);
        _delay_ms(1000);
    }
}

// display a number (not in percent) for one second then clear the screen for 200ms
// usually used to display constants (Fullheight is flashed at startup for example)
void flashValue(uint16_t value)
{
    displayInt(value, FALSE);
    _delay_ms(1000);
    Clear_Max7219();
    _delay_ms(200);
}

// write the duty cycle of the heart beat led
void pwmWrite(uint8_t value)
{
    OCR2B = value;
}

// initialize the timer1 for 10ms interrupt
void initTimer1()
{
    //--------------------------------------------------
    // Timer1 for the heartbeat update
    //--------------------------------------------------
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // Clear Timer on Compare Match (Mode 4), no pin output, TOP=OCR1A, 64 prescaler
    OCR1A = ((F_CPU / 64) / 100);                      // every 10ms
    TIMSK1 = (1 << OCIE1A);
}

// init the pwm pin
void pwmInit()
{
    DDRD |= _BV(HB_LED);
    TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // Fast pwm mode, output in OC2B pin (PD3)
    TCCR2B = _BV(CS20);                             // no prescaling for the timer
    pwmWrite(0);                                    // initially set the duty cycle at 0
}

// initialization function, to init several peripherals, GPIOs and timers
void init()
{
    debugInit();
    //--------------------------------------------------
    // GPIOs
    //--------------------------------------------------
    UCSR0B &= ~_BV(RXEN0); // Disable UART RX
    DDRD = _BV(PIN_UART_TX);
    DDRD |= _BV(PD4); // PD4 as output for A9 of bargraph
    i2c_init();
    initMillis();
#ifndef TEST
    initTimer1();
#endif
    pwmInit();
    InitShiftRegister();

    errorDisplayFSM.OnLastPhaseDone = errorDisplayFSM_OnLastPhaseDoneHandler;
    levelDisplayFSM.OnFirstPhaseStarted = levelDisplayFSM_OnFirstPhaseStarted;

    sei();
}

volatile uint8_t heartBeatValue = 20; // the initial value of the hearbeat pwm
volatile int8_t heartBeatDelta = 1;   // the step to progress the pwm value
void heartBeat()
{
    if ((heartBeatValue > 100) | (heartBeatValue < 20)) // every time a max/min value is reached change the direction of the pwm
        heartBeatDelta = -heartBeatDelta;

    heartBeatValue += heartBeatDelta; // increase (or decrease) the pwm value
    pwmWrite(heartBeatValue);
}

// interrupt handler for the timer1 compare match that ticks every 10ms
ISR(TIMER1_COMPA_vect)
{
    heartBeat();              // update the heartbeat pwm
    displayInShiftRegister(); //
}

// push an error code to the errors stack
void pushError(uint8_t errorValue)
{
    for (int i = sizeof(errors) - 1; i > 0; i--) // shift the values inside the errors stack
        errors[i] = errors[i - 1];

    errors[0] = errorValue; // the first element is the latest error
}

int main(void)
{
    init();

// this code is used for tests in simulator, so the measurments are generated and the timers are replaced by delays
#ifdef TEST
    levelInPercent = 11;
    int8_t step = 6;
    int8_t errorCounter = 20;
    while (1)
    {
        if (errorCounter++ == 20)
        {
            errors[0] = 12;
            errors[1] = 24;
            errorCounter = 0;
        }

        if (levelInPercent > 90)
            step = -6;
        else if (levelInPercent < 11)
            step = 6;
        levelInPercent += step;
        displayInShiftRegister();

        debug_dec(levelDisplayFSM.value);
        debug_putc(' ');

        if ((StateOfSevenSeg == SevenSegDisplayingWaterLevel ? levelDisplayFSM : errorDisplayFSM).isFlickerDone(1))
            _delay_ms(100);
        else
            _delay_ms(500);
    }
#endif

    initVL53L0X(0);
    Max7219_Init();

    // configure the VLC53L0X sensor
    setSignalRateLimit(0.1);
    setVcselPulsePeriod(VcselPeriodPreRange, 18);
    setVcselPulsePeriod(VcselPeriodFinalRange, 14);
    setMeasurementTimingBudget(500 * 1000UL);

    // read the fullheight from EEPROM, it's eventually updated if calibration is enabled
    FullHeight = eeprom_read_word(&EE_FullHeight);

    flashValue(FullHeight); // display full height for 1 sec then clear sceen

    calibrate(); // do the calibration (if enable)

    while (1) // main loop
    {
        uint16_t distance = measureDistance(); // start the distance measurement and return the result
        if ((distance | 1) == 8191)            // is it an error code?
        {
            pushError(distance - 8100); // display the error code in the seven segments (either 90 or 91)
            continue;
        }

        levelInPercent = ((FullHeight - distance) * 100 / FULL_WATER);

        if (levelInPercent > 100) // maximum 100%
        {
            levelInPercent = 100;
            // FullHeight = FULL_WATER + mean_distance; // update fullheight
        }
        debug_dec(levelInPercent); // send value through serial
        debug_str("% ");

        displayInt(levelInPercent, TRUE); // update the MAX7219
        _delay_ms(200);
    }
}
