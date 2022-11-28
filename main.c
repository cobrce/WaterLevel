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

#define HB_LED PD3
#define PIN_UART_TX PD1
#define PIN_I2C_SDA PC4
#define PIN_I2C_SCL PC5
#include "vl53l0x-non-arduino/VL53L0X.h"
#include "vl53l0x-non-arduino/util/i2cmaster.h"
#include "vl53l0x-non-arduino/util/debugPrint.h"
#include "vl53l0x-non-arduino/util/millis.h"

#define __COMPILING_AVR_LIBC__ 1

#ifndef FALSE
#define FALSE 0
#define TRUE 1
#endif // !FALSE

#define SENSOR_HEIGHT 25
#define FULL_WATER 190
#define TOO_FAR_HEIGHT (FULL_WATER + SENSOR_HEIGHT + 10)
#define TOO_CLOSE_HEIGHT 10
#define HYSTERESIS 5

uint16_t EEMEM EE_FullHeight = FULL_WATER + SENSOR_HEIGHT;
volatile uint16_t FullHeight = 0;
volatile uint16_t distance; // for debug
volatile uint16_t levelInPercent;

// calculate a bargraph representation of a value then shift it to fit the shift register
uint16_t calculateBarGraph(uint16_t valueInPercent, uint8_t shift)
{
    if (valueInPercent > 100)
        valueInPercent = 100;
    uint16_t result = 0;

    valueInPercent = ((valueInPercent / 10) + 1) % 11; // each led in bargraph represents 10% rounded to the next 10%, seiled at 10

    while (valueInPercent--)
        result = (result << 1) | 1;

    return result << shift;
}

// this function calculates the value of 7seg display whether it's an error code or percentage
uint8_t errors[4] = {0};

enum
{
    SevenSegDisplayingError,
    SevenSegDisplayingWaterLevel
} StateOfSevenSeg = SevenSegDisplayingWaterLevel;

enum
{
    SevenSegFirstDigit,
    SevenSegSecondDigit,
    SevenSegThirdDigit
} PhaseOfSevenSeg = SevenSegFirstDigit;

uint8_t SevenSegNumbers[] = {
    0b1110111, // 0
    0b0110000, // 1
    0b1101011, // 2
    0b1111001, // 3
    0b0111100, // 4
    0b1011101, // 5
    0b1011111, // 6
    0b1110000, // 7
    0b1111111, // 8
    0b1111101  // 9
};

const uint8_t SevenSegP = 0b1100111;
const uint8_t SevenSegF = 0b1000111;

volatile uint8_t SevenSegErrorCodeIndex = 1; // like this 2 or 3 chars can be displayed during SevenSegSecondDigit phase and leave third digit for blank char
uint8_t sevenSegDisplayError()
{
    uint8_t result = 0;
    switch (PhaseOfSevenSeg)
    {
    case SevenSegFirstDigit:
        SevenSegErrorCodeIndex = 1;
        PhaseOfSevenSeg = SevenSegSecondDigit;
        result = SevenSegF;
        break;

    case SevenSegSecondDigit:
    {
        uint8_t value = errors[0];
        for (int i = 0; i < SevenSegErrorCodeIndex; i++)
            value /= 10;
        value %= 10;
        result = SevenSegNumbers[value]; // convert number into 7 seg representation

        if (SevenSegErrorCodeIndex == 0) // shifting the errors queue
        {
            for (uint8_t i = 0; i < sizeof(errors); i++)
                errors[i] = errors[i] + 1;
            errors[sizeof(errors)-1] = 0;
            PhaseOfSevenSeg = SevenSegThirdDigit;
        }
        else
        {
            SevenSegErrorCodeIndex--;
        }
    }
    break;

    case SevenSegThirdDigit:
        StateOfSevenSeg = SevenSegDisplayingWaterLevel; // the last digit of this error is displayed, switching back to water level display, if another error is in queue "Calculate7Seg" will call this again
        PhaseOfSevenSeg = SevenSegFirstDigit;
        break;
    }
    return result;
}

uint8_t sevenSegDisplayWaterLevel()
{
    return 0;
}

volatile uint32_t lastSevenSegUpdate = 0;
volatile uint8_t previous7SegValue = 0;
uint8_t calculate7Seg()
{
    uint32_t now = millis();
    if (lastSevenSegUpdate && (now - lastSevenSegUpdate) < 500)
        return previous7SegValue;

    lastSevenSegUpdate = now;

    if (StateOfSevenSeg == SevenSegDisplayingWaterLevel)
    {
        if (errors[0]) // we have an error
        {
            StateOfSevenSeg = SevenSegDisplayingError;
            PhaseOfSevenSeg = SevenSegFirstDigit;
        }
    }

    switch (StateOfSevenSeg)
    {
    case SevenSegDisplayingError:
        return (previous7SegValue = sevenSegDisplayError());
        break;

    default: // SevenSegDisplayingWaterLevel
        return (previous7SegValue = sevenSegDisplayWaterLevel());
        break;
    }
}

void displayInShiftRegister()
{
    uint16_t bargraphValue = calculateBarGraph(levelInPercent, 7);
    uint8_t sevneSegValue = calculate7Seg();
}

uint16_t measureDistance() // in cm
{
    statInfo_t xTraStats;

    distance = readRangeSingleMillimeters(&xTraStats) / 10;

    debug_dec(distance);
    debug_str("cm ");
    return distance;
}

volatile uint8_t AutoRefreshCounter = 0;
void displayInt(uint16_t value, uint8_t isPercent)
{
    if (++AutoRefreshCounter == 20)
    {
        AutoRefreshCounter = 0;
        Max7219_Init();
    }
    uint8_t oldSreg = SREG;
    cli();

    uint8_t *data[5];

    data[0] = (uint8_t *)(percent);
    data[1] = (uint8_t *)(numbers[(value) % 10]);
    data[2] = (uint8_t *)(numbers[(value / 10) % 10]);
    data[3] = (uint8_t *)(numbers[(value / 100) % 10]);
    data[4] = (uint8_t *)(numbers[(value / 1000) % 10]);

    uint8_t **first_data = &data[isPercent ? 0 : 1];

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

void displayError(uint16_t error_code)
{
    while (TRUE)
    {
        displayInt(error_code, FALSE);
        _delay_ms(1000);
    }
}

void flashValue(uint16_t value)
{
    displayInt(value, FALSE);
    _delay_ms(1000);
    Clear_Max7219();
    _delay_ms(200);
}

void pwmWrite(uint8_t value)
{
    OCR2B = value;
}

void initTimer1()
{
    //--------------------------------------------------
    // Timer1 for the heartbeat update
    //--------------------------------------------------
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // Clear TImer on Compar Match Mode (4), no pin output, TOP=OCR1A, 64 prescaler
    OCR1A = ((F_CPU / 64) / 100);                      // every 10ms
    TIMSK1 = (1 << OCIE1A);
}

void pwmInit()
{
    DDRD |= _BV(HB_LED);
    TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS20);
    pwmWrite(0);
}

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
    initTimer1();
    pwmInit();
    sei();
}

volatile uint8_t heartBeatValue = 20;
volatile int8_t heartBeatDelta = 1;
void heartBeat()
{
    // PORTD ^= _BV(HB_LED);

    if ((heartBeatValue > 100) | (heartBeatValue < 20))
        heartBeatDelta = -heartBeatDelta;

    heartBeatValue += heartBeatDelta;
    pwmWrite(heartBeatValue);
}

ISR(TIMER1_COMPA_vect)
{
    heartBeat();
}

int main(void)
{
    init();

    initVL53L0X(0);
    // startContinuous(0);
    Max7219_Init();

    setSignalRateLimit(0.1);
    setVcselPulsePeriod(VcselPeriodPreRange, 18);
    setVcselPulsePeriod(VcselPeriodFinalRange, 14);
    setMeasurementTimingBudget(500 * 1000UL);

    FullHeight = eeprom_read_word(&EE_FullHeight);

    flashValue(FullHeight); // display full height for 1 sec then clear sceen

    calibrate();

    while (1)
    {
        heartBeat();

        uint16_t distance = measureDistance();
        if ((distance | 1) == 8191)
            continue;

        levelInPercent = ((FullHeight - distance) * 100 / FULL_WATER);

        if (levelInPercent > 100) // update full height
        {
            levelInPercent = 100;
            // FullHeight = FULL_WATER + mean_distance;
        }
        debug_dec(levelInPercent);
        debug_str("% ");

        displayInt(levelInPercent, TRUE);
        _delay_ms(200);
    }
}
