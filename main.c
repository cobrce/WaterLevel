// #define TEST
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

#define SENSOR_HEIGHT 30
#define FULL_WATER 190
#define TOO_FAR_HEIGHT (FULL_WATER + SENSOR_HEIGHT + 10)
#define TOO_CLOSE_HEIGHT 10
#define HYSTERESIS 5

uint16_t EEMEM EE_FullHeight = FULL_WATER + SENSOR_HEIGHT;
volatile uint16_t FullHeight = 0;
volatile uint16_t distance; // for debug
volatile uint16_t levelInPercent;

// calculate a bargraph representation of a value then shift it to fit the shift register
uint32_t calculateBarGraph(uint16_t valueInPercent, uint8_t shift)
{

    if (valueInPercent > 100)
        valueInPercent = 100;
    uint32_t result = 0;

    uint8_t valueInTens = ((valueInPercent / 10) + 1) % 11; // each led in bargraph represents 10% rounded to the next 10%, seiled at 10

    while (valueInTens--)
        result = (result << 1) | 1;

    return result << shift;
}

// a queue of detected errors to be displayed in in the seven segment
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

uint8_t SevenSegFlicker = 0;    // telling "sevenSegDisplayError" and "sevenSegDisplayWaterLevel" that the next char should be blank
uint8_t SevenSegFlickerEnd = 0; // telling "calculate7Seg" that the previous character was a flicker blank, so updated sooner (100ms instead of 500ms)

uint8_t SevenSegNumbers[] = {
    0b0111111, // 0
    0b0000110, // 1
    0b1011011, // 2
    0b1001111, // 3
    0b1100110, // 4
    0b1101101, // 5
    0b1111101, // 6
    0b0000111, // 7
    0b1111111, // 8
    0b1101111  // 9
};

const uint8_t SevenSegP = 0b1110011;
const uint8_t SevenSegF = 0b1110001;

uint8_t swapBits(uint8_t input)
{
    uint8_t result = 0;
    for (int i = 0; i < 8; i++)
    {
        result <<= 1;
        result |= (input & 1);
        input >>= 1;
    }
    return result;
}

uint8_t extractSevenSegDigit(uint32_t value, uint8_t index)
{
    for (int i = 0; i < index; i++)
        value /= 10;
    value %= 10;
    return SevenSegNumbers[value]; // convert number into 7 seg representation
}

volatile uint8_t SevenSegErrorCodeIndex = 1; // like this 2 or 3 chars can be displayed during SevenSegSecondDigit phase and leave third digit for blank char
uint8_t sevenSegDisplayError()
{
    uint8_t result;
    result = 0;
    switch (PhaseOfSevenSeg)
    {
    case SevenSegFirstDigit:
        SevenSegErrorCodeIndex = 1;
        PhaseOfSevenSeg = SevenSegSecondDigit;
        SevenSegFlicker = 1;
        result = SevenSegF;
        break;

    case SevenSegSecondDigit:
    {
        if (SevenSegFlicker)
        {
            result = 0;
            SevenSegFlicker = 0;
            SevenSegFlickerEnd = 1;
        }
        else
        {
            SevenSegFlicker = 1;
            uint8_t value = errors[0];
            result = extractSevenSegDigit(value, SevenSegErrorCodeIndex);
            if (SevenSegErrorCodeIndex == 0) // shifting the errors queue
            {
                for (uint8_t i = 0; i < sizeof(errors) - 1; i++)
                    errors[i] = errors[i + 1];
                errors[sizeof(errors) - 1] = 0;
                PhaseOfSevenSeg = SevenSegThirdDigit;
                SevenSegFlicker = 0;
            }
            else
            {
                SevenSegErrorCodeIndex--;
            }
        }
    }
    break;

    case SevenSegThirdDigit:
        StateOfSevenSeg = SevenSegDisplayingWaterLevel; // the last digit of this error is displayed, switching back to water level display, if another error is in queue "Calculate7Seg" will call this again
        PhaseOfSevenSeg = SevenSegFirstDigit;
        SevenSegFlicker = 0;
        break;
    }
    return result;
}

uint8_t sevenSegWaterLevelIndex;
uint8_t levelInPercentBuffered = 0;
uint8_t sevenSegDisplayWaterLevel()
{
    uint8_t result = 0;

    switch (PhaseOfSevenSeg)
    {
    case SevenSegFirstDigit:
        levelInPercentBuffered = levelInPercent;
        sevenSegWaterLevelIndex = 2;
        PhaseOfSevenSeg = SevenSegSecondDigit;
        SevenSegFlicker = 1;
        result = SevenSegP;
        break;

    case SevenSegSecondDigit:
        if (SevenSegFlicker)
        {
            result = 0;
            SevenSegFlicker = 0;
            SevenSegFlickerEnd = 1;
        }
        else
        {
            result = extractSevenSegDigit(levelInPercentBuffered, sevenSegWaterLevelIndex);
            SevenSegFlicker = 1;

            if (sevenSegWaterLevelIndex == 0)
            {
                PhaseOfSevenSeg = SevenSegThirdDigit;
                SevenSegFlicker = 0;
            }
            else
            {
                sevenSegWaterLevelIndex--;
            }
        }

        break;

    default: // SevenSegThirdDigit
        PhaseOfSevenSeg = SevenSegFirstDigit;
        SevenSegFlicker = 0;
        break;
    }
    return result;
}

volatile uint32_t lastSevenSegUpdate = 0;
volatile uint8_t previous7SegValue = 0;
// this function calculates the value of 7seg display whether it's an error code or percentage
uint8_t calculate7Seg()
{
#ifndef TEST
    uint32_t now = millis();
    if (lastSevenSegUpdate && (now - lastSevenSegUpdate) < (SevenSegFlickerEnd ? 100 : 500))
        return previous7SegValue;
    lastSevenSegUpdate = now;
    SevenSegFlickerEnd = 0;
#endif

    if (StateOfSevenSeg == SevenSegDisplayingWaterLevel)
    {
        if (errors[0]) // we have an error
        {
            StateOfSevenSeg = SevenSegDisplayingError;
            PhaseOfSevenSeg = SevenSegFirstDigit;
            SevenSegFlicker = 0;
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

#define setA9 PORTD |= _BV(PD4)
#define clearA9 PORTD &= ~_BV(PD4)

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
    UpdateRegister();

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
#ifndef TEST
    initTimer1();
#endif
    pwmInit();
    InitShiftRegister();
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
    displayInShiftRegister();
}

void pushError(uint8_t errorValue)
{
    for (int i = sizeof(errors) - 1; i> 0;i--)
        errors[i] = errors[i-1];
    
    errors[0] = errorValue;
}

int main(void)
{
    init();

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

        debug_dec(levelInPercentBuffered);
        debug_putc(' ');

        if (SevenSegFlickerEnd)
        {
            SevenSegFlickerEnd = 0;
            _delay_ms(100);
        }
        else
        {
            _delay_ms(500);
        }
    }
#endif

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
        {
            pushError(distance - 8100); // either 90 or 91
            continue;
        }

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
