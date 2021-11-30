#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "MAX7219_attiny.h"
#include "font.h"
#include "errors.h"

#define __COMPILING_AVR_LIBC__ 1

#define EchoPin PB6
#define TriggerPin PB3

#ifndef FALSE
#define FALSE 0
#define TRUE 1
#endif // !FALSE

#define SENSOR_HEIGHT 10
#define FULL_WATER 200
#define TOO_FAR_HEIGHT (FULL_WATER + SENSOR_HEIGHT + 10)
#define TOO_CLOSE_HEIGHT 10
uint32_t EEMEM EE_FullHeight = FULL_WATER+SENSOR_HEIGHT;
volatile uint32_t FullHeight = 0;
volatile uint32_t distance; // for debug

inline void SetupTimer()
{
    TCCR0 = (0 << CS02) | (0 << CS01) | (1 << CS00); // prescaler 1 (non null to enable timer)
}

volatile uint32_t TimerOverflow = 0;

ISR(TIMER0_OVF0_vect)
{
    TimerOverflow++; /* Increment Timer Overflow count */
}

inline void SetupTimerOverFlowInterrupt()
{
    TIMSK = _BV(TOIE0);
    sei();
}

static inline void Trigger()
{
    PORTB |= _BV(TriggerPin);
    _delay_us(10);
    PORTB &= !_BV(TriggerPin);
}

// volatile uint32_t time;
// volatile uint8_t cntr = 5;

uint32_t MeasureDistance() // in cm
{
    Trigger();

    TimerOverflow = 0;

    while (!(PINB & _BV(EchoPin) && TimerOverflow != 700))
        ;

    TCNT0 = 0;
    TimerOverflow = 0;

    while ((PINB & _BV(EchoPin)) && TimerOverflow != 700)
        ;

    // distance = (time = (TCNT0 + TimerOverflow * 255) / 8) / (466 / 8);

    return (TCNT0 + TimerOverflow * 255) / 466;
}

#ifdef CALIBRATE
inline uint16_t CalibrateFullHeight()
{
    distance = MeasureDistance();

    if (distance > TOO_FAR_HEIGHT)
        return CALBIRATION_SENSOR_TOO_FAR;
    else if (distance < TOO_CLOSE_HEIGHT)
        return CALBIRATION_SENSOR_TOO_CLOSE;

    FullHeight = FULL_WATER + distance;
    eeprom_write_dword(&EE_FullHeight, FullHeight);
    return 0;
}
#endif

volatile uint8_t AutoRefresh = 0;
void DisplayInt(uint32_t value, uint8_t isPercent)
{
    if (++AutoRefresh == 20)
    {
        AutoRefresh = 0;
        Max7219_Init();
    }
    uint8_t oldSreg = SREG;
    cli();

    uint8_t *data[5];

    data[0] = (uint8_t *)percent;
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

inline uint16_t TwoPercentAlign(uint16_t percent)
{
    // return (percent) - (percent%2);
    return percent & ~(1);
}

void DisplayError(uint16_t error_code)
{
    while (TRUE)
    {
        DisplayInt(error_code, FALSE);
        _delay_ms(1000);
    }
}

void FlashValue(uint32_t value)
{
    DisplayInt(value, FALSE);
    _delay_ms(1000);
    Clear_Max7219();
    _delay_ms(200);
}

int main(void)
{
    Max7219_Init();

    DDRB |= _BV(TriggerPin);
    PORTB &= _BV(EchoPin); // no pull up

    SetupTimer();

    SetupTimerOverFlowInterrupt();

    // while (1)
    // {
    //     DisplayInt(MeasureDistance(),FALSE);
    //     _delay_ms(50);
    // }

    FullHeight = eeprom_read_dword(&EE_FullHeight);
    FlashValue(FullHeight); // display full height for 1 sec then clear sceen

#ifdef CALIBRATE
    uint16_t error_code = CalibrateFullHeight();
    if (error_code)
        DisplayError(error_code); // display error code infinitly (until reset)

    FlashValue(distance);   // display distance for 1 sec then clear sceen
    FlashValue(FullHeight); // display full height for 1 sec then clear sceen
#endif

    uint32_t mean_distance;
    while (1)
    {
        mean_distance = 0;
        for (int i = 0; i < 20;)
        {
            distance = MeasureDistance();
            if (distance > TOO_FAR_HEIGHT)
            {
                FlashValue(MEASURE_SENSOR_TOO_FAR);
            }
            else if (distance < TOO_CLOSE_HEIGHT)
            {
                FlashValue(MEASURE_SENSOR_TOO_FAR);
            }
            else
            {
                mean_distance += distance;
                _delay_ms(50);
                i++;
            }
        }
        mean_distance /= 20;
        uint16_t percent = ((FullHeight - mean_distance) * 100 / FullHeight);
        // percent = TwoPercentAlign(percent);

        if (percent > 100) // update full height
        {
            percent = 100;
            FullHeight = FULL_WATER + mean_distance;
        }
        DisplayInt(percent, TRUE);
        _delay_ms(500);
    }
}
