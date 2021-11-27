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


#define FullWater 200
uint32_t EEMEM EE_FullHeight = 200;
volatile uint32_t FullHeight = 0;
volatile uint32_t distance; // for debug

void SetupTimer()
{
    TCCR0 = (0 << CS02) | (0 << CS01) | (1 << CS00); // prescaler 1 (non null to enable timer)
}

volatile uint32_t TimerOverflow = 0;

ISR(TIMER0_OVF0_vect)
{
    TimerOverflow++; /* Increment Timer Overflow count */
}

void SetupTimerOverFlowInterrupt()
{
    TIMSK = _BV(TOIE0);
    sei();
}

char SignalStart()
{
    return ((PINB >> EchoPin) & 1);
}

char SignalEnd()
{
    return !SignalStart();
}

void Trigger()
{
    PORTB |= _BV(TriggerPin);
    _delay_us(10);
    PORTB &= !_BV(TriggerPin);
}

volatile uint32_t time;
volatile uint8_t cntr = 5;

uint32_t MeasureDistance()
{
    Trigger();

    while (!(PINB & _BV(EchoPin)))
    {
    }

    TCNT0 = 0;
    TimerOverflow = 0;

    while ((PINB & _BV(EchoPin)))
    {
    }

    cntr = TCNT0; // for debug
    time = TCNT0 + (TimerOverflow * 256);

    // uint16_t distance = time * 34300 / 2; // time in seconds
    return time / 466; // time in 0.125us
}

uint16_t CalibrateFullHeight()
{
    distance = MeasureDistance();
    if (distance > 200)
        return CALBIRATION_SENSOR_TOO_FAR;
    FullHeight = FullWater + distance;
    eeprom_write_dword(&EE_FullHeight,FullHeight);
    return 0;
}

void DisplayInt(uint32_t value, uint8_t isPercent)
{
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
    DisplayInt(value,FALSE);
    _delay_ms(1000);
    Clear_Max7219();
    _delay_ms(200);
}

int main(void)
{
    Max7219_Init();

    DDRB |= _BV(TriggerPin);
        
    FullHeight = eeprom_read_dword(&EE_FullHeight);
    FlashValue(FullHeight); // display full height for 1 sec then clear sceen
    
    
    SetupTimer();

    SetupTimerOverFlowInterrupt();

    uint16_t error_code = CalibrateFullHeight();
    if (error_code)
        DisplayError(error_code); // display error code infinitly (until reset)

    FlashValue(distance); // display distance for 1 sec then clear sceen
    FlashValue(FullHeight); // display full height for 1 sec then clear sceen
    

    while (1)
    {
        distance = MeasureDistance();
        uint16_t percent = ((FullHeight - distance) * 100 / FullHeight);
        // percent = TwoPercentAlign(percent);

        if (percent > 100) // update full height
        {
            percent = 100;
            FullHeight = FullWater + distance;
        }
        DisplayInt(percent, TRUE);
        _delay_ms(1000);
    }
}