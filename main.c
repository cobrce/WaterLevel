#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "MAX7219_attiny.h"
#include "font.h"
#include "errors.h"
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

uint16_t MeasureDistance() // in cm
{
    statInfo_t xTraStats;

    distance = readRangeSingleMillimeters(&xTraStats) / 10;

    debug_dec(distance);
    debug_str("cm ");
    return distance;
}

{
    distance = MeasureDistance();


    FullHeight = FULL_WATER + distance;
    eeprom_write_word(&EE_FullHeight, FullHeight);
    return 0;
}

volatile uint8_t AutoRefreshCounter = 0;
void DisplayInt(uint16_t value, uint8_t isPercent)
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

void DisplayError(uint16_t error_code)
{
    while (TRUE)
    {
        DisplayInt(error_code, FALSE);
        _delay_ms(1000);
    }
}

void FlashValue(uint16_t value)
{
    DisplayInt(value, FALSE);
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
	TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10);//Clear TImer on Compar Match Mode (4), no pin output, TOP=OCR1A, 64 prescaler
	OCR1A = ((F_CPU / 64) / 100);				//every 10ms
	TIMSK1 = (1<<OCIE1A);
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

    FlashValue(FullHeight); // display full height for 1 sec then clear sceen


    calibrate();

    while (1)
    {
        heartBeat();

        uint16_t distance = MeasureDistance();
        if ((distance | 1) == 8191)
            continue;

        uint16_t percent = ((FullHeight - distance) * 100 / FULL_WATER);

        if (percent > 100) // update full height
        {
            percent = 100;
            // FullHeight = FULL_WATER + mean_distance;
        }
        debug_dec(percent);
        debug_str("% ");

        DisplayInt(percent, TRUE);
        _delay_ms(200);
    }
}
