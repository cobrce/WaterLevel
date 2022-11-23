#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "MAX7219_attiny.h"
#include "font.h"
#include "errors.h"

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
    do
    {
        distance = readRangeSingleMillimeters(&xTraStats) / 10;
    } while ((distance | 1) == 8191);

    debug_dec(distance);
    debug_str("cm ");
    return distance;
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
    eeprom_write_word(&EE_FullHeight, FullHeight);
    return 0;
}
#endif

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

void FlashValue(uint16_t value)
{
    DisplayInt(value, FALSE);
    _delay_ms(1000);
    Clear_Max7219();
    _delay_ms(200);
}

#define NUMBER_OF_SAMPLES 5

uint16_t MeasureMeanDistance()
{
    uint16_t mean_distance = 0;
    for (int i = 0; i < NUMBER_OF_SAMPLES;)
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
            _delay_ms(1);
            i++;
        }
    }
    return mean_distance / NUMBER_OF_SAMPLES;
}

void init()
{

    debugInit();
    //--------------------------------------------------
    // GPIOs
    //--------------------------------------------------
    UCSR0B &= ~_BV(RXEN0);   // Disable UART RX
    DDRD = _BV(PIN_UART_TX); // Set UART TX as output
    i2c_init();
    initMillis();
    sei();
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

    // FlashValue(FullHeight); // display full height for 1 sec then clear sceen

#ifdef CALIBRATE
    uint16_t error_code = CalibrateFullHeight();
    if (error_code)
        DisplayError(error_code); // display error code infinitly (until reset)

    FlashValue(distance);   // display distance for 1 sec then clear sceen
    FlashValue(FullHeight); // display full height for 1 sec then clear sceen
#endif

    while (1)
    {
        uint16_t mean_distance = MeasureMeanDistance();

        uint16_t percent = ((FullHeight - mean_distance) * 100 / FULL_WATER);

        if (percent > 100) // update full height
        {
            percent = 100;
            FullHeight = FULL_WATER + mean_distance;
        }
        debug_dec(percent);
        debug_str("% ");

        // DisplayInt(percent, TRUE);
        _delay_ms(500);
    }


    // uint16_t mean_distance = 0; // MeasureMeanDistance();
    // uint16_t percent = 100;     // TwoPercentAlign((FullHeight - mean_distance) * 100 / FULL_WATER);
    // while (1)
    // {
    //     uint16_t new_mean_distance = MeasureMeanDistance();

    //     if ((new_mean_distance > (mean_distance + HYSTERESIS)) || // consider only critical changes
    //         (new_mean_distance < (mean_distance - HYSTERESIS)))
    //     {
    //         mean_distance = new_mean_distance;

    //         percent = ((FullHeight - mean_distance) * 100 / FULL_WATER);
    //         percent = TwoPercentAlign(percent);

    //         if (percent > 100) // update full height
    //         {
    //             percent = 100;
    //             FullHeight = FULL_WATER + mean_distance;
    //         }
    //     }
    //     DisplayInt(percent, TRUE);
    //     _delay_ms(500);
    // }
}
