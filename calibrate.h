#ifndef CALIBRATE_H_
#define CALIBRATE_H_

#define CALIBRATE

#ifdef CALIBRATE
extern uint16_t EEMEM EE_FullHeight;
extern volatile uint16_t FullHeight;
extern volatile uint16_t distance; // decalred as global for debug
inline uint16_t calibrateFullHeight()
{
    uint16_t result = 0;
    measureDistance(&result);

    if (result > TOO_FAR_HEIGHT)
        return CALBIRATION_SENSOR_TOO_FAR;
    else if (result < TOO_CLOSE_HEIGHT)
        return CALBIRATION_SENSOR_TOO_CLOSE;

    // to make current value as 98%
    FullHeight = FULL_WATER * 102/100 + result;

    eeprom_write_word(&EE_FullHeight, FullHeight);
    distance = result;
    return 0;
}
#endif

volatile uint8_t calibrateRequested = 0;
inline void calibrate()
{
#ifdef CALIBRATE
    if (!calibrateRequested)
        return;
    uint16_t error_code = calibrateFullHeight();
    if (error_code)
    {
        flashValue(error_code); // display error code infinitly (until reset)
        pushError(error_code - 8100);
    }

    flashValue(distance);   // display distance for 1 sec then clear sceen
    flashValue(FullHeight); // display full height for 1 sec then clear sceen
#endif
}

#endif
