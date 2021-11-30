#include <stdint.h>
#include <avr/io.h>
#include "stdarg.h"

#ifndef byte
typedef uint8_t byte;
#endif

#define ROTATED

void Max7219_Init();
void Write_Max7219_Byte(uint8_t value);
void Write_Max7219(byte address, short in_01, short in_02, short in_03, short in_04);

#ifdef ROTATED
inline
#endif
    uint8_t
    get_line_from_8x8_matrix(uint8_t *matrix_8x8, uint8_t line)
{
#ifdef ROTATED
    return (matrix_8x8[line]);
#else
    uint8_t output = 0;

    for (int i = 0; i < 8; i++)
        output |= ((matrix_8x8[i] >> (7 - line)) & 1) << (7 - i);

    return output;
#endif
}
#define Max7219PinCLK PB0
#define Max7219PinCS PB1
#define Max7219PinDIN PB2

#define Max7219_HIGHT 8
#define Max7219_WIDTH 8

void Max7219_Init()
{
    DDRB |= 1 << Max7219PinCLK | 1 << Max7219PinCS | 1 << Max7219PinDIN;
    Write_Max7219(0x09, 0x00, 0x00, 0x00, 0x00); // using an led matrix (not digits)
    Write_Max7219(0x0A, 0x00, 0x00, 0x00, 0x00); // Brightness 0x00-0x0F 0x01=dark .... 0x0F=bright
    Write_Max7219(0x0B, 0x07, 0x07, 0x07, 0x07); // Scan limit = 7
    Write_Max7219(0x0C, 0x01, 0x01, 0x01, 0x01); // Normal operation mode
    Write_Max7219(0x0F, 0x00, 0x00, 0x00, 0x00); // Disable display test
}

void Write_Max7219_Byte(uint8_t value)
{
    uint8_t i;

    __asm("nop");
    for (i = 0; i < 8; ++i, value <<= 1)
    {
        PORTB &= ~(1 << Max7219PinCLK);
        __asm("nop");
        if (value & 0x80)
        {
            PORTB |= (1 << Max7219PinDIN);
        }
        else
        {
            PORTB &= ~(1 << Max7219PinDIN);
        }
        PORTB |= (1 << Max7219PinCLK);
    }
}

void Clear_Max7219()
{

    for (int col = 0; col < 8; col++)
    {
        Write_Max7219(col + 1,
                      0,
                      0,
                      0,
                      0);
    }
}

void Write_Max7219(byte address, short in_01, short in_02, short in_03, short in_04)
{
    PORTB &= ~(1 << Max7219PinCS);

    Write_Max7219_Byte(address);
    Write_Max7219_Byte(in_01);
    Write_Max7219_Byte(address);
    Write_Max7219_Byte(in_02);
    Write_Max7219_Byte(address);
    Write_Max7219_Byte(in_03);
    Write_Max7219_Byte(address);
    Write_Max7219_Byte(in_04);

    PORTB = 1 << Max7219PinCS;
}