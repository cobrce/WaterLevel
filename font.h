#include <avr/pgmspace.h>

#define ROTATED

#ifndef ROTATED
const char numbers[10][8] =
    {
        {0x7C, 0xFE, 0x8E, 0x9A, 0xB2, 0xFE, 0x7C, 0x00}, // U+0030 (0)
        {0x02, 0x42, 0xFE, 0xFE, 0x02, 0x02, 0x00, 0x00}, // U+0031 (1)
        {0x46, 0xCE, 0x9A, 0x92, 0xF6, 0x66, 0x00, 0x00}, // U+0032 (2)
        {0x44, 0xC6, 0x92, 0x92, 0xFE, 0x6C, 0x00, 0x00}, // U+0033 (3)
        {0x18, 0x38, 0x68, 0xCA, 0xFE, 0xFE, 0x0A, 0x00}, // U+0034 (4)
        {0xE4, 0xE6, 0xA2, 0xA2, 0xBE, 0x9C, 0x00, 0x00}, // U+0035 (5)
        {0x3C, 0x7E, 0xD2, 0x92, 0x9E, 0x0C, 0x00, 0x00}, // U+0036 (6)
        {0xC0, 0xC0, 0x8E, 0x9E, 0xF0, 0xE0, 0x00, 0x00}, // U+0037 (7)
        {0x6C, 0xFE, 0x92, 0x92, 0xFE, 0x6C, 0x00, 0x00}, // U+0038 (8)
        {0x60, 0xF2, 0x92, 0x96, 0xFC, 0x78, 0x00, 0x00}  // U+0039 (9)
};
char percent[] = {0x62, 0x66, 0x0C, 0x18, 0x30, 0x66, 0x46, 0x00}; // U+0025 (%)
#else
const char numbers[10][8] =
    {
        {0x7C, 0xC6, 0xCE, 0xDE, 0xF6, 0xE6, 0x7C, 0x00},
        {0x30, 0x70, 0x30, 0x30, 0x30, 0x30, 0xFC, 0x00},
        {0x78, 0xCC, 0x0C, 0x38, 0x60, 0xCC, 0xFC, 0x00},
        {0x78, 0xCC, 0x0C, 0x38, 0x0C, 0xCC, 0x78, 0x00},
        {0x1C, 0x3C, 0x6C, 0xCC, 0xFE, 0x0C, 0x1E, 0x00},
        {0xFC, 0xC0, 0xF8, 0x0C, 0x0C, 0xCC, 0x78, 0x00},
        {0x38, 0x60, 0xC0, 0xF8, 0xCC, 0xCC, 0x78, 0x00},
        {0xFC, 0xCC, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x00},
        {0x78, 0xCC, 0xCC, 0x78, 0xCC, 0xCC, 0x78, 0x00},
        {0x78, 0xCC, 0xCC, 0x7C, 0x0C, 0x18, 0x70, 0x00}
};
char percent[] = {0x00, 0xC6, 0xCC, 0x18, 0x30, 0x66, 0xC6, 0x00};
#endif