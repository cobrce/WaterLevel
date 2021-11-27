# --
# Copyright (c) 2016, Lukasz Marcin Podkalicki <lpodkalicki@gmail.com>
# --

MCU=attiny26
FUSE_L=0xE4
FUSE_H=0xF7
F_CPU=8000000
CC=C:/WinAVR-20100110/bin/avr-gcc
LD=C:/WinAVR-20100110/bin/avr-ld
OBJCOPY=C:/WinAVR-20100110/bin/avr-objcopy
SIZE=C:/WinAVR-20100110/bin/avr-size
AVRDUDE=avrdude
CFLAGS= -Wall -g -Os -gdwarf-2 -mmcu=${MCU} -DF_CPU=${F_CPU} -I. -D${MCU} -std=c99 
TARGET=main

SRCS = main.c

all:
	${CC} ${CFLAGS} -o ${TARGET}.o ${SRCS}
	${LD} -o ${TARGET}.elf ${TARGET}.o
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.o ${TARGET}.hex
	${SIZE} -C --mcu=${MCU} ${TARGET}.elf

flash: all
	${AVRDUDE} -p ${MCU} -c stk500v1 -P COM4 -b 115200 -U flash:w:${TARGET}.hex:i -F

fuse:
	$(AVRDUDE) -p ${MCU} -c stk500v1 -P COM4 -b 115200 -U hfuse:w:${FUSE_H}:m -U lfuse:w:${FUSE_L}:m

clean:
	rm -f *.c~ *.o *.elf *.hex
