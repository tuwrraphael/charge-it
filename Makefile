MCU = atmega8
F_CPU = 12000000

.PHONY: clean
default: main.hex

FILES = build/main.o \

CC = avr-gcc
LD = avr-gcc

CFLAGS = -Wall -O3 -DF_CPU=$(F_CPU)UL -mmcu=$(MCU)

build:
	mkdir $@

build/%.o: %.c Makefile | build
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf build main.hex

main.elf: $(FILES)
	$(LD) -o $@ $^

main.hex: main.elf
	avr-objcopy -O ihex -j .text -j .data main.elf main.hex

flash: main.hex
	avrdude -p atmega8 -c usbasp -U flash:w:main.hex