MCU = atmega8
F_CPU = 12000000

.PHONY: clean
default: main.hex

FILES = build/main.o \
build/uart_debug.o

CC = avr-gcc
LD = avr-gcc

CFLAGS = -Wall -O3 -DF_CPU=$(F_CPU)UL -mmcu=$(MCU) -DDEBUGUART

build:
	mkdir $@

build/%.o: %.c Makefile | build
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf build main.hex

main.elf: $(FILES)
	$(LD) $(CFLAGS) -Wl,-Map=$(MCU).map,--cref -o $@ $^

main.hex: main.elf
	avr-objcopy -O ihex -j .text -j .data main.elf main.hex

flash: main.hex
	avrdude -D -p atmega8 -c usbasp -U flash:w:main.hex