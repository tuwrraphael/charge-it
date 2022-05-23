ifeq ($(strip $(SIMULATION)),TRUE)
MCU = atmega328p
else
MCU = atmega8
endif
F_CPU = 12000000

.PHONY: clean
default: main.txt

FILES = build/main.o \
build/uart_debug.o \
build/moving_average.o \
build/update_state.o \

TEST_FILES = \
build/tests/moving_average.o \
build/tests/test_moving_average.o 

CC = avr-gcc
LD = avr-gcc

TEST_CC = g++
TEST_LD = g++
TEST_CFLAGS = -Wall -I /usr/include -I /usr/src/gtest -L /usr/local/lib -lpthread -DF_CPU=$(F_CPU)

CFLAGS = -Wall -Wstrict-prototypes -frename-registers -fshort-enums -fpack-struct -funsigned-char -funsigned-bitfields -std=c99 -O3 -DF_CPU=$(F_CPU)UL -mmcu=$(MCU)
LDFLAGS = -Wl,-Map=$(MCU).map,--cref 

ifeq ($(strip $(SIMULATION)),TRUE)
CFLAGS += -DUSE_SIMULATION
LDFLAGS += -Wl,--section-start=.mysection=0x800300
endif

clean:
	rm -rf build main.hex atmega8.map main.elf main.txt atmega328p.map

build:
	mkdir -p build/tests

# google test

build/tests/%.o: tests/%.c | build Makefile
	$(TEST_CC) -c $< -o $@ $(TEST_CFLAGS)

build/tests/%.o: %.c | build Makefile config.h
	$(TEST_CC) -c $< -o $@ $(TEST_CFLAGS)

build/test: $(TEST_FILES)
	$(TEST_LD) /usr/src/gtest/src/gtest_main.cc /usr/src/gtest/src/gtest-all.cc $(TEST_FILES) $(TEST_CFLAGS) -o build/test

test: build/test
	./build/test

# app

build/%.o: %.c | build Makefile
	$(CC) $(CFLAGS) -c $< -o $@

main.elf: $(FILES)
	$(LD) $(CFLAGS) $(LDFLAGS) -o $@ $^

main.txt : main.hex
	cp main.hex main.txt

main.hex: main.elf
	avr-objcopy -O ihex -j .text -j .data main.elf main.hex

flash: main.hex
	avrdude -D -p atmega8 -c usbasp -U flash:w:main.hex