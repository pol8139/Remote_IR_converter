PROG = test_remote_r
OBJS = ${PROG}.o
MCU = attiny13a
F_CPU = 9600000UL

ELF = $(PROG).elf
HEX = $(PROG).hex

CC = avr-gcc
CFLAGS = -g -Os -std=c99 -mmcu=$(MCU) -DF_CPU=$(F_CPU)
LDFLAGS = -g -Os -mmcu=$(MCU)
OBJCOPY = avr-objcopy
SIZE = avr-size

all: $(HEX)

$(HEX): $(ELF)
	$(OBJCOPY) -j .text -j .data -O ihex $(ELF) $(HEX)
	$(SIZE) --mcu=$(MCU) --format=avr $(ELF)

$(ELF): $(OBJS)
	$(CC) $(CFLAGS) -o $(ELF)    $^

clean:
	-$(RM) $(ELF) $(HEX) $(OBJS)
