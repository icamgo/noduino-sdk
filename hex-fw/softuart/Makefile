MCU		:= attiny13
CC		:= avr-gcc
OBJCOPY := avr-objcopy

F_CPU	:= 1200000

CFLAGS	:= -g -mmcu=$(MCU) -Os -DF_CPU=${F_CPU}
#CFLAGS	:= -std=c99 -Wall -g -Os -mmcu=${MCU} -DF_CPU=${F_CPU} -I.

INC 	:= -I .

APP =	softuart

SRCS	= $(wildcard *.c)
OBJS	= $(patsubst %.c,%.o, $(SRCS))

all: $(APP).hex

$(APP).hex : $(APP) 
	$(OBJCOPY) -R .eeprom -O ihex $(APP) $(APP).hex 

$(APP): $(SRCS)
	$(CC) $(CFLAGS) $(INC) -Os $(SRCS) -o $@

clean:
	rm -f *.hex *.o $(APP)
test:
	sudo modprobe -r ftdi_sio
	sudo avrdude -cft232r1 -pt13 -P ft0 -U hfuse:r:-:h -B 1
	sudo modprobe ftdi_sio
flash:
	sudo modprobe -r ftdi_sio
	#sudo avrdude -cft232r1 -pt13 -P ft0 -U flash:w:$(APP).hex -B 1
	# 01 - > 4.8/8 MHz 0x65
	#sudo avrdude -cft232r1 -pt13 -P ft0 -U flash:w:$(APP).hex -U lfuse:w:0x65:m -B 1
	# 10 - > 4.8/4 MHz 0x6A
	sudo avrdude -cft232r1 -pt13 -P ft0 -U flash:w:$(APP).hex -U lfuse:w:0x6A:m -B 1
	# 11 -> 128/8 KHz 0x67, 
	# WARNING: This can make the chip can not be programed next time! 
	#sudo avrdude -cft232r1 -pt13 -P ft0 -U flash:w:$(APP).hex -U lfuse:w:0x67:m -B 1
	sudo modprobe ftdi_sio
rc:
	sudo modprobe -r ftdi_sio
	#sudo avrdude -cft232r1 -pt13 -P ft0 -U flash:w:$(APP).hex -B 1
	# 01 - > 4.8/8 MHz 0x65
	sudo avrdude -cft232r1 -pt13 -B 1 -P ft0 -F -V -U lfuse:w:0x65:m 
	sudo modprobe ftdi_sio
