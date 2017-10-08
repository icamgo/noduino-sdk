MCU		:= attiny45
CC		:= avr-gcc
OBJCOPY := avr-objcopy
SIZE	:= avr-size

CFLAGS	:= -g -mmcu=$(MCU) -O3

INC 	:= -I .

APP = lora

SRCS	= $(wildcard *.c)
OBJS	= $(patsubst %.c,%.o, $(SRCS))

all: $(APP).hex

$(APP).hex : $(APP) 
	$(OBJCOPY) -R .eeprom -O ihex $(APP) $(APP).hex 

$(APP): $(SRCS)
	$(CC) $(CFLAGS) $(INC) $(SRCS) -o $@
	${SIZE} -C --mcu=${MCU} $(APP)

clean:
	rm -f *.hex *.o $(APP)
test:
	sudo modprobe -r ftdi_sio
	sudo avrdude -cft232r1 -pt45 -P ft0 -U lfuse:r:-:h -B 1
flash:
	sudo modprobe -r ftdi_sio
	#sudo avrdude -cft232r1 -pt45 -P ft0 -U flash:w:$(APP).hex -B 1
	# 0010 - > 8.0 MHz lfuse = 0x62
	# 0011 - > 6.4 MHz lfuse = 0x63
	#sudo avrdude -cft232r1 -F -pt45 -P ft0 -U flash:w:$(APP).hex -U lfuse:w:0x62:m -B 1
	sudo avrdude -cft232r1 -F -pt45 -P ft0 -U flash:w:$(APP).hex -B 1
	# WARNING: This can make the chip can not be programed next time! 
	#sudo avrdude -cft232r1 -pt45 -P ft0 -U flash:w:$(APP).hex -U lfuse:w:0x67:m -B 1
