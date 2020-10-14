#ifndef __VARIANT_EMF32DIP_H__
#define __VARIANT_EMF32DIP_H__

#define A4  PD4
#define A5  PD5
#define A6  PD6
#define A7  PD7

#define Pin2AdcChannel(x) (((x) == A4)?4:(((x) == A5)?5:(((x) == A6)?6:(((x) == A7)?7:0xff))))

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

typedef struct _Pin2PortMapArray
{
  	GPIO_Port_TypeDef GPIOx_Port; 	/* defined in em_gpio.h */
  	uint8_t  Pin_abstraction;	/* emu pin*/    
    unsigned int  adc_channel;
    TIMER_TypeDef *pTimer;   //Timer1 to Timer4.
    unsigned int  timerChannelLoc;  //Timer bit3~0 channel (0-2)  bit7~4 routeLoc(0-3).  
} Pin2PortMapArray ;

/* Pins table to be instanciated into variant.cpp */
extern const Pin2PortMapArray g_Pin2PortMapArray[] ;

#ifdef __cplusplus
}
#endif

#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial2

#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2

#define SCL    (PC11)
#define SDA    (PC10)

#define SPI_INTERFACES_COUNT 1

#ifdef USE_SPI1
   /**SPI1 GPIO Configuration    
    PC8     ------> SPI1_SCK
    PC7     ------> SPI1_MISO
    PC6     ------> SPI1_MOSI 
    */
 #define BOARD_SPI1_MOSI_PIN   (PC6)   
 #define BOARD_SPI1_MISO_PIN   (PC7)   
 #define BOARD_SPI1_SCK_PIN    (PC8)   
 #define BOARD_SPI1_CS_PIN     (PC13)   
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#ifdef __cplusplus
} //extern "C"{
#endif // __cplusplus

#endif
