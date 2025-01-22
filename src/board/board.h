#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

typedef enum
{
    SX1261_CHIP = 0,
    SX1262_CHIP,
    SX126x_COUNT
} SX126x_CHIP_ID;

#define LED             22

/* SPI gpio */
#define ESP32_MOSI      32
#define ESP32_MISO      33
#define ESP32_SCK       25
#define ESP32_RST       26

/* board specific SX1262 gpio */
#define ESP32_NSS_1     27
#define ESP32_BUSY_1    35
#define ESP32_DIO_1     34

#define ESP32_NSS_2     13
#define ESP32_BUSY_2    15
#define ESP32_DIO_2     2

#if 0
/* ESP8285 pinout  */
#define ESP8285_MISO    12
#define ESP8285_MOSI    13
#define ESP8285_SCK     14
#define ESP8285_NSS_1   15  /* NOTE: sx1262 number 1 */
#define ESP8285_RST_1   0   /* NOTE: sx1262 number 1 */
#define ESP8285_BUSY_1  2   /* NOTE: sx1262 number 1 */
#define ESP8285_DIO_1   4   /* NOTE: sx1262 number 1 */
#endif

/* SX126X physical properties */
#define XTAL_FREQ                       ( double )32000000
#define FREQ_DIV                        ( double )pow( 2.0, 25.0 )
#define FREQ_STEP                       ( double )( XTAL_FREQ / FREQ_DIV )

/* board specifc GPIO */
typedef struct
{
    uint8_t nss;
    uint8_t busy;
    uint8_t dio;
} GpioConfig_t;
#if 0
void BoardDisableIrq(void);
void BoardEnableIrq(void);
#endif
#endif /* BOARD_H */
