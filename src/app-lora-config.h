#ifndef APP_LORA_CONFIG
#define APP_LORA_CONFIG

#include <stdint.h>
#include "radio.h"

#define MESSAGE_SIZE         255

typedef struct
{
    RadioModems_t modem;
    uint32_t frequency;
    int8_t power;
    uint8_t bandwidthIndex;
    uint8_t spreadFactor;
    uint8_t codingRate;
    uint8_t preambleLen;
    bool fixLen;
    bool crcOn;
    bool iqInversion;
    uint32_t txTimeout;
    uint8_t buffSize;
} AppLoRaTxConfig_t;

typedef struct
{
    RadioModems_t modem;
    uint32_t frequency;
    uint8_t bandwidthIndex;
    uint8_t spreadFactor;
    uint8_t codingRate;
    uint8_t preambleLen;
    bool fixLen;
    bool crcOn;
    uint8_t symbolTimeout;
    bool iqInversion;
    uint32_t rxTimeout;
    uint8_t buffSize;
    bool rxContinuous;
} AppLoRaRxConfig_t;

extern const AppLoRaTxConfig_t APP_LORA_TX_CONFIG;
extern const AppLoRaRxConfig_t APP_LORA_RX_CONFIG;

#endif /* APP_LORA_CONFIG */