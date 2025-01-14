#include "app-lora-config.h"

const AppLoRaTxConfig_t APP_LORA_TX_CONFIG = {
        .modem = MODEM_LORA,
        .frequency = 433000000,     /* Hz */
        .power = 22,                /* dbm */
        .bandwidthIndex = 2,        /* 0: 125; 1:250; 2:500 kHz */
        .spreadFactor = LORA_SF7,
        .codingRate = 1,            /* 1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8 */
        .preambleLen = 8,           /* bytes */
        .fixLen = LORA_PACKET_EXPLICIT,
        .crcOn = true,
        .iqInversion = false,
        .txTimeout = 5000,          /* ms */
        .buffSize = MESSAGE_SIZE,   /* bytes */
};

const AppLoRaRxConfig_t APP_LORA_RX_CONFIG = {
        .modem = MODEM_LORA,
        .frequency = 433000000,     /* Hz */
        .bandwidthIndex = 2,        /* 0: 125; 1:250; 2:500 kHz */
        .spreadFactor = LORA_SF7,
        .codingRate = 1,            /* 1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8 */
        .preambleLen = 8,           /* bytes */
        .fixLen = LORA_PACKET_EXPLICIT,
        .crcOn = true,
        .symbolTimeout = 0,         /* ms */
        .iqInversion = false,
        .rxTimeout = 5000,          /* ms */
        .buffSize = MESSAGE_SIZE,   /* bytes */
        .rxContinuous = false
};
