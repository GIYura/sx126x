#include <Arduino.h>

#include "app-lora-config.h"
#include "radio.h"
#include "board.h"

#define TICKS_TO_START_TX   5 /* tick multiplied by CAD timeout */

typedef struct
{
    uint8_t* buff;
    uint8_t size;
    uint32_t counter;
} TxPayload_t;

typedef struct 
{
    uint8_t* buff;
    uint8_t size;
    int16_t rssi;
    int8_t snr;
    uint32_t counter;
} RxPayload_t;

/* Application callbacks on Radio events */
static void OnTxDone(SX1262_ID id);
static void OnRxDone(SX1262_ID id, uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr);
static void OnTxTimeout(SX1262_ID id);
static void OnRxTimeout(SX1262_ID id);
static void OnRxError(SX1262_ID id);
static void OnCadDone(SX1262_ID id, bool cadResult);

static RadioEvents_t m_radioEvents = { 0 };

static uint8_t m_rxBuffer[MESSAGE_SIZE] = { 0 };
static RxPayload_t m_rxPayload = { .buff = m_rxBuffer, .size = MESSAGE_SIZE, .rssi = 0, .snr = 0, .counter = 0 };

static uint8_t m_txBuffer[MESSAGE_SIZE] = { 0 };
static TxPayload_t m_txPayload = { .buff = m_txBuffer, .size = MESSAGE_SIZE, .counter = 0 };

static volatile bool m_txDone = { false };
static volatile bool m_rxDone = { false };
static time_t m_cadTime = { 0 };

static GpioConfig_t m_sx1262GpioConfig = { 0 };

static uint32_t m_txToStartCounter = 0;
static bool m_txToStart = false;

/* Init */
void setup(void)
{
    Serial.begin(9600);
    
    /* NOTE: delay to let the system start and switch to 'monitor' */
    delay(5000);

    /* NOTE: fill tx buffer with indexes for test only */
    for (uint8_t i = 0; i < m_txPayload.size; i++)
    {
        m_txPayload.buff[i] = i + 1;
    }

    /* Initialize the Radio callbacks */
    m_radioEvents.TxDone = &OnTxDone;
    m_radioEvents.RxDone = &OnRxDone;
    m_radioEvents.TxTimeout = &OnTxTimeout;
    m_radioEvents.RxTimeout = &OnRxTimeout;
    m_radioEvents.RxError = &OnRxError;
    m_radioEvents.CadDone = &OnCadDone;

    /* hardware config (board specific) */
    m_sx1262GpioConfig.busy = ESP32_BUSY_1;
    m_sx1262GpioConfig.nss = ESP32_NSS_1;
    m_sx1262GpioConfig.dio = ESP32_DIO_1;

    Radio.Init(SX1262_ID_TX, &m_radioEvents, &m_sx1262GpioConfig);

    m_sx1262GpioConfig.busy = ESP32_BUSY_2;
    m_sx1262GpioConfig.nss = ESP32_NSS_2;
    m_sx1262GpioConfig.dio = ESP32_DIO_2;

    Radio.Init(SX1262_ID_RX, &m_radioEvents, &m_sx1262GpioConfig);

    /* Set radio channel */
    Radio.SetChannel(SX1262_ID_TX, APP_LORA_TX_CONFIG.frequency);
    Radio.SetChannel(SX1262_ID_RX, APP_LORA_RX_CONFIG.frequency);

    /* Set public network */
    Radio.SetPublicNetwork(SX1262_ID_TX, true);
    Radio.SetPublicNetwork(SX1262_ID_RX, true);

    /* Set Radio TX configuration */
    Radio.SetTxConfig(SX1262_ID_TX,
                    APP_LORA_TX_CONFIG.modem,
                    APP_LORA_TX_CONFIG.power,
                    0,
                    APP_LORA_TX_CONFIG.bandwidthIndex,
                    APP_LORA_TX_CONFIG.spreadFactor,
                    APP_LORA_TX_CONFIG.codingRate,
                    APP_LORA_TX_CONFIG.preambleLen,
                    APP_LORA_TX_CONFIG.fixLen,
                    APP_LORA_TX_CONFIG.crcOn,
                    0,
                    0,
                    APP_LORA_TX_CONFIG.iqInversion,
                    APP_LORA_TX_CONFIG.txTimeout);

    Radio.SetMaxPayloadLength(SX1262_ID_TX, APP_LORA_TX_CONFIG.modem, APP_LORA_TX_CONFIG.buffSize);

    Radio.SetRxConfig(SX1262_ID_RX,
                    APP_LORA_RX_CONFIG.modem,
                    APP_LORA_RX_CONFIG.bandwidthIndex,
                    APP_LORA_RX_CONFIG.spreadFactor,
                    APP_LORA_RX_CONFIG.codingRate,
                    0,
                    APP_LORA_RX_CONFIG.preambleLen,
                    APP_LORA_RX_CONFIG.symbolTimeout,
                    APP_LORA_RX_CONFIG.fixLen,
                    APP_LORA_RX_CONFIG.buffSize,
                    APP_LORA_RX_CONFIG.crcOn,
                    0,
                    0,
                    APP_LORA_RX_CONFIG.iqInversion,
                    APP_LORA_RX_CONFIG.rxContinuous);

    /* start radio #2 in RX mode */
    Radio.Rx(SX1262_ID_RX, APP_LORA_RX_CONFIG.rxTimeout);

    Serial.printf("### SX1262 #2 RX mode ### \r\n");
    Serial.printf("---------------------------\r\n");
}

/* Run */
void loop(void)
{
/* NOTE: for SX1262 test only */
#if 0
    while(1)
    {
        uint8_t gain = m_sx1262.ReadRegister(SX1262_ID_TX, SX126X_REG_RX_GAIN);
        Serial.printf("SX1262 #1 GAIN: [0x%x]\r\n", gain);

        uint8_t ocp = m_sx1262.ReadRegister(SX1262_ID_TX, SX126X_REG_OCP_CONFIGURATION);
        Serial.printf("SX1262 #1 OCP CONFIG: [0x%x]\r\n", ocp);

        RadioStatus_t status = m_sx1262.GetStatus(SX1262_ID_TX);
        Serial.printf("SX1262 #1 STATUS: [0x%x]\r\n", status);

        gain = m_sx1262.ReadRegister(SX1262_ID_RX, SX126X_REG_RX_GAIN);
        Serial.printf("SX1262 #2 GAIN: [0x%x]\r\n", gain);

        ocp = m_sx1262.ReadRegister(SX1262_ID_RX, SX126X_REG_OCP_CONFIGURATION);
        Serial.printf("SX1262 #2 OCP CONFIG: [0x%x]\r\n", ocp);

        status = m_sx1262.GetStatus(SX1262_ID_RX);
        Serial.printf("SX1262 #2 STATUS: [0x%x]\r\n", status);

        delay(1000);
    }
#endif

    if (m_txToStart)
    {
        m_txToStart = false;

        /* start radio #1 in TX mode */
        Radio.Send(SX1262_ID_TX, m_txBuffer, APP_LORA_TX_CONFIG.buffSize);

        Serial.printf("### SX1262 #1 TX mode ### \r\n");
        Serial.printf("---------------------------\r\n");
    }

    /* Handle Radio events */
    Radio.IrqProcess();

    if (m_txDone)
    {
        m_txDone = false;

        Serial.printf("TX counter: [%d]\r\n", m_txPayload.counter);

#if defined (TX_PAYLOAD_PRINT)
        Serial.printf("TX message: [ ");
        for (uint8_t i = 0; i < MESSAGE_SIZE; i++)
        {
            Serial.printf("%x ", m_txPayload.buff[i]);
        }
        Serial.printf(" ]\r\n");
#endif

        delay(100);

        Radio.Send(SX1262_ID_TX, m_txBuffer, APP_LORA_TX_CONFIG.buffSize);
    }

    if (m_rxDone)
    {
        m_rxDone = false;

        Serial.printf("RX counter: [%d]\r\n", m_rxPayload.counter);

#if defined (RX_PAYLOAD_PRINT)
        Serial.printf("RX packet size: [%d] bytes\r\n", m_rxPayload.size);
        Serial.printf("RX packet: [ ");
        Serial.printf("RSSI = [%d] dBm; SNR = [%d]\r\n", m_rxPayload.rssi, m_rxPayload.snr);

        for (uint8_t i = 0; i < m_rxPayload.size; i++)
        {
            Serial.printf("%02X ", m_rxBuffer[i]);
        }
        Serial.printf(" ]\r\n");
#endif

        Radio.Rx(SX1262_ID_RX, APP_LORA_RX_CONFIG.rxTimeout);
    }
}

/**@brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(SX1262_ID id)
{
    Serial.printf("OnTxDone SX1262 ID: [%d]\r\n", id);

    m_txDone = true;

    ++m_txPayload.counter;
}

/**@brief Function to be executed on Radio Rx Done event
 */
static void OnRxDone(SX1262_ID id, uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr)
{
    Serial.printf("OnRxDone SX1262 ID: [%d]\r\n", id);

    if (m_rxPayload.size <= MESSAGE_SIZE)
    {
        m_rxDone = true;
        memcpy(m_rxPayload.buff, payload, size);
        m_rxPayload.rssi = rssi;
        m_rxPayload.snr = snr;
        m_rxPayload.size = size;
        ++m_rxPayload.counter;
    }
    else
    {
        Serial.printf("Received something\r\n");
    }
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
static void OnTxTimeout(SX1262_ID id)
{
/* TODO: */
    Serial.printf("OnTxTimeout SX1262 ID: [%d]\r\n", id);
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
static void OnRxTimeout(SX1262_ID id)
{ 
    Serial.printf("OnRxTimeout SX1262 ID: [%d]\r\n", id);

    /* run CAD */
    Radio.Standby(id);
    Radio.SetCadParams(id, LORA_CAD_08_SYMBOL, LORA_SF12, LORA_SF7, LORA_CAD_ONLY, 0);
    m_cadTime = millis();

    Serial.printf("CAD start at: [%d] ms\r\n", m_cadTime);

    Radio.StartCad(id);
}

/**@brief Function to be executed on Radio Rx Error event
 */
static void OnRxError(SX1262_ID id)
{
    Serial.printf("OnRxError SX1262 ID: [%d]\r\n", id);

    /* restart Radio RX */
    Radio.Rx(id, APP_LORA_RX_CONFIG.rxTimeout);
}

/**@brief Function to be executed on CAD Done event
 */
static void OnCadDone(SX1262_ID id, bool cadResult)
{
    Serial.printf("OnCadDone SX1262 ID: [%d]\r\n", id);

    time_t cadStop = millis();
    time_t duration = cadStop - m_cadTime;

    Serial.printf("CAD stopped at: [%d] ms\r\n", cadStop);

    if (cadResult)
    {
        Serial.printf("CAD returned channel busy after [%d] ms\r\n", duration);

        Radio.Rx(id, APP_LORA_RX_CONFIG.rxTimeout);

        if (++m_txToStartCounter >= TICKS_TO_START_TX)
        {
            m_txToStart = true;
        }

        Serial.printf("To TX counter: [%d]\r\n", m_txToStartCounter);
    }
}
