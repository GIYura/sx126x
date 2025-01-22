//#include <Arduino.h>

//#include <SPI.h>
#include <assert.h>
#include <string.h>

#if 0
#include "sx126x.h"
#include "sx126x-regs.h"
#include "board.h"

/* SPI settings */
//static SPISettings m_spiSettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);

static bool m_imageCalibrated = false;
static volatile uint32_t m_frequencyError = 0;
static bool m_spiInitialized = false;

static void SX1262GpioPrint(SX1262_ID id, const BoardConfig_t* const boardConfig);

void SX126x::GpioInit(SX1262_ID id, const GpioConfig_t* const gpioConfig)
{
    assert(id < SX1262_ID_NUMBER);
    assert(gpioConfig);
#if 0
    m_boardConfig[id].PIN_LORA_BUSY = gpioConfig->busy;
    m_boardConfig[id].PIN_LORA_NSS = gpioConfig->nss;
    m_boardConfig[id].PIN_LORA_RESET = ESP32_RST;
    m_boardConfig[id].PIN_LORA_DIO = gpioConfig->dio;
#endif
#if 0
    pinMode(m_boardConfig[id].PIN_LORA_NSS, OUTPUT);
    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, HIGH);

    pinMode(m_boardConfig[id].PIN_LORA_BUSY, INPUT);
    pinMode(m_boardConfig[id].PIN_LORA_DIO, INPUT);
    pinMode(m_boardConfig[id].PIN_LORA_RESET, OUTPUT);
    digitalWrite(m_boardConfig[id].PIN_LORA_RESET, HIGH);
#endif

    /* default config */
    m_boardConfig[id].CHIP_TYPE = SX1262_CHIP;
    m_boardConfig[id].RADIO_RXEN = -1;             /* LORA ANTENNA RX ENABLE (eByte E22 module only) */
    m_boardConfig[id].RADIO_TXEN = -1;             /* LORA ANTENNA TX ENABLE (eByte E22 module only) */
    m_boardConfig[id].USE_DIO2_ANT_SWITCH = true;  /* DIO2 antena control */
    m_boardConfig[id].USE_DIO3_TCXO = false;       /* NOTE: not supported, should be = false */
    m_boardConfig[id].USE_DIO3_ANT_SWITCH = false; /* NOTE: DIO3 doesn't control antena */
    m_boardConfig[id].USE_LDO = false;             /* Whether SX126x uses LDO or DCDC power regulator */
    m_boardConfig[id].USE_RXEN_ANT_PWR = false;    /* Whether RX_EN is used as antenna power */
    m_boardConfig[id].TCXO_CTRL_VOLTAGE = TCXO_CTRL_1_7V;

    SX1262GpioPrint(id, &m_boardConfig[id]);

    SpiInit();
}

void SX126x::Begin(SX1262_ID id, DioIrqHandler dioIrq) 
{
    assert (id < SX1262_ID_NUMBER);

    //(id) ? Serial.printf("### SX1262 RX Begin\r\n") : Serial.printf("### SX1262 TX Begin\r\n");

    Reset(id);

    IoIrqInit(id, dioIrq);
    Wakeup(id);
    SetStandby(id, STDBY_RC);

    if (m_boardConfig[id].USE_DIO3_TCXO)
    {
/* NOTE: calibrate done when TCXO used */
        CalibrationParams_t calibParam;
        SetDio3AsTcxoCtrl(id, m_boardConfig[id].TCXO_CTRL_VOLTAGE, RADIO_TCXO_SETUP_TIME << 6);
        calibParam.Value = 0x7F;
        Calibrate(id, &calibParam);
    }

    /* setup according HW config */
    m_boardConfig[id].USE_DIO2_ANT_SWITCH ? SetDio2AsRfSwitchCtrl(id, true) : SetDio2AsRfSwitchCtrl(id, false);
    m_boardConfig[id].USE_LDO ?  SetRegulatorMode(id, USE_LDO) : SetRegulatorMode(id, USE_DCDC);

    SetOperatingMode(id, MODE_STDBY_RC);
}

void SX126x::ReInit(SX1262_ID id, DioIrqHandler dioIrq)
{
/* TODO: */
#if 0
    IoIrqInit(id, dioIrq);
#endif
}

void SX126x::SetPayload(SX1262_ID id, const uint8_t* const payload, uint8_t size)
{
    assert(id < SX1262_ID_NUMBER);
    assert(payload);

    WriteBuffer(id, 0x00, payload, size);
}

uint8_t SX126x::GetPayload(SX1262_ID id, uint8_t* const payload, uint8_t* const size, uint8_t maxSize)
{
    assert(id < SX1262_ID_NUMBER);
    assert(payload);
    assert(size);

    uint8_t offset = 0;

    GetRxBufferStatus(id, size, &offset);
    if (*size > maxSize)
    {
        return 1;
    }
    ReadBuffer(id, offset, payload, *size);
    return 0;
}

void SX126x::SendPayload(SX1262_ID id, const uint8_t* const payload, uint8_t size, uint32_t timeout)
{
    assert(id < SX1262_ID_NUMBER);
    assert(payload);

    SetPayload(id, payload, size);
    SetTx(id, timeout);
}

uint8_t SX126x::SetFskSyncWord(SX1262_ID id, const uint8_t* const syncWord)
{
    assert(id < SX1262_ID_NUMBER);
    assert(syncWord);

    WriteRegisters(id, SX126X_REG_SYNC_WORD_0, syncWord, 8);

    return 0;
}

void SX126x::SetFskCrcSeed(SX1262_ID id, uint16_t seed)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[2];

    buf[0] = (uint8_t)((seed >> 8) & 0xFF);
    buf[1] = (uint8_t)(seed & 0xFF);

    switch (GetPacketType(id))
    {
    case PACKET_TYPE_GFSK:
        WriteRegisters(id, SX126X_REG_CRC_INITIAL_MSB, buf, 2);
        break;

    default:
        break;
    }
}

void SX126x::SetFskCrcPolynomial(SX1262_ID id, uint16_t polynomial)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[2];

    buf[0] = (uint8_t)((polynomial >> 8) & 0xFF);
    buf[1] = (uint8_t)(polynomial & 0xFF);

    switch (GetPacketType(id))
    {
    case PACKET_TYPE_GFSK:
        WriteRegisters(id, SX126X_REG_CRC_POLYNOMIAL_MSB, buf, 2);
        break;

    default:
        break;
    }
}

void SX126x::SetFskWhiteningSeed(SX1262_ID id, uint16_t seed)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t regValue = 0;

    switch (GetPacketType(id))
    {
    case PACKET_TYPE_GFSK:
        regValue = ReadRegister(id, SX126X_REG_WHITENING_INITIAL_MSB) & 0xFE;
        regValue = ((seed >> 8) & 0x01) | regValue;
        WriteRegister(id, SX126X_REG_WHITENING_INITIAL_MSB, regValue);
        WriteRegister(id, SX126X_REG_WHITENING_INITIAL_LSB, (uint8_t)seed);
        break;

    default:
        break;
    }
}

/* Table 11-1: Commands Selecting the Operating Modes of the Radio */
void SX126x::SetSleep(SX1262_ID id, const SleepParams_t* const sleepConfig)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t value = sleepConfig->Value;

    WriteCommand(id, RADIO_SET_SLEEP, &value, 1);

    SetOperatingMode(id, MODE_SLEEP);
}

void SX126x::SetStandby(SX1262_ID id, RadioStandbyModes_t standbyMode)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t mode = (uint8_t)standbyMode;

    WriteCommand(id, RADIO_SET_STANDBY, &mode, 1);

    (mode == STDBY_RC) ? SetOperatingMode(id, MODE_STDBY_RC) : SetOperatingMode(id, MODE_STDBY_XOSC);
}

void SX126x::SetFs(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);

    WriteCommand(id, RADIO_SET_FS, 0, 0);

    SetOperatingMode(id, MODE_FS);
}

void SX126x::SetTx(SX1262_ID id, uint32_t timeout)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[3];

    buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
    buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
    buf[2] = (uint8_t)(timeout & 0xFF);
    WriteCommand(id, RADIO_SET_TX, buf, 3);

    SetOperatingMode(id, MODE_TX);
}

void SX126x::SetRx(SX1262_ID id, uint32_t timeout)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[3];

    WriteRegister(id, SX126X_REG_RX_GAIN, 0x94); /* default gain */

    buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
    buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
    buf[2] = (uint8_t)(timeout & 0xFF);
    WriteCommand(id, RADIO_SET_RX, buf, 3);

    SetOperatingMode(id, MODE_RX);
}

void SX126x::SetStopRxTimerOnPreambleDetect(SX1262_ID id, bool enable)
{
    assert(id < SX1262_ID_NUMBER);

    WriteCommand(id, RADIO_SET_STOPRXTIMERONPREAMBLE, (uint8_t*)&enable, 1);
}

void SX126x::SetRxDutyCycle(SX1262_ID id, uint32_t rxTime, uint32_t sleepTime)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[6];

    buf[0] = (uint8_t)((rxTime >> 16) & 0xFF);
    buf[1] = (uint8_t)((rxTime >> 8) & 0xFF);
    buf[2] = (uint8_t)(rxTime & 0xFF);
    buf[3] = (uint8_t)((sleepTime >> 16) & 0xFF);
    buf[4] = (uint8_t)((sleepTime >> 8) & 0xFF);
    buf[5] = (uint8_t)(sleepTime & 0xFF);
    WriteCommand(id, RADIO_SET_RXDUTYCYCLE, buf, 6);

    SetOperatingMode(id, MODE_RX_DC);
}

void SX126x::SetCad(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);

    WriteCommand(id, RADIO_SET_CAD, 0, 0);

    SetOperatingMode(id, MODE_CAD);
}

void SX126x::SetTxContinuousWave(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);

    WriteCommand(id, RADIO_SET_TXCONTINUOUSWAVE, 0, 0);
}

void SX126x::SetTxInfinitePreamble(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);

    WriteCommand(id, RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0);
}

void SX126x::SetRegulatorMode(SX1262_ID id, RadioRegulatorMode_t mode)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t value = (uint8_t)mode;

    WriteCommand(id, RADIO_SET_REGULATORMODE, &value, 1);
}

void SX126x::Calibrate(SX1262_ID id, const CalibrationParams_t* const calibParam)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t value = calibParam->Value;

    WriteCommand(id, RADIO_CALIBRATE, &value, 1);
}

void SX126x::CalibrateImage(SX1262_ID id, uint32_t frequency)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t calFreq[2] = {0x00, 0x00};

    if (frequency > 900000000)
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if (frequency > 850000000)
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xDB;
    }
    else if (frequency > 770000000)
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if (frequency > 460000000)
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if (frequency > 425000000)
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }

    WriteCommand(id, RADIO_CALIBRATEIMAGE, calFreq, 2);
}

void SX126x::SetPaConfig(SX1262_ID id, uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[4];

    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    WriteCommand(id, RADIO_SET_PACONFIG,  buf, 4);
}

void SX126x::SetRxTxFallbackMode(SX1262_ID id, uint8_t fallbackMode)
{
    assert(id < SX1262_ID_NUMBER);

    WriteCommand(id, RADIO_SET_TXFALLBACKMODE, &fallbackMode, 1); 
}

/* Table 11-2: Commands to Access the Radio Registers and FIFO Buffer */
void SX126x::WriteRegisters(SX1262_ID id, uint16_t address, const uint8_t* const buffer, uint16_t size)
{
    assert(id < SX1262_ID_NUMBER);
    assert(buffer);

    CheckDeviceReady(id);
#if 0

    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, LOW);

    SPI.beginTransaction(m_spiSettings);
    SPI.transfer(RADIO_WRITE_REGISTER);
    SPI.transfer((address & 0xFF00) >> 8);
    SPI.transfer(address & 0x00FF);

    for (uint16_t i = 0; i < size; i++)
    {
        SPI.transfer(buffer[i]);
    }

    SPI.endTransaction();
    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, HIGH);

    WaitOnBusy(id);
#endif
}

void SX126x::ReadRegisters(SX1262_ID id, uint16_t address, uint8_t* const buffer, uint16_t size)
{
    assert(id < SX1262_ID_NUMBER);
    assert(buffer);

    CheckDeviceReady(id);
#if 0
    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, LOW);

    SPI.beginTransaction(m_spiSettings);
    SPI.transfer(RADIO_READ_REGISTER);
    SPI.transfer((address & 0xFF00) >> 8);
    SPI.transfer(address & 0x00FF);
    SPI.transfer(0x00);
    for (uint16_t i = 0; i < size; i++)
    {
        buffer[i] = SPI.transfer(0x00);
    }
    SPI.endTransaction();

    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, HIGH);

    WaitOnBusy(id);
#endif
}

void SX126x::WriteBuffer(SX1262_ID id, uint8_t offset, const uint8_t* const buffer, uint8_t size)
{
    assert(id < SX1262_ID_NUMBER);
    assert(buffer);

    CheckDeviceReady(id);

#if 0
    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, LOW);

    SPI.beginTransaction(m_spiSettings);
    SPI.transfer(RADIO_WRITE_BUFFER);
    SPI.transfer(offset);
    for (uint16_t i = 0; i < size; i++)
    {
        SPI.transfer(buffer[i]);
    }
    SPI.endTransaction();

    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, HIGH);

    WaitOnBusy(id);
#endif
}

void SX126x::ReadBuffer(SX1262_ID id, uint8_t offset, uint8_t* const buffer, uint8_t size)
{
    assert(id < SX1262_ID_NUMBER);
    assert(buffer);

    CheckDeviceReady(id);

#if 0
    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, LOW);

    SPI.beginTransaction(m_spiSettings);
    SPI.transfer(RADIO_READ_BUFFER);
    SPI.transfer(offset);
    SPI.transfer(0x00);
    for (uint16_t i = 0; i < size; i++)
    {
        buffer[i] = SPI.transfer(0x00);
    }
    SPI.endTransaction();

    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, HIGH);

    WaitOnBusy(id);
#endif
}

void SX126x::WriteRegister(SX1262_ID id, uint16_t address, uint8_t value)
{
    assert(id < SX1262_ID_NUMBER);

    WriteRegisters(id, address, &value, 1);
}

uint8_t SX126x::ReadRegister(SX1262_ID id, uint16_t address)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t data;
    ReadRegisters(id, address, &data, 1);
    return data;
}

/* Table 11-3: Commands Controlling the Radio IRQs and DIOs */
void SX126x::SetDioIrqParams(SX1262_ID id, uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);

    WriteCommand(id, RADIO_CFG_DIOIRQ, buf, 8);
}

uint16_t SX126x::GetIrqStatus(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t irqStatus[2];
    ReadCommand(id, RADIO_GET_IRQSTATUS, irqStatus, 2);
    return (irqStatus[0] << 8) | irqStatus[1];
}

void SX126x::ClearIrqStatus(SX1262_ID id, uint16_t irq)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irq & 0x00FF);
    WriteCommand(id, RADIO_CLR_IRQSTATUS, buf, 2);
}

void SX126x::SetDio2AsRfSwitchCtrl(SX1262_ID id, uint8_t enable)
{
    assert(id < SX1262_ID_NUMBER);

    WriteCommand(id, RADIO_SET_RFSWITCHMODE, &enable, 1);
}

void SX126x::SetDio3AsTcxoCtrl(SX1262_ID id, RadioTcxoCtrlVoltage_t tcxoVoltage, uint32_t timeout)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buff[4];

    buff[0] = tcxoVoltage & 0x07;
    
    if (timeout == 0)
    {
        timeout = 5000 / 15.625;
    }
    else
    {
        timeout = timeout / 15.625;
    }

    buff[1] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buff[2] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buff[3] = ( uint8_t )( timeout & 0xFF );

    WriteCommand(id, RADIO_SET_TCXOMODE, buff, 4);
}

/* Table 11-4: Commands Controlling the RF and Packets Settings */
void SX126x::SetRfFrequency(SX1262_ID id, uint32_t frequency)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[4];
    uint32_t freq = 0;

/* TODO: image calibrate should be done when TCXO used */
#if 0
    if (!m_imageCalibrated)
    {
        m_imageCalibrated = true;
        CalibrateImage(id, frequency);
    }
#endif
    //freq = (uint32_t)((double)frequency / (double)FREQ_STEP);

    buf[0] = (uint8_t)((freq >> 24) & 0xFF);
    buf[1] = (uint8_t)((freq >> 16) & 0xFF);
    buf[2] = (uint8_t)((freq >> 8) & 0xFF);
    buf[3] = (uint8_t)(freq & 0xFF);
    
    WriteCommand(id, RADIO_SET_RFFREQUENCY, buf, 4);
}

void SX126x::SetPacketType(SX1262_ID id, RadioPacketTypes_t packetType)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t value = (uint8_t)packetType;

    m_packetType[id] = packetType;

    WriteCommand(id, RADIO_SET_PACKETTYPE, &value, 1);
}

RadioPacketTypes_t SX126x::GetPacketType(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);
#if 1
    return m_packetType[id];
#else
    uint8_t pktType = 0;
    ReadCommand(id, RADIO_GET_PACKETTYPE, (uint8_t *)&pktType, 1);
    return pktType;
#endif
}

void SX126x::SetTxParams(SX1262_ID id, int8_t power, RadioRampTimes_t rampTime)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[2];

    if (m_boardConfig[id].CHIP_TYPE == SX1261_CHIP)
    {
        if (power > 14)
        {
            SetPaConfig(id, 0x06, 0x00, 0x01, 0x01);
        }
        else
        {
            SetPaConfig(id, 0x04, 0x00, 0x01, 0x01);
        }
        if (power >= 14)
        {
            power = 14;
        }
        else if (power < -17)
        {
            power = -17;
        }
        WriteRegister(id, SX126X_REG_OCP_CONFIGURATION, 0x18); /* current max is 80 mA for the whole device */
    }
    else
    {
        /* WORKAROUND */
        Workaround_AntennaMismatch(id);
        /* WORKAROUND */

        SetPaConfig(id, 0x04, 0x07, 0x00, 0x01);
        if (power > 22)
        {
            power = 22;
        }
        else if (power < -9)
        {
            power = -9;
        }
        WriteRegister(id, SX126X_REG_OCP_CONFIGURATION, 0x38); /* current max 160mA for the whole device */
    }
    buf[0] = power;
    buf[1] = (uint8_t)rampTime;
    WriteCommand(id, RADIO_SET_TXPARAMS, buf, 2);
}

void SX126x::SetModulationParams(SX1262_ID id, const ModulationParams_t* const modulationParams)
{
    assert(id < SX1262_ID_NUMBER);
    assert(modulationParams);

    uint8_t n = 0;
    uint32_t tempVal = 0;
    uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* NOTE: */
#if 0
    /* NOTE: Check if required configuration corresponds to the stored packet type
    If not, silently update radio packet type */
    if (m_packetType[id] != modulationParams->PacketType)
    {
        SetPacketType(id, modulationParams->PacketType);
    }
#endif
    
    switch (modulationParams->PacketType)
    {
    case PACKET_TYPE_GFSK:
        n = 8;
        tempVal = (uint32_t)(32 * ((double)XTAL_FREQ / (double)modulationParams->Params.Gfsk.BitRate));
        buf[0] = (tempVal >> 16) & 0xFF;
        buf[1] = (tempVal >> 8) & 0xFF;
        buf[2] = tempVal & 0xFF;
        buf[3] = modulationParams->Params.Gfsk.ModulationShaping;
        buf[4] = modulationParams->Params.Gfsk.Bandwidth;
        //tempVal = (uint32_t)((double)modulationParams->Params.Gfsk.Fdev / (double)FREQ_STEP);
        buf[5] = (tempVal >> 16) & 0xFF;
        buf[6] = (tempVal >> 8) & 0xFF;
        buf[7] = (tempVal & 0xFF);
        break;

    case PACKET_TYPE_LORA:
        n = 4;
        buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
        buf[1] = modulationParams->Params.LoRa.Bandwidth;
        buf[2] = modulationParams->Params.LoRa.CodingRate;
        buf[3] = modulationParams->Params.LoRa.LowDatarateOptimize;
        break;

    default:
        //Serial.printf("SX1262: Failed to set modulation params\r\n");
        return;
        break;
    }

    WriteCommand(id, RADIO_SET_MODULATIONPARAMS, buf, n);
}
 
void SX126x::SetPacketParams(SX1262_ID id, const PacketParams_t* const packetParams)
{
    assert(id < SX1262_ID_NUMBER);
    assert(packetParams);

    uint8_t n = 0;
    uint8_t crcVal = 0;
    uint8_t buf[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* NOTE: */
#if 0
    /* NOTE: Check if required configuration corresponds to the stored packet type
    If not, silently update radio packet type */
    if (m_packetType[id] != packetParams->PacketType)
    {
        SetPacketType(id, packetParams->PacketType);
    }
#endif

    switch (packetParams->PacketType)
    {
    case PACKET_TYPE_GFSK:
        if (packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_IBM)
        {
            SetFskCrcSeed(id, CRC_IBM_SEED);
            SetFskCrcPolynomial(id, CRC_POLYNOMIAL_IBM);
            crcVal = RADIO_CRC_2_BYTES;
        }
        else if (packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_CCIT)
        {
            SetFskCrcSeed(id, CRC_CCITT_SEED);
            SetFskCrcPolynomial(id, CRC_POLYNOMIAL_CCITT);
            crcVal = RADIO_CRC_2_BYTES_INV;
        }
        else
        {
            crcVal = packetParams->Params.Gfsk.CrcLength;
        }
        n = 9;
        buf[0] = (packetParams->Params.Gfsk.PreambleLength >> 8) & 0xFF;
        buf[1] = packetParams->Params.Gfsk.PreambleLength;
        buf[2] = packetParams->Params.Gfsk.PreambleMinDetect;
        buf[3] = (packetParams->Params.Gfsk.SyncWordLength);
        buf[4] = packetParams->Params.Gfsk.AddrComp;
        buf[5] = packetParams->Params.Gfsk.HeaderType;
        buf[6] = packetParams->Params.Gfsk.PayloadLength;
        buf[7] = crcVal;
        buf[8] = packetParams->Params.Gfsk.DcFree;
        break;

    case PACKET_TYPE_LORA:
        n = 6;
        buf[0] = (packetParams->Params.LoRa.PreambleLength >> 8) & 0xFF;
        buf[1] = packetParams->Params.LoRa.PreambleLength;
        buf[2] = packetParams->Params.LoRa.HeaderType;
        buf[3] = packetParams->Params.LoRa.PayloadLength;
        buf[4] = packetParams->Params.LoRa.CrcMode;
        buf[5] = packetParams->Params.LoRa.InvertIQ;
        break;

    default:
        //Serial.printf("SX1262: Failed to set packet params\r\n");
        return;
        break;
    }
    
    WriteCommand(id, RADIO_SET_PACKETPARAMS, buf, n);
}

void SX126x::SetCadParams(  SX1262_ID id,
                            RadioLoRaCadSymbols_t cadSymbolNum,
                            uint8_t cadDetPeak,
                            uint8_t cadDetMin,
                            RadioCadExitModes_t cadExitMode,
                            uint32_t cadTimeout)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[7];

    buf[0] = (uint8_t)cadSymbolNum;
    buf[1] = cadDetPeak;
    buf[2] = cadDetMin;
    buf[3] = (uint8_t)cadExitMode;
    buf[4] = (uint8_t)((cadTimeout >> 16) & 0xFF);
    buf[5] = (uint8_t)((cadTimeout >> 8) & 0xFF);
    buf[6] = (uint8_t)(cadTimeout & 0xFF);
    WriteCommand(id, RADIO_SET_CADPARAMS, buf, 7);

    SetOperatingMode(id, MODE_CAD);
}

void SX126x::SetBufferBaseAddress(SX1262_ID id, uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[2] = {0x00, 0x00};

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    WriteCommand(id, RADIO_SET_BUFFERBASEADDRESS, buf, 2);
}

void SX126x::SetLoRaSymbNumTimeout(SX1262_ID id, uint8_t symbNum)
{
    assert(id < SX1262_ID_NUMBER);

    WriteCommand(id, RADIO_SET_LORASYMBTIMEOUT, &symbNum, 1);
}

/* Table 11-5: Commands Returning the Radio Status */
RadioStatus_t SX126x::GetStatus(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t stat = 0;
    RadioStatus_t status;

    ReadCommand(id, RADIO_GET_STATUS, (uint8_t *)&stat, 1);
    status.Value = stat;
    return status;
}

int8_t SX126x::GetRssiInst(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[1];
    int8_t rssi = 0;

    ReadCommand(id, RADIO_GET_RSSIINST, buf, 1);
    rssi = -buf[0] >> 1;
    return rssi;
}

void SX126x::GetRxBufferStatus(SX1262_ID id, uint8_t* const payloadLength, uint8_t* const rxStartBufferPointer)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[2];

    ReadCommand(id, RADIO_GET_RXBUFFERSTATUS, buf, 2);

    *payloadLength = buf[0];
    *rxStartBufferPointer = buf[1];
}

void SX126x::GetPacketStatus(SX1262_ID id, PacketStatus_t* const pktStatus)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t status[3];

    ReadCommand(id, RADIO_GET_PACKETSTATUS, status, 3);

    pktStatus->packetType = GetPacketType(id);

    switch (pktStatus->packetType)
    {
    case PACKET_TYPE_GFSK:
        pktStatus->Params.Gfsk.RxStatus = status[0];
        pktStatus->Params.Gfsk.RssiSync = -status[1] >> 1;
        pktStatus->Params.Gfsk.RssiAvg = -status[2] >> 1;
        pktStatus->Params.Gfsk.FreqError = 0;
        break;

    case PACKET_TYPE_LORA:
        pktStatus->Params.LoRa.RssiPkt = -status[0] >> 1;
        /* Returns SNR value [dB] rounded to the nearest integer value */
        pktStatus->Params.LoRa.SnrPkt = (((int8_t)status[1]) + 2) >> 2;
        pktStatus->Params.LoRa.SignalRssiPkt = -status[2] >> 1;
        pktStatus->Params.LoRa.FreqError = m_frequencyError;
        break;

    default:
    case PACKET_TYPE_NONE:
        /* In that specific case, we set everything in the pktStatus to zeros
        and reset the packet type accordingly */
        memset(pktStatus, 0, sizeof(PacketStatus_t));
        pktStatus->packetType = PACKET_TYPE_NONE;
        break;
    }
}

RadioError_t SX126x::GetDeviceErrors(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);

    RadioError_t error;

    ReadCommand(id, RADIO_GET_ERROR, (uint8_t *)&error, 2);
    return error;
}

void SX126x::ClearDeviceErrors(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[2] = {0x00, 0x00};
    WriteCommand(id, RADIO_CLR_ERROR, buf, 2);
}

/* SX126x auxiliary commands */
RadioOperatingModes_t SX126x::GetOperatingMode(SX1262_ID id)
{
    assert (id < SX1262_ID_NUMBER);

    return m_operatingMode[id];
}

void SX126x::SetOperatingMode(SX1262_ID id, RadioOperatingModes_t mode)
{
    assert (id < SX1262_ID_NUMBER);

    m_operatingMode[id] = mode;
}

uint32_t SX126x::GetRandom(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[4] = {0, 0, 0, 0};

    ReadRegisters(id, SX126X_REG_RANDOM_NUMBER_0, buf, 4);

    return (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
}

void SX126x::SetRxBoosted(SX1262_ID id, uint32_t timeout)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[3];

    WriteRegister(id, SX126X_REG_RX_GAIN, 0x96); /* max LNA gain, increase current by ~2mA for around ~3dB in sensivity */

    buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
    buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
    buf[2] = (uint8_t)(timeout & 0xFF);
    WriteCommand(id, RADIO_SET_RX, buf, 3);

    SetOperatingMode(id, MODE_RX);
}

void SX126x::SetRfTxPower(SX1262_ID id, int8_t power)
{
    assert(id < SX1262_ID_NUMBER);

    SetTxParams(id, power, RADIO_RAMP_40_US);
}

uint32_t SX126x::GetTcxoWakeupTime(void)
{
    return RADIO_TCXO_SETUP_TIME;
}

/*
NOTE: private
*/
void SX126x::SpiInit(void)
{
    if (!m_spiInitialized)
    {
        m_spiInitialized = true;

        //SPI.begin(ESP32_SCK, ESP32_MISO, ESP32_MOSI);
    }
}

void SX126x::IoIrqInit(SX1262_ID id, DioIrqHandler dioIrq)
{
    assert(id < SX1262_ID_NUMBER);

    //uint8_t dioPin = digitalPinToInterrupt(m_boardConfig[id].PIN_LORA_DIO);

    //attachInterrupt(dioPin, dioIrq, RISING);
}

void SX126x::Reset(SX1262_ID id)
{
    assert (id < SX1262_ID_NUMBER);
#if 0
    pinMode(m_boardConfig[id].PIN_LORA_RESET, OUTPUT);
    digitalWrite(m_boardConfig[id].PIN_LORA_RESET, LOW);
    delay(10);
    digitalWrite(m_boardConfig[id].PIN_LORA_RESET, HIGH);
    delay(20);
#endif
}

void SX126x::WaitOnBusy(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);

    uint16_t timeout = 1000;

    //while ((digitalRead(m_boardConfig[id].PIN_LORA_BUSY) == HIGH) && (--timeout > 0));
}

void SX126x::Wakeup(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);
#if 0
    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, LOW);

    SPI.beginTransaction(m_spiSettings);
    SPI.transfer(RADIO_GET_STATUS);
    SPI.transfer(0x00);
    SPI.endTransaction();

    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, HIGH);
#endif
    WaitOnBusy(id);

    SetOperatingMode(id, MODE_STDBY_RC);
}

void SX126x::WriteCommand(SX1262_ID id, RadioCommands_t command, const uint8_t* const buffer, uint16_t size)
{
    assert(id < SX1262_ID_NUMBER);

    CheckDeviceReady(id);
#if 0
    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, LOW);

    SPI.beginTransaction(m_spiSettings);
    SPI.transfer((uint8_t)command);

    for (uint16_t i = 0; i < size; i++)
    {
        SPI.transfer(buffer[i]);
    }

    SPI.endTransaction();

    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, HIGH);

    if (command != RADIO_SET_SLEEP)
    {
        WaitOnBusy(id);
    }
#endif
}

void SX126x::ReadCommand(SX1262_ID id, RadioCommands_t command, uint8_t* const buffer, uint16_t size)
{
    assert(id < SX1262_ID_NUMBER);

    CheckDeviceReady(id);
#if 0
    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, LOW);

    SPI.beginTransaction(m_spiSettings);
    SPI.transfer((uint8_t)command);
    SPI.transfer(0x00);
    for (uint16_t i = 0; i < size; i++)
    {
        buffer[i] = SPI.transfer(0x00);
    }

    SPI.endTransaction();

    digitalWrite(m_boardConfig[id].PIN_LORA_NSS, HIGH);

    WaitOnBusy(id);
#endif
}

/* Table 11-5: Commands Returning the Radio Status */
void SX126x::GetStats(SX1262_ID id, uint16_t* const nbPacketReceived, uint16_t* const nbPacketCrcError, uint16_t* const nbPacketLenError)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[6];

    ReadCommand(id, RADIO_GET_STATS, buf, 6);

    *nbPacketReceived = (buf[0] << 8) | buf[1];
    *nbPacketCrcError = (buf[2] << 8) | buf[3];
    *nbPacketLenError = (buf[4] << 8) | buf[5];
}

void SX126x::ResetStats(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);

    uint8_t buf[6] = {0x00};

    WriteCommand(id, RADIO_RESET_STATS, buf, 6);
}

void SX126x::CheckDeviceReady(SX1262_ID id)
{
    assert(id < SX1262_ID_NUMBER);

    if ((GetOperatingMode(id) == MODE_SLEEP) || (GetOperatingMode(id) == MODE_RX_DC))
    {
        Wakeup(id);
    }
    WaitOnBusy(id);
}

/* NOTE: WORKAROUNDS */
void SX126x::Workaround_LoRaBW500(SX1262_ID id, RadioLoRaBandwidths_t bandwidth)
{
/*
15.1 Modulation Quality with 500 kHz LoRa Bandwidth
15.1.1 Description
Some sensitivity degradation may be observed on any LoRa device, when receiving signals transmitted by the SX1268 with
LoRa BW of 500 kHz.
*/
    assert(id < SX1262_ID_NUMBER);

    uint8_t txModulationReg = 0;

    if ((m_packetType[id] == PACKET_TYPE_LORA) && (bandwidth == LORA_BW_500))
    {
        /* RegTxModulation = @address 0x0889 */
        txModulationReg = ReadRegister(id, SX126X_REG_TX_MODULATION);
        txModulationReg &= ~(1 << 2);
        WriteRegister(id, SX126X_REG_TX_MODULATION, txModulationReg);
    }
    else
    {
        /* RegTxModulation = @address 0x0889 */
        txModulationReg = ReadRegister(id, SX126X_REG_TX_MODULATION);
        txModulationReg |= (1 << 2);
        WriteRegister(id, SX126X_REG_TX_MODULATION, txModulationReg);
    }
}

void SX126x::Workaround_AntennaMismatch(SX1262_ID id)
{
/*
Better Resistance of the SX1262 Tx to Antenna Mismatch
15.2 Better Resistance of the SX1268 Tx to Antenna Mismatch
*/
    assert(id < SX1262_ID_NUMBER);

    uint8_t txClamp = 0;

    txClamp = ReadRegister(id, SX126X_REG_TX_CLAMP_CONFIGURATION);
    txClamp |= 0x1E;
    WriteRegister(id, SX126X_REG_TX_CLAMP_CONFIGURATION, txClamp);
}

void SX126x::Workaround_InvertedIqOpertation(SX1262_ID id, bool irqPolarityInverted)
{
/*
15.4 Optimizing the Inverted IQ Operation

Bit 2 at address 0x0736 must be set to:
•“0” when using inverted IQ polarity (see the SetPacketParam(...) command)
•“1” when using standard IQ polarity
*/
    assert(id < SX1262_ID_NUMBER);

    uint8_t irqPol = 0;

    if (irqPolarityInverted == LORA_IQ_INVERTED)
    {
        irqPol = ReadRegister(id, SX126X_REG_IRQ_POLARITY);
        irqPol &= ~(1 << 2);
        WriteRegister(id, SX126X_REG_IRQ_POLARITY, irqPol);

    }
    else
    {
        irqPol = ReadRegister(id, SX126X_REG_IRQ_POLARITY);
        irqPol |= (1 << 2);
        WriteRegister(id, SX126X_REG_IRQ_POLARITY, irqPol);
    }
}

void SX126x::Workaround_ImplicitHeaderModeTimeout(SX1262_ID id)
{
/*
15.3 Implicit Header Mode Timeout Behavior
*/
    assert(id < SX1262_ID_NUMBER);

    uint8_t value = 0;

    WriteRegister(id, SX126X_REG_RTC_CONTROL, 0);

    value = ReadRegister(id, 0x0944);
    value |= 0x02;
    WriteRegister(id, 0x0944, value);
}
/* NOTE: WORKAROUNDS */

static void SX1262GpioPrint(SX1262_ID id, const BoardConfig_t* const boardConfig)
{
    assert(id < SX1262_ID_NUMBER);
#if 0
    (id) ? Serial.printf(" ### SX1262 RX GPIO Init\r\n") : Serial.printf(" ### SX1262 TX GPIO Init\r\n");

    Serial.printf("SX1262 ID [%s] BUSY pin [%d]\r\n", (id == SX1262_ID_TX) ? "RX" : "TX", boardConfig->PIN_LORA_BUSY);
    Serial.printf("SX1262 ID [%d] DIO pin [%d]\r\n", id, boardConfig->PIN_LORA_DIO);
    Serial.printf("SX1262 ID [%d] NSS pin [%d]\r\n", id, boardConfig->PIN_LORA_NSS);
    Serial.printf("SX1262 ID [%d] RESET pin [%d]\r\n", id, boardConfig->PIN_LORA_RESET);
#endif
}
#endif
