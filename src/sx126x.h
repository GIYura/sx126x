#ifndef SX126X_H
#define SX126X_H

#include "board.h"
#include "sx126x-definitions.h"

class SX126x
{
    public:
        SX126x(){};

        /* SX126x init */
        void GpioInit(SX1262_ID id, const GpioConfig_t* const gpioConfig);
        void Begin(SX1262_ID id, DioIrqHandler dioIrq);
        void ReInit(SX1262_ID id, DioIrqHandler dioIrq);

        /* SX126x payload settings */
        void SetPayload(SX1262_ID id, const uint8_t* const payload, uint8_t size);
        uint8_t GetPayload(SX1262_ID id, uint8_t* const payload, uint8_t* const size, uint8_t maxSize);
        void SendPayload(SX1262_ID id, const uint8_t* const payload, uint8_t size, uint32_t timeout);
        
        /* SX126x FSK settings */
        uint8_t SetFskSyncWord(SX1262_ID id, const uint8_t* const syncWord);
        void SetFskCrcSeed(SX1262_ID id, uint16_t seed);
        void SetFskCrcPolynomial(SX1262_ID id, uint16_t polynomial);
        void SetFskWhiteningSeed(SX1262_ID id, uint16_t seed);

        /* Table 11-1: Commands Selecting the Operating Modes of the Radio */
        void SetSleep(SX1262_ID id, const SleepParams_t* const sleepConfig);
        void SetStandby(SX1262_ID id, RadioStandbyModes_t standbyMode);
        void SetFs(SX1262_ID id);
        void SetTx(SX1262_ID id, uint32_t timeout);
        void SetRx(SX1262_ID id, uint32_t timeout);
        void SetStopRxTimerOnPreambleDetect(SX1262_ID id, bool enable);
        void SetRxDutyCycle(SX1262_ID id, uint32_t rxTime, uint32_t sleepTime);
        void SetCad(SX1262_ID id);
        void SetTxContinuousWave(SX1262_ID id);
        void SetTxInfinitePreamble(SX1262_ID id);
        void SetRegulatorMode(SX1262_ID id, RadioRegulatorMode_t mode);
        void Calibrate(SX1262_ID id, const CalibrationParams_t* const calibParam);
        void CalibrateImage(SX1262_ID id, uint32_t frequency);
        void SetPaConfig(SX1262_ID id, uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut);
        void SetRxTxFallbackMode(SX1262_ID id, uint8_t fallbackMode);

        /* Table 11-2: Commands to Access the Radio Registers and FIFO Buffer */
        void WriteRegisters(SX1262_ID id, uint16_t address, const uint8_t* const buffer, uint16_t size);
        void ReadRegisters(SX1262_ID id, uint16_t address, uint8_t* const buffer, uint16_t size);
        void WriteBuffer(SX1262_ID id, uint8_t offset, const uint8_t* const buffer, uint8_t size);
        void ReadBuffer(SX1262_ID id, uint8_t offset, uint8_t* const buffer, uint8_t size);
        void WriteRegister(SX1262_ID id, uint16_t address, uint8_t value);
        uint8_t ReadRegister(SX1262_ID id, uint16_t address);

        /* Table 11-3: Commands Controlling the Radio IRQs and DIOs */
        void SetDioIrqParams(SX1262_ID id, uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
        uint16_t GetIrqStatus(SX1262_ID id);
        void ClearIrqStatus(SX1262_ID id, uint16_t irq);
        void SetDio2AsRfSwitchCtrl(SX1262_ID id, uint8_t enable);
        void SetDio3AsTcxoCtrl(SX1262_ID id, RadioTcxoCtrlVoltage_t tcxoVoltage, uint32_t timeout);

        /* Table 11-4: Commands Controlling the RF and Packets Settings */
        void SetRfFrequency(SX1262_ID id, uint32_t frequency);
        void SetPacketType(SX1262_ID id, RadioPacketTypes_t packetType);
        RadioPacketTypes_t GetPacketType(SX1262_ID id);
        void SetTxParams(SX1262_ID id, int8_t power, RadioRampTimes_t rampTime);
        void SetModulationParams(SX1262_ID id, const ModulationParams_t* const modulationParams);
        void SetPacketParams(SX1262_ID id, const PacketParams_t* const packetParams);
        void SetCadParams(  SX1262_ID id,
                            RadioLoRaCadSymbols_t cadSymbolNum,
                            uint8_t cadDetPeak,
                            uint8_t cadDetMin,
                            RadioCadExitModes_t cadExitMode,
                            uint32_t cadTimeout);
        void SetBufferBaseAddress(SX1262_ID id, uint8_t txBaseAddress, uint8_t rxBaseAddress);
        void SetLoRaSymbNumTimeout(SX1262_ID id, uint8_t symbNum);

        /* Table 11-5: Commands Returning the Radio Status */
        RadioStatus_t GetStatus(SX1262_ID id);
        int8_t GetRssiInst(SX1262_ID id);
        void GetRxBufferStatus(SX1262_ID id, uint8_t* const payloadLength, uint8_t* const rxStartBufferPointer);
        void GetPacketStatus(SX1262_ID id, PacketStatus_t* const pktStatus);
        RadioError_t GetDeviceErrors(SX1262_ID id);
        void ClearDeviceErrors(SX1262_ID id);

        /* SX126x auxiliary commands */
        RadioOperatingModes_t GetOperatingMode(SX1262_ID id);
        void SetOperatingMode(SX1262_ID id, RadioOperatingModes_t mode);
        uint32_t GetRandom(SX1262_ID id);
        void SetRxBoosted(SX1262_ID id, uint32_t timeout);
        void SetRfTxPower(SX1262_ID id, int8_t power);
        uint32_t GetTcxoWakeupTime(void);

/* NOTE: WORKAROUND */
        void Workaround_LoRaBW500(SX1262_ID id, RadioLoRaBandwidths_t bandwidth);
        void Workaround_InvertedIqOpertation(SX1262_ID id, bool irqPolarityInverted);
        void Workaround_ImplicitHeaderModeTimeout(SX1262_ID id);
/* NOTE: WORKAROUND */

private:
        void SpiInit(void);
        void IoIrqInit(SX1262_ID id, DioIrqHandler dioIrq);
        void Reset(SX1262_ID id);
        void WaitOnBusy(SX1262_ID id);
        void Wakeup(SX1262_ID id);

        void WriteCommand(SX1262_ID id, RadioCommands_t command, const uint8_t* const buffer, uint16_t size);
        void ReadCommand(SX1262_ID id, RadioCommands_t command, uint8_t* const buffer, uint16_t size);

        /* Table 11-5: Commands Returning the Radio Status */
        void GetStats(SX1262_ID id, uint16_t* const nbPacketReceived, uint16_t* const nbPacketCrcError, uint16_t* const nbPacketLenError);
        void ResetStats(SX1262_ID id);

        void CheckDeviceReady(SX1262_ID id);

/* NOTE: WORKAROUND */
        void Workaround_AntennaMismatch(SX1262_ID id);
/* NOTE: WORKAROUND */

        BoardConfig_t m_boardConfig[SX1262_ID_NUMBER];
        RadioOperatingModes_t m_operatingMode[SX1262_ID_NUMBER];
        RadioPacketTypes_t m_packetType[SX1262_ID_NUMBER];
};

#endif /* SX126X_H */