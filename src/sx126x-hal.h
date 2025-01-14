#ifndef SX126X_HAL_H
#define SX126X_HAL_H

typedef void(DioIrqHandler)(void);

class Hal
{
    public:
        Hal(){};

        void GpioInit(void);

        void Reset(void);

        void Wakeup(void);

        void WriteCommand(const uint8_t* const cmd, uint16_t cmdLen, const uint8_t* const buffer, uint16_t size);
        void ReadCommand(const uint8_t* const cmd, uint16_t cmdLen, uint8_t* const buffer, uint16_t size);
        
        void WriteRegister(uint8_t addr, uint8_t value);
        void ReadRegister(uint8_t addr, uint8_t* const value);
    
        void RegisterDioIrq(DioIrqHandler dioIrq);

    private:
        void WaitOnBusy(void);
};


#endif /* SX126X_HAL_H */
