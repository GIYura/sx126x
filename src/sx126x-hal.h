#ifndef SX126X_HAL_H
#define SX126X_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef void(DioIrqHandler)(void);

void SX126xHal_GpioInit(void);
void SX126xHal_Reset(void);
void SX126xHal_Wakeup(void);
void SX126xHal_WriteCommand(const uint8_t* const cmd, uint16_t cmdLen, const uint8_t* const buffer, uint16_t size);
void SX126xHal_ReadCommand(const uint8_t* const cmd, uint16_t cmdLen, uint8_t* const buffer, uint16_t size);
void SX126xHal_WriteRegister(uint8_t addr, uint8_t value);
void SX126xHal_ReadRegister(uint8_t addr, uint8_t* const value);
void SX126xHal_RegisterDioIrq(DioIrqHandler dioIrq);

#endif /* SX126X_HAL_H */
