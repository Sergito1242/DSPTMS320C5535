#include "stdio.h"
#include "ezdsp5535.h"
#ifndef I2C_H_
#define I2C_H_


void I2C_masterInit();
void I2C_setSlaveAddress(Int16 address);
void I2C_masterSendStart();
void I2C_masterSendSingleByte(Int16 byte);
void I2C_masterSendNext(Int16 byte);
void I2C_masterMultiByteSendStop();
Int16 I2C_isBusy();
void I2C_clearInterruptFlag();

#endif
