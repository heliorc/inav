/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"

#include "sensors/gyro.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_imuf9001.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/bus_spi.h"
#include "drivers/bus_busdev_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
#include "fc/config.h"
#include "fc/runtime_config.h"
#include "sensors/boardalignment.h"

#include "drivers/system.h"


volatile uint16_t imufCurrentVersion = IMUF_FIRMWARE_VERSION;
volatile uint32_t isImufCalibrating = 0;
volatile imuFrame_t imufQuat;

static inline void gpio_write_pin(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, uint32_t pinState)
{
    if (pinState != 0)
    {
        GPIOx->BSRRL = (uint32_t)GPIO_Pin;
    }
    else
    {
        GPIOx->BSRRH = (uint32_t)GPIO_Pin;
    }
}

static inline void dmaSpiCsLo(void)
{
    gpio_write_pin(GPIOA, GPIO_Pin_4, 0);
}

static inline void dmaSpiCsHi(void)
{
    gpio_write_pin(GPIOA, GPIO_Pin_4, 1);
}

void crcConfig(void)
{
    return;
}

inline uint32_t getCrcImuf9001(uint32_t* data, uint32_t size)
{
    CRC_ResetDR(); //reset data register
    for(uint32_t x=0; x<size; x++ )
    {
        CRC_CalcCRC(data[x]);
    }
    return CRC_GetCRC();
}

inline void appendCrcToData(uint32_t* data, uint32_t size)
{
    data[size] = getCrcImuf9001(data, size);;
}

static void resetImuf9001(void)
{
    //reset IMU
    dmaSpiCsLo();
    delay(500);
    dmaSpiCsHi();

}

bool imufSendReceiveSpiBlocking(const busDevice_t *bus, uint8_t *dataTx, uint8_t *daRx, uint8_t length)
{
    spiBusTransfer(bus, dataTx, daRx, length);
    return true;
}

static int imuf9001SendReceiveCommand(const gyroDev_t *gyro, gyroCommands_t commandToSend, imufCommand_t *reply, imufCommand_t *data)
{

    imufCommand_t command;
    uint32_t attempt, crcCalc;
    int failCount = 500;    

    memset(reply, 0, sizeof(command));

    if (data)
    {
        memcpy(&command, data, sizeof(command));
    }
    else
    {
        memset(&command, 0, sizeof(command));
    }

    command.command = commandToSend;
    command.crc     = getCrcImuf9001((uint32_t *)&command, 11);;


    while (failCount-- > 0)
    {
        delayMicroseconds(1000);

        if( GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) ) //IMU is ready to talk
        {
            failCount -= 100;
            imufSendReceiveSpiBlocking(gyro->busDev, (uint8_t *)&command, (uint8_t *)reply, sizeof(imufCommand_t));

            crcCalc = getCrcImuf9001((uint32_t *)reply, 11);
            //this is the only valid reply we'll get if we're in BL mode
            if(crcCalc == reply->crc && reply->command == IMUF_COMMAND_LISTENING ) //this tells us the IMU was listening for a command, else we need to reset synbc
            {
                for (attempt = 0; attempt < 100; attempt++)
                {
                    //reset command, just waiting for reply data now
                    command.command = IMUF_COMMAND_NONE;
                    command.crc     = getCrcImuf9001((uint32_t *)&command, 11);

                    delayMicroseconds(100); //give pin time to set

                    if( IORead(IOGetByTag(IO_TAG(MPU_INT_EXTI))) ) //IMU is ready to talk
                    {
                        //reset attempts
                        attempt = 100;

                        imufSendReceiveSpiBlocking(gyro->busDev, (uint8_t *)&command, (uint8_t *)reply, sizeof(imufCommand_t));
                        crcCalc = getCrcImuf9001((uint32_t *)reply, 11);

                        if(crcCalc == reply->crc && reply->command == commandToSend ) //this tells us the IMU understood the last command
                        {
                            return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

static int updateImuf(void)
{
    return 0;
}

int imuf9001Whoami(const gyroDev_t *gyro)
{
    uint32_t attempt;
    imufCommand_t reply;

    for (attempt = 0; attempt < 10; attempt++)
    {
        if (imuf9001SendReceiveCommand(gyro, IMUF_COMMAND_REPORT_INFO, &reply, NULL))
        {
            imufCurrentVersion = (*(imufVersion_t *)&(reply.param1)).firmware;
            if (imufCurrentVersion < IMUF_FIRMWARE_VERSION) {
                //force update
                if (!updateImuf())
                {
                    return 0;
                }
            } else {
                return IMUF_FIRMWARE_VERSION;
            }
        }
    }
    return (0);
}

uint8_t imuf9001SpiDetect(gyroDev_t *gyro)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return(0);
    }

    #ifdef USE_GYRO_IMUF9001
    #ifdef IMUF9001_SPI_INSTANCE
        gyro->busDev = busDeviceInit(BUSTYPE_SPI, DEVHW_IMUF9001, gyro->imuSensorToUse, OWNER_MPU);
    #else
        #error IMUF9001 is SPI only
    #endif
    //#ifdef IMUF9001_CS_PIN
    //    gyro->busDev->busdev.spi.csnPin = gyro->busDev->busdev.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(IMUF9001_CS_PIN)) : gyro->busDev->busdev.spi.csnPin;
    //#else
    //   #error IMUF9001 must use a CS pin (IMUF9001_CS_PIN)
    //#endif
    //#ifdef IMUF9001_RST_PIN
    //    gyro->busDev->busdev.spi.rstPin = IOGetByTag(IO_TAG(IMUF9001_RST_PIN));
    //#else
    //    #error IMUF9001 must use a RST pin (IMUF9001_RST_PIN)
    //#endif
    //sensor = imuf9001SpiDetect(gyro);
    //// some targets using MPU_9250_SPI, ICM_20608_SPI or ICM_20602_SPI state sensor is MPU_65xx_SPI
    //if (sensor != GYRO_NONE) {
    //    return true;
    //}
    #endif

    crcConfig();
    //config exti as input, not exti for now
    //IOInit(IOGetByTag( IO_TAG(MPU_INT_EXTI) ), OWNER_MPU, RESOURCE_EXTI, 0);
    //IOConfigGPIO(IOGetByTag( IO_TAG(MPU_INT_EXTI) ), IOCFG_IPD);

    GPIO_InitTypeDef gpioInitStruct;
    //config pins
    gpioInitStruct.GPIO_Pin   = GPIO_Pin_0;
    gpioInitStruct.GPIO_Mode  = GPIO_Mode_IN;
    gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInitStruct.GPIO_OType = GPIO_OType_PP;
    gpioInitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &gpioInitStruct);
    //config pins
    gpioInitStruct.GPIO_Pin   = GPIO_Pin_4;
    gpioInitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInitStruct.GPIO_OType = GPIO_OType_PP;
    gpioInitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpioInitStruct);
    dmaSpiCsHi();

    delayMicroseconds(100);

    //IOInit(gyro->busDev->busdev.spi.csnPin, OWNER_MPU, RESOURCE_SPI_CS, 0);
    //IOConfigGPIO(gyro->busDev->busdev.spi.csnPin, SPI_IO_CS_CFG);
    //IOHi(gyro->busDev->busdev.spi.csnPin);

    //IOInit( IOGetByTag(IO_TAG(IMUF9001_RST_PIN)), OWNER_MPU, RESOURCE_SPI_CS, 0);
    //IOConfigGPIO( IOGetByTag(IO_TAG(IMUF9001_RST_PIN)), SPI_IO_CS_CFG);
    //IOHi( IOGetByTag(IO_TAG(IMUF9001_RST_PIN)));

    hardwareInitialised = true;

    for (int x=0; x<10; x++)
    {
        int returnCheck;
        if (x)
        {
            resetImuf9001();
            delay(300 * x);
        }
        returnCheck = imuf9001Whoami(gyro);
        if(returnCheck)
        {
            return returnCheck;
        }
    }
    return 0;
}

void imufSpiAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

static gyroToBoardCommMode_t VerifyAllowedCommMode(uint32_t commMode)
{
    switch (commMode)
    {
        case GTBCM_SETUP:
        case GTBCM_GYRO_ONLY_PASSTHRU:
        case GTBCM_GYRO_ACC_PASSTHRU:
        case GTBCM_GYRO_ONLY_FILTER_F:
        case GTBCM_GYRO_ACC_FILTER_F:
        case GTBCM_GYRO_ACC_QUAT_FILTER_F:
            return (gyroToBoardCommMode_t)commMode;
            break;
        default:
            return GTBCM_DEFAULT;
    }
}

void imufSpiGyroInit(gyroDev_t *gyro)
{
    uint32_t attempt = 0;
    imufCommand_t txData;
    imufCommand_t rxData;

    rxData.param1 = VerifyAllowedCommMode(gyroConfig()->imuf_mode);
    rxData.param2 = ( (uint16_t)(gyroConfig()->imuf_rate+1) << 16)            | (uint16_t)gyroConfig()->imuf_w;
    rxData.param3 = ( (uint16_t)gyroConfig()->imuf_roll_q << 16)              | (uint16_t)gyroConfig()->imuf_pitch_q;
    rxData.param4 = ( (uint16_t)gyroConfig()->imuf_yaw_q << 16)               | (uint16_t)gyroConfig()->imuf_roll_lpf_cutoff_hz;
    rxData.param5 = ( (uint16_t)gyroConfig()->imuf_pitch_lpf_cutoff_hz << 16) | (uint16_t)gyroConfig()->imuf_yaw_lpf_cutoff_hz;
    rxData.param6 = ( (uint16_t)0 << 16)                                      | (uint16_t)0;
    rxData.param7 = ( (uint16_t)0 << 16)                                      | (uint16_t)0;
    rxData.param8 = ( (int16_t)boardAlignment()->rollDeciDegrees << 16 )      | returnGyroAlignmentForImuf9001();
    rxData.param9 = ( (int16_t)boardAlignment()->yawDeciDegrees << 16 )       | (int16_t)boardAlignment()->pitchDeciDegrees;

    for (attempt = 0; attempt < 10; attempt++)
    {
        if(attempt)
        {
            resetImuf9001();
            delay(300 * attempt);
        }

        if (imuf9001SendReceiveCommand(gyro, IMUF_COMMAND_SETUP, &txData, &rxData))
        {
            //enable EXTI
            gyroIntExtiInit(gyro);
            return;
        }
    }
    ENABLE_ARMING_FLAG(ARMING_DISABLED_HARDWARE_FAILURE);
}

bool imufReadAccData(accDev_t *acc) {
    UNUSED(acc);
    return true;
}

bool imufSpiAccDetect(accDev_t *acc)
{
    acc->initFn = imufSpiAccInit;
    acc->readFn = imufReadAccData;

    return true;
}

bool imufSpiGyroDetect(gyroDev_t *gyro)
{
    // MPU6500 is used as a equivalent of other gyros by some flight controllers
    gyro->initFn = imufSpiGyroInit;
    gyro->readFn = NULL;
    gyro->scale = 1.0f;
    imuf9001SpiDetect(gyro);
    return true;
}

void imufStartCalibration(void)
{
    isImufCalibrating = IMUF_IS_CALIBRATING; //reset by EXTI
}

void imufEndCalibration(void)
{
    isImufCalibrating = IMUF_NOT_CALIBRATING; //reset by EXTI
}