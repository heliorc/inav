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

#include "build/debug.h"

#include "sensors/gyro.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_imuf9001.h"
#include "drivers/light_led.h"

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

#include "target/imuf/imufbin.c" //imuf bin definition for updating

volatile uint16_t imufCurrentVersion = 0;
volatile uint32_t isImufCalibrating = 0;
volatile imuFrame_t imufQuat;

static void resetBlink()
{
    for(uint32_t x = 0; x<40; x++)
    {
        LED0_TOGGLE;
        delay(20);
    }
    LED0_OFF;
}

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

static inline void imufRstLo(void)
{
    gpio_write_pin(IMUF_RST_PORT, IMUF_RST_PIN, 0);
}

static inline void imufRstHi(void)
{
    gpio_write_pin(IMUF_RST_PORT, IMUF_RST_PIN, 1);
}

static void imufEXTIHi(void)
{
    gpio_write_pin(IMUF_EXTI_PORT, IMUF_EXTI_PIN, 1);
}

static void imufEXTILo(void)
{
    gpio_write_pin(IMUF_EXTI_PORT, IMUF_EXTI_PIN, 0);
}

void configExtiAsInput(void)
{
    //config exti as input, not exti for now
    GPIO_InitTypeDef gpioInitStruct;
    //config pins
    gpioInitStruct.GPIO_Pin   = IMUF_EXTI_PIN;
    gpioInitStruct.GPIO_Mode  = GPIO_Mode_IN;
    gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInitStruct.GPIO_OType = GPIO_OType_PP;
    gpioInitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(IMUF_EXTI_PORT, &gpioInitStruct);
    //config pins

    delayMicroseconds(100);
}

void configExtiAsOutput(void)
{
    //config exti as input, not exti for now
    GPIO_InitTypeDef gpioInitStruct;
    //config pins
    gpioInitStruct.GPIO_Pin   = IMUF_EXTI_PIN;
    gpioInitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInitStruct.GPIO_OType = GPIO_OType_PP;
    gpioInitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(IMUF_EXTI_PORT, &gpioInitStruct);
    //config pins

    delayMicroseconds(100);
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

void resetImuf9001(void)
{
    static int runOnce = 1;

    if(runOnce)
    {
        runOnce = 0;
        GPIO_InitTypeDef gpioInitStruct;
        gpioInitStruct.GPIO_Pin   = IMUF_RST_PIN;
        gpioInitStruct.GPIO_Mode  = GPIO_Mode_OUT;
        gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        gpioInitStruct.GPIO_OType = GPIO_OType_OD;
        gpioInitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
        GPIO_Init(IMUF_RST_PORT, &gpioInitStruct);
    }

    imufRstLo();
    resetBlink();
    imufRstHi();
    delay(100);

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

    if(commandToSend == BL_WRITE_FIRMWARES)
    {
        command.param1  = data->param1;
        command.param2  = data->param2;
        command.param3  = data->param3;
        command.param4  = data->param4;
        command.param5  = data->param5;
        command.param6  = data->param6;
        command.param7  = data->param7;
        command.param8  = data->param8;
        command.param9  = data->param9;
        command.param10 = data->param10;
    }
    
    command.command = commandToSend;
    command.crc     = getCrcImuf9001((uint32_t *)&command, 11);;


    while (failCount-- > 0)
    {
        delayMicroseconds(1000);

        if( GPIO_ReadInputDataBit(IMUF_EXTI_PORT, IMUF_EXTI_PIN) ) //IMU is ready to talk
        {
            failCount -= 100;
            imufSendReceiveSpiBlocking(gyro->busDev, (uint8_t *)&command, (uint8_t *)reply, sizeof(imufCommand_t));

            crcCalc = getCrcImuf9001((uint32_t *)reply, 11);
            //this is the only valid reply we'll get if we're in BL mode
            if(crcCalc == reply->crc && (reply->command == IMUF_COMMAND_LISTENING || reply->command == BL_LISTENING)) //this tells us the IMU was listening for a command, else we need to reset synbc
            {
                for (attempt = 0; attempt < 100; attempt++)
                {
                    //reset command, just waiting for reply data now
                    command.command = IMUF_COMMAND_NONE;
                    command.crc     = getCrcImuf9001((uint32_t *)&command, 11);

                    if(commandToSend == BL_ERASE_ALL)
                    {
                        delay(500);
                    }
                    delayMicroseconds(100); //give pin time to set

                    if( GPIO_ReadInputDataBit(IMUF_EXTI_PORT, IMUF_EXTI_PIN) ) //IMU is ready to talk
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

static int UpgradeBootloader(const gyroDev_t *gyro, uint32_t sizeOfBin)
{
    imufCommand_t reply;
    imufCommand_t data;
    #define START_LOCATION 0x08002000

    volatile uint32_t startLocation = (uint32_t)((uint32_t*)(imuf_bin));
    memset(&data, 0, sizeof(data));
    //erase firmware on MCU
    if( imuf9001SendReceiveCommand(gyro, BL_ERASE_ALL, &reply, &data) )
    {
        //good data
        if( imuf9001SendReceiveCommand(gyro, BL_PREPARE_PROGRAM, &reply, &data) )
        {
            for(uint32_t x=0;x<(sizeOfBin);x+=32)
            {
                data.param1 = START_LOCATION+x;

                data.param2 = (*(__IO uint32_t *)(startLocation+x));
                data.param3 = (*(__IO uint32_t *)(startLocation+x+4));
                data.param4 = (*(__IO uint32_t *)(startLocation+x+8));
                data.param5 = (*(__IO uint32_t *)(startLocation+x+12));
                data.param6 = (*(__IO uint32_t *)(startLocation+x+16));
                data.param7 = (*(__IO uint32_t *)(startLocation+x+20));
                data.param8 = (*(__IO uint32_t *)(startLocation+x+24));
                data.param9 = (*(__IO uint32_t *)(startLocation+x+28));

                if( imuf9001SendReceiveCommand(gyro, BL_WRITE_FIRMWARES, &reply, &data) )
                {
                    //continue writing
                    LED0_TOGGLE;
                }
                else
                {
                    //error handler
                    for(uint32_t x = 0; x<40; x++)
                    {
                        LED0_TOGGLE;
                        delay(200);
                    }
                    LED0_OFF;
                    return 0;
                }
            }

            if( imuf9001SendReceiveCommand(gyro, BL_END_PROGRAM, &reply, &data) )
            {
                return 1;
            }

        }
    }

    return 0;

}

int updateImuf(const gyroDev_t *gyro)
{

    imufCommand_t reply;
    imufCommand_t data;

    //blink
    resetBlink();

    //config BL pin as output (shared with EXTI, this happens before EXTI init though)
    configExtiAsOutput();

    imufEXTIHi();    //set bl pin Hi
    resetImuf9001(); //reset imuf, make three blinks
    resetImuf9001(); //reset imuf, make three blinks
    resetImuf9001(); //reset imuf, make three blinks
    delay(100);      //delay 100 ms. give IMUF BL time to look for bl init pin
    imufEXTILo();    //set bl pin Lo

    //config EXTI as input so we can check imuf status
    configExtiAsInput();

    //check if BL is active
    memset(&data, 0, sizeof(data));

    if (imuf9001SendReceiveCommand(gyro, BL_REPORT_INFO, &reply, &data))
    {
        //good data
        if (UpgradeBootloader(gyro, imuf_bin_len))
        {
            //reboot
            //systemReset();
            return 1;
        }
    }
    return 0;
}

int imuf9001Whoami(const gyroDev_t *gyro)
{
    uint32_t attempt;
    uint32_t up_attempt = 0;
    imufCommand_t reply;

    for (attempt = 0; attempt < 5; attempt++)
    {
        if (imuf9001SendReceiveCommand(gyro, IMUF_COMMAND_REPORT_INFO, &reply, NULL))
        {
            imufCurrentVersion = (*(imufVersion_t *)&(reply.param1)).firmware;
            if (imufCurrentVersion < imuf_bin_ver) {
                //force update
                while(1)
                {
                    if (updateImuf(gyro))
                    {
                        return imuf_bin_ver;
                    }

                    if(up_attempt++ > 10)
                    {
                        return 0;
                    }
                }
            } else {
                return imuf_bin_ver;
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
    #endif

    crcConfig();
    configExtiAsInput();

    hardwareInitialised = true;

    for (int x=0; x<6; x++)
    {
        int returnCheck;
        if (x>3)
        {
            resetImuf9001();
            resetImuf9001();
        }
        if(x > 4)
        {
            updateImuf(gyro);
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
    //acc->acc_1G = 512 * 4;
    acc->acc_1G = 512 * 8;
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
    rxData.param2 = ( (uint16_t)(gyroConfig()->imuf_rate+1) << 16 );
    rxData.param3 = ( (uint16_t)gyroConfig()->imuf_pitch_q << 16 ) | (uint16_t)gyroConfig()->imuf_pitch_w;
    rxData.param4 = ( (uint16_t)gyroConfig()->imuf_roll_q << 16 ) | (uint16_t)gyroConfig()->imuf_roll_w;
    rxData.param5 = ( (uint16_t)gyroConfig()->imuf_yaw_q << 16 ) | (uint16_t)gyroConfig()->imuf_yaw_w;
    rxData.param6 = ( (uint16_t)gyroConfig()->imuf_pitch_lpf_cutoff_hz << 16) | (uint16_t)gyroConfig()->imuf_roll_lpf_cutoff_hz;
    rxData.param7 = ( (uint16_t)gyroConfig()->imuf_yaw_lpf_cutoff_hz << 16) | (uint16_t)0;
    rxData.param8 = ( (int16_t)boardAlignment()->rollDeciDegrees << 16 ) | returnGyroAlignmentForImuf9001();
    rxData.param9 = ( (int16_t)boardAlignment()->yawDeciDegrees << 16 ) | (int16_t)boardAlignment()->pitchDeciDegrees;

    for (attempt = 0; attempt < 10; attempt++)
    {
        //if(attempt)
        //{
        //    resetImuf9001();
        //    delay(300 * attempt);
        //}

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
    gyro->scale  = 1.0f;
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