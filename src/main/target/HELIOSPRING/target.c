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

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
//#include "drivers/dma.h"

//BUSDEV_REGISTER_SPI_TAG(busdev_mpu6000,     DEVHW_IMUF9001,     IMUF9001_SPI_INSTANCE,    IMUF9001_CS_PIN,     MPU_INT_EXTI,       0,  DEVFLAGS_NONE);

//BUSDEV_REGISTER_I2C(    busdev_ms5611,      DEVHW_MS5611,       BARO_I2C_BUS,       0x77,               NONE,                       DEVFLAGS_USE_RAW_REGISTERS);

//BUSDEV_REGISTER_I2C_TAG(busdev_hmc5883_int, DEVHW_HMC5883,      MAG_I2C_BUS_INT,    0x1E,               NONE,                   0,  DEVFLAGS_NONE);

//BUSDEV_REGISTER_I2C_TAG(busdev_hmc5883,     DEVHW_HMC5883,      MAG_I2C_BUS_EXT,    0x1E,               NONE,                   1,  DEVFLAGS_NONE);
//BUSDEV_REGISTER_I2C_TAG(busdev_qmc5883,     DEVHW_QMC5883,      MAG_I2C_BUS_EXT,    0x0D,               NONE,                   1,  DEVFLAGS_NONE);
//BUSDEV_REGISTER_I2C_TAG(busdev_mag3110,     DEVHW_MAG3110,      MAG_I2C_BUS_EXT,    0x0E,               NONE,                   1,  DEVFLAGS_NONE);


const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
   
    // Motors/Servos
    { TIM8,  IO_TAG(PC6), TIM_Channel_1, TIMER_OUTPUT_ENABLED, IOCFG_AF_PP_PD, GPIO_AF_TIM8, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR }, // S1_OUT D1_ST7
    { TIM8,  IO_TAG(PC7), TIM_Channel_2, TIMER_OUTPUT_ENABLED, IOCFG_AF_PP_PD, GPIO_AF_TIM8, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR }, // S2_OUT D1_ST2
    { TIM3,  IO_TAG(PC8), TIM_Channel_3, TIMER_OUTPUT_ENABLED, IOCFG_AF_PP_PD, GPIO_AF_TIM3, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR | TIM_USE_FW_SERVO }, // S3_OUT D1_ST6
    { TIM3,  IO_TAG(PC9), TIM_Channel_4, TIMER_OUTPUT_ENABLED, IOCFG_AF_PP_PD, GPIO_AF_TIM3, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR | TIM_USE_FW_SERVO }, // S4_OUT D1_ST1
    { TIM1,  IO_TAG(PA8), TIM_Channel_1, TIMER_OUTPUT_ENABLED, IOCFG_AF_PP_PD, GPIO_AF_TIM1, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR | TIM_USE_FW_SERVO }, // S4_OUT D1_ST1
    // LED strip
    //{ TIM1,  TIM_Channel_1, IO_TAG(PA8),  0,      IOCFG_AF_PP, GPIO_AF_TIM1, TIM_USE_LED               }, // D1_ST0
};
