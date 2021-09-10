/*
 * ads1115.c
 *
 *  Created on: Aug 1, 2021
 *      Author: blobby
 */

/*
 * Copyright (C) 2017 OTA keys S.A.
 *               2018 Acutam Automation, LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ads101x
 * @{
 *
 * @file
 * @brief       ADS101x/111x ADC device driver
 *
 * @author      Vincent Dupont <vincent@otakeys.com>
 * @author      Matthew Blue <matthew.blue.neuro@gmail.com>
 * @}
 */

#include "assert.h"


#include "periph/ads101x.h"
#include "periph/ads101x_params.h"
#include "periph/ads101x_regs.h"

#include "i2c.h"
#include "Utils/Commons.h"


//#ifndef ADS101X_READ_DELAY
//#define ADS101X_READ_DELAY (8 * US_PER_MS)    /* Compatible with 128SPS */
//#endif

//#define DEV (dev->params.i2c)
//#define ADDR (dev->params.addr)

static int _ads101x_init(const ads101x_params_t *params);
/* Buffer used for transmission */
uint8_t aTxBuffer[ADS101X_BUFFER_SIZE];

/* Buffer used for reception */
uint8_t aRxBuffer[ADS101X_BUFFER_SIZE];

ads101x_data_t data_input;

int ads101x_init(ads101x_params_t *params, uint8_t *dataRx)
{
	uint8_t Txregs[3] = {ADS101X_CONF_ADDR,
			ADS101X_AIN0_SINGM
			| ADS101X_PGA_FSR_4V096
			| ADS101X_MODE_CON,
			ADS101X_DATAR_128
			| ADS101X_CONF_COMP_DIS
			};
	params->mux_gain = Txregs[1];
	return write_read_I2C_device_DMA(params->i2cHandle, params->addr, Txregs, dataRx, 3, 2);
}

int ads101x_alert_init(ads101x_alert_t *dev,
                       const ads101x_alert_params_t *params)
{
    assert(dev && params);

    dev->params = *params;
    dev->cb = NULL;
    dev->arg = NULL;

    /* Set up alerts */
    ads101x_set_alert_parameters(dev, dev->params.low_limit,
                                 dev->params.high_limit);

    //return _ads101x_init_test(DEV, ADDR);
}

int ads101x_rotate_mux_gain(ads101x_params_t *params, uint8_t *dataRx)
{
	uint8_t Txregs[3] = {ADS101X_CONF_ADDR,
			ADS101X_MUX_MASK
			| ADS101X_PGA_MASK
			| ADS101X_MODE_CON,
			ADS101X_DATAR_128
			| ADS101X_CONF_COMP_DIS
			};

	if ((params->mux_gain & ADS101X_MUX_MASK) == ADS101X_AIN0_SINGM)
	{
		params->mux_gain |= ADS101X_MUX_MASK;
		params->mux_gain &= ADS101X_AIN1_SINGM;
	}

	if ((params->mux_gain & ADS101X_MUX_MASK) == ADS101X_AIN1_SINGM)
		{
			params->mux_gain |= ADS101X_MUX_MASK;
			params->mux_gain &= ADS101X_AIN2_SINGM;
		}

	if ((params->mux_gain & ADS101X_MUX_MASK) == ADS101X_AIN2_SINGM)
			{
				params->mux_gain |= ADS101X_MUX_MASK;
				params->mux_gain &= ADS101X_AIN3_SINGM;
			}
	if ((params->mux_gain & ADS101X_MUX_MASK) == ADS101X_AIN3_SINGM)
			{
//event to init
//avoid i2c sending?
			}

	return write_read_I2C_device_DMA(params->i2cHandle, params->addr, Txregs, dataRx, 3, 2);

}

int ads101x_read_raw(const ads101x_params_t *params, ads101x_data_t *data)
{
	uint8_t *dataRx;
	if ((params->mux_gain & ADS101X_MUX_MASK) == ADS101X_AIN0_SINGM)
	{
		dataRx = data->ain0;
	}
	if ((params->mux_gain & ADS101X_MUX_MASK) == ADS101X_AIN1_SINGM)
	{
		dataRx = data->ain1;
	}
	if ((params->mux_gain & ADS101X_MUX_MASK) == ADS101X_AIN2_SINGM)
	{
		dataRx = data->ain2;
	}
	if ((params->mux_gain & ADS101X_MUX_MASK) == ADS101X_AIN3_SINGM)
	{
		dataRx = data->ain3;
	}
	return read_I2C_device_DMA(params->i2cHandle, params->addr, dataRx, 2);
}

int ads101x_enable_alert(ads101x_alert_t *dev,
                         ads101x_alert_cb_t cb, void *arg)
{
	/*
    uint8_t regs[2];

    if (!gpio_is_valid(dev->params.alert_pin)) {
        return ADS101X_OK;
    }

    /* Read control register */
	/*
    i2c_acquire(DEV);
    i2c_read_regs(DEV, ADDR, ADS101X_CONF_ADDR, &regs, 2, 0x0);

    /* Enable alert comparator */
	/*
    regs[1] &= ~ADS101X_CONF_COMP_DIS;
    i2c_write_regs(DEV, ADDR, ADS101X_CONF_ADDR, &regs, 2, 0x0);

    i2c_release(DEV);

    /* Enable interrupt */
	/*
    dev->arg = arg;
    dev->cb = cb;
    gpio_init_int(dev->params.alert_pin, GPIO_IN, GPIO_FALLING, cb, arg);
*/
    return ADS101X_OK;
}

int ads101x_set_alert_parameters(const ads101x_alert_t *dev,
                                 int16_t low_limit, int16_t high_limit)
{
	/*
    uint8_t regs[2];

    i2c_acquire(DEV);

    /* Set up low_limit */
	/*
    regs[0] = (uint8_t)(low_limit >> 8);
    regs[1] = (uint8_t)low_limit;
    i2c_write_regs(DEV, ADDR, ADS101X_LOW_LIMIT_ADDR, &regs, 2, 0x0);

    /* Set up high_limit */
	/*
    regs[0] = (uint8_t)(high_limit >> 8);
    regs[1] = (uint8_t)high_limit;
    i2c_write_regs(DEV, ADDR, ADS101X_HIGH_LIMIT_ADDR, &regs, 2, 0x0);

    /* Read control register */
	/*
    i2c_read_regs(DEV, ADDR, ADS101X_CONF_ADDR, &regs, 2, 0x0);

    /* Set up window mode */
	/*
    if (low_limit != 0) {
        /* Enable window mode */
	/*
        regs[1] |= ADS101X_CONF_COMP_MODE_WIND;
    }
    else {
        /* Disable window mode */
	/*
        regs[1] &= ~ADS101X_CONF_COMP_MODE_WIND;
    }
    i2c_write_regs(DEV, ADDR, ADS101X_CONF_ADDR, &regs, 2, 0x0);

    i2c_release(DEV);
*/
    return ADS101X_OK;
}
