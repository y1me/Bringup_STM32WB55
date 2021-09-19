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

int ads101x_init( ads101x_params_t *,  ads101x_data_t *);
int ads101x_rotate_mux_gain(ads101x_params_t *, ads101x_data_t *);
int ads101x_read_raw( ads101x_params_t *, ads101x_data_t *);


void ADS115_StateMachine_Iteration(ads101x_params_t *, ads101x_data_t *);
/* USER CODE END Private Prototypes */

typedef struct {
    const char * name;
    int (* const func)(ads101x_params_t *, ads101x_data_t *);
} stateFunctionRow_t;

static stateFunctionRow_t ADS1115_stateFunction[] = {
        // NAME         // FUNC
	{ "ST_ADS1115_INIT",	ads101x_init },
    { "ST_ADS1115_MUX",	ads101x_rotate_mux_gain},
    { "ST_ADS1115_CONV",	ads101x_read_raw },
    { "ST_ADS1115_ERROR",	ads101x_init }
};

typedef struct {
	state_ads1115_t currState;
    event_ads1115_t event;
    state_ads1115_t nextState;
} stateTransMatrixRow_t;

static stateTransMatrixRow_t ADS1115_stateTransMatrix[] = {
    // CURR STATE  v// EVENT           // NEXT STATE
    { ST_ADS1115_INIT,	EV_ADS1115_INIT_DONE,		ST_ADS1115_CONV  },
    { ST_ADS1115_CONV,	EV_ADS1115_1ST_CONV_DONE,	ST_ADS1115_MUX },//ST_ADS1115_CONV  },
    //{ ST_ADS1115_CONV,	EV_ADS1115_2ND_CONV_DONE,	ST_ADS1115_MUX },
	{ ST_ADS1115_MUX,	EV_ADS1115_MUX_DONE,		ST_ADS1115_CONV  },
    { ST_ADS1115_CONV,  EV_ADS1115_DO_INIT,			ST_ADS1115_INIT  },
    { ST_ADS1115_INIT,  EV_ADS1115_DO_INIT,			ST_ADS1115_INIT  }
};

/* Buffer used for transmission */
uint8_t aTxBuffer[ADS101X_BUFFER_SIZE];

/* Buffer used for reception */
uint8_t aRxBuffer[ADS101X_BUFFER_SIZE];

ads101x_data_t config_data;

int ads101x_init(ads101x_params_t *params, ads101x_data_t *data)
{
	int ret;
	uint8_t data_init[3] = ADS101X_INIT_PARAMS;
	if (I2C_status() != I2C_FREE)
	{
		ret = ADS101X_I2CBUSY;
		goto out;
	}
	data->pointer = data_init[0];
	data->config[0] = data_init[1];
	data->config[1] = data_init[2];
	params->mux_gain = data->config[0];
	params->mux_gain &= (ADS101X_MUX_MASK | ADS101X_PGA_MASK);

	ret = write_read_I2C_device_DMA(params->i2cHandle, params->addr, data->config, data->config, 3, 2);
	if (ret == I2C_OK)
	{
		params->currState=ST_ADS1115_INIT;
		params->event=EV_ADS1115_NONE;
	}
	else
	{
		params->currState=ST_ADS1115_CONV;
		params->event=EV_ADS1115_DO_INIT;
	}
	out :
	return ret;

}

int ads101x_rotate_mux_gain(ads101x_params_t *params, ads101x_data_t *data)
{
	data->pointer = ADS101X_CONF_ADDR;
	uint8_t gain_mux_save;
	uint8_t data_to_send[3] = {data->pointer,
						data->config[0],
						data->config[1]
						};
	gain_mux_save = params->mux_gain;
	int ret;
	if (I2C_status() != I2C_FREE)
	{
		ret = ADS101X_I2CBUSY;
		goto out;
	}

	if ((params->mux_gain & ADS101X_MUX_MASK) == ADS101X_AIN0_SINGM)
	{
		params->mux_gain &= ~(ADS101X_MUX_MASK);
		params->mux_gain |= ADS101X_AIN1_SINGM;
	}

	if ((params->mux_gain & ADS101X_MUX_MASK) == ADS101X_AIN1_SINGM)
	{
		params->mux_gain &= ~(ADS101X_MUX_MASK);
		params->mux_gain |= ADS101X_AIN2_SINGM;
	}


	if ((params->mux_gain & ADS101X_MUX_MASK) == ADS101X_AIN3_SINGM)
	{
		//event to init
		//avoid i2c sending?
		ret = ADS101X_OK;

		params->event = EV_ADS1115_DO_INIT;
		goto out;
	}
	if ((params->mux_gain & ADS101X_MUX_MASK) == ADS101X_AIN2_SINGM)
	{
		params->mux_gain &= ~(ADS101X_MUX_MASK);
		params->mux_gain |= ADS101X_AIN3_SINGM;
	}

	data->config[0] &= ~(ADS101X_MUX_MASK);
	data->config[0] |= params->mux_gain;
	data_to_send[1] = data->config[0];
	ret = write_I2C_device_DMA(params->i2cHandle, params->addr, data_to_send, 3);
	if (ret == I2C_OK)
	{
		params->currState=ST_ADS1115_MUX;
		params->event=EV_ADS1115_NONE;
	}
	else
	{
		params->mux_gain = gain_mux_save;
		params->currState=ST_ADS1115_CONV;
		params->event=EV_ADS1115_2ND_CONV_DONE;
	}
	out :
	return ret;

}
void conv_ready( void)
{
	ads101x_read_raw(ads101x_params,&config_data);
}

int ads101x_read_raw( ads101x_params_t *params, ads101x_data_t *data)
{
	int ret;
	//uint8_t gain_mux_save;
	uint8_t *dataRx, address_pointer = ADS101X_CONV_RES_ADDR;
	if (I2C_status() != I2C_FREE)
	{
		ret = ADS101X_I2CBUSY;
		goto out;
	}
	//gain_mux_save = params->mux_gain;
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
	ret = write_read_I2C_device_DMA(params->i2cHandle, params->addr, &address_pointer, dataRx, 1, 2);
	if (ret == I2C_OK)
	{
		params->currState=ST_ADS1115_CONV;
		params->event=EV_ADS1115_NONE;
	}
	else
	{
		//params->mux_gain = gain_mux_save;
		params->currState=ST_ADS1115_MUX;
		params->event=EV_ADS1115_MUX_DONE;
	}
	out :
	return ret;
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

void Running_ADS115_StateMachine_Iteration(void)
{
	ADS115_StateMachine_Iteration(ads101x_params,&config_data);
}

void ADS115_StateMachine_Iteration(ads101x_params_t *params, ads101x_data_t *data)
{
	if (I2C_status() == I2C_OK && params->event == EV_ADS1115_NONE)
	{
		I2C_clear_last_event();
		if(params->currState == ST_ADS1115_INIT)
		{
			params->event = EV_ADS1115_INIT_DONE;
		}
		if(params->currState == ST_ADS1115_MUX)
		{
			params->event = EV_ADS1115_MUX_DONE;
		}
		if(params->currState == ST_ADS1115_CONV)
		{
			params->event = EV_ADS1115_1ST_CONV_DONE;
		}

	}

    // Iterate through the state transition matrix, checking if there is both a match with the current state
    // and the event
    for(int i = 0; i < sizeof(ADS1115_stateTransMatrix)/sizeof(ADS1115_stateTransMatrix[0]); i++) {
        if(ADS1115_stateTransMatrix[i].currState == params->currState) {
            if(ADS1115_stateTransMatrix[i].event == params->event) {

                // Transition to the next state
            	params->currState =  ADS1115_stateTransMatrix[i].nextState;

                // Call the function associated with transition
            	if ( (ADS1115_stateFunction[params->currState].func) != NULL )
            	{
            		(ADS1115_stateFunction[params->currState].func)(params,data);
            	}
            	break;
            }
        }
    }
}
