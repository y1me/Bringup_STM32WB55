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
 * @brief       Default configuration for ADS101x/111x devices
 *
 * @author      Vincent Dupont <vincent@otakeys.com>
 * @author      Matthew Blue <matthew.blue.neuro@gmail.com>
 */

#ifndef ADS101X_PARAMS_H
#define ADS101X_PARAMS_H

#include "ads101x.h"
#include "ads101x_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the ADS101x/111x driver
 * @{
 */

#ifndef ADS101X_PARAM_I2C
#define ADS101X_PARAM_I2C       (ADS101X_PARAM_I2C)
#endif

#ifndef ADS101X_PARAM_ADDR
#define ADS101X_PARAM_ADDR       (CONFIG_ADS101X_I2C_ADDRESS)
#endif
#ifndef ADS101X_PARAM_MUX_GAIN
#define ADS101X_PARAM_MUX_GAIN   (ADS101X_AIN0_SINGM \
                                  | ADS101X_PGA_FSR_4V096)
#endif

#ifndef ADS101X_PARAM_ALERT_STATUS
#define ADS101X_PARAM_ALERT_STATUS       (ADS101X_PARAM_ALERT_FALSE)
#endif

#ifndef ADS101X_PARAM_LOW_LIMIT
#define ADS101X_PARAM_LOW_LIMIT  (10000U)
#endif
#ifndef ADS101X_PARAM_HIGH_LIMIT
#define ADS101X_PARAM_HIGH_LIMIT (20000U)
#endif

#ifndef ADS101X_PARAMS
#define ADS101X_PARAMS          { .i2cHandle        = ADS101X_PARAM_I2C,\
                                  .addr       		= ADS101X_PARAM_ADDR,\
                                  .mux_gain   		= ADS101X_PARAM_MUX_GAIN }
#endif

#ifndef ADS101X_ALERT_PARAMS
#define ADS101X_ALERT_PARAMS    { .i2cHandle        = ADS101X_PARAM_I2C,\
                                  .addr       		= ADS101X_PARAM_ADDR,\
								  .alert_pin_status	= ADS101X_PARAM_ALERT_STATUS,\
                                  .low_limit  		= ADS101X_PARAM_LOW_LIMIT,\
                                  .high_limit 		= ADS101X_PARAM_HIGH_LIMIT }
#endif
//#ifndef ADS101X_SAUL_INFO
//#define ADS101X_SAUL_INFO       { .name = "ads101x" }
//#endif
/** @} */

/**
 * @brief   ADS101X/111x defaults if not defined for a board or application
 */
static const ads101x_params_t ads101x_params[] =
{
    ADS101X_PARAMS
};

/**
 * @brief   ADS101X/111x alert defaults if not defined for a board or application
 */
static const ads101x_alert_params_t ads101x_alert_params[] =
{
    ADS101X_ALERT_PARAMS
};

/**
 * @brief   Additional meta information to keep in the SAUL registry
 */
//static const saul_reg_info_t ads101x_saul_info[] =
//{
//    ADS101X_SAUL_INFO
//};

#ifdef __cplusplus
}
#endif

#endif /* ADS101X_PARAMS_H */
/** @} */
