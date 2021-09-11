/*
 * Copyright (C) 2017 OTA keys S.A.
 *               2018 Acutam Automation, LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup   drivers_ads101x ADS101x/111x ADC device driver
 * @ingroup    drivers_sensors
 * @ingroup    drivers_saul
 * @brief      I2C Analog-to-Digital Converter device driver
 *
 * This driver works with ADS1013-5 and ADS1113-5.
 *
 * This driver provides @ref drivers_saul capabilities.
 * @{
 *
 * @file
 * @brief      ADS101x/111x ADC device driver
 *
 * ADC and alert functionality are separated into two devices to
 * prevent wasteful representations on muxed devices.
 *
 * @author     Vincent Dupont <vincent@otakeys.com>
 * @author     Matthew Blue <matthew.blue.neuro@gmail.com>
 */

#ifndef ADS101X_H
#define ADS101X_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32wbxx.h"
/**
 * @defgroup drivers_ads101x_config    ADS101 driver compile configuration
 * @ingroup config_drivers_sensors
 * @{
 */
extern I2C_HandleTypeDef hi2c1;
/**
 * @brief   Set ADS101x/111x default I2C address
 *
 * Default STM32 I2C module
 */

#ifndef ADS101X_PARAM_I2C
#define ADS101X_PARAM_I2C    (&hi2c1)
#endif
/**
 * alert pin status
 */
#ifndef ADS101X_PARAM_ALERT_TRUE
#define ADS101X_PARAM_ALERT_TRUE    1
#endif
#ifndef ADS101X_PARAM_ALERT_FALSE
#define ADS101X_PARAM_ALERT_FALSE    0
#endif
/** @} */
/**
 * Tx or Rx Buffer size
 */
#ifndef ADS101X_BUFFER_SIZE
#define ADS101X_BUFFER_SIZE    4
#endif
/** @} */
/**
 * Tx data Buffer size
 */
#ifndef ADS101X_TX_DATA_SIZE
#define ADS101X_TX_DATA_SIZE    1
#endif
/** @} */
/**
 * Rx data Buffer size
 */
#ifndef ADS101X_RX_DATA_SIZE
#define ADS101X_RX_DATA_SIZE    3
#endif
/** @} */
/**
 * I2C timeout
 */
#ifndef ADS101X_I2C_TIMEOUT
#define ADS101X_I2C_TIMEOUT    2
#endif
/** @} */
/**
 * Address pin tied to: GND (0x48), Vcc (0x49), SDA (0x50), SCL (0x51)
 */
#ifndef CONFIG_ADS101X_I2C_ADDRESS
#define CONFIG_ADS101X_I2C_ADDRESS    (0x49)
#endif
/** @} */
/**
 * @brief   Named return values
 */
enum {
    ADS101X_OK          =  0,       	/**< everything was fine */
    ADS101X_NOI2C       = -1,       	/**< I2C communication failed */
	ADS101X_I2CBUSY     = -2,       	/**< I2C communication failed */
    ADS101X_NODEV       = -3,       	/**< no ADS101X device found on the bus */
    ADS101X_NODATA      = -4        	/**< no data available */
};

/**
 * @brief   ADS101x/111x input data
 */
typedef struct ads101x_data {
	uint8_t config[2];					/**< data from config register */
	uint8_t ain0[2];					/**< data from single-ended input AIN0 */
	uint8_t ain1[2];					/**< data from single-ended input AIN1 */
	uint8_t ain2[2];					/**< data from single-ended input AIN2 */
	uint8_t ain3[2];					/**< data from single-ended input AIN3 */
} ads101x_data_t;

/**
 * @brief   ADS101x/111x params
 */
typedef struct ads101x_params {
	I2C_HandleTypeDef* i2cHandle;		/**< i2c device */
    uint8_t addr;						/**< i2c address */
    uint8_t mux_gain;					/**< Mux and gain boolean settings */
} ads101x_params_t;

/**
 * @brief   ADS101x/111x alert params
 */
typedef struct ads101x_alert_params {
	I2C_HandleTypeDef* i2cHandle;		/**< i2c device */
    uint8_t addr;						/**< i2c address */
    uint8_t alert_pin_status;			/**< alert pin (GPIO_UNDEF if not connected) */
    int16_t low_limit;					/**< alert low value */
    int16_t high_limit;					/**< alert high value */
} ads101x_alert_params_t;

/**
 * @brief   ADS101x/111x device descriptor
 */
typedef struct ads101x {
    ads101x_params_t params;    		/**< device driver configuration */
} ads101x_t;

/**
 * @brief   ADS101x/111x alert callback
 */
typedef void (*ads101x_alert_cb_t)(void *);

/**
 * @brief   ADS101x/111x alert device descriptor
 */
typedef struct ads101x_alert {
    ads101x_alert_params_t params;    /**< device driver configuration */
    ads101x_alert_cb_t cb;            /**< alert callback */
    void *arg;                        /**< alert callback param */
} ads101x_alert_t;

/**
 * @brief   Initialize an ADS101x/111x ADC device (ADC only)
 *
 * @param[in,out] dev  device descriptor
 * @param[in] params   device configuration
 *
 * @return zero on successful initialization, non zero on error
 */
int ads101x_init(ads101x_params_t *, uint8_t *);

/**
 * @brief   Initialize an ADS101x/111x alert device
 *
 * @param[in,out] dev  device descriptor
 * @param[in] params   device configuration
 *
 * @return zero on successful initialization, non zero on error
 */
int ads101x_alert_init(ads101x_alert_t *dev,
                       const ads101x_alert_params_t *params);

/**
 * @brief   Set mux and gain
 *
 * Mux settings have no effect on ADS1013-4 and ADS1113-4.
 * Gain settings have no effect on ADS1013 and ADS1113.
 *
 * @param[in] dev       device descriptor
 * @param[in] mux_gain  mux and gain boolean values
 *
 * @return zero on successful read, non zero on error
 */
int ads101x_set_mux_gain(const ads101x_t *dev, uint8_t mux_gain);

/**
 * @brief   Read a raw ADC value
 *
 * @param[in] dev   device descriptor
 * @param[out] raw  read value
 *
 * @return zero on successful read, non zero on error
 */
int ads101x_read_raw(const ads101x_params_t *, ads101x_data_t *);

/**
 * @brief   Enable alert interrupt
 *
 * Alert settings have no effect on ADS1013 and ADS1113.
 *
 * @param[in] dev   device descriptor
 * @param[in] cb    callback called when the alert fires
 * @param[in] arg   callback argument
 *
 * @return zero on success, non zero on error
 */
int ads101x_enable_alert(ads101x_alert_t *dev,
                         ads101x_alert_cb_t cb, void *arg);

/**
 * @brief   Set the alert parameters
 *
 * Alert settings have no effect on ADS1013 and ADS1113.
 *
 * @param[in,out] dev      device descriptor
 * @param[in] low_limit    alert low limit
 * @param[in] high_limit   alert high limit
 *
 * @return zero on success, non zero on error
 */
int ads101x_set_alert_parameters(const ads101x_alert_t *dev,
                                 int16_t low_limit, int16_t high_limit);

#ifdef __cplusplus
}
#endif

#endif /* ADS101X_H */
/** @} */
