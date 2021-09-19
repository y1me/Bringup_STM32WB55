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
 * @brief       Register definition for ADS101x/111x devices
 *
 * @author      Vincent Dupont <vincent@otakeys.com>
 * @author      Matthew Blue <matthew.blue.neuro@gmail.com>
 */

#ifndef ADS101X_REGS_H
#define ADS101X_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    ADS101x/111x register addresses
 * @{
 */
#define ADS101X_CONV_RES_ADDR      (0)
#define ADS101X_CONF_ADDR          (1)
#define ADS101X_LOW_LIMIT_ADDR     (2)
#define ADS101X_HIGH_LIMIT_ADDR    (3)
/** @} */

/**
 * @name    ADS101x/111x Config flags
 *
 * Comparator flags have no effect on ADS1013 and ADS1113.
 *
 * @{
 */
#define ADS101X_CONF_OS_CONV          (1 << 7)
#define ADS101X_CONF_COMP_MODE_WIND   (1 << 4)
#define ADS101X_CONF_COMP_LATCHING    (1 << 2)
#define ADS101X_CONF_COMP_MASK        ((1 << 1) | (1 << 0))
#define ADS101X_CONF_COMP_ONE         ((0 << 1) | (0 << 0))
#define ADS101X_CONF_COMP_TWO         ((0 << 1) | (1 << 0))
#define ADS101X_CONF_COMP_FOUR        ((1 << 1) | (0 << 0))
#define ADS101X_CONF_COMP_DIS         ((1 << 1) | (1 << 0))
/** @} */

/**
 * @name    ADS101x/111x mux settings
 *
 * Supports both single mode and differential.
 * This has no effect on ADS1013-4 and ADS1113-4.
 *
 * @{
 */
#define ADS101X_MUX_MASK           ((1 << 6) | (1 << 5) | (1 << 4))
#define ADS101X_AIN0_DIFFM_AIN1    ((0 << 6) | (0 << 5) | (0 << 4))
#define ADS101X_AIN0_DIFFM_AIN3    ((0 << 6) | (0 << 5) | (1 << 4))
#define ADS101X_AIN1_DIFFM_AIN3    ((0 << 6) | (1 << 5) | (0 << 4))
#define ADS101X_AIN2_DIFFM_AIN3    ((0 << 6) | (1 << 5) | (1 << 4))
#define ADS101X_AIN0_SINGM         ((1 << 6) | (0 << 5) | (0 << 4))
#define ADS101X_AIN1_SINGM         ((1 << 6) | (0 << 5) | (1 << 4))
#define ADS101X_AIN2_SINGM         ((1 << 6) | (1 << 5) | (0 << 4))
#define ADS101X_AIN3_SINGM         ((1 << 6) | (1 << 5) | (1 << 4))
/** @} */

/**
 * @name    ADS101x/111x programmable gain
 *
 * Sets the full-scale range (max voltage value).
 * This has no effect on ADS1013 and ADS1113 (both use 2.048V FSR).
 *
 * @{
 */
#define ADS101X_PGA_MASK         ((1 << 3) | (1 << 2) | (1 << 1))
#define ADS101X_PGA_FSR_6V144    ((0 << 3) | (0 << 2) | (0 << 1))
#define ADS101X_PGA_FSR_4V096    ((0 << 3) | (0 << 2) | (1 << 1))
#define ADS101X_PGA_FSR_2V048    ((0 << 3) | (1 << 2) | (0 << 1))
#define ADS101X_PGA_FSR_1V024    ((0 << 3) | (1 << 2) | (1 << 1))
#define ADS101X_PGA_FSR_0V512    ((1 << 3) | (0 << 2) | (0 << 1))
#define ADS101X_PGA_FSR_0V256    ((1 << 3) | (0 << 2) | (1 << 1))
/** @} */

/**
 * @name    ADS101x/111x operation mode
 *
 * Determines how quickly samples are taken (even on one-shot mode)
 *
 * @{
 */
#define ADS101X_MODE_MASK     ((1 << 8))
#define ADS101X_MODE_SSM      ((1 << 8))
#define ADS101X_MODE_CON      ((0 << 8))
/** @} */

/**
 * @name    ADS101x/111x data rate settings
 *
 * Determines how quickly samples are taken (even on one-shot mode)
 *
 * @{
 */
#define ADS101X_DATAR_MASK    ((1 << 7) | (1 << 6) | (1 << 5))
#define ADS101X_DATAR_8       ((0 << 7) | (0 << 6) | (0 << 5))
#define ADS101X_DATAR_16      ((0 << 7) | (0 << 6) | (1 << 5))
#define ADS101X_DATAR_32      ((0 << 7) | (1 << 6) | (0 << 5))
#define ADS101X_DATAR_64      ((0 << 7) | (1 << 6) | (1 << 5))
#define ADS101X_DATAR_128     ((1 << 7) | (0 << 6) | (0 << 5))
#define ADS101X_DATAR_250     ((1 << 7) | (0 << 6) | (1 << 5))
#define ADS101X_DATAR_475     ((1 << 7) | (1 << 6) | (0 << 5))
#define ADS101X_DATAR_860     ((1 << 7) | (1 << 6) | (1 << 5))
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ADS101X_REGS_H */
/** @} */
