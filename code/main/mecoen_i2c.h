/*
 * energy_meter_i2c.h
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */

#ifndef MAIN_MECOEN_I2C_H_
#define MAIN_MECOEN_I2C_H_

#include <time.h>

#define I2C_MASTER_SCL_IO           GPIO_NUM_22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000


#define DS3231                 0x68        /*!< Slave address of the DS3231 RTC */
#define AT24C32                0x57        /*!< Slave address of the AT24C32 EEPROM module embedded in the DS3231 module */

void init_rtc_ds3231(struct tm *current_time);

void rtc_ds3231 (void *arg);

#endif /* MAIN_MECOEN_I2C_H_ */
