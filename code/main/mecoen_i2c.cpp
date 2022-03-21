/*
 * energy_meter_i2c.cpp
 *
 *  Created on: 13 de mar. de 2022
 *      Author: marcus
 */

#include "driver/i2c.h"
#include "mecoen_i2c.h"

//#include "ds3231.h"

//static esp_err_t i2c_master_init(void)
//{
//    int i2c_master_port = I2C_MASTER_NUM;
//
//    i2c_config_t conf = { };
//    conf.mode = I2C_MODE_MASTER;
//    conf.sda_io_num = I2C_MASTER_SDA_IO;
//    conf.scl_io_num = I2C_MASTER_SCL_IO;
//    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
//
//    i2c_param_config(i2c_master_port, &conf);
//
//    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
//}

