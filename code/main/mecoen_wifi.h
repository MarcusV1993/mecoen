/*
 * mecoen_wifi.h
 *
 *  Created on: 20 de mar. de 2022
 *      Author: marcus
 */

#ifndef MAIN_MECOEN_WIFI_H_
#define MAIN_MECOEN_WIFI_H_

#include "esp_system.h"

// AP default configuration
#define MECOEN_WIFI_AP_SSID      "mecoen"
#define MECOEN_WIFI_AP_PASS      "12345678"
#define MECOEN_WIFI_AP_CHANNEL   9 // 1 - 13
#define MECOEN_WIFI_AP_MAX_STA_CONN       2
// end AP default configuration

// STA default configuration
// To-do: Configure for UTF characters
#define MECOEN_WIFI_STA_SSID      "Goncalves_2g" /*"POCO X3 Pro"*/
#define MECOEN_WIFI_STA_PASS      "3NMLWWXFFW"
#define MECOEN_WIFI_MAXIMUM_RETRY  5

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
// end STA default configuration

void wifi_init_ap_sta();

#endif /* MAIN_MECOEN_WIFI_H_ */
