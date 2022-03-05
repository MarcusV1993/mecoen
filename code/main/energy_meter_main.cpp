/*
 * Final project to achieve degree in Electrical Engineering with Emphasis in Electronic Systems at University of São Paulo
 * Project: Real Time Home Energy Meter with Web Application Interface
 * Author: Marcus Vinicius Gonçalves Mendes
 * Advisors: Roseli de Deus Lopes (EPUSP), Marcelo Knörich Zuffo (EPUSP), Alberto Ferreira de Sousa (UFES)
 *
 */

/*
 * Access point adapted from:
 * https://github.com/espressif/esp-idf/tree/v4.3/examples/wifi/getting_started/softAP
 * */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "esp_timer.h"
#include "esp_adc_cal.h"
#include "esp_bit_defs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

//#include "wifi_ap.cpp"
//#include "wifi_sta.cpp"

//#include "ds3231.h"


// ADC definitions
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel_v = ADC_CHANNEL_6;     // GPIO34
static const adc_channel_t channel_i = ADC_CHANNEL_7;	  // GPIO35
static const adc_atten_t atten = ADC_ATTEN_DB_11; // max = 3.9V -> https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html
static const adc_unit_t unit = ADC_UNIT_1;

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   4          //Multisampling

#define V_SAMPLES		2500		// Number of voltage samples
#define SAMPLING_PERIOD_MS 1
static const uint32_t SAMPLING_PERIOD_US = 1e6 / V_SAMPLES; // Real sampling frequency slightly lower than 1e6/SAMPLING_PERIOD_US
static const float signal_to_rms = 1 / (V_SAMPLES * (1000 * SAMPLING_PERIOD_MS));

// voltage sensor parameters
#define ZMPT101B_VCC            4.4
#define ZMPT101B_R1			   9840 // 10k
#define ZMPT101B_R2			   9970 // 10k
/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/
#define ZMPT101B_VMAX		 0.8202 /* Calibrar /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\*/
/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/
static const float zmpt101b_dc_bias = 1192/*(ZMPT101B_VCC * 1000 / 2) * ZMPT101B_R2 / (ZMPT101B_R1 + ZMPT101B_R2)*/; // Calculated: 1.107 V Measured: 1217 mV
static const float zmpt101b_calibration = (220 * sqrt(2)) / (ZMPT101B_VMAX * 1000 / 2);
// end voltage sensor parameters

// current sensor parameters
#define SCT013_VCC 				3.3
#define SCT013_NUMBER_TURNS    2000
#define SCT013_BURDEN_RESISTOR  466 // 470
#define SCT013_R1             21700 // 22k
#define SCT013_R2              9870 // 10k
static const float sct013_dc_bias = 1046;/*SCT013_VCC * 1000 * SCT013_R2 / (SCT013_R1 + SCT013_R2);*/ // Calculated: 1,032 V Measured: 1130 mV
static const float sct013_calibration = (SCT013_NUMBER_TURNS / SCT013_BURDEN_RESISTOR) / 1000;
// end current sensor parameters
// end ADC definitions

// AP definitions
#define ESP_WIFI_SSID      "energy_meter"
#define ESP_WIFI_PASS      "pass1234"
#define ESP_WIFI_CHANNEL   9
#define MAX_STA_CONN       3

static const char *TAG_AP = "wifi softAP";

// end AP definitions

// I2C definitions
#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define DS3231                 0x68        /*!< Slave address of the DS3231 RTC */
#define AT24C32                0x57        /*!< Slave address of the AT24C32 EEPROM module embedded in the DS3231 module */
// end I2C definitions

// structures
typedef struct Signal
{
	float samples[V_SAMPLES];
	float rms_previous, rms;
} Signal;

typedef struct Circuit_phase
{
	Signal voltage, current, power;
} Circuit_phase;
// end structures


// time related functions
#define NOP() asm volatile ("nop")


unsigned long IRAM_ATTR micros()
{
    return (unsigned long) (esp_timer_get_time());
}


void IRAM_ATTR delayMicroseconds(uint32_t us)
{
    uint32_t m = micros();
    if(us){
        uint32_t e = (m + us);
        if(m > e){ //overflow
            while(micros() > e){
                NOP();
            }
        }
        while(micros() < e){
            NOP();
        }
    }
}


double
ojGetTimeSec()
{
	int64_t t = esp_timer_get_time();
	double t_seconds = (double) t / 1000000.0;

	return (t_seconds);
}


void
ojSleepMsec(double miliseconds)
{
	double t = ojGetTimeSec();
    while ((ojGetTimeSec() - t) < (miliseconds / 1000.0));
}
// end time related functions


// ADC related functions
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}


void
init_adc()
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

	// Init breaks pontentiometer sensor (https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html)
    // Usamos a ADC_UNIT_1

	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten((adc1_channel_t) channel_v, atten);
	adc1_config_channel_atten((adc1_channel_t) channel_i, atten); // Second ADC channel

    //Characterize ADC
    adc_chars = (esp_adc_cal_characteristics_t *) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    printf("\nadc initialized!\n");
}


double
get_adc1_value(adc_channel_t adc_channel)
{
	uint32_t adc_reading = 0;

	// Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++)
		adc_reading += adc1_get_raw((adc1_channel_t) adc_channel);

	adc_reading /= NO_OF_SAMPLES;
	//Convert adc_reading to voltage in mV
	int voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

	return (voltage);
}


void
read_phase(void *arg)
{
	Circuit_phase *phase = (Circuit_phase *) arg;
	phase->voltage.rms = phase->voltage.rms_previous = 0.0;

	int sample = 0;

	while(1)
	{
//		Reading ADC
		phase->voltage.samples[sample] = get_adc1_value(channel_v);
		phase->current.samples[sample] = get_adc1_value(channel_i);

//		Convert ADC readings to real world value
		phase->voltage.samples[sample] = zmpt101b_calibration * (phase->voltage.samples[sample] - zmpt101b_dc_bias);
		phase->current.samples[sample] =   sct013_calibration * (phase->current.samples[sample] -   sct013_dc_bias);

//		Instant power
		phase->power.samples[sample] = phase->voltage.samples[sample] * phase->current.samples[sample];

//		RMS
		phase->voltage.rms += phase->voltage.samples[sample]*phase->voltage.samples[sample];
		phase->current.rms += phase->current.samples[sample]*phase->current.samples[sample];

		sample++;
		if(sample >= V_SAMPLES)
		{
			sample = 0;
//	Imprecision for not calculating RMS considering integer multiples of the signal period
			phase->voltage.rms_previous = sqrt(phase->voltage.rms);
			phase->current.rms_previous = sqrt(phase->current.rms);

			phase->power.rms_previous = phase->voltage.rms_previous * phase->current.rms_previous;

			phase->voltage.rms = 0.0;
			phase->current.rms = 0.0;
		}

		delayMicroseconds(SAMPLING_PERIOD_US);
	}
}
// end ADC related functions


// AP related functions
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG_AP, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG_AP, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    //  convert from C to C++
    //  https://esp32.com/viewtopic.php?t=1317
    wifi_config_t wifi_config = { };
    memset(&wifi_config, 0, sizeof(wifi_config));
    strcpy(reinterpret_cast<char*>(wifi_config.ap.ssid), ESP_WIFI_SSID);
    wifi_config.ap.ssid_len = strlen(ESP_WIFI_SSID);
    wifi_config.ap.channel = ESP_WIFI_CHANNEL;
    strcpy(reinterpret_cast<char*>(wifi_config.ap.password), ESP_WIFI_PASS);
    wifi_config.ap.max_connection = MAX_STA_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    if (strlen(ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_AP, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             ESP_WIFI_SSID, ESP_WIFI_PASS, ESP_WIFI_CHANNEL);
}
// end AP related functions


// NVS related functions
static esp_err_t
init_nvs()
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}

	return ret;
}
// NVS related functions


// I2C functions
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

// end I2C functions


Circuit_phase phase_a;

static const int REASON = 4;
float phase_copy[V_SAMPLES/REASON][2];

#ifdef __cplusplus
extern "C"
#endif
void app_main()
{
	esp_err_t ret;
	init_adc();

	ret = init_nvs();
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG_AP, "ESP_WIFI_MODE_AP");
	wifi_init_softap();

//    ret = ds3231_init_desc()

//    ESP_LOGI(TAG_STA, "ESP_WIFI_MODE_STA");
//    wifi_init_sta();

    xTaskCreatePinnedToCore(read_phase, "read_phase", 2048, &phase_a, 5, NULL, 1);
    printf("\nread_phase task initialized!\n");

	vTaskDelay(500 / portTICK_RATE_MS);
    int cnt = 0;

    printf("\nMain loop initialized!\n");
    while (1)
    {
		cnt++;

		delayMicroseconds(1000000);

		for (int i = 0; i < V_SAMPLES/REASON; i++)
		{
			phase_copy[i][0] = phase_a.voltage.samples[i];
			phase_copy[i][1] = phase_a.current.samples[i];
		}

		for (int i = 0; i < V_SAMPLES/REASON; i++)
		{
			printf("%04.2f %04.2f\n", phase_copy[i][0], phase_copy[i][1]);
			fflush(stdout);
			vTaskDelay(10 / portTICK_RATE_MS);
		}

		printf("\n\n");
		fflush(stdout);
    }
}
