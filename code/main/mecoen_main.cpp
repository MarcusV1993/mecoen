/*
 * Final project to achieve degree in Electrical Engineering with Emphasis in Electronic Systems at University of São Paulo
 * Project: Real Time Home Energy Meter with Web Application Interface
 * Author: Marcus Vinicius Gonçalves Mendes
 * Advisors: Roseli de Deus Lopes (EPUSP), Marcelo Knörich Zuffo (EPUSP), Alberto Ferreira de Sousa (UFES)
 *
*/

/*
 * Web server adapted from:
 * https://github.com/caiomb/esp32-http_webserver
*/

// includes
//// includes std c libraries
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
//// end includes std c libraries

//// includes esp-idf libraries
////// includes esp-idf system
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h" // memmory types
////// end includes esp-idf system

////// includes esp -idf error handling
#include "esp_err.h"
////// end includes esp -idf error handling

////// includes esp-idf freeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
////// end includes esp-idf freeRTOS

////// includes -esp-idf time
#include "esp_timer.h"
#include "driver/gptimer.h"

//////// includes esp-idf time ntp
#include "esp_sntp.h"
//////// end includes esp-idf time ntp
////// end includes -esp-idf time

////// includes esp-idf ADC
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "driver/gpio.h"
////// end includes esp-idf ADC


//// end includes esp-idf libraries
// end includes
//#include "mecoen_adc.h"
#include "mecoen_wifi.h"
#include "server.h"
#include "mecoen_commons.h"
#include "mecoen_definitions.h"
#include "mecoen_fft.h"
//#include "mecoen_time.h"
#include "mecoen_i2c.h"


//// const ADC
////// const ADC period
static const int32_t SAMPLING_PERIOD_US = 1e6 / SAMPLING_FREQUENCY; // Real sampling frequency slightly lower than 1e6/SAMPLING_PERIOD_US
static const float sampling_frequency = 1e6 / SAMPLING_PERIOD_US;
static const float signal_to_rms = 1 / (SAMPLING_FREQUENCY * SAMPLING_PERIOD_US);
////// end const ADC period

////// const ADC ports and configuration
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel_v = ADC_CHANNEL_6;     // GPIO34
static const adc_channel_t channel_i = ADC_CHANNEL_7;	  // GPIO35
static const adc_atten_t atten = ADC_ATTEN_DB_11; // max = 3.9V -> https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html
static const adc_unit_t unit = ADC_UNIT_1;
////// end const ADC ports and configuration

////// const ADC voltage sensor
static const float zmpt101b_dc_bias = 1139/*(ZMPT101B_VCC * 1000 / 2) * ZMPT101B_R2 / (ZMPT101B_R1 + ZMPT101B_R2)*/; // Calculated: 1.107 V Measured: 1217 mV
static const float zmpt101b_calibration = (220 * sqrt(2)) / (ZMPT101B_VMAX * 1000 / 2);
////// end const ADC voltage sensor

////// const ADC current sensor
static const float sct013_dc_bias = 1025;/*SCT013_VCC * 1000 * SCT013_R2 / (SCT013_R1 + SCT013_R2);*/ // Calculated: 1,032 V Measured: 1130 mV
static const float sct013_calibration = (SCT013_NUMBER_TURNS / SCT013_BURDEN_RESISTOR);
////// end const ADC current sensor
//// end const ADC
// end const

// global variables
Circuit_phase phase_a;
float phase_copy[N_ARRAY_LENGTH / REASON][3];


//// global variables semaphores
//TaskHandle_t task_adc = NULL, task_fft = NULL;
SemaphoreHandle_t semaphore_adc = xSemaphoreCreateMutex();
SemaphoreHandle_t semaphore_fft = xSemaphoreCreateMutex();
//// end global variables semaphores

//// global variables time
////// global variables time ntp
/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
RTC_DATA_ATTR static int boot_count = 0;
static const char *TAG_TIME = "mecoen_time";
////// end global variables time ntp
//// end global variables time

//// global variables ADC
float zmpt101b_vdc = ZMPT101B_VDC;
float  sct013_vdc = SCT013_VDC;
//// end global variables ADC
// end global variables


// functions
//// functions time
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


////// functions time ntp
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_CUSTOM
void sntp_sync_time(struct timeval *tv)
{
   settimeofday(tv, NULL);
   ESP_LOGI(TAG_TIME, "Time is synchronized from custom code");
   sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
}
#endif


void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG_TIME, "Notification of a time synchronization event");
}


static void initialize_sntp(void)
{
    ESP_LOGI(TAG_TIME, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);

/*
 * If 'NTP over DHCP' is enabled, we set dynamic pool address
 * as a 'secondary' server. It will act as a fallback server in case that address
 * provided via NTP over DHCP is not accessible
 */
#if LWIP_DHCP_GET_NTP_SRV && SNTP_MAX_SERVERS > 1
    sntp_setservername(1, "pool.ntp.org");

#if LWIP_IPV6 && SNTP_MAX_SERVERS > 2          // statically assigned IPv6 address is also possible
    ip_addr_t ip6;
    if (ipaddr_aton("2a01:3f7::1", &ip6)) {    // ipv6 ntp source "ntp.netnod.se"
        sntp_setserver(2, &ip6);
    }
#endif  /* LWIP_IPV6 */

#else   /* LWIP_DHCP_GET_NTP_SRV && (SNTP_MAX_SERVERS > 1) */
    // otherwise, use DNS address from a pool
    sntp_setservername(0, MECOEN_SNTP_SERVER_NAME);
#endif

    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();

    ESP_LOGI(TAG_TIME, "List of configured NTP servers:");

    for (uint8_t i = 0; i < SNTP_MAX_SERVERS; ++i){
        if (sntp_getservername(i)){
            ESP_LOGI(TAG_TIME, "server %d: %s", i, sntp_getservername(i));
        } else {
            // we have either IPv4 or IPv6 address, let's print it
            char buff[INET6_ADDRSTRLEN];
            ip_addr_t const *ip = sntp_getserver(i);
            if (ipaddr_ntoa_r(ip, buff, INET6_ADDRSTRLEN) != NULL)
                ESP_LOGI(TAG_TIME, "server %d: %s", i, buff);
        }
    }
}


static void obtain_time(void)
{
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { };
    int retry = 0;
    const int retry_count = 15;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG_TIME, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
	setenv("TZ", "EBST3", 1);
	tzset();
	localtime_r(&now, &timeinfo);
    timeval now_timeval = {now, 0};
    settimeofday(&now_timeval, NULL);
//char strftime_buf[64];
//strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
//printf("The current date/time is: %s\n", strftime_buf);
}


void
init_time()
{
	++boot_count;
	ESP_LOGI(TAG_TIME, "Boot count: %d", boot_count);

	time_t now;
	struct tm timeinfo;
	time(&now);
	localtime_r(&now, &timeinfo);
	setenv("TZ", "EBST3", 1);
	tzset();
	localtime_r(&now, &timeinfo);
//char strftime_buf[64];
//strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
//printf("The current date/time is: %s\n", strftime_buf);
	// Is time set? If not, tm_year will be (1970 - 1900).
	if (timeinfo.tm_year < (2016 - 1900)) {
		ESP_LOGI(TAG_TIME, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
		obtain_time();
		// update 'now' variable with current time
		time(&now);
	}

	// Set timezone to Brasilia
	setenv("TZ", "EBST3", 1);
	tzset();
	localtime_r(&now, &timeinfo);

	if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
		struct timeval outdelta;
		while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
			adjtime(NULL, &outdelta);
			ESP_LOGI(TAG_TIME, "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
						(long)outdelta.tv_sec,
						outdelta.tv_usec/1000,
						outdelta.tv_usec%1000);
			vTaskDelay(2000 / portTICK_PERIOD_MS);
		}
	}
}
////// end functions time ntp
//// end functions time


//// functions ADC
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

//	uint32_t ret = 0;
	int sample_num = 0;

//	xTaskNotifyStateClearIndexed( NULL, indexToWaitOn );
//	configASSERT(task_fft != NULL);
//	xSemaphoreTake(semaphore_adc_fft, portMAX_DELAY);

	for (sample_num = 0; sample_num < N_ARRAY_LENGTH; sample_num++)
	{
		phase->voltage.samples[sample_num] = get_adc1_value(channel_v);
		phase->current.samples[sample_num] = get_adc1_value(channel_i);
		zmpt101b_vdc = ( ( (N_ARRAY_LENGTH - 1) * zmpt101b_vdc) + phase->voltage.samples[sample_num]) / N_ARRAY_LENGTH;
		sct013_vdc = ( ( (N_ARRAY_LENGTH - 1) * sct013_vdc) + phase->current.samples[sample_num]) / N_ARRAY_LENGTH;
	}
	sample_num = 0;

	while(1)
	{
		// Remove last read value from moving average DC
		zmpt101b_vdc -= phase->voltage.samples[sample_num] / N_ARRAY_LENGTH;
		sct013_vdc -= phase->current.samples[sample_num] / N_ARRAY_LENGTH;

//		Reading ADC
		phase->voltage.samples[sample_num] = get_adc1_value(channel_v);
//		delayMicroseconds(1);
		phase->current.samples[sample_num] = get_adc1_value(channel_i);

		// Add newest read value to moving average DC
		zmpt101b_vdc += phase->voltage.samples[sample_num] / N_ARRAY_LENGTH;
		sct013_vdc += phase->current.samples[sample_num] / N_ARRAY_LENGTH;

//		Remove DC bias
//		phase->voltage.samples[sample_num] = (phase->voltage.samples[sample_num] - zmpt101b_dc_bias);
//		phase->current.samples[sample_num] = (phase->current.samples[sample_num] - sct013_dc_bias);

//		Convert mV -> V
//		phase->voltage.samples[sample_num] /= 1000;
//		phase->current.samples[sample_num] /= 1000;

//		Convert ADC readings to real world value
//		phase->voltage.samples[sample] = zmpt101b_calibration * (phase->voltage.samples[sample] - zmpt101b_dc_bias);
//		phase->current.samples[sample] =   sct013_calibration * (phase->current.samples[sample] -   sct013_dc_bias);

//		Instant power
//		phase->power.samples[sample_num] = phase->voltage.samples[sample_num] * phase->current.samples[sample_num];

//		RMS
//		phase->voltage.rms += phase->voltage.samples[sample_num]*phase->voltage.samples[sample_num];
//		phase->current.rms += phase->current.samples[sample_num]*phase->current.samples[sample_num];

		sample_num++;
		if (sample_num >= N_ARRAY_LENGTH)
		{
			sample_num = 0;

//			xSemaphoreGive(semaphore_fft);
//			xTaskNotifyGiveIndexed(task_fft, indexToWaitOn); // Notify fft task that array has been filled
//printf("\nADC give task\n");
//delayMicroseconds(100);
//			xSemaphoreTake(semaphore_adc, portMAX_DELAY);
//			ulTaskNotifyTakeIndexed(indexToWaitOn, pdFALSE, portMAX_DELAY/*pdMS_TO_TICKS(5000)*/); // Wait until fft task finished copying array
//printf("\nADC take task\n");

//	Imprecision for not calculating RMS considering integer multiples of the signal period
//			phase->voltage.rms_previous = sqrt(phase->voltage.rms / SAMPLING_FREQUENCY);
//			phase->current.rms_previous = sqrt(phase->current.rms / SAMPLING_FREQUENCY);
//
//			phase->power.rms_previous = phase->voltage.rms_previous * phase->current.rms_previous;
//
//			phase->voltage.rms = 0.0;
//			phase->current.rms = 0.0;
		}

		delayMicroseconds(SAMPLING_PERIOD_US / 4);
	}
}


/*****************************************************************************************************************************/
void
adc_read_interrupt(adc_channel_t channel_v1, adc_channel_t channel_i1, uint32_t *readings)
{
/*
 * Makes multi-sampling readings of the voltage and current inputs, and returns the sum of the samples in "readings" array
 * Input: channel_v1: channel of the voltage sensor
 *        channel_i1: channel of the current sensor
 *        readings: pointer in which will be stored the sum of the uint32t readings.
 *        	Even positions store voltage readings
 *        	Odd positions store current readings
 */
	readings[0] = 0;
	readings[1] = 0;

	// Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++)
	{
		readings[0] += adc1_get_raw((adc1_channel_t) channel_v1);
		readings[1] += adc1_get_raw((adc1_channel_t) channel_i1);
	}
}


void
read_phase2(void *arg)
{
	Circuit_phase *phase = (Circuit_phase *) arg;
	phase->voltage.rms = phase->voltage.rms_previous = 0.0;

	float zmpt101b_vdc2 = 0.0;
	float sct013_vdc2 = 0.0;

	uint32_t readings[2];

	int sample_num = 0;

	// Calculate the average DC value of each sensor
	for (sample_num = 0; sample_num < N_ARRAY_LENGTH; sample_num++)
	{
		// Reading ADC
		adc_read_interrupt(channel_v, channel_i, readings);
		//Convert adc_reading to voltage in mV
		phase->voltage.samples[sample_num] = (float) esp_adc_cal_raw_to_voltage(readings[0] / NO_OF_SAMPLES, adc_chars);
		phase->current.samples[sample_num] = (float) esp_adc_cal_raw_to_voltage(readings[1] / NO_OF_SAMPLES, adc_chars);
		zmpt101b_vdc2 += phase->voltage.samples[sample_num];
		sct013_vdc2 += phase->current.samples[sample_num];
	}
	zmpt101b_vdc2 /= N_ARRAY_LENGTH;
	sct013_vdc2 /= N_ARRAY_LENGTH;
	sample_num = 0;

	zmpt101b_vdc = zmpt101b_vdc2;
	sct013_vdc = sct013_vdc2;
	while(1)
	{
		// Remove last read value from moving average DC
//		zmpt101b_vdc2 -= phase->voltage.samples[sample_num] / N_ARRAY_LENGTH;
//		sct013_vdc2 -= phase->current.samples[sample_num] / N_ARRAY_LENGTH;

		zmpt101b_vdc -= phase->voltage.samples[sample_num] / N_ARRAY_LENGTH;
		sct013_vdc -= phase->current.samples[sample_num] / N_ARRAY_LENGTH;

//		Reading ADC
		adc_read_interrupt(channel_v, channel_i, readings);
		//Convert adc_reading to voltage in mV
		phase->voltage.samples[sample_num] = (float) esp_adc_cal_raw_to_voltage(readings[0] / NO_OF_SAMPLES, adc_chars);
		phase->current.samples[sample_num] = (float) esp_adc_cal_raw_to_voltage(readings[1] / NO_OF_SAMPLES, adc_chars);

		// Add newest read value to moving average DC
//		zmpt101b_vdc2 += phase->voltage.samples[sample_num] / N_ARRAY_LENGTH;
//		sct013_vdc2 += phase->current.samples[sample_num] / N_ARRAY_LENGTH;

		zmpt101b_vdc += phase->voltage.samples[sample_num] / N_ARRAY_LENGTH;
		sct013_vdc += phase->current.samples[sample_num] / N_ARRAY_LENGTH;

//		Remove DC bias
//		phase->voltage.samples[sample_num] = phase->voltage.samples[sample_num] - zmpt101b_vdc2;
//		phase->current.samples[sample_num] = phase->current.samples[sample_num] - sct013_vdc2;

//		Convert real world value
//		phase->voltage.samples[sample_num] *= ZMPT101B_CONSTANT_MULTIPLIER;
//		phase->current.samples[sample_num] *= SCT013_CONSTANT_MULTIPLIER;

		sample_num++;
		if (sample_num >= N_ARRAY_LENGTH)
			sample_num = 0;

		delayMicroseconds(SAMPLING_PERIOD_US / 4);
	}
}
//// end functions ADC
// end functions

#ifdef __cplusplus
extern "C"
#endif
void app_main()
{
	// Initializers
		// SNTP
	/**
	 * NTP server address could be aquired via DHCP,
	 * see following menuconfig options:
	 * 'LWIP_DHCP_GET_NTP_SRV' - enable STNP over DHCP
	 * 'LWIP_SNTP_DEBUG' - enable debugging messages
	 *
	 * NOTE: This call should be made BEFORE esp aquires IP address from DHCP,
	 * otherwise NTP option would be rejected by default.
	 */
	#ifdef LWIP_DHCP_GET_NTP_SRV
		sntp_servermode_dhcp(1);      // accept NTP offers from DHCP server, if any
	#endif

		// Initialize Wi-Fi in Access Point + Station mode
	wifi_init_ap_sta();

		// Circuit phases
	init_phase(&phase_a);

		// ADC
	init_adc();
	printf("\ninit_adc!\n");

		// FFT
	ESP_ERROR_CHECK(init_fft());
	printf("\ninit_fft!\n");

		// SNTP
	init_time();
	char strftime_buf[64];
	struct tm timeinfo;
	timeval now;

		// I2C | RTC DS3231
	gettimeofday(&now, NULL);
	localtime_r(&now.tv_sec, &timeinfo);
	init_rtc_ds3231(&timeinfo);
	// end Initializers

	// Take semaphore to sync FFT and ADC tasks
//	xSemaphoreTake(semaphore_fft, portMAX_DELAY);
//	xSemaphoreTake(semaphore_adc, portMAX_DELAY);

	// Create Tasks
		// Web server
	xTaskCreate(&http_server, "http_server", 2048, NULL, 5, NULL);

		// ADC
//    xTaskCreatePinnedToCore(read_phase, "read_phase", 2048, &phase_a, 5, &task_adc, 1);
    xTaskCreatePinnedToCore(read_phase2, "read_phase2", 2048, &phase_a, 5, NULL/*&task_adc*/, 1);
    printf("\nread_phase task initialized!\n");

    	// FFT
//    xTaskCreatePinnedToCore(fft_continuous, "fft_continuous", 2048, &phase_a, 5, &task_fft, 1);
//	printf("\nread_fft task initialized!\n");

		// I2C | RTC DS3231
//    xTaskCreate(rtc_ds3231, "rtc_ds3231", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
	// end Create Tasks

	vTaskDelay(500 / portTICK_RATE_MS);
    printf("\nMain loop initialized!\n");
    while (1)
    {
		delayMicroseconds((int) 1e6); // 5 s
//        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 s

//		gettimeofday(&now, NULL);
//		localtime_r(&now.tv_sec, &timeinfo);
//		strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
//		printf("The current date/time is: %s\n", strftime_buf);
		phase_a.voltage.rms = 0;
		phase_a.current.rms = 0;

		for (int i = 0; i < N_ARRAY_LENGTH / REASON; i++)
		{
			phase_copy[i][0] = phase_a.voltage.samples[i] - zmpt101b_vdc;
			phase_copy[i][1] = phase_a.current.samples[i] - sct013_vdc;

//			phase_copy[i][0] = (phase_a.voltage.samples[i] - zmpt101b_vdc) * 0.60595;
//			phase_copy[i][1] = (phase_a.current.samples[i] - sct013_vdc) * 0.00932;

			phase_a.voltage.rms += phase_copy[i][0] * phase_copy[i][0];
			phase_a.current.rms += phase_copy[i][1] * phase_copy[i][1];

			phase_copy[i][2] = phase_copy[i][0] * phase_copy[i][1];
		}

		phase_a.voltage.rms = sqrt(phase_a.voltage.rms / (N_ARRAY_LENGTH / REASON) );
		phase_a.current.rms = sqrt(phase_a.current.rms / (N_ARRAY_LENGTH / REASON) );
		phase_a.power_apparent = phase_a.voltage.rms * phase_a.current.rms;

//		vTaskDelay(10 / portTICK_RATE_MS);

		for (int i = 0; i < N_ARRAY_LENGTH / REASON; i++)
		{
			printf("%08.4f %08.4f %08.4f\n", phase_copy[i][0], phase_copy[i][1], phase_copy[i][2]);
			fflush(stdout);
			vTaskDelay(10 / portTICK_RATE_MS);
		}
		printf("\nVrms = %06.2f; Irms = %06.2f; P = %06.2f\n", phase_a.voltage.rms, phase_a.current.rms, phase_a.power_apparent);

		phase_a.voltage.rms = phase_a.voltage.rms_previous;
		phase_a.current.rms = phase_a.current.rms_previous;

		printf("\n\n");
		fflush(stdout);
    }
}
