#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h" // deprecated
//#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "server.h"

#include "mecoen_adc.h"
#include "mecoen_definitions.h"
#include "mecoen_storage.h"

#define EXAMPLE_ESP_WIFI_SSID      "mecoen"
#define EXAMPLE_ESP_WIFI_PASS      "12345678"
#define EXAMPLE_MAX_STA_CONN       2

static const char *TAG = "wifi softAP";
static EventGroupHandle_t s_wifi_event_group;
const static char http_html_hdr[] = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\n\r\n";
static char http_index_html_server[] = "\
<html>\
<body>\
<center><h1>Projeto de Graduacao em Engenharia Eletrica com Enfase em Eletronica e Sistemas</h1></center>\
<center><h1>Medidor de Consumo de Energia Eletrica Domestico de Tempo Real com Interface Via Aplicativo Web</h1></center>\
<center>#                                                                                                       </center>\
<center>#                                                                                                       </center>\
</body>\
</html>";

char message[100];


extern Circuit_phase phase_a;


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id)
    {
        case SYSTEM_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "station:" MACSTR " join, AID=%d",
                     MAC2STR(event->event_info.sta_connected.mac),
                     event->event_info.sta_connected.aid);
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "station:" MACSTR "leave, AID=%d",
                     MAC2STR(event->event_info.sta_disconnected.mac),
                     event->event_info.sta_disconnected.aid);
            break;
        default:
            break;
    }
    return ESP_OK;
}

void wifi_init_softap()
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    //  convert from C to C++
    //  https://esp32.com/viewtopic.php?t=1317
    wifi_config_t wifi_config = { };
    memset(&wifi_config, 0, sizeof(wifi_config));
    strcpy(reinterpret_cast<char*>(wifi_config.ap.ssid), EXAMPLE_ESP_WIFI_SSID);
    wifi_config.ap.ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID);
    strcpy(reinterpret_cast<char*>(wifi_config.ap.password), EXAMPLE_ESP_WIFI_PASS);
    wifi_config.ap.max_connection = EXAMPLE_MAX_STA_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) 
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config)); // ESP_IF_WIFI_AP => WIFI_IF_AP
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}


static void http_server_netconn_serve(struct netconn *conn)
{
    struct netbuf *inbuf;
    err_t err;
    err = netconn_recv(conn, &inbuf);
    if (err == ERR_OK) 
    {
        char *output = strstr(http_index_html_server, "#");
//        static int count = 0;
        sprintf(message, "Vrms = %07.2f", phase_a.voltage.rms_previous); //sprintf(message, "Vrms = %d", count++);
        strcpy(output + 1, message);
        output[strlen(message) + 1] = ';';
        output = strstr(output + 1, "#");
        sprintf(message, "Vrms = %07.2f", phase_a.current.rms_previous);
		strcpy(output + 1, message);
		output[strlen(message) + 1] = '.';

        netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);
        netconn_write(conn, http_index_html_server, sizeof(http_index_html_server)-1, NETCONN_NOCOPY);
    }
    netconn_close(conn);
    netbuf_delete(inbuf);
}

static void http_server(void *pvkeys)
{
    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);

    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) 
        {
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
        }
    } while(err == ERR_OK);

    netconn_close(conn);
    netconn_delete(conn);
}

void http_server_setup()
{
	ESP_ERROR_CHECK(init_nvs()); printf("\ninit_nvs\n");

    wifi_init_softap();
    xTaskCreate(&http_server, "http_server", 2048, NULL, 5, NULL);
}
