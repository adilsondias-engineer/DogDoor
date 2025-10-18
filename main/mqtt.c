#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include <esp_event.h>
#include <esp_system.h>
#include <esp_app_format.h>
#include <esp_netif.h>
#include <esp_eth.h>
#include <mqtt_client.h>
#include <esp_mac.h>

#include "utils.h"
#include "application.h"
#include  "main.h"

#define MQTT_TOPIC "dogdoor"
#define MQTT_IP_TOPIC MQTT_TOPIC "/ip"
#define MQTT_STATUS_TOPIC MQTT_TOPIC "/status"
#define MQTT_REQUEST_TOPIC MQTT_TOPIC "/request"
#define MQTT_SERVER_IP "192.168.1.2"//"mqtt.local"

typedef struct
{
    char topic[64];
    char data[256];
    int16_t data_len;
} mqtt_msg_t;

//-----------------------------------------------------------------------------
static bool s_mqtt_subscribed = false;
static bool s_force_mqtt_updates = false;
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static QueueHandle_t s_tx_queue;
static bool s_initialized = false;
static bool s_connected = false;

esp_netif_t *wifiInterface = NULL;

void mqtt_do_work();
static void _mqtt_task(void *Param);

//-----------------------------------------------------------------------------
static void _mqtt_task(void *Param)
{ 
  const uint32_t task_delay_ms = 250;

  while(1)
  {
    delay_ms(task_delay_ms);
    mqtt_do_work();
  }  
}


//-----------------------------------------------------------------------------
void force_mqtt_update()
{
    s_force_mqtt_updates = true;
}

//-----------------------------------------------------------------------------
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        printf("MQTT Connected to server\n");
        s_connected = true;
        esp_mqtt_client_subscribe(client, MQTT_REQUEST_TOPIC, 0);
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_subscribed = false;
        s_connected = false;
        printf("MQTT Server Disconnect!\n");
        break;

    case MQTT_EVENT_DATA:
        event->data[event->data_len] = 0;
        application_handle_mqtt_request_msg(event->data);
        break;

    case MQTT_EVENT_ERROR:
        printf("MQTT Event Error!\n");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        s_mqtt_subscribed = true;
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        s_mqtt_subscribed = false;
        break;

    case MQTT_EVENT_PUBLISHED:
    default:
        break;
    }
    fflush(stdout);
}

//-----------------------------------------------------------------------------
void mqtt_init()
{

    wifiInterface = esp_netif_get_default_netif();

    s_tx_queue = xQueueCreate(20, sizeof(mqtt_msg_t));

    uint8_t mac[8];
    char client_id[64];
    esp_base_mac_addr_get(mac);
    sprintf(client_id, "pub-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://" MQTT_SERVER_IP,
        .broker.address.port = 1883,
        .credentials.username="dogdoor",
        .credentials.authentication.password="dogdoor",
        .credentials.client_id = client_id};


    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);

    s_initialized = true;

    xTaskCreate( _mqtt_task, "mqtt_task", 8192, NULL, 5, NULL);
}

//-----------------------------------------------------------------------------
void mqtt_write_topic(char *topic, uint8_t *p_data, size_t data_len)
{
    mqtt_msg_t msg;
    snprintf(msg.topic, sizeof(msg.topic), "%s/%s", MQTT_TOPIC, topic);
    memcpy(msg.data, p_data, data_len);
    msg.data_len = data_len;

    if (s_initialized && s_connected)
        xQueueSend(s_tx_queue, &msg, pdMS_TO_TICKS(100));
}

//-----------------------------------------------------------------------------
// Run on the wifi task
void mqtt_do_work()
{
    static float last_status_update = 0;
    static float last_ip_update = 0;

    if (!s_mqtt_client || !s_connected)
    {
        return;
    }

    if (s_force_mqtt_updates)
    {
        last_status_update = 0;
        last_ip_update = 0;
        s_force_mqtt_updates = false;
    }

    uint32_t status_update_rate_s = 5;
    if ((last_status_update == 0) || ((system_uptime_s() - last_status_update) > status_update_rate_s))
    {
        esp_mqtt_client_publish(s_mqtt_client, MQTT_STATUS_TOPIC, application_get_mqtt_status_msg(), 0, 1, 0);
        last_status_update = system_uptime_s();
    }

    if ((last_ip_update == 0) || ((system_uptime_s() - last_ip_update) > 60))
    {
        char ip_msg[128];
        char *p_msg = ip_msg;
        esp_netif_ip_info_t ip_info;

        if (wifiInterface)
        {

            if (esp_netif_is_netif_up(wifiInterface))
            {

               // esp_net
                esp_netif_get_ip_info(wifiInterface, &ip_info);
                //  printf(LOG_COLOR(LOG_COLOR_BLUE) "IP address: " LOG_COLOR(LOG_COLOR_GREEN) IPSTR "\n", IP2STR(&ip_info.ip));
                //   printf(LOG_COLOR(LOG_COLOR_BLUE) "netmask: " LOG_COLOR(LOG_COLOR_GREEN) IPSTR "\n", IP2STR(&ip_info.netmask));
                //  printf(LOG_COLOR(LOG_COLOR_BLUE) "gateway: " LOG_COLOR(LOG_COLOR_GREEN) IPSTR "\n", IP2STR(&ip_info.gw));
            }
            //"{ \"ip_addr\":\"%s\",", wifi_get_ip_addr_str()
          
            p_msg += sprintf(p_msg, "{ \"ip_addr\":\"%d.%d.%d.%d\",", IP2STR(&ip_info.ip));
            p_msg += sprintf(p_msg, "\"mdns_name\":\"%s\",", "Dog Door" /*wifi_get_mdns_name_str()*/);
            // ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
            p_msg += sprintf(p_msg, "\"free_memory\":\"%" PRIu32 " bytes\"", esp_get_free_heap_size() );
            p_msg += sprintf(p_msg, "}");
        }
        else
        {
            p_msg += sprintf(p_msg, "{ \"ip_addr\":\"%s\",", "NO IP" );
            p_msg += sprintf(p_msg, "\"mdns_name\":\"%s\",", "Dog Door" /*wifi_get_mdns_name_str()*/);
            p_msg += sprintf(p_msg, "\"free_memory\":\"%" PRIu32 " bytes\"", esp_get_free_heap_size() );
            p_msg += sprintf(p_msg, "}");
        }
        printf("..mqtt publishing stats");
        esp_mqtt_client_publish(s_mqtt_client, MQTT_IP_TOPIC, ip_msg, 0, 1, 0);
        last_ip_update = system_uptime_s();
    }

    mqtt_msg_t msg;
    while (xQueueReceive(s_tx_queue, &msg, 0) == pdPASS)
    {
        printf("..mqtt publishing msg");
        esp_mqtt_client_publish(s_mqtt_client, msg.topic, msg.data, msg.data_len, 1, 0);
    }
}
