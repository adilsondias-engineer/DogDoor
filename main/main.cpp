#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"
#include "driver/gpio.h"
#include <esp_sntp.h>

#include "DendoStepper.h"
#include "main.h"
#include <ultrasonic.h>

#include "utils.h"
#include "application.h"
#include "mqtt.h"
#include "http.h"

// #include <sys/time.h>
#include "esp_event.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "protocol_examples_common.h"
#include "string.h"
// #ifdef CONFIG_EXAMPLE_USE_CERT_BUNDLE
//mbedtls esp-tls
//#include "esp_crt_bundle.h"
// #endif

#include <sys/socket.h>
#if CONFIG_EXAMPLE_CONNECT_WIFI
#include "esp_wifi.h"
#endif

#define MOTOR_STEP_PIN GPIO_NUM_13
#define MOTOR_DIR_PIN GPIO_NUM_14
#define MOTOR_ENABLE_PIN GPIO_NUM_15

#define INSIDE_MOV_SENSOR_PIN GPIO_NUM_18
#define OUTSIDE_MOV_SENSOR_PIN GPIO_NUM_19
#define CLOSED_LIMIT_PIN GPIO_NUM_32
#define OPEN_LIMIT_PIN GPIO_NUM_33

#define SENSOR_TRIGGER_PIN GPIO_NUM_5
#define SENSOR_ECHO_PIN GPIO_NUM_23

#define MAX_DISTANCE_CM 50 // 0.5m(50cm),1m(100), 5m(500) max
// #define DOG_APP_TAG "Dog App"
// #define TAG "Dog App"
#define MQTT_STATUS_TOPIC MQTT_TOPIC "/status"

httpd_handle_t http_server;

static const char *DOG_APP_TAG = "Dog App";
static const char *TAG = "Dog App";

char* triggeredSensor = (char*) "NONE";

DendoStepper step;
// DendoStepper step1;

enum doorStatus
{
    OPEN,
    OPENING,
    CLOSING,
    FORCECLOSING,
    CLOSED
};

std::unique_ptr<nvs::NVSHandle> handle = NULL;
esp_err_t err = NULL;
int currentDoorStatus = CLOSED;
int recoveredState = 0;
bool closedDoorLimitTriggered = false;
bool openDoorLimitTriggered = false;
bool movimentDetected = false;
bool wrongdir = false;
bool speedSet = false;
bool somethingWithinRange = false;
float detectedDistance = 0;

#define MOTOR_STEP_ANGLE 1.8
#define STEPS_PER_MM 800 // 266.67// 400 // calculator  https://zalophusdokdo.github.io/StepperMotorsCalculator/en/index.html
// GT2 pulley 20th 1.8 angle belt 2mm
#define MOVE_POSITION_MM 350

#define MOTOR_OPENING_SPEED 20000  //more tan 18500 cause lots of noises and 25000 doesn't work
#define MOTOR_OPENING_ACCELARATION 500
#define MOTOR_OPENING_DECELARATION 50

#define MOTOR_CLOSING_SPEED 10000
#define MOTOR_CLOSING_ACCELARATION 500
#define MOTOR_CLOSING_DECELARATION 50

static void _ntp_set_time_task(); // void *pvParameter
void _ntp_time_sync_notification_cb(struct timeval *tv);

void _ntp_set_time_task() // void *pvParameter
{
    // while (true)
    //{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "0.au.pool.ntp.org");
    esp_sntp_setservername(1, "1.au.pool.ntp.org");
    esp_sntp_setservername(2, "2.au.pool.ntp.org");
    esp_sntp_setservername(3, "3.au.pool.ntp.org");
    sntp_set_time_sync_notification_cb(_ntp_time_sync_notification_cb);
    esp_sntp_init();

    // printf("Date/Time updated at: %s\n", get_system_time_str());
    // vTaskDelay(pdMS_TO_TICKS(300000)); // 1s
    // }

    /* A task should NEVER return */
    // vTaskDelete(NULL);
}

void _ntp_time_sync_notification_cb(struct timeval *tv)
{
    printf("Date/Time updated at: %s\n", get_system_time_str());
}

//-----------------------------------------------------------------------------
extern "C" void openDoor_http_handle()
{
    currentDoorStatus = OPENING;
}

extern "C" char* getSensorTriggered(){
    return triggeredSensor;
}

extern "C" bool isMovementDetected(){
    return movimentDetected;
}

extern "C" void closeDoor_http_handle()
{
    currentDoorStatus = FORCECLOSING;
}

extern "C" float getDetectedDistance(){
    return detectedDistance;
}
extern "C" const char *getDoorStatus()
{
    const char *status = "NOT DEFINED";
    switch (currentDoorStatus)
    {
    case OPEN:
        status = "OPEN";
        break;
    case CLOSED:
        status = "CLOSED";
        break;
    case OPENING:
        status = "OPENING";
        break;
    case CLOSING:
        status = "CLOSING";
        break;
    }
    return status;
}
static void Check_Sensor_Task(void *pvParameter);
static void Handle_Door_Task(void *pvParameter);
static void Check_Door_Closed_Task(void *pvParameter);
static void Check_Door_Opened_Task(void *pvParameter);
static void Ultrasonic_Check(void *pvParameters);
/*
#define HASH_LEN 32

#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_BIND_IF
/ The interface name value can refer to if_desc in esp_netif_defaults.h /
#if CONFIG_EXAMPLE_FIRMWARE_UPGRADE_BIND_IF_ETH
static const char *bind_interface_name = EXAMPLE_NETIF_DESC_ETH;
#elif CONFIG_EXAMPLE_FIRMWARE_UPGRADE_BIND_IF_STA
static const char *bind_interface_name = EXAMPLE_NETIF_DESC_STA;
#endif
#endif

//static const char *TAG = "simple_ota_example";
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

#define OTA_URL_SIZE 256

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
        break;
    }
    return ESP_OK;
}*/
/*
void simple_ota_example_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Starting OTA example task");
#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_BIND_IF
    esp_netif_t *netif = get_example_netif_from_desc(bind_interface_name);
    if (netif == NULL) {
        ESP_LOGE(TAG, "Can't find netif from interface description");
        abort();
    }
    struct ifreq ifr;
    esp_netif_get_netif_impl_name(netif, ifr.ifr_name);
    ESP_LOGI(TAG, "Bind interface name is %s", ifr.ifr_name);
#endif
    esp_http_client_config_t config = {
        .url = CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL,
        .event_handler = _http_event_handler,
#ifdef CONFIG_EXAMPLE_USE_CERT_BUNDLE
        .crt_bundle_attach = esp_crt_bundle_attach,
#else
        .cert_pem = (char *)server_cert_pem_start,
#endif / CONFIG_EXAMPLE_USE_CERT_BUNDLE /

        .keep_alive_enable = true,
#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_BIND_IF
        .if_name = &ifr,
#endif
    };

#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL_FROM_STDIN
    char url_buf[OTA_URL_SIZE];
    if (strcmp(config.url, "FROM_STDIN") == 0) {
        example_configure_stdin_stdout();
        fgets(url_buf, OTA_URL_SIZE, stdin);
        int len = strlen(url_buf);
        url_buf[len - 1] = '\0';
        config.url = url_buf;
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong firmware upgrade image url");
        abort();
    }
#endif

#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
    config.skip_cert_common_name_check = true;
#endif

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };
    ESP_LOGI(TAG, "Attempting to download update from %s", config.url);
    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed");
    }
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i) {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s %s", label, hash_print);
}

static void get_sha256_of_partitions(void)
{
    uint8_t sha_256[HASH_LEN] = { 0 };
    esp_partition_t partition;

    // get sha256 digest for bootloader
    partition.address   = ESP_BOOTLOADER_OFFSET;
    partition.size      = ESP_PARTITION_TABLE_OFFSET;
    partition.type      = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    // get sha256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");
}
*/

void Ultrasonic_Check(void *pvParameters)
{

   
ultrasonic_sensor_t sensor = {
        .trigger_pin = SENSOR_TRIGGER_PIN,
        .echo_pin = SENSOR_ECHO_PIN
    };

    ultrasonic_init(&sensor);

    while (true)
    {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            //printf("Error %d: ", res);
            /*switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }*/
        }
        else{
           // printf("Distance: %0.04f cm\n", distance*100);
            if( distance*100 < MAX_DISTANCE_CM){
              //  printf("Distance: %0.04f cm\n", distance*100);
                detectedDistance = distance*100;
                somethingWithinRange = true;
            }else{
                somethingWithinRange = false;
                detectedDistance = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Check_Sensor_Task(void *params)
{
    int currentState = 0;
    //  int count = 0;

    //  int data[2];
    while (true)
    {
        currentState = gpio_get_level(INSIDE_MOV_SENSOR_PIN);

        if (currentState == 1 )//&& somethingWithinRange)

        {
            triggeredSensor = (char*)"Indoor";
            movimentDetected = true;
        }
        else
        {
            currentState = gpio_get_level(OUTSIDE_MOV_SENSOR_PIN);
            if (currentState == 1)

            {

                triggeredSensor = (char*)"Outdoor";
                movimentDetected = true;
                
            }
            else
            { 
                triggeredSensor = (char*) "NONE";
                movimentDetected = false;
                
            }
        }
        //ESP_LOGI(DOG_APP_TAG, "Check_Sensor_Task movimentDetected %d  currentDoorStatus   %d\n", movimentDetected, currentDoorStatus);
        if (recoveredState == 1)
        {
            vTaskDelay(pdMS_TO_TICKS(3000)); // 3s
            recoveredState = 0;
        }
        else
            // ESP_LOGI(DOG_APP_TAG, "GPIO %d was pressed %d times. The value is %d\n", MOV_SENSOR_PIN, count, currentState);
            if (currentState == 1 /*&& somethingWithinRange*/ && currentDoorStatus == CLOSED && recoveredState == 0) // if the door is closed/idle and there is something next to it, open it
            {
                // count++;
                // printf("GPIO %d was pressed %d times. The value is %d\n", MOV_SENSOR_PIN, count, currentState);
                ESP_LOGI(DOG_APP_TAG, "Check_Sensor_Task Moviment Detected  %d\n", currentState);
                currentDoorStatus = OPENING;

                // Write

                // ESP_LOGI(DOG_APP_TAG,"Updating doorStatus in NVS ... ");
                /*
                {
                   err = handle->set_item("Check_Sensor_Task currentDoorStatus", currentDoorStatus);
                    ESP_LOGI(DOG_APP_TAG, "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");

                    // Commit written value.
                    // After setting any values, nvs_commit() must be called to ensure changes are written
                    // to flash storage. Implementations may write to storage at other times,
                    // but this is not guaranteed.
                    ESP_LOGI(DOG_APP_TAG, "Committing updates in NVS ... ");
                    err = handle->commit();
                    ESP_LOGI(DOG_APP_TAG, "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
                }
                */
                ESP_LOGI(DOG_APP_TAG, "Check_Sensor_Task Sleeping 1s\n");
                vTaskDelay(pdMS_TO_TICKS(1000)); // 1s
            }
            else if (currentDoorStatus == OPEN && currentState == 0) // renable  don't close the door if there is anything around //currentState == 0 &&
            {
                /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
                currentDoorStatus = CLOSING;
                // ESP_LOGI(DOG_APP_TAG,"Updating doorOpen in NVS ... ");
                /*
                                if (handle != NULL)
                                {
                                    err = handle->set_item("Check_Sensor_Task currentDoorStatus", currentDoorStatus);
                                    ESP_LOGI(DOG_APP_TAG, "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");

                                    // Commit written value.
                                    // After setting any values, nvs_commit() must be called to ensure changes are written
                                    // to flash storage. Implementations may write to storage at other times,
                                    // but this is not guaranteed.
                                    ESP_LOGI(DOG_APP_TAG, "Committing updates in NVS ... ");
                                    err = handle->commit();
                                    ESP_LOGI(DOG_APP_TAG, "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
                                }*/
                vTaskDelay(pdMS_TO_TICKS(500)); //0.5s
            }

        vTaskDelay(pdMS_TO_TICKS(500)); // 0.5s
    } // for

    /* A task should NEVER return */
    vTaskDelete(NULL);
}

void Check_Door_Closed_Task(void *params)
{

    while (true)
    {
        int currentState = gpio_get_level(CLOSED_LIMIT_PIN);
        // ESP_LOGI(DOG_APP_TAG, "DOOR is closed %d\n", currentState);
        if (currentState == 0)
        {

           // ESP_LOGI(DOG_APP_TAG, "DOOR is closed %d\n", currentState);
            closedDoorLimitTriggered = true;
            // openDoorLimitTriggered = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms
    }

    /* A task should NEVER return */
    vTaskDelete(NULL);
}

void Check_Door_Opened_Task(void *params)
{

    while (true)
    {
        int currentState = gpio_get_level(OPEN_LIMIT_PIN);
        //ESP_LOGI(DOG_APP_TAG, "DOOR is open %d\n", currentState);
        if (currentState == 0)
        {
         //  ESP_LOGI(DOG_APP_TAG, "openDoorLimitTriggered DOOR is open %d\n", currentState);
            openDoorLimitTriggered = true;
            //  closedDoorLimitTriggered = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms
    }

    /* A task should NEVER return */
    vTaskDelete(NULL);
}

void Handle_Door_Task(void *params)
{

    // step1.setSpeed(1000, 100, 100);

    // step.runInf(true);
    // ESP_LOGI(DOG_APP_TAG,"Current position %f\n", (float)step.getPositionMm());

    //    step.resetAbsolute();
    step.setStepsPerMm(STEPS_PER_MM);
    // step1.setStepsPerMm(STEPS_PER_MM);
    // ESP_LOGI(DOG_APP_TAG, "Current position 1 %f\n", (float)step.getPositionMm());
    // ESP_LOGI(DOG_APP_TAG, "Current position 2 %f\n", (float)step1.getPositionMm());
    // step.runPosMm(-1000);

    //  int data[2];
    while (true)
    {

        //   ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task currentDoorStatus %d\n", currentDoorStatus);
        if (currentDoorStatus == OPENING) // open the door
        {

            if (speedSet == false)
            {
                step.setSpeed(MOTOR_OPENING_SPEED, MOTOR_OPENING_ACCELARATION, MOTOR_OPENING_DECELARATION);
                speedSet = true;
            }
            ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task OPENING currentDoorStatus %d\n", currentDoorStatus);
            // step.setDir(true);
            step.enableMotor();
            //step.disableMotor();
            // step1.setDir(true);
            // step1.enableMotor();
            ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task Motor enabled\n");
            if (step.runPosMm(MOVE_POSITION_MM, true) <= 0) //&& step1.runPosMm(MOVE_POSITION_MM) <= 0)
            {
                // ESP_LOGI(DOG_APP_TAG, "Current position %f\n", (float)step.getPositionMm());
                // ESP_LOGI(DOG_APP_TAG, "Current direction - opening %d\n", step.getDir());
                //  ESP_LOGI(DOG_APP_TAG, "Current position %f\n", (float)step1.getPositionMm());
                // step.stop();
            }
            else
            {
                ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task Error");
                // step.disableMotor();
                // step1.disableMotor();
            }
            // ESP_LOGI(DOG_APP_TAG, "Checking State\n");
            while (step.getState() != 1) // && step1.getState() != 1 /*IDLE*/)
            {
                // ESP_LOGI(DOG_APP_TAG,"Current position %f\n", (float)step.getPositionMm());
                // ESP_LOGI(DOG_APP_TAG, "Current direction - opening %d\n", step.getDir());
                if (openDoorLimitTriggered)
                {
                    // openDoorLimitTriggered = true;
                    // closedDoorLimitTriggered = false;
                    ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task Open Limit triggered stopping %d\n", openDoorLimitTriggered);
                    step.stop();
                    closedDoorLimitTriggered = false;
                    currentDoorStatus = OPEN;
                    speedSet = false;
                    wrongdir = false;
                }
                /*
                if (closedDoorLimitTriggered && wrongdir == false) // wrong direction
                {
                    // openDoorLimitTriggered = true;
                    // closedDoorLimitTriggered = false;
                    ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task Close Limit triggered, changing direction %d\n", closedDoorLimitTriggered);

                    //step.stop();
                    ESP_LOGI(DOG_APP_TAG, ".... changing direction\n");
                    // step.runPosMm(MOVE_POSITION_MM * -1, false);
                    if (step.runPosMm((MOVE_POSITION_MM), true) <= 0) //&& step1.runPosMm(MOVE_POSITION_MM) <= 0)
                    {
                        // ESP_LOGI(DOG_APP_TAG, "Current position %f\n", (float)step.getPositionMm());
                        // ESP_LOGI(DOG_APP_TAG, "Current direction - opening %d\n", step.getDir());
                        //  ESP_LOGI(DOG_APP_TAG, "Current position %f\n", (float)step1.getPositionMm());
                        // step.stop();
                    }
                    else
                    {
                        ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task Error");
                        // step.disableMotor();
                        // step1.disableMotor();
                    }
                    openDoorLimitTriggered = false;
                    wrongdir = true;
                }*/
                vTaskDelay(pdMS_TO_TICKS(10)); // 1sec
            }

            if (currentDoorStatus == OPEN)
            {
                step.stop();
               // step.disableMotor();
            }
            // currentDoorStatus = OPEN;
            //  step.stop();
            //  ESP_LOGI(DOG_APP_TAG, "Disabling Motor\n");
            //  step.disableMotor();
            //  step1.disableMotor();
            //  doorOpen = 2;
            //  ESP_LOGI(DOG_APP_TAG, "Updating doorOpen in NVS ... ");
            /*
                        if (handle != NULL)
                        {
                            err = handle->set_item("Handle_Door_Task currentDoorStatus", currentDoorStatus);
                            ESP_LOGI(DOG_APP_TAG, "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");

                            // Commit written value.
                            // After setting any values, nvs_commit() must be called to ensure changes are written
                            // to flash storage. Implementations may write to storage at other times,
                            // but this is not guaranteed.
                            // ESP_LOGI(DOG_APP_TAG, "Committing updates in NVS ... ");
                            err = handle->commit();
                            ESP_LOGI(DOG_APP_TAG, "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
                        }*/
            ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task expected OPEN=0 currentDoorStatus %d\n", currentDoorStatus);
            ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task Door Open\n");
        }
        else if (currentDoorStatus == CLOSING || currentDoorStatus == FORCECLOSING)
        {

            if (speedSet == false)
            {
                step.setSpeed(MOTOR_CLOSING_SPEED, MOTOR_CLOSING_ACCELARATION, MOTOR_CLOSING_DECELARATION);
                speedSet = true;
            }

            ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task  expected CLOSING=2 currentDoorStatus %d\n", currentDoorStatus);
            /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
            // vTaskDelay(pdMS_TO_TICKS(5000)); // 5s
            // step.setDir(false);
            step.enableMotor();
            //  step1.setDir(false);
            //  step1.enableMotor();
            if (step.runPosMm(MOVE_POSITION_MM * -1, false) <= 0) // && step1.runPosMm(MOVE_POSITION_MM * -1) <= 0)
            {
                // ESP_LOGI(DOG_APP_TAG, "Current position %f\n", (float)step.getPositionMm());
                // ESP_LOGI(DOG_APP_TAG, "Current direction - closing %d\n", step.getDir());

                //  ESP_LOGI(DOG_APP_TAG, "Current position %f\n", (float)step1.getPositionMm());
                // step.stop();
            }
            else
            {
                ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task Error");
                //  step.disableMotor();
                //  step1.disableMotor();
            }
            while (step.getState() != 1) // && step1.getState() != 1 /*IDLE*/)
            {
                // ESP_LOGI(DOG_APP_TAG,"Current position %f\n", (float)step.getPositionMm());
                // ESP_LOGI(DOG_APP_TAG, "Current direction - closing %d\n", step.getDir());
                if (closedDoorLimitTriggered && currentDoorStatus == CLOSING)
                {
                    // openDoorLimitTriggered = true;
                    // closedDoorLimitTriggered = false;
                    ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task Close Limit triggered stopping %d\n", closedDoorLimitTriggered);
                    step.stop();
                    openDoorLimitTriggered = false;
                    currentDoorStatus = CLOSED;
                    wrongdir = false;
                    speedSet = false;
                }
                /*
                if (openDoorLimitTriggered && wrongdir == false) // wrong direction
                {
                    // openDoorLimitTriggered = true;
                    // closedDoorLimitTriggered = false;
                    ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task Open Limit triggered, changing direction %d\n", openDoorLimitTriggered);
                   // step.stop();
                    ESP_LOGI(DOG_APP_TAG, ".... changing direction\n");
                    // step.runPosMm(MOVE_POSITION_MM, true);
                    if (step.runPosMm(MOVE_POSITION_MM, true) <= 0) // && step1.runPosMm(MOVE_POSITION_MM * -1) <= 0)
                    {
                        // ESP_LOG  I(DOG_APP_TAG, "Current position %f\n", (float)step.getPositionMm());
                        // ESP_LOGI(DOG_APP_TAG, "Current direction - closing %d\n", step.getDir());

                        //  ESP_LOGI(DOG_APP_TAG, "Current position %f\n", (float)step1.getPositionMm());
                        // step.stop();
                    }
                    else
                    {
                        ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task Error");
                        //  step.disableMotor();
                        //  step1.disableMotor();
                    }
                    closedDoorLimitTriggered = false;
                    wrongdir = true;
                }*/

                if (movimentDetected && currentDoorStatus == CLOSING)
                {
                    movimentDetected = false;
                    openDoorLimitTriggered = false;
                    currentDoorStatus = OPENING;
                    ESP_LOGI(DOG_APP_TAG, "Handle_Door_Task Moviment detected while closing -  reverting \n");
                    step.stop();
                    speedSet = false;
                }
                vTaskDelay(pdMS_TO_TICKS(10)); // 10ms
            }
            if (currentDoorStatus == CLOSED)
            {
                step.disableMotor();
            }
            // if (currentDoorStatus == CLOSING)
            // {
            //     currentDoorStatus = CLOSED;
            // }
            // step.stop();
            // step1.disableMotor();
            // doorOpen = 0;
            ESP_LOGI(DOG_APP_TAG, ".. expected CLOSED=3 currentDoorStatus %d\n", currentDoorStatus);
            ESP_LOGI(DOG_APP_TAG, "Door Closed\n");
            //  ESP_LOGI(DOG_APP_TAG, "Updating doorOpen in NVS ... ");
            /*
            if (handle != NULL)
            {
                err = handle->set_item("Handle_Door_Task currentDoorStatus", currentDoorStatus);

                ESP_LOGI(DOG_APP_TAG, "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
                // Commit written value.
                // After setting any values, nvs_commit() must be called to ensure changes are written
                // to flash storage. Implementations may write to storage at other times,
                // but this is not guaranteed.
                // ESP_LOGI(DOG_APP_TAG, "Committing updates in NVS ... ");
                err = handle->commit();
                ESP_LOGI(DOG_APP_TAG, "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
            }*/
        }
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10)); // 1s
    } // for

    /* A task should NEVER return */
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{

    // AEST-10AEDT,M10.1.0,M4.1.0/3
    setenv("TZ", "AEST-10AEDT,M10.1.0,M4.1.0/3", 1);
    tzset();

    ESP_LOGI(DOG_APP_TAG, "Starting ... \n");
    esp_rom_gpio_pad_select_gpio(INSIDE_MOV_SENSOR_PIN);
    gpio_set_direction(INSIDE_MOV_SENSOR_PIN, GPIO_MODE_INPUT);

    esp_rom_gpio_pad_select_gpio(OUTSIDE_MOV_SENSOR_PIN);
    gpio_set_direction(OUTSIDE_MOV_SENSOR_PIN, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(MOV_SENSOR_PIN, GPIO_PULLUP_ONLY);

    esp_rom_gpio_pad_select_gpio(OPEN_LIMIT_PIN);
    gpio_set_direction(OPEN_LIMIT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(OPEN_LIMIT_PIN, GPIO_PULLUP_ONLY);

    esp_rom_gpio_pad_select_gpio(CLOSED_LIMIT_PIN);
    gpio_set_direction(CLOSED_LIMIT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(CLOSED_LIMIT_PIN, GPIO_PULLUP_ONLY);

    esp_rom_gpio_pad_select_gpio(MOTOR_STEP_PIN);
    gpio_set_direction(MOTOR_STEP_PIN, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(MOTOR_DIR_PIN);
    gpio_set_direction(MOTOR_DIR_PIN, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(MOTOR_ENABLE_PIN);
    gpio_set_direction(MOTOR_ENABLE_PIN, GPIO_MODE_OUTPUT);


 


    ESP_LOGI(DOG_APP_TAG, "... Starting OTA\n");
    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // 1.OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // 2.NVS partition contains data in new format and cannot be recognized by this version of code.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // get_sha256_of_partitions();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

#if CONFIG_EXAMPLE_CONNECT_WIFI
    /* Ensure to disable any WiFi power save mode, this allows best throughput
     * and hence timings for overall OTA operation.
     */
    esp_wifi_set_ps(WIFI_PS_NONE);
#endif // CONFIG_EXAMPLE_CONNECT_WIFI

    //  xTaskCreate(&simple_ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);

    // xTaskCreate(_ntp_set_time_task, "_ntp_set_time_task", 2048, NULL, 1, NULL);
    _ntp_set_time_task();
    http_init();
    http_start_webserver(&http_server);
    mqtt_init();
    application_init();

    /*
        //  Initialize NVS
        err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            // NVS partition was truncated and needs to be erased
            // Retry nvs_flash_init
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);

        // Open
        ESP_LOGI(DOG_APP_TAG, "\n");
        ESP_LOGI(DOG_APP_TAG, "Opening Non-Volatile Storage (NVS) handle... ");
        // Handle will automatically close when going out of scope or when it's reset.
        handle = nvs::open_nvs_handle("storage", NVS_READWRITE, &err);
        if (err != ESP_OK)
        {
            ESP_LOGE(DOG_APP_TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(DOG_APP_TAG, "Done\n");
    */
    // Read
    ESP_LOGI(DOG_APP_TAG, "Checking current door state ...\n");
    // int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
    // err = handle->get_item("currentDoorStatus", currentDoorStatus);
    if (openDoorLimitTriggered)
    {
        currentDoorStatus = OPEN;
    }
    else if (closedDoorLimitTriggered)
    {
        currentDoorStatus = CLOSED;
    }
    else
    {
        currentDoorStatus = OPENING; // force opening if stuck in the middle
    }
    ESP_LOGI(DOG_APP_TAG, "Current door state : %d\n", currentDoorStatus);
    /*
     switch (err)
     {
     case ESP_OK:
         ESP_LOGI(DOG_APP_TAG, "Done\n");
         ESP_LOGI(DOG_APP_TAG, "currentDoorStatus = %d\n", currentDoorStatus);
         recoveredState = 1;
         break;
     case ESP_ERR_NVS_NOT_FOUND:
         ESP_LOGE(DOG_APP_TAG, "The value is not initialized yet!\n");
         break;
     default:
         ESP_LOGE(DOG_APP_TAG, "Error (%s) reading!\n", esp_err_to_name(err));
     }*/

    /*

        esp_chip_info_t chip_info;
        uint32_t flash_size;
        esp_chip_info(&chip_info);
        printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
               CONFIG_IDF_TARGET,
               chip_info.cores,
               (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
               (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
               (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
               (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

        unsigned major_rev = chip_info.revision / 100;
        unsigned minor_rev = chip_info.revision % 100;
        printf("silicon revision v%d.%d, ", major_rev, minor_rev);
        if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
            printf("Get flash size failed");
            return;
        }

        printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
               (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

        printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

       for (int i = 10; i >= 0; i--) {
            printf("Restarting in %d seconds...\n", i);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }*/
    // printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();

    ESP_LOGI(DOG_APP_TAG, "Sleeping for 2 seconds before enabling motor \n");
    vTaskDelay(pdMS_TO_TICKS(2000)); // 2s

    DendoStepper_config_t step_cfg = {
        .stepPin = (uint8_t)MOTOR_STEP_PIN,
        .dirPin = (uint8_t)MOTOR_DIR_PIN,
        .enPin = (uint8_t)MOTOR_ENABLE_PIN,
        .miStep = MICROSTEP_16,
        .stepAngle = MOTOR_STEP_ANGLE}; // 1.8

    /*      DendoStepper_config_t step1_cfg = {
            .stepPin = 18,
             .dirPin = 19,
            .enPin = 21,
           .miStep = MICROSTEP_1,
           .stepAngle = 1.8};
    */
    step.config(&step_cfg);
    //   step1.config(&step1_cfg);

    step.init();
    //   step1.init();
    /*
    int position = 0;
    int dir = 1;
        if(step.runPosMm(130) <=0 ){
            ESP_LOGI(DOG_APP_TAG,"Current position %f\n", (float)step.getPositionMm());
            //step.stop();
        }else{
            ESP_LOGI(DOG_APP_TAG,"Error");
            step.disableMotor();
        }
       while(step.getState() != 1 /IDLE/) {
         //ESP_LOGI(DOG_APP_TAG,"Current position %f\n", (float)step.getPositionMm());
       }
       //step.stop();
       step.disableMotor();*/
    /* while(1){
        //step1.runPos(10000);
        ESP_LOGI(DOG_APP_TAG,"Current position %d\n", position);
        // step.runAbs(5000);
        if(position <=0) {
          step.setDir(false);
          step.runPosMm(10);
          position+=10;
          dir=1;
        }else if( position >=150){
         step.setDir(true);
         step.runPosMm(-10);
         //step.stop();
          position-=10;
          dir=-1;
         //break;
        }else{
         step.runPosMm(10*dir);
         position+=10*dir;
        }
        //step.runPosMm(-100);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

     }*/

    // step.runPosMm(100);

    // step.stop();
    ESP_LOGI(DOG_APP_TAG, "Creating tasks .... \n");

    xTaskCreatePinnedToCore(Check_Door_Opened_Task, "Check_Door_Opened_Task", 2048, NULL, 1, NULL,1);
    xTaskCreatePinnedToCore(Check_Door_Closed_Task, "Check_Door_Closed_Task", 2048, NULL, 1, NULL,1);
    xTaskCreatePinnedToCore(Check_Sensor_Task, "Check_Sensor_Task", 3072, NULL, 1, NULL,1);
 //   xTaskCreate(Ultrasonic_Check, "Ultrasonic_Check", 2048, NULL, 1, NULL);

    xTaskCreatePinnedToCore(Handle_Door_Task, "Handle_Door_Task", 22384, NULL, 1, NULL,1);

    gpio_dump_io_configuration(stdout, (1ULL << INSIDE_MOV_SENSOR_PIN) | (1ULL << OUTSIDE_MOV_SENSOR_PIN) | (1ULL << CLOSED_LIMIT_PIN) | (1ULL << OPEN_LIMIT_PIN));

    ESP_LOGI(DOG_APP_TAG, "Running ... \n");

    /// ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
}
