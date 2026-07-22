#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"

// ==============================================================================
// 1. DEFINITIONEN & STRUKTUREN
// ==============================================================================
#define WIFI_SSID           "hsa005"
#define WIFI_PASS           "hsa%2026"    
#define SKIN_IP             "192.168.4.1"
#define DATA_PORT_SKIN      17010
#define DATA_PORT_ESP       17011
#define CTRL_PORT_SKIN      17000

// Schwellenwerte für die LEDs
#define PROXIMITY_THRESHOLD_TOUCH   0.80f  
#define PROXIMITY_THRESHOLD_STOP    0.03f  
#define PROXIMITY_THRESHOLD_CLEAR   0.016f 

#define MAX_SENSORS 32 

static const char *TAG = "ROBOT_NODE";

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;
#define MAXIMUM_RETRY  5

typedef struct {
    uint8_t payload[20];
} e_skin_packet_t;

QueueHandle_t eskin_data_queue; 
int global_udp_sock = -1;
struct sockaddr_in skin_data_addr;

const uint8_t LOCK_CMD_PKT[]   = {0x72, 0x6C, 0x7B, 0xCB, 0xDC, 0xC0, 0x1A, 0x00};
const uint8_t START_CMD_PKT[]  = {0x72, 0x6C, 0x7B, 0xCB, 0xDC, 0xC0, 0x0A, 0x00};
const uint8_t UDR_63HZ_PKT[]   = {0x72, 0x6C, 0x7B, 0xCB, 0xDC, 0xA0, 0xD0, 63}; 
const uint8_t DUMMY_DATA_PKT[] = {0xFF, 0x7F, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA};

const uint8_t LED_GREEN_PKT[]  = {0xCA, 0x7F, 0x7F, 0x00, 0x00, 0x7F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA};
const uint8_t LED_RED_PKT[]    = {0xCA, 0x7F, 0x7F, 0x7F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA};
const uint8_t LED_BLUE_PKT[]   = {0xCA, 0x7F, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA};

// ==============================================================================
// 2. WI-FI INITIALISIERUNG
// ==============================================================================
/**
 * @brief Handles Wi-Fi station and IP events during connection setup.
 *
 * Inputs:
 * - arg: Optional user context supplied by ESP-IDF; unused by this handler.
 * - event_base: Event family used to distinguish Wi-Fi events from IP events.
 * - event_id: Specific event within the selected event family.
 * - event_data: Event-specific payload supplied by ESP-IDF; unused here.
 *
 * Outputs:
 * - Starts or retries the Wi-Fi connection when required.
 * - Sets WIFI_CONNECTED_BIT or WIFI_FAIL_BIT in s_wifi_event_group.
 * - Returns no value.
 */
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Initializes the ESP32-C6 as a Wi-Fi station and waits for connection setup to finish.
 *
 * Inputs:
 * - None; the station credentials and retry limit are read from compile-time constants.
 *
 * Outputs:
 * - Creates s_wifi_event_group and initializes the ESP-IDF network and Wi-Fi stack.
 * - Starts station mode and blocks until either a connection or failure is reported.
 * - Returns no value; initialization failures are handled by ESP_ERROR_CHECK.
 */
void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL);
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS }, };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

// ==============================================================================
// 3. DATENVERARBEITUNG
// ==============================================================================
/**
 * @brief Processes e-skin sensor packets and updates the serial proximity output and skin LEDs.
 *
 * Inputs:
 * - pvParameters: Optional FreeRTOS task context; unused by this task.
 * - Sensor packets are received continuously from eskin_data_queue.
 *
 * Outputs:
 * - Prints the highest measured proximity value at 4 Hz via the standard output UART.
 * - Sends green, red, or blue LED packets through global_udp_sock when thresholds are crossed.
 * - Does not return during normal operation.
 */
void collision_processing_task(void *pvParameters) {
    e_skin_packet_t rx_pkt;
    float sensor_prox[MAX_SENSORS] = {0.0f};
    int current_color = -1;
    TickType_t last_print_time = xTaskGetTickCount(); 

    while (1) {
        if (xQueueReceive(eskin_data_queue, &rx_pkt, portMAX_DELAY) == pdTRUE) {
            if (rx_pkt.payload[0] == 0xFF) {
                
                uint16_t sc_id = ((rx_pkt.payload[1] & 0x7F) << 7) | (rx_pkt.payload[2] & 0x7F);
                uint32_t prox_raw = 0;
                prox_raw |= ((rx_pkt.payload[3] & 0x7F) << 9);
                prox_raw |= ((rx_pkt.payload[4] & 0x7F) << 2);
                prox_raw |= ((rx_pkt.payload[10] >> 3) & 0x03);
                float prox_val = (float)prox_raw / 65536.0f;

                if (sc_id < MAX_SENSORS) {
                    sensor_prox[sc_id] = prox_val;
                }

                float max_prox = 0.0f;
                for (int i = 0; i < MAX_SENSORS; i++) {
                    if (sensor_prox[i] > max_prox) {
                        max_prox = sensor_prox[i];
                    }
                }

                // ==================================================
                // DATEN SENDEN (4 Hz)
                // Da D6 der Terminal-Pin ist, geht dieses printf 
                // automatisch über das grüne Kabel an den STM32!
                // ==================================================
                TickType_t current_time = xTaskGetTickCount();
                if ((current_time - last_print_time) >= pdMS_TO_TICKS(250)) {
                    last_print_time = current_time;
                    
                    // Schickt die nackte Zahl (z.B. "0.555\n") über D6
                    printf("%.3f\n", max_prox);
                }

                // LED Logik
                if (max_prox > PROXIMITY_THRESHOLD_TOUCH) {
                    if (current_color != 2 && global_udp_sock != -1) {
                        sendto(global_udp_sock, LED_BLUE_PKT, sizeof(LED_BLUE_PKT), 0, (struct sockaddr *)&skin_data_addr, sizeof(skin_data_addr));
                        current_color = 2;
                    }
                } 
                else if (max_prox > PROXIMITY_THRESHOLD_STOP) {
                    if (current_color != 1 && global_udp_sock != -1) {
                        sendto(global_udp_sock, LED_RED_PKT, sizeof(LED_RED_PKT), 0, (struct sockaddr *)&skin_data_addr, sizeof(skin_data_addr));
                        current_color = 1;
                    }
                } 
                else if (max_prox < PROXIMITY_THRESHOLD_CLEAR) {
                    if (current_color != 0 && global_udp_sock != -1) {
                        sendto(global_udp_sock, LED_GREEN_PKT, sizeof(LED_GREEN_PKT), 0, (struct sockaddr *)&skin_data_addr, sizeof(skin_data_addr));
                        current_color = 0;
                    }
                }
            }
        }
    }
}

// ==============================================================================
// 4. UDP SENSOR TASK
// ==============================================================================
/**
 * @brief Configures UDP communication with the e-skin device and receives sensor packets.
 *
 * Inputs:
 * - pvParameters: Optional FreeRTOS task context; unused by this task.
 * - Network addresses, ports, and command packets are read from compile-time constants.
 *
 * Outputs:
 * - Initializes global_udp_sock and skin_data_addr.
 * - Sends the lock, start, update-rate, and initial data packets to the e-skin device.
 * - Copies valid received sensor packets into eskin_data_queue.
 * - Does not return during normal operation.
 */
void udp_sensor_task(void *pvParameters) {
    global_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in esp_addr = { .sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_ANY), .sin_port = htons(DATA_PORT_ESP) };
    bind(global_udp_sock, (struct sockaddr *)&esp_addr, sizeof(esp_addr));

    struct sockaddr_in skin_ctrl_addr = { .sin_family = AF_INET, .sin_addr.s_addr = inet_addr(SKIN_IP), .sin_port = htons(CTRL_PORT_SKIN) };
    skin_data_addr.sin_family = AF_INET;
    skin_data_addr.sin_addr.s_addr = inet_addr(SKIN_IP);
    skin_data_addr.sin_port = htons(DATA_PORT_SKIN);

    sendto(global_udp_sock, LOCK_CMD_PKT, sizeof(LOCK_CMD_PKT), 0, (struct sockaddr *)&skin_ctrl_addr, sizeof(skin_ctrl_addr));
    vTaskDelay(pdMS_TO_TICKS(50));
    sendto(global_udp_sock, START_CMD_PKT, sizeof(START_CMD_PKT), 0, (struct sockaddr *)&skin_ctrl_addr, sizeof(skin_ctrl_addr));
    vTaskDelay(pdMS_TO_TICKS(50));
    sendto(global_udp_sock, UDR_63HZ_PKT, sizeof(UDR_63HZ_PKT), 0, (struct sockaddr *)&skin_ctrl_addr, sizeof(skin_ctrl_addr));
    vTaskDelay(pdMS_TO_TICKS(50));
    sendto(global_udp_sock, DUMMY_DATA_PKT, sizeof(DUMMY_DATA_PKT), 0, (struct sockaddr *)&skin_data_addr, sizeof(skin_data_addr));

    uint8_t rx_buffer[128];
    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(global_udp_sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        
        if (len >= 20 && rx_buffer[0] == 0xFF) {
            e_skin_packet_t new_packet;
            memcpy(new_packet.payload, rx_buffer, 20);
            xQueueSend(eskin_data_queue, &new_packet, 0); 
        }
    }
}

// ==============================================================================
// 5. MAIN
// ==============================================================================
/**
 * @brief Initializes persistent storage, networking, queues, and application tasks.
 *
 * Inputs:
 * - None; ESP-IDF invokes this function as the application entry point.
 *
 * Outputs:
 * - Initializes NVS, including recovery from known NVS page/version errors.
 * - Creates eskin_data_queue, establishes Wi-Fi, and starts both application tasks.
 * - Returns no value.
 */
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    eskin_data_queue = xQueueCreate(10, sizeof(e_skin_packet_t));
    wifi_init_sta();

    xTaskCreate(udp_sensor_task, "udp_task", 4096, NULL, 5, NULL);
    xTaskCreate(collision_processing_task, "collision_task", 4096, NULL, 4, NULL);
}
