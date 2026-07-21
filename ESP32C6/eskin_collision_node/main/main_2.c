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
#include "driver/gpio.h"
#include "lwip/sockets.h"

// ==============================================================================
// 1. DEFINITIONEN & STRUKTUREN
// ==============================================================================
#define WIFI_SSID           "hsa005"
#define WIFI_PASS           "hsa%2026"    
#define SKIN_IP             "192.168.4.1"
#define DATA_PORT_SKIN      17010
#define DATA_PORT_ESP       17011
#define CTRL_PORT_SKIN      17000

#define COLLISION_GPIO              GPIO_NUM_21  // Signal-Pin zum STM32
#define PROXIMITY_THRESHOLD_STOP    0.50f
#define PROXIMITY_THRESHOLD_CLEAR   0.40f


//#define PROXIMITY_THRESHOLD_TOUCH   0.80f  // Schwellenwert für direkte Berührung (BLAU)
//#define PROXIMITY_THRESHOLD_STOP    0.08f  // Schwellenwert für 4-5cm Annäherung (ROT)
//#define PROXIMITY_THRESHOLD_CLEAR   0.04f  // Hysterese-Boden zum Zurücksetzen (GRÜN)

static const char *TAG = "ROBOT_NODE";

// FreeRTOS Event Group für Wi-Fi
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;
#define MAXIMUM_RETRY  5

// Struktur für die Queue (20 Bytes Rohdaten der E-Skin)
typedef struct {
    uint8_t payload[20];
} e_skin_packet_t;

QueueHandle_t eskin_data_queue; // Globale Queue

// ==============================================================================
// 2. WI-FI LOGIK
// ==============================================================================
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Verbindungsversuch wiederholt...");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WLAN-Verbindung steht. Bereit für UDP-Sockets.");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Verbindung zu SSID fehlgeschlagen.");
    }
}

// ==============================================================================
// 3. COLLISION PROCESSING TASK (Datenverarbeitung & STM32 Signal)
// ==============================================================================
void collision_processing_task(void *pvParameters) {
    gpio_reset_pin(COLLISION_GPIO);
    gpio_set_direction(COLLISION_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(COLLISION_GPIO, 0); 

    e_skin_packet_t rx_pkt;
    bool is_stopped = false; 

    while (1) {
        // Task schläft ressourcenschonend, bis ein Paket in der Queue landet
        if (xQueueReceive(eskin_data_queue, &rx_pkt, portMAX_DELAY) == pdTRUE) {
            
            // Validierung: E-Skin Pakete beginnen mit 0xFF
            if (rx_pkt.payload[0] == 0xFF) {
                
                // Bit-Shifting zur Rekonstruktion des Proximity-Wertes
                uint32_t prox_raw = 0;
                prox_raw |= ((rx_pkt.payload[3] & 0x7F) << 9);
                prox_raw |= ((rx_pkt.payload[4] & 0x7F) << 2);
                prox_raw |= ((rx_pkt.payload[10] >> 3) & 0x03);

                // Normierung
                float prox_val = (float)prox_raw / 65536.0f;

                // Hysterese-Logik
                if (!is_stopped && prox_val > PROXIMITY_THRESHOLD_STOP) {
                    ESP_LOGW(TAG, "Kollision droht! Prox: %.3f -> Stoppe Arm.", prox_val);
                    gpio_set_level(COLLISION_GPIO, 1);
                    is_stopped = true;
                } else if (is_stopped && prox_val < PROXIMITY_THRESHOLD_CLEAR) {
                    ESP_LOGI(TAG, "Weg wieder frei. Prox: %.3f", prox_val);
                    gpio_set_level(COLLISION_GPIO, 0);
                    is_stopped = false;
                }
            }
        }
    }
}

// ==============================================================================
// 4. UDP SENSOR TASK (Netzwerk-Kommunikation)
// ==============================================================================
const uint8_t LOCK_CMD_PKT[]   = {0x72, 0x6C, 0x7B, 0xCB, 0xDC, 0xC0, 0x1A, 0x00};
const uint8_t START_CMD_PKT[]  = {0x72, 0x6C, 0x7B, 0xCB, 0xDC, 0xC0, 0x0A, 0x00};
const uint8_t UDR_63HZ_PKT[]   = {0x72, 0x6C, 0x7B, 0xCB, 0xDC, 0xA0, 0xD0, 63}; 

// Korrigiertes Dummy-Paket (SC_ID_ALL = 0x3FFF -> 0x7F, 0x7F)
const uint8_t DUMMY_DATA_PKT[] = {0xFF, 0x7F, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                  0x00, 0x00, 0x00, 0xAA};

void udp_sensor_task(void *pvParameters) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Fehler beim Erstellen des Sockets");
        vTaskDelete(NULL);
    }

    struct sockaddr_in esp_addr = { .sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_ANY), .sin_port = htons(DATA_PORT_ESP) };
    bind(sock, (struct sockaddr *)&esp_addr, sizeof(esp_addr));

    struct sockaddr_in skin_ctrl_addr = { .sin_family = AF_INET, .sin_addr.s_addr = inet_addr(SKIN_IP), .sin_port = htons(CTRL_PORT_SKIN) };
    struct sockaddr_in skin_data_addr = { .sin_family = AF_INET, .sin_addr.s_addr = inet_addr(SKIN_IP), .sin_port = htons(DATA_PORT_SKIN) };

    ESP_LOGI(TAG, "Sende Init-Sequenz an E-Skin...");
    sendto(sock, LOCK_CMD_PKT, sizeof(LOCK_CMD_PKT), 0, (struct sockaddr *)&skin_ctrl_addr, sizeof(skin_ctrl_addr));
    vTaskDelay(pdMS_TO_TICKS(50));
    sendto(sock, START_CMD_PKT, sizeof(START_CMD_PKT), 0, (struct sockaddr *)&skin_ctrl_addr, sizeof(skin_ctrl_addr));
    vTaskDelay(pdMS_TO_TICKS(50));
    sendto(sock, UDR_63HZ_PKT, sizeof(UDR_63HZ_PKT), 0, (struct sockaddr *)&skin_ctrl_addr, sizeof(skin_ctrl_addr));
    vTaskDelay(pdMS_TO_TICKS(50));
    sendto(sock, DUMMY_DATA_PKT, sizeof(DUMMY_DATA_PKT), 0, (struct sockaddr *)&skin_data_addr, sizeof(skin_data_addr));

    uint8_t rx_buffer[128];
    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        
        if (len >= 20 && rx_buffer[0] == 0xFF) {
            e_skin_packet_t new_packet;
            memcpy(new_packet.payload, rx_buffer, 20);
            
            // Paket ohne Blockieren in die Queue schieben
            xQueueSend(eskin_data_queue, &new_packet, 0); 
        }
    }
}

// ==============================================================================
// 5. MAIN ENTRY POINT (Startpunkt)
// ==============================================================================
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Queue für maximal 10 Pakete erstellen
    eskin_data_queue = xQueueCreate(10, sizeof(e_skin_packet_t));
    if (eskin_data_queue == NULL) {
        ESP_LOGE(TAG, "Fehler beim Erstellen der Queue!");
        return;
    }

    wifi_init_sta();

    // Beide Tasks starten
    xTaskCreate(udp_sensor_task, "udp_task", 4096, NULL, 5, NULL);
    xTaskCreate(collision_processing_task, "collision_task", 4096, NULL, 4, NULL);
}