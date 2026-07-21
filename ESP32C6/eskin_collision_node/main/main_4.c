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

#define COLLISION_GPIO              GPIO_NUM_21  

// Neue, feiner abgestimmte Schwellenwerte für 4-5cm und Berührung
#define PROXIMITY_THRESHOLD_TOUCH   0.80f  // Berührung -> BLAU
#define PROXIMITY_THRESHOLD_STOP    0.03f  // 4-5 cm Annäherung -> ROT
#define PROXIMITY_THRESHOLD_CLEAR   0.015f  // Weg frei -> GRÜN

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

// Globale Netzwerk-Variablen, damit der Kollisions-Task Farben senden kann
int global_udp_sock = -1;
struct sockaddr_in skin_data_addr;


// ==============================================================================
// 2. E-SKIN PROTOKOLL (Byte-Arrays aus Python abgeleitet)
// ==============================================================================
const uint8_t LOCK_CMD_PKT[]   = {0x72, 0x6C, 0x7B, 0xCB, 0xDC, 0xC0, 0x1A, 0x00};
const uint8_t START_CMD_PKT[]  = {0x72, 0x6C, 0x7B, 0xCB, 0xDC, 0xC0, 0x0A, 0x00};
const uint8_t UDR_63HZ_PKT[]   = {0x72, 0x6C, 0x7B, 0xCB, 0xDC, 0xA0, 0xD0, 63}; 
const uint8_t DUMMY_DATA_PKT[] = {0xFF, 0x7F, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA};

// Farben-Pakete (Übersetzt aus der led_rgb() Python Funktion)
const uint8_t LED_GREEN_PKT[]  = {0xCA, 0x7F, 0x7F, 0x00, 0x00, 0x7F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA};
const uint8_t LED_RED_PKT[]    = {0xCA, 0x7F, 0x7F, 0x7F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA};
const uint8_t LED_BLUE_PKT[]   = {0xCA, 0x7F, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA};


// ==============================================================================
// 3. WI-FI LOGIK
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

    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

// ==============================================================================
// 4. COLLISION PROCESSING TASK (Daten & LED State-Machine)
// ==============================================================================
#define MAX_SENSORS 32 // Speicherplatz für bis zu 32 Sensoren auf der Haut

void collision_processing_task(void *pvParameters) {
    gpio_reset_pin(COLLISION_GPIO);
    gpio_set_direction(COLLISION_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(COLLISION_GPIO, 0); 

    e_skin_packet_t rx_pkt;
    int current_color = -1; // -1 = uninitialisiert, 0=Grün, 1=Rot, 2=Blau
    
    // Gedächtnis-Array: Merkt sich den letzten bekannten Wert JEDES Sensors
    float sensor_prox[MAX_SENSORS] = {0.0f};

    while (1) {
        if (xQueueReceive(eskin_data_queue, &rx_pkt, portMAX_DELAY) == pdTRUE) {
            
            if (rx_pkt.payload[0] == 0xFF) {
                
                // 1. Sensor-ID extrahieren (Bit-Shifting von Byte 1 und 2)
                uint16_t sc_id = ((rx_pkt.payload[1] & 0x7F) << 7) | (rx_pkt.payload[2] & 0x7F);
                
                // 2. Prox-Wert extrahieren
                uint32_t prox_raw = 0;
                prox_raw |= ((rx_pkt.payload[3] & 0x7F) << 9);
                prox_raw |= ((rx_pkt.payload[4] & 0x7F) << 2);
                prox_raw |= ((rx_pkt.payload[10] >> 3) & 0x03);
                float prox_val = (float)prox_raw / 65536.0f;

                // 3. Wert im Gedächtnis-Array speichern
                if (sc_id < MAX_SENSORS) {
                    sensor_prox[sc_id] = prox_val;
                }

                // 4. Den HÖCHSTEN Prox-Wert finden
                float max_prox = 0.0f;
                for (int i = 0; i < MAX_SENSORS; i++) {
                    if (sensor_prox[i] > max_prox) {
                        max_prox = sensor_prox[i];
                    }
                }

                // --- NEU: Kontinuierlicher Live-Print (gedrosselt) ---
                static int live_log_counter = 0;
                live_log_counter++;
                // Zeigt jeden 15. Wert an (also ca. 4 Mal pro Sekunde)
                if (live_log_counter % 15 == 0) {
                    printf("Live-Distanz: %.3f\n", max_prox);
                }
                // -----------------------------------------------------

                

                // 5. State-Machine reagiert ab sofort nur noch auf "max_prox"
                if (max_prox > PROXIMITY_THRESHOLD_TOUCH) {
                    if (current_color != 2 && global_udp_sock != -1) {
                        sendto(global_udp_sock, LED_BLUE_PKT, sizeof(LED_BLUE_PKT), 0, (struct sockaddr *)&skin_data_addr, sizeof(skin_data_addr));
                        current_color = 2;
                        gpio_set_level(COLLISION_GPIO, 1);
                        ESP_LOGW(TAG, "BERÜHRUNG! LED = BLAU (Max Prox: %.3f)", max_prox);
                    }
                } 
                else if (max_prox > PROXIMITY_THRESHOLD_STOP) {
                    if (current_color != 1 && global_udp_sock != -1) {
                        sendto(global_udp_sock, LED_RED_PKT, sizeof(LED_RED_PKT), 0, (struct sockaddr *)&skin_data_addr, sizeof(skin_data_addr));
                        current_color = 1;
                        gpio_set_level(COLLISION_GPIO, 1); 
                        ESP_LOGW(TAG, "Kollision droht! LED = ROT (Max Prox: %.3f)", max_prox);
                    }
                } 
                else if (max_prox < PROXIMITY_THRESHOLD_CLEAR) {
                    if (current_color != 0 && global_udp_sock != -1) {
                        sendto(global_udp_sock, LED_GREEN_PKT, sizeof(LED_GREEN_PKT), 0, (struct sockaddr *)&skin_data_addr, sizeof(skin_data_addr));
                        current_color = 0;
                        gpio_set_level(COLLISION_GPIO, 0);
                        ESP_LOGI(TAG, "Weg frei. LED = GRÜN (Max Prox: %.3f)", max_prox);
                    }
                }
            }
        }
    }
}

// ==============================================================================
// 5. UDP SENSOR TASK (Netzwerk-Kommunikation)
// ==============================================================================
void udp_sensor_task(void *pvParameters) {
    global_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (global_udp_sock < 0) {
        vTaskDelete(NULL);
    }

    struct sockaddr_in esp_addr = { .sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_ANY), .sin_port = htons(DATA_PORT_ESP) };
    bind(global_udp_sock, (struct sockaddr *)&esp_addr, sizeof(esp_addr));

    struct sockaddr_in skin_ctrl_addr = { .sin_family = AF_INET, .sin_addr.s_addr = inet_addr(SKIN_IP), .sin_port = htons(CTRL_PORT_SKIN) };
    
    // Konfiguration der globalen Data-Adresse (für LEDs)
    skin_data_addr.sin_family = AF_INET;
    skin_data_addr.sin_addr.s_addr = inet_addr(SKIN_IP);
    skin_data_addr.sin_port = htons(DATA_PORT_SKIN);

    ESP_LOGI(TAG, "Sende Init-Sequenz an E-Skin...");
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
// 6. MAIN ENTRY POINT
// ==============================================================================
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