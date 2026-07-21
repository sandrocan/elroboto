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
#include "driver/uart.h"     
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

// UART Pin-Konfiguration
#define UART_TX_PIN                 GPIO_NUM_16 // Entspricht D6
#define UART_PORT_NUM               UART_NUM_1
#define UART_BAUD_RATE              115200

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

// Initialisierungs-Pakete für die E-Skin
const uint8_t LOCK_CMD_PKT[]   = {0x72, 0x6C, 0x7B, 0xCB, 0xDC, 0xC0, 0x1A, 0x00};
const uint8_t START_CMD_PKT[]  = {0x72, 0x6C, 0x7B, 0xCB, 0xDC, 0xC0, 0x0A, 0x00};
const uint8_t UDR_63HZ_PKT[]   = {0x72, 0x6C, 0x7B, 0xCB, 0xDC, 0xA0, 0xD0, 63}; 
const uint8_t DUMMY_DATA_PKT[] = {0xFF, 0x7F, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA};

// ==============================================================================
// 2. HARDWARE INITIALISIERUNGEN (UART & Wi-Fi)
// ==============================================================================
void init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 256, 256, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    
    // RX ist deaktiviert (UART_PIN_NO_CHANGE)
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART1 initialisiert. Sende Telemetrie auf TX Pin %d", UART_TX_PIN);
}

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
// 3. DATENVERARBEITUNG (Reine WLAN zu UART Brücke)
// ==============================================================================
void collision_processing_task(void *pvParameters) {
    e_skin_packet_t rx_pkt;
    float sensor_prox[MAX_SENSORS] = {0.0f};
    int uart_send_counter = 0;

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

                // Den höchsten Distanzwert über alle Sensoren ermitteln
                float max_prox = 0.0f;
                for (int i = 0; i < MAX_SENSORS; i++) {
                    if (sensor_prox[i] > max_prox) {
                        max_prox = sensor_prox[i];
                    }
                }

                // Sende den Wert via UART an den STM32 (gedrosselt, um ihn nicht zu überlasten)
                uart_send_counter++;
                if (uart_send_counter % 3 == 0) { 
                    char tx_buffer[32];
                    int len = snprintf(tx_buffer, sizeof(tx_buffer), "%.3f\n", max_prox);
                    uart_write_bytes(UART_PORT_NUM, tx_buffer, len);
                }
            }
        }
    }
}

// ==============================================================================
// 4. UDP SENSOR TASK (Netzwerk-Empfang)
// ==============================================================================
void udp_sensor_task(void *pvParameters) {
    global_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in esp_addr = { .sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_ANY), .sin_port = htons(DATA_PORT_ESP) };
    bind(global_udp_sock, (struct sockaddr *)&esp_addr, sizeof(esp_addr));

    struct sockaddr_in skin_ctrl_addr = { .sin_family = AF_INET, .sin_addr.s_addr = inet_addr(SKIN_IP), .sin_port = htons(CTRL_PORT_SKIN) };
    skin_data_addr.sin_family = AF_INET;
    skin_data_addr.sin_addr.s_addr = inet_addr(SKIN_IP);
    skin_data_addr.sin_port = htons(DATA_PORT_SKIN);

    // Initialisierungs-Pakete an die Haut senden
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
// 5. MAIN ENTRY POINT
// ==============================================================================
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_uart();

    eskin_data_queue = xQueueCreate(10, sizeof(e_skin_packet_t));
    wifi_init_sta();

    xTaskCreate(udp_sensor_task, "udp_task", 4096, NULL, 5, NULL);
    xTaskCreate(collision_processing_task, "collision_task", 4096, NULL, 4, NULL);
}