#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_board_init.h"
#include "speech_commands_action.h"
#include "model_path.h"
#include "esp_process_sdkconfig.h"
#include "esp_camera.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_tls.h"
#include "esp_http_client.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_heap_caps.h"
#include "jarvis.h"

#define BOUNDARY "X-ESPIDF_MULTIPART"
#define SERVER "api.telegram.org"
#define PORT "443"
#define PATH "/bot7397170163:AAEyJvVdmEfe1Jt2hKoWH-KZZWu-OOIr56s/sendPhoto"
#define WIFI_SSID "Goshuajoh"
#define WIFI_PASS "Spidermanisc00l"

#define BOARD_LCD_MOSI 47
#define BOARD_LCD_MISO -1
#define BOARD_LCD_SCK 21
#define BOARD_LCD_CS 44
#define BOARD_LCD_DC 43
#define BOARD_LCD_RST -1
#define BOARD_LCD_BL 48
#define BOARD_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define BOARD_LCD_BK_LIGHT_ON_LEVEL 0
#define BOARD_LCD_BK_LIGHT_OFF_LEVEL !BOARD_LCD_BK_LIGHT_ON_LEVEL
#define BOARD_LCD_H_RES 240
#define BOARD_LCD_V_RES 240
#define BOARD_LCD_CMD_BITS 8
#define BOARD_LCD_PARAM_BITS 8
#define LCD_HOST SPI2_HOST

#define CAMERA_MODULE_NAME "ESP-S3-EYE"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1

#define CAMERA_PIN_VSYNC 6
#define CAMERA_PIN_HREF 7
#define CAMERA_PIN_PCLK 13
#define CAMERA_PIN_XCLK 15

#define CAMERA_PIN_SIOD 4
#define CAMERA_PIN_SIOC 5

#define CAMERA_PIN_D0 11
#define CAMERA_PIN_D1 9
#define CAMERA_PIN_D2 8
#define CAMERA_PIN_D3 10
#define CAMERA_PIN_D4 12
#define CAMERA_PIN_D5 18
#define CAMERA_PIN_D6 17
#define CAMERA_PIN_D7 16

#define BOOT_BUTTON_GPIO GPIO_NUM_0

#define EXAMPLE_MAX_CHAR_SIZE 64

#define EXAMPLE_ESP_WIFI_SSID "Goshuajoh"
#define EXAMPLE_ESP_WIFI_PASS "Spidermanisc00l"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN

extern const uint8_t telegram_certificate_pem_start[] asm("_binary_telegram_certificate_pem_start");
extern const uint8_t telegram_certificate_pem_end[] asm("_binary_telegram_certificate_pem_end");

volatile bool freeze_frame = false;

static const char *TAG = "digicam";
static esp_lcd_panel_handle_t panel_handle = NULL;
static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static SemaphoreHandle_t captureSemaphore;
static TaskHandle_t lcdTaskHandle = NULL;

volatile bool display_message = false;
char message[64] = {0};
SemaphoreHandle_t messageMutex;

static const unsigned char font8x8_basic[128][8] = {
    ['Y'] = {0x41, 0x22, 0x14, 0x08, 0x08, 0x08, 0x08, 0x00},
    ['e'] = {0x00, 0x3E, 0x41, 0x7F, 0x40, 0x3E, 0x00, 0x00},
    ['s'] = {0x00, 0x3E, 0x40, 0x3E, 0x01, 0x3E, 0x00, 0x00},
    ['i'] = {0x00, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00},
    ['r'] = {0x00, 0x1E, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00},
    ['!'] = {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04, 0x00},
    ['M'] = {0x41, 0x63, 0x55, 0x49, 0x41, 0x41, 0x41, 0x00},
    ['o'] = {0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, 0x00},
    ['d'] = {0x01, 0x01, 0x1F, 0x21, 0x21, 0x1F, 0x00, 0x00},
    [':'] = {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00, 0x00},
    ['0'] = {0x3E, 0x45, 0x49, 0x51, 0x61, 0x3E, 0x00, 0x00},
    ['1'] = {0x04, 0x0C, 0x14, 0x04, 0x04, 0x04, 0x00, 0x00},
    ['2'] = {0x3E, 0x41, 0x02, 0x1C, 0x20, 0x7F, 0x00, 0x00},
    ['3'] = {0x3E, 0x41, 0x06, 0x06, 0x41, 0x3E, 0x00, 0x00},
};
#define RGB565_LCD_WHITE 0xFFFF // White color in RGB565 format
#define RGB565_LCD_BLACK 0x0000
#define SCALE_FACTOR 3
static TimerHandle_t messageTimerHandle = NULL;

int capture_mode = 0;
int64_t last_press_time = 0;
int64_t current_time = 0;
bool waiting_for_double_press = false;

camera_config_t camera_config = {
    .ledc_channel = LEDC_CHANNEL_0,
    .ledc_timer = LEDC_TIMER_0,
    .pin_d0 = CAMERA_PIN_D0,
    .pin_d1 = CAMERA_PIN_D1,
    .pin_d2 = CAMERA_PIN_D2,
    .pin_d3 = CAMERA_PIN_D3,
    .pin_d4 = CAMERA_PIN_D4,
    .pin_d5 = CAMERA_PIN_D5,
    .pin_d6 = CAMERA_PIN_D6,
    .pin_d7 = CAMERA_PIN_D7,
    .pin_xclk = CAMERA_PIN_XCLK,
    .pin_pclk = CAMERA_PIN_PCLK,
    .pin_vsync = CAMERA_PIN_VSYNC,
    .pin_href = CAMERA_PIN_HREF,
    .pin_sscb_sda = CAMERA_PIN_SIOD,
    .pin_sscb_scl = CAMERA_PIN_SIOC,
    .pin_pwdn = CAMERA_PIN_PWDN,
    .pin_reset = CAMERA_PIN_RESET,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .xclk_freq_hz = 16000000,
    .pixel_format = PIXFORMAT_RGB565, // PIXFORMAT_JPEG if using JPEG
    .frame_size = FRAMESIZE_240X240,  // FRAMESIZE_QVGA, FRAMESIZE_CIF, etc.
    .jpeg_quality = 50,               // 0-63 lower number means higher quality
    .fb_count = 1,                    // If more than one, i2s runs in continuous mode. Use only with JPEG
};

camera_config_t camera_config1 = {
    .ledc_channel = LEDC_CHANNEL_0,
    .ledc_timer = LEDC_TIMER_0,
    .pin_d0 = CAMERA_PIN_D0,
    .pin_d1 = CAMERA_PIN_D1,
    .pin_d2 = CAMERA_PIN_D2,
    .pin_d3 = CAMERA_PIN_D3,
    .pin_d4 = CAMERA_PIN_D4,
    .pin_d5 = CAMERA_PIN_D5,
    .pin_d6 = CAMERA_PIN_D6,
    .pin_d7 = CAMERA_PIN_D7,
    .pin_xclk = CAMERA_PIN_XCLK,
    .pin_pclk = CAMERA_PIN_PCLK,
    .pin_vsync = CAMERA_PIN_VSYNC,
    .pin_href = CAMERA_PIN_HREF,
    .pin_sscb_sda = CAMERA_PIN_SIOD,
    .pin_sscb_scl = CAMERA_PIN_SIOC,
    .pin_pwdn = CAMERA_PIN_PWDN,
    .pin_reset = CAMERA_PIN_RESET,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .xclk_freq_hz = 20000000,
    .pixel_format = PIXFORMAT_JPEG, // PIXFORMAT_JPEG if using JPEG
    .frame_size = FRAMESIZE_SVGA,   // FRAMESIZE_QVGA, FRAMESIZE_CIF, etc.
    .jpeg_quality = 5,              // 0-63 lower number means higher quality
    .fb_count = 1,                  // If more than one, i2s runs in continuous mode. Use only with JPEG
};

void messageTimerCallback(TimerHandle_t xTimer)
{
    xSemaphoreTake(messageMutex, portMAX_DELAY);
    display_message = false;
    memset(message, 0, sizeof(message));
    xSemaphoreGive(messageMutex);
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        if (!esp_http_client_is_chunked_response(evt->client))
        {
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            printf("%.*s", evt->data_len, (char *)evt->data);
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGI(TAG, "HTTP_EVENT_REDIRECT");
        break;
    }
    return ESP_OK;
}

void configure_gpio()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
}

static int s_retry_num = 0;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            ESP_LOGI(TAG, "connect to the AP fail");
            return;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void lcd_task(void *pvParameters)
{
    esp_err_t err;
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)pvParameters;

    esp_err_t err1 = esp_camera_init(&camera_config);
    if (err1 != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);

    while (1)
    {
        if (!freeze_frame)
        {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb)
            {
                ESP_LOGE(TAG, "Camera Capture Failed");
                esp_camera_fb_return(fb);
                continue;
            }

            xSemaphoreTake(messageMutex, portMAX_DELAY);
            if (display_message)
            {
                uint16_t *pixels = (uint16_t *)heap_caps_malloc((240 * 240) * sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
                if (NULL == pixels)
                {
                    ESP_LOGE(TAG, "Memory for bitmap is not enough");
                    return;
                }
                memcpy(pixels, logo_en_240x240_lcd, (240 * 240) * sizeof(uint16_t));

                // Display message overlay
                const char *text = message;
                int text_len = strlen(text);
                int char_width = 8 * SCALE_FACTOR;
                int char_height = 8 * SCALE_FACTOR;
                int x_start = (fb->width - (text_len * char_width)) / 2;
                int y_start = (fb->height - char_height) / 2;

                for (int i = 0; i < text_len; i++)
                {
                    char c = text[i];
                    for (int j = 0; j < 8; j++)
                    {
                        for (int k = 0; k < 8; k++)
                        {
                            if (font8x8_basic[(int)c][j] & (1 << (7 - k)))
                            {
                                int px = x_start + i * char_width + k * SCALE_FACTOR;
                                int py = y_start + j * SCALE_FACTOR;
                                for (int dy = 0; dy < SCALE_FACTOR; dy++)
                                {
                                    for (int dx = 0; dx < SCALE_FACTOR; dx++)
                                    {
                                        int tx = px + dx;
                                        int ty = py + dy;
                                        if (tx < 240 && ty < 240)
                                        {
                                            pixels[ty * 240 + tx] = RGB565_LCD_BLACK;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 240, 240, (uint16_t *)pixels);
                heap_caps_free(pixels);
            }
            xSemaphoreGive(messageMutex);
            if (!display_message)
            {
                err = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, BOARD_LCD_H_RES, BOARD_LCD_V_RES, fb->buf);
            }
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "LCD Panel Draw Bitmap Failed");
            }

            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay as needed
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay as needed
        }
    }
}

void capture_and_send_image_task(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(captureSemaphore, portMAX_DELAY) == pdTRUE)
        {
            if (lcdTaskHandle != NULL)
            {
                vTaskDelete(lcdTaskHandle);
                lcdTaskHandle = NULL;
            }

            freeze_frame = true;
            esp_camera_deinit();
            esp_camera_init(&camera_config1);
            sensor_t *s = esp_camera_sensor_get();
            s->set_vflip(s, 1);
            s->set_brightness(s, 1); // Increase brightness
            s->set_contrast(s, 0);   // Adjust contrast
            s->set_saturation(s, 0); // Adjust saturation
            if (capture_mode == 0)
            {
                s->set_special_effect(s, 0); // No special effect
            }
            else if (capture_mode == 1)
            {
                s->set_special_effect(s, 2); // Grayscale
            }
            else if (capture_mode == 2)
            {
                s->set_special_effect(s, 3); // Sepia
            }
            else if (capture_mode == 3)
            {
                s->set_special_effect(s, 4); // Negative
            }
            s->set_whitebal(s, 1);                   // Enable white balance
            s->set_awb_gain(s, 1);                   // Enable AWB gain
            s->set_wb_mode(s, 0);                    // Auto white balance mode
            s->set_exposure_ctrl(s, 1);              // Enable exposure control
            s->set_aec2(s, 0);                       // Disable AEC2
            s->set_gain_ctrl(s, 1);                  // Enable gain control
            s->set_agc_gain(s, 10);                  // Adjust AGC gain
            s->set_gainceiling(s, (gainceiling_t)4); // Adjust gain ceiling
            s->set_bpc(s, 1);                        // Enable black pixel correction
            s->set_wpc(s, 1);                        // Enable white pixel correction
            s->set_raw_gma(s, 1);                    // Enable gamma correction
            s->set_lenc(s, 1);                       // Enable lens correction
            s->set_hmirror(s, 0);                    // Disable horizontal mirror
            s->set_dcw(s, 1);                        // Enable downsize/enlarge
            s->set_colorbar(s, 0);                   // Disable color bar

            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb)
            {
                ESP_LOGE(TAG, "Camera capture failed");
                freeze_frame = false;
                continue;
            }

            uint8_t *jpeg_buf = fb->buf;
            size_t jpeg_len = fb->len;

            char url[512];
            snprintf(url, sizeof(url), "https://%s%s", SERVER, PATH);

            char boundary[] = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
            char content_type[128];
            snprintf(content_type, sizeof(content_type), "multipart/form-data; boundary=%s", boundary);

            char *chat_id = "219745533";

            char form_data_start[512];
            snprintf(form_data_start, sizeof(form_data_start),
                     "--%s\r\n"
                     "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n"
                     "%s\r\n"
                     "--%s\r\n"
                     "Content-Disposition: form-data; name=\"photo\"; filename=\"image.jpg\"\r\n"
                     "Content-Type: image/jpeg\r\n\r\n",
                     boundary, chat_id, boundary);

            char form_data_end[128];
            snprintf(form_data_end, sizeof(form_data_end), "\r\n--%s--\r\n", boundary);

            int total_len = strlen(form_data_start) + jpeg_len + strlen(form_data_end);
            char content_length[16];
            snprintf(content_length, sizeof(content_length), "%d", total_len);

            esp_http_client_config_t config = {
                .url = url,
                .event_handler = _http_event_handler,
                .cert_pem = (const char *)telegram_certificate_pem_start,
                .timeout_ms = 10000,
            };

            esp_http_client_handle_t client = esp_http_client_init(&config);

            esp_http_client_set_method(client, HTTP_METHOD_POST);
            esp_http_client_set_header(client, "Content-Type", content_type);
            esp_http_client_set_header(client, "Content-Length", content_length);

            esp_err_t err = esp_http_client_open(client, total_len);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
                esp_http_client_cleanup(client);
                esp_camera_fb_return(fb);
                freeze_frame = false;
                continue;
            }

            esp_http_client_write(client, form_data_start, strlen(form_data_start));
            esp_http_client_write(client, (const char *)jpeg_buf, jpeg_len);
            esp_http_client_write(client, form_data_end, strlen(form_data_end));

            char recv_buf[1024];
            int response_len = esp_http_client_fetch_headers(client);
            response_len = esp_http_client_read_response(client, recv_buf, sizeof(recv_buf));
            if (response_len >= 0)
            {
                ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %lld",
                         esp_http_client_get_status_code(client),
                         esp_http_client_get_content_length(client));
                ESP_LOGI(TAG, "HTTP Response: %.*s", response_len, recv_buf);
            }
            else
            {
                ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
            }

            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            esp_camera_fb_return(fb);
            esp_camera_deinit();
            // esp_camera_init(&camera_config);
            // s->set_vflip(s, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            freeze_frame = false;
            xTaskCreatePinnedToCore(&lcd_task, "lcd_task", 3 * 1024, (void *)panel_handle, 5, &lcdTaskHandle, 1);
        }
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int detect_flag = 0;
static esp_afe_sr_iface_t *afe_handle = NULL;
static volatile int task_flag = 0;
srmodel_list_t *models = NULL;
static int play_voice = -2;

void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_channel_num(afe_data);
    int feed_channel = esp_get_feed_channel();
    assert(nch <= feed_channel);
    int16_t *i2s_buff = (int16_t *)heap_caps_malloc(audio_chunksize * sizeof(int16_t) * feed_channel, MALLOC_CAP_SPIRAM);
    assert(i2s_buff);

    while (task_flag)
    {
        esp_get_feed_data(false, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);
        afe_handle->feed(afe_data, i2s_buff);
        vTaskDelay(1); // Add delay to prevent watchdog timeout
    }
    if (i2s_buff)
    {
        free(i2s_buff);
        i2s_buff = NULL;
    }
    vTaskDelete(NULL);
}

void detect_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    printf("multinet:%s\n", mn_name);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *model_data = multinet->create(mn_name, 6000);
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    esp_mn_commands_update_from_sdkconfig(multinet, model_data);
    assert(mu_chunksize == afe_chunksize);
    multinet->print_active_speech_commands(model_data);

    printf("------------detect start------------\n");
    while (task_flag)
    {
        afe_fetch_result_t *res = afe_handle->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL)
        {
            printf("fetch error!\n");
            break;
        }

        if (res->wakeup_state == WAKENET_DETECTED)
        {
            printf("WAKEWORD DETECTED\n");
            multinet->clean(model_data);
            xSemaphoreTake(messageMutex, portMAX_DELAY);
            snprintf(message, sizeof(message), "Yessir!");
            display_message = true;
            xTimerStart(messageTimerHandle, 0);
            xSemaphoreGive(messageMutex);
        }
        else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED)
        {
            play_voice = -1;
            detect_flag = 1;
            printf("AFE_FETCH_CHANNEL_VERIFIED, channel index: %d\n", res->trigger_channel_id);
        }

        if (detect_flag == 1)
        {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

            if (mn_state == ESP_MN_STATE_DETECTING)
            {
                continue;
            }

            if (mn_state == ESP_MN_STATE_DETECTED)
            {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                for (int i = 0; i < mn_result->num; i++)
                {
                    printf("TOP %d, command_id: %d, phrase_id: %d, string: %s, prob: %f\n",
                           i + 1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->string, mn_result->prob[i]);
                    if (mn_result->command_id[i] == 0 || mn_result->command_id[i] == 1 || mn_result->command_id[i] == 2)
                    {
                        printf("Detected Correctly, attempting to execute camera task!\n");
                        display_message = false; // new
                        ESP_LOGI(TAG, "Free heap size: %d bytes", xPortGetFreeHeapSize());
                        xSemaphoreGive(captureSemaphore);
                        break;
                    }
                }
                printf("-----------listening-----------\n");
            }

            if (mn_state == ESP_MN_STATE_TIMEOUT)
            {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                printf("timeout, string:%s\n", mn_result->string);
                afe_handle->enable_wakenet(afe_data);
                detect_flag = 0;
                display_message = false;
                printf("\n-----------awaits to be waken up-----------\n");
                continue;
            }
        }
        vTaskDelay(1); // Add delay to prevent watchdog timeout
    }
    if (model_data)
    {
        multinet->destroy(model_data);
        model_data = NULL;
    }
    printf("detect exit\n");
    vTaskDelete(NULL);
}

void app_main()
{
    messageMutex = xSemaphoreCreateMutex();
    models = esp_srmodel_init("model");
    ESP_ERROR_CHECK(esp_board_init(16000, 2, 32));
    afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;

    afe_config_t afe_config = {
        .aec_init = true,
        .se_init = true,
        .vad_init = true,
        .wakenet_init = true,
        .voice_communication_init = false,
        .voice_communication_agc_init = false,
        .voice_communication_agc_gain = 15,
        .vad_mode = VAD_MODE_3,
        .wakenet_model_name = NULL,
        .wakenet_model_name_2 = NULL,
        .wakenet_mode = DET_MODE_2CH_90,
        .afe_mode = SR_MODE_LOW_COST,
        .afe_perferred_core = 0,
        .afe_perferred_priority = 5,
        .afe_ringbuf_size = 50,
        .memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM,
        .afe_linear_gain = 1.0,
        .agc_mode = AFE_MN_PEAK_AGC_MODE_2,
        .pcm_config = {
            .total_ch_num = 3,
            .mic_num = 2,
            .ref_num = 1,
            .sample_rate = 16000,
        },
        .debug_init = false,
        .debug_hook = {{AFE_DEBUG_HOOK_MASE_TASK_IN, NULL}, {AFE_DEBUG_HOOK_FETCH_TASK_IN, NULL}},
    };

    afe_config.aec_init = false;
    afe_config.se_init = false;
    afe_config.vad_init = false;
    afe_config.afe_ringbuf_size = 10;
    afe_config.pcm_config.total_ch_num = 2;
    afe_config.pcm_config.mic_num = 1;
    afe_config.pcm_config.ref_num = 1;
    afe_config.pcm_config.sample_rate = 16000;

    afe_config.wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);

    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init_sta();
    configure_gpio();

    messageTimerHandle = xTimerCreate("MessageTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, messageTimerCallback);
    if (messageTimerHandle == NULL)
    {
        ESP_LOGE(TAG, "Message timer creation failed!");
        return;
    }

    void app_lcd_set_color(int color)
    {
        uint16_t *buffer = (uint16_t *)heap_caps_malloc(BOARD_LCD_H_RES * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
        if (NULL == buffer)
        {
            ESP_LOGE(TAG, "Memory for bitmap is not enough");
        }
        else
        {
            for (size_t i = 0; i < BOARD_LCD_H_RES; i++)
            {
                buffer[i] = color;
            }

            for (int y = 0; y < BOARD_LCD_V_RES; y++)
            {
                esp_lcd_panel_draw_bitmap(panel_handle, 0, y, BOARD_LCD_H_RES, y + 1, buffer);
            }

            free(buffer);
        }
    }

    esp_err_t register_lcd(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb)
    {
        ESP_LOGI(TAG, "Initialize SPI bus");
        spi_bus_config_t bus_conf = {
            .sclk_io_num = BOARD_LCD_SCK,
            .mosi_io_num = BOARD_LCD_MOSI,
            .miso_io_num = BOARD_LCD_MISO,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = BOARD_LCD_H_RES * BOARD_LCD_V_RES * sizeof(uint16_t),
        };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_conf, SPI_DMA_CH_AUTO));

        ESP_LOGI(TAG, "Install panel IO");
        esp_lcd_panel_io_handle_t io_handle = NULL;
        esp_lcd_panel_io_spi_config_t io_config = {
            .dc_gpio_num = BOARD_LCD_DC,
            .cs_gpio_num = BOARD_LCD_CS,
            .pclk_hz = BOARD_LCD_PIXEL_CLOCK_HZ,
            .lcd_cmd_bits = BOARD_LCD_CMD_BITS,
            .lcd_param_bits = BOARD_LCD_PARAM_BITS,
            .spi_mode = 0,
            .trans_queue_depth = 10,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = BOARD_LCD_RST,
            .rgb_endian = LCD_RGB_ENDIAN_RGB,
            .bits_per_pixel = 16,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        esp_lcd_panel_invert_color(panel_handle, true);

        esp_lcd_panel_disp_on_off(panel_handle, true);

        app_lcd_set_color(0x000000);

        return ESP_OK;
    }
    register_lcd(xQueueFrameI, xQueueFrameO, true);
    gpio_set_direction(BOARD_LCD_BL, GPIO_MODE_OUTPUT);
    gpio_set_level(BOARD_LCD_BL, BOARD_LCD_BK_LIGHT_ON_LEVEL);

    captureSemaphore = xSemaphoreCreateBinary();
    task_flag = 1;

    xTaskCreatePinnedToCore(&detect_Task, "detect", 5 * 1024, (void *)afe_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(&feed_Task, "feed", 4 * 1024, (void *)afe_data, 5, NULL, 0);
    // xTaskCreatePinnedToCore(&lcd_task, "lcd_task", 3 * 1024, (void *)panel_handle, 5, NULL, 1);
    xTaskCreatePinnedToCore(&lcd_task, "lcd_task", 3 * 1024, (void *)panel_handle, 5, &lcdTaskHandle, 1);
    xTaskCreatePinnedToCore(&capture_and_send_image_task, "capture_and_send_image_task", 11 * 1024, NULL, 4, NULL, 0);
    while (1)
    {
        if (gpio_get_level(BOOT_BUTTON_GPIO) == 0)
        {
            current_time = esp_timer_get_time() / 1000; // Get current time in milliseconds
            if (waiting_for_double_press && (current_time - last_press_time) <= 3000)
            {
                // Double press detected
                capture_mode = (capture_mode + 1) % 4;
                printf("Double press detected, changing capture mode to %d\n", capture_mode);
                xSemaphoreTake(messageMutex, portMAX_DELAY);
                snprintf(message, sizeof(message), "Mode: %d", capture_mode);
                display_message = true;
                xTimerStart(messageTimerHandle, 0);
                xSemaphoreGive(messageMutex);
                waiting_for_double_press = false;
            }
            else
            {
                // Single press detected, wait for a potential double press
                // printf("Pseudo Detected, attempting to execute camera task!\n");
                // ESP_LOGI(TAG, "Free heap size: %d bytes", xPortGetFreeHeapSize());
                // xSemaphoreGive(captureSemaphore);
                waiting_for_double_press = true;
                last_press_time = current_time;
            }

            // Wait to avoid multiple triggers
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Reset waiting for double press if time exceeds the interval
        if (waiting_for_double_press && (esp_timer_get_time() / 1000 - last_press_time) > 3000)
        {
            // Single press action
            printf("Pseudo Detected, attempting to execute camera task!\n");
            ESP_LOGI(TAG, "Free heap size: %d bytes", xPortGetFreeHeapSize());
            xSemaphoreGive(captureSemaphore);
            waiting_for_double_press = false;
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
    }
}
