#include <stdint.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LCD_HOST SPI2_HOST

#define PIN_NUM_MOSI 3
#define PIN_NUM_CLK 4
#define PIN_NUM_CS 1
#define PIN_NUM_DC 2
#define PIN_NUM_RST -1
#define PIN_NUM_BL 5

#define ENC_PIN_A 7
#define ENC_PIN_B 8
#define ENC_PIN_SW 6

#define LCD_WIDTH 160
#define LCD_HEIGHT 128

#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define LCD_TX_CHUNK_BYTES 32
#define ST7735_MADCTL 0x36

static const char *TAG = "spot_ui";

volatile int counter = 0;

static spi_device_handle_t lcd_spi;
static volatile uint8_t encoder_state = 0;

static const uint8_t font5x7[96][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x5f, 0x00, 0x00},
    {0x00, 0x07, 0x00, 0x07, 0x00}, {0x14, 0x7f, 0x14, 0x7f, 0x14},
    {0x24, 0x2a, 0x7f, 0x2a, 0x12}, {0x23, 0x13, 0x08, 0x64, 0x62},
    {0x36, 0x49, 0x55, 0x22, 0x50}, {0x00, 0x05, 0x03, 0x00, 0x00},
    {0x00, 0x1c, 0x22, 0x41, 0x00}, {0x00, 0x41, 0x22, 0x1c, 0x00},
    {0x14, 0x08, 0x3e, 0x08, 0x14}, {0x08, 0x08, 0x3e, 0x08, 0x08},
    {0x00, 0x50, 0x30, 0x00, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08},
    {0x00, 0x60, 0x60, 0x00, 0x00}, {0x20, 0x10, 0x08, 0x04, 0x02},
    {0x3e, 0x51, 0x49, 0x45, 0x3e}, {0x00, 0x42, 0x7f, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4b, 0x31},
    {0x18, 0x14, 0x12, 0x7f, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3c, 0x4a, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1e},
    {0x00, 0x36, 0x36, 0x00, 0x00}, {0x00, 0x56, 0x36, 0x00, 0x00},
    {0x08, 0x14, 0x22, 0x41, 0x00}, {0x14, 0x14, 0x14, 0x14, 0x14},
    {0x00, 0x41, 0x22, 0x14, 0x08}, {0x02, 0x01, 0x51, 0x09, 0x06},
    {0x32, 0x49, 0x79, 0x41, 0x3e}, {0x7e, 0x11, 0x11, 0x11, 0x7e},
    {0x7f, 0x49, 0x49, 0x49, 0x36}, {0x3e, 0x41, 0x41, 0x41, 0x22},
    {0x7f, 0x41, 0x41, 0x22, 0x1c}, {0x7f, 0x49, 0x49, 0x49, 0x41},
    {0x7f, 0x09, 0x09, 0x09, 0x01}, {0x3e, 0x41, 0x49, 0x49, 0x7a},
    {0x7f, 0x08, 0x08, 0x08, 0x7f}, {0x00, 0x41, 0x7f, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3f, 0x01}, {0x7f, 0x08, 0x14, 0x22, 0x41},
    {0x7f, 0x40, 0x40, 0x40, 0x40}, {0x7f, 0x02, 0x0c, 0x02, 0x7f},
    {0x7f, 0x04, 0x08, 0x10, 0x7f}, {0x3e, 0x41, 0x41, 0x41, 0x3e},
    {0x7f, 0x09, 0x09, 0x09, 0x06}, {0x3e, 0x41, 0x51, 0x21, 0x5e},
    {0x7f, 0x09, 0x19, 0x29, 0x46}, {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7f, 0x01, 0x01}, {0x3f, 0x40, 0x40, 0x40, 0x3f},
    {0x1f, 0x20, 0x40, 0x20, 0x1f}, {0x3f, 0x40, 0x38, 0x40, 0x3f},
    {0x63, 0x14, 0x08, 0x14, 0x63}, {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x61, 0x51, 0x49, 0x45, 0x43}, {0x00, 0x7f, 0x41, 0x41, 0x00},
    {0x02, 0x04, 0x08, 0x10, 0x20}, {0x00, 0x41, 0x41, 0x7f, 0x00},
    {0x04, 0x02, 0x01, 0x02, 0x04}, {0x40, 0x40, 0x40, 0x40, 0x40},
    {0x00, 0x01, 0x02, 0x04, 0x00}, {0x20, 0x54, 0x54, 0x54, 0x78},
    {0x7f, 0x48, 0x44, 0x44, 0x38}, {0x38, 0x44, 0x44, 0x44, 0x20},
    {0x38, 0x44, 0x44, 0x48, 0x7f}, {0x38, 0x54, 0x54, 0x54, 0x18},
    {0x08, 0x7e, 0x09, 0x01, 0x02}, {0x0c, 0x52, 0x52, 0x52, 0x3e},
    {0x7f, 0x08, 0x04, 0x04, 0x78}, {0x00, 0x44, 0x7d, 0x40, 0x00},
    {0x20, 0x40, 0x44, 0x3d, 0x00}, {0x7f, 0x10, 0x28, 0x44, 0x00},
    {0x00, 0x41, 0x7f, 0x40, 0x00}, {0x7c, 0x04, 0x18, 0x04, 0x78},
    {0x7c, 0x08, 0x04, 0x04, 0x78}, {0x38, 0x44, 0x44, 0x44, 0x38},
    {0x7c, 0x14, 0x14, 0x14, 0x08}, {0x08, 0x14, 0x14, 0x18, 0x7c},
    {0x7c, 0x08, 0x04, 0x04, 0x08}, {0x48, 0x54, 0x54, 0x54, 0x20},
    {0x04, 0x3f, 0x44, 0x40, 0x20}, {0x3c, 0x40, 0x40, 0x20, 0x7c},
    {0x1c, 0x20, 0x40, 0x20, 0x1c}, {0x3c, 0x40, 0x30, 0x40, 0x3c},
    {0x44, 0x28, 0x10, 0x28, 0x44}, {0x0c, 0x50, 0x50, 0x50, 0x3c},
    {0x44, 0x64, 0x54, 0x4c, 0x44}, {0x00, 0x08, 0x36, 0x41, 0x00},
    {0x00, 0x00, 0x7f, 0x00, 0x00}, {0x00, 0x41, 0x36, 0x08, 0x00},
    {0x10, 0x08, 0x08, 0x10, 0x08}, {0x00, 0x06, 0x09, 0x09, 0x06}};

static esp_err_t st7735_send_cmd(uint8_t cmd)
{
    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &cmd;
    gpio_set_level(PIN_NUM_DC, 0);
    return spi_device_polling_transmit(lcd_spi, &t);
}

static esp_err_t st7735_send_data(const uint8_t *data, int len)
{
    if (len <= 0) {
        return ESP_OK;
    }
    spi_transaction_t t = {0};
    t.length = len * 8;
    t.tx_buffer = data;
    gpio_set_level(PIN_NUM_DC, 1);
    return spi_device_polling_transmit(lcd_spi, &t);
}

static void st7735_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    st7735_send_cmd(0x2A);
    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;
    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;
    st7735_send_data(data, 4);

    st7735_send_cmd(0x2B);
    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;
    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;
    st7735_send_data(data, 4);

    st7735_send_cmd(0x2C);
}

static void st7735_fill_screen(uint16_t color)
{
    uint16_t chunk[LCD_TX_CHUNK_BYTES / 2];
    uint16_t swapped_color = (uint16_t)((color << 8) | (color >> 8));
    for (int i = 0; i < (int)(LCD_TX_CHUNK_BYTES / 2); i++) {
        chunk[i] = swapped_color;
    }

    st7735_set_addr_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    int total_bytes = LCD_WIDTH * LCD_HEIGHT * 2;
    while (total_bytes > 0) {
        int tx_len = total_bytes > LCD_TX_CHUNK_BYTES ? LCD_TX_CHUNK_BYTES : total_bytes;
        st7735_send_data((const uint8_t *)chunk, tx_len);
        total_bytes -= tx_len;
    }
}

static void st7735_draw_char(uint16_t x, uint16_t y, char c, uint16_t color)
{
    if (c < 32 || c > 127 || x >= LCD_WIDTH || y >= LCD_HEIGHT) {
        return;
    }

    uint8_t glyph_index = (uint8_t)(c - 32);
    uint16_t x1 = x + 5;
    uint16_t y1 = y + 7;
    if (x1 >= LCD_WIDTH) {
        x1 = LCD_WIDTH - 1;
    }
    if (y1 >= LCD_HEIGHT) {
        y1 = LCD_HEIGHT - 1;
    }

    st7735_set_addr_window(x, y, x1, y1);
    uint16_t row_pixel[6];
    uint16_t width = x1 - x + 1;
    uint16_t height = y1 - y + 1;
    uint16_t fg = (uint16_t)((color << 8) | (color >> 8));
    uint16_t bg = (uint16_t)((COLOR_BLACK << 8) | (COLOR_BLACK >> 8));

    for (uint16_t row = 0; row < height; row++) {
        for (uint16_t col = 0; col < width; col++) {
            uint16_t out = bg;
            if (col < 5 && row < 7) {
                if ((font5x7[glyph_index][col] >> row) & 0x01) {
                    out = fg;
                }
            }
            row_pixel[col] = out;
        }
        st7735_send_data((const uint8_t *)row_pixel, width * 2);
    }
}

static void st7735_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color)
{
    while (*str != '\0') {
        if (x + 6 > LCD_WIDTH) {
            break;
        }
        st7735_draw_char(x, y, *str, color);
        x += 6;
        str++;
    }
}

static void st7735_init(void)
{
    ESP_ERROR_CHECK(st7735_send_cmd(0x01));
    vTaskDelay(pdMS_TO_TICKS(150));

    ESP_ERROR_CHECK(st7735_send_cmd(0x11));
    vTaskDelay(pdMS_TO_TICKS(120));

    ESP_ERROR_CHECK(st7735_send_cmd(ST7735_MADCTL));
    uint8_t madctl = 0x60;
    ESP_ERROR_CHECK(st7735_send_data(&madctl, 1));

    ESP_ERROR_CHECK(st7735_send_cmd(0x3A));
    uint8_t color_mode = 0x05;
    ESP_ERROR_CHECK(st7735_send_data(&color_mode, 1));

    ESP_ERROR_CHECK(st7735_send_cmd(0x29));
    vTaskDelay(pdMS_TO_TICKS(20));
}

static void backlight_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t channel_cfg = {
        .gpio_num = PIN_NUM_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 205,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}

static void display_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_TX_CHUNK_BYTES,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(LCD_HOST, &devcfg, &lcd_spi));

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << PIN_NUM_DC,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    if (PIN_NUM_RST >= 0) {
        gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    }

    backlight_init();
    st7735_init();
    st7735_fill_screen(COLOR_BLACK);
    ESP_LOGI(TAG, "Display initialized");
}

static void IRAM_ATTR encoder_isr_handler(void *arg)
{
    (void)arg;
    static const int8_t transition_table[16] = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0,
    };

    uint8_t a = (uint8_t)gpio_get_level(ENC_PIN_A);
    uint8_t b = (uint8_t)gpio_get_level(ENC_PIN_B);
    uint8_t new_state = (a << 1) | b;
    uint8_t idx = (encoder_state << 2) | new_state;
    counter += transition_table[idx];
    encoder_state = new_state;
}

static void encoder_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENC_PIN_A) | (1ULL << ENC_PIN_B) | (1ULL << ENC_PIN_SW),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_set_intr_type(ENC_PIN_SW, GPIO_INTR_DISABLE));

    uint8_t a = (uint8_t)gpio_get_level(ENC_PIN_A);
    uint8_t b = (uint8_t)gpio_get_level(ENC_PIN_B);
    encoder_state = (a << 1) | b;

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC_PIN_A, encoder_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC_PIN_B, encoder_isr_handler, NULL));
    ESP_LOGI(TAG, "Encoder initialized");
}

static void ui_task(void *arg)
{
    (void)arg;
    char line[32];
    TickType_t last_wake = xTaskGetTickCount();
    int last_value = 0x7FFFFFFF;

    while (1) {
        int value = counter;
        if (value != last_value) {
            st7735_fill_screen(COLOR_BLACK);
            st7735_draw_string(4, 20, "ESP32 Spot UI", COLOR_WHITE);
            snprintf(line, sizeof(line), "Counter: %d", value);
            st7735_draw_string(4, 40, line, COLOR_WHITE);
            last_value = value;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    display_init();
    encoder_init();
    xTaskCreate(ui_task, "ui_task", 4096, NULL, 5, NULL);
}
