#include <stdbool.h>
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
#define COLOR_NAVY 0x0010
#define COLOR_DARKGRAY 0x4208
#define COLOR_CYAN 0x07FF
#define COLOR_YELLOW 0xFFE0
#define COLOR_GREEN 0x07E0
#define COLOR_ORANGE 0xFD20
#define COLOR_BLUE 0x03BF

#define LCD_TX_CHUNK_BYTES 32
#define ST7735_MADCTL 0x36

static const char *TAG = "spot_ui";

volatile int counter = 0;

static spi_device_handle_t lcd_spi;
static volatile uint8_t encoder_state = 0;
static volatile int8_t encoder_delta = 0;

typedef enum {
    SCREEN_MAIN = 0,
    SCREEN_SETTINGS,
} screen_id_t;

typedef enum {
    MAIN_PULSE1 = 0,
    MAIN_PULSE2,
    MAIN_INTERVAL,
    MAIN_AUTO_WELD,
    MAIN_SETTINGS_ICON,
    MAIN_CHARGE_V,
    MAIN_COUNT,
} main_field_t;

typedef enum {
    SET_CAP_CHARGE = 0,
    SET_MAX_CHARGE_CURRENT,
    SET_MAX_CHARGE_POWER,
    SET_BUZZER,
    SET_SAVE_MODE,
    SET_EXIT,
    SET_COUNT,
} setting_item_t;

typedef struct {
    screen_id_t screen;
    bool edit_mode;
    int main_selected;
    int settings_selected;

    int pulse1_tenths;
    int pulse2_tenths;
    int interval_tenths;
    int auto_weld_tenths;
    int charge_cent;

    int cap_charge_on;
    int max_charge_current_tenths;
    int max_charge_power;
    int buzzer_on;
    int save_mode;
} ui_state_t;

static ui_state_t g_ui = {
    .screen = SCREEN_MAIN,
    .edit_mode = false,
    .main_selected = MAIN_PULSE1,
    .settings_selected = SET_CAP_CHARGE,

    .pulse1_tenths = 25,
    .pulse2_tenths = 25,
    .interval_tenths = 10,
    .auto_weld_tenths = 8,
    .charge_cent = 532,

    .cap_charge_on = 1,
    .max_charge_current_tenths = 100,
    .max_charge_power = 60,
    .buzzer_on = 1,
    .save_mode = 0,
};

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

static void st7735_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT || w == 0 || h == 0) {
        return;
    }

    uint16_t x1 = x + w - 1;
    uint16_t y1 = y + h - 1;
    if (x1 >= LCD_WIDTH) {
        x1 = LCD_WIDTH - 1;
    }
    if (y1 >= LCD_HEIGHT) {
        y1 = LCD_HEIGHT - 1;
    }

    uint16_t chunk[LCD_TX_CHUNK_BYTES / 2];
    uint16_t swapped = (uint16_t)((color << 8) | (color >> 8));
    for (int i = 0; i < (int)(LCD_TX_CHUNK_BYTES / 2); i++) {
        chunk[i] = swapped;
    }

    st7735_set_addr_window(x, y, x1, y1);
    int total_bytes = (x1 - x + 1) * (y1 - y + 1) * 2;
    while (total_bytes > 0) {
        int tx_len = total_bytes > LCD_TX_CHUNK_BYTES ? LCD_TX_CHUNK_BYTES : total_bytes;
        st7735_send_data((const uint8_t *)chunk, tx_len);
        total_bytes -= tx_len;
    }
}

static void st7735_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (w < 2 || h < 2) {
        return;
    }
    st7735_fill_rect(x, y, w, 1, color);
    st7735_fill_rect(x, y + h - 1, w, 1, color);
    st7735_fill_rect(x, y, 1, h, color);
    st7735_fill_rect(x + w - 1, y, 1, h, color);
}

static void st7735_fill_screen(uint16_t color)
{
    st7735_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, color);
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
    uint16_t bg = 0;

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
    static const int8_t table[16] = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0,
    };

    uint8_t a = (uint8_t)gpio_get_level(ENC_PIN_A);
    uint8_t b = (uint8_t)gpio_get_level(ENC_PIN_B);
    uint8_t state = (a << 1) | b;
    uint8_t idx = (encoder_state << 2) | state;
    int8_t step = table[idx];
    encoder_delta += step;
    counter += step;
    encoder_state = state;
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

static void draw_box(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t fill, bool selected, bool editing)
{
    st7735_fill_rect(x, y, w, h, fill);
    st7735_draw_rect(x, y, w, h, selected ? COLOR_WHITE : COLOR_DARKGRAY);
    if (editing && selected && w > 4 && h > 4) {
        st7735_draw_rect(x + 2, y + 2, w - 4, h - 4, COLOR_YELLOW);
    }
}

static void format_1decimal(char *out, size_t len, int tenths)
{
    int whole = tenths / 10;
    int frac = tenths >= 0 ? tenths % 10 : -(tenths % 10);
    snprintf(out, len, "%d.%d", whole, frac);
}

static void format_2decimal(char *out, size_t len, int cent)
{
    int whole = cent / 100;
    int frac = cent >= 0 ? cent % 100 : -(cent % 100);
    snprintf(out, len, "%d.%02d", whole, frac);
}

static void draw_main_screen(const ui_state_t *s)
{
    char buf[20];

    st7735_fill_screen(COLOR_BLACK);

    draw_box(2, 2, 36, 40, COLOR_GREEN, s->main_selected == MAIN_PULSE1, s->edit_mode);
    st7735_draw_string(6, 18, "P1", COLOR_BLACK);

    draw_box(40, 2, 36, 40, COLOR_BLUE, s->main_selected == MAIN_PULSE2, s->edit_mode);
    st7735_draw_string(44, 18, "P2", COLOR_WHITE);

    draw_box(78, 2, 36, 40, COLOR_CYAN, s->main_selected == MAIN_INTERVAL, s->edit_mode);
    st7735_draw_string(82, 18, "INT", COLOR_BLACK);

    draw_box(116, 2, 42, 40, COLOR_ORANGE, s->main_selected == MAIN_SETTINGS_ICON, s->edit_mode);
    st7735_draw_string(121, 10, "SET", COLOR_BLACK);
    st7735_draw_string(121, 22, "ICON", COLOR_BLACK);

    draw_box(2, 44, 36, 40, COLOR_CYAN, s->main_selected == MAIN_AUTO_WELD, s->edit_mode);
    st7735_draw_string(6, 56, "AUTO", COLOR_BLACK);
    st7735_draw_string(6, 68, "WELD", COLOR_BLACK);

    draw_box(40, 44, 36, 40, COLOR_DARKGRAY, s->main_selected == MAIN_CHARGE_V, s->edit_mode);
    st7735_draw_string(44, 56, "CHG", COLOR_WHITE);
    st7735_draw_string(44, 68, "V", COLOR_WHITE);

    draw_box(78, 44, 80, 40, COLOR_NAVY, false, false);
    st7735_draw_string(82, 50, "TOP AREA", COLOR_WHITE);
    st7735_draw_string(82, 62, "2/3 SCREEN", COLOR_WHITE);

    st7735_draw_rect(2, 86, 156, 40, COLOR_DARKGRAY);
    format_1decimal(buf, sizeof(buf), s->pulse1_tenths);
    st7735_draw_string(6, 90, "P1:", COLOR_GREEN);
    st7735_draw_string(30, 90, buf, COLOR_WHITE);
    st7735_draw_string(52, 90, "ms", COLOR_YELLOW);

    format_1decimal(buf, sizeof(buf), s->pulse2_tenths);
    st7735_draw_string(76, 90, "P2:", COLOR_BLUE);
    st7735_draw_string(100, 90, buf, COLOR_WHITE);
    st7735_draw_string(122, 90, "ms", COLOR_YELLOW);

    format_1decimal(buf, sizeof(buf), s->interval_tenths);
    st7735_draw_string(6, 102, "INT:", COLOR_CYAN);
    st7735_draw_string(30, 102, buf, COLOR_WHITE);
    st7735_draw_string(52, 102, "ms", COLOR_YELLOW);

    format_1decimal(buf, sizeof(buf), s->auto_weld_tenths);
    st7735_draw_string(76, 102, "AUTO:", COLOR_CYAN);
    st7735_draw_string(106, 102, buf, COLOR_WHITE);
    st7735_draw_string(128, 102, "s", COLOR_YELLOW);

    format_2decimal(buf, sizeof(buf), s->charge_cent);
    st7735_draw_string(6, 114, "CHG V:", COLOR_YELLOW);
    st7735_draw_string(42, 114, buf, COLOR_WHITE);
    st7735_draw_string(70, 114, "V", COLOR_YELLOW);

    st7735_draw_string(88, 114, s->edit_mode ? "EDIT" : "NAV", COLOR_YELLOW);
}

static void draw_settings_screen(const ui_state_t *s)
{
    char val[16];
    const char *names[SET_COUNT] = {
        "CAP CHARGE",
        "MAX I",
        "MAX P",
        "BUZZER",
        "SAVE",
        "EXIT",
    };

    st7735_fill_screen(COLOR_BLACK);
    st7735_fill_rect(0, 0, 160, 12, COLOR_BLUE);
    st7735_draw_string(4, 2, "SETTINGS", COLOR_WHITE);

    for (int i = 0; i < SET_COUNT; i++) {
        int y = 14 + i * 18;
        bool sel = (i == s->settings_selected);
        if (sel) {
            st7735_fill_rect(0, y - 1, 160, 16, COLOR_NAVY);
        }

        st7735_draw_string(4, y + 3, names[i], COLOR_WHITE);

        val[0] = '\0';
        switch (i) {
            case SET_CAP_CHARGE:
                snprintf(val, sizeof(val), "%s", s->cap_charge_on ? "ON" : "OFF");
                break;
            case SET_MAX_CHARGE_CURRENT:
                format_1decimal(val, sizeof(val), s->max_charge_current_tenths);
                break;
            case SET_MAX_CHARGE_POWER:
                snprintf(val, sizeof(val), "%dW", s->max_charge_power);
                break;
            case SET_BUZZER:
                snprintf(val, sizeof(val), "%s", s->buzzer_on ? "ON" : "OFF");
                break;
            case SET_SAVE_MODE:
                snprintf(val, sizeof(val), "%s", s->save_mode ? "SAVE" : "NO SAVE");
                break;
            case SET_EXIT:
                snprintf(val, sizeof(val), "BACK");
                break;
            default:
                break;
        }

        st7735_draw_string(98, y + 3, val, sel && s->edit_mode ? COLOR_YELLOW : COLOR_WHITE);
        if (sel && s->edit_mode && i != SET_EXIT) {
            st7735_draw_string(84, y + 3, "*", COLOR_YELLOW);
        }
    }
}

static void render_ui(const ui_state_t *s)
{
    if (s->screen == SCREEN_MAIN) {
        draw_main_screen(s);
    } else {
        draw_settings_screen(s);
    }
}

static int clamp_i(int v, int lo, int hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static bool apply_main_steps(ui_state_t *s, int steps)
{
    if (steps == 0) {
        return false;
    }

    if (!s->edit_mode) {
        int n = s->main_selected + steps;
        while (n < 0) {
            n += MAIN_COUNT;
        }
        s->main_selected = n % MAIN_COUNT;
        return true;
    }

    switch (s->main_selected) {
        case MAIN_PULSE1:
            s->pulse1_tenths = clamp_i(s->pulse1_tenths + steps, 0, 99);
            break;
        case MAIN_PULSE2:
            s->pulse2_tenths = clamp_i(s->pulse2_tenths + steps, 0, 99);
            break;
        case MAIN_INTERVAL:
            s->interval_tenths = clamp_i(s->interval_tenths + steps, 0, 99);
            break;
        case MAIN_AUTO_WELD:
            s->auto_weld_tenths = clamp_i(s->auto_weld_tenths + steps, 0, 99);
            break;
        case MAIN_CHARGE_V:
            s->charge_cent = clamp_i(s->charge_cent + steps, 300, 700);
            break;
        case MAIN_SETTINGS_ICON:
        default:
            break;
    }
    return true;
}

static bool apply_settings_steps(ui_state_t *s, int steps)
{
    if (steps == 0) {
        return false;
    }

    if (!s->edit_mode) {
        int n = s->settings_selected + steps;
        while (n < 0) {
            n += SET_COUNT;
        }
        s->settings_selected = n % SET_COUNT;
        return true;
    }

    switch (s->settings_selected) {
        case SET_CAP_CHARGE:
            if (steps != 0) {
                s->cap_charge_on = !s->cap_charge_on;
            }
            break;
        case SET_MAX_CHARGE_CURRENT:
            s->max_charge_current_tenths = clamp_i(s->max_charge_current_tenths + steps, 10, 200);
            break;
        case SET_MAX_CHARGE_POWER:
            s->max_charge_power = clamp_i(s->max_charge_power + steps, 10, 120);
            break;
        case SET_BUZZER:
            if (steps != 0) {
                s->buzzer_on = !s->buzzer_on;
            }
            break;
        case SET_SAVE_MODE:
            if (steps != 0) {
                s->save_mode = !s->save_mode;
            }
            break;
        case SET_EXIT:
        default:
            break;
    }
    return true;
}

static bool apply_encoder_steps(ui_state_t *s, int steps)
{
    if (s->screen == SCREEN_MAIN) {
        return apply_main_steps(s, steps);
    }
    return apply_settings_steps(s, steps);
}

static bool handle_button_press(ui_state_t *s)
{
    if (s->screen == SCREEN_MAIN) {
        if (!s->edit_mode && s->main_selected == MAIN_SETTINGS_ICON) {
            s->screen = SCREEN_SETTINGS;
            s->edit_mode = false;
            return true;
        }
        s->edit_mode = !s->edit_mode;
        return true;
    }

    if (!s->edit_mode && s->settings_selected == SET_EXIT) {
        s->screen = SCREEN_MAIN;
        s->edit_mode = false;
        return true;
    }

    s->edit_mode = !s->edit_mode;
    return true;
}

static void ui_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    int8_t accum = 0;
    int sw_last = 1;
    bool dirty = true;

    while (1) {
        int8_t d = encoder_delta;
        if (d != 0) {
            d = encoder_delta;
            encoder_delta = 0;

            accum = (int8_t)(accum + d);
            int steps = accum / 4;
            accum = (int8_t)(accum % 4);
            if (apply_encoder_steps(&g_ui, steps)) {
                dirty = true;
            }
        }

        int sw = gpio_get_level(ENC_PIN_SW);
        if (sw_last == 1 && sw == 0) {
            if (handle_button_press(&g_ui)) {
                dirty = true;
            }
            ESP_LOGI(TAG, "screen=%d mode=%s main=%d set=%d", g_ui.screen,
                     g_ui.edit_mode ? "EDIT" : "NAV", g_ui.main_selected, g_ui.settings_selected);
        }
        sw_last = sw;

        if (dirty) {
            render_ui(&g_ui);
            dirty = false;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(30));
    }
}

void app_main(void)
{
    display_init();
    encoder_init();
    xTaskCreate(ui_task, "ui_task", 6144, NULL, 5, NULL);
}
