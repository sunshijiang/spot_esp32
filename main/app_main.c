#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ui.h"
#include "display.h"
#include "encoder.h"
#include "weld.h"
#include "charge.h"
#include "adc_monitor.h"
#include "buzzer.h"
#include "st7735s.h"
#include "ina226.h"

static const char *TAG = "app_main";

static void ui_task(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        ESP_LOGI(TAG, "ui_task running");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "spot-esp32 skeleton startup");

    st7735s_init();
    ina226_init();
    display_init();
    ui_init();
    encoder_init();
    weld_init();
    charge_init();
    adc_monitor_init();
    buzzer_init();

    xTaskCreate(ui_task, "ui_task", 4096, NULL, 5, NULL);
}
