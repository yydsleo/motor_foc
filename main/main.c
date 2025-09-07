#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "foc_motor.h"
#include "esp_rom_gpio.h"

static const char* TAG = "main";

void app_main(void)
{
    foc_motor_init();

    while(1)
    {
        motor_run(get_left_motor());
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}