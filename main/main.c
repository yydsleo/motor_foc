#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "motor_foc.h"
#include "esp_rom_gpio.h"

static const char* TAG = "main";

void foc_driver() {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        motor_run(get_left_motor());
    }
}
void app_main(void)
{
    TaskHandle_t foc_driver_task_handle;
    foc_motor_init();
    xTaskCreatePinnedToCore(&foc_driver, "foc_driver", 4 * 1024, NULL, 5, &foc_driver_task_handle, 1);

    while(1)
    {
        // vTaskDelay(pdMS_TO_TICKS(1));
        xTaskNotifyGive(foc_driver_task_handle);
        /*
        BaseType_t status = pdFALSE;
        vTaskNotifyGiveFromISR(foc_driver_task_handle, &status);
        if (status == pdTRUE) {
            YIELD_FROM_ISR();
        }
        */
    }
}