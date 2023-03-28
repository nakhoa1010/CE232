#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#define LED 2 // GPIO pin được sử dụng để điều khiển LED

void app_main()
{
    gpio_pad_select_gpio(LED); //Chọn pad làm chức năng GPIO từ IOMUX.
    gpio_set_direction(LED, GPIO_MODE_OUTPUT); // pinMode(LED, OUTPUT);

    while (1) {
        gpio_set_level(LED, 1); // digitalWrite(LED, HIGH);
        printf("LED on\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);//Delay 1s
        gpio_set_level(LED, 0);
        printf("LED off\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}