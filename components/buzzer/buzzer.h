#ifndef BUZZER_H
#define BUZZER_H

#include "esp_err.h"

#define BUZZER_GPIO_PIN     4
#define BUZZER_LEDC_CHANNEL LEDC_CHANNEL_0
#define BUZZER_LEDC_TIMER   LEDC_TIMER_0

esp_err_t buzzer_init(void);
esp_err_t buzzer_start_alarm(void);
esp_err_t buzzer_stop_alarm(void);

#endif
