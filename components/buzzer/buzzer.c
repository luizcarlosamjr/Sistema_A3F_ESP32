#include "buzzer.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "buzzer";

static TaskHandle_t alarm_task_handle = NULL;
static bool alarm_running = false;

// Frequência fixa (buzzer ativo)
#define BUZZER_ACTIVE_FREQ  2500    // Não importa muito, mas precisa ser > 0
#define BUZZER_DUTY         4000    // Duty médio (~12.5%)

static void buzzer_alarm_task(void *arg)
{
    while (alarm_running)
    {
        // Bip 1
        ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Pausa curta
        ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Bip 2
        ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Pausa longa
        ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Garantir buzzer desligado
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);

    vTaskDelete(NULL);
}

esp_err_t buzzer_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = BUZZER_LEDC_TIMER,
        .freq_hz          = BUZZER_ACTIVE_FREQ
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .gpio_num       = BUZZER_GPIO_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = BUZZER_LEDC_CHANNEL,
        .timer_sel      = BUZZER_LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    ESP_LOGI(TAG, "Buzzer inicializado");
    return ESP_OK;
}

esp_err_t buzzer_start_alarm(void)
{
    if (alarm_running) {
        ESP_LOGW(TAG, "Alarme já ativo");
        return ESP_OK;
    }

    alarm_running = true;

    if (xTaskCreate(buzzer_alarm_task, "buzzer_alarm_task", 2048, NULL, 5, &alarm_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Falha ao criar a task do alarme");
        alarm_running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Alarme iniciado");
    return ESP_OK;
}

esp_err_t buzzer_stop_alarm(void)
{
    if (!alarm_running) {
        ESP_LOGW(TAG, "Alarme já parado");
        return ESP_OK;
    }

    alarm_running = false;

    // Espera task finalizar
    while (alarm_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);

    ESP_LOGI(TAG, "Alarme parado");
    return ESP_OK;
}
