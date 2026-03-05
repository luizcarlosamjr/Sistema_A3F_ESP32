#include "keypad.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define KEYPAD_ROW_COUNT 4
#define KEYPAD_COL_COUNT 4

// Definição dos pinos
static const gpio_num_t row_pins[KEYPAD_ROW_COUNT] = {14, 27, 26, 25};
static const gpio_num_t col_pins[KEYPAD_COL_COUNT] = {3, 32, 19, 18};

            // Mapa das teclas
static const char key_map[4][4] =
{
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

// Última tecla detectada para evitar repetição
static char last_key = '\0';

esp_err_t keypad_init(void)
{
    // Configurar linhas como saída (em HIGH)
    for (int i = 0; i < KEYPAD_ROW_COUNT; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << row_pins[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE
        };
        gpio_config(&cfg);
        gpio_set_level(row_pins[i], 1);
    }

    // Configurar colunas como entrada com pull-up interno
    for (int i = 0; i < KEYPAD_COL_COUNT; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << col_pins[i]),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE
        };
        gpio_config(&cfg);
    }

    return ESP_OK;
}

char keypad_read_nonblock(void)
{
    for (int row = 0; row < KEYPAD_ROW_COUNT; row++) {

        // Ativa somente a linha atual
        for (int r = 0; r < KEYPAD_ROW_COUNT; r++)
            gpio_set_level(row_pins[r], 1);

        gpio_set_level(row_pins[row], 0);
        vTaskDelay(pdMS_TO_TICKS(3)); // estabilização

        for (int col = 0; col < KEYPAD_COL_COUNT; col++) {

            if (gpio_get_level(col_pins[col]) == 0) {

                char key = key_map[row][col];

                // Evita repetição da mesma tecla
                if (key != last_key) {
                    last_key = key;
                    return key;
                } else {
                    return '\0'; // tecla ainda segurada
                }
            }
        }
    }

    // Nenhuma tecla pressionada
    last_key = '\0';
    return '\0';
}

char keypad_get_key(void)
{
    char key = '\0';

    while (key == '\0') {
        key = keypad_read_nonblock();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Debounce final
    vTaskDelay(pdMS_TO_TICKS(120));
    return key;
}
