#ifndef KEYPAD_H
#define KEYPAD_H

#include "esp_err.h"

// Inicializa o teclado matricial
esp_err_t keypad_init(void);

// Faz leitura de uma tecla (bloqueante)
// Retorna:
//   - caractere do teclado ('0'–'9', 'A'–'D', '*', '#')
//   - '\0' caso nada seja pressionado
char keypad_get_key(void);

// Leitura não bloqueante (retorna imediatamente)
// Igual ao get_key(), mas não espera pressionar
char keypad_read_nonblock(void);

#endif
