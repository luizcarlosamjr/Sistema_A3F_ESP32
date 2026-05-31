#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "freertos/message_buffer.h"
#include "cJSON.h"


#include "wifi.h"
#include "my_mqtt.h"

#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_netif.h"


// biblioteca de periféricos
#include "keypad.h"
#include <ultrasonic.h>
#include "driver/touch_pad.h"
#include "ssd1306.h"
#include "driver/i2c.h"



#define TOUCH_PAD_GPIO13_CHANNEL TOUCH_PAD_NUM4


// Credencial que garante o acesso
#define CREDENCIAL "DD"

// configurações reconhecimento facial
#define MAX_DISTANCE_CM 500 // 5m max
#define TRIGGER_GPIO 5
#define ECHO_GPIO 15

#define TOUCH_THRESHOLD     500
#define TOUCH_MIN_TIME_MS   5000
#define TOUCH_POLL_MS       50


// variáveis globais

SSD1306_t dev;

// tag "SISTEMA"  NO TERMINAL
static const char *TAG = "SISTEMA";

// reconhecimento facial
ultrasonic_sensor_t sensor;
float limiar = 15;
float prox;

// semáforos para o cascateamento das tarefas
SemaphoreHandle_t Idle_bs;
SemaphoreHandle_t ID_bs;
SemaphoreHandle_t Digital_bs;
SemaphoreHandle_t FaceID_bs;

// variáveis para a implementação do wifi e do mqtt
SemaphoreHandle_t wificonnectedSemaphore;
SemaphoreHandle_t mqttconnectedSemaphore;

// semáforo de publicação que libera o broker pra publicar
SemaphoreHandle_t mqtt_pubSemaphore;

// variáveis de acesso para envio
int att_cred[5];
int img_biometria, img_facial;

struct acesso
{
    char att_cred[5];
    int img_biometria;
    int img_facial;
    char timestamp[30];
};

struct acesso autorizado;
struct acesso negado;


// variável de envio

 // é a variável que vai ser testada no Broker antes de publicar
 // staus_pub = 2 -> sem estado
 // status_pub = 1 -> acesso pub de acesso autorizado
 // status_pub = 0 -> acesso pub de acesso negado
int status_pub = 2;





// variáceis do sntp
time_t now;
struct tm timeinfo; 
// ===================================================================================
//





// inicializa o display

void display_init(void) {
    
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

    ssd1306_init(&dev, 128, 64);

    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
}






static const char *TAG_SNTP = "Sincronização SNTP";


RTC_DATA_ATTR static int boot_count = 0;

static void obtain_time(void);
static void initialize_sntp(void);

char strftime_buf[64];

#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_CUSTOM
void sntp_sync_time(struct timeval *tv)
{
   settimeofday(tv, NULL);
   ESP_LOGI(TAG_SNTP, "Time is synchronized from custom code");
   sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
}
#endif

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG_SNTP, "Notification of a time synchronization event");
}




// ===============================================================================================================================
// funções auxiliares


void PreencheAcesso_ID(int status, char id[])
{
    if(status == 1) // autorizado
    {
        autorizado.att_cred[0] = id[0];
        autorizado.att_cred[1] = id[1];
        autorizado.att_cred[2] = id[2];
        autorizado.att_cred[3] = id[3];
        autorizado.att_cred[4] = id[4];
    }
    if(status == 0) // negado
    {
        negado.att_cred[0] = id[0];
        negado.att_cred[1] = id[1];
        negado.att_cred[2] = id[2];
        negado.att_cred[3] = id[3];
        negado.att_cred[4] = id[4];
        negado.img_biometria = -1;
        negado.img_facial = -1;
        ESP_LOGW("TENTATIVA DE ACESSO", "CREDENCIAL %s", negado.att_cred);
        status_pub = 0;
        xSemaphoreGive(mqtt_pubSemaphore);
    }
}

void PreencheAcesso_Digital(int status, int bio)
{
    if(status == 1)  // autorizado
    {
        autorizado.img_biometria = bio;
    }
    if(status == 0) // negado
    {
        negado.att_cred[0] = autorizado.att_cred[0];
        negado.att_cred[1] = autorizado.att_cred[1];
        negado.att_cred[2] = autorizado.att_cred[2];
        negado.att_cred[3] = autorizado.att_cred[3];
        negado.att_cred[4] = autorizado.att_cred[4];
        negado.img_biometria = bio;
        negado.img_facial = -1;
        ESP_LOGW("TENTATIVA DE ACESSO", "CREDENCIAL: %s e Biometria: %d ", negado.att_cred, negado.img_biometria);
        status_pub = 0;
        xSemaphoreGive(mqtt_pubSemaphore);
    }
}

void PreencheAcesso_FaceID(int status, int face)
{
    if(status == 1)  // autorizado
    {
        autorizado.img_facial = face;
        status_pub = 1;
        xSemaphoreGive(mqtt_pubSemaphore);
    }
    if(status == 0) // negado
    {
        negado.att_cred[0] = autorizado.att_cred[0];
        negado.att_cred[1] = autorizado.att_cred[1];
        negado.att_cred[2] = autorizado.att_cred[2];
        negado.att_cred[3] = autorizado.att_cred[3];
        negado.att_cred[4] = autorizado.att_cred[4];
        negado.img_biometria = autorizado.img_biometria;
        negado.img_facial = face;
        ESP_LOGW("TENTATIVA DE ACESSO", "CREDENCIAL: %s, Biometria: %d e FaceID: %d", negado.att_cred, negado.img_biometria, negado.img_facial);
        status_pub = 0;
        xSemaphoreGive(mqtt_pubSemaphore);
    }
}


// ==============================================
// tarefas 


void taskInputIDLE(void *pvParameters)
{

    char linha1[17], linha2[17], linha3[17];
    char key;


    // inicia as structs de acesso de informação zeradas
    memset(autorizado.att_cred, 0, sizeof(autorizado.att_cred));
    autorizado.img_biometria = 0;
    autorizado.img_facial = 0;


    memset(negado.att_cred, 0, sizeof(negado.att_cred));
    negado.img_biometria = 0;
    negado.img_facial = 0;



    while (1)
    {
        xSemaphoreTake(Idle_bs, portMAX_DELAY);
        ESP_LOGI("SISTEMA", "Pressione A para entrar");
        

        ssd1306_clear_screen(&dev, false);
        snprintf(linha1, sizeof(linha1), "SISTEMA:");
        snprintf(linha2, sizeof(linha2), "Pressione A");
        ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
        ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);


        key = 0;
        key = keypad_get_key();
        if (key == 'A') 
        {
            ESP_LOGI("SISTEMA", "A Pressionado");

            ssd1306_clear_screen(&dev, false);
            snprintf(linha1, sizeof(linha1), "SISTEMA:");
            snprintf(linha2, sizeof(linha2), "'A' Pressionado'");
            snprintf(linha3, sizeof(linha3), "Aguarde a ID");
            ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
            ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);
            ssd1306_display_text(&dev, 4, linha3, strlen(linha3), false);

            xSemaphoreGive(ID_bs);
        }else{
            ESP_LOGI("SISTEMA","Tecla incorreta");

            ssd1306_clear_screen(&dev, false);
            snprintf(linha1, sizeof(linha1), "SISTEMA:");
            snprintf(linha2, sizeof(linha2), "Tecla Incorreta");
            ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
            ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);
        }

    }
}

void taskInputid(void *pvParameters) 
{

    char linha1[17], linha2[17], linha3[17];
    char key_buffer[5] = {0};
    int index = 0;
    
    
    while (1)
    {
        xSemaphoreTake(ID_bs, portMAX_DELAY);

        ESP_LOGI("ID", "Digite a credencial: ");
        ssd1306_clear_screen(&dev, false);
        snprintf(linha1, sizeof(linha1), "Identificação");
        snprintf(linha2, sizeof(linha2), "Digite sua ID:");
        ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
        ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);
        index = 0;
        memset(key_buffer, 0, sizeof(key_buffer));



    while (1)
    {  
        char key = keypad_get_key();

        if (key != 0)
        {
            ESP_LOGI(TAG, "Tecla pressionada: %c", key);
            // Limpa com 'B'

            if (key == 'B')
            {
                index = 0;
                memset(key_buffer, 0, sizeof(key_buffer));
                ESP_LOGI(TAG, "Buffer limpo");
                ssd1306_clear_screen(&dev, false);
                snprintf(linha1, sizeof(linha1), "Identificaçao");
                snprintf(linha2, sizeof(linha2), "ID Limpa");
                ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
                ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);
                
            }

            else if (key == 'A')
            {
                // Confirma credencial
                if (strcmp(key_buffer, CREDENCIAL) == 0)
                {
                    PreencheAcesso_ID(1, key_buffer);
                    ESP_LOGW("Credencial", "%s \n", autorizado.att_cred); // isso foi um teste, retirar depois
                    
                    ESP_LOGI(TAG, "Acesso Valido");
                    ssd1306_clear_screen(&dev, false);
                    snprintf(linha1, sizeof(linha1), "Identificaçao");
                    snprintf(linha2, sizeof(linha2), "ID Valida");
                    ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
                    ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);
                    xSemaphoreGive(Digital_bs);
                }
                else
                {
                    PreencheAcesso_ID(0, key_buffer);

                    ESP_LOGW(TAG, "Credencial inválida");

                    ssd1306_clear_screen(&dev, false);
                    snprintf(linha1, sizeof(linha1), "Identificação");
                    snprintf(linha2, sizeof(linha2), "ID Invalida:");
                    ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
                    ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    ESP_LOGI("Sistema", "Acesso Negado");
                    ssd1306_clear_screen(&dev, false);
                    snprintf(linha1, sizeof(linha1), "SISTEMA:");
                    snprintf(linha2, sizeof(linha2), "ACESSO NEGADO");
                    ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
                    ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    xSemaphoreGive(Idle_bs);
                    break;
                    // resetar a verificação pro idle e talvez gerar o log de acesso negado 
                }
                // Reseta buffer
                index = 0;
                memset(key_buffer, 0, sizeof(key_buffer));
                break;   // ← devolve controle da task
            }
            else if (index < 4)
            {
                // Armazena apenas 4 dígitos
                key_buffer[index++] = key;
                snprintf(linha3, sizeof(linha3), key_buffer);
                ssd1306_display_text(&dev, 4, linha3, strlen(linha3), false);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}    
   
}

void taskInputDigital(void *pvParameters) 
{

    char linha1[17], linha2[17];             
    uint16_t val;
    uint16_t filtered_value = 0;
    uint16_t raw_value_touch = 0;   

    bool touching = false;
    uint32_t touch_start = 0;

    while (1)
    {
        
        xSemaphoreTake(Digital_bs, portMAX_DELAY); 
        ESP_LOGI("Biometria", "Valide a biometria");

        ssd1306_clear_screen(&dev, false);
        snprintf(linha1, sizeof(linha1), "Biometria");
        snprintf(linha2, sizeof(linha2), "Valide a Digital");
        ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
        ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);

        
        touching = false;
        touch_start = 0;

        while(1)
        {
            touch_pad_read_raw_data(TOUCH_PAD_GPIO13_CHANNEL, &raw_value_touch);
            touch_pad_read_filtered(TOUCH_PAD_GPIO13_CHANNEL, &filtered_value);
            touch_pad_read(TOUCH_PAD_GPIO13_CHANNEL, &val);
            vTaskDelay(1000 / portTICK_PERIOD_MS);

             if (raw_value_touch < TOUCH_THRESHOLD)
            {
                if (!touching)
                {
                    // 👉 dedo acabou de tocar
                    touching = true;
                    touch_start = xTaskGetTickCount();

                    ssd1306_clear_screen(&dev, false);
                    snprintf(linha1, sizeof(linha1), "Biometria");
                    snprintf(linha2, sizeof(linha2), "Verificando...");
                    ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
                    ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);
                }

                // 👉 dedo continua tocando
                if ((xTaskGetTickCount() - touch_start) >= pdMS_TO_TICKS(TOUCH_MIN_TIME_MS))
                {   
                    
                    
                    PreencheAcesso_Digital(1, raw_value_touch);
                    ssd1306_clear_screen(&dev, false);
                    snprintf(linha1, sizeof(linha1), "Biometria");
                    snprintf(linha2, sizeof(linha2), "Digital Valida");
                    ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
                    ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);

                    
                    PreencheAcesso_Digital(1, raw_value_touch); 
                    xSemaphoreGive(FaceID_bs);
                    break;
                }
            } else
            {
                // 👉 dedo saiu antes do tempo mínimo
                if (touching)

                {   
                    
                    ESP_LOGW("Biometria", "Toque muito curto");
                    ESP_LOGW("Biometria", "Digital Inválida");
                    ssd1306_clear_screen(&dev, false);
                    snprintf(linha1, sizeof(linha1), "Biometria");
                    snprintf(linha2, sizeof(linha2), "Invalida");
                    ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
                    ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    ssd1306_clear_screen(&dev, false);
                    snprintf(linha1, sizeof(linha1), "SISTEMA:");
                    snprintf(linha2, sizeof(linha2), "ACESSO NEGADO");
                    ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
                    ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                     touching = false;
                    touch_start = 0;
                    PreencheAcesso_Digital(0, raw_value_touch);
                    xSemaphoreGive(Idle_bs);
                    break;
                }
            }
        }
    }
}
        
      
        
        
        
 

void taskInputFaceID(void *pvParameters)
{

   char linha1[17], linha2[17], linha3[17];

   while (1)
    {
      xSemaphoreTake(FaceID_bs, portMAX_DELAY);
      ESP_LOGI("FaceID", "Faça o reconhecimento facial");


      while(1)
      {
           
            
            
            //testa se tem algum erro
            esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &prox);

            if (res != ESP_OK)
            {

                // no switch case testa se tem algum erro de inicialização  do sensor/leitura e qual é
                printf("Error %d: ", res);
                switch (res)

                {
                    case ESP_ERR_ULTRASONIC_PING:
                        printf("Cannot ping (device is in invalid state)\n");
                        break;
                    case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                        printf("Ping timeout (no device found)\n");
                        break;
                    case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                        printf("Echo timeout (i.e. medida too big)\n");
                        break;
                    default:
                        printf("%s\n", esp_err_to_name(res));
                }
            }

            if (prox*100 < limiar)   // exemplo de limiar
            {

                    ssd1306_clear_screen(&dev, false);
                    snprintf(linha1, sizeof(linha1), "FaceID");
                    snprintf(linha2, sizeof(linha2), "Verificando...");
                    ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
                    ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    
                    ESP_LOGI("FaceID", "Reconhecimento facial OK");
                    ssd1306_clear_screen(&dev, false);
                    snprintf(linha1, sizeof(linha1), "FaceID");
                    snprintf(linha2, sizeof(linha2), "Reconhecimento");
                    snprintf(linha3, sizeof(linha3), "facial OK");
                    ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
                    ssd1306_display_text(&dev, 4, linha2, strlen(linha2), false);
                    ssd1306_display_text(&dev, 5, linha3, strlen(linha3), false);

                    PreencheAcesso_FaceID(1, prox*100);

                    vTaskDelay(pdMS_TO_TICKS(1500));

                    ESP_LOGI("Sistema", "Acesso liberado");
                    ssd1306_clear_screen(&dev, false);
                    snprintf(linha1, sizeof(linha1), "SISTEMA:");
                    snprintf(linha2, sizeof(linha2), "ACESSO LIBERADO");
                    ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
                    ssd1306_display_text(&dev, 3, linha2, strlen(linha2), false);


                    vTaskDelay(pdMS_TO_TICKS(3000));

                    // Aqui a struct de acesso vai estar completa e será enviada via mqtt

                    xSemaphoreGive(Idle_bs);
                    break;   // ← devolve controle da task
            } 
            /*if (prox*100>20 || prox*100<60)
            {
                PreencheAcesso_FaceID(0, prox*100);
                
                // novamente deixei o else para simbolizar um acesso facial recusado, quando na prática não tem
                // como recusar o reconhecimento por distância sem possivelmente causar um erro 
                break;

            }*/
            ESP_LOGW("FaceID", "Aproxime-se da câmera. Você está a: %0.04f cm\n", prox*100);
            ssd1306_clear_screen(&dev, false);
            snprintf(linha1, sizeof(linha1), "FaceID");
            snprintf(linha2, sizeof(linha2), "Aproxime-se");
            snprintf(linha3, sizeof(linha3), "da camera");
            ssd1306_display_text(&dev, 2, linha1, strlen(linha1), false);
            ssd1306_display_text(&dev, 4, linha2, strlen(linha2), false);
            ssd1306_display_text(&dev, 5, linha3, strlen(linha3), false);  
            vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
}

    // inicializad o MQTT
void wifiConnected(void *params)
{
    while (1)
    {
        if (xSemaphoreTake(wificonnectedSemaphore, portMAX_DELAY))
        {
            mqtt_start();
            xSemaphoreTake(mqttconnectedSemaphore, portMAX_DELAY);
            xSemaphoreGive(Idle_bs);
            xSemaphoreGive(mqttconnectedSemaphore);
        }
    }
}

void comunicacao_broker(void *params)
{

        ESP_LOGI("MQTT", "Subscrição no Tópico estabelecida");
        char msg[250];

        xSemaphoreTake(mqttconnectedSemaphore, portMAX_DELAY);


        while (1)
        
        {
            xSemaphoreTake(mqtt_pubSemaphore, portMAX_DELAY);



            if (status_pub == 1)
            {
            // pub se autorizado
                sprintf(msg, "Tentativa de acesso AUTORIZADA com Credencial: %s; \nBiometria: %d.png; \n FaceID: %d.png em %s \n",
                autorizado.att_cred,
                autorizado.img_biometria,
                autorizado.img_facial,
                strftime_buf);
                ESP_LOGI("broker terminal", "%s", strftime_buf);
                mqtt_publish("CEL080/3FA1_Test", msg);
                
                memset(msg, 0, sizeof(msg));
                status_pub = 2;
                memset(strftime_buf, 0, sizeof(strftime_buf));

                //xSemaphoreGive(Idle_bs);
            }


            if (status_pub == 0)
            {
                // pub se negado
                sprintf(msg, "Tentativa de acesso NEGADA com Credencial: %s; \nBiometria: %d.png; \n FaceID: %d.png em %s \n",
                negado.att_cred,
                negado.img_biometria,
                negado.img_facial,
                strftime_buf);
                ESP_LOGI("broker terminal", "%s", strftime_buf);
                mqtt_publish("CEL080/3FA1_Test", msg);

                memset(msg, 0, sizeof(msg));
                status_pub = 2;
                memset(strftime_buf, 0, sizeof(strftime_buf));


                //xSemaphoreGive(Idle_bs);    

            }

            vTaskDelay(pdMS_TO_TICKS(500));
        }

}


void app_main(void)
{

  // inicializa periféricos
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);


  touch_pad_init();
  touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
  touch_pad_config(TOUCH_PAD_GPIO13_CHANNEL, -1);
  touch_pad_filter_start(10);
 
  keypad_init();
  display_init();

  sensor.trigger_pin = TRIGGER_GPIO;
  sensor.echo_pin = ECHO_GPIO;
  ultrasonic_init(&sensor);

  // Criação dos semáforos
  Idle_bs = xSemaphoreCreateBinary();
  xSemaphoreTake(Idle_bs, 0);
  ID_bs = xSemaphoreCreateBinary();
  xSemaphoreTake(ID_bs, 0);
  Digital_bs = xSemaphoreCreateBinary();
  xSemaphoreTake(Digital_bs, 0);
  FaceID_bs = xSemaphoreCreateBinary();
  xSemaphoreTake(FaceID_bs, 0);

  wificonnectedSemaphore = xSemaphoreCreateBinary();
  mqttconnectedSemaphore = xSemaphoreCreateBinary();
  mqtt_pubSemaphore = xSemaphoreCreateBinary();


  
  wifi_start();


  // coisas SNTP
    ++boot_count;
    ESP_LOGI(TAG_SNTP, "Boot count: %d", boot_count);


    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG_SNTP, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }

    if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
        struct timeval outdelta;
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
            adjtime(NULL, &outdelta);
            ESP_LOGI(TAG_SNTP, "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
                        (long)outdelta.tv_sec,
                        outdelta.tv_usec/1000,
                        outdelta.tv_usec%1000);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }


  // Criação das Tarefas
  xTaskCreate(taskInputIDLE, "InputIDLE", 2048, NULL, 1, NULL);
  xTaskCreate(taskInputid, "InputID", 2048, NULL, 1, NULL);
  xTaskCreate(taskInputDigital, "InputDigital", 2048, NULL, 1, NULL);
  xTaskCreate(taskInputFaceID, "InputFaceID", 2048, NULL, 1, NULL);

  // Conexão mqtt
  xTaskCreate(wifiConnected, "Conexao MQTT", 4096, NULL, 2, NULL);
  xTaskCreate(comunicacao_broker, "Comunicacao com o Broker", 4096, NULL, 2, NULL);

}


// funções sntp

static void obtain_time(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());

    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG_SNTP, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }


    // liberar o semáforo do mqtt aqui
    time(&now);
    localtime_r(&now, &timeinfo);

}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG_SNTP, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();
}