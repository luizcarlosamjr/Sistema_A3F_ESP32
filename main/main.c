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
#define CREDENTIAL "DD"

// configurações sensor ultrassônico simulando o reconhecimento facial
#define MAX_DISTANCE_CM 500 // 5m max
#define TRIGGER_GPIO 5
#define ECHO_GPIO 15

// configurações sensor de toque simulando o reconhecimento biométrico
#define TOUCH_THRESHOLD     500
#define TOUCH_MIN_TIME_MS   5000
#define TOUCH_POLL_MS       50


// variáveis globais

SSD1306_t dev;

// tag "SISTEMA"  NO TERMINAL
static const char *TAG = "SISTEMA";

// variável do sensor ultrassonico
ultrasonic_sensor_t sensor;
float limiar = 15;
float prox;

// semáforos para o cascateamento das tarefas
SemaphoreHandle_t idleSemaphore;
SemaphoreHandle_t credentialSemaphore;
SemaphoreHandle_t fingerprintScanSemaphore;
SemaphoreHandle_t facialRecognitionSemaphore;

// variáveis para a implementação do wifi e do mqtt
SemaphoreHandle_t wifiConnectedSemaphore;
SemaphoreHandle_t mqttConnectedSemaphore;

// semáforo de publicação que libera o broker pra publicar
SemaphoreHandle_t mqttPubSemaphore;

// variáveis de acesso para envio

struct access
{
    char registeredCredential[5];
    int fingerPrintLog;
    int facialCaptureLog;
    char timestamp[30];
};

struct access authorized;
struct access denied; 


// variável de envio

 // é a variável que vai ser testada no Broker antes de publicar
const int pubNoState = 2;
const int pubAuthorizedAccess = 1;
const int pubDeniedAccess = 0;
int publicationStatus = pubNoState;





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


void fillAccessLevel1(int validationStatus, char attemptedCredential[])
{
    if(validationStatus == true) // autorizado
    {
        authorized.registeredCredential[0] = attemptedCredential[0];
        authorized.registeredCredential[1] = attemptedCredential[1];
        authorized.registeredCredential[2] = attemptedCredential[2];
        authorized.registeredCredential[3] = attemptedCredential[3];
        authorized.registeredCredential[4] = attemptedCredential[4];
    }
    if(validationStatus == false) // negado
    {
        denied.registeredCredential[0] = attemptedCredential[0];
        denied.registeredCredential[1] = attemptedCredential[1];
        denied.registeredCredential[2] = attemptedCredential[2];
        denied.registeredCredential[3] = attemptedCredential[3];
        denied.registeredCredential[4] = attemptedCredential[4];
        denied.fingerPrintLog = -1;
        denied.facialCaptureLog = -1;
        ESP_LOGW("TENTATIVA DE ACESSO", "CREDENCIAL %s", denied.registeredCredential);
        publicationStatus = pubDeniedAccess;
        xSemaphoreGive(mqttPubSemaphore);
    }
}

void fillAccessLevel2(int validationStatus, int bio)
{
    if(validationStatus == true)  // autorizado
    {
        authorized.fingerPrintLog = bio;
    }
    if(validationStatus == false) // negado
    {
        denied.registeredCredential[0] = authorized.registeredCredential[0];
        denied.registeredCredential[1] = authorized.registeredCredential[1];
        denied.registeredCredential[2] = authorized.registeredCredential[2];
        denied.registeredCredential[3] = authorized.registeredCredential[3];
        denied.registeredCredential[4] = authorized.registeredCredential[4];
        denied.fingerPrintLog = bio;
        denied.facialCaptureLog = -1;
        ESP_LOGW("TENTATIVA DE ACESSO", "CREDENCIAL: %s e Biometria: %d ", denied.registeredCredential, denied.fingerPrintLog);
        publicationStatus = pubDeniedAccess;
        xSemaphoreGive(mqttPubSemaphore);
    }
}

void fillAccessLevel3(int validationStatus, int face)
{
    if(validationStatus == true)  // autorizado
    {
        authorized.facialCaptureLog = face;
        publicationStatus = pubAuthorizedAccess;
        xSemaphoreGive(mqttPubSemaphore);
    }
    if(validationStatus == false) // negado
    {
        denied.registeredCredential[0] = authorized.registeredCredential[0];
        denied.registeredCredential[1] = authorized.registeredCredential[1];
        denied.registeredCredential[2] = authorized.registeredCredential[2];
        denied.registeredCredential[3] = authorized.registeredCredential[3];
        denied.registeredCredential[4] = authorized.registeredCredential[4];
        denied.fingerPrintLog = authorized.fingerPrintLog;
        denied.facialCaptureLog = face;
        ESP_LOGW("TENTATIVA DE ACESSO", "CREDENCIAL: %s, Biometria: %d e FaceID: %d", denied.registeredCredential, denied.fingerPrintLog, denied.facialCaptureLog);
        publicationStatus = pubDeniedAccess;
        xSemaphoreGive(mqttPubSemaphore);
    }
}


// ==============================================
// tarefas 


void taskInputIdle(void *pvParameters)
{

    char oledLine1[17], oledLine2[17], oledLine3[17];
    char key;


    // inicia as structs de acesso de informação zeradas
    memset(authorized.registeredCredential, 0, sizeof(authorized.registeredCredential));
    authorized.fingerPrintLog = 0;
    authorized.facialCaptureLog = 0;


    memset(denied.registeredCredential, 0, sizeof(denied.registeredCredential));
    denied.fingerPrintLog = 0;
    denied.facialCaptureLog = 0;



    while (1)
    {
        xSemaphoreTake(idleSemaphore, portMAX_DELAY);
        ESP_LOGI("SISTEMA", "Pressione A para entrar");
        

        ssd1306_clear_screen(&dev, false);
        snprintf(oledLine1, sizeof(oledLine1), "SISTEMA:");
        snprintf(oledLine2, sizeof(oledLine2), "Pressione A");
        ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
        ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);


        key = 0;
        key = keypad_get_key();
        if (key == 'A') 
        {
            ESP_LOGI("SISTEMA", "A Pressionado");

            ssd1306_clear_screen(&dev, false);
            snprintf(oledLine1, sizeof(oledLine1), "SISTEMA:");
            snprintf(oledLine2, sizeof(oledLine2), "'A' Pressionado'");
            snprintf(oledLine3, sizeof(oledLine3), "Aguarde a ID");
            ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
            ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);
            ssd1306_display_text(&dev, 4, oledLine3, strlen(oledLine3), false);

            xSemaphoreGive(credentialSemaphore);
        }else{
            ESP_LOGI("SISTEMA","Tecla incorreta");

            ssd1306_clear_screen(&dev, false);
            snprintf(oledLine1, sizeof(oledLine1), "SISTEMA:");
            snprintf(oledLine2, sizeof(oledLine2), "Tecla Incorreta");
            ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
            ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);
        }

    }
}

void taskInputCredential(void *pvParameters) 
{

    char oledLine1[17], oledLine2[17], oledLine3[17];
    char key_buffer[5] = {0};
    int index = 0;
    
    
    while (1)
    {
        xSemaphoreTake(credentialSemaphore, portMAX_DELAY);

        ESP_LOGI("ID", "Digite a credencial: ");
        ssd1306_clear_screen(&dev, false);
        snprintf(oledLine1, sizeof(oledLine1), "Identificação");
        snprintf(oledLine2, sizeof(oledLine2), "Digite sua ID:");
        ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
        ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);
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
                snprintf(oledLine1, sizeof(oledLine1), "Identificaçao");
                snprintf(oledLine2, sizeof(oledLine2), "ID Limpa");
                ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
                ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);
                
            }

            else if (key == 'A')
            {
                // Confirma credencial
                if (strcmp(key_buffer, CREDENTIAL) == 0)
                {
                    fillAccessLevel1(true, key_buffer);
                    ESP_LOGW("Credencial", "%s \n", authorized.registeredCredential); // isso foi um teste, retirar depois
                    
                    ESP_LOGI(TAG, "Acesso Valido");
                    ssd1306_clear_screen(&dev, false);
                    snprintf(oledLine1, sizeof(oledLine1), "Identificaçao");
                    snprintf(oledLine2, sizeof(oledLine2), "ID Valida");
                    ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
                    ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);
                    xSemaphoreGive(fingerprintScanSemaphore);
                }
                else
                {
                    fillAccessLevel1(false, key_buffer);

                    ESP_LOGW(TAG, "Credencial inválida");

                    ssd1306_clear_screen(&dev, false);
                    snprintf(oledLine1, sizeof(oledLine1), "Identificação");
                    snprintf(oledLine2, sizeof(oledLine2), "ID Invalida:");
                    ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
                    ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    ESP_LOGI("Sistema", "Acesso Negado");
                    ssd1306_clear_screen(&dev, false);
                    snprintf(oledLine1, sizeof(oledLine1), "SISTEMA:");
                    snprintf(oledLine2, sizeof(oledLine2), "ACESSO NEGADO");
                    ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
                    ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    xSemaphoreGive(idleSemaphore);
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
                snprintf(oledLine3, sizeof(oledLine3), key_buffer);
                ssd1306_display_text(&dev, 4, oledLine3, strlen(oledLine3), false);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}    
   
}

void taskInputFingerPrint(void *pvParameters) 
{

    char oledLine1[17], oledLine2[17];             
    uint16_t val;
    uint16_t filtered_value = 0;
    uint16_t raw_value_touch = 0;   

    bool touching = false;
    uint32_t touch_start = 0;

    while (1)
    {
        
        xSemaphoreTake(fingerprintScanSemaphore, portMAX_DELAY); 
        ESP_LOGI("Biometria", "Valide a biometria");

        ssd1306_clear_screen(&dev, false);
        snprintf(oledLine1, sizeof(oledLine1), "Biometria");
        snprintf(oledLine2, sizeof(oledLine2), "Valide a Digital");
        ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
        ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);

        
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
                    // dedo acabou de tocar
                    touching = true;
                    touch_start = xTaskGetTickCount();

                    ssd1306_clear_screen(&dev, false);
                    snprintf(oledLine1, sizeof(oledLine1), "Biometria");
                    snprintf(oledLine2, sizeof(oledLine2), "Verificando...");
                    ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
                    ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);
                }

                // dedo continua tocando
                if ((xTaskGetTickCount() - touch_start) >= pdMS_TO_TICKS(TOUCH_MIN_TIME_MS))
                {   
                    
                    
                    fillAccessLevel2(true, raw_value_touch);
                    ssd1306_clear_screen(&dev, false);
                    snprintf(oledLine1, sizeof(oledLine1), "Biometria");
                    snprintf(oledLine2, sizeof(oledLine2), "Digital Valida");
                    ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
                    ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);

                    
                    fillAccessLevel2(false, raw_value_touch); 
                    xSemaphoreGive(facialRecognitionSemaphore);
                    break;
                }
            } else
            {
                // dedo saiu antes do tempo mínimo
                if (touching)

                {   
                    
                    ESP_LOGW("Biometria", "Toque muito curto");
                    ESP_LOGW("Biometria", "Digital Inválida");
                    ssd1306_clear_screen(&dev, false);
                    snprintf(oledLine1, sizeof(oledLine1), "Biometria");
                    snprintf(oledLine2, sizeof(oledLine2), "Invalida");
                    ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
                    ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    ssd1306_clear_screen(&dev, false);
                    snprintf(oledLine1, sizeof(oledLine1), "SISTEMA:");
                    snprintf(oledLine2, sizeof(oledLine2), "ACESSO NEGADO");
                    ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
                    ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    touching = false;
                    touch_start = 0;
                    fillAccessLevel2(false, raw_value_touch);
                    xSemaphoreGive(idleSemaphore);
                    break;
                }
            }
        }
    }
}     

void taskInputFaceId(void *pvParameters)
{

   char oledLine1[17], oledLine2[17], oledLine3[17];

   while (1)
    {
      xSemaphoreTake(facialRecognitionSemaphore, portMAX_DELAY);
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
                    snprintf(oledLine1, sizeof(oledLine1), "FaceID");
                    snprintf(oledLine2, sizeof(oledLine2), "Verificando...");
                    ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
                    ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    
                    ESP_LOGI("FaceID", "Reconhecimento facial OK");
                    ssd1306_clear_screen(&dev, false);
                    snprintf(oledLine1, sizeof(oledLine1), "FaceID");
                    snprintf(oledLine2, sizeof(oledLine2), "Reconhecimento");
                    snprintf(oledLine3, sizeof(oledLine3), "facial OK");
                    ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
                    ssd1306_display_text(&dev, 4, oledLine2, strlen(oledLine2), false);
                    ssd1306_display_text(&dev, 5, oledLine3, strlen(oledLine3), false);

                    fillAccessLevel3(true, prox*100);

                    vTaskDelay(pdMS_TO_TICKS(1500));

                    ESP_LOGI("Sistema", "Acesso liberado");
                    ssd1306_clear_screen(&dev, false);
                    snprintf(oledLine1, sizeof(oledLine1), "SISTEMA:");
                    snprintf(oledLine2, sizeof(oledLine2), "ACESSO LIBERADO");
                    ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
                    ssd1306_display_text(&dev, 3, oledLine2, strlen(oledLine2), false);


                    vTaskDelay(pdMS_TO_TICKS(3000));

                    // Aqui a struct de acesso vai estar completa e será enviada via mqtt

                    xSemaphoreGive(idleSemaphore);
                    break;   // ← devolve controle da task
            } 
            /*if (prox*100>20 || prox*100<60)
            {
                fillAccessLevel3(false, prox*100);
                
                // novamente deixei o else para simbolizar um acesso facial recusado, quando na prática não tem
                // como recusar o reconhecimento por distância sem possivelmente causar um erro 
                break;

            }*/
            ESP_LOGW("FaceID", "Aproxime-se da câmera. Você está a: %0.04f cm\n", prox*100);
            ssd1306_clear_screen(&dev, false);
            snprintf(oledLine1, sizeof(oledLine1), "FaceID");
            snprintf(oledLine2, sizeof(oledLine2), "Aproxime-se");
            snprintf(oledLine3, sizeof(oledLine3), "da camera");
            ssd1306_display_text(&dev, 2, oledLine1, strlen(oledLine1), false);
            ssd1306_display_text(&dev, 4, oledLine2, strlen(oledLine2), false);
            ssd1306_display_text(&dev, 5, oledLine3, strlen(oledLine3), false);  
            vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
}

    // inicializad o MQTT
void wifiConnected(void *params)
{
    while (1)
    {
        if (xSemaphoreTake(wifiConnectedSemaphore, portMAX_DELAY))
        {
            mqtt_start();
            xSemaphoreTake(mqttConnectedSemaphore, portMAX_DELAY);
            xSemaphoreGive(idleSemaphore);
            xSemaphoreGive(mqttConnectedSemaphore);
        }
    }
}

void comunicacaoBroker(void *params)
{

        ESP_LOGI("MQTT", "Subscrição no Tópico estabelecida");
        char publishedMsg[250];

        xSemaphoreTake(mqttConnectedSemaphore, portMAX_DELAY);


        while (1)
        
        {
            xSemaphoreTake(mqttPubSemaphore, portMAX_DELAY);



            if (publicationStatus == pubAuthorizedAccess)
            {
            // pub se autorizado
                sprintf(publishedMsg, "Tentativa de acesso AUTORIZADA com Credencial: %s; \nBiometria: %d.png; \n FaceID: %d.png em %s \n",
                authorized.registeredCredential,
                authorized.fingerPrintLog,
                authorized.facialCaptureLog,
                strftime_buf);
                ESP_LOGI("broker terminal", "%s", strftime_buf);
                mqtt_publish("CEL080/3FA1_Test", publishedMsg);
                
                memset(publishedMsg, 0, sizeof(publishedMsg));
                publicationStatus = pubNoState;
                memset(strftime_buf, 0, sizeof(strftime_buf));

                //xSemaphoreGive(idleSemaphore);
            }


            if (publicationStatus == pubDeniedAccess)
            {
                // pub se negado
                sprintf(publishedMsg, "Tentativa de acesso NEGADA com Credencial: %s; \nBiometria: %d.png; \n FaceID: %d.png em %s \n",
                denied.registeredCredential,
                denied.fingerPrintLog,
                denied.facialCaptureLog,
                strftime_buf);
                ESP_LOGI("broker terminal", "%s", strftime_buf);
                mqtt_publish("CEL080/3FA1_Test", publishedMsg);

                memset(publishedMsg, 0, sizeof(publishedMsg));
                publicationStatus = pubNoState;
                memset(strftime_buf, 0, sizeof(strftime_buf));


                //xSemaphoreGive(idleSemaphore);    

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
  idleSemaphore = xSemaphoreCreateBinary();
  xSemaphoreTake(idleSemaphore, 0);
  credentialSemaphore = xSemaphoreCreateBinary();
  xSemaphoreTake(credentialSemaphore, 0);
  fingerprintScanSemaphore = xSemaphoreCreateBinary();
  xSemaphoreTake(fingerprintScanSemaphore, 0);
  facialRecognitionSemaphore = xSemaphoreCreateBinary();
  xSemaphoreTake(facialRecognitionSemaphore, 0);

  wifiConnectedSemaphore = xSemaphoreCreateBinary();
  mqttConnectedSemaphore = xSemaphoreCreateBinary();
  mqttPubSemaphore = xSemaphoreCreateBinary();


  
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
  const int mainTaskStackDepth = 2048;
  const int mainTaskPriority = 1;
  const int pvParams = NULL;
  const int pxCreatedTask = NULL;

  xTaskCreate(taskInputIdle, "InputIdle", mainTaskStackDepth, pvParams, mainTaskPriority, pxCreatedTask);
  xTaskCreate(taskInputCredential, "InputCredential", mainTaskStackDepth, pvParams, mainTaskPriority, pxCreatedTask);
  xTaskCreate(taskInputFingerPrint, "InputFingerPrint", mainTaskStackDepth, pvParams, mainTaskPriority, pxCreatedTask);
  xTaskCreate(taskInputFaceId, "InputFaceId", mainTaskStackDepth, pvParams, mainTaskPriority, pxCreatedTask);

  // Conexão mqtt
  const int connectionTaskStackDepth = 4096
  const int connectionTaskPriority = 2;
  xTaskCreate(wifiConnected, "Conexao MQTT", connectionTaskStackDepth, pvParams, connectionTaskPriority, pxCreatedTask);
  xTaskCreate(comunicacaoBroker, "Comunicacao com o Broker", connectionTaskStackDepth, pvParams, connectionTaskPriority, pxCreatedTask);

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