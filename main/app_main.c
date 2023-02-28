/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "Fingerprint.h"
#include "i2c-lcd.h"
#include "keypad.h"

//#define BROKER             "mqtt://192.168.3.1:1883" 
 #define BROKER              "mqtt://test.mosquitto.org:1883"  

#define TOPIC_DOOR          "Prj/Door"
#define TOPIC_FLOOR1_HUM        "Prj/Floor1/hum"
#define TOPIC_FLOOR1_TEM        "Prj/Floor1/tem"

#define TOPIC_FLOOR1   "Prj/Floor1"

#define TOPIC_FLOOR2        "Prj/Floor2"
#define TOPIC_FLOOR3        "Prj/Floor3"
#define TOPIC_ACCOUNT       "Prj/Account"
#define TOPIC_PASSWORD      "Prj/Password"
#define TOPIC_DOOROPENCOUNTER      "Prj/DoorOpenCounter"
#define TOPIC_FIREALARM     "Prj/Fire"
#define TOPIC_FIRSTINIT     "Prj/FirstInit"

#define DOOR_CMD_LIGHT               '1'
#define DOOR_CMD_LIGHT_ON            '1'
#define DOOR_CMD_LIGHT_OFF           '0'
#define DOOR_CMD_DOOR                '2'
#define DOOR_CMD_DOOR_OPEN           '1'
#define DOOR_CMD_DOOR_CLOSE          '0'
#define PRE_ID          '0'

#define FLOOR1_CMD_LIGHT        '4'
#define FLOOR1_CMD_LIGHT_ON     '1'
#define FLOOR1_CMD_LIGHT_OFF    '0'
#define FLOOR1_CMD_TEMP         '5'
#define FLOOR1_CMD_HUMI         '6'

esp_mqtt_client_handle_t client;

static const char *TAG = "Graduation Door";

Fingureprint fp;
uint8_t search_id = 1;
uint8_t enrol_id = 1;
uint8_t enrol_time = 0;

#define LOCK 23
#define LOCK_BTN_INDOOR 15
//#define LOCK_BTN_OUTDOOR 34

#define LED 18
#define LED_BUTTON 19

char defaultPassword[5] = "1212";
char keyBuffer[5] = "";
const char emptyString[] = "";


static void my_App_Init(void)
{
    //lock init
    gpio_set_direction(LOCK_BTN_INDOOR, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LOCK_BTN_INDOOR, GPIO_PULLUP_ONLY);

    // gpio_set_direction(LOCK_BTN_OUTDOOR, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(LOCK_BTN_OUTDOOR, GPIO_PULLUP_ONLY);

    gpio_set_direction(LOCK, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(LOCK, 1);

    //device init
    gpio_set_direction(LED_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LED_BUTTON, GPIO_PULLUP_ONLY);
    gpio_set_direction(LED, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(LED, 1);
}


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM_0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

static void LED_button_control(void* arg)
{

    for(;;) {
        //int current_button_state = gpio_get_level(LED_BUTTON);
        //printf("%d\n", current_button_state);
        if(gpio_get_level(LED_BUTTON)==0){
            while(gpio_get_level(LED_BUTTON)==0);
            int current_state = gpio_get_level(LED);
            if(current_state == 1){
                ESP_LOGI(TAG, "on");
            esp_mqtt_client_publish(client, TOPIC_DOOR, "11", 2, 1, 0);             
            }
            else{
                ESP_LOGI(TAG, "off");
            esp_mqtt_client_publish(client, TOPIC_DOOR, "10", 2, 1, 0);             
            }
        }

        vTaskDelay(10);
    }
}

static void LOCK_button_indoor_control(void* arg)
{

    for(;;) {
        //int current_button_state = gpio_get_level(LOCK_BTN_INDOOR);
        //printf("butt: %d\n", current_button_state);
        if(gpio_get_level(LOCK_BTN_INDOOR)==0){
            while(gpio_get_level(LOCK_BTN_INDOOR)==0);
            int current_state = gpio_get_level(LOCK);
            //printf("%d\n", current_state);
            if(current_state == 1){
                ESP_LOGI(TAG, "on");
            esp_mqtt_client_publish(client, TOPIC_DOOR, "21", 2, 1, 0);             
            }
            else{
                ESP_LOGI(TAG, "off");
            esp_mqtt_client_publish(client, TOPIC_DOOR, "20", 2, 1, 0);
            enterPassword(keyBuffer);             
            }
        }
        vTaskDelay(10);
    }
}

// static void LOCK_button_outdoor_control(void* arg)
// {

//     for(;;) {
//         //int current_button_state = gpio_get_level(LOCK_BTN_OUTDOOR);
//         //printf("butt: %d\n", current_button_state);
//         if(gpio_get_level(LOCK_BTN_OUTDOOR)==0){
//             while(gpio_get_level(LOCK_BTN_OUTDOOR)==0);
//             // int current_state = gpio_get_level(LOCK);
//             // //printf("%d\n", current_state);
//             // if(current_state == 1){
//             //     ESP_LOGI(TAG, "off");
//             esp_mqtt_client_publish(client, TOPIC_DOOR, "20", 2, 1, 0);
//             enterPassword(keyBuffer);             
//             // }
//         }
//         vTaskDelay(10);
//     }
// }

static void KeypadPassword(void* arg)
{

    for(;;)
    {
        char keypressed = keypad_getkey();
        if(keypressed == '#'){
            checkPasswordKeypad(keyBuffer, defaultPassword, LOCK);
            const char empty_string[] = "";
            strcpy(keyBuffer, empty_string);
            //if password was correct and the door opened, send 0
            if(gpio_get_level(LOCK) == 0){
                esp_mqtt_client_publish(client, TOPIC_DOOROPENCOUNTER, "0", 1, 1, 0);
            }
        }

        if(keypressed == 'C'){
            lcd_init();
            lcd_clear();
            lcd_put_cur(0, 1);
            lcd_send_string("Enter password: ");
            lcd_put_cur(1, 3);
        }
        
        if(keypressed == '*'){
            //clear the input password
            enterPassword(keyBuffer);
        }

        if(keypressed == 'A'){
            //close the door when out of the house
            esp_mqtt_client_publish(client, TOPIC_DOOR, "20", 2, 1, 0);
            enterPassword(keyBuffer); 
        }

        if(keypressed == '0'||keypressed == '1'||keypressed == '2'||keypressed == '3'||
        keypressed == '4'||keypressed == '5'||keypressed == '6'||keypressed == '7'||
        keypressed == '8'||keypressed == '9'){ 
            int keyBufferLength = strlen(keyBuffer);
            if(keyBufferLength < 5){
                if(keypressed == '0') strcat(keyBuffer, "0");
                if(keypressed == '1') strcat(keyBuffer, "1");
                if(keypressed == '2') strcat(keyBuffer, "2");
                if(keypressed == '3') strcat(keyBuffer, "3");
                if(keypressed == '4') strcat(keyBuffer, "4");
                if(keypressed == '5') strcat(keyBuffer, "5");
                if(keypressed == '6') strcat(keyBuffer, "6");
                if(keypressed == '7') strcat(keyBuffer, "7");
                if(keypressed == '8') strcat(keyBuffer, "8");
                if(keypressed == '9') strcat(keyBuffer, "9");
                //lcd_send_string(keypressed);
                lcd_send_data(keypressed);
                for(int i = 0; i < strlen(keyBuffer)) {
                    ESP_LOGI(TAG, "%c", keyBuffer[i]);
                }
            }
            
        }
            vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


// static void checkFinger(void* arg)
// {
//     for(;;)
//     {
//       search_id = searchUser(&fp);
//       ESP_LOGI("main", "ID = %d", search_id);
//       vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }

static void fingerprint(void* arg)
{
    char buffer_id[5];
    char found_id[5];

    for(;;)
    {
        if(enrol_time){
            //if((eTaskGetState(task_search_fp) == eRunning) || (eTaskGetState(task_keypad_password) == eRunning)){
                // vTaskSuspend(task_search_fp);
                // vTaskSuspend(task_keypad_password);
            //}
            //ESP_LOGI("mainii", "flag = %d", enrol_time);
            ESP_LOGI("main", "ID = %d", enrol_id);
            if(enrollUser(enrol_id) != AS608_ERROR)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                sprintf(buffer_id, "0%d", enrol_id);
                esp_mqtt_client_publish(client, TOPIC_ACCOUNT, buffer_id, strlen(buffer_id), 1, 0);
                enrol_id ++;
            }
            else{
                ESP_LOGI("main", "Co loi xay ra moi thu lai");
            }
        }else{
                search_id = searchUser(&fp);
                ESP_LOGI("main", "ID = %d", search_id);
                if(search_id != 255){
                    gpio_set_level(LOCK, 0);
                    sprintf(buffer_id, "%d", search_id);
                    esp_mqtt_client_publish(client, TOPIC_DOOROPENCOUNTER, buffer_id, strlen(buffer_id), 1, 0);
                    // sprintf(found_id, "%d", search_id);
                    
                    // esp_mqtt_client_publish(client, TOPIC_DOOROPENCOUNTER, search_id, strlen(search_id), 1, 0);
                }
                
                vTaskDelay(1000 / portTICK_PERIOD_MS);
        //     if((eTaskGetState(task_search_fp) == eSuspended) && (eTaskGetState(task_keypad_password) == eSuspended)){
        //         vTaskResume(task_search_fp);
        //         vTaskResume(task_keypad_password);
        //     }
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

static void send_cf(){
    esp_mqtt_client_publish(client, TOPIC_PASSWORD, "1", 1, 1, 0);
}
/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        msg_id = esp_mqtt_client_subscribe(client, TOPIC_ACCOUNT, 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, TOPIC_PASSWORD, 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, TOPIC_DOOR, 1);
        ESP_LOGI(TAG, "senut sbscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, TOPIC_DOOROPENCOUNTER, 1);
        ESP_LOGI(TAG, "senut sbscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, TOPIC_FIRSTINIT, 1);
        ESP_LOGI(TAG, "senut sbscribe successful, msg_id=%d", msg_id);

        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        

        if(strncmp(event->topic,"Prj/Account", event->topic_len)==0 && strncmp(event->data,"rq", event->data_len)==0){
            ESP_LOGI(TAG, "Enrol time!");
            enrol_time = 1;
            //gpio_set_level(LED, 1);
        }
        else if(strncmp(event->topic,"Prj/Account", event->topic_len)==0 && strncmp(event->data,"nrq", event->data_len)==0){
            ESP_LOGI(TAG, "Not enrol time");
            enrol_time = 0;
            //Return Password mode
            lcd_clear();
            lcd_put_cur(0, 1);
            lcd_send_string("Enter password: ");
            lcd_put_cur(1, 3);
            //gpio_set_level(LED, 0);
        }
        else if(strncmp(event->topic,"Prj/Password", event->topic_len)==0){
            if(strncmp(event->data,"1", event->data_len) != 0){
                strncpy(defaultPassword, event->data, event->data_len);
                ESP_LOGI(TAG, "Password has changed!");
                esp_mqtt_client_publish(client, TOPIC_PASSWORD, "1", 1, 1, 0);
            }
            //esp_mqtt_client_publish(client, TOPIC_PASSWORD, "1", 1, 1, 0);
            //gpio_set_level(LED, 0);
        }       
        else if(strncmp(event->topic,"Prj/Door", event->topic_len)==0){
            if(event->data_len != 0){
                if(strncmp(event->data,"21", event->data_len)==0){
                ESP_LOGI(TAG, "OPEN DOOR");
                gpio_set_level(LOCK, 0);
            }
            if(strncmp(event->data,defaultPassword, event->data_len)==0){
                ESP_LOGI(TAG, "OPEN DOOR");
                gpio_set_level(LOCK, 0);
                esp_mqtt_client_publish(client, TOPIC_DOOROPENCOUNTER, "0", 1, 1, 0);
            }
            if(strncmp(event->data,"20", event->data_len)==0){
                ESP_LOGI(TAG, "CLOSE DOOR");
                gpio_set_level(LOCK, 1);
            }
            if(strncmp(event->data,"11", event->data_len)==0){
                ESP_LOGI(TAG, "LED ON");
                gpio_set_level(LED, 0);
            }
            if(strncmp(event->data,"10", event->data_len)==0){
                ESP_LOGI(TAG, "LED OFF");
                gpio_set_level(LED, 1);
            }
            }         
        }
        else if(strncmp(event->topic,"Prj/FirstInit", event->topic_len)==0){   
            printf("aaa%d\n", event->data_len );
            if((event->data_len) == 4){
                strncpy(defaultPassword, event->data, event->data_len);
                ESP_LOGI(TAG, "Password has changed!");
                //esp_mqtt_client_publish(client, TOPIC_PASSWORD, "ok", 1, 1, 0);
            }
            else if((event->data_len) == 2){
                uint8_t first_number, second_number, last_id;
                first_number = (event->data)[0];
                second_number = (event->data)[1];
                if(first_number > '0' && first_number <'9')
                {
                    first_number = first_number - 48;
                }
                if(second_number > '0' && second_number <'9')
                {
                    second_number = second_number - 48;
                }
                if(second_number == '0')
                {
                    second_number = 0;
                }
                last_id = first_number * 10 + second_number;
                printf("bb%d\n", last_id );
                enrol_id = last_id + 1;
                //esp_mqtt_client_publish(client, TOPIC_PASSWORD, "ok", 1, 1, 0);
            }
            else{  
                if(*(event->data) > '0' && *(event->data) <'9'){
                    uint8_t last_id;
                    last_id = (*event->data) - 48;
                    printf("bb%d\n", last_id );
                    enrol_id = last_id + 1;
                }else if(*(event->data) == '0'){
                    emptyDatabase();// clear finger print data
                    uint8_t last_id = 0;
                    printf("bb%d\n", last_id );
                    enrol_id = last_id + 1;
                }             
                            // uint8_t last_id = (uint8_t)atoi(event->data);
                            // printf("bb%d\n", last_id );
                            // enrol_id = last_id + 1;
                            // printf("next ID: %d\n", enrol_id );
                            //esp_mqtt_client_publish(client, TOPIC_PASSWORD, "ok", 1, 1, 0);
            }
            //esp_mqtt_client_publish(client, TOPIC_PASSWORD, "1", 1, 1, 0);
            //gpio_set_level(LED, 0);
        }    
        else {
            ESP_LOGI(TAG, "Nothing match");
        }    
        break;
    
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "published ok");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }  
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = BROKER,
        .keepalive = 60,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void)
{

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
  * Read "Establishing Wi-Fi or Ethernet Connection" section in
  * examples/protocols/README.md for more information about this function.
  */
  ESP_ERROR_CHECK(example_connect());

  mqtt_app_start();
  my_App_Init();
     //KeyPass
  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI(TAG, "I2C initialized successfully");

  ///                     R1  R2  R3  R4  C1  C2  C3  C4 
  gpio_num_t keypad[8] = {13, 12, 14, 27, 26, 25, 33, 32};

    /// Initialize keyboard
  keypad_initalize(keypad);

  
  lcd_init();
  lcd_clear();
  lcd_put_cur(0, 1);
  lcd_send_string("Enter password: ");
  lcd_put_cur(1, 3);

  //Fingerprint

  as608Init(&fp, 0x00);
  if(begin(57600) != AS608_OK){
    ESP_LOGI("main", "Loi begin");
  };
  verifyPassword(&fp);
  getParameters(&fp);
  ESP_LOGI("main", "Status : 0x%x\n", fp.status_reg); 
  ESP_LOGI("main", "Sys ID : 0x%x\n", fp.system_id);
  ESP_LOGI("main", "Capacity : %d\n", fp.capacity);
  ESP_LOGI("main", "Security level: %d\n", fp.security_level);
  ESP_LOGI("main", "Device address : %x\n", fp.device_addr);
  ESP_LOGI("main", "Packet length : %d\n", fp.packet_len);
  ESP_LOGI("main", "Baud rate : %d\n", fp.baud_rate);

  //emptyDatabase();
  
  

//   xTaskCreate(checkFinger, "find id with fingerprint", 2048, NULL, configMAX_PRIORITIES-1, task_search_fp);
  xTaskCreate(KeypadPassword, "unLOCK using keypad", 2048, NULL, configMAX_PRIORITIES-1, NULL);
  xTaskCreate(fingerprint, "enroll fingerprint", 2048, NULL, configMAX_PRIORITIES-1, NULL);
  xTaskCreate(LOCK_button_indoor_control, "push LOCK cmd", 2048, NULL, configMAX_PRIORITIES-1, NULL);
  //xTaskCreate(LOCK_button_outdoor_control, "push LOCK cmd", 2048, NULL, configMAX_PRIORITIES-1, NULL);
  xTaskCreate(LED_button_control, "push led cmd", 2048, NULL, configMAX_PRIORITIES-1, NULL);

}