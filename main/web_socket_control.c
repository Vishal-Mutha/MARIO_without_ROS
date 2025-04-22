#include <stdio.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include <string.h>
#include <stdlib.h>

static const char resp[] =
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"    <title>Servo Control Panel</title>\n"
"    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
"    <style>\n"
"        body {\n"
"            font-family: Arial, sans-serif;\n"
"            margin: 40px auto;\n"
"            max-width: 400px;\n"
"            padding: 0 20px;\n"
"            background: #f5f7fa;\n"
"            color: #333;\n"
"        }\n"
"        h2 {\n"
"            text-align: center;\n"
"            margin-bottom: 30px;\n"
"        }\n"
"        form {\n"
"            background: #fff;\n"
"            padding: 15px 20px;\n"
"            margin-bottom: 20px;\n"
"            border-radius: 8px;\n"
"            box-shadow: 0 2px 5px rgba(0,0,0,0.1);\n"
"        }\n"
"        input[type=\"number\"],\n"
"        select {\n"
"            width: 100%;\n"
"            padding: 10px;\n"
"            margin-top: 8px;\n"
"            margin-bottom: 12px;\n"
"            border: 1px solid #ccc;\n"
"            border-radius: 5px;\n"
"            font-size: 16px;\n"
"        }\n"
"        input[type=\"submit\"] {\n"
"            background-color: #007bff;\n"
"            color: white;\n"
"            border: none;\n"
"            padding: 10px 15px;\n"
"            border-radius: 5px;\n"
"            font-size: 16px;\n"
"            cursor: pointer;\n"
"            width: 100%;\n"
"        }\n"
"        input[type=\"submit\"]:hover {\n"
"            background-color: #0056b3;\n"
"        }\n"
"        #response {\n"
"            text-align: center;\n"
"            font-weight: bold;\n"
"            margin-top: 20px;\n"
"        }\n"
"    </style>\n"
"</head>\n"
"<body>\n"
"\n"
"    <h2>Servo Control Panel</h2>\n"
"\n"
"    <form id=\"baseForm\">\n"
"        <label for=\"baseInput\">Base Angle (0-180)</label>\n"
"        <input type=\"number\" id=\"baseInput\" name=\"base\" min=\"0\" max=\"180\" required>\n"
"        <input type=\"submit\" value=\"Set Base\">\n"
"    </form>\n"
"\n"
"    <form id=\"shoulderForm\">\n"
"        <label for=\"shoulderInput\">Shoulder Angle (0-180)</label>\n"
"        <input type=\"number\" id=\"shoulderInput\" name=\"shoulder\" min=\"0\" max=\"180\" required>\n"
"        <input type=\"submit\" value=\"Set Shoulder\">\n"
"    </form>\n"
"\n"
"    <form id=\"elbowForm\">\n"
"        <label for=\"elbowInput\">Elbow Angle (0-180)</label>\n"
"        <input type=\"number\" id=\"elbowInput\" name=\"elbow\" min=\"0\" max=\"180\" required>\n"
"        <input type=\"submit\" value=\"Set Elbow\">\n"
"    </form>\n"
"\n"
"    <form id=\"gripperForm\">\n"
"        <label for=\"gripperInput\">Gripper</label>\n"
"        <select id=\"gripperInput\" name=\"gripper\">\n"
"            <option value=\"open\">Open</option>\n"
"            <option value=\"close\">Close</option>\n"
"        </select>\n"
"        <input type=\"submit\" value=\"Set Gripper\">\n"
"    </form>\n"
"\n"
"    <div id=\"response\"></div>\n"
"\n"
"    <script>\n"
"        document.getElementById(\"baseForm\").addEventListener(\"submit\", function(event) {\n"
"            event.preventDefault();\n"
"            sendRequest(\"base\", document.getElementById(\"baseInput\").value);\n"
"        });\n"
"\n"
"        document.getElementById(\"shoulderForm\").addEventListener(\"submit\", function(event) {\n"
"            event.preventDefault();\n"
"            sendRequest(\"shoulder\", document.getElementById(\"shoulderInput\").value);\n"
"        });\n"
"\n"
"        document.getElementById(\"elbowForm\").addEventListener(\"submit\", function(event) {\n"
"            event.preventDefault();\n"
"            sendRequest(\"elbow\", document.getElementById(\"elbowInput\").value);\n"
"        });\n"
"\n"
"        document.getElementById(\"gripperForm\").addEventListener(\"submit\", function(event) {\n"
"            event.preventDefault();\n"
"            sendRequest(\"gripper\", document.getElementById(\"gripperInput\").value);\n"
"        });\n"
"\n"
"        function sendRequest(paramName, value) {\n"
"            var params = paramName + \"=\" + encodeURIComponent(value);\n"
"            var xhr = new XMLHttpRequest();\n"
"            xhr.open(\"GET\", \"/setServo?\" + params, true);\n"
"            xhr.onreadystatechange = function() {\n"
"                if (xhr.readyState === 4 && xhr.status === 200) {\n"
"                    document.getElementById(\"response\").innerHTML = \"Response: \" + xhr.responseText;\n"
"                }\n"
"            };\n"
"            xhr.send();\n"
"        }\n"
"    </script>\n"
"\n"
"</body>\n"
"</html>\n"
"";

#define SSID "your ssid"
#define PASS "your password"

int base = 0;
int shoulder = 0;
int elbow = 0;
char gripper[64] = "close";

static const char *TAG = "ESP32 Server";

#define MOVE_SPEED_DEGREES_PER_SECOND 5  // Degrees per second
#define DELAY_BETWEEN_UPDATES_MS 200     // Delay between updates in milliseconds

// Servo configuration constants
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microseconds
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define num_of_servos 4

int servo_pins[num_of_servos] = {15, 5, 18, 19};        // GPIO pins connected to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20ms period (50Hz PWM frequency)

// Global handles for accessing in the servo task
mcpwm_cmpr_handle_t comparator[num_of_servos] = {NULL, NULL, NULL, NULL};

static inline uint32_t angle_to_compare(int angle)
{
    // Clamp angle to valid range
    if (angle < SERVO_MIN_DEGREE) angle = SERVO_MIN_DEGREE;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;

    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
           (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void set_servo_positions_base(int base_p)
{
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[0], angle_to_compare(base_p)));
}

void set_servo_positions_shoulder(int shoulder_p)
{
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[1], angle_to_compare(shoulder_p)));
}

void set_servo_positions_elbow(int elbow_p)
{
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[0], angle_to_compare(elbow_p)));
}

void set_servo_positions_gripper(char gripper_p[64])
{
    int posi = 0;
    if (gripper_p == "open") {
        posi = 60;
    } else if (gripper_p == "close") {
        posi = 0;
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[0], angle_to_compare(posi)));
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection ... \n");
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        break;
    default:
        break;
    }
}

void wifi_connection()
{
    // 1 - Wi-Fi/LwIP Init Phase
    esp_netif_init();                    // TCP/IP initiation 					s1.1
    esp_event_loop_create_default();     // event loop 			                s1.2
    esp_netif_create_default_wifi_sta(); // WiFi station 	                    s1.3
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); // 					                    s1.4
    // 2 - Wi-Fi Configuration Phase
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = SSID,
            .password = PASS}};
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    // 3 - Wi-Fi Start Phase
    esp_wifi_start();
    // 4- Wi-Fi Connect Phase
    esp_wifi_connect();
}

esp_err_t get_handler(httpd_req_t *req)
{


    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t get_handler_str(httpd_req_t *req)
{
    // Read the URI line and get the host
    char *buf;
    size_t buf_len;
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1)
    {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK)
        {
            ESP_LOGI(TAG, "Host: %s", buf);
        }
        free(buf);
    }

    // Read the URI line and get the parameters
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query: %s", buf);
            char param[32];
            if (httpd_query_key_value(buf, "base", param, sizeof(param)) == ESP_OK) {
                base = atoi(param);
                printf("%d\n", base);
                set_servo_positions_base(base);
                // ESP_LOGI(TAG, "The string value = %s", param);
            }
            if (httpd_query_key_value(buf, "shoulder", param, sizeof(param)) == ESP_OK) {
                shoulder = atoi(param);
                printf("%d\n", shoulder);
                set_servo_positions_shoulder(shoulder);
                // ESP_LOGI(TAG, "The int value = %s", param);
            }
            if (httpd_query_key_value(buf, "elbow", param, sizeof(param)) == ESP_OK) {
                elbow = atoi(param);
                printf("%d\n", elbow);
                set_servo_positions_elbow(elbow);
                // ESP_LOGI(TAG, "The int value = %s", param);
            }
            if (httpd_query_key_value(buf, "gripper", param, sizeof(param)) == ESP_OK) {
                strcpy(gripper, param);
                printf("%s\n", gripper);
                set_servo_positions_gripper(gripper);
                // ESP_LOGI(TAG, "The int value = %s", param);
            }
        }
        free(buf);
    }
    return ESP_OK;
}

/* URI handler structure for GET /uri */
httpd_uri_t uri_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_handler,
    .user_ctx = NULL};

httpd_uri_t uri_get_input = {
    .uri = "/setServo",
    .method = HTTP_GET,
    .handler = get_handler_str,
    .user_ctx = NULL};

httpd_handle_t start_webserver(void)
{
    // httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // raise from default 4096 bytes to e.g. 8192:
    config.stack_size = 8 * 1024;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_get_input);
    }
    return server;
}

void stop_webserver(httpd_handle_t server)
{
    if (server)
    {
        httpd_stop(server);
    }
}

void app_main(void)
{
    // Create MCPWM timer configuration
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper[2] = {NULL, NULL};
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group as the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper[0]));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper[1]));

    // Connect the operator to the timer
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper[0], timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper[1], timer));

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_gen_handle_t generator[num_of_servos] = {NULL, NULL, NULL, NULL};
    mcpwm_generator_config_t generator_config[num_of_servos] = {
        {.gen_gpio_num = servo_pins[0]},
        {.gen_gpio_num = servo_pins[1]},
        {.gen_gpio_num = servo_pins[2]},
        {.gen_gpio_num = servo_pins[3]}
    };

    for (int i = 0; i < 2; i++) {
        // Create the comparator and generator for each servo
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper[0], &comparator_config, &comparator[i]));
        ESP_ERROR_CHECK(mcpwm_new_generator(oper[0], &generator_config[i], &generator[i]));

        // Set the initial compare value to center the servo at 0 degrees
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], angle_to_compare(0)));

        // Set actions on timer and compare event
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator[i], MCPWM_GEN_ACTION_LOW)));
    }

    for (int i = 2; i < num_of_servos; i++) {
        // Create the comparator and generator for each servo
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper[1], &comparator_config, &comparator[i]));
        ESP_ERROR_CHECK(mcpwm_new_generator(oper[1], &generator_config[i], &generator[i]));

        // Set the initial compare value to center the servo at 0 degrees
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], angle_to_compare(0)));

        // Set actions on timer and compare event
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator[i], MCPWM_GEN_ACTION_LOW)));
    }

    // Start the timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));


    nvs_flash_init();
    wifi_connection();
    start_webserver();
}
