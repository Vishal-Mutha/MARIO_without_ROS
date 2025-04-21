#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "example";

#define MOVE_SPEED_DEGREES_PER_SECOND 5  // Degrees per second
#define DELAY_BETWEEN_UPDATES_MS 200     // Delay between updates in milliseconds

// Servo configuration constants
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microseconds
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define num_of_servos 4

int current_angles[num_of_servos] = {0, 0, 0, 0};

int servo_pins[num_of_servos] = {15, 5, 18, 19};        // GPIO pins connected to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20ms period (50Hz PWM frequency)

// Global handles for accessing in the servo task
mcpwm_cmpr_handle_t comparator[num_of_servos] = {NULL, NULL, NULL, NULL};

static inline uint32_t example_angle_to_compare(int angle)
{
    // Clamp angle to valid range
    if (angle < SERVO_MIN_DEGREE) angle = SERVO_MIN_DEGREE;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;
    
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / 
           (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

// Task to handle reading servo angles from UART

void servo_control_task(void *pvParameters)
{
    char rx_buffer[128];
    char *token;
    int angles[num_of_servos];
    int idx;
    
    // Configure UART parameters
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    
    printf("\nEnter angles for servos (format: angle1 angle2 angle3 angle4): ");

    while(1) {
        // Read data from UART indefinitely (no timeout)
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)rx_buffer, sizeof(rx_buffer) - 1, pdMS_TO_TICKS(1000));  // Blocking call
    
        if (len > 0) {
            // Null terminate the received string
            rx_buffer[len] = 0;
            
            // Parse input values
            token = strtok(rx_buffer, " \r\n");
            idx = 0;
            
            // Extract angles from user input
            while (token != NULL && idx < num_of_servos) {
                angles[idx] = atoi(token);
                token = strtok(NULL, " \r\n");
                idx++;
            }
            
            // Gradually move the servos to the new positions
            for (int i = 0; i < idx && i < num_of_servos; i++) {
                ESP_LOGI(TAG, "Setting servo %d to angle %d", i, angles[i]);
                
                // Get the current angle of the servo from stored value
                int current_angle = current_angles[i];
                
                // Move the servo step by step to the target angle
                int delta_angle = angles[i] - current_angle;
                int steps = abs(delta_angle) / MOVE_SPEED_DEGREES_PER_SECOND;  // Calculate the number of steps
                int step_delay_ms = 1000 / MOVE_SPEED_DEGREES_PER_SECOND;  // Delay between each step (in milliseconds)
                
                // Gradually move the servo
                for (int j = 0; j < steps; j++) {
                    int intermediate_angle = current_angle + (delta_angle * (j + 1)) / steps;
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], example_angle_to_compare(intermediate_angle)));
                    
                    // Store the new angle after the update
                    current_angles[i] = intermediate_angle;
                    
                    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
                }
                
                // Final adjustment to ensure the servo reaches the target angle
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], example_angle_to_compare(angles[i])));
                
                // Update the stored angle to the target angle
                current_angles[i] = angles[i];
            }
        }
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
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], example_angle_to_compare(0)));

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
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], example_angle_to_compare(0)));

        // Set actions on timer and compare event
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator[i], MCPWM_GEN_ACTION_LOW)));
    }    

    // Start the timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    // Set initial servo positions
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[0], example_angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[1], example_angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[2], example_angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[3], example_angle_to_compare(30)));
    
    // Create task to handle servo control
    ESP_LOGI(TAG, "Starting servo control task");
    ESP_LOGI(TAG, "Enter angles as: angle1 angle2 angle3 angle4 (-90 to 90 degrees)");
    
    xTaskCreate(servo_control_task, "servo_control_task", 4096, NULL, 5, NULL);
}