#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "example";

#define MOVE_SPEED_DEGREES_PER_SECOND 5  // Degrees per second
#define DELAY_BETWEEN_POSITIONS_MS 3000  // Delay between position changes in milliseconds

// Servo configuration constants
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microseconds
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define num_of_servos 4

int current_angles[num_of_servos] = {0, 0, 0, 0};

// Predefined sequence of positions

#define NUM_POSITIONS 7
int position_sequence[NUM_POSITIONS][num_of_servos] = {
    {0, 0, 0, 5},       // Position 1
    {0, 60, 40, 5},     // Position 2
    {0, 60, 60, -40},   // Position 3
    {0, 60, 60, 5},  // Position 4
    {60, 0, 0, 5}, // Position 5
    {0, 0, 0, 5}        // Position 6 (return to home)
};

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

// Task to control servo movements through predefined positions
void servo_sequence_task(void *pvParameters)
{
    int current_position = 0;
    
    ESP_LOGI(TAG, "Starting servo sequence with %d positions", NUM_POSITIONS);
    
    while(1) {
        // Get the next position in the sequence
        int *target_angles = position_sequence[current_position];
        
        ESP_LOGI(TAG, "Moving to position %d: %d, %d, %d, %d", 
                 current_position + 1, 
                 target_angles[0], 
                 target_angles[1], 
                 target_angles[2], 
                 target_angles[3]);
        
        // Move each servo to its target angle
        for (int i = 0; i < num_of_servos; i++) {
            // Get the current angle of the servo from stored value
            int current_angle = current_angles[i];
            int target_angle = target_angles[i];
            
            // Move the servo step by step to the target angle
            int delta_angle = target_angle - current_angle;
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
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], example_angle_to_compare(target_angle)));
            
            // Update the stored angle to the target angle
            current_angles[i] = target_angle;
        }
        
        // Wait for a while at this position
        ESP_LOGI(TAG, "Holding position for %d milliseconds", DELAY_BETWEEN_POSITIONS_MS);
        vTaskDelay(pdMS_TO_TICKS(DELAY_BETWEEN_POSITIONS_MS));
        
        // Move to the next position in the sequence
        current_position = (current_position + 1) % NUM_POSITIONS;
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
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[3], example_angle_to_compare(0)));
    
    // Create task to handle servo sequence
    ESP_LOGI(TAG, "Starting servo sequence task");
    ESP_LOGI(TAG, "The servos will cycle through seven positions:");
    ESP_LOGI(TAG, "Position 1: 0, 0, 0, 5");
    ESP_LOGI(TAG, "Position 2: 0, 60, 40, 5");
    ESP_LOGI(TAG, "Position 3: 0, 60, 60, -40");
    ESP_LOGI(TAG, "Position 4: 0, 60, 60, 5");
    ESP_LOGI(TAG, "Position 5: 60, 0, 0, 5");
    ESP_LOGI(TAG, "Position 6: 0, 0, 0, 5");
    
    xTaskCreate(servo_sequence_task, "servo_sequence_task", 4096, NULL, 5, NULL);
}