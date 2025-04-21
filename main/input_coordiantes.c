#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include <string.h>
#include <stdlib.h>
#include "inverseKinematics.c"

static const char *TAG = "example";

// Servo configuration constants
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microseconds
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define num_of_servos 4
#define SERVO_MOVE_STEP 1        // How many degrees to move in each step
#define SERVO_MOVE_INTERVAL_MS 20 // Time between each step in milliseconds

int servo_pins[num_of_servos] = {15, 5, 18, 19};        // GPIO pins connected to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20ms period (50Hz PWM frequency)

// Global handles for accessing in the servo task
mcpwm_cmpr_handle_t comparator[num_of_servos] = {NULL, NULL, NULL, NULL};

// Global variables to store current and target angles
static int current_angles[num_of_servos] = {0, 0, 0, 30};  // Initialize with starting positions
static int target_angles[num_of_servos] = {0, 0, 0, 30};   // Initialize with same positions

static inline uint32_t example_angle_to_compare(int angle)
{
    // Clamp angle to valid range
    if (angle < SERVO_MIN_DEGREE) angle = SERVO_MIN_DEGREE;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;

    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
           (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

// Function to move servos smoothly
void move_servos_smoothly(void)
{
    bool movement_complete = false;

    while (!movement_complete) {
        movement_complete = true;  // Assume all movements are complete

        // Update each servo position
        for (int i = 0; i < num_of_servos; i++) {
            if (current_angles[i] < target_angles[i]) {
                current_angles[i] += SERVO_MOVE_STEP;
                if (current_angles[i] > target_angles[i]) {
                    current_angles[i] = target_angles[i];
                }
                movement_complete = false;
            }
            else if (current_angles[i] > target_angles[i]) {
                current_angles[i] -= SERVO_MOVE_STEP;
                if (current_angles[i] < target_angles[i]) {
                    current_angles[i] = target_angles[i];
                }
                movement_complete = false;
            }

            // Apply the new angle
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i],
                            example_angle_to_compare(current_angles[i])));
        }

        // Wait before next movement
        vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_INTERVAL_MS));
    }
}

// Function to set target angles from inverse kinematics calculation
void set_target_angles_from_ik(JointAngles angles) {
    if (angles.isValid) {
        // Convert radian angles to degrees and set as target
        target_angles[0] = (int)round(radToDeg(angles.theta1));
        target_angles[1] = (int)round(radToDeg(angles.theta2));
        target_angles[2] = (int)round(radToDeg(angles.theta3));
        // Keep the last joint (gripper) at its current position
    } else {
        ESP_LOGW(TAG, "Invalid angles calculated. Not updating targets.");
    }
}

// Task to handle reading coordinates from UART
void servo_control_task(void *pvParameters)
{
    char rx_buffer[128];
    char *token;
    double coordinates[3]; // x, y, z
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

    printf("\nEnter target position (format: x y z): ");

    while(1) {
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)rx_buffer, sizeof(rx_buffer) - 1, pdMS_TO_TICKS(1000));

        if (len > 0) {
            rx_buffer[len] = 0;
            token = strtok(rx_buffer, " \r\n");
            idx = 0;

            // Extract coordinates from user input
            while (token != NULL && idx < 3) {
                coordinates[idx] = atof(token);
                token = strtok(NULL, " \r\n");
                idx++;
            }

            // If we got all three coordinates
            if (idx == 3) {
                double xc = coordinates[0];
                double yc = coordinates[1];
                double zc = coordinates[2];
                
                // Link lengths (replace with your actual values)
                double l1 = 10.0; // First link length (vertical)
                double l2 = 15.0; // Second link length
                double l3 = 12.0; // Third link length
                
                // Target position for the end effector
                Position target = {xc, yc, zc};
                printf("Target position: x=%.2f, y=%.2f, z=%.2f\n", target.x, target.y, target.z);
                
                // Calculate inverse kinematics
                JointAngles angles = calculateInverseKinematics(target, l1, l2, l3);
                
                // Display the results in degrees
                printf("\nJoint angles:\n");
                printf("theta1 = %.2f degrees\n", radToDeg(angles.theta1));
                printf("theta2 = %.2f degrees\n", radToDeg(angles.theta2));
                printf("theta3 = %.2f degrees\n", radToDeg(angles.theta3));
                
                if (angles.isValid) {
                    printf("\nSolution is within joint constraints (-90° to 90°)\n");
                    
                    // Set the calculated angles as target angles
                    set_target_angles_from_ik(angles);
                    
                    // Move servos smoothly to new positions
                    move_servos_smoothly();
                    
                    // Print current positions after movement is complete
                    printf("Movement complete. Current angles: ");
                    for (int i = 0; i < num_of_servos; i++) {
                        printf("%d ", current_angles[i]);
                    }
                    printf("\n");
                    
                    // Verify with forward kinematics
                    Position result = calculateForwardKinematics(angles, l1, l2, l3);
                    printf("Final position: x=%.2f, y=%.2f, z=%.2f\n", result.x, result.y, result.z);
                    
                    // Calculate error
                    double error = sqrt(pow(target.x - result.x, 2) + 
                                       pow(target.y - result.y, 2) + 
                                       pow(target.z - result.z, 2));
                    printf("Position error: %.6f\n", error);
                } else {
                    printf("\nERROR: OUT OF RANGE! Target position is not reachable.\n");
                    printf("Try a different target position or adjust link lengths.\n");
                }
            } else {
                printf("Invalid input. Please enter three values for x, y, and z coordinates.\n");
            }
            
            printf("\nEnter next target position (format: x y z): ");
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

    // Connect the operators to the timer
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
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], example_angle_to_compare(current_angles[i])));

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
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i], example_angle_to_compare(current_angles[i])));

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
    for (int i = 0; i < num_of_servos; i++) {
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator[i],
                        example_angle_to_compare(current_angles[i])));
    }

    // Create task to handle servo control
    ESP_LOGI(TAG, "Starting servo control task");
    ESP_LOGI(TAG, "Enter target position as: x y z coordinates");

    xTaskCreate(servo_control_task, "servo_control_task", 4096, NULL, 5, NULL);
}