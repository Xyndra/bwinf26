// Processes sensor data received from the car via ESP-NOW.
// Converts car-frame IMU positions to display coordinates and tracks obstacles.

#include "sensor.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <cmath>

#include "config.h"
#include "map_visualizer.h"

// Map origin (center of 0-100 coordinate space) and initial heading (up)
constexpr double offset_pos_x = 50;
constexpr double offset_pos_y = 50;
constexpr double offset_angle_z = 270;

constexpr double imu_scaling_factor = IMU_SCALING_FACTOR;
constexpr double lidar_scaling_factor = imu_scaling_factor;

QueueHandle_t sensor_queue;             // receives sensor_data from ESP-NOW callback
struct sensor_data sensor_data_buffer;  // latest received packet

void sensor_init() {
    sensor_queue = xQueueCreate(1, sizeof(struct sensor_data));
}

// Called from the ESP-NOW receive callback (non-task context)
void get_sensor_data_from_isr(struct sensor_data* data) {
    xQueueSendFromISR(sensor_queue, data, NULL);
}

void sensor_task(void* pvParameters) {
    while (1) {
        xQueueReceive(sensor_queue, &sensor_data_buffer, portMAX_DELAY);

        ESP_LOGI("SENSOR_TASK", "Received data: pos=(%.2f, %.2f) angle=%.2f lidar=%.2f",
                 sensor_data_buffer.pos_X, sensor_data_buffer.pos_Y,
                 sensor_data_buffer.ang_Z, sensor_data_buffer.lidar_distance);

        // The rover's X/Y axes are swapped relative to the display frame
        double pos_y = offset_pos_x + sensor_data_buffer.pos_X * -1.0 * imu_scaling_factor;
        double pos_x = offset_pos_y + sensor_data_buffer.pos_Y * -1.0 * imu_scaling_factor;
        double angle_z = offset_angle_z - sensor_data_buffer.ang_Z;

        // Clamp to map bounds
        if (pos_x < 0) pos_x = 0;
        if (pos_x > 100) pos_x = 100;
        if (pos_y < 0) pos_y = 0;
        if (pos_y > 100) pos_y = 100;

        sensor_data_buffer.lidar_distance *= lidar_scaling_factor;

        set_car_position(pos_x, pos_y, angle_z);

        // Add obstacle if close enough
        if (sensor_data_buffer.lidar_distance < OBSTACLE_DISTANCE_THRESHOLD) {
            double obs_angle = angle_z / 360.0 * 2.0 * M_PI;
            int obs_x = pos_x + sensor_data_buffer.lidar_distance * cos(obs_angle);
            int obs_y = pos_y + sensor_data_buffer.lidar_distance * sin(obs_angle);
            add_point(obs_x, obs_y);
        }
    }
}
