// Watchy rover controller firmware
//
// Tasks:
//   accel_task  (prio 4) - reads accelerometer, sends drive commands via ESP-NOW
//   map_task    (prio 3) - renders car + obstacles on the e-paper display
//   sensor_task (prio 2) - receives sensor data from the car, updates map
//
// Buttons: Top-Right = start/stop driving, Top-Left = reboot

#include <Arduino.h>
#include <Display.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <GxEPD2_BW.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "config.h"
#include "helper.h"
#include "interface.h"
#include "nvs_flash.h"
#include "sensor.h"

GxEPD2_BW<WatchyDisplay, WatchyDisplay::HEIGHT> display(
    WatchyDisplay(DISPLAY_CS, DISPLAY_DC, DISPLAY_RES, DISPLAY_BUSY));

BMA423 sensor;  // accelerometer (configured in helper.h)

uint8_t rover_mac[] = CAR1_MAC;  // MAC of the car this watch controls

SemaphoreHandle_t xSemaphoreCarUpdate;
SemaphoreHandle_t xSemaphoreMotorControl;
bool motor_on = false;  // true while driving is active

int car_x = 50;  // car position in map coords (0-100, origin = top-left)
int car_y = 50;
int car_angle = 270;

struct barrier {
    int x, y, angle;
};
std::vector<barrier> barriers;  // obstacle ring buffer

void set_car_position(int x, int y, int angle) {
    car_x = x;
    car_y = y;
    car_angle = angle + 180;
}

// Oldest point is dropped when buffer is full
void add_barrier(int x, int y, int angle) {
    if (barriers.size() >= MAX_OBSTACLES)
        barriers.erase(barriers.begin());
    barriers.push_back({x, y, angle});
}

// Configure GPIO pins and show the startup screen
void initDisplay() {
    pinMode(DISPLAY_CS, OUTPUT);
    pinMode(DISPLAY_RES, OUTPUT);
    pinMode(DISPLAY_DC, OUTPUT);
    pinMode(DISPLAY_BUSY, OUTPUT);
    pinMode(BTN_BOTTOM_LEFT, INPUT);
    pinMode(BTN_BOTTOM_RIGHT, INPUT);
    pinMode(BTN_TOP_LEFT, INPUT);
    pinMode(BTN_TOP_RIGHT, INPUT);

    display.epd2.selectSPI(SPI, SPISettings(20000000, MSBFIRST, SPI_MODE0));
    display.init(0, true, 10, true);
    display.setFullWindow();
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMonoBold18pt7b);
    // display.setCursor(0, 75);
    // display.print("Press\nTop Right\nto Start!");
    display.display(false);
}

// Show 'D' (driving) or 'P' (parked) in the top-right corner
void draw_motor() {
    char clear = motor_on ? 'P' : 'D';
    char show = motor_on ? 'D' : 'P';
    display.drawChar(180, 30, clear, GxEPD_WHITE, GxEPD_WHITE, 1);
    display.drawChar(180, 30, show, GxEPD_BLACK, GxEPD_WHITE, 1);
}

// Init WiFi in station mode (required before ESP-NOW)
static void wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

// Print this watch's MAC to the serial console (useful for pairing)
static void print_mac() {
    uint8_t mac[6];
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK)
        printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    else
        printf("Failed to read MAC address\n");
}

// Receive callback: validate magic and forward to sensor task
void on_data_recv(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len) {
    struct sensor_data* data = (struct sensor_data*)incomingData;
    if (data->magic != INTERFACE_MAGIC) {
        printf("Invalid magic number\n");
        return;
    }
    get_sensor_data_from_isr(data);
}

// Redraw the map every MAP_REFRESH_INTERVAL_MS
void map_task(void* pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    int partial_refresh_counter = 0;

    while (1) {
        display.fillScreen(GxEPD_WHITE);

        // Draw car as a triangle pointing in car_angle direction
        double rad = car_angle * M_PI / 180.0;
        int cx = car_x * MAP_TO_DISPLAY_SCALE;
        int cy = car_y * MAP_TO_DISPLAY_SCALE;
        int x0 = cx + CAR_TRIANGLE_SIZE * cos(rad);
        int y0 = cy - CAR_TRIANGLE_SIZE * sin(rad);
        int x1 = cx - CAR_TRIANGLE_SIZE * (cos(rad) + sin(rad));
        int y1 = cy + CAR_TRIANGLE_SIZE * (sin(rad) + cos(rad));
        int x2 = cx - CAR_TRIANGLE_SIZE * (cos(rad) - sin(rad));
        int y2 = cy + CAR_TRIANGLE_SIZE * (sin(rad) - cos(rad));
        display.fillTriangle(x0, y0, x1, y1, x2, y2, GxEPD_BLACK);

        draw_motor();

        // Draw obstacles as dots
        for (const auto& b : barriers)
            display.fillCircle(b.x * MAP_TO_DISPLAY_SCALE, b.y * MAP_TO_DISPLAY_SCALE,
                               OBSTACLE_DOT_RADIUS, GxEPD_BLACK);

        // E-paper needs occasional full refresh to avoid ghosting
        if (partial_refresh_counter < PARTIAL_REFRESH_LIMIT) {
            display.display(true);
            partial_refresh_counter++;
        } else {
            display.display(false);
            partial_refresh_counter = 0;
        }

        vTaskDelayUntil(&lastWakeTime, MAP_REFRESH_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

// Read accelerometer tilt and send drive commands to the car
void accel_task(void* pvParameters) {
    _bmaConfig();

    while (1) {
        if (!motor_on) {
            vTaskDelay(ACCEL_TASK_INTERVAL_MS / portTICK_PERIOD_MS);
            continue;
        }

        Accel acc;
        if (sensor.getAccel(acc)) {
            // Tilt mapping (BMA423, 2G range, ~1000 raw = 90 degrees):
            //   -x = forward, x = backward, y = left, -y = right
            double x = acc.x * -1.0;
            double speed;
            if (x >= ACCEL_FULL_TILT)
                speed = MAX_SPEED;
            else if (x <= -ACCEL_FULL_TILT)
                speed = -MAX_SPEED;
            else
                speed = x / ACCEL_FULL_TILT * MAX_SPEED;

            if (!motor_on) speed = 0;

            double y = acc.y * -1.0;
            double direction;
            if (y >= ACCEL_FULL_TILT)
                direction = MAX_ANGLE;
            else if (y <= -ACCEL_FULL_TILT)
                direction = -MAX_ANGLE;
            else
                direction = y / ACCEL_FULL_TILT * MAX_ANGLE;

            struct drive_data data = {INTERFACE_MAGIC, speed, direction};
            ESP_ERROR_CHECK(esp_now_send(rover_mac, (uint8_t*)&data, sizeof(data)));
        }

        vTaskDelay(ACCEL_TASK_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

// ISR: Top-Right button toggles driving on/off
static void toggle_motor(void* args) {
    motor_on = !motor_on;
    xSemaphoreGiveFromISR(xSemaphoreMotorControl, NULL);
}

// ISR: Top-Left button reboots the watch
static void reset_watch(void* args) {
    abort();
}

// One-shot task: init peripherals, then spawn the three worker tasks
void setup(void* pvParameters) {
    initDisplay();

    sensor_init();

    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();
    print_mac();
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_recv_cb(on_data_recv);

    // Register the car as an ESP-NOW peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, rover_mac, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
        printf("Failed to add peer\n");

    xTaskCreate(accel_task, "accel_task", TASK_STACK_SIZE, NULL, PRIORITY_ACCEL, NULL);
    xTaskCreate(map_task, "map_task", TASK_STACK_SIZE, NULL, PRIORITY_MAP, NULL);
    xTaskCreate(sensor_task, "sensor_task", TASK_STACK_SIZE, NULL, PRIORITY_SENSOR, NULL);

    vTaskDelete(NULL);
}

extern "C" void app_main() {
    if ((xSemaphoreCarUpdate = xSemaphoreCreateBinary()) == NULL ||
        (xSemaphoreMotorControl = xSemaphoreCreateBinary()) == NULL) {
        ESP_LOGE("app_main", "insufficient heap for semaphore");
        abort();
    }

    // Register button interrupts
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_set_intr_type((gpio_num_t)BTN_TOP_RIGHT, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)BTN_TOP_RIGHT, toggle_motor, NULL));
    ESP_ERROR_CHECK(gpio_set_intr_type((gpio_num_t)BTN_TOP_LEFT, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)BTN_TOP_LEFT, reset_watch, NULL));

    xTaskCreate(setup, "setup", TASK_STACK_SIZE, NULL, configMAX_PRIORITIES, NULL);
    vTaskStartScheduler();

    ESP_LOGE("app_main", "insufficient RAM");
    abort();
}
