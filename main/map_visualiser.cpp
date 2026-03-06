#include "GxEPD2.h"
#include "config.h"
#include "main.h"
#include "map_visualizer.h"
#include <vector>

std::vector<barrier> barriers;

void draw_map() {
    draw_car(car_x, car_y);
    draw_motor();
    draw_points();
}

void add_point(int x, int y) {
    if (barriers.size() >= MAX_OBSTACLES)
            barriers.erase(barriers.begin());
    barriers.push_back({x, y});
}

void draw_car(int x, int y) {
    display.fillTriangle(100 + CAR_TRIANGLE_SIZE, 100 + CAR_TRIANGLE_SIZE,
       100, 100 - CAR_TRIANGLE_SIZE,
       100 - CAR_TRIANGLE_SIZE, 100 + CAR_TRIANGLE_SIZE, GxEPD_BLACK);
}
void draw_points() {
    for (const auto& b : barriers) {
        double rad = car_angle * M_PI / 180.0;
        // Translate barrier relative to car position, then scale
        double dx = (b.x - car_x) * MAP_TO_DISPLAY_SCALE;
        double dy = (b.y - car_y) * MAP_TO_DISPLAY_SCALE;
        // Rotate around the static car display center (100, 100)
        int screen_x = 100 + (int)(dx * cos(rad) - dy * sin(rad));
        int screen_y = 100 + (int)(dx * sin(rad) + dy * cos(rad));
        display.fillCircle(screen_x, screen_y, OBSTACLE_DOT_RADIUS, GxEPD_BLACK);
    }
}
// Show 'D' (driving) or 'P' (parked) in the top-right corner
void draw_motor() {
    char clear = motor_on ? 'P' : 'D';
    char show = motor_on ? 'D' : 'P';
    display.drawChar(180, 30, clear, GxEPD_WHITE, GxEPD_WHITE, 1);
    display.drawChar(180, 30, show, GxEPD_BLACK, GxEPD_WHITE, 1);
}
