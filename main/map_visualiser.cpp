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
}
void draw_points() {
    for (const auto& b : barriers)
            display.fillCircle(b.x * MAP_TO_DISPLAY_SCALE, b.y * MAP_TO_DISPLAY_SCALE,
                                OBSTACLE_DOT_RADIUS, GxEPD_BLACK);
}
// Show 'D' (driving) or 'P' (parked) in the top-right corner
void draw_motor() {
    char clear = motor_on ? 'P' : 'D';
    char show = motor_on ? 'D' : 'P';
    display.drawChar(180, 30, clear, GxEPD_WHITE, GxEPD_WHITE, 1);
    display.drawChar(180, 30, show, GxEPD_BLACK, GxEPD_WHITE, 1);
}
