#pragma once

struct barrier {
    int x, y;
};

void draw_map();
void add_point(int x, int y);
void draw_car(int x, int y);
void draw_points();
void draw_motor();
static void toggle_draw_mode(void* args);
