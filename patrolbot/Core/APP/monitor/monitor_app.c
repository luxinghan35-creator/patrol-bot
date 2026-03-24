//
// Created by luxin on 2026/3/24.
//
#include "monitor_app.h"
#include "oled.h"
#include "gy906.h"
#include "system_def.h"
#include <stdio.h>

volatile uint8_t system_alarm_flag = 0;

void Monitor_App_Init(void) {
    OLED_Init();
}

void Monitor_App_TaskLoop(void) {
    float obj_temp = R3X_GY906_Read_ObjTemp();

    if (obj_temp > 60.0f && obj_temp < 500.0f) {
        system_alarm_flag = 1;
    } else if (obj_temp < 50.0f) {
        system_alarm_flag = 0;
    }

    OLED_NewFrame();
    char buf[32];

    if (system_alarm_flag == 1) {
        OLED_PrintASCIIString(0, 0, "!! ALARM !!", &afont16x8, OLED_COLOR_REVERSED);
        sprintf(buf, "TEMP: %d C", (int)obj_temp);
        OLED_PrintASCIIString(0, 20, buf, &afont16x8, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(0, 40, "MOTORS LOCKED", &afont16x8, OLED_COLOR_NORMAL);
    } else {
        sprintf(buf, "Temp: %d C", (int)obj_temp);
        OLED_PrintASCIIString(0, 0, buf, &afont16x8, OLED_COLOR_NORMAL);
        sprintf(buf, "Yaw: %d", (int)current_yaw);
        OLED_PrintASCIIString(0, 20, buf, &afont16x8, OLED_COLOR_NORMAL);
        sprintf(buf, "SYS OK | %lu", rx_frame_cnt);
        OLED_PrintASCIIString(0, 40, buf, &afont16x8, OLED_COLOR_NORMAL);
    }
    OLED_ShowFrame();
}