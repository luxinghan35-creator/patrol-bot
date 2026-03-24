//
// Created by luxin on 2026/3/24.
//

#ifndef PATROLBOT_SYSTEM_DEF_H
#define PATROLBOT_SYSTEM_DEF_H
#include <stdint.h>
extern volatile uint8_t system_alarm_flag;
extern volatile float current_yaw;
extern volatile float target_yaw;
extern volatile uint32_t rx_frame_cnt;

#endif //PATROLBOT_SYSTEM_DEF_H