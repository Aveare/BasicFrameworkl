#ifndef LED_TASK_H
#define LED_TASK_H

#include "main.h"
#include "stdint.h"

#define RGB_FLOW_COLOR_LENGHT 6
#define RGB_FLOW_COLOR_CHANGE_TIME 50

void led_RGB_flow_task(void);
void FlowRGBShow(uint32_t aRGB);

#endif
