#include "led_task.h"

static uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {
    0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF,
    0xFF00FF00, 0x00FF0000, 0xFF0000FF
};

void FlowRGBShow(uint32_t aRGB)
{
    uint8_t r = (aRGB >> 16) & 0xFF;
    uint8_t g = (aRGB >> 8) & 0xFF;
    uint8_t b = (aRGB >> 0) & 0xFF;

    if (r > 0)
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

    if (g > 0)
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

    if (b > 0)
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
}

void led_RGB_flow_task(void)
{
    static float delta_alpha, delta_red, delta_green, delta_blue;
    static float alpha, red, green, blue;
    static uint32_t aRGB;

    for (size_t i = 0; i < RGB_FLOW_COLOR_LENGHT; ++i)
    {
        alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
        red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
        green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
        blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

        delta_alpha = (float)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (float)((RGB_flow_color[i] & 0xFF000000) >> 24);
        delta_red = (float)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (float)((RGB_flow_color[i] & 0x00FF0000) >> 16);
        delta_green = (float)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (float)((RGB_flow_color[i] & 0x0000FF00) >> 8);
        delta_blue = (float)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (float)((RGB_flow_color[i] & 0x000000FF) >> 0);

        delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
        for (size_t j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; ++j)
        {
            alpha += delta_alpha;
            red += delta_red;
            green += delta_green;
            blue += delta_blue;

            aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
            FlowRGBShow(aRGB);
        }
    }
}
