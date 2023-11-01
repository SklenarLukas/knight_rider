#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <stdio.h>
#include "esp_err.h"
#include "time.h"

const gpio_num_t LED1 = GPIO_NUM_15;
const gpio_num_t LED2 = GPIO_NUM_2;
const gpio_num_t LED3 = GPIO_NUM_4;
const gpio_num_t LED4 = GPIO_NUM_5;
const gpio_num_t LED5 = GPIO_NUM_18;
const gpio_num_t LED6 = GPIO_NUM_19;
const gpio_num_t LED7 = GPIO_NUM_21;
const gpio_num_t LED8 = GPIO_NUM_22;

gpio_num_t leds[8] = {LED1, LED2, LED3, LED4, LED5, LED6, LED7, LED8};

#define LEDC_MODE LEDC_LOW_SPEED_MODE

uint32_t displayStartTime = 0;
const uint32_t SHINE_TIME_MS = 110;

const uint8_t channel[] = {0, 1, 2, 3, 4, 5, 6, 7};

const int max = 4;
const int quarter3 = 3;
const int half = 2;
const int quarter = 1;

enum State
{
    STATE_OFF,
    LEFT_START,      // 100 75 50 25 0 0 0 0;
    LEFT_EXPANSION1, // 100 100 75 50 25 0 0 0;
    LEFT_EXPANSION2, // 100 100 100 75 50 25 0 0;
    LEFT_MOVE1,      // 0 25 50 75 100 100 100 0;
    LEFT_MOVE2,      // 0 0 25 50 75 100 100 100;
    LEFT_REDUCTION,  // 0 0 0 25 50 75 100 100;
    LEFT_END,        // 0 0 0 0 25 50 75 100;

    RIGHT_EXPANSION1, // 0 0 0 25 50 75 100 100; = left reduction
    RIGHT_EXPANSION2, // 0 0 25 50 75 100 100 100; = left move2
    RIGHT_MOVE1,      // 0 100 100 100 75 50 25 0;
    RIGHT_MOVE2,      // 100 100 100 75 50 25 0 0; = left expansion 2
    RIGHT_REDUCTION   // 100 100 75 50 25 0 0 0; = left expansion1
};

enum State currentState = STATE_OFF;

void displayResetTime()
{
    displayStartTime = (uint32_t)(clock() * 1000 / CLOCKS_PER_SEC);
}

void ledsInit()
{
    for (int i = 0; i < 8; ++i)
    {
        gpio_reset_pin(leds[i]);
        gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
    }
    displayResetTime();
}

void deactivateAll()
{
    for (uint8_t i = 0; i < sizeof(leds) / sizeof(leds[0]); i++)
    {
        ledc_set_duty(LEDC_MODE, channel[i], 0);
        ledc_update_duty(LEDC_MODE, channel[i]);
    }
}

static void example_ledc_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_2_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    for (int i = 0; i < 8; i++)
    {
        ledc_channel_config_t ledc_channel = {
            .channel = i,
            .duty = 0,
            .gpio_num = leds[i],
            .speed_mode = LEDC_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_0};
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
}

void setMultipleLedDuty(const int *channels, const int *duties, int numChannels)
{
    for (int i = 0; i < numChannels; i++)
    {
        ledc_set_duty(LEDC_MODE, channels[i], duties[i]);
        ledc_update_duty(LEDC_MODE, channels[i]);
    }
}

void leftStart()
{
    int channels[] = {0, 1, 2, 3};
    int duties[] = {max, quarter3, half, quarter};
    setMultipleLedDuty(channels, duties, 4);
}

void leftExpansion1()
{
    int channels[] = {0, 1, 2, 3, 4};
    int duties[] = {max, max, quarter3, half, quarter};
    setMultipleLedDuty(channels, duties, 5);
}

void leftExpansion2()
{
    int channels[] = {0, 1, 2, 3, 4, 5};
    int duties[] = {max, max, max, quarter3, half, quarter};
    setMultipleLedDuty(channels, duties, 6);
}

void leftMove1()
{
    int channels[] = {1, 2, 3, 4, 5, 6};
    int duties[] = {quarter, half, quarter3, max, max, max};
    setMultipleLedDuty(channels, duties, 6);
}

void leftMove2()
{
    int channels[] = {1, 2, 3, 4, 5, 6, 7};
    int duties[] = {quarter, half, quarter3, max, max, max, 0};
    setMultipleLedDuty(channels, duties, 7);
}

void leftReduction()
{
    int channels[] = {3, 4, 5, 6, 7};
    int duties[] = {quarter, half, quarter3, max, max};
    setMultipleLedDuty(channels, duties, 5);
}

void leftEnd()
{
    int channels[] = {4, 5, 6, 7};
    int duties[] = {quarter, half, quarter3, max};
    setMultipleLedDuty(channels, duties, 4);
}

void rightMove1()
{
    int channels[] = {1, 2, 3, 4, 5, 6};
    int duties[] = {max, max, max, quarter3, half, quarter};
    setMultipleLedDuty(channels, duties, 6);
}

typedef void (*StateFunction)(void);

StateFunction stateFunctions[] = {
    deactivateAll,
    leftStart,
    leftExpansion1,
    leftExpansion2,
    leftMove1,
    leftMove2,
    leftReduction,
    leftEnd,
    leftReduction,
    leftMove2,
    leftMove1,
    leftExpansion2,
    leftExpansion1};

int stateTransitions[] = {
    LEFT_START,
    LEFT_EXPANSION1,
    LEFT_EXPANSION2,
    LEFT_MOVE1,
    LEFT_MOVE2,
    LEFT_REDUCTION,
    LEFT_END,
    RIGHT_EXPANSION1,
    RIGHT_EXPANSION2,
    RIGHT_MOVE1,
    RIGHT_MOVE2,
    RIGHT_REDUCTION,
    LEFT_START};

void carProcess()
{
    uint32_t currentTime = (uint32_t)(clock() * 1000 / CLOCKS_PER_SEC);
    uint32_t timeDiff = currentTime - displayStartTime;

    if (SHINE_TIME_MS < timeDiff)
    {
        deactivateAll();
        stateFunctions[currentState]();
        displayResetTime();
        currentState = stateTransitions[currentState];
    }
}

void app_main(void)
{
    ledsInit();
    example_ledc_init();

    while (1)
    {
        carProcess();
        vTaskDelay(10 / portTICK_PERIOD_MS); // watchdog
    }
}

