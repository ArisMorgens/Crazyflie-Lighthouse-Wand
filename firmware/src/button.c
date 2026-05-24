#include "choose_app.h"
#ifdef BUILD_WAND_APP

#define DEBUG_MODULE "button"
#include "debug.h"

#include "deck.h"

#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "uart1.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "param.h"


#define PIN_BUTTON      DECK_GPIO_IO1      // Input button
#define PIN_LED_FEEDBACK DECK_GPIO_IO2     // Optional LED

static bool buttonPressed = false;         // Exported variable

// ---------- optional access for other code ----------
bool button_is_pressed(void) {
    return buttonPressed;
}

// ---------- Reading + debounce task ----------
static void buttonTask(void *param) {
    bool lastRead = false;
    bool stableState = false;
    TickType_t lastDebounce = 0;
    const TickType_t debounceTime = pdMS_TO_TICKS(40);

    while (1) {
        bool reading = digitalRead(PIN_BUTTON);

        if (reading != lastRead)
            lastDebounce = xTaskGetTickCount();

        if (xTaskGetTickCount() - lastDebounce > debounceTime) {
            if (reading != stableState) {
                stableState = reading;
                buttonPressed = stableState;        // update parameter
                digitalWrite(PIN_LED_FEEDBACK, stableState);

                DEBUG_PRINT("Button = %d\n", buttonPressed);
            }
        }

        lastRead = reading;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------- Deck driver registration ----------
static void buttonInit() {
    DEBUG_PRINT("Initialize Wand Button!\n");
    pinMode(PIN_BUTTON, INPUT);     // Set pin to input
    pinMode(PIN_LED_FEEDBACK, OUTPUT);     // Set pin to output

    digitalWrite(DECK_GPIO_IO1, LOW);
    digitalWrite(DECK_GPIO_IO2, LOW);

    xTaskCreate(buttonTask, "buttonTask", 256, NULL, 3, NULL);
}

static bool buttonTest() {
    return true;
}

static const DeckDriver buttonDriver = {
    .name = "WandButton",
    .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2,
    .init = buttonInit,
    .test = buttonTest,
};
DECK_DRIVER(buttonDriver);

// Parameter so Wand & Receivers can read this state
PARAM_GROUP_START(wand)
PARAM_ADD(PARAM_UINT8, buttonPressed, &buttonPressed)
PARAM_GROUP_STOP(wand)

LOG_GROUP_START(wand)
LOG_ADD(LOG_UINT8, buttonPressed, &buttonPressed)
LOG_GROUP_STOP(wand)

#endif // BUILD_WAND_APP