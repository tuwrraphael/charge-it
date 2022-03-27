#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include "app_state.h"

void uart_debug_init();

void debug_appstate(appstate_t *appstate);

void debug_summary(appstate_t *appstate);

void debug_powersave(appstate_t *appstate);

#endif