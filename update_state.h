#ifndef UPDATE_STATE_H
#define UPDATE_STATE_H

#include "app_state.h"
#include "boolean.h"

void update_state(appstate_t *appstate,
                  boolean_t app_timer_elapsed);

void update_state_powersave(appstate_t *appstate, boolean_t app_timer_elapsed);
#endif