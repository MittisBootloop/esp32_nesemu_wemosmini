#pragma once
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief running intro and menu (choose rom/game)
 *
 * @return - integer for choosing game partition
 */
int runMenu();
void setBr(int bright);