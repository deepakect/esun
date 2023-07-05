/* Copyright @ Blu Systems Pvt Ltd */
#ifndef __MONITOR_H
#define __MONITOR_H

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define EXAMPLE_READ_LEN   256
#define GET_UNIT(x)        ((x>>3) & 0x1)

void monitor_main(void);

#endif // #ifndef __MONITOR_H
