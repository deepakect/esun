/* Copyright @ Blu Systems Pvt Ltd */
#include "hal/adc_types.h"

#include "sdkconfig.h"
#include "cmn/config.h"

/* Device specific configurations */

#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3 ||                  \
    CONFIG_IDF_TARGET_ESP32H2 || CONFIG_IDF_TARGET_ESP32C2
//Deepak
/*
adc_channel_t channel[3] = {ADC_CHANNEL_2, ADC_CHANNEL_3,
                                   (ADC_CHANNEL_0 | 1 << 3)};
                                   */
/* Changing channel to ADC1 ch 2*/

adc_channel_t channel = ADC_CHANNEL_2;
#endif
#if CONFIG_IDF_TARGET_ESP32S2
adc_channel_t channel[3] = {ADC_CHANNEL_2, ADC_CHANNEL_3,
                                   (ADC_CHANNEL_0 | 1 << 3)};
#endif
#if CONFIG_IDF_TARGET_ESP32
adc_channel_t channel[1] = {ADC_CHANNEL_7};
#endif

