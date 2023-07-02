/* Copyright @ Blu Systems Pvt Ltd */
#include "esp_log.h"
#include "sdkconfig.h"
#include "cmn/cmn.h"
// #include "freertos/timers.h"
#include "driver/gpio.h"
#include "monitor.h"

static TaskHandle_t adc_task_hndl;
static const char *TAG = "NRGY:MONITOR";

#define RELAY_PIN GPIO_NUM_3 // Relay pin location
bool replay_position = 0;    // Varible to check relay position, use extern to access it elsewhere
#define TURN_ON_RELAY gpio_set_level(RELAY_PIN, 1)
#define TURN_OFF_RELAY gpio_set_level(RELAY_PIN, 0)
bool exit_adc_loop;

/* callback */
static bool IRAM_ATTR adc_cb_conversion(adc_continuous_handle_t hndl,
                                        const adc_continuous_evt_data_t *data,
                                        void *usrdata)
{
  BaseType_t isYield = pdFALSE;

  /* Conversion is done, notify the driver */
  vTaskNotifyGiveFromISR(adc_task_hndl, &isYield);

  return (isYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num,
                                adc_continuous_handle_t *out_handle)
{
  adc_continuous_handle_t handle = NULL;

  adc_continuous_handle_cfg_t adc_config = {
      .max_store_buf_size = 1024,
      .conv_frame_size = EXAMPLE_READ_LEN,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

  adc_continuous_config_t dig_cfg = {
      .sample_freq_hz = 1000, // Changing sampling freq to 1k, since 20k not needed
      .conv_mode = ADC_CONV_MODE,
      .format = ADC_OUTPUT_TYPE,
  };

  adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
  dig_cfg.pattern_num = channel_num;
  for (int i = 0; i < channel_num; i++)
  {
    uint8_t unit = GET_UNIT(channel[i]);
    uint8_t ch = channel[i] & 0x7;
    adc_pattern[i].atten = ADC_ATTEN_DB_11;
    adc_pattern[i].channel = ch;
    adc_pattern[i].unit = unit;
    adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
    ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
    ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
  }
  dig_cfg.adc_pattern = adc_pattern;
  ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
  exit_adc_loop = 0; // enable loop exit mechanism
  *out_handle = handle;
}

#if !CONFIG_IDF_TARGET_ESP32
static bool check_valid_data(const adc_digi_output_data_t *data)
{
  const unsigned int unit = data->type2.unit;
  if (unit > 2)
    return false;
  if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(unit))
    return false;

  return true;
}
#endif

// Software timer callback, this timer can be used for ADC timer
void TimerExpiredActionCallback(TimerHandle_t xTimer)
{
  TURN_OFF_RELAY;
  // Action to be takes when timer expires
  // Stop ADC
  adc_continuous_stop(handle);
  ESP_LOGI(TAG, "ADC Sampling stopped");
  // stop adc loop
  exit_adc_loop = 1;
  // proceed for adc calculation
}

void monitor_main(void)
{
  esp_err_t ret;
  uint32_t ret_num = 0;
  uint8_t result[EXAMPLE_READ_LEN] = {0};

  // Create a timer for ADC to run till that time
  TimerHandle_t timerHandleADC;
  memset(result, 0xcc, EXAMPLE_READ_LEN);

  timerHandleADC = xTimerCreate("ADCTimer", pdMS_TO_TICKS(1000), pdFALSE, // NO AUTO RELOAD, its one shot
                                (void *)0,
                                TimerExpiredActionCallback // Callback for after timer expiry
  );

  if (timerHandleADC == NULL)
  {
    ESP_LOGI(TAG, "Timer Creation failed");
    while (1)
      ;
  }

  xTimerStart(timerHandleADC, 0);
  // Init Relay GPIO as OUTPUT mode
  gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);
  if (gpio_get_level(RELAY_PIN) == 0)
  {
    ESP_LOGI(TAG, "Relay is OFF");
    replay_position = 0; // gpio_get_level API few clock cycles to read, lets read once and preserve value, untill unless needed to read it again
  }

  else
  {
    ESP_LOGI(TAG, "Relay is ON");
    replay_position = 1;
  }

  adc_task_hndl = xTaskGetCurrentTaskHandle();

  adc_continuous_handle_t handle = NULL;
  continuous_adc_init(&channel, sizeof(channel) / sizeof(adc_channel_t),
                      &handle);

  adc_continuous_evt_cbs_t cbs = {
      .on_conv_done = adc_cb_conversion,
  };
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
  ESP_ERROR_CHECK(adc_continuous_start(handle));
  TURN_ON_RELAY;
  while (1)
  {

    /**
     * This is to show you the way to use the ADC continuous mode driver event
     * callback. This `ulTaskNotifyTake` will block when the data processing in
     * the task is fast. However in this example, the data processing (print) is
     * slow, so you barely block here.
     *
     * Without using this event callback (to notify this task), you can still
     * just call `adc_continuous_read()` here in a loop, with/without a certain
     * block timeout.
     */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (exit_adc_loop == 1)
      break;

    while (!exit_adc_loop)
    {
      ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
      if (ret == ESP_OK)
      {
        ESP_LOGI("TASK", "ret is %x, ret_num is %" PRIu32, ret, ret_num);
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
          adc_digi_output_data_t *p = (void *)&result[i];
#if CONFIG_IDF_TARGET_ESP32
          ESP_LOGI(TAG, "Unit: %d, Channel: %d, Value: %x", 1, p->type1.channel,
                   p->type1.data);
#else
          if (ADC_CONV_MODE == ADC_CONV_BOTH_UNIT ||
              ADC_CONV_MODE == ADC_CONV_ALTER_UNIT)
          {
            if (check_valid_data(p))
            {
              ESP_LOGI(TAG, "Unit: %d,_Channel: %d, Value: %x, Analog Vaue : %d",
                       p->type2.unit + 1, p->type2.channel, p->type2.data, (int)(p->type2.data / 1.444));
            }
            else
            {
              ESP_LOGI(TAG, "Invalid data [%d_%d_%x]", p->type2.unit + 1,
                       p->type2.channel, p->type2.data);
            }
          }
#if CONFIG_IDF_TARGET_ESP32S2
          else if (ADC_CONV_MODE == ADC_CONV_SINGLE_UNIT_2)
          {
            ESP_LOGI(TAG, "Unit: %d, Channel: %d, Value: %x", 2,
                     p->type1.channel, p->type1.data);
          }
          else if (ADC_CONV_MODE == ADC_CONV_SINGLE_UNIT_1)
          {
            ESP_LOGI(TAG, "Unit: %d, Channel: %d, Value: %x", 1,
                     p->type1.channel, p->type1.data);
          }
#endif // #if CONFIG_IDF_TARGET_ESP32S2
#endif
        }
        /**
         * Because printing is slow, so every time you call `ulTaskNotifyTake`,
         * it will immediately return. To avoid a task watchdog timeout, add a
         * delay here. When you replace the way you process the data, usually
         * you don't need this delay (as this task will block for a while).
         */
        vTaskDelay(1);
      }
      else if (ret == ESP_ERR_TIMEOUT)
      {
        // We try to read `EXAMPLE_READ_LEN` until API returns timeout, which
        // means there's no available data
        break;
      }
    }
  }

  ESP_ERROR_CHECK(adc_continuous_stop(handle));
  ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
