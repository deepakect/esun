/* Copyright @ Blu Systems Pvt Ltd */
#include "esp_log.h"
#include "sdkconfig.h"
#include "cmn/cmn.h"
#include "driver/gpio.h"
#include "monitor.h"

static TaskHandle_t adc_task_hndl;
static const char *TAG = "NRGY:MONITOR";

#define RELAY_PIN GPIO_NUM_3 // Relay pin location
bool replay_position = 0;    // Varible to check relay position, use extern to access it elsewhere
#define TURN_ON_RELAY gpio_set_level(RELAY_PIN, 1)
#define TURN_OFF_RELAY gpio_set_level(RELAY_PIN, 0)
bool exit_adc_loop;

#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_2

static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static float Current_consumed = 0;
uint32_t time_to_run;

// Software timer callback, this timer can be used for ADC timer
static void TimerExpiredActionCallback(TimerHandle_t xTimer)
{
  if (replay_position)
  {
    TURN_OFF_RELAY;
    ESP_LOGI(TAG, "Relay off!");
  }
  // Action to be takes when timer expires
  // Stop ADC
  //adc_continuous_stop(handle);
  ESP_LOGI(TAG, "ADC Sampling stopped");
  // stop adc loop
  exit_adc_loop = 1;
  ESP_LOGI(TAG, "Power consumed : %.4f kWhr", Current_consumed/(float)3600);
  // proceed for adc calculation
}

void monitor_main(void)
{

  time_to_run =  15 * 60 * 1000; //make minutes to milli-seconds

  // Create a timer for ADC to run till that time
  TimerHandle_t timerHandleADC;

  timerHandleADC = xTimerCreate("ADCTimer", pdMS_TO_TICKS(time_to_run), pdFALSE, // NO AUTO RELOAD, its one shot
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

  //-------------ADC1 Init---------------//
  adc_oneshot_unit_handle_t adc1_handle;
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  //-------------ADC1 Config---------------//
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_11,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));

  adc_cali_handle_t adc1_cali_handle = NULL;
  bool do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);

  if (!replay_position)
  {
    TURN_ON_RELAY;
    ESP_LOGI(TAG,"Relay ON!");
  }
  while (!exit_adc_loop)
  {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
    //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0]);
   #if 1
    if (do_calibration1)
    {
      ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0][0], &voltage[0][0]));
       // Current_consumed += (230 * (float)voltage[0][0] / (float)200) / (long)(3600000000);
       Current_consumed += (float)((float)250 * 4 / (float)1000);
       //if(Current_consumed == 100 || Current_consumed == 250 || Current_consumed == 500 || Current_consumed == 750 )
       ESP_LOGI(TAG,"%.2f kWs", Current_consumed);
      //ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV, current: %2.4f Amps", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, voltage[0][0], Current_consumed);
    }
    #endif
        vTaskDelay(1000);
  }
}


static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}