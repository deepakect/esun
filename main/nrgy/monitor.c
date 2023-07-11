/* Copyright @ Blu Systems Pvt Ltd */
#include "esp_log.h"
#include "sdkconfig.h"
#include "cmn/cmn.h"
#include "driver/gpio.h"
#include "monitor.h"

//static TaskHandle_t adc_task_hndl;
static const char *TAG = "NRGY:MONITOR";

#define RELAY_PIN GPIO_NUM_3 // Relay pin location
bool g_replay_position = 0;    // Varible to check relay position, use extern to access it elsewhere
#define TURN_ON_RELAY gpio_set_level(RELAY_PIN, 1); ESP_LOGI(TAG,"Relay ON!")
#define TURN_OFF_RELAY gpio_set_level(RELAY_PIN, 0);ESP_LOGI(TAG,"Relay OFF!")
bool g_exit_adc_loop;

#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_2
#define ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED 1
#define ACS_PRESENCE_CHECK    1
static float g_ACSZeroCurrentReading;
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static float g_Current_consumed = 0;
static float g_PresentConsumption = 0;
adc_oneshot_unit_handle_t adc1_handle;
extern TaskHandle_t g_MonitorTaskHandle;
adc_cali_handle_t adc1_cali_handle = NULL;

char TaskList[500];

// Software timer callback, this timer can be used for ADC timer
static void TimerExpiredActionCallback(TimerHandle_t xTimer)
{
  if (g_replay_position)
  {
    TURN_OFF_RELAY;
  }
  // Action to be takes when timer expires
  // Stop ADC
  //adc_continuous_stop(handle);
  ESP_LOGI(TAG, "ADC Sampling stopped");
  // stop adc loop
  g_exit_adc_loop = 1;
  ESP_LOGI(TAG, "Power consumed : %.4f kWhr", g_Current_consumed/(float)3600);
  // proceed for task deletion
  example_adc_calibration_deinit(adc1_cali_handle);
  adc1_cali_handle = NULL;
  vTaskDelete(g_MonitorTaskHandle);
  g_MonitorTaskHandle = NULL;
}

void monitor_main(void * timerToRun)
{
  static int adc_raw_avg;
  static int adc_voltage;
  int temp_value;
  uint32_t time_to_run = 10;

  ESP_LOGI(TAG,"time to run : %d minutes", (int)timerToRun);
  time_to_run =  ((int )timerToRun) * 60 * 1000; //make minutes to milli-seconds
  
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
    ESP_LOGI(TAG, "Status : Relay is OFF");
    g_replay_position = 0; // gpio_get_level API few clock cycles to read, lets read once and preserve value, untill unless needed to read it again
  }

  else
  {
    ESP_LOGI(TAG, "Status : Relay is ON");
    g_replay_position = 1;
  }

  //adc_task_hndl = xTaskGetCurrentTaskHandle();

  //-------------ADC1 Init---------------//
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  //-------------ADC1 Config---------------//
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_12, //ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_11,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));

  bool do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);
  //Calculate chip supply voltage at zero passing current
  if (!g_replay_position)
  {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &temp_value));
    adc_raw_avg = temp_value;
    temp_value = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &temp_value));
    adc_raw_avg += temp_value;
    temp_value = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &temp_value));
    adc_raw_avg += temp_value;
    temp_value = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &temp_value));
    adc_raw_avg += temp_value;
    temp_value = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &temp_value));
    adc_raw_avg += temp_value;
    temp_value = 0;
    adc_raw_avg = adc_raw_avg / 5;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw_avg, &adc_voltage));
    //Record zero passing current reading
   // g_ACSZeroCurrentReading = (float)adc_voltage;
    // Presently rounding off for 500mVs as zero current voltage.
    g_ACSZeroCurrentReading = (float)501;
    ESP_LOGI(TAG, "ACS supply voltage %.2f Volts", g_ACSZeroCurrentReading/(float)100);
  }
  // Turn relay on, now start actual operation
  if (!g_replay_position)
  {
    TURN_ON_RELAY;
  }
  //while (!g_exit_adc_loop)
  while(1)
  {
    //For stable reading, its recomendded to have capacitor with average reading from ADC.
    // Presently taking 5 ADC reading, can be improved further to achieve accuracy
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &temp_value));
    adc_raw_avg = temp_value;
    temp_value = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &temp_value));
    adc_raw_avg += temp_value;
    temp_value = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &temp_value));
    adc_raw_avg += temp_value;
    temp_value = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &temp_value));
    adc_raw_avg += temp_value;
    temp_value = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &temp_value));
    adc_raw_avg += temp_value;
    temp_value = 0;
    adc_raw_avg = adc_raw_avg/5;
   #if 1
    if (do_calibration1)
    {
      ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw_avg, &adc_voltage));
#if ACS_PRESENCE_CHECK      
      if( adc_voltage <= 460)
      {
        ESP_LOGI(TAG,"ACS not found");
        break;
      }
#endif
      // Often adc output voltage fluctuates and if it's less than zero crossing current voltage (0.1 * Vadc)
      // Ensure if it's less, to ruound-off to zero consumption
      if ((adc_voltage - g_ACSZeroCurrentReading) < 0)
      {
        g_PresentConsumption = 0;
      }
      else
      {
        // 200mV per ampere would give total current sensed by ACS
        // Presently 230V AC fixed voltage, need to find out better way to tackle this
        g_PresentConsumption = (230 * ((float)(adc_voltage - g_ACSZeroCurrentReading) / (float)200));
      }
      //Keep accumalating readings for every second
      //reading are in kws - killo-watt seconds
      g_Current_consumed += g_PresentConsumption/ (float)1000;
      // suppress the log, once we are ok with the calculations
      ESP_LOGI(TAG,"Total Consumption %.2f kWs , Present consumption : %.2f Watts", g_Current_consumed, g_PresentConsumption);
    }
    #endif
        vTaskDelay(999);
        #if 0
        vTaskList(TaskList);
        ESP_LOGI(TAG,"%s", TaskList);
        ESP_LOGI(TAG," Tatal Tasks : %d, total ticks since boot :%d ", uxTaskGetNumberOfTasks(), xTaskGetTickCount());
        #endif
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
            .bitwidth = ADC_BITWIDTH_12, //ADC_BITWIDTH_DEFAULT,
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