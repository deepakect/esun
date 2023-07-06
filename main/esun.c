/* Copyright @ Blu Systems Pvt Ltd */
#include <stdio.h>

#include "nrgy/monitor.h"
#include "sys/health.h"
#include "wifi/softap.h"
#include "blufi/blufi.h"
#include "ble/server.h"
#include "nvs/nvs.h"

TaskHandle_t g_MonitorTaskHandle = NULL;
int gtimerToRun = 60; //time in minutes

void app_main(void)
{
  /* Init nvs flash */
  nvs_init();
  
  health_init();

  ble_srvr_init();

  blufi_init();

  wifi_sap_init();

  //Create monitor task
  xTaskCreate(monitor_main, "MONITOR_TASK", 2000, (void*) gtimerToRun, tskIDLE_PRIORITY, &g_MonitorTaskHandle);
  //if(g_MonitorTaskHandle != NULL)
  //vTaskStartScheduler();
  //monitor_main(gtimerToRun);

}
