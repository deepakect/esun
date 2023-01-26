/* Copyright @ Blu Systems Pvt Ltd */
#include <stdio.h>

#include "nrgy/monitor.h"
#include "sys/health.h"
#include "wifi/softap.h"
#include "blufi/blufi.h"
#include "ble/server.h"

void app_main(void)
{
  
  //nvs_init();
  
  health_init();

  ble_srvr_init();

  blufi_init();

  wifi_sap_init();

  monitor_main();

}
