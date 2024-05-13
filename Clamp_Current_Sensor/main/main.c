/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "EmonLib.h"

static const char TAG[] = "ClampCurrentSensor";

#define CURRENT_ALARM_THRESHOLD    5 // A

static bool eventTrigger = false;

void app_main(void)
{
    EnergyMonitor_init(); // Init ADC
    EnergyMonitor_currentCalib(90.91);  // Calibrate current: ICAL = 100A / (R * 50mA)

    while (1)
    {
        double Irms = EnergyMonitor_calcIrms(1676);  // Calculate Irms only

        char floatString1[20];
        snprintf(floatString1, sizeof(floatString1), "%0.1f", Irms);
        printf("Current value: Irms = %s A\n", floatString1);

        if (Irms >= CURRENT_ALARM_THRESHOLD) {
            if  (eventTrigger == false) {
                eventTrigger = true;
                ESP_LOGE(TAG, "WARNING OVERLOAD !");
            }
        } else {
            if (eventTrigger == true) {
                eventTrigger = false;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
