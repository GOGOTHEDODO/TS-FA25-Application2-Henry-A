/* --------------------------------------------------------------
   Application: 02 - Rev2
   Release Type: Baseline Preemption
   Class: Real Time Systems - Fa 2025
   Author: [Henry Abshire]
   Email: [he248516@ucf.edu]
   Company: [University of Central Florida]
   Website: N/A
   AI Use: Commented inline -- None
---------------------------------------------------------------*/

/*----------------------------------------------------------------
Senario: Earthquake Early Warning System (EEWS)

-----------------------------------------------------------------*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "math.h"

#define LED_PIN GPIO_NUM_2 // Using GPIO2 for the LED
#define LDR_PIN GPIO_NUM_32
#define LDR_ADC_CHANNEL ADC1_CHANNEL_4
// TODO99: Consider Adding AVG_WINDOW and SENSOR_THRESHOLD as global defines

int AVG_WINDOW = 10;
int SENSOR_THRESHOLD = 500;
double GAMA = 0.7;

// Consider supressing the output
void systemActiveIndicatorTask(void *pvParameters)
{
    bool led_status = false;
    TickType_t currentTime = pdTICKS_TO_MS(xTaskGetTickCount());

    while (1)
    {
        currentTime = pdTICKS_TO_MS(xTaskGetTickCount());
        gpio_set_level(LED_PIN, 1); // TODO: Set LED pin high or low based on led_status flag;
        led_status = true;    // TODO: toggle state for next loop
        printf("System Alive and monitoring: %s @ %lu\n", led_status ? "ON" : "OFF", currentTime);
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 ms using MS to Ticks Function vs alternative which is MS / ticks per ms
        gpio_set_level(LED_PIN, 0); //set low
        led_status = false;   //toggle
        printf("System Alive and monitoring: %s @ %lu\n", led_status ? "ON" : "OFF", currentTime);
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 ms
    }
    vTaskDelete(NULL); // We'll never get here; tasks run forever
}

// TODO10: Task to print a message every 1000 ms (1 seconds)
void printSeismicReadings(void *pvParameters)
{
    TickType_t currentTime = pdTICKS_TO_MS(xTaskGetTickCount());
    TickType_t previousTime = 0;
    int count = 0; 
    while (1)
    {
        previousTime = currentTime;
        currentTime = pdTICKS_TO_MS(xTaskGetTickCount());
        // Prints a periodic message based on a thematic area. Output a timestamp (ms) and period (ms
        printf("Seismic monitor update: system stable. No major erros @ time %lu [period = %lu]!\n", currentTime, currentTime - previousTime);
        count++;
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000 ms
    }
    vTaskDelete(NULL); // We'll never get here; tasks run forever
}

long fib(long i)
{
    if (i <= 1)
        return i;
    //printf("FIBONACCI %ld \n", i);
    return fib(i - 1) + fib(i - 2);
}

// TODO11: Create new task for sensor reading every 500ms
void sensor_task(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount(); // Initialize last wake time

    // Variables to compute LUX
    int raw;
    float Vmeasure = 0.;
    float Rmeasure = 0.;
    float lux = 0.;
    // Variables for moving average
    int luxreadings[AVG_WINDOW];
    for (int i = 0; i < AVG_WINDOW; ++i) {
        luxreadings[i] = 0; // Initialize all readings to zero
    }
    int idx = 0;
    float sum = 0;
    // See TODO 99
    //  Pre-fill the readings array with an initial sample to avoid startup anomaly
    for (int i = 0; i < AVG_WINDOW; ++i)
    {
        raw = adc1_get_raw(LDR_ADC_CHANNEL);
        Vmeasure = ((float) raw / 4096.0) * 3.3; // TODO11b correct this with the equation seen earlier
        //book equation
        //Rmeasure = (2000.0 * Vmeasure)/(1 - ((float) Vmeasure/3.3)); // TODO11c correct this with the equation seen earlier
        //corrected equation from math
        Rmeasure = (10000.0 * (Vmeasure/3.3))/(1 - Vmeasure/3.3);
        //simplified equation not using due to me possibly being bad a math
        //Rmeasure = 10000.0/(3.3/Vmeasure - 1);

        lux = ( pow((50.0*1000.0*pow(10.0,GAMA))/Rmeasure, (1.0/GAMA)) );      // TODO11d correct this with the equation seen earlier
        luxreadings[i] = lux;
        sum += luxreadings[i];
    }

    const TickType_t periodTicks = pdMS_TO_TICKS(500); // e.g. 500 ms period
    //lastWakeTime = xTaskGetTickCount();     // initialize last wake time
    TickType_t prevTime = 0;
    while (1)
    {
        // Read current sensor value
        raw = adc1_get_raw(LDR_ADC_CHANNEL);
        // printf("**raw **: Sensor %d\n", raw);

        // Compute LUX
        Vmeasure = ((float)raw / 4096.0) * 3.3;
        Rmeasure = (10000.0 * (Vmeasure / 3.3)) / (1 - Vmeasure / 3.3);
        // Rmeasure = (10000.0 * (Vmeasure / 3.3)) / (1 - Vmeasure / 3.3);
        lux = (pow(50.0 * 1000.0 * pow(10.0, GAMA) / Rmeasure, (1.0 / GAMA)));

        // Update moving average buffer
        sum -= luxreadings[idx]; // remove oldest value from sum

        luxreadings[idx] = lux; // place new reading
        sum += lux;             // add new value to sum
        idx = (idx + 1) % AVG_WINDOW;
        int avg = sum / AVG_WINDOW; // compute average

        // TODO11h Check threshold and print alert if exceeded or below based on context
        if (avg >= SENSOR_THRESHOLD)
        {
            printf("**Alert**: Sensor average %d significant seismic activity detected!\n", avg);
        }
        else
        {
            printf("Sensor average %d, below threshold %d\n", avg, SENSOR_THRESHOLD);
        }

        // TODO11j: Print out time period [to help with answering Eng/Analysis quetionst (hint check Application Solution #1 )
        //printf("Sensor Time Peroid = %lu ms\n %lu, %lu", (pdTICKS_TO_MS(xTaskGetTickCount()) - pdTICKS_TO_MS(prevTime)), pdTICKS_TO_MS(prevTime), pdTICKS_TO_MS(lastWakeTime));

        printf("Sensor Time Peroid = %lu ms\n", (pdTICKS_TO_MS(xTaskGetTickCount()) - pdTICKS_TO_MS(prevTime)));

        prevTime = lastWakeTime;
        // https://wokwi.com/projects/430683087703949313
        // TODO11k Replace vTaskDelay with vTaskDelayUntil with parameters &lastWakeTime and periodTicks
        //THIS IS WHERE I CAUSE STARVATION UNCOMMENT TO SEE ONLY SENSOR WORK OR IF YOU WOULD LIKE TO KNOW THE FIRST 30 FIB NUMBERS
        //I DON"T KNOW IF THAT WOULD EVER BE USEFULL, BUT HEY YOU DO YOU
        //fib(30);
        vTaskDelayUntil(&lastWakeTime, periodTicks); //this updates last wake time, which is why the period is off
    }
}

void app_main()
{
    // Initialize LED GPIO
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LDR_PIN); 
    gpio_set_direction(LDR_PIN, GPIO_MODE_INPUT);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_11); 
    // Instantiate/ Create tasks:
    // . pointer to task function,
    // . descriptive name, [has a max length; located in the FREERTOS_CONFIG.H]
    // . stack depth,
    // . parameters [optional] = NULL
    // . priority [0 = low],
    // . pointer referencing this created task [optional] = NULL
    // Learn more here https://www.freertos.org/Documentation/02-Kernel/04-API-references/01-Task-creation/01-xTaskCreate

    //priotiy zero is not used becasue theoretically if we are using time slicing, the idle task is priority 0 and could be sliced to
    //by using one we always put something over doing nothing
    xTaskCreatePinnedToCore(systemActiveIndicatorTask, "LEDTask", 2048, NULL, 1, NULL, 1); //blinking the led is the lowest priority task as it is nice to know
                                                                          // but the pint task will give the same system running info, 
    xTaskCreatePinnedToCore(printSeismicReadings, "PrintTask", 2048, NULL, 2, NULL, 1); //print is second lowest as it provides important info, 
                                                                                     //but is not system critical
    xTaskCreatePinnedToCore(sensor_task, "SensorTask", 8192, NULL, 3, NULL, 1); //the sensor task is the highest priority as it is critical to system operation
    // TODO12 Add in new Sensor task; make sure it has the correct priority to preempt
}