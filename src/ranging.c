#include <stdio.h>
#include <stdbool.h>
#include "ranging.h"
#include "FreeRTOS.h"
#include "task.h"

void rangingTask()
{
    while (true)
    {
        printf("RangingTask\r\n");
        vTaskDelay(2000);
    }
}

void rangingInit()
{
    
    


    // Ranging Task Init
    static StaticTask_t rangingStaticTask;
    static StackType_t rangingStaticStack[configMINIMAL_STACK_SIZE];

    xTaskCreateStatic(rangingTask, "rangingTask", configMINIMAL_STACK_SIZE, NULL,
                      configMAX_PRIORITIES - 1, rangingStaticStack, &rangingStaticTask);
}
