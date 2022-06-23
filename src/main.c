/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stm32f0xx_hal.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "led.h"
#include "button.h"

#include "cfg.h"
#include "eeprom.h"

#include "usb_device.h"
#include "usbcomm.h"

#include "lps25h.h"
#include "test_support.h"
#include "production_test.h"

#include "deca_device_api.h"
#include "examples_defines.h"

#include "uwb.h"
#define POWER_LEVELS 10

const uint8_t *uid = (uint8_t *)MCU_ID_ADDRESS;

static void handleButton(void);
static void bootload(void);

void test_run_info(unsigned char *data)
{
    printf("%s\r\n", data);
}

static StaticTask_t xExampleTask;
static StackType_t ucExampleStack[configMINIMAL_STACK_SIZE];

static void example_task(void *pvParameters)
{
  printf("ID:%08x\r\n", dwt_readdevid());
  printf("====examples====\r\n");
  build_examples();
  example_pointer();
}

void systemInit()
{
  bool selftestPasses = true;

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();

  // Light up all LEDs to test
  ledOn(ledRanging);
  ledOn(ledSync);
  ledOn(ledMode);
  buttonInit(buttonIdle);

  printf("\r\n\r\n====================\r\n");

  printf("SYSTEM\t: CPU-ID: ");
  for (int i = 0; i < 12; i++)
  {
    printf("%02x", uid[i]);
  }
  printf("\r\n");

  // Initializing pressure sensor (if present ...)
  lps25hInit(&hi2c1);
  testSupportPrintStart("Initializing pressure sensor");
  if (lps25hTestConnection())
  {
    printf("[OK]\r\n");
    lps25hSetEnabled(true);
  }
  else
  {
    printf("[FAIL] (%u)\r\n", (unsigned int)hi2c1.ErrorCode);
    selftestPasses = false;
  }

  testSupportPrintStart("Pressure sensor self-test");
  testSupportReport(&selftestPasses, lps25hSelfTest());

  // Initializing i2c eeprom
  eepromInit(&hi2c1);
  testSupportPrintStart("EEPROM self-test");
  testSupportReport(&selftestPasses, eepromTest());

  cfgInit();

  ledOff(ledRanging);
  ledOff(ledSync);
  ledOff(ledMode);

  usbcommSetSystemStarted(true);

  // xTaskCreateStatic(example_task, "exampleTask", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 3, ucExampleStack, &xExampleTask);
  uwbInit();
  uwbStart();
}

static StaticTask_t xMainTask;
static StackType_t ucMainStack[configMINIMAL_STACK_SIZE];

static void main_task(void *pvParameters)
{
  char ch;
  systemInit();

  while (1)
  {
    usbcommPrintWelcomeMessage();
    ledTick();
    handleButton();
    vTaskDelay(100);
    usbcommRead(&ch, 1);
    if (ch == 'u')
    {
      bootload();
    }
  }
}

int main()
{
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Configure the system clock
  SystemClock_Config();

  // Setup main task
  xTaskCreateStatic(main_task, "main", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, ucMainStack, &xMainTask);

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();

  // Should never reach there
  while (1)
    ;

  return 0;
}

/* Function required to use "printf" to print on serial console */
int _write(int fd, const void *buf, size_t count)
{
  // stdout
  if (fd == 1)
  {
#ifdef USE_FTDI_UART
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, count, HAL_MAX_DELAY);
#else
    usbcommWrite(buf, count);
#endif
  }

  // stderr
  if (fd == 2)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, count, HAL_MAX_DELAY);
  }

  return count;
}

static void handleButton(void)
{
  ButtonEvent be = buttonGetState();

  if (be == buttonShortPress)
  {
    ledBlink(ledRanging, true);
    // TODO: Implement and remove ledblink
  }
  else if (be == buttonLongPress)
  {
    ledBlink(ledSync, true);
    // TODO: Implement and remove ledblink
  }

  buttonProcess();
}

// Enter bootloader from software: Taken from micropython machine_bootloader function
static void bootload(void)
{
  printf("Entering DFU Mode\r\n");
  HAL_Delay(500);

  HAL_RCC_DeInit();
  HAL_DeInit();

  __HAL_REMAPMEMORY_SYSTEMFLASH();

  // arm-none-eabi-gcc 4.9.0 does not correctly inline this
  //     //     // MSP function, so we write it out explicitly here.
  //__set_MSP(*((uint32_t*) 0x00000000));
  __ASM volatile("movs r3, #0\nldr r3, [r3, #0]\nMSR msp, r3\n"
                 :
                 :
                 : "r3", "sp");

  ((void (*)(void)) * ((uint32_t *)0x00000004))();

  while (1)
    ;
}

// Freertos required callbacks
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  static StaticTask_t xIdleTaskTCB;
  static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vAssertCalled(unsigned long ulLine, const char *const pcFileName)
{
  printf("Assert failed at %s:%lu", pcFileName, ulLine);
  while (1)
    ;
}
