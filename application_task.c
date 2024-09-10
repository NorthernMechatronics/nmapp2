/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, Northern Mechatronics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <am_mcu_apollo.h>
#include <am_util.h>

#include <FreeRTOS.h>
#include <task.h>

#include "am_bsp.h"

#include "button.h"
#include "led.h"

#include "application_task.h"
#include "application_task_cli.h"

#if defined(BSP_NM180410) || defined(BSP_NM180411)
    #define APPLICATION_LED AM_BSP_GPIO_LED0
    #define APPLICATION_LED_TIMER_NUMBER    (1)
    #define APPLICATION_LED_TIMER_SEGMENT   AM_HAL_CTIMER_TIMERA
    #define APPLICATION_LED_TIMER_INTERRUPT AM_HAL_CTIMER_INT_TIMERA1C0
#else
    #define APPLICATION_LED AM_BSP_GPIO_LED1
    #define APPLICATION_LED_TIMER_NUMBER    (2)
    #define APPLICATION_LED_TIMER_SEGMENT   AM_HAL_CTIMER_TIMERB
    #define APPLICATION_LED_TIMER_INTERRUPT AM_HAL_CTIMER_INT_TIMERB2C0
#endif

static TaskHandle_t application_task_handle;
static uint32_t current_led_effect;

// Button press handler to cycle through all the LED effects.
static void on_button_pressed(void)
{
    current_led_effect = (current_led_effect + 1) % (LED_COMMAND_MAX + 1);
    if (current_led_effect < LED_COMMAND_MAX)
    {
        led_command_t command = { .ui32Id = current_led_effect, .ui32Repeat = 0 };
        led_send(&command);
    }
    else
    {
        led_command_t command = { .ui32Id = LED_COMMAND_OFF, .ui32Repeat = 0 };
        led_send(&command);
    }
}

static void setup_button(void)
{
    uint32_t handle;
    button_config(&handle, AM_BSP_GPIO_BUTTON0, g_AM_BSP_GPIO_BUTTON0, 1);
    
    // The following register a single short press sequence to the button.
    // Other press sequence are possible.  For example, a two presses sequence
    // with a short press first followed by a long press would be
    // 
    // button_sequence_register(handle, 2, 0b10, on_button_pressed);
    //
    // In general, the order of the sequence starts from LSB to MSB;
    // 0 being a short press and 1 being a long press.
    button_sequence_register(handle, 1, 0b0, on_button_pressed);
}

static void setup_led(void)
{
    // Configure the LED that is connected to a GPIO with CTIMER output.
    const led_config_t led_cfg = {
        .ui32Number    = APPLICATION_LED_TIMER_NUMBER,
        .ui32Segment   = APPLICATION_LED_TIMER_SEGMENT,
        .ui32Interrupt = APPLICATION_LED_TIMER_INTERRUPT,
        .ui32ActiveLow    = 0,
        .ui32Pin       = APPLICATION_LED,
    };

    led_config(&led_cfg);
    current_led_effect = LED_COMMAND_MAX;
}

static void application_task_setup(void)
{
    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED0, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED0, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED1, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED1, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED2, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED2, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED3, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED3, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED4, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED4, AM_HAL_GPIO_OUTPUT_CLEAR);

// The Petal ecosystem has the ability to shutdown the LoRa radio.  In addition,
// the Petal development board has the ability to shutdown the I/O level shifters
// for power savings.
#if defined(BSP_NM180410) || defined(BSP_NM180411)
    am_hal_gpio_pinconfig(AM_BSP_GPIO_PETAL_CORE_nLORA_EN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_PETAL_CORE_nLORA_EN, AM_HAL_GPIO_OUTPUT_SET);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_PETAL_DEV_IO_EN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_PETAL_DEV_IO_EN, AM_HAL_GPIO_OUTPUT_SET);
#endif

    setup_button();
    setup_led();
}

static void application_task_loop(void)
{
    vTaskDelay(pdMS_TO_TICKS(500));
    // if no LED effect is active, just toggle the LED
    if (led_status_get() == LED_STATUS_IDLE)
    {
        am_hal_gpio_state_write(APPLICATION_LED, AM_HAL_GPIO_OUTPUT_TOGGLE);
    }
}

static void application_task(void *parameter)
{
#if defined(CLI_ENABLE)
    application_task_cli_register();
#endif
    application_task_setup();
    while (1)
    {
        application_task_loop();
    }
}

void application_task_create(uint32_t priority)
{
    xTaskCreate(application_task, "application", 512, 0, priority, &application_task_handle);
}
