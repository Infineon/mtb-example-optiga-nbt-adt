// SPDX-FileCopyrightText: 2024 Infineon Technologies AG
// SPDX-License-Identifier: MIT

/**
 * \file main.c
 * \brief Main function starting up FreeRTOS for NBT asynchronous data transfer usecase.
 */
#include <stddef.h>
#include <stdio.h>

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "infineon/ifx-logger.h"
#include "infineon/logger-printf.h"
#include "infineon/logger-cyhal-rtos.h"
#include "infineon/ifx-protocol.h"
#include "infineon/i2c-cyhal.h"
#include "infineon/ifx-t1prime.h"
#include "infineon/nbt-cmd.h"

#include "nbt-utilities.h"

/**
 * \brief String used as source information for logging.
 */
#define LOG_TAG "NBT example"

/**
 * \brief NBT framework logger.
 */
ifx_logger_t logger_implementation;

/**
 * \brief ModusToolbox CYHAL I2C driver for communication with NBT.
 */
static cyhal_i2c_t i2c_device;

/**
 * \brief Adapter between ModusToolbox CYHAL I2C driver and NBT library framework.
 */
static ifx_protocol_t driver_adapter;

/**
 * \brief Communication protocol stack for NBT library framework.
 */
static ifx_protocol_t communication_protocol;

/**
 * \brief NBT abstraction.
 */
static nbt_cmd_t nbt;

/**
 * \brief FreeRTOS mutex waiting for user button presses.
 */
static SemaphoreHandle_t btn_irq_sleeper;

/**
 * \brief Interrupt handler for user button.
 *
 * \param[in] handler_arg ignored.
 * \param[in] event ignored.
 */
static void btn_irq(void *handler_arg, cyhal_gpio_event_t event)
{
    (void) handler_arg;
    (void) event;

    static BaseType_t xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(btn_irq_sleeper, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * \brief Callback data for btn_irq().
 */
static cyhal_gpio_callback_data_t btn_irq_data = {.callback = btn_irq, .callback_arg = NULL};

/**
 * \brief Configures NBT for asynchronous data transfer usecase.
 * \details No ModusToolbox or FreeRTOS specifics, can be used on any platform.
 * \param[in] nbt NBT abstraction for communication.
 * \return ifx_status_t \c SUCCESS if successful, any other value in case of error.
 */
ifx_status_t nbt_configure_adt(nbt_cmd_t *nbt)
{
    if (nbt == NULL)
    {
        return IFX_ERROR(LIB_NBT_APDU, NBT_SET_CONFIGURATION, IFX_ILLEGAL_ARGUMENT);
    }
    const nbt_file_access_policy_t fap_cc = {.file_id = NBT_FILEID_CC,
                                             .i2c_read_access_condition = NBT_ACCESS_ALWAYS,
                                             .i2c_write_access_condition = NBT_ACCESS_NEVER,
                                             .nfc_read_access_condition = NBT_ACCESS_ALWAYS,
                                             .nfc_write_access_condition = NBT_ACCESS_NEVER};
    const nbt_file_access_policy_t fap_ndef = {.file_id = NBT_FILEID_NDEF,
                                               .i2c_read_access_condition = NBT_ACCESS_ALWAYS,
                                               .i2c_write_access_condition = NBT_ACCESS_ALWAYS,
                                               .nfc_read_access_condition = NBT_ACCESS_ALWAYS,
                                               .nfc_write_access_condition = NBT_ACCESS_NEVER};
    const nbt_file_access_policy_t fap_fap = {.file_id = NBT_FILEID_FAP,
                                              .i2c_read_access_condition = NBT_ACCESS_ALWAYS,
                                              .i2c_write_access_condition = NBT_ACCESS_ALWAYS,
                                              .nfc_read_access_condition = NBT_ACCESS_ALWAYS,
                                              .nfc_write_access_condition = NBT_ACCESS_ALWAYS};
    const nbt_file_access_policy_t fap_proprietary1 = {.file_id = NBT_FILEID_PROPRIETARY1,
                                                       .i2c_read_access_condition = NBT_ACCESS_ALWAYS,
                                                       .i2c_write_access_condition = NBT_ACCESS_NEVER,
                                                       .nfc_read_access_condition = NBT_ACCESS_ALWAYS,
                                                       .nfc_write_access_condition = NBT_ACCESS_ALWAYS};
    const nbt_file_access_policy_t fap_proprietary2 = {.file_id = NBT_FILEID_PROPRIETARY2,
                                                       .i2c_read_access_condition = NBT_ACCESS_ALWAYS,
                                                       .i2c_write_access_condition = NBT_ACCESS_ALWAYS,
                                                       .nfc_read_access_condition = NBT_ACCESS_ALWAYS,
                                                       .nfc_write_access_condition = NBT_ACCESS_NEVER};
    const nbt_file_access_policy_t fap_proprietary3 = {.file_id = NBT_FILEID_PROPRIETARY3,
                                                       .i2c_read_access_condition = NBT_ACCESS_NEVER,
                                                       .i2c_write_access_condition = NBT_ACCESS_NEVER,
                                                       .nfc_read_access_condition = NBT_ACCESS_NEVER,
                                                       .nfc_write_access_condition = NBT_ACCESS_NEVER};
    const nbt_file_access_policy_t fap_proprietary4 = {.file_id = NBT_FILEID_PROPRIETARY4,
                                                       .i2c_read_access_condition = NBT_ACCESS_NEVER,
                                                       .i2c_write_access_condition = NBT_ACCESS_NEVER,
                                                       .nfc_read_access_condition = NBT_ACCESS_NEVER,
                                                       .nfc_write_access_condition = NBT_ACCESS_NEVER};
    const nbt_file_access_policy_t *faps[] = {&fap_cc, &fap_ndef, &fap_fap, &fap_proprietary1, &fap_proprietary2, &fap_proprietary3, &fap_proprietary4};
    const struct nbt_configuration configuration = {.fap = (nbt_file_access_policy_t **) faps,
                                                    .fap_len = sizeof(faps) / sizeof(struct nbt_configuration *),
                                                    .communication_interface = NBT_COMM_INTF_NFC_ENABLED_I2C_ENABLED,
                                                    .irq_function = NBT_GPIO_FUNCTION_DISABLED};

    return nbt_configure(nbt, &configuration);
}

/**
 * \brief Example FreeRTOS task showing how to use NBT asynchronous data transfer functionality.
 *
 * \details
 *   * Opens communication channel to NBT.
 *   * Configures NBT for asynchronous data transfer usecase.
 *   * Reads configuration from NBT proprietary file 1.
 *   * Sets LED state depending on configuration.
 *   * Writes LED state to NBT proprietary file 2.
 *   * Waits for button interrupt and starts loop again.
 *
 * \see nbt_configure_adt()
 */
void nbt_adt_task(void *arg)
{
    // Activate communication channel to NBT
    uint8_t *atpo = NULL;
    size_t atpo_len = 0U;
    ifx_status_t status = ifx_protocol_activate(&communication_protocol, &atpo, &atpo_len);
    if (ifx_error_check(status))
    {
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_FATAL, "Could not open communication channel to NBT");
        goto cleanup;
    }

    // Set NBT to ADT configuration
    status = nbt_configure_adt(&nbt);
    if (ifx_error_check(status))
    {
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_FATAL, "Could not set NBT to ADT configuration");
        goto cleanup;
    }

    // Give semaphore in order to execute an initial read/write without a button-press
    xSemaphoreGive(btn_irq_sleeper);

    while (1)
    {
        // Wait for button interrupt
        while (xSemaphoreTake(btn_irq_sleeper, portMAX_DELAY) != pdPASS)
            ;

        // Use NBT command abstraction
        status = nbt_select_nbt_application(&nbt);
        if (ifx_error_check(status))
        {
            // In ADT mode, the NBT's communication states are mutually exclusive.
            // Thus, the I2C communication is blocked as long as an NFC field is present.
            // For this example, this means that the host MCU will be unable to communicate via I2C as long as the mobile phone remains tapped to the NBT'S NFC antenna.
            ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_WARN, "Could not select NBT application (NFC field still present?)!");
            continue;
        }

        // Read LED state from proprietary file 1
        uint8_t file_content[1];
        status = nbt_read_file(&nbt, NBT_FILEID_PROPRIETARY1, 0U, 1U, file_content);
        if (ifx_error_check(status))
        {
            ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_WARN, "Could not read NBT proprietary file 1");
            continue;
        }

        // Update LED based on given configuration
        bool led_on = file_content[0] == 0x01U;
        if(led_on)
        {
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
            ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_INFO, "LED turned ON");
        }
        else
        {
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
            ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_INFO, "LED turned OFF");
        }

        // Write back configuration as confirmation for phone
        uint8_t update_data[] = {led_on ? 0x01U : 0x00U};
        status = nbt_write_file(&nbt, NBT_FILEID_PROPRIETARY2, 0U, update_data, sizeof(update_data));
        if (ifx_error_check(status))
        {
            ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_WARN, "Could not write NBT proprietary file 2");
            continue;
        }
    }

cleanup:
    cyhal_i2c_free(&i2c_device);
    ifx_protocol_destroy(&communication_protocol);
    nbt_destroy(&nbt);
    vTaskDelete(NULL);
}

/**
 * \brief Main function starting NBT asynchronous data transfer usecase via FreeRTOS tasks.
 * \details Prepares ModusToolbox and NBT framwework components and starts actual ADT task
 * \see nbt_adt_task
 */
int main(void)
{
    ///////////////////////////////////////////////////////////////////////////
    // ModusTooblbox start-up boilerplate
    ///////////////////////////////////////////////////////////////////////////
    cy_rslt_t result;
#if defined(CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    __enable_irq();

    ///////////////////////////////////////////////////////////////////////////
    // ModusTooblbox component configuration
    ///////////////////////////////////////////////////////////////////////////

    // RetargetIO for logging data via serial connection
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    printf("\x1b[2J\x1b[;H");
    printf("****************** "
           "NBT: Asynchronous Data Transfer "
           "****************** \r\n\n");

    // I2C driver for communication with NBT
    cyhal_i2c_cfg_t i2c_cfg = {.is_slave = false, .address = 0x00U, .frequencyhal_hz = 400000U};
    result = cyhal_i2c_init(&i2c_device, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    result = cyhal_i2c_configure(&i2c_device, &i2c_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    // User LED displaying current state
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    // User button to trigger update checks
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    btn_irq_sleeper = xSemaphoreCreateBinary();
    if (btn_irq_sleeper == NULL)
    {
        CY_ASSERT(0);
    }
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &btn_irq_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_RISE, configMAX_PRIORITIES - 1U, true);

    ///////////////////////////////////////////////////////////////////////////
    // NBT library configuration
    ///////////////////////////////////////////////////////////////////////////

    // Logging framework
    ifx_status_t status = logger_printf_initialize(&logger_implementation);
    if (ifx_error_check(status))
    {
        CY_ASSERT(0);
    }
    status = ifx_logger_set_level(&logger_implementation, IFX_LOG_DEBUG);
    if (ifx_error_check(status))
    {
        CY_ASSERT(0);
    }
    status = logger_cyhal_rtos_initialize(ifx_logger_default, &logger_implementation);
    if (ifx_error_check(status))
    {
        CY_ASSERT(0);
    }
    status = ifx_logger_set_level(ifx_logger_default, IFX_LOG_DEBUG);
    if (ifx_error_check(status))
    {
        CY_ASSERT(0);
    }
    status = logger_cyhal_rtos_start(ifx_logger_default, NULL);
    if (ifx_error_check(status))
    {
        CY_ASSERT(0);
    }

    // I2C driver adapter
    status = i2c_cyhal_initialize(&driver_adapter, &i2c_device, NBT_DEFAULT_I2C_ADDRESS);
    if (ifx_error_check(status))
    {
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_ERROR, "Could not initialize I2C driver adapter");
        CY_ASSERT(0);
    }

    // Communication protocol (data link layer)
    status = ifx_t1prime_initialize(&communication_protocol, &driver_adapter);
    if (ifx_error_check(status))
    {
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_ERROR, "Could not initialize NBT communication protocol");
        CY_ASSERT(0);
    }
    ifx_protocol_set_logger(&communication_protocol, ifx_logger_default);

    // NBT command abstraction
    status = nbt_initialize(&nbt, &communication_protocol, ifx_logger_default);
    if (ifx_error_check(status))
    {
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_ERROR, "Could not initialize NBT abstraction");
        CY_ASSERT(0);
    }

    ///////////////////////////////////////////////////////////////////////////
    // FreeRTOS start-up
    ///////////////////////////////////////////////////////////////////////////
    xTaskCreate(nbt_adt_task, (char *) "NBT example", 2048U, 0U, configMAX_PRIORITIES - 1U, NULL);
    vTaskStartScheduler();

    ///////////////////////////////////////////////////////////////////////////
    // Cleanup (should not be reached)
    ///////////////////////////////////////////////////////////////////////////
    cy_retarget_io_deinit();
    ifx_logger_destroy(&logger_implementation);

    CY_ASSERT(0);
    return -1;
}
