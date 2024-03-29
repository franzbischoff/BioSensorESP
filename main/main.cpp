// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "settings.h"

#include <cmath>
#if defined(SAVE_DATA_TO_FILE) || defined(LOAD_DATA_FROM_FILE)
    #include "esp_littlefs.h"
    #define LOG_ROTATION_SIZE 30
#endif
#include "esp_log.h"
#include "spi_flash_mmap.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "driver/touch_pad.h"
#include "soc/touch_sensor_channel.h"
#if defined(USE_AD8232_SENSOR)
    #include "driver/gpio.h"
    #include "esp_adc/adc_oneshot.h"
    #include "esp_adc/adc_cali.h"
    #include "esp_adc/adc_cali_scheme.h"
#else
    #include "max32664.hpp"
#endif
#include "mpx/mpx.hpp"

#define HIST_SIZE (uint16_t)(HISTORY_SIZE_S * SAMPLING_RATE_HZ)
#define FLOSS_LANDMARK (HIST_SIZE - (SAMPLING_RATE_HZ * FLOSS_LANDMARK_S))
#define BUFFER_SIZE (WINDOW_SIZE)
#define RING_BUFFER_SIZE (BUFFER_SIZE + 20U) // queue must have a little more room

// ESP_PLATFORM

//>> multitask configurations
void task_compute(void *pv_parameters);
void task_read_signal(void *pv_parameters);

typedef enum cores { CORE_0 = 0x00, CORE_1 = 0x01 } cores_t;

//>> log tag
static const char TAG[] = "main";
#ifndef MAIN_LOG_LEVEL
    #define MAIN_LOG_LEVEL ESP_LOG_INFO
#endif

//>> global variables
static volatile bool buffer_init = false; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static volatile RingbufHandle_t ring_buf; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static volatile bool is_running = false;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

#if defined(SAVE_DATA_TO_FILE) || defined(LOAD_DATA_FROM_FILE)
static FILE *file = nullptr; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
#endif

/// @brief used to initialize the touch pads threshold
static void tp_set_thresholds()
{
    uint16_t touch_value;
    uint16_t touch_sum = 0;
    for (uint8_t i = 0; i < 10; i++) {
        // sample some readings
        touch_pad_read(TOUCH_PAD_GPIO12_CHANNEL, &touch_value);
        touch_sum += touch_value;
    }

    ESP_ERROR_CHECK(touch_pad_set_thresh(TOUCH_PAD_GPIO12_CHANNEL, touch_sum * 2 / 30));
}

/// @brief Interrupt handler for the touch pads
static void tp_rtc_intr(void *arg)
{
    uint32_t pad_intr = touch_pad_get_status();
    // clear interrupt
    touch_pad_clear_status();

    // zero means no interrupt, any other value means a pad was touched
    // caveat: 1 means channel 0 was touched
    if ((pad_intr & (0x01 << TOUCH_PAD_GPIO12_CHANNEL)) != 0) {
        is_running = !is_running; // toggle running state
    }
}

///   @brief Task to compute the matrix profile
/// @param pv_parameters pointer to the task parameters
void task_compute(void *pv_parameters) // This is a task.
{
    (void)pv_parameters;

    [[maybe_unused]] const uint16_t floss_landmark = FLOSS_LANDMARK;
    float buffer[BUFFER_SIZE];
    uint16_t recv_count;
    int16_t delay_adjust = 100;

    for (uint16_t i = 0; i < BUFFER_SIZE; ++i) { // NOLINT(modernize-loop-convert)
        buffer[i] = 0.0F;
    }

    mp::Mpx mpx(WINDOW_SIZE, 0.5F, 0U, HIST_SIZE);
    mpx.prune_buffer();

#if defined(SAVE_DATA_TO_FILE)
    // headers
    if (file == nullptr) {
        ESP_LOGE(TAG, "File is not open");
        vTaskDelete(nullptr);
    }
    fputs("\"data\",\"floss\"\n", file);
#endif

    for (;;) // -H776 A Task shall never return or exit.
    {
        while (!is_running) {
            // User can pause the session
            ESP_LOGD(TAG, "compute: paused");
            vTaskDelay((portTICK_PERIOD_MS * 200));
        }

        if (buffer_init) { // wait for buffer to be initialized
            recv_count = 0;

            for (uint16_t i = 0; i < BUFFER_SIZE; ++i) {
                size_t item_size = 0; // size in bytes (usually 4 bytes)
                float *data = static_cast<float *>(xRingbufferReceive(ring_buf, &item_size, 0));

                if (item_size > 4) {
                    ESP_LOGD(TAG, "Item_size: %d", item_size);
                }

                if (data != nullptr) {
                    buffer[i] = *data; // read data from the ring buffer
                    recv_count = i + 1;
                    vRingbufferReturnItem(ring_buf, static_cast<void *>(data));
                } else {
                    break; // no more data in the ring buffer
                }
            }

            if (recv_count > 0) {
                // ESP_LOGI(TAG, "recv_count: %u", recv_count);

                mpx.compute(buffer, recv_count); /////////////////
                mpx.floss();

                // const uint16_t profile_len = mpx.get_profile_len();
                [[maybe_unused]] float *floss = mpx.get_floss();
                [[maybe_unused]] float *matrix = mpx.get_matrix();

                if (esp_log_level_get(TAG) >= ESP_LOG_DEBUG) {
                    char log_buf[64];
                    for (uint16_t i = 0; i < recv_count; ++i) {
                        // sprintf(log_buf, "%.1f %.2f %.2f", buffer[i], floss[floss_landmark + i],
                        //                matrix[floss_landmark + i]);
                        sprintf(log_buf, "%.2f,%.2f\n", buffer[i], floss[floss_landmark + i]);
#if defined(SAVE_DATA_TO_FILE)
                        if (file == nullptr) {
                            ESP_LOGE(TAG, "File is not open");
                            vTaskDelete(nullptr);
                        }
                        fputs(log_buf, file);
#endif
                        esp_rom_printf("%s", log_buf); // indexes[floss_landmark + i]);
                    }
                }
                // ESP_LOGD(TAG, "[Consumer] %d", recv_count); // handle about 400 samples per second
                if (recv_count > 30) {
                    delay_adjust -= 5;
                    if (delay_adjust <= 0) {
                        delay_adjust = 1;
                    }
                } else {
                    if (recv_count < 10) {
                        delay_adjust += 5;
                    }
                }
            }
        } else {
            vTaskDelay((portTICK_PERIOD_MS * 2)); // for stability
        }

        vTaskDelay((portTICK_PERIOD_MS * delay_adjust)); // for stability
    }
}

#if defined(USE_AD8232_SENSOR)
/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_bitwidth_t width, adc_atten_t atten,
                                 adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = width,
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
            .unit_id = unit, .atten = atten, .bitwidth = width, .default_vref = 1100};
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

// static void adc_calibration_deinit(adc_cali_handle_t handle)
// {
//     #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
//     ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
//     ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

//     #elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
//     ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
//     ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
//     #endif
// }

/// @brief Task to read the signal from the ecg sensor (12-bit ADC)
/// @param pv_parameters pointer to the task parameters
void task_read_signal(void *pv_parameters) // This is a task.
{
    (void)pv_parameters;

    // const uint8_t no_of_samples = 64; // Multisampling
    const adc_channel_t channel = ADC_CHANNEL_PIN;
    const adc_bitwidth_t adc_width = ADC_BITWITDH;
    const adc_atten_t adc_atten = ADC_ATTENUATION;
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1, .clk_src = ADC_RTC_CLK_SRC_RC_FAST, .ulp_mode = ADC_ULP_MODE_DISABLE};

    TickType_t last_wake_time;

    uint16_t initial_counter = 0;
    float adc_res;
    const uint32_t timer_interval = 1000U / SAMPLING_RATE_HZ; // 4ms = 250Hz

    // short period filter
    const float eps_f = 0.05F;
    const float alpha = powf(eps_f, 1.0F / SHORT_FILTER);
    // large (wander) period filter
    const float l_alpha = powf(eps_f, 1.0F / WANDER_FILTER);

    intptr_t adc_reading;
    intptr_t adc_voltage;
    bool sensor_started = false;

    float adc_sum = 0.0F;
    float adc_num = 0.0F;
    float adc_sum2 = 0.0F;
    float adc_num2 = 0.0F;

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {.atten = adc_atten, .bitwidth = adc_width};

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));

    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 =
        adc_calibration_init(ADC_UNIT_1, channel, adc_width, adc_atten, &adc1_cali_chan0_handle);

    #if defined(DEBUG)
    const uint16_t default_vref = 1100; // Use adc2_vref_to_gpio() to obtain a better estimate
    esp_adc_cal_characteristics_t *adc_chars;
    adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, adc_atten, adc_width, default_vref, adc_chars);

    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGD(TAG, "Characterized using Two Point Value");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGD(TAG, "Characterized using eFuse Vref");
    } else {
        ESP_LOGD(TAG, "Characterized using Default Vref");
    }
    #endif

    gpio_set_direction(POSITIVE_LO_PIN, GPIO_MODE_INPUT); // Setup for leads off detection LO +
    gpio_set_direction(NEGATIVE_LO_PIN, GPIO_MODE_INPUT); // Setup for leads off detection LO -

    ESP_LOGD(TAG, "task_read_signal started");

    vTaskDelay((portTICK_PERIOD_MS * 1000)); // Wait for sensor to stabilize

    last_wake_time = xTaskGetTickCount(); // get the current tick count

    for (;;) // -H776 A Task shall never return or exit.
    {
        vTaskDelayUntil(&last_wake_time, (portTICK_PERIOD_MS * timer_interval)); // for stability

        while (!is_running) {
            // User can pause the session
            // vTaskDelay((portTICK_PERIOD_MS * 200));
            // for now, restar the session to save the files in flash
            esp_restart();
        }

        if ((gpio_get_level(POSITIVE_LO_PIN) == 1) || (gpio_get_level(NEGATIVE_LO_PIN) == 1)) {
            // leads off
            ESP_LOGD(TAG, "!");
            vTaskDelay((portTICK_PERIOD_MS * 50)); // for stability
        } else {
            if (!sensor_started) {
                sensor_started = true;
                initial_counter = 0;

                ESP_LOGD(TAG, "[Producer] Sensor started, now it can be used.");
            }

            // ADC_ATTEN_DB_6  150 mV ~ 1750 mV
            // ADC_ATTEN_DB_11 150 mV ~ 2450 mV
            adc_reading = 0;
            // Multisampling
            // for (int i = 0; i < no_of_samples; i++) {
            // adc_reading += adc1_get_raw(channel);
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &adc_reading));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, channel, adc_reading);
            if (do_calibration1_chan0) {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_reading, &adc_voltage));
                ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, channel, adc_voltage);
            }
            // }
            // adc_reading /= no_of_samples;

            if (adc_reading < 500) {
                // noise?
                vTaskDelay((portTICK_PERIOD_MS * 1)); // for stability
                continue;
            }

    #if defined(DEBUG)
            // Convert adc_reading to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
            ESP_LOGD(TAG, "Raw: %d\tVoltage: %dmV", adc_reading, voltage);
    #endif

            adc_sum = adc_sum * alpha + (float)adc_reading;
            adc_num = adc_num * alpha + 1.0F;
            adc_sum2 = adc_sum2 * l_alpha + (float)adc_reading;
            adc_num2 = adc_num2 * l_alpha + 1.0F;
            adc_res = (adc_sum / adc_num - adc_sum2 / adc_num2); // subtract the mean

            // value in mv scale is
            // Vout = adc_res * 2450mV / 4095 (for ADC_ATTEN_11db) or adc_res * 0.5982906  0.921406F
            // Vout = adc_res * 1750mV / 4095 (for ADC_ATTEN_6db)  or adc_res * 0.4273504  0.485196F

            // value in mV scale
            if (adc_atten == ADC_ATTEN_DB_12) {
                adc_res *= 0.921406F;
            } else if (adc_atten == ADC_ATTEN_DB_6) {
                adc_res *= 0.485196F;
            }

            if (adc_res > 150.0F || adc_res < -150.0F) {
                vTaskDelay((portTICK_PERIOD_MS * 1)); // for stability
                continue;
            }

            const UBaseType_t resbuf = xRingbufferSend(ring_buf, &adc_res, sizeof(adc_res), (portTICK_PERIOD_MS * 100));

            if (resbuf == pdTRUE) {
                if (!buffer_init) {
                    if (++initial_counter >= BUFFER_SIZE) {
                        buffer_init = true; // this is read by the receiver task
                        ESP_LOGD(TAG, "[Producer] DEBUG: Buffer started, starting to compute");
                    }
                }
            } else {
                ESP_LOGD(TAG, "Failed to send item (timeout), %u", initial_counter);
            }
        }
    }
}
#else
/// @brief Task to read the signal from the sensor
/// @param pv_parameters pointer to the task parameters
void task_read_signal(void *pv_parameters) // This is a task.
{
    (void)pv_parameters;

    TickType_t last_wake_time;

    uint16_t initial_counter = 0;
    float ir_res;
    const uint32_t timer_interval = 1000U / SAMPLING_RATE_HZ; // 4ms = 250Hz

    #if !defined(LOAD_DATA_FROM_FILE)

    // I2C
    const gpio_num_t i2c_scl = I2C_MASTER_SCL;
    const gpio_num_t i2c_sda = I2C_MASTER_SDA;

    // Reset pin, MFIO pin
    const gpio_num_t res_pin = I2C_MAX32664_RESET_PIN;
    const gpio_num_t mfio_pin = I2C_MAX32664_MFIO_PIN;
    // Possible widths: 69, 118, 215, 411us
    const uint16_t width = I2C_MAX32664_PULSE_WIDTH;
    // Possible samples: 50, 100, 200, 400, 800, 1000, 1600, 3200 samples/second
    // Not every sample amount is possible with every width; check out our hookup
    // guide for more information.
    const uint16_t samples = I2C_MAX32664_SAMPLE_RATE;

    // short period filter
    const float eps_f = 0.05F;
    const float alpha = powf(eps_f, 1.0F / SHORT_FILTER);
    // large (wander) period filter
    const float l_alpha = powf(eps_f, 1.0F / WANDER_FILTER);

    // Takes address, reset pin, and MFIO pin.
    maxim::Max32664Hub bio_hub(res_pin, mfio_pin);
    maxim::BioData body;

    bool sensor_started = false;

    float ir_sum = 0.0F;
    float ir_num = 0.0F;
    float ir_sum2 = 0.0F;
    float ir_num2 = 0.0F;

    ESP_ERROR_CHECK(bio_hub.i2c_bus_init(i2c_sda, i2c_scl));

    if (bio_hub.begin() == maxim::SUCCESS) { // Zero errors!
        ESP_LOGI(TAG, "Sensor started!");
    } else {
        ESP_LOGE(TAG, "Sensor not started!");
        esp_restart();
    }

    if (bio_hub.config_sensor() == maxim::SUCCESS) {
        ESP_LOGI(TAG, "Sensor configured!");
    } else {
        ESP_LOGE(TAG, "Sensor not configured!");
        esp_restart();
    }

    if (bio_hub.set_pulse_width(width) == maxim::SUCCESS && bio_hub.set_sample_rate(samples) == maxim::SUCCESS) {
        ESP_LOGI(TAG, "Pulse width and sample rate set!");
    } else {
        ESP_LOGE(TAG, "Pulse width and sample rate not set!");
        esp_restart();
    }

    #endif

    ESP_LOGD(TAG, "task_read_signal started");

    vTaskDelay((portTICK_PERIOD_MS * 1000)); // Wait for sensor to stabilize

    last_wake_time = xTaskGetTickCount(); // get the current tick count

    for (;;) // -H776 A Task shall never return or exit.
    {
        vTaskDelayUntil(&last_wake_time, (portTICK_PERIOD_MS * timer_interval)); // for stability

        while (!is_running) {
            // User can pause the session
            // body = bio_hub.read_sensor(); // keep reading the sensor to not overflow the buffer
            // vTaskDelay((portTICK_PERIOD_MS * timer_interval));
            // for now, restar the session to save the files in flash
            esp_restart();
        }

    #if defined(LOAD_DATA_FROM_FILE)
        ir_res = 0.0F;

        if (file != nullptr) {
            char line[20];

            if (fgets(line, sizeof(line), file) == nullptr) {
                ESP_LOGI(TAG, "End of file reached, rewind");
                rewind(file);
                // file = nullptr;
                // All done, unmount partition and disable LittleFS

                // ESP_LOGI(TAG, "LittleFS unmounted");

            } else {
                float v = std::stof(line);
                ir_res = v;
            }
        }
    #else

        // cppcheck-suppress redundantAssignment
        body = bio_hub.read_sensor();
        uint32_t ir_led = body.ir_led;

        if (!sensor_started) {
            if (ir_led == 0) {
                sensor_started = true;
                initial_counter = 0;

                ESP_LOGD(TAG, "[Producer] Sensor started, now it can be used.");
            } else {
                ++initial_counter;
                if (initial_counter > 2500) {
                    ESP_LOGD(TAG, "[Producer] Sensor not properly started, rebooting...");
                    esp_restart();
                }
            }
        }

        if (ir_led > 10000) {
            ir_sum = ir_sum * alpha + (float)ir_led;
            ir_num = ir_num * alpha + 1.0F;
            ir_sum2 = ir_sum2 * l_alpha + (float)ir_led;
            ir_num2 = ir_num2 * l_alpha + 1.0F;
            ir_res = (ir_sum / ir_num - ir_sum2 / ir_num2);
            ir_res /= 20.0F;

    #endif

        if (ir_res > 50.0F || ir_res < -50.0F) {
            continue;
        }

        const UBaseType_t resbuf = xRingbufferSend(ring_buf, &ir_res, sizeof(ir_res), (portTICK_PERIOD_MS * 100));

        if (resbuf == pdTRUE) {
            if (!buffer_init) {
                if (++initial_counter >= BUFFER_SIZE) {
                    buffer_init = true; // this is read by the receiver task
                    ESP_LOGD(TAG, "[Producer] DEBUG: Buffer started, starting to compute");
                }
            }
        } else {
            ESP_LOGD(TAG, "Failed to send item (timeout), %u", initial_counter);
        }
    #if !defined(LOAD_DATA_FROM_FILE)
    }
    else // NOLINT(readability-misleading-indentation)
    {
        // ESP_LOGD(TAG, "[Producer] IR: %d", body.irLed);
        vTaskDelay((portTICK_PERIOD_MS * 1)); // for stability
    }
    #endif
}
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
/// @brief main function
void app_main(void)
{
#if defined(SAVE_DATA_TO_FILE) || defined(LOAD_DATA_FROM_FILE)
    esp_register_shutdown_handler([]() {
        ESP_LOGI(TAG, "Shutdown handler called");
        if (file != nullptr) {
            fclose(file);
            esp_vfs_littlefs_unregister("littlefs");
        }
    });
#endif

    ESP_ERROR_CHECK(touch_pad_init());
    // For interrupt tp we need to set the touch sensor to FSM mode at 'TOUCH_FSM_MODE_TIMER'.
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    touch_pad_config(TOUCH_PAD_GPIO12_CHANNEL, 0); // initialize the RTC IO
    tp_set_thresholds();
    touch_pad_isr_register(tp_rtc_intr, nullptr);
    touch_pad_intr_enable();

    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
    esp_log_level_set("mpx", ESP_LOG_ERROR);

    // ESP_LOGD(TAG, "Heap: %u", esp_get_free_heap_size());
    // ESP_LOGD(TAG, "Max alloc Heap: %u", esp_get_minimum_free_heap_size());
    ESP_LOGD(TAG, "SDK version: %s", esp_get_idf_version());

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGD(TAG, "ESP32%s rev %d, %d CPU cores, WiFi%s%s%s, ", (chip_info.model & CHIP_ESP32S2) ? "-S2" : "",
             chip_info.revision, chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
             (chip_info.features & CHIP_FEATURE_IEEE802154) ? "/802.15.4" : "");

    // ESP_LOGD(TAG, "%uMB of %s flash", spi_flash_get_chip_size() / (uint32_t)(1024 * 1024),
    //          (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // I (228) psram: This chip is ESP32-D0WD
    // I (228) spiram: Found 64MBit SPI RAM device
    // I (228) spiram: SPI RAM mode: flash 40m sram 40m
    // I (233) spiram: PSRAM initialized, cache is in low/high (2-core) mode.

    //     if (esp_spiram_is_initialized()) {
    //         ESP_LOGD(TAG, "%uMB of %s PSRAM", esp_spiram_get_size() / (uint32_t)(1024 * 1024),
    //                  (chip_info.features & CHIP_FEATURE_EMB_PSRAM) ? "embedded" : "external");
    //     } else {
    //         ESP_LOGD(TAG, "No PSRAM found");
    //     }

    // Initialize NVS.
    ring_buf = xRingbufferCreateNoSplit(sizeof(float), RING_BUFFER_SIZE);

    if (ring_buf == nullptr) {
        ESP_LOGE(TAG, "[Setup] Error creating the ring_buf. Aborting.");
        esp_restart();
    }

#if defined(SAVE_DATA_TO_FILE) || defined(LOAD_DATA_FROM_FILE)
    ESP_LOGI(TAG, "Initializing LittleFS");

    esp_vfs_littlefs_conf_t const conf = {.base_path = "/littlefs",
                                          .partition_label = "littlefs",
                                          .partition = NULL,
                                          .format_if_mount_failed = true,
                                          .read_only = false,
                                          .dont_mount = false,
                                          .grow_on_mount = true};

    // Use settings defined above to initialize and mount LittleFS filesystem.
    // Note: esp_vfs_littlefs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        esp_restart();
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read LittleFS partition information (%s)", esp_err_to_name(ret));
        esp_restart();
    }

    ESP_LOGI(TAG, "LittleFS: %u / %u", used, total);

    if (esp_littlefs_mounted("littlefs")) {
        ESP_LOGI(TAG, "LittleFS mounted");
    } else {
        ESP_LOGE(TAG, "LittleFS not mounted");
    }

#endif

    ESP_LOGI(TAG, "System initialized, waiting for user input (touch)");

    while (!is_running) {
        vTaskDelay((portTICK_PERIOD_MS * 500));
        ESP_LOGI(TAG, "main: paused");
    }

#if defined(SAVE_DATA_TO_FILE) || defined(LOAD_DATA_FROM_FILE)

    // #if defined(LOAD_DATA_FROM_FILE)
    // file = fopen("/littlefs/recorded.csv", "r");

    // if (file != nullptr) {
    //     ESP_LOGD(TAG, "File exists");

    //     char line[20];

    //     ESP_LOGD(TAG, "Reading file");
    //     while (fgets(line, sizeof(line), file) != nullptr) {
    //         ESP_LOGD(TAG, "Line: %s", line);
    //         vTaskDelay((portTICK_PERIOD_MS * 200));
    //     }
    //     ESP_LOGD(TAG, "Done Reading file");

    //     fclose(file);
    //     file = nullptr;
    // }
    // #elif defined(SAVE_DATA_TO_FILE)

    char filename[32];
    char idx[8];

    // look up for the next file name
    for (uint8_t i = 1; i <= LOG_ROTATION_SIZE; i++) {
        sprintf(filename, "/littlefs/record%02u.csv", i);
        file = fopen(filename, "r");
        // file does not exist
        if (file == nullptr) {
            ESP_LOGI(TAG, "Opening for write file %02u", i);
            file = fopen("/littlefs/last.txt", "w");
            sprintf(idx, "%02u", i);
            fputs(idx, file);
            fclose(file);
            file = fopen(filename, "w");
            break;
        }
        // file exists, try the next one
        fclose(file);
        file = nullptr;
    }

    if (file == nullptr) {
        ESP_LOGD(TAG, "All files exist.");

        file = fopen("/littlefs/last.txt", "r");
        fgets(idx, sizeof(idx), file);
        uint8_t i = (uint8_t)atoi(idx); // NOLINT (clang-tidy/cert-err34-c)

        ++i;
        if (i > LOG_ROTATION_SIZE)
            i = 1;

        sprintf(filename, "/littlefs/record%02u.csv", i);
        ESP_LOGI(TAG, "Opening for write file %02u", i);
        file = fopen("/littlefs/last.txt", "w");
        sprintf(idx, "%02u", i);
        fputs(idx, file);
        fclose(file);
        file = fopen(filename, "w");
    }

    if (file == nullptr) {
        esp_vfs_littlefs_unregister(conf.partition_label);
        ESP_LOGE(TAG, "Failed to open file for write");
        esp_restart();
    }

#endif

    ESP_LOGD(TAG, "Creating task 0");

    TaskHandle_t read_task_handle = nullptr;
    TaskHandle_t compute_task_handle = nullptr;
    BaseType_t res;

    // Producer Task
    res = xTaskCreatePinnedToCore(
        task_read_signal,     // Task function
        "ReadSignal",         // Just a name
        20000,                // Stack size in bytes
        nullptr,              // Parameter passed as input of the task (can be NULL)
        tskIDLE_PRIORITY + 3, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        &read_task_handle,    // Task Handle (can be NULL)
        CORE_0);

    configASSERT(read_task_handle);

    if (res == pdPASS) {
        ESP_LOGD(TAG, "Task ReadSignal created");
    } else {
        ESP_LOGE(TAG, "Failed to create task ReadSignal");
    }

    ESP_LOGD(TAG, "Creating task 1");

    // Consumer Task
    res = xTaskCreatePinnedToCore(
        task_compute,         // Task function
        "Compute",            // Just a name
        20000,                // This stack size in bytes can be checked & adjusted by reading the Stack Highwater
        nullptr,              // Parameter passed as input of the task (can be NULL)
        tskIDLE_PRIORITY + 3, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        &compute_task_handle, // Task Handle (can be NULL)
        CORE_1);

    configASSERT(compute_task_handle);

    if (res == pdPASS) {
        ESP_LOGD(TAG, "Task Compute created");
    } else {
        ESP_LOGE(TAG, "Failed to create task Compute");
    }

    ESP_LOGD(TAG, "Main entering in infinite loop");
    // Main loop
    while (true) {
#if defined(LOG_MEM_LOAD)
        if (esp_log_level_get(TAG) == ESP_LOG_VERBOSE) {
            ESP_LOGV(TAG, "Task Compute StackHighWaterMark: %u", uxTaskGetStackHighWaterMark(compute_task_handle));
            ESP_LOGV(TAG, "Task Read    StackHighWaterMark: %u", uxTaskGetStackHighWaterMark(read_task_handle));
            ESP_LOGV(TAG, "Minimum  8bits-aligned Free Heap: %u", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
            ESP_LOGV(TAG, "Minimum 32bits-aligned Free Heap: %u", heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT));
            ESP_LOGV(TAG, "Current  8bits-aligned Free Heap: %u/%u", heap_caps_get_free_size(MALLOC_CAP_8BIT),
                     heap_caps_get_total_size(MALLOC_CAP_8BIT));
            ESP_LOGV(TAG, "Current 32bits-aligned Free Heap: %u/%u", heap_caps_get_free_size(MALLOC_CAP_32BIT),
                     heap_caps_get_total_size(MALLOC_CAP_32BIT));
            // xRingbufferPrintInfo(ring_buf);
        }
#endif

#if defined(LOG_CPU_LOAD)
        if (esp_log_level_get(TAG) == ESP_LOG_VERBOSE) {
            char taskbuffer[100];
            vTaskGetRunTimeStats(taskbuffer);
            ESP_LOGV(TAG, "\n%s", taskbuffer);
        }
#endif

        // pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc(arraysize * sizeof(TaskStatus_t));
        // vPortFree(pxTaskStatusArray);

        vTaskDelay((portTICK_PERIOD_MS * 5000));
    }
}

#ifdef __cplusplus
}
#endif
