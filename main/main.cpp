// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "settings.h"
#include "mpx/mpx.hpp"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_littlefs.h>
#include <esp_spi_flash.h>
#include <esp32/spiram.h>
#include <cmath>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>
#include <max32664.hpp>

#define SAMPLING_HZ SAMPLING_RATE_HZ
#define HIST_SIZE (uint16_t)(HISTORY_SIZE_S * SAMPLING_HZ)
#define FLOSS_LANDMARK (HIST_SIZE - (SAMPLING_HZ * FLOSS_LANDMARK_S))
#define BUFFER_SIZE (WINDOW_SIZE)
#define RING_BUFFER_SIZE (BUFFER_SIZE + 20U) // queue must have a little more room

#ifndef SHORT_FILTER
    #define SHORT_FILTER ((float)(SAMPLING_HZ) / 10.0F)
#endif
#ifndef WANDER_FILTER
    #define WANDER_FILTER ((float)(SAMPLING_HZ) / 2.0F)
#endif

// ESP_PLATFORM

//>> multitask configurations
[[noreturn]] void task_compute(void *pv_parameters);
[[noreturn]] void task_read_signal(void *pv_parameters);

enum CORES { CORE_0 = 0x00, CORE_1 = 0x01 };

//>> log tag
static const char TAG[] = "main";
#ifndef MAIN_LOG_LEVEL
    #define MAIN_LOG_LEVEL ESP_LOG_INFO
#endif

//>> global variables
static volatile bool buffer_init = false; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static volatile RingbufHandle_t ring_buf; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

#if defined(FILE_DATA)
static volatile FILE *file; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
#endif

///   @brief Task to compute the matrix profile
/// @param pv_parameters pointer to the task parameters
[[noreturn]] void task_compute(void *pv_parameters) // This is a task.
{
    (void)pv_parameters;

    [[maybe_unused]] const uint16_t floss_landmark = FLOSS_LANDMARK;
    float buffer[BUFFER_SIZE];
    uint16_t recv_count;
    int16_t delay_adjust = 100;

    for (uint16_t i = 0; i < BUFFER_SIZE; ++i) { // NOLINT(modernize-loop-convert)
        buffer[i] = 0.0F;
    }

    MatrixProfile::Mpx mpx(WINDOW_SIZE, 0.5F, 0U, HIST_SIZE);
    mpx.prune_buffer();

    for (;;) // -H776 A Task shall never return or exit.
    {
        if (buffer_init) { // wait for buffer to be initialized
            recv_count = 0;

            for (uint16_t i = 0; i < BUFFER_SIZE; ++i) {
                size_t item_size; // size in bytes (usually 4 bytes)
                float *data = static_cast<float *>(xRingbufferReceive(ring_buf, &item_size, 0));

                if (item_size > 4) {
                    ESP_LOGD(TAG, "Item_size: %u", item_size);
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

                // if (esp_log_level_get(TAG) >= ESP_LOG_DEBUG) {
                //   char log_buf[64];
                //   for (uint16_t i = 0; i < recv_count; ++i) {
                //     // sprintf(log_buf, "%.1f %.2f %.2f", buffer[i], floss[floss_landmark + i],
                //     //                matrix[floss_landmark + i]);
                //     sprintf(log_buf, "%.1f %.2f", buffer[i], matrix[floss_landmark + i]); // NOLINT(cert-err33-c)
                //     esp_rom_printf("%s\n", log_buf);                                      // indexes[floss_landmark +
                //     i]);
                //   }
                // }
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

/// @brief Task to read the signal from the sensor
/// @param pv_parameters pointer to the task parameters
[[noreturn]] void task_read_signal(void *pv_parameters) // This is a task.
{
    (void)pv_parameters;

    TickType_t last_wake_time;

    uint16_t initial_counter = 0;
    float ir_res;
    const uint32_t timer_interval = 1000U / SAMPLING_HZ; // 4ms = 250Hz

#ifndef FILE_DATA

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

    uint32_t ir_led;
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

#ifndef FILE_DATA
        body = bio_hub.read_sensor();
        ir_led = body.ir_led;

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
#else

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
#ifndef FILE_DATA
        } else {
            // ESP_LOGD(TAG, "[Producer] IR: %d", body.irLed);
            vTaskDelay((portTICK_PERIOD_MS * 1)); // for stability
        }
#endif
    }
}

#ifdef __cplusplus
extern "C" {
#endif
/// @brief main function
[[noreturn]] void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
    esp_log_level_set("mpx", ESP_LOG_ERROR);

    ESP_LOGD(TAG, "Heap: %u", esp_get_free_heap_size());
    ESP_LOGD(TAG, "Max alloc Heap: %u", esp_get_minimum_free_heap_size());
    ESP_LOGD(TAG, "SDK version: %s", esp_get_idf_version());

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGD(TAG, "ESP32%s rev %d, %d CPU cores, WiFi%s%s%s, ", (chip_info.model & CHIP_ESP32S2) ? "-S2" : "",
             chip_info.revision, chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
             (chip_info.features & CHIP_FEATURE_IEEE802154) ? "/802.15.4" : "");

    ESP_LOGD(TAG, "%uMB of %s flash", spi_flash_get_chip_size() / (uint32_t)(1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // I (228) psram: This chip is ESP32-D0WD
    // I (228) spiram: Found 64MBit SPI RAM device
    // I (228) spiram: SPI RAM mode: flash 40m sram 40m
    // I (233) spiram: PSRAM initialized, cache is in low/high (2-core) mode.

#ifdef CONFIG_ESP32_SPIRAM_SUPPORT
    if (esp_spiram_is_initialized()) {
        ESP_LOGD(TAG, "%uMB of %s PSRAM", esp_spiram_get_size() / (uint32_t)(1024 * 1024),
                 (chip_info.features & CHIP_FEATURE_EMB_PSRAM) ? "embedded" : "external");
    } else {
        ESP_LOGD(TAG, "No PSRAM found");
    }
#endif

    // Initialize NVS.
    ring_buf = xRingbufferCreateNoSplit(sizeof(float), RING_BUFFER_SIZE);

    if (ring_buf == nullptr) {
        ESP_LOGE(TAG, "[Setup] Error creating the ring_buf. Aborting.");
        esp_restart();
    }

#if defined(FILE_DATA)
    ESP_LOGI(TAG, "Initializing LittleFS");

    esp_vfs_littlefs_conf_t const conf = {
        .base_path = "/littlefs",
        .partition_label = "littlefs",
        .format_if_mount_failed = false,
        .dont_mount = false,
    };

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
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);

    ESP_LOGI(TAG, "LittleFS: %u / %u", used, total);

    if (esp_littlefs_mounted("littlefs")) {
        ESP_LOGI(TAG, "LittleFS mounted");
    } else {
        ESP_LOGE(TAG, "LittleFS not mounted");
    }

    file = fopen("/littlefs/floss.csv", "r");

    if (file == nullptr) {
        esp_vfs_littlefs_unregister(conf.partition_label);
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
#endif

    ESP_LOGD(TAG, "Creating task 0");

    TaskHandle_t read_task_handle = nullptr;
    TaskHandle_t compute_task_handle = nullptr;
    BaseType_t res;

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
