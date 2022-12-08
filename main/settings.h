/**
 * @file settings.h
 * @author Francisco Bischoff (franzbischoff@gmail.com)
 * @brief Settings for the Matrix Profile library
 * @version 0.1
 * @date 2022-11-07
 *
 * @copyright CC BY-NC-SA 4.0 2022
 *
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

// #define USE_AD8232_SENSOR /*!< Use the AD8232 sensor */
// #define FILE_DATA
// #define LOAD_DATA_FROM_FILE /*!< Load data from file */
#define SAVE_DATA_TO_FILE /*!< Save data to file */

// #define LOG_CPU_LOAD /*!< Log CPU load */
// #define LOG_MEM_LOAD /*!< Log memory load */

#define SAMPLING_RATE_HZ 250.0F /*!< Sampling rate of the reader task */
#define WINDOW_SIZE 100U        /*!< Size of the sliding window */
#define HISTORY_SIZE_S 20U      /*!< Size of the history buffer in seconds */
#define FLOSS_LANDMARK_S 10U    /*!< Size of the FLOSS landmark in seconds */

#if defined(USE_AD8232_SENSOR)
    #define POSITIVE_LO_PIN GPIO_NUM_13
    #define NEGATIVE_LO_PIN GPIO_NUM_12
    #define ADC_CHANNEL_PIN ADC1_CHANNEL_5 // GPIO34 if ADC1
    #define ADC_ATTENUATION ADC_ATTEN_11db // 0-3.7V ADC_ATTEN_6b 0-1.98V
#endif

// #define DEBUG

#endif // SETTINGS_H_
