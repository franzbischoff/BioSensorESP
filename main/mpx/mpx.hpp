/**
 * @file mpx.hpp
 * @author Francisco Bischoff (franzbischoff@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-11-07
 *
 * @copyright CC BY-NC-SA 4.0 2022
 *
 */

#ifndef MPX_MPX_HPP_
#define MPX_MPX_HPP_

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <memory>
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"

namespace mp {

#ifdef ESP_PLATFORM
    #if defined(configUSE_FREERTOS_PROVIDED_HEAP)
        #define pvPortCalloc(X, Y) pvPortMalloc(X *Y)
    #else
        #define pvPortCalloc calloc
    #endif

    #define RAND() static_cast<int32_t>(esp_random())
    #define LOG_DEBUG(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
#else
    #define pvPortCalloc calloc
    #define pvPortMalloc malloc
    #define vPortFree free
    #define RAND() rand()
    #define LOG_DEBUG(tag, format, ...) printf(format, ##__VA_ARGS__)
#endif

#if !defined(NULL)
    #define NULL 0
#endif

#if !defined(MIN)
    #define MIN(y, x) ((x) < (y) && (x) == (x) ? (x) : (y))
#endif
#if !defined(MAX)
    #define MAX(y, x) ((x) > (y) && (x) == (x) ? (x) : (y))
#endif

/// @brief This class contains the whole process of receiving streaming data, computing
///         the Matrix Profile and the FLOSS. The get_* functions returns a pointer to
///         the respective data. The data is stored in a "circular buffer", so the pointer
///         will always point to the oldest data, being the most recent at the end of the
///         buffer. The size of the buffer is normally defined by HISTORY_SIZE_S * SAMPLING_HZ macros.

class Mpx {
public:
    // cppcheck-suppress noExplicitConstructor

    /// @brief Constructor
    /// @param window_size required. Size of the sliding window
    /// @param ez optional. Exclusion zone parameter. Default is 0.5F (half of the window size).
    /// @param time_constraint not used yet. Time constraint parameter. Default is 0U, no time constraint.
    /// @param buffer_size optional. Size of the buffer. Default is 5000 (20 seconds at 250Hz).
    Mpx(uint16_t window_size, float ez = 0.5F, uint16_t time_constraint = 0U, uint16_t buffer_size = 5000U);

    /// @brief Destructor
    ~Mpx();

    /// @brief Computes the matrix profile using the incoming data
    /// @param data an array of float values
    /// @param size size of the array
    /// @return returns the size left in the buffer. If the buffer is full (after using prune_buffer), it returns 0.
    uint16_t compute(const float *data, uint16_t size);

    /// @brief Prunes the buffer filling it with a random walk
    void prune_buffer();

    /// @brief Computes the FLOSS of the current MP
    void floss();

    /// @brief Returns the data buffer
    /// @return returns a pointer to the data buffer
    float *get_data_buffer() { return _data_buffer; };

    /// @brief Returns the computed streaming MP
    /// @return returns a pointer to the MP buffer
    float *get_matrix() { return _vmatrix_profile; };

    /// @brief Returns the computed streaming MP indices
    /// @return returns a pointer to the MP indices buffer
    int16_t *get_indexes() { return _vprofile_index; };

    /// @brief Returns the computed FLOSS
    /// @return returns a pointer to the FLOSS buffer
    float *get_floss() { return _floss; };

    /// @brief Returns the computed ideal arc counts
    /// @return returns a pointer to the ideal arc counts buffer
    float *get_iac() { return _iac; };

    float *get_vmmu() { return _vmmu; };

    float *get_vsig() { return _vsig; };

    float *get_ddf() { return _vddf; };

    float *get_ddg() { return _vddg; };

    float *get_vww() { return _vww; };

    uint16_t get_buffer_used() { return _buffer_used; };

    int16_t get_buffer_start() { return _buffer_start; };

    uint16_t get_profile_len() { return _profile_len; };

    float get_last_movsum() { return _last_accum + _last_resid; };

    float get_last_mov2sum() { return _last_accum2 + _last_resid2; };

private:
    bool _new_data(const float *data, uint16_t size);
    void _floss_iac();
    void _movmean();
    void _movsig();
    void _muinvn(uint16_t size = 0U);
    void _mp_next(uint16_t size = 0U);
    void _ddf(uint16_t size = 0U);
    void _ddg(uint16_t size = 0U);
    void _ww_s();

    const uint16_t _window_size;
    const float _ez;
    const uint16_t _buffer_size;
    const uint16_t _profile_len; /// A public variable.
    const uint16_t _range;       /// same as profile_length - 1
    const uint16_t _exclusion_zone;

    uint16_t _buffer_used = 0U;
    int16_t _buffer_start = 0;

    float _last_accum = 0.0F;
    float _last_resid = 0.0F;
    float _last_accum2 = 0.0F;
    float _last_resid2 = 0.0F;

    // arrays
    float *_data_buffer = nullptr;
    float *_vmatrix_profile = nullptr;
    int16_t *_vprofile_index = nullptr;
    float *_floss = nullptr;
    float *_iac = nullptr;
    float *_vmmu = nullptr;
    float *_vsig = nullptr;
    float *_vddf = nullptr;
    float *_vddg = nullptr;
    float *_vww = nullptr;
};
} // namespace mp
#endif // MPX_MPX_HPP_
