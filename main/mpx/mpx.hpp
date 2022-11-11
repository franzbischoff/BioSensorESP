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

#ifdef ESP_PLATFORM
    #include <cmath>
    #include <cstdio>
    #include <cstdlib>
    #include <memory>
    #include "esp_log.h"
    #include "esp_random.h"
    #include "freertos/FreeRTOS.h"
#endif

namespace mp {

#ifdef ESP_PLATFORM
    #if defined(configUSE_FREERTOS_PROVIDED_HEAP)
        #define pvPortCalloc(X, Y) pvPortMalloc(X *Y)
    #else
        #define pvPortCalloc calloc
    #endif

    #define RAND() (int32_t)(esp_random())
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
    float *get_data_buffer() { return data_buffer_; };

    /// @brief Returns the computed streaming MP
    /// @return returns a pointer to the MP buffer
    float *get_matrix() { return vmatrix_profile_; };

    /// @brief Returns the computed streaming MP indices
    /// @return returns a pointer to the MP indices buffer
    int16_t *get_indexes() { return vprofile_index_; };

    /// @brief Returns the computed FLOSS
    /// @return returns a pointer to the FLOSS buffer
    float *get_floss() { return floss_; };

    /// @brief Returns the computed ideal arc counts
    /// @return returns a pointer to the ideal arc counts buffer
    float *get_iac() { return iac_; };

    float *get_vmmu() { return vmmu_; };

    float *get_vsig() { return vsig_; };

    float *get_ddf() { return vddf_; };

    float *get_ddg() { return vddg_; };

    float *get_vww() { return vww_; };

    uint16_t get_buffer_used() { return buffer_used_; };

    int16_t get_buffer_start() { return buffer_start_; };

    uint16_t get_profile_len() { return profile_len_; };

    float get_last_movsum() { return last_accum_ + last_resid_; };

    float get_last_mov2sum() { return last_accum2_ + last_resid2_; };

private:
    bool new_data_(const float *data, uint16_t size);
    void floss_iac_();
    void movmean_();
    void movsig_();
    void muinvn_(uint16_t size = 0U);
    void mp_next_(uint16_t size = 0U);
    void ddf_(uint16_t size = 0U);
    void ddg_(uint16_t size = 0U);
    void ww_s_();

    const uint16_t window_size_;
    const float ez_;
    const uint16_t buffer_size_;
    const uint16_t profile_len_; /// A public variable.
    const uint16_t range_;       /// same as profile_length - 1
    const uint16_t exclusion_zone_;

    uint16_t buffer_used_ = 0U;
    int16_t buffer_start_ = 0;

    float last_accum_ = 0.0F;
    float last_resid_ = 0.0F;
    float last_accum2_ = 0.0F;
    float last_resid2_ = 0.0F;

    // arrays
    float *data_buffer_ = nullptr;
    float *vmatrix_profile_ = nullptr;
    int16_t *vprofile_index_ = nullptr;
    float *floss_ = nullptr;
    float *iac_ = nullptr;
    float *vmmu_ = nullptr;
    float *vsig_ = nullptr;
    float *vddf_ = nullptr;
    float *vddg_ = nullptr;
    float *vww_ = nullptr;
};
} // namespace mp
#endif // MPX_MPX_HPP_
