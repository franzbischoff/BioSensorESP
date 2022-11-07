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
    #include <esp_random.h>
    #include <esp_log.h>
    #include <freertos/FreeRTOS.h>
    // #include <esp_dsp.h>

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
// uint16_t = 0 to 65535
// int16_t = -32768 to +32767

namespace MatrixProfile {
/**
 * @brief
 */
class Mpx {
public:
    // cppcheck-suppress noExplicitConstructor
    Mpx(uint16_t window_size, float ez = 0.5F, uint16_t time_constraint = 0U, uint16_t buffer_size = 5000U);
    ~Mpx(); // destructor

    /**
     *
     * @brief Computes the matrix profile using the incoming data
     * @param data an array of float values
     * @param size size of the array
     * @return returns the size left in the buffer. If the buffer is full (after using prune_buffer), it returns 0.
     */
    uint16_t compute(const float *data, uint16_t size);
    void prune_buffer();
    void floss();

    // Getters
    float *get_data_buffer() { return data_buffer_; };

    float *get_matrix() { return vmatrix_profile_; };

    int16_t *get_indexes() { return vprofile_index_; };

    float *get_floss() { return floss_; };

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
    // const uint16_t time_constraint_;
    const uint16_t buffer_size_;
    uint16_t buffer_used_ = 0U;
    int16_t buffer_start_ = 0;

    uint16_t profile_len_;
    uint16_t range_; // profile lenght - 1

    uint16_t exclusion_zone_;

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
} // namespace MatrixProfile
#endif // MPX_MPX_HPP_
