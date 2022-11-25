// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "mpx.hpp"

namespace mp {

static const char TAG[] = "mpx";

Mpx::Mpx(const uint16_t window_size, float ez, uint16_t time_constraint, const uint16_t buffer_size)
    : _window_size(window_size), _ez(ez), _buffer_size(buffer_size), _profile_len(buffer_size - _window_size + 1U),
      _range(_profile_len - 1U), _exclusion_zone((uint16_t)(lroundf((float)_window_size * _ez + __FLT_EPSILON__) + 1U)),
      _buffer_start((int16_t)buffer_size), _data_buffer((float *)pvPortCalloc(_buffer_size + 1U, sizeof(float))),
      _vmatrix_profile((float *)pvPortCalloc(_profile_len + 1U, sizeof(float))),
      _vprofile_index((int16_t *)pvPortCalloc(_profile_len + 1U, sizeof(int16_t))),
      _floss((float *)pvPortCalloc(_profile_len + 1U, sizeof(float))),
      _iac((float *)pvPortCalloc(_profile_len + 1U, sizeof(float))),
      _vmmu((float *)calloc(_profile_len + 1U, sizeof(float))),
      _vsig((float *)pvPortCalloc(_profile_len + 1U, sizeof(float))),
      _vddf((float *)pvPortCalloc(_profile_len + 1U, sizeof(float))),
      _vddg((float *)pvPortCalloc(_profile_len + 1U, sizeof(float))),
      _vww((float *)pvPortCalloc(_window_size + 1U, sizeof(float)))
{
    // change the default value to 0

    if (_vmatrix_profile != nullptr && _vprofile_index != nullptr) {
        for (uint16_t i = 0U; i < _profile_len; ++i) {
            _vmatrix_profile[i] = -1000000.0F;
            _vprofile_index[i] = -1;
        }
    }

    this->_floss_iac();   // computes the empirical IAC using random data
    this->prune_buffer(); // prunes the buffer filling it with a random walk
}

Mpx::~Mpx()
{
    // free arrays
    vPortFree(this->_vww);
    vPortFree(this->_vddg);
    vPortFree(this->_vddf);
    vPortFree(this->_vsig);
    vPortFree(this->_vmmu);
    vPortFree(this->_iac);
    vPortFree(this->_vprofile_index);
    vPortFree(this->_vmatrix_profile);
    vPortFree(this->_data_buffer);
    vPortFree(this->_floss);
}

// cppcheck-suppress unusedFunction
uint16_t Mpx::compute(const float *data, uint16_t size)
{
    bool const first = _new_data(data, size); // store new data on buffer

    // If the buffer is not pruned, the new data may be the first one, then we need to precompute the
    // entire helping data structures. Or we just update them with the new data.
    if (first) {
        _muinvn(0U);
        _ddf(0U);
        _ddg(0U);
    } else {
        _muinvn(size);  // compute next mean and sig
        _ddf(size);     // compute next ddf
        _ddg(size);     // compute next ddg
        _mp_next(size); // shift MP
    }

    _ww_s(); // compute normalized window

    // if (_time_constraint > 0) {
    //   diag_start = _buffer_size - _time_constraint - _window_size;
    // }

    uint16_t const diag_start = this->_buffer_start;
    uint16_t const diag_end = this->_profile_len - this->_exclusion_zone;

    for (uint16_t i = diag_start; i < diag_end; ++i) {
        // this mess is just the inner_product but _data_buffer needs to be minus _vmmu[i] before multiply
        float c = 0.0F;

        // inner product demeaned
        for (uint16_t j = 0U; j < _window_size; ++j) {
            c += (_data_buffer[i + j] - _vmmu[i]) * _vww[j];
        }

        uint16_t off_min = 0U;

        if (first) {
            off_min = _range - i - 1;
        } else {
            // cppcheck-suppress duplicateExpression
            off_min = MAX(_range - size, _range - i - 1); // -V501
        }

        uint16_t const off_start = _range;

        for (uint16_t offset = off_start; offset > off_min; offset--) {
            // min is offset + diag; max is (profile_len - 1); each iteration has the size of off_max
            uint16_t const off_diag = offset - (_range - i);

            c += _vddf[offset] * _vddg[off_diag] + _vddf[off_diag] * _vddg[offset];

            if ((_vsig[offset] < 0.0F) || (_vsig[off_diag] < 0.0F)) { // wild sig, misleading
                continue;
            }

            float const c_cmp = c * _vsig[offset] * _vsig[off_diag];

            // RMP
            if (c_cmp > _vmatrix_profile[off_diag]) {
                _vmatrix_profile[off_diag] = c_cmp;
                _vprofile_index[off_diag] = (int16_t)(offset); // May be added by 1 if index starts on 1
            }
        }
    }

    return (this->_buffer_size - this->_buffer_used);
}

void Mpx::prune_buffer()
{
    // prune buffer with a random walk
    _data_buffer[0] = 0.001F;

    for (uint16_t i = 1U; i < _buffer_size; ++i) {
        float mock = (float)((RAND() % 1000) - 500);
        mock /= 1000.0F;
        _data_buffer[i] = _data_buffer[i - 1] + mock;
    }
    _buffer_used = _buffer_size;
    _buffer_start = 0;
    _muinvn(0U);
    _ddf(0U);
    _ddg(0U);
}

// cppcheck-suppress unusedFunction
void Mpx::floss()
{
    // Computes the FLOSS using the empirical IAC

    // first zero the array
    for (uint16_t i = 0U; i < this->_profile_len; ++i) {
        this->_floss[i] = 0.0F;
    }

    // compute the FLOSS from the 0 to the end of the profile but the exclusion zone
    for (uint16_t i = 0U; i < (this->_profile_len - this->_exclusion_zone - 1); ++i) {
        int16_t const j = _vprofile_index[i];

        // it is not supposed to point beyond the profile
        if (j >= this->_profile_len) {
            LOG_DEBUG(TAG, "DEBUG: j >= this->profile_len_");
            continue;
        }

        // if the index -1, means that the profile is not computed
        if (j < 0) {
            if (j < -1) {
                LOG_DEBUG(TAG, "DEBUG: j < -1");
            }
            continue;
        }

        // in RMP j is always greater than i
        if (j < i) {
            LOG_DEBUG(TAG, "DEBUG: i = %d ; j = %d ", i, j);
            continue;
        }
        // RMP, i is always < j
        this->_floss[i] += 1.0F;
        this->_floss[j] -= 1.0F;
    }

    // const float a = 1.939274;
    // const float b = 1.69815;
    // const float c = 4.035477;
    ////  const float len = (float)this->_profile_len;
    ////  const float x = 1.0F / len;
    ////  const float llen = len * 1.1494F;
    ////  float iac = 0.001F; // cppcheck-suppress unreadVariable

    // cumsum
    for (uint16_t i = 0U; i < this->_range; ++i) {
        this->_floss[i + 1U] += this->_floss[i];
        if (i < this->_window_size || i > (this->_profile_len - this->_window_size)) {
            this->_floss[i] = 1.0F;
        } else {
            // iac = a * b * powf(i * x, a - 1.0) * powf(1.0 - powf(i * x, a), b - 1.0) * len / c;
            // iac = 0.816057 * len * powf(i * x, 0.939274) * powf(1 - powf(i * x, 1.93927), 0.69815);
            // iac = 0.8245 * powf(i * x, 0.94) * powf(1.0 - powf(i * x, 1.94), 0.7) * len;
            //     // const float idx = (float)i * x;
            //     // iac = powf(idx, 1.08F) * powf(1.0F - idx, 0.64F) * llen; // faster
            // iac = a * b * powf(idx, (a - 1)) * powf(1 - powf(idx, a), (b - 1)) * len / 4.035477;
            if (this->_floss[i] > this->_iac[i]) {
                this->_floss[i] = 1.0F;
            } else {
                this->_floss[i] /= this->_iac[i];
            }
        }
    }

    // x <- seq(0, 1, length.out = cac_size)
    //  mode <- 0.6311142 # best point to analyze the segment change
    //  iac <- a * b * x^(a - 1) * (1 - x^a)^(b - 1) * cac_size / 4.035477
}

/// @brief computes the empirical IAC using random data
void Mpx::_floss_iac()
{
    uint16_t *mpi = nullptr;

    mpi = ((uint16_t *)pvPortCalloc(this->_profile_len + 1U, sizeof(uint16_t)));

    if (mpi == nullptr) {
        LOG_DEBUG(TAG, "Memory allocation failed");
        return;
    }

    for (uint16_t i = 0U; i < this->_profile_len; ++i) {
        this->_iac[i] = 0.0F;
    }

    for (uint16_t k = 0U; k < 10; ++k) { // repeat 10 times to smooth the result
        for (uint16_t i = 0U; i < (this->_profile_len - this->_exclusion_zone - 1); ++i) {
            mpi[i] = (RAND() % (this->_range - (i + this->_exclusion_zone))) + (i + this->_exclusion_zone);
        }

        for (uint16_t i = 0U; i < (this->_profile_len - this->_exclusion_zone - 1); ++i) {
            uint16_t const j = mpi[i];

            if (j >= this->_profile_len) {
                LOG_DEBUG(TAG, "j >= this->profile_len_");
                continue;
            }
            // RMP, i is always < j
            this->_iac[i] += 0.1F;
            this->_iac[j] -= 0.1F;
        }
    }
    // cumsum
    for (uint16_t i = 0U; i < this->_range; ++i) {
        this->_iac[i + 1U] += this->_iac[i];
    }

    vPortFree(mpi);
}

/// @brief Shifts the MP and MPI buffers to the left
/// @param size amount of samples to shift
void Mpx::_mp_next(uint16_t size)
{
    uint16_t const j = this->_profile_len - size;

    // update 1 step
    for (uint16_t i = 0; i < j; ++i) {
        _vmatrix_profile[i] = _vmatrix_profile[i + size];

        _vprofile_index[i] = (int16_t)(_vprofile_index[i + size] - size); // the index must be reduced

        // avoid too negative values
        if (_vprofile_index[i] < -1) {
            _vprofile_index[i] = -1;
        }
    }

    for (uint16_t i = j; i < _profile_len; ++i) {
        _vmatrix_profile[i] = -1000000.0F;
        _vprofile_index[i] = -1;
    }
}

/// @brief Shifts the ddf buffer to the left
/// @param size amount of samples to shift
void Mpx::_ddf(uint16_t size)
{
    // differentials have 0 as their first entry. This simplifies index
    // calculations slightly and allows us to avoid special "first line"
    // handling.

    uint16_t start = _buffer_start;

    if (size > 0U) {
        // shift data
        for (uint16_t i = _buffer_start; i < (_range - size); ++i) {
            this->_vddf[i] = this->_vddf[i + size];
        }

        start = (_range - size);
    }

    for (uint16_t i = start; i < _range; ++i) {
        this->_vddf[i] = 0.5F * (this->_data_buffer[i] - this->_data_buffer[i + this->_window_size]);
    }

    // DEBUG: this should already be zero
    this->_vddf[_range] = 0.0F;
}

/// @brief Shifts the ddg buffer to the left
/// @param size amount of samples to shift
void Mpx::_ddg(uint16_t size)
{
    // ddg: (data[(w+1):data_len] - mov_avg[2:(data_len - w + 1)]) + (data[1:(data_len - w)] - mov_avg[1:(data_len -
    // w)]) (subtract the mov_mean of all data, but the first window) + (subtract the mov_mean of all data, but the last
    // window)

    uint16_t start = _buffer_start;

    if (size > 0U) {
        // shift data
        for (uint16_t i = _buffer_start; i < (_range - size); ++i) {
            this->_vddg[i] = this->_vddg[i + size];
        }

        start = (_range - size);
    }

    for (uint16_t i = start; i < _range; ++i) {
        this->_vddg[i] = (this->_data_buffer[i + this->_window_size] - this->_vmmu[i + 1U]) +
                         (this->_data_buffer[i] - this->_vmmu[i]);
    }

    this->_vddg[_range] = 0.0F;
}

/// @brief Computes the newest window, demeaned
void Mpx::_ww_s()
{
    for (uint16_t i = 0U; i < _window_size; ++i) {
        this->_vww[i] = (this->_data_buffer[_range + i] - this->_vmmu[_range]);
    }
}

/// @brief Computes the moving mean of the data buffer using the muinvn algorithm
///        which compensates for floating point errors.
void Mpx::_movmean()
{
    float accum = this->_data_buffer[_buffer_start];
    float resid = 0.0F;
    float movsum;

    for (uint16_t i = 1U; i < this->_window_size; ++i) {
        float const m = this->_data_buffer[_buffer_start + i];
        float const p = accum;
        accum = accum + m;
        float const q = accum - p;
        resid = resid + ((p - (accum - q)) + (m - q));
    }

    if (resid > 0.001F) {
        LOG_DEBUG(TAG, "movsum: Residual value is large. Some precision may be lost. res = %.2f", resid);
    }

    movsum = accum + resid;
    this->_vmmu[_buffer_start] = (float)(movsum / (float)this->_window_size);

    for (uint16_t i = (this->_window_size + _buffer_start); i < this->_buffer_size; ++i) {
        float const m = this->_data_buffer[i - this->_window_size];
        float const n = this->_data_buffer[i];
        float const p = accum - m;
        float const q = p - accum;
        resid = resid + ((accum - (p - q)) - (m + q));

        accum = p + n;
        float const t = accum - p;
        resid = resid + ((p - (accum - t)) + (n - t));

        movsum = accum + resid;
        this->_vmmu[i - this->_window_size + 1U] = (float)(movsum / (float)this->_window_size);
    }

    this->_last_accum = accum;
    this->_last_resid = resid;
}

/// @brief Computes the moving sig of the data buffer using the muinvn algorithm
///        which compensates for floating point errors.
void Mpx::_movsig()
{
    float accum = this->_data_buffer[_buffer_start] * this->_data_buffer[_buffer_start];
    float resid = 0.0F;
    float mov2sum;

    for (uint16_t i = 1U; i < this->_window_size; ++i) {
        float const m = this->_data_buffer[_buffer_start + i] * this->_data_buffer[_buffer_start + i];
        float const p = accum;
        accum = accum + m;
        float const q = accum - p;
        resid = resid + ((p - (accum - q)) + (m - q));
    }

    if (resid > 0.001F) {
        LOG_DEBUG(TAG, "mov2sum: Residual value is large. Some precision may be lost. res = %.2f", resid);
    }

    mov2sum = accum + resid;
    float const psig = mov2sum - this->_vmmu[_buffer_start] * this->_vmmu[_buffer_start] * (float)this->_window_size;

    // For sd > 1.19e-7; window 25 -> sig will be <= 1.68e+6 (psig >= 3.54e-13) and for window 350 -> sig will be
    // <= 4.5e+5 (psig >= 4.94e-12) For sd < 100; window 25 -> sig will be >= 0.002 (psig <= 25e4) and for window 350 ->
    // sig will be >= 0.0005 (psig <= 4e6)

    if (psig > __FLT_EPSILON__ && psig < 4000000.0F) {
        this->_vsig[_buffer_start] = 1.0F / sqrtf(psig);
    } else {
        LOG_DEBUG(TAG, "DEBUG: psig1 precision, %.3f", psig);
        this->_vsig[_buffer_start] = -1.0F;
    }

    for (uint16_t i = (this->_window_size + _buffer_start); i < this->_buffer_size; ++i) {
        float const m = this->_data_buffer[i - this->_window_size] * this->_data_buffer[i - this->_window_size];
        float const n = this->_data_buffer[i] * this->_data_buffer[i];
        float const p = accum - m;
        float const q = p - accum;
        resid = resid + ((accum - (p - q)) - (m + q));
        accum = p + n;
        float const t = accum - p;
        resid = resid + ((p - (accum - t)) + (n - t));
        mov2sum = accum + resid;
        float const ppsig = mov2sum - this->_vmmu[i - this->_window_size + 1U] *
                                          this->_vmmu[i - this->_window_size + 1U] * (float)this->_window_size;

        if (ppsig > __FLT_EPSILON__ && ppsig < 4000000.0F) {
            this->_vsig[i - this->_window_size + 1U] = 1.0F / sqrtf(ppsig);
        } else {
            LOG_DEBUG(TAG, "DEBUG: ppsig precision, %.3f", ppsig);
            this->_vsig[i - this->_window_size + 1U] = -1.0F;
        }
    }

    this->_last_accum2 = accum;
    this->_last_resid2 = resid;
}

/// @brief Updates the moving mean and sig of the data buffer using the muinvn algorithm
///        which compensates for floating point errors.
/// @param size the size of the new data
void Mpx::_muinvn(uint16_t size)
{
    if (size == 0U) {
        _movmean();
        _movsig();
        return;
    }

    uint16_t const j = this->_profile_len - size;

    // update 1 step
    for (uint16_t i = 0U; i < j; ++i) {
        _vmmu[i] = _vmmu[i + size];
        _vsig[i] = _vsig[i + size];
    }

    // compute new mmu sig
    float accum = this->_last_accum;   // NOLINT(misc-const-correctness) - this variable can't be const
    float accum2 = this->_last_accum2; // NOLINT(misc-const-correctness) - this variable can't be const
    float resid = this->_last_resid;   // NOLINT(misc-const-correctness) - this variable can't be const
    float resid2 = this->_last_resid2; // NOLINT(misc-const-correctness) - this variable can't be const

    for (uint16_t i = j; i < _profile_len; ++i) {
        /* mean */
        float m = _data_buffer[i - 1];
        float n = _data_buffer[i - 1 + _window_size];
        float p = accum - m;
        float q = p - accum;
        resid = resid + ((accum - (p - q)) - (m + q));

        accum = p + n;
        float t = accum - p;
        resid = resid + ((p - (accum - t)) + (n - t));
        _vmmu[i] = (float)((accum + resid) / (float)_window_size);

        /* sig */
        m = _data_buffer[i - 1] * _data_buffer[i - 1];
        n = _data_buffer[i - 1 + _window_size] * _data_buffer[i - 1 + _window_size];
        p = accum2 - m;
        q = p - accum2;
        resid2 = resid2 + ((accum2 - (p - q)) - (m + q));

        accum2 = p + n;
        t = accum2 - p;
        resid2 = resid2 + ((p - (accum2 - t)) + (n - t));

        float const psig = (accum2 + resid2) - _vmmu[i] * _vmmu[i] * (float)_window_size;
        if (psig > __FLT_EPSILON__ && psig < 4000000.0F) {
            _vsig[i] = 1.0F / sqrtf(psig);
        } else {
            LOG_DEBUG(TAG, "DEBUG: psig precision, %.3f", psig);
            _vsig[i] = -1.0F;
        }
    }

    this->_last_accum = accum;
    this->_last_accum2 = accum2;
    this->_last_resid = resid;
    this->_last_resid2 = resid2;
}

/// @brief Shifts the data buffer to the left by the given amount and fills with new data
/// @param data The new data to fill the buffer with
/// @param size The size of the new data
/// @return Returns true if it is the first time the buffer is filled
bool Mpx::_new_data(const float *data, uint16_t size)
{
    bool first = true;

    if ((2U * size) > _buffer_size) {
        LOG_DEBUG(TAG, "Data size is too large");
        return false;
    } else if (size < (_window_size) && _buffer_used < _window_size) {
        LOG_DEBUG(TAG, "Data size is too small");
        return false;
    } else {
        if ((_buffer_start != _buffer_size) || _buffer_used > 0U) {
            first = false;
            // we must shift data
            for (uint16_t i = 0U; i < (_buffer_size - size); ++i) {
                this->_data_buffer[i] = this->_data_buffer[size + i];
            }
            // then copy
            for (uint16_t i = 0U; i < size; ++i) {
                this->_data_buffer[(_buffer_size - size + i)] = data[i];
            }
        } else {
            // fresh start, buffer must be already filled with zeroes
            for (uint16_t i = 0U; i < size; ++i) {
                this->_data_buffer[(_buffer_size - size + i)] = data[i];
            }
        }

        _buffer_used += size;
        _buffer_start = (int16_t)(_buffer_start - size);

        if (_buffer_used > _buffer_size) {
            _buffer_used = _buffer_size;
        }

        if (_buffer_start < 0) {
            _buffer_start = 0;
        }
    }

    return first;
}

} // namespace mp
