// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "mpx.hpp"

static const char TAG[] = "mpx";

namespace MatrixProfile {
Mpx::Mpx(const uint16_t window_size, float ez, uint16_t time_constraint, const uint16_t buffer_size)
    : window_size_(window_size), ez_(ez), buffer_size_(buffer_size), buffer_start_((int16_t)buffer_size),
      profile_len_(buffer_size - window_size_ + 1U), range_(profile_len_ - 1U),
      exclusion_zone_((uint16_t)(lroundf((float)window_size_ * ez_ + __FLT_EPSILON__) + 1U)), // -V2004
      data_buffer_((float *)pvPortCalloc(buffer_size_ + 1U, sizeof(float))),
      vmatrix_profile_((float *)pvPortCalloc(profile_len_ + 1U, sizeof(float))),
      vprofile_index_((int16_t *)pvPortCalloc(profile_len_ + 1U, sizeof(int16_t))),
      floss_((float *)pvPortCalloc(profile_len_ + 1U, sizeof(float))),
      iac_((float *)pvPortCalloc(profile_len_ + 1U, sizeof(float))),
      vmmu_((float *)calloc(profile_len_ + 1U, sizeof(float))),
      vsig_((float *)pvPortCalloc(profile_len_ + 1U, sizeof(float))),
      vddf_((float *)pvPortCalloc(profile_len_ + 1U, sizeof(float))),
      vddg_((float *)pvPortCalloc(profile_len_ + 1U, sizeof(float))),
      vww_((float *)pvPortCalloc(window_size_ + 1U, sizeof(float)))
{
    // change the default value to 0

    if (vmatrix_profile_ != nullptr && vprofile_index_ != nullptr) {
        for (uint16_t i = 0U; i < profile_len_; ++i) {
            vmatrix_profile_[i] = -1000000.0F;
            vprofile_index_[i] = -1;
        }
    }

    this->floss_iac_();
    this->prune_buffer();
}

void Mpx::movmean_()
{
    float accum = this->data_buffer_[buffer_start_];
    float resid = 0.0F;
    float movsum;

    for (uint16_t i = 1U; i < this->window_size_; ++i) {
        float const m = this->data_buffer_[buffer_start_ + i];
        float const p = accum;
        accum = accum + m;
        float const q = accum - p;
        resid = resid + ((p - (accum - q)) + (m - q));
    }

    if (resid > 0.001F) {
        LOG_DEBUG(TAG, "movsum: Residual value is large. Some precision may be lost. res = %.2f", resid);
    }

    movsum = accum + resid;
    this->vmmu_[buffer_start_] = (float)(movsum / (float)this->window_size_);

    for (uint16_t i = (this->window_size_ + buffer_start_); i < this->buffer_size_; ++i) {
        float const m = this->data_buffer_[i - this->window_size_];
        float const n = this->data_buffer_[i];
        float const p = accum - m;
        float const q = p - accum;
        resid = resid + ((accum - (p - q)) - (m + q));

        accum = p + n;
        float const t = accum - p;
        resid = resid + ((p - (accum - t)) + (n - t));

        movsum = accum + resid;
        this->vmmu_[i - this->window_size_ + 1U] = (float)(movsum / (float)this->window_size_);
    }

    this->last_accum_ = accum;
    this->last_resid_ = resid;
}

void Mpx::movsig_()
{
    float accum = this->data_buffer_[buffer_start_] * this->data_buffer_[buffer_start_];
    float resid = 0.0F;
    float mov2sum;

    for (uint16_t i = 1U; i < this->window_size_; ++i) {
        float const m = this->data_buffer_[buffer_start_ + i] * this->data_buffer_[buffer_start_ + i];
        float const p = accum;
        accum = accum + m;
        float const q = accum - p;
        resid = resid + ((p - (accum - q)) + (m - q));
    }

    if (resid > 0.001F) {
        LOG_DEBUG(TAG, "mov2sum: Residual value is large. Some precision may be lost. res = %.2f", resid);
    }

    mov2sum = accum + resid;
    float const psig = mov2sum - this->vmmu_[buffer_start_] * this->vmmu_[buffer_start_] * (float)this->window_size_;

    // For sd > 1.19e-7; window 25 -> sig will be <= 1.68e+6 (psig >= 3.54e-13) and for window 350 -> sig will be
    // <= 4.5e+5 (psig >= 4.94e-12) For sd < 100; window 25 -> sig will be >= 0.002 (psig <= 25e4) and for window 350 ->
    // sig will be >= 0.0005 (psig <= 4e6)

    if (psig > __FLT_EPSILON__ && psig < 4000000.0F) {
        this->vsig_[buffer_start_] = 1.0F / sqrtf(psig);
    } else {
        LOG_DEBUG(TAG, "DEBUG: psig1 precision, %.3f", psig);
        this->vsig_[buffer_start_] = -1.0F;
    }

    for (uint16_t i = (this->window_size_ + buffer_start_); i < this->buffer_size_; ++i) {
        float const m = this->data_buffer_[i - this->window_size_] * this->data_buffer_[i - this->window_size_];
        float const n = this->data_buffer_[i] * this->data_buffer_[i];
        float const p = accum - m;
        float const q = p - accum;
        resid = resid + ((accum - (p - q)) - (m + q));
        accum = p + n;
        float const t = accum - p;
        resid = resid + ((p - (accum - t)) + (n - t));
        mov2sum = accum + resid;
        float const ppsig = mov2sum - this->vmmu_[i - this->window_size_ + 1U] *
                                          this->vmmu_[i - this->window_size_ + 1U] * (float)this->window_size_;

        if (ppsig > __FLT_EPSILON__ && ppsig < 4000000.0F) {
            this->vsig_[i - this->window_size_ + 1U] = 1.0F / sqrtf(ppsig);
        } else {
            LOG_DEBUG(TAG, "DEBUG: ppsig precision, %.3f", ppsig);
            this->vsig_[i - this->window_size_ + 1U] = -1.0F;
        }
    }

    this->last_accum2_ = accum;
    this->last_resid2_ = resid;
}

void Mpx::muinvn_(uint16_t size)
{
    if (size == 0U) {
        movmean_();
        movsig_();
        return;
    }

    uint16_t const j = this->profile_len_ - size;

    // update 1 step
    for (uint16_t i = 0U; i < j; ++i) {
        vmmu_[i] = vmmu_[i + size];
        vsig_[i] = vsig_[i + size];
    }

    // compute new mmu sig
    float accum = this->last_accum_;   // NOLINT(misc-const-correctness) - this variable can't be const
    float accum2 = this->last_accum2_; // NOLINT(misc-const-correctness) - this variable can't be const
    float resid = this->last_resid_;   // NOLINT(misc-const-correctness) - this variable can't be const
    float resid2 = this->last_resid2_; // NOLINT(misc-const-correctness) - this variable can't be const

    for (uint16_t i = j; i < profile_len_; ++i) {
        /* mean */
        float m = data_buffer_[i - 1];
        float n = data_buffer_[i - 1 + window_size_];
        float p = accum - m;
        float q = p - accum;
        resid = resid + ((accum - (p - q)) - (m + q));

        accum = p + n;
        float t = accum - p;
        resid = resid + ((p - (accum - t)) + (n - t));
        vmmu_[i] = (float)((accum + resid) / (float)window_size_);

        /* sig */
        m = data_buffer_[i - 1] * data_buffer_[i - 1];
        n = data_buffer_[i - 1 + window_size_] * data_buffer_[i - 1 + window_size_];
        p = accum2 - m;
        q = p - accum2;
        resid2 = resid2 + ((accum2 - (p - q)) - (m + q));

        accum2 = p + n;
        t = accum2 - p;
        resid2 = resid2 + ((p - (accum2 - t)) + (n - t));

        float const psig = (accum2 + resid2) - vmmu_[i] * vmmu_[i] * (float)window_size_;
        if (psig > __FLT_EPSILON__ && psig < 4000000.0F) {
            vsig_[i] = 1.0F / sqrtf(psig);
        } else {
            LOG_DEBUG(TAG, "DEBUG: psig precision, %.3f", psig);
            vsig_[i] = -1.0F;
        }
    }

    this->last_accum_ = accum;
    this->last_accum2_ = accum2;
    this->last_resid_ = resid;
    this->last_resid2_ = resid2;
}

bool Mpx::new_data_(const float *data, uint16_t size)
{
    bool first = true;

    if ((2U * size) > buffer_size_) {
        LOG_DEBUG(TAG, "Data size is too large");
        return false;
    } else if (size < (window_size_) && buffer_used_ < window_size_) {
        LOG_DEBUG(TAG, "Data size is too small");
        return false;
    } else {
        if ((buffer_start_ != buffer_size_) || buffer_used_ > 0U) {
            first = false;
            // we must shift data
            for (uint16_t i = 0U; i < (buffer_size_ - size); ++i) {
                this->data_buffer_[i] = this->data_buffer_[size + i];
            }
            // then copy
            for (uint16_t i = 0U; i < size; ++i) {
                this->data_buffer_[(buffer_size_ - size + i)] = data[i];
            }
        } else {
            // fresh start, buffer must be already filled with zeroes
            for (uint16_t i = 0U; i < size; ++i) {
                this->data_buffer_[(buffer_size_ - size + i)] = data[i];
            }
        }

        buffer_used_ += size;
        buffer_start_ = (int16_t)(buffer_start_ - size);

        if (buffer_used_ > buffer_size_) {
            buffer_used_ = buffer_size_;
        }

        if (buffer_start_ < 0) {
            buffer_start_ = 0;
        }
    }

    return first;
}

void Mpx::mp_next_(uint16_t size)
{
    uint16_t const j = this->profile_len_ - size;

    // update 1 step
    for (uint16_t i = 0; i < j; ++i) {
        vmatrix_profile_[i] = vmatrix_profile_[i + size];

        vprofile_index_[i] = (int16_t)(vprofile_index_[i + size] - size); // the index must be reduced

        // avoid too negative values
        if (vprofile_index_[i] < -1) {
            vprofile_index_[i] = -1;
        }
    }

    for (uint16_t i = j; i < profile_len_; ++i) {
        vmatrix_profile_[i] = -1000000.0F;
        vprofile_index_[i] = -1;
    }
}

void Mpx::ddf_(uint16_t size)
{
    // differentials have 0 as their first entry. This simplifies index
    // calculations slightly and allows us to avoid special "first line"
    // handling.

    uint16_t start = buffer_start_;

    if (size > 0U) {
        // shift data
        for (uint16_t i = buffer_start_; i < (range_ - size); ++i) {
            this->vddf_[i] = this->vddf_[i + size];
        }

        start = (range_ - size);
    }

    for (uint16_t i = start; i < range_; ++i) {
        this->vddf_[i] = 0.5F * (this->data_buffer_[i] - this->data_buffer_[i + this->window_size_]);
    }

    // DEBUG: this should already be zero
    this->vddf_[range_] = 0.0F;
}

void Mpx::ddg_(uint16_t size)
{
    // ddg: (data[(w+1):data_len] - mov_avg[2:(data_len - w + 1)]) + (data[1:(data_len - w)] - mov_avg[1:(data_len -
    // w)]) (subtract the mov_mean of all data, but the first window) + (subtract the mov_mean of all data, but the last
    // window)

    uint16_t start = buffer_start_;

    if (size > 0U) {
        // shift data
        for (uint16_t i = buffer_start_; i < (range_ - size); ++i) {
            this->vddg_[i] = this->vddg_[i + size];
        }

        start = (range_ - size);
    }

    for (uint16_t i = start; i < range_; ++i) {
        this->vddg_[i] = (this->data_buffer_[i + this->window_size_] - this->vmmu_[i + 1U]) +
                         (this->data_buffer_[i] - this->vmmu_[i]);
    }

    this->vddg_[range_] = 0.0F;
}

void Mpx::ww_s_()
{
    for (uint16_t i = 0U; i < window_size_; ++i) {
        this->vww_[i] = (this->data_buffer_[range_ + i] - this->vmmu_[range_]);
    }
}

void Mpx::prune_buffer()
{
    // prune buffer
    data_buffer_[0] = 0.001F;

    for (uint16_t i = 1U; i < buffer_size_; ++i) {
        float mock = (float)((RAND() % 1000) - 500);
        mock /= 1000.0F;
        data_buffer_[i] = data_buffer_[i - 1] + mock;
    }
    buffer_used_ = buffer_size_;
    buffer_start_ = 0;
    muinvn_(0U);
    ddf_(0U);
    ddg_(0U);
}

void Mpx::floss_iac_()
{
    uint16_t *mpi = nullptr;

    mpi = ((uint16_t *)pvPortCalloc(this->profile_len_ + 1U, sizeof(uint16_t)));

    if (mpi == nullptr) {
        LOG_DEBUG(TAG, "Memory allocation failed");
        return;
    }

    for (uint16_t i = 0U; i < this->profile_len_; ++i) {
        this->iac_[i] = 0.0F;
    }

    for (uint16_t k = 0U; k < 10; ++k) { // repeat 10 times to smooth the result
        for (uint16_t i = 0U; i < (this->profile_len_ - this->exclusion_zone_ - 1); ++i) {
            mpi[i] = (RAND() % (this->range_ - (i + this->exclusion_zone_))) + (i + this->exclusion_zone_);
        }

        for (uint16_t i = 0U; i < (this->profile_len_ - this->exclusion_zone_ - 1); ++i) {
            uint16_t const j = mpi[i];

            if (j >= this->profile_len_) {
                LOG_DEBUG(TAG, "j >= this->profile_len_");
                continue;
            }
            // RMP, i is always < j
            this->iac_[i] += 0.1F;
            this->iac_[j] -= 0.1F;
        }
    }
    // cumsum
    for (uint16_t i = 0U; i < this->range_; ++i) {
        this->iac_[i + 1U] += this->iac_[i];
    }

    vPortFree(mpi); // test for edit session
}

// cppcheck-suppress unusedFunction
void Mpx::floss()
{
    for (uint16_t i = 0U; i < this->profile_len_; ++i) {
        this->floss_[i] = 0.0F;
    }

    for (uint16_t i = 0U; i < (this->profile_len_ - this->exclusion_zone_ - 1); ++i) {
        int16_t const j = vprofile_index_[i];

        if (j >= this->profile_len_) {
            LOG_DEBUG(TAG, "DEBUG: j >= this->profile_len_");
            continue;
        }

        if (j < 0) {
            if (j < -1) {
                LOG_DEBUG(TAG, "DEBUG: j < -1");
            }
            // LOG_DEBUG(TAG, "DEBUG: j < 0");
            // j = (rand() % (this->range_ - (i + this->exclusion_zone_))) + (i + this->exclusion_zone_);
            // vprofile_index_[i] = j;
            continue;
        }

        if (j < i) {
            LOG_DEBUG(TAG, "DEBUG: i = %d ; j = %d ", i, j);
        }
        // RMP, i is always < j
        this->floss_[i] += 1.0F;
        this->floss_[j] -= 1.0F;
    }

    // const float a = 1.939274;
    // const float b = 1.69815;
    // const float c = 4.035477;
    ////  const float len = (float)this->profile_len_;
    ////  const float x = 1.0F / len;
    ////  const float llen = len * 1.1494F;
    ////  float iac = 0.001F; // cppcheck-suppress unreadVariable

    // cumsum
    for (uint16_t i = 0U; i < this->range_; ++i) {
        this->floss_[i + 1U] += this->floss_[i];
        if (i < this->window_size_ || i > (this->profile_len_ - this->window_size_)) {
            this->floss_[i] = 1.0F;
        } else {
            // iac = a * b * powf(i * x, a - 1.0) * powf(1.0 - powf(i * x, a), b - 1.0) * len / c;
            // iac = 0.816057 * len * powf(i * x, 0.939274) * powf(1 - powf(i * x, 1.93927), 0.69815);
            // iac = 0.8245 * powf(i * x, 0.94) * powf(1.0 - powf(i * x, 1.94), 0.7) * len;
            //     // const float idx = (float)i * x;
            //     // iac = powf(idx, 1.08F) * powf(1.0F - idx, 0.64F) * llen; // faster
            // iac = a * b * powf(idx, (a - 1)) * powf(1 - powf(idx, a), (b - 1)) * len / 4.035477;
            if (this->floss_[i] > this->iac_[i]) {
                this->floss_[i] = 1.0F;
            } else {
                this->floss_[i] /= this->iac_[i];
            }
        }
    }

    // x <- seq(0, 1, length.out = cac_size)
    //  mode <- 0.6311142 # best point to analyze the segment change
    //  iac <- a * b * x^(a - 1) * (1 - x^a)^(b - 1) * cac_size / 4.035477
}

// cppcheck-suppress unusedFunction
uint16_t Mpx::compute(const float *data, uint16_t size)
{
    bool const first = new_data_(data, size); // store new data on buffer

    if (first) {
        muinvn_(0U);
        ddf_(0U);
        ddg_(0U);
    } else {
        muinvn_(size);  // compute next mean and sig
        ddf_(size);     // compute next ddf
        ddg_(size);     // compute next ddg
        mp_next_(size); // shift MP
    }

    ww_s_();

    // if (time_constraint_ > 0) {
    //   diag_start = buffer_size_ - time_constraint_ - window_size_;
    // }

    uint16_t const diag_start = buffer_start_;
    uint16_t const diag_end = this->profile_len_ - this->exclusion_zone_;

    uint32_t debug_wild_sig = 0U;

    for (uint16_t i = diag_start; i < diag_end; ++i) {
        // this mess is just the inner_product but data_buffer_ needs to be minus vmmu_[i] before multiply

        float c = 0.0F;

        // inner product demeaned
        for (uint16_t j = 0U; j < window_size_; ++j) {
            c += (data_buffer_[i + j] - vmmu_[i]) * vww_[j];
        }

        uint16_t off_min = 0U;

        if (first) {
            off_min = range_ - i - 1;
        } else {
            // cppcheck-suppress duplicateExpression
            off_min = MAX(range_ - size, range_ - i - 1); // -V501
        }

        uint16_t const off_start = range_;

        for (uint16_t offset = off_start; offset > off_min; offset--) {
            // min is offset + diag; max is (profile_len - 1); each iteration has the size of off_max
            uint16_t const off_diag = offset - (range_ - i);
            // 3586 = i, offset = 4900

            c += vddf_[offset] * vddg_[off_diag] + vddf_[off_diag] * vddg_[offset];

            if ((vsig_[offset] < 0.0F) || (vsig_[off_diag] < 0.0F)) { // wild sig, misleading
                ++debug_wild_sig;
                continue;
            }

            float const c_cmp = c * vsig_[offset] * vsig_[off_diag];

            // RMP
            // min off_diag is 0; max off_diag is (diag_end-1) == (profile_len_ - exclusion_zone_ - 1)
            if (c_cmp > vmatrix_profile_[off_diag]) {
                // LOG_DEBUG(TAG, "%f", c_cmp);
                vmatrix_profile_[off_diag] = c_cmp;
                vprofile_index_[off_diag] = (int16_t)(offset); // + 1U);
            }
        }
    }

    if (debug_wild_sig > 0U) {
        ;
        // LOG_DEBUG(TAG, "DEBUG: wild sig: %u", debug_wild_sig);
    }

    return (this->buffer_size_ - this->buffer_used_);
}

Mpx::~Mpx()
{
    // free arrays
    vPortFree(this->vww_);
    vPortFree(this->vddg_);
    vPortFree(this->vddf_);
    vPortFree(this->vsig_);
    vPortFree(this->vmmu_);
    vPortFree(this->iac_);
    vPortFree(this->vprofile_index_);
    vPortFree(this->vmatrix_profile_);
    vPortFree(this->data_buffer_);
    vPortFree(this->floss_);
}
} // namespace MatrixProfile
