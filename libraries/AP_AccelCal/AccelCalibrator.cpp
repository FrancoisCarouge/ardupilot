/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Authored by Jonathan Challinger, 3D Robotics Inc.

#include "AccelCalibrator.h"
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/typed_linear_algebra.h>

#include <cmath>
#include <tuple>
#include <utility>

namespace {

namespace tla = typed_linear_algebra;

constexpr auto metre_per_second_squared = mp_units::si::metre / (mp_units::si::second * mp_units::si::second);
using Acceleration = tla::Quantity<metre_per_second_squared>;
using Ratio = tla::Quantity<mp_units::one>;

// The parameters: three offsets [m/s/s], three diagonal scale factors [1], and
// for the ellipsoid fit three off-diagonal ones [1]; the residual's Jacobian
// with respect to them: per offset [1], per scale factor [m/s/s].
template <size_t Count> struct gauss_newton;

template <> struct gauss_newton<6> {
    using Parameters = tla::Vector<Acceleration, Acceleration, Acceleration, Ratio, Ratio, Ratio>;
    using Jacobian = tla::Vector<Ratio, Ratio, Ratio, Acceleration, Acceleration, Acceleration>;
};

template <> struct gauss_newton<9> {
    using Parameters = tla::Vector<Acceleration, Acceleration, Acceleration, Ratio, Ratio, Ratio, Ratio, Ratio, Ratio>;
    using Jacobian = tla::Vector<Ratio, Ratio, Ratio, Acceleration, Acceleration, Acceleration,
                                 Acceleration, Acceleration, Acceleration>;
};

// The Jacobian of the residual, gravity less the length of the corrected
// sample, with respect to the parameters: its first Count elements.
template <size_t Count>
typename gauss_newton<Count>::Jacobian residual_jacobian(const Vector3f &sample, const AccelCalibrator::param_t &params)
{
    const Vector3f &offset = params.offset;
    const Vector3f &diag = params.diag;
    const Vector3f &offdiag = params.offdiag;
    Matrix3f M(
        diag.x    , offdiag.x , offdiag.y,
        offdiag.x , diag.y    , offdiag.z,
        offdiag.y , offdiag.z , diag.z
    );

    float A =  (diag.x    * (sample.x + offset.x)) + (offdiag.x * (sample.y + offset.y)) + (offdiag.y * (sample.z + offset.z));
    float B =  (offdiag.x * (sample.x + offset.x)) + (diag.y    * (sample.y + offset.y)) + (offdiag.z * (sample.z + offset.z));
    float C =  (offdiag.y * (sample.x + offset.x)) + (offdiag.z * (sample.y + offset.y)) + (diag.z    * (sample.z + offset.z));
    float length = (M*(sample+offset)).length();

    typename gauss_newton<Count>::Jacobian ret;
    // 0-2: offsets
    ret.template at<0>(-1.0f * (((diag.x    * A) + (offdiag.x * B) + (offdiag.y * C))/length) * mp_units::one);
    ret.template at<1>(-1.0f * (((offdiag.x * A) + (diag.y    * B) + (offdiag.z * C))/length) * mp_units::one);
    ret.template at<2>(-1.0f * (((offdiag.y * A) + (offdiag.z * B) + (diag.z    * C))/length) * mp_units::one);
    // 3-5: diagonals
    ret.template at<3>(-1.0f * ((sample.x + offset.x) * A)/length * metre_per_second_squared);
    ret.template at<4>(-1.0f * ((sample.y + offset.y) * B)/length * metre_per_second_squared);
    ret.template at<5>(-1.0f * ((sample.z + offset.z) * C)/length * metre_per_second_squared);
    if constexpr (Count == 9) {
        // 6-8: off-diagonals
        ret.template at<6>(-1.0f * (((sample.y + offset.y) * A) + ((sample.x + offset.x) * B))/length * metre_per_second_squared);
        ret.template at<7>(-1.0f * (((sample.z + offset.z) * A) + ((sample.x + offset.x) * C))/length * metre_per_second_squared);
        ret.template at<8>(-1.0f * (((sample.z + offset.z) * B) + ((sample.y + offset.y) * C))/length * metre_per_second_squared);
    }
    return ret;
}

// The normal equations JᵀJ and JᵀF of the first Count parameters,
// accumulated over the samples element by element, without temporaries:
// the embedded stack frame limit.
template <size_t Count, typename Normal, typename Gradient, typename SampleAt, typename ResidualOf>
NOINLINE void accumulate_normal_equations(Normal &JTJ, Gradient &JTFI, const AccelCalibrator::param_t &params,
                                          uint16_t sample_count, const SampleAt &sample_at,
                                          const ResidualOf &residual_of)
{
    for (uint16_t k = 0; k < sample_count; k++) {
        const Vector3f sample = sample_at(k);
        const auto jacob = residual_jacobian<Count>(sample, params);
        const Acceleration residual{residual_of(sample, params) * metre_per_second_squared};
        [&]<size_t... Index>(std::index_sequence<Index...>) {
            ((JTJ.template at<Index / Count, Index % Count>(
                  JTJ.template at<Index / Count, Index % Count>() +
                  jacob.template at<Index / Count>() * jacob.template at<Index % Count>())), ...);
        }(std::make_index_sequence<Count * Count>{});
        [&]<size_t... Index>(std::index_sequence<Index...>) {
            ((JTFI.template at<Index>(JTFI.template at<Index>() + jacob.template at<Index>() * residual)), ...);
        }(std::make_index_sequence<Count>{});
    }
}

// The Gauss-Newton step Δ of JᵀJ Δ = JᵀF, solved by elimination with partial
// pivoting, as numerical values in the parameters' units.
template <size_t Count, typename Normal, typename Gradient>
NOINLINE void solve_step(const Normal &JTJ, const Gradient &JTFI, float (&values)[Count])
{
    using Parameters = typename gauss_newton<Count>::Parameters;
    // ((JᵀJ)⁻¹ JᵀF)ᵀ = (JᵀF)ᵀ (JᵀJ)⁻¹, JᵀJ symmetric
    const Parameters step{fcarouge::transposed(fcarouge::transposed(JTFI) / JTJ)};
    [&]<size_t... Index>(std::index_sequence<Index...>) {
        ((values[Index] = step.template at<Index>().numerical_value_in(
              std::remove_cvref_t<decltype(step.template at<Index>())>::unit)), ...);
    }(std::make_index_sequence<Count>{});
}

// One Gauss-Newton iteration on the first Count parameters: the parameters
// moved by -Δ. False if JᵀJ is singular.
template <size_t Count, typename SampleAt, typename ResidualOf>
bool gauss_newton_step(AccelCalibrator::param_t &params, uint16_t sample_count,
                       const SampleAt &sample_at, const ResidualOf &residual_of)
{
    using Jacobian = typename gauss_newton<Count>::Jacobian;
    using Normal = tla::Matrix<Jacobian, Jacobian>;
    using Gradient = decltype(Jacobian{} * Acceleration{});

    Normal JTJ{};
    Gradient JTFI{};
    accumulate_normal_equations<Count>(JTJ, JTFI, params, sample_count, sample_at, residual_of);

    float values[Count];
    solve_step<Count>(JTJ, JTFI, values);
    for (size_t i = 0; i < Count; i++) {
        if (!std::isfinite(values[i])) {
            return false;
        }
    }

    for (size_t i = 0; i < Count; i++) {
        Vector3f &group = i < 3 ? params.offset : (i < 6 ? params.diag : params.offdiag);
        group[i % 3] -= values[i];
    }
    return true;
}

} // namespace


const extern AP_HAL::HAL& hal;
/*
 * TODO
 * - time out when not receiving samples
 */

////////////////////////////////////////////////////////////
///////////////////// PUBLIC INTERFACE /////////////////////
////////////////////////////////////////////////////////////

AccelCalibrator::AccelCalibrator() :
_conf_tolerance(ACCEL_CAL_TOLERANCE),
_sample_buffer(nullptr)
{
    clear();
}
/*
    Select options, initialise variables and initiate accel calibration
    Options:
    Fit Type:       Will assume that if accelerometer static samples around all possible orientation
                    are spread in space will cover a surface of AXIS_ALIGNED_ELLIPSOID or any general 
                    ELLIPSOID as chosen

    Num Samples:    Number of samples user should will be gathering, please note that with type of surface
                    chosen the minimum number of samples required will vary, for instance in the case of AXIS
                    ALIGNED ELLIPSOID atleast 6 distinct samples are required while for generic ELLIPSOIDAL fit
                    which will include calculation of offdiagonal parameters too requires atleast 8 parameters
                    to form a distinct ELLIPSOID

    Sample Time:    Time over which the samples will be gathered and averaged to add to a single sample for least
                    square regression

    offset,diag,offdiag: initial parameter values for LSQ calculation
*/
void AccelCalibrator::start(enum accel_cal_fit_type_t fit_type, uint8_t num_samples, float sample_time) {
    start(fit_type, num_samples, sample_time, Vector3f(0,0,0), Vector3f(1,1,1), Vector3f(0,0,0));
}

void AccelCalibrator::start(enum accel_cal_fit_type_t fit_type, uint8_t num_samples, float sample_time, Vector3f offset, Vector3f diag, Vector3f offdiag) {
    if (_status == ACCEL_CAL_FAILED || _status == ACCEL_CAL_SUCCESS) {
        clear();
    }
    if (_status != ACCEL_CAL_NOT_STARTED) {
        return;
    }

    _conf_num_samples = num_samples;
    _conf_sample_time = sample_time;
    _conf_fit_type = fit_type;

    const uint16_t faces = 2*_conf_num_samples-4;
    const float a = (4.0f * M_PI / (3.0f * faces)) + M_PI / 3.0f;
    const float theta = 0.5f * acosf(cosf(a) / (1.0f - cosf(a)));
    _min_sample_dist = GRAVITY_MSS * 2*sinf(theta/2);

    _param.s.offset = offset;
    _param.s.diag = diag;
    _param.s.offdiag = offdiag;

    switch (_conf_fit_type) {
        case ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID:
            if (_conf_num_samples < 6) {
                set_status(ACCEL_CAL_FAILED);
                return;
            }
            break;
        case ACCEL_CAL_ELLIPSOID:
            if (_conf_num_samples < 8) {
                set_status(ACCEL_CAL_FAILED);
                return;
            }
            break;
    }

    set_status(ACCEL_CAL_WAITING_FOR_ORIENTATION);
}


// set Accel calibrator status to make itself ready for future accel cals
void AccelCalibrator::clear() {
    set_status(ACCEL_CAL_NOT_STARTED);
}

// returns true if accel calibrator is running
bool AccelCalibrator::running() {
    return _status == ACCEL_CAL_WAITING_FOR_ORIENTATION || _status == ACCEL_CAL_COLLECTING_SAMPLE;
}

// set Accel calibrator to start collecting samples in the next cycle
void AccelCalibrator::collect_sample() {
    set_status(ACCEL_CAL_COLLECTING_SAMPLE);
}

// collect and avg sample to be passed onto LSQ estimator after all requisite orientations are done
void AccelCalibrator::new_sample(const Vector3f& delta_velocity, float dt) {
    if (_status != ACCEL_CAL_COLLECTING_SAMPLE) {
        return;
    }

    if (_samples_collected >= _conf_num_samples) {
        set_status(ACCEL_CAL_FAILED);
        return;
    }

    _sample_buffer[_samples_collected].delta_velocity += delta_velocity;
    _sample_buffer[_samples_collected].delta_time += dt;

    _last_samp_frag_collected_ms = AP_HAL::millis();

    if (_sample_buffer[_samples_collected].delta_time > _conf_sample_time) {
        Vector3f sample = _sample_buffer[_samples_collected].delta_velocity/_sample_buffer[_samples_collected].delta_time;
        if (!accept_sample(sample)) {
            set_status(ACCEL_CAL_FAILED);
            return;
        }

        _samples_collected++;

        if (_samples_collected >= _conf_num_samples) {
            run_fit(MAX_ITERATIONS, _fitness);

            if (_fitness < _conf_tolerance && accept_result()) {
                set_status(ACCEL_CAL_SUCCESS);
            } else {
                set_status(ACCEL_CAL_FAILED);
            }
        } else {
            set_status(ACCEL_CAL_WAITING_FOR_ORIENTATION);
        }
    }
}

// determines if the result is acceptable
bool AccelCalibrator::accept_result() const {
    if (fabsf(_param.s.offset.x) > GRAVITY_MSS ||
        fabsf(_param.s.offset.y) > GRAVITY_MSS ||
        fabsf(_param.s.offset.z) > GRAVITY_MSS ||
        _param.s.diag.x < 0.8f || _param.s.diag.x > 1.2f ||
        _param.s.diag.y < 0.8f || _param.s.diag.y > 1.2f ||
        _param.s.diag.z < 0.8f || _param.s.diag.z > 1.2f) {
        return false;
    } else {
        return true;
    }
}

// interface for LSq estimator to read sample buffer sent after conversion from delta velocity
// to averaged acc over time
bool AccelCalibrator::get_sample(uint8_t i, Vector3f& s) const {
    if (i >= _samples_collected) {
        return false;
    }
    s = _sample_buffer[i].delta_velocity / _sample_buffer[i].delta_time;
    return true;
}

// returns true and sample corrected with diag offdiag parameters as calculated by LSq estimation procedure
// returns false if no correct parameter exists to be applied along with existing sample without corrections
bool AccelCalibrator::get_sample_corrected(uint8_t i, Vector3f& s) const {
    if (_status != ACCEL_CAL_SUCCESS || !get_sample(i,s)) {
        return false;
    }

    Matrix3f M (
        _param.s.diag.x    , _param.s.offdiag.x , _param.s.offdiag.y,
        _param.s.offdiag.x , _param.s.diag.y    , _param.s.offdiag.z,
        _param.s.offdiag.y , _param.s.offdiag.z , _param.s.diag.z
    );

    s = M*(s+_param.s.offset);

    return true;
}

// checks if no new sample has been received for considerable amount of time
void AccelCalibrator::check_for_timeout() {
    const uint32_t timeout = _conf_sample_time*2*1000 + 500;
    if (_status == ACCEL_CAL_COLLECTING_SAMPLE && AP_HAL::millis() - _last_samp_frag_collected_ms > timeout) {
        set_status(ACCEL_CAL_FAILED);
    }
}

// returns spherical fit parameters
void AccelCalibrator::get_calibration(Vector3f& offset) const {
    offset = -_param.s.offset;
}

// returns axis aligned ellipsoidal fit parameters
void AccelCalibrator::get_calibration(Vector3f& offset, Vector3f& diag) const {
    offset = -_param.s.offset;
    diag = _param.s.diag;
}

// returns generic ellipsoidal fit parameters
void AccelCalibrator::get_calibration(Vector3f& offset, Vector3f& diag, Vector3f& offdiag) const {
    offset = -_param.s.offset;
    diag = _param.s.diag;
    offdiag = _param.s.offdiag;
}

/////////////////////////////////////////////////////////////
////////////////////// PRIVATE METHODS //////////////////////
/////////////////////////////////////////////////////////////

/*
  The sample acceptance distance is determined as follows:
  For any regular polyhedron with triangular faces, the angle theta subtended
  by two closest points is defined as
 
       theta = arccos(cos(A)/(1-cos(A)))
 
  Where:
       A = (4pi/F + pi)/3
  and
       F = 2V - 4 is the number of faces for the polyhedron in consideration,
       which depends on the number of vertices V
 
  The above equation was proved after solving for spherical triangular excess
  and related equations.
 */
bool AccelCalibrator::accept_sample(const Vector3f& sample)
{
    if (_sample_buffer == nullptr) {
        return false;
    }

    for(uint8_t i=0; i < _samples_collected; i++) {
        Vector3f other_sample;
        get_sample(i, other_sample);

        if ((other_sample - sample).length() < _min_sample_dist){
            return false;
        }
    }
    return true;
}

// sets status of calibrator and takes appropriate actions
void AccelCalibrator::set_status(enum accel_cal_status_t status) {
    switch (status) {
        case ACCEL_CAL_NOT_STARTED:                 
            //Calibrator not started
            _status = ACCEL_CAL_NOT_STARTED;

            _samples_collected = 0;
            if (_sample_buffer != nullptr) {
                free(_sample_buffer);
                _sample_buffer = nullptr;
            }

            break;

        case ACCEL_CAL_WAITING_FOR_ORIENTATION:     
            //Callibrator has been started and is waiting for user to ack after orientation setting
            if (!running()) {
                _samples_collected = 0;
                if (_sample_buffer == nullptr) {
                    _sample_buffer = (struct AccelSample*)calloc(_conf_num_samples,sizeof(struct AccelSample));
                    if (_sample_buffer == nullptr) {
                        set_status(ACCEL_CAL_FAILED);
                        break;
                    }
                }
            }
            if (_samples_collected >= _conf_num_samples) {
                break;
            }
            _status = ACCEL_CAL_WAITING_FOR_ORIENTATION;
            break;

        case ACCEL_CAL_COLLECTING_SAMPLE:
            // Calibrator is waiting on collecting samples from accelerometer for amount of 
            // time as requested by user/GCS
            if (_status != ACCEL_CAL_WAITING_FOR_ORIENTATION) {
                break;
            }
            _last_samp_frag_collected_ms = AP_HAL::millis();
            _status = ACCEL_CAL_COLLECTING_SAMPLE;
            break;

        case ACCEL_CAL_SUCCESS:
            // Calibrator has successfully fitted the samples to user requested surface model 
            // and has passed tolerance test
            if (_status != ACCEL_CAL_COLLECTING_SAMPLE) {
                break;
            }

            _status = ACCEL_CAL_SUCCESS;
            break;

        case ACCEL_CAL_FAILED:
            // Calibration has failed with reasons that can range from 
            // bad sample data leading to failure in tolerance test to lack of distinct samples
            if (_status == ACCEL_CAL_NOT_STARTED) {
                break;
            }

            _status = ACCEL_CAL_FAILED;
            break;
    };
}

/*
    Run Gauss Newton fitting algorithm over the sample space and come up with offsets, diagonal/scale factors
    and crosstalk/offdiagonal parameters
*/
void AccelCalibrator::run_fit(uint8_t max_iterations, float& fitness)
{
    if (_sample_buffer == nullptr) {
        return;
    }
    fitness = calc_mean_squared_residuals(_param.s);
    float min_fitness = fitness;
    union param_u fit_param = _param;
    uint8_t num_iterations = 0;

    const auto sample_at = [this](uint16_t k) {
        Vector3f sample;
        get_sample(k, sample);
        return sample;
    };
    const auto residual_of = [this](const Vector3f &sample, const struct param_t &params) {
        return calc_residual(sample, params);
    };

    while(num_iterations < max_iterations) {
        const bool solved = get_num_params() == 9
            ? gauss_newton_step<9>(fit_param.s, _samples_collected, sample_at, residual_of)
            : gauss_newton_step<6>(fit_param.s, _samples_collected, sample_at, residual_of);
        if (!solved) {
            return;
        }

        fitness = calc_mean_squared_residuals(fit_param.s);

        if (isnan(fitness) || isinf(fitness)) {
            return;
        }

        if (fitness < min_fitness) {
            min_fitness = fitness;
            _param = fit_param;
        }

        num_iterations++;
    }
}

// calculates residual from samples(corrected using supplied parameter) to gravity
// used to create Fitness column matrix
float AccelCalibrator::calc_residual(const Vector3f& sample, const struct param_t& params) const {
    Matrix3f M (
        params.diag.x    , params.offdiag.x , params.offdiag.y,
        params.offdiag.x , params.diag.y    , params.offdiag.z,
        params.offdiag.y , params.offdiag.z , params.diag.z
    );
    return GRAVITY_MSS - (M*(sample+params.offset)).length();
}

// calculated the total mean squared fitness of all the collected samples using parameters
// supplied
float AccelCalibrator::calc_mean_squared_residuals(const struct param_t& params) const
{
    if (_sample_buffer == nullptr || _samples_collected == 0) {
        return 1.0e30f;
    }
    float sum = 0.0f;
    for(uint16_t i=0; i < _samples_collected; i++){
        Vector3f sample;
        get_sample(i, sample);
        float resid = calc_residual(sample, params);
        sum += sq(resid);
    }
    sum /= _samples_collected;
    return sum;
}

// returns number of parameters are required for selected Fit type
uint8_t AccelCalibrator::get_num_params() const {
    switch (_conf_fit_type) {
        case ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID:
            return 6;
        case ACCEL_CAL_ELLIPSOID:
            return 9;
        default:
            return 6;
    }
}
