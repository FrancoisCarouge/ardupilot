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
#include <AP_LinearAlgebra/AP_LinearAlgebra.h>
#include <AP_LinearAlgebra/AP_LinearAlgebra_Units.h>

#include <type_traits>

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

    _param.offset = offset;
    _param.diag = diag;
    _param.offdiag = offdiag;

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
    if (fabsf(_param.offset.x) > GRAVITY_MSS ||
        fabsf(_param.offset.y) > GRAVITY_MSS ||
        fabsf(_param.offset.z) > GRAVITY_MSS ||
        _param.diag.x < 0.8f || _param.diag.x > 1.2f ||
        _param.diag.y < 0.8f || _param.diag.y > 1.2f ||
        _param.diag.z < 0.8f || _param.diag.z > 1.2f) {
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
        _param.diag.x    , _param.offdiag.x , _param.offdiag.y,
        _param.offdiag.x , _param.diag.y    , _param.offdiag.z,
        _param.offdiag.y , _param.offdiag.z , _param.diag.z
    );

    s = M*(s+_param.offset);

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
    offset = -_param.offset;
}

// returns axis aligned ellipsoidal fit parameters
void AccelCalibrator::get_calibration(Vector3f& offset, Vector3f& diag) const {
    offset = -_param.offset;
    diag = _param.diag;
}

// returns generic ellipsoidal fit parameters
void AccelCalibrator::get_calibration(Vector3f& offset, Vector3f& diag, Vector3f& offdiag) const {
    offset = -_param.offset;
    diag = _param.diag;
    offdiag = _param.offdiag;
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
namespace {

using AP_LinearAlgebra::ColumnVector;
using AP_LinearAlgebra::RowVector;
using AP_LinearAlgebra::Units::MetresPerSecondSquared;
using AP_LinearAlgebra::Units::Unitless;

constexpr auto metres_per_second_squared = mp_units::si::metre / mp_units::si::second / mp_units::si::second;

/*
  the fitted parameters: offsets (m/s/s), diagonal and, for an ellipsoid,
  off-diagonal scale factors (unitless); and the Jacobian, the partial
  derivatives of a residual (m/s/s) with respect to them
 */
template <uint8_t N> struct Fit;

template <> struct Fit<6> {
    using Params = ColumnVector<float,
                                MetresPerSecondSquared, MetresPerSecondSquared, MetresPerSecondSquared,
                                Unitless, Unitless, Unitless>;
    using Jacobian = RowVector<float,
                               Unitless, Unitless, Unitless,
                               MetresPerSecondSquared, MetresPerSecondSquared, MetresPerSecondSquared>;

    static Params params(const AccelCalibrator::param_t &p)
    {
        const auto a = metres_per_second_squared;
        const auto u = mp_units::one;
        return Params{p.offset.x * a, p.offset.y * a, p.offset.z * a,
                      p.diag.x * u, p.diag.y * u, p.diag.z * u};
    }
    static void set(AccelCalibrator::param_t &p, const Params &x)
    {
        const auto a = metres_per_second_squared;
        const auto u = mp_units::one;
        p.offset = Vector3f(x.at<0>().numerical_value_in(a), x.at<1>().numerical_value_in(a), x.at<2>().numerical_value_in(a));
        p.diag = Vector3f(x.at<3>().numerical_value_in(u), x.at<4>().numerical_value_in(u), x.at<5>().numerical_value_in(u));
    }
    static Jacobian jacobian(const float j[])
    {
        const auto a = metres_per_second_squared;
        const auto u = mp_units::one;
        return Jacobian{j[0] * u, j[1] * u, j[2] * u, j[3] * a, j[4] * a, j[5] * a};
    }
};

template <> struct Fit<9> {
    using Params = ColumnVector<float,
                                MetresPerSecondSquared, MetresPerSecondSquared, MetresPerSecondSquared,
                                Unitless, Unitless, Unitless,
                                Unitless, Unitless, Unitless>;
    using Jacobian = RowVector<float,
                               Unitless, Unitless, Unitless,
                               MetresPerSecondSquared, MetresPerSecondSquared, MetresPerSecondSquared,
                               MetresPerSecondSquared, MetresPerSecondSquared, MetresPerSecondSquared>;

    static Params params(const AccelCalibrator::param_t &p)
    {
        const auto a = metres_per_second_squared;
        const auto u = mp_units::one;
        return Params{p.offset.x * a, p.offset.y * a, p.offset.z * a,
                      p.diag.x * u, p.diag.y * u, p.diag.z * u,
                      p.offdiag.x * u, p.offdiag.y * u, p.offdiag.z * u};
    }
    static void set(AccelCalibrator::param_t &p, const Params &x)
    {
        const auto a = metres_per_second_squared;
        const auto u = mp_units::one;
        p.offset = Vector3f(x.at<0>().numerical_value_in(a), x.at<1>().numerical_value_in(a), x.at<2>().numerical_value_in(a));
        p.diag = Vector3f(x.at<3>().numerical_value_in(u), x.at<4>().numerical_value_in(u), x.at<5>().numerical_value_in(u));
        p.offdiag = Vector3f(x.at<6>().numerical_value_in(u), x.at<7>().numerical_value_in(u), x.at<8>().numerical_value_in(u));
    }
    static Jacobian jacobian(const float j[])
    {
        const auto a = metres_per_second_squared;
        const auto u = mp_units::one;
        return Jacobian{j[0] * u, j[1] * u, j[2] * u, j[3] * a, j[4] * a, j[5] * a, j[6] * a, j[7] * a, j[8] * a};
    }
};

/*
  the normal equations and their solution are kept out of run_fit_typed():
  typed matrix operations return values, and these temporaries would
  otherwise add up in a single stack frame
 */
template <typename NormalMatrix, typename Jacobian>
NOINLINE void accumulate(NormalMatrix &JTJ, const Jacobian &J)
{
    // JTJ = JTJ + transpose(J) * J, with the units checked on the typed
    // expression and the elements added in place: the typed expression
    // copies the whole matrix at each operation, too much stack on a
    // microcontroller
    static_assert(std::is_same_v<decltype(JTJ + fcarouge::transposed(J) * J), NormalMatrix>);
    auto &jtj = JTJ.data();
    const auto &j = J.data();
    for (size_t row = 0; row < NormalMatrix::rows; row++) {
        for (size_t col = 0; col < NormalMatrix::columns; col++) {
            jtj(row, col) += j(row) * j(col);
        }
    }
}

template <typename NormalVector, typename Jacobian>
NOINLINE void accumulate(NormalVector &JTFI, const Jacobian &J, const MetresPerSecondSquared &residual)
{
    JTFI = JTFI + fcarouge::transposed(J) * residual;
}

// Gauss Newton step: solve JTJ * delta = JTFI, JTJ being symmetric; a
// singular JTJ gives non-finite parameters
template <typename Params, typename NormalMatrix, typename NormalVector>
NOINLINE Params solve(const NormalMatrix &JTJ, const NormalVector &JTFI)
{
    return fcarouge::transposed(fcarouge::transposed(JTFI) / JTJ);
}

} // namespace

void AccelCalibrator::run_fit(uint8_t max_iterations, float& fitness)
{
    if (get_num_params() == 9) {
        run_fit_typed<9>(max_iterations, fitness);
    } else {
        run_fit_typed<6>(max_iterations, fitness);
    }
}

template <uint8_t N>
void AccelCalibrator::run_fit_typed(uint8_t max_iterations, float& fitness)
{
    using Params = typename Fit<N>::Params;
    using Jacobian = typename Fit<N>::Jacobian;
    using NormalMatrix = decltype(fcarouge::transposed(Jacobian{}) * Jacobian{});
    using NormalVector = decltype(fcarouge::transposed(Jacobian{}) * MetresPerSecondSquared{});

    if (_sample_buffer == nullptr) {
        return;
    }
    fitness = calc_mean_squared_residuals(_param);
    float min_fitness = fitness;
    struct param_t fit_param = _param;
    uint8_t num_iterations = 0;

    while(num_iterations < max_iterations) {
        NormalMatrix JTJ{};
        NormalVector JTFI{};

        for(uint16_t k = 0; k<_samples_collected; k++) {
            Vector3f sample;
            get_sample(k, sample);

            float jacob[ACCEL_CAL_MAX_NUM_PARAMS] {};
            calc_jacob(sample, fit_param, jacob);
            const Jacobian J = Fit<N>::jacobian(jacob);
            const MetresPerSecondSquared residual = calc_residual(sample, fit_param) * metres_per_second_squared;

            accumulate(JTJ, J);
            accumulate(JTFI, J, residual);
        }

        // a singular JTJ gives non-finite parameters, rejected below
        const Params delta = solve<Params>(JTJ, JTFI);
        Fit<N>::set(fit_param, Fit<N>::params(fit_param) - delta);

        fitness = calc_mean_squared_residuals(fit_param);

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

// calculate jacobian, a matrix that defines relation to variation in fitness with variation in each of the parameters
// this is used in LSq estimator to adjust variation in parameter to be used for next iteration of LSq
void AccelCalibrator::calc_jacob(const Vector3f& sample, const struct param_t& params, float ret[ACCEL_CAL_MAX_NUM_PARAMS]) const {
    switch (_conf_fit_type) {
        case ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID:
        case ACCEL_CAL_ELLIPSOID: {
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

            // 0-2: offsets
            ret[0] = -1.0f * (((diag.x    * A) + (offdiag.x * B) + (offdiag.y * C))/length);
            ret[1] = -1.0f * (((offdiag.x * A) + (diag.y    * B) + (offdiag.z * C))/length);
            ret[2] = -1.0f * (((offdiag.y * A) + (offdiag.z * B) + (diag.z    * C))/length);
            // 3-5: diagonals
            ret[3] = -1.0f * ((sample.x + offset.x) * A)/length;
            ret[4] = -1.0f * ((sample.y + offset.y) * B)/length;
            ret[5] = -1.0f * ((sample.z + offset.z) * C)/length;
            // 6-8: off-diagonals
            ret[6] = -1.0f * (((sample.y + offset.y) * A) + ((sample.x + offset.x) * B))/length;
            ret[7] = -1.0f * (((sample.z + offset.z) * A) + ((sample.x + offset.x) * C))/length;
            ret[8] = -1.0f * (((sample.z + offset.z) * B) + ((sample.y + offset.y) * C))/length;
            return;
        }
    }
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
