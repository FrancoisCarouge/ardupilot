#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>

#include <AP_LinearAlgebra/AP_LinearAlgebra.h>
#include <AP_LinearAlgebra/AP_LinearAlgebra_Units.h>

#include <type_traits>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using AP_LinearAlgebra::Matrix;

template <size_t R, size_t C>
static void expect_matrix_eq(const Matrix<float, R, C> &a, const Matrix<float, R, C> &b)
{
    for (size_t k = 0; k < R * C; k++) {
        EXPECT_FLOAT_EQ(a.v[k], b.v[k]);
    }
}

TEST(LinearAlgebraMatrix, ZeroIdentity)
{
    const Matrix<float, 2, 3> z = Matrix<float, 2, 3>::Zero();
    for (float e : z.v) {
        EXPECT_FLOAT_EQ(e, 0.0f);
    }
    const Matrix<float, 3, 3> i = Matrix<float, 3, 3>::Identity();
    for (size_t r = 0; r < 3; r++) {
        for (size_t c = 0; c < 3; c++) {
            EXPECT_FLOAT_EQ(i(r, c), r == c ? 1.0f : 0.0f);
        }
    }
}

TEST(LinearAlgebraMatrix, Arithmetic)
{
    const Matrix<float, 2, 2> a{{1, 2, 3, 4}};
    const Matrix<float, 2, 2> b{{5, 6, 7, 8}};
    const Matrix<float, 2, 2> sum{{6, 8, 10, 12}};
    const Matrix<float, 2, 2> difference{{-4, -4, -4, -4}};
    const Matrix<float, 2, 2> product{{19, 22, 43, 50}};
    const Matrix<float, 2, 2> scaled{{2, 4, 6, 8}};
    const Matrix<float, 2, 2> transposed{{1, 3, 2, 4}};
    expect_matrix_eq(a + b, sum);
    expect_matrix_eq(a - b, difference);
    expect_matrix_eq(a * b, product);
    expect_matrix_eq(a * 2.0f, scaled);
    expect_matrix_eq(2.0f * a, scaled);
    expect_matrix_eq(scaled / 2.0f, a);
    expect_matrix_eq(-a, a * -1.0f);
    expect_matrix_eq(a.transpose(), transposed);
}

TEST(LinearAlgebraMatrix, NonSquareProduct)
{
    const Matrix<float, 2, 3> a{{1, 2, 3, 4, 5, 6}};
    const Matrix<float, 3, 1> x{{1, 0, -1}};
    const Matrix<float, 2, 1> y = a * x;
    EXPECT_FLOAT_EQ(y(0), -2.0f);
    EXPECT_FLOAT_EQ(y(1), -2.0f);
}

TEST(LinearAlgebraMatrix, RightDivision)
{
    // a / b = a * inverse(b), here with a row swap needed by the pivoting
    const Matrix<float, 3, 3> b{{0, 2, 1, 1, 1, 0, 3, 0, 1}};
    const Matrix<float, 2, 3> a{{1, 2, 3, 4, 5, 6}};
    const Matrix<float, 2, 3> x = a / b;
    const Matrix<float, 2, 3> back = x * b;
    for (size_t k = 0; k < 6; k++) {
        EXPECT_NEAR(back.v[k], a.v[k], 1e-5f);
    }
    // scalar division of a 1x1 matrix
    const Matrix<float, 1, 1> s{{4}};
    const Matrix<float, 2, 1> c{{2, 6}};
    const Matrix<float, 2, 1> q = c / s;
    EXPECT_FLOAT_EQ(q(0), 0.5f);
    EXPECT_FLOAT_EQ(q(1), 1.5f);
}

TEST(LinearAlgebraMatrix, NonSquareDivision)
{
    // more rows than columns: the minimum norm solution of x * b = a
    const Matrix<float, 3, 1> b{{1, 2, 2}};
    const Matrix<float, 2, 1> a{{3, 6}};
    const Matrix<float, 2, 3> x = a / b;
    const Matrix<float, 2, 1> back = x * b;
    for (size_t k = 0; k < 2; k++) {
        EXPECT_NEAR(back.v[k], a.v[k], 1e-5f);
    }
    // x = a * transpose(b) / (transpose(b) * b): row 0 is 3 * [1 2 2] / 9
    EXPECT_NEAR(x(0, 0), 1.0f / 3.0f, 1e-6f);
    EXPECT_NEAR(x(0, 1), 2.0f / 3.0f, 1e-6f);
    EXPECT_NEAR(x(0, 2), 2.0f / 3.0f, 1e-6f);

    // fewer rows than columns: the least squares solution of x * b = a
    const Matrix<float, 1, 2> c{{1, 1}};
    const Matrix<float, 1, 2> d{{2, 4}};
    const Matrix<float, 1, 1> y = d / c;
    EXPECT_NEAR(y(0, 0), 3.0f, 1e-6f);

    // scalar division by a column vector
    const Matrix<float, 1, 3> z = 9.0f / b;
    EXPECT_NEAR(z(0, 0), 1.0f, 1e-6f);
    EXPECT_NEAR(z(0, 1), 2.0f, 1e-6f);
    EXPECT_NEAR(z(0, 2), 2.0f, 1e-6f);
}

TEST(LinearAlgebraTyped, UnitVector)
{
    using namespace mp_units::si::unit_symbols;
    using AP_LinearAlgebra::Units::Metres;
    using AP_LinearAlgebra::Units::MetresPerSecond;
    using State = AP_LinearAlgebra::ColumnVector<float, Metres, MetresPerSecond>;

    const State x{2.0f * m, 3.0f * (m / s)};
    const State y{x + x};

    static_assert(std::is_same_v<std::remove_cvref_t<decltype(y.at<0>())>, Metres>);
    static_assert(std::is_same_v<std::remove_cvref_t<decltype(y.at<1>())>, MetresPerSecond>);

    EXPECT_FLOAT_EQ(y.at<0>().numerical_value_in(m), 4.0f);
    EXPECT_FLOAT_EQ(y.at<1>().numerical_value_in(m / s), 6.0f);
    // the same quantity in another unit
    EXPECT_FLOAT_EQ(y.at<0>().numerical_value_in(cm), 400.0f);
}

AP_GTEST_MAIN()
