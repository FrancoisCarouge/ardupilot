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

/*
  Strongly typed linear algebra over a fixed size matrix backend.

  Matrix<T, R, C> is the storage backend of the FrancoisCarouge
  TypedLinearAlgebra typed matrices: fixed size, row major, owning, and
  allocation free. The typed aliases give each element its own type, for
  example an mp-units quantity, so that unit mismatches fail to compile.
 */
#pragma once

#include <stddef.h>

#include <tuple>

#include "AP_LinearAlgebra_MathsMacrosPush.h"
#include <fcarouge/typed_linear_algebra.hpp>
#include "AP_LinearAlgebra_MathsMacrosPop.h"

namespace AP_LinearAlgebra {

template <typename T, size_t R, size_t C>
struct Matrix {
    T v[R * C] {};

    static constexpr Matrix Zero()
    {
        return Matrix{};
    }

    static constexpr Matrix Identity()
    {
        Matrix m{};
        for (size_t i = 0; i < R && i < C; i++) {
            m(i, i) = T(1);
        }
        return m;
    }

    // element access, as expected by TypedLinearAlgebra
    constexpr T &operator()(size_t i, size_t j)
    {
        return v[i * C + j];
    }
    constexpr const T &operator()(size_t i, size_t j) const
    {
        return v[i * C + j];
    }
    constexpr T &operator()(size_t i)
    {
        return v[i];
    }
    constexpr const T &operator()(size_t i) const
    {
        return v[i];
    }
    constexpr T &operator[](size_t i, size_t j)
    {
        return v[i * C + j];
    }
    constexpr const T &operator[](size_t i, size_t j) const
    {
        return v[i * C + j];
    }
    constexpr T &operator[](size_t i)
    {
        return v[i];
    }
    constexpr const T &operator[](size_t i) const
    {
        return v[i];
    }

    constexpr Matrix<T, C, R> transpose() const
    {
        Matrix<T, C, R> m{};
        for (size_t i = 0; i < R; i++) {
            for (size_t j = 0; j < C; j++) {
                m(j, i) = (*this)(i, j);
            }
        }
        return m;
    }

    friend constexpr Matrix operator-(Matrix a)
    {
        for (size_t k = 0; k < R * C; k++) {
            a.v[k] = -a.v[k];
        }
        return a;
    }
    friend constexpr Matrix operator+(Matrix a, const Matrix &b)
    {
        for (size_t k = 0; k < R * C; k++) {
            a.v[k] += b.v[k];
        }
        return a;
    }
    friend constexpr Matrix operator-(Matrix a, const Matrix &b)
    {
        for (size_t k = 0; k < R * C; k++) {
            a.v[k] -= b.v[k];
        }
        return a;
    }
    friend constexpr Matrix operator*(Matrix a, const T &s)
    {
        for (size_t k = 0; k < R * C; k++) {
            a.v[k] *= s;
        }
        return a;
    }
    friend constexpr Matrix operator*(const T &s, Matrix a)
    {
        return a * s;
    }
    friend constexpr Matrix operator/(Matrix a, const T &s)
    {
        for (size_t k = 0; k < R * C; k++) {
            a.v[k] /= s;
        }
        return a;
    }

    template <size_t N>
    friend constexpr Matrix<T, R, N> operator*(const Matrix &a, const Matrix<T, C, N> &b)
    {
        Matrix<T, R, N> m{};
        for (size_t i = 0; i < R; i++) {
            for (size_t j = 0; j < N; j++) {
                for (size_t k = 0; k < C; k++) {
                    m(i, j) += a(i, k) * b(k, j);
                }
            }
        }
        return m;
    }

    /*
      right division x = a / b, the solution of x * b = a, as in Eigen:
      for a square b this is a * inverse(b); otherwise the minimum norm
      (more rows than columns) or least squares (fewer rows) solution. Typed
      matrix libraries form, for example, a state vector divided by a state
      vector to get the type of a transition matrix. A rank deficient b gives
      non-finite elements.
     */
    template <size_t N>
    friend constexpr Matrix<T, R, N> operator/(const Matrix &a, const Matrix<T, N, C> &b)
    {
        if constexpr (N == C) {
            return solve_right(a, b);
        } else if constexpr (N > C) {
            return solve_right(a, b.transpose() * b) * b.transpose();
        } else {
            return solve_right(a * b.transpose(), b * b.transpose());
        }
    }

    // scalar s / b: the row vector x solving x * b = s, for a column vector b
    friend constexpr Matrix<T, 1, R> operator/(const T &s, const Matrix &b)
        requires (C == 1)
    {
        return Matrix<T, 1, 1>{{s}} / b;
    }

private:
    // x * b = a for a square b, by Gaussian elimination with partial pivoting
    template <size_t RA, size_t K>
    static constexpr Matrix<T, RA, K> solve_right(const Matrix<T, RA, K> &a, const Matrix<T, K, K> &b)
    {
        // solve transpose(b) * transpose(x) = transpose(a)
        Matrix<T, K, K> m = b.transpose();
        Matrix<T, K, RA> x = a.transpose();
        for (size_t col = 0; col < K; col++) {
            size_t pivot = col;
            for (size_t i = col + 1; i < K; i++) {
                const T candidate = m(i, col) < T(0) ? -m(i, col) : m(i, col);
                const T best = m(pivot, col) < T(0) ? -m(pivot, col) : m(pivot, col);
                if (candidate > best) {
                    pivot = i;
                }
            }
            if (pivot != col) {
                for (size_t j = 0; j < K; j++) {
                    const T t = m(col, j);
                    m(col, j) = m(pivot, j);
                    m(pivot, j) = t;
                }
                for (size_t j = 0; j < RA; j++) {
                    const T t = x(col, j);
                    x(col, j) = x(pivot, j);
                    x(pivot, j) = t;
                }
            }
            for (size_t i = col + 1; i < K; i++) {
                const T factor = m(i, col) / m(col, col);
                for (size_t j = col; j < K; j++) {
                    m(i, j) -= factor * m(col, j);
                }
                for (size_t j = 0; j < RA; j++) {
                    x(i, j) -= factor * x(col, j);
                }
            }
        }
        for (size_t col = K; col-- > 0;) {
            for (size_t j = 0; j < RA; j++) {
                T sum = x(col, j);
                for (size_t k = col + 1; k < K; k++) {
                    sum -= m(col, k) * x(k, j);
                }
                x(col, j) = sum / m(col, col);
            }
        }
        return x.transpose();
    }
};

// a matrix whose element (i, j) has the type of the product of the i-th row
// index and the j-th column index
template <typename T, typename RowIndexes, typename ColumnIndexes>
using TypedMatrix = fcarouge::typed_matrix<
    Matrix<T, std::tuple_size_v<RowIndexes>, std::tuple_size_v<ColumnIndexes>>,
    RowIndexes, ColumnIndexes>;

// a column vector whose i-th element has the i-th type
template <typename T, typename... Types>
using ColumnVector = fcarouge::typed_column_vector<Matrix<T, sizeof...(Types), 1>, Types...>;

// a row vector whose j-th element has the j-th type
template <typename T, typename... Types>
using RowVector = fcarouge::typed_row_vector<Matrix<T, 1, sizeof...(Types)>, Types...>;

} // namespace AP_LinearAlgebra
