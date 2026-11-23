#pragma once

// Minimal linear algebra for robot kinematics.
// Deliberately dependency free: the project ships no third party math library so
// every formula in the kinematics chapter can be traced back to this file.

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace af {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vec3() = default;
    Vec3(double xv, double yv, double zv) : x(xv), y(yv), z(zv) {}

    [[nodiscard]] double operator[](std::size_t index) const {
        return index == 0 ? x : (index == 1 ? y : z);
    }
    double& operator[](std::size_t index) { return index == 0 ? x : (index == 1 ? y : z); }

    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
    Vec3& operator*=(double scale) {
        x *= scale;
        y *= scale;
        z *= scale;
        return *this;
    }

    [[nodiscard]] double squared_norm() const { return x * x + y * y + z * z; }
    [[nodiscard]] double norm() const { return std::sqrt(squared_norm()); }
    [[nodiscard]] Vec3 normalized() const {
        const double length = norm();
        return length > 1e-12 ? Vec3{x / length, y / length, z / length} : Vec3{};
    }
};

inline Vec3 operator+(Vec3 a, const Vec3& b) { return a += b; }
inline Vec3 operator-(Vec3 a, const Vec3& b) { return a -= b; }
inline Vec3 operator-(const Vec3& v) { return Vec3{-v.x, -v.y, -v.z}; }
inline Vec3 operator*(Vec3 v, double s) { return v *= s; }
inline Vec3 operator*(double s, Vec3 v) { return v *= s; }
inline Vec3 operator/(Vec3 v, double s) { return v *= (1.0 / s); }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double distance(const Vec3& a, const Vec3& b) { return (a - b).norm(); }

struct Mat3 {
    std::array<std::array<double, 3>, 3> m{};

    static Mat3 identity() {
        Mat3 out;
        out.m = {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
        return out;
    }

    [[nodiscard]] Vec3 col(std::size_t index) const {
        return Vec3{m[0][index], m[1][index], m[2][index]};
    }
    void set_col(std::size_t index, const Vec3& value) {
        m[0][index] = value.x;
        m[1][index] = value.y;
        m[2][index] = value.z;
    }

    [[nodiscard]] Mat3 transposed() const {
        Mat3 out;
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) out.m[r][c] = m[c][r];
        }
        return out;
    }

    [[nodiscard]] Mat3 operator*(const Mat3& other) const {
        Mat3 out;
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) {
                out.m[r][c] = m[r][0] * other.m[0][c] + m[r][1] * other.m[1][c] +
                              m[r][2] * other.m[2][c];
            }
        }
        return out;
    }

    [[nodiscard]] Vec3 operator*(const Vec3& v) const {
        return Vec3{m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                    m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                    m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
};

struct Mat4 {
    std::array<std::array<double, 4>, 4> m{};

    static Mat4 identity() {
        Mat4 out;
        for (std::size_t i = 0; i < 4; ++i) out.m[i][i] = 1.0;
        return out;
    }

    static Mat4 from_rotation_translation(const Mat3& rotation, const Vec3& translation) {
        Mat4 out = identity();
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) out.m[r][c] = rotation.m[r][c];
        }
        out.m[0][3] = translation.x;
        out.m[1][3] = translation.y;
        out.m[2][3] = translation.z;
        return out;
    }

    [[nodiscard]] Mat4 operator*(const Mat4& other) const {
        Mat4 out;
        for (std::size_t r = 0; r < 4; ++r) {
            for (std::size_t c = 0; c < 4; ++c) {
                double sum = 0.0;
                for (std::size_t k = 0; k < 4; ++k) sum += m[r][k] * other.m[k][c];
                out.m[r][c] = sum;
            }
        }
        return out;
    }

    [[nodiscard]] Vec3 transform_point(const Vec3& p) const {
        return Vec3{m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3],
                    m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3],
                    m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3]};
    }

    [[nodiscard]] Vec3 transform_vector(const Vec3& p) const {
        return Vec3{m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z,
                    m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z,
                    m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z};
    }

    [[nodiscard]] Mat3 rotation() const {
        Mat3 out;
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) out.m[r][c] = m[r][c];
        }
        return out;
    }

    [[nodiscard]] Vec3 translation() const { return Vec3{m[0][3], m[1][3], m[2][3]}; }
};

// Rotation matrix about an arbitrary unit axis (Rodrigues formula).
inline Mat3 rotation_from_axis_angle(const Vec3& axis, double angle) {
    const Vec3 unit = axis.normalized();
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double t = 1.0 - c;

    Mat3 out;
    out.m = {{{t * unit.x * unit.x + c, t * unit.x * unit.y - s * unit.z,
               t * unit.x * unit.z + s * unit.y},
              {t * unit.x * unit.y + s * unit.z, t * unit.y * unit.y + c,
               t * unit.y * unit.z - s * unit.x},
              {t * unit.x * unit.z - s * unit.y, t * unit.y * unit.z + s * unit.x,
               t * unit.z * unit.z + c}}};
    return out;
}

// Rotation vector (axis * angle) of a rotation matrix, used as the orientation
// error in the inverse kinematics solver.
inline Vec3 rotation_vector(const Mat3& r) {
    const double trace = r.m[0][0] + r.m[1][1] + r.m[2][2];
    double cos_angle = (trace - 1.0) * 0.5;
    cos_angle = std::fmax(-1.0, std::fmin(1.0, cos_angle));
    const double angle = std::acos(cos_angle);

    if (angle < 1e-9) return Vec3{};
    if (angle > kPi - 1e-6) {
        // near 180 degrees: derive the axis from the diagonal
        Vec3 axis{std::sqrt(std::fmax(0.0, (r.m[0][0] + 1.0) * 0.5)),
                  std::sqrt(std::fmax(0.0, (r.m[1][1] + 1.0) * 0.5)),
                  std::sqrt(std::fmax(0.0, (r.m[2][2] + 1.0) * 0.5))};
        if (r.m[0][1] < 0.0) axis.y = -axis.y;
        if (r.m[0][2] < 0.0) axis.z = -axis.z;
        return axis.normalized() * angle;
    }
    const Vec3 axis{r.m[2][1] - r.m[1][2], r.m[0][2] - r.m[2][0], r.m[1][0] - r.m[0][1]};
    return axis * (angle / (2.0 * std::sin(angle)));
}

// Row major dense matrix, only as large as the IK problem needs (6 x 6).
class DenseMatrix {
public:
    DenseMatrix(std::size_t rows, std::size_t cols, double fill = 0.0)
        : rows_(rows), cols_(cols), data_(rows * cols, fill) {}

    [[nodiscard]] std::size_t rows() const { return rows_; }
    [[nodiscard]] std::size_t cols() const { return cols_; }

    double& operator()(std::size_t r, std::size_t c) { return data_[r * cols_ + c]; }
    [[nodiscard]] double operator()(std::size_t r, std::size_t c) const {
        return data_[r * cols_ + c];
    }

    [[nodiscard]] DenseMatrix operator*(const DenseMatrix& other) const {
        DenseMatrix out(rows_, other.cols_);
        for (std::size_t r = 0; r < rows_; ++r) {
            for (std::size_t c = 0; c < other.cols_; ++c) {
                double sum = 0.0;
                for (std::size_t k = 0; k < cols_; ++k) sum += (*this)(r, k) * other(k, c);
                out(r, c) = sum;
            }
        }
        return out;
    }

    [[nodiscard]] DenseMatrix transposed() const {
        DenseMatrix out(cols_, rows_);
        for (std::size_t r = 0; r < rows_; ++r) {
            for (std::size_t c = 0; c < cols_; ++c) out(c, r) = (*this)(r, c);
        }
        return out;
    }

    void add_to_diagonal(double value) {
        const std::size_t n = rows_ < cols_ ? rows_ : cols_;
        for (std::size_t i = 0; i < n; ++i) (*this)(i, i) += value;
    }

private:
    std::size_t rows_{0};
    std::size_t cols_{0};
    std::vector<double> data_;
};

// Gaussian elimination with partial pivoting. Returns an empty vector when the
// matrix is singular so callers can report kNoConvergence instead of NaN.
inline std::vector<double> solve_linear(DenseMatrix a, std::vector<double> b) {
    const std::size_t n = a.rows();
    if (a.cols() != n || b.size() != n) return {};

    for (std::size_t col = 0; col < n; ++col) {
        std::size_t pivot = col;
        double best = std::fabs(a(col, col));
        for (std::size_t row = col + 1; row < n; ++row) {
            const double value = std::fabs(a(row, col));
            if (value > best) {
                best = value;
                pivot = row;
            }
        }
        if (best < 1e-12) return {};
        if (pivot != col) {
            for (std::size_t c = 0; c < n; ++c) std::swap(a(col, c), a(pivot, c));
            std::swap(b[col], b[pivot]);
        }
        const double diag = a(col, col);
        for (std::size_t row = col + 1; row < n; ++row) {
            const double factor = a(row, col) / diag;
            if (factor == 0.0) continue;
            for (std::size_t c = col; c < n; ++c) a(row, c) -= factor * a(col, c);
            b[row] -= factor * b[col];
        }
    }

    std::vector<double> x(n, 0.0);
    for (std::size_t i = n; i-- > 0;) {
        double sum = b[i];
        for (std::size_t c = i + 1; c < n; ++c) sum -= a(i, c) * x[c];
        x[i] = sum / a(i, i);
    }
    return x;
}

}  // namespace af
