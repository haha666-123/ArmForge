#include "test_framework.h"

#include "core/Math.h"

using namespace af;

TEST(math_vec3_basics) {
    const Vec3 a{1.0, 2.0, 3.0};
    const Vec3 b{4.0, -5.0, 6.0};

    CHECK_NEAR((a + b).x, 5.0, 1e-12);
    CHECK_NEAR((a - b).z, -3.0, 1e-12);
    CHECK_NEAR(dot(a, b), 12.0, 1e-12);
    CHECK_NEAR(a.norm(), std::sqrt(14.0), 1e-12);

    const Vec3 c = cross(Vec3{1, 0, 0}, Vec3{0, 1, 0});
    CHECK_NEAR(c.x, 0.0, 1e-12);
    CHECK_NEAR(c.y, 0.0, 1e-12);
    CHECK_NEAR(c.z, 1.0, 1e-12);

    const Vec3 unit = Vec3{3.0, 4.0, 0.0}.normalized();
    CHECK_NEAR(unit.norm(), 1.0, 1e-12);
}

TEST(math_mat3_identity_and_product) {
    const Mat3 identity = Mat3::identity();
    const Mat3 r = rotation_from_axis_angle(Vec3{0, 0, 1}, kPi / 2.0);

    const Mat3 product = identity * r;
    CHECK_NEAR(product.m[0][0], r.m[0][0], 1e-12);

    const Vec3 rotated = r * Vec3{1.0, 0.0, 0.0};
    CHECK_NEAR(rotated.x, 0.0, 1e-9);
    CHECK_NEAR(rotated.y, 1.0, 1e-9);
}

TEST(math_rotation_vector_round_trip) {
    const Vec3 axis = Vec3{1.0, 2.0, -0.5}.normalized();
    const double angle = 0.9;

    const Mat3 r = rotation_from_axis_angle(axis, angle);
    const Vec3 recovered = rotation_vector(r);

    CHECK_NEAR(recovered.norm(), angle, 1e-9);
    const Vec3 direction = recovered.normalized();
    CHECK_NEAR(dot(direction, axis), 1.0, 1e-9);
}

TEST(math_solve_linear_3x3) {
    DenseMatrix a(3, 3);
    a(0, 0) = 2.0;
    a(0, 1) = 1.0;
    a(1, 0) = 1.0;
    a(1, 1) = 3.0;
    a(2, 2) = 4.0;

    const std::vector<double> b{3.0, 4.0, 8.0};
    const std::vector<double> x = solve_linear(a, b);

    CHECK(x.size() == 3);
    CHECK_NEAR(x[0], 1.0, 1e-12);
    CHECK_NEAR(x[1], 1.0, 1e-12);
    CHECK_NEAR(x[2], 2.0, 1e-12);
}

TEST(math_solve_linear_singular) {
    DenseMatrix singular(2, 2);
    singular(0, 0) = 1.0;
    singular(0, 1) = 2.0;
    singular(1, 0) = 2.0;
    singular(1, 1) = 4.0;

    const std::vector<double> x = solve_linear(singular, {1.0, 2.0});
    CHECK(x.empty());
}

TEST(math_mat4_transform) {
    Mat4 t = Mat4::identity();
    t.m[0][3] = 1.0;
    t.m[1][3] = 2.0;
    t.m[2][3] = 3.0;

    const Vec3 p = t.transform_point(Vec3{1.0, 1.0, 1.0});
    CHECK_NEAR(p.x, 2.0, 1e-12);
    CHECK_NEAR(p.y, 3.0, 1e-12);
    CHECK_NEAR(p.z, 4.0, 1e-12);

    const Vec3 v = t.transform_vector(Vec3{1.0, 1.0, 1.0});
    CHECK_NEAR(v.x, 1.0, 1e-12);
}
