// Minimal quaternion / vector math so the SDK stays dependency free.
#ifndef N100_MATH_HPP_
#define N100_MATH_HPP_

#include <cmath>

namespace n100 {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }

  double norm() const { return std::sqrt(x * x + y * y + z * z); }
};

// Hamilton convention, stored as (w, x, y, z), unit length by contract.
struct Quat {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Quat() = default;
  Quat(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

  double norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

  Quat normalized() const {
    const double n = norm();
    if (n < 1e-12) return Quat();
    return {w / n, x / n, y / n, z / n};
  }

  Quat conjugate() const { return {w, -x, -y, -z}; }

  Quat operator*(const Quat& o) const {
    return {w * o.w - x * o.x - y * o.y - z * o.z,
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w};
  }

  // v expressed in the body frame -> expressed in the reference frame.
  Vec3 rotate(const Vec3& v) const {
    const Vec3 u{x, y, z};
    const Vec3 t{2.0 * (u.y * v.z - u.z * v.y),
                 2.0 * (u.z * v.x - u.x * v.z),
                 2.0 * (u.x * v.y - u.y * v.x)};
    return {v.x + w * t.x + (u.y * t.z - u.z * t.y),
            v.y + w * t.y + (u.z * t.x - u.x * t.z),
            v.z + w * t.z + (u.x * t.y - u.y * t.x)};
  }

  // v expressed in the reference frame -> expressed in the body frame.
  Vec3 inverseRotate(const Vec3& v) const { return conjugate().rotate(v); }

  // Rotation of `angle` radians about a principal axis.
  static Quat fromAxisAngleX(double angle) {
    return {std::cos(angle * 0.5), std::sin(angle * 0.5), 0.0, 0.0};
  }
  static Quat fromAxisAngleY(double angle) {
    return {std::cos(angle * 0.5), 0.0, std::sin(angle * 0.5), 0.0};
  }
  static Quat fromAxisAngleZ(double angle) {
    return {std::cos(angle * 0.5), 0.0, 0.0, std::sin(angle * 0.5)};
  }

  // Intrinsic Z-Y-X composition, i.e. yaw then pitch then roll.
  static Quat fromEulerZYX(double roll, double pitch, double yaw) {
    return (fromAxisAngleZ(yaw) * fromAxisAngleY(pitch) * fromAxisAngleX(roll))
        .normalized();
  }
};

// Intrinsic Z-Y-X (yaw, pitch, roll) decomposition in radians.
struct Euler {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

inline Euler toEuler(const Quat& q) {
  Euler e;
  e.roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y));

  double sin_pitch = 2.0 * (q.w * q.y - q.z * q.x);
  if (sin_pitch > 1.0) sin_pitch = 1.0;
  if (sin_pitch < -1.0) sin_pitch = -1.0;
  e.pitch = std::asin(sin_pitch);

  e.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return e;
}

// Gravity direction expressed in the body frame, as used by walking policies.
// Level and upright gives (0, 0, -1).
inline Vec3 projectedGravity(const Quat& orientation) {
  return orientation.inverseRotate(Vec3{0.0, 0.0, -1.0});
}

}  // namespace n100

#endif  // N100_MATH_HPP_
