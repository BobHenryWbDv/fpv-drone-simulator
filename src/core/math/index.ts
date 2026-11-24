/**
 * High-performance math library for FPV drone simulator
 * Optimized for zero-allocation hot paths, SIMD-friendly patterns
 * Compatible with gl-matrix API but with stricter typing and custom extensions
 */

// Re-export gl-matrix for compatibility
export * from 'gl-matrix';

// ============================================================================
// Type Definitions
// ============================================================================

export type Vec3 = Float32Array & { __brand: 'Vec3' };
export type Quat = Float32Array & { __brand: 'Quat' };
export type Mat3 = Float32Array & { __brand: 'Mat3' };
export type Mat4 = Float32Array & { __brand: 'Mat4' };

// Branding helpers (compile-time only, zero runtime cost)
const brandVec3 = <T extends Float32Array>(arr: T): Vec3 => arr as Vec3;
const brandQuat = <T extends Float32Array>(arr: T): Quat => arr as Quat;
const brandMat3 = <T extends Float32Array>(arr: T): Mat3 => arr as Mat3;
const brandMat4 = <T extends Float32Array>(arr: T): Mat4 => arr as Mat4;

// ============================================================================
// Vec3 Operations (3-element Float32Array)
// ============================================================================

export const Vec3 = {
  create(): Vec3 {
    return brandVec3(new Float32Array(3));
  },

  fromValues(x: number, y: number, z: number): Vec3 {
    const out = new Float32Array(3);
    out[0] = x; out[1] = y; out[2] = z;
    return brandVec3(out);
  },

  clone(a: Vec3): Vec3 {
    const out = new Float32Array(3);
    out[0] = a[0]; out[1] = a[1]; out[2] = a[2];
    return brandVec3(out);
  },

  copy(out: Vec3, a: Vec3): Vec3 {
    out[0] = a[0]; out[1] = a[1]; out[2] = a[2];
    return out;
  },

  set(out: Vec3, x: number, y: number, z: number): Vec3 {
    out[0] = x; out[1] = y; out[2] = z;
    return out;
  },

  // Arithmetic
  add(out: Vec3, a: Vec3, b: Vec3): Vec3 {
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
    return out;
  },

  sub(out: Vec3, a: Vec3, b: Vec3): Vec3 {
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
    return out;
  },

  mul(out: Vec3, a: Vec3, b: Vec3): Vec3 {
    out[0] = a[0] * b[0];
    out[1] = a[1] * b[1];
    out[2] = a[2] * b[2];
    return out;
  },

  div(out: Vec3, a: Vec3, b: Vec3): Vec3 {
    out[0] = a[0] / b[0];
    out[1] = a[1] / b[1];
    out[2] = a[2] / b[2];
    return out;
  },

  scale(out: Vec3, a: Vec3, s: number): Vec3 {
    out[0] = a[0] * s;
    out[1] = a[1] * s;
    out[2] = a[2] * s;
    return out;
  },

  scaleAndAdd(out: Vec3, a: Vec3, b: Vec3, s: number): Vec3 {
    out[0] = a[0] + b[0] * s;
    out[1] = a[1] + b[1] * s;
    out[2] = a[2] + b[2] * s;
    return out;
  },

  negate(out: Vec3, a: Vec3): Vec3 {
    out[0] = -a[0];
    out[1] = -a[1];
    out[2] = -a[2];
    return out;
  },

  // Products
  dot(a: Vec3, b: Vec3): number {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  },

  cross(out: Vec3, a: Vec3, b: Vec3): Vec3 {
    const ax = a[0], ay = a[1], az = a[2];
    const bx = b[0], by = b[1], bz = b[2];
    out[0] = ay * bz - az * by;
    out[1] = az * bx - ax * bz;
    out[2] = ax * by - ay * bx;
    return out;
  },

  // Length & normalization
  length(a: Vec3): number {
    return Math.hypot(a[0], a[1], a[2]);
  },

  lengthSq(a: Vec3): number {
    return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
  },

  normalize(out: Vec3, a: Vec3): Vec3 {
    const len = Math.hypot(a[0], a[1], a[2]);
    if (len > 0) {
      out[0] = a[0] / len;
      out[1] = a[1] / len;
      out[2] = a[2] / len;
    } else {
      out[0] = 0; out[1] = 0; out[2] = 0;
    }
    return out;
  },

  // Distance
  distance(a: Vec3, b: Vec3): number {
    const dx = a[0] - b[0];
    const dy = a[1] - b[1];
    const dz = a[2] - b[2];
    return Math.hypot(dx, dy, dz);
  },

  distanceSq(a: Vec3, b: Vec3): number {
    const dx = a[0] - b[0];
    const dy = a[1] - b[1];
    const dz = a[2] - b[2];
    return dx * dx + dy * dy + dz * dz;
  },

  // Interpolation
  lerp(out: Vec3, a: Vec3, b: Vec3, t: number): Vec3 {
    out[0] = a[0] + (b[0] - a[0]) * t;
    out[1] = a[1] + (b[1] - a[1]) * t;
    out[2] = a[2] + (b[2] - a[2]) * t;
    return out;
  },

  // Min/Max/Clamp
  min(out: Vec3, a: Vec3, b: Vec3): Vec3 {
    out[0] = Math.min(a[0], b[0]);
    out[1] = Math.min(a[1], b[1]);
    out[2] = Math.min(a[2], b[2]);
    return out;
  },

  max(out: Vec3, a: Vec3, b: Vec3): Vec3 {
    out[0] = Math.max(a[0], b[0]);
    out[1] = Math.max(a[1], b[1]);
    out[2] = Math.max(a[2], b[2]);
    return out;
  },

  clamp(out: Vec3, a: Vec3, min: Vec3, max: Vec3): Vec3 {
    out[0] = Math.max(min[0], Math.min(max[0], a[0]));
    out[1] = Math.max(min[1], Math.min(max[1], a[1]));
    out[2] = Math.max(min[2], Math.min(max[2], a[2]));
    return out;
  },

  // Transform
  transformQuat(out: Vec3, a: Vec3, q: Quat): Vec3 {
    const x = a[0], y = a[1], z = a[2];
    const qx = q[0], qy = q[1], qz = q[2], qw = q[3];

    // t = 2 * q.xyz × a
    const tx = 2 * (qy * z - qz * y);
    const ty = 2 * (qz * x - qx * z);
    const tz = 2 * (qx * y - qy * x);

    // a + q.w * t + q.xyz × t
    out[0] = x + qw * tx + qy * tz - qz * ty;
    out[1] = y + qw * ty + qz * tx - qx * tz;
    out[2] = z + qw * tz + qx * ty - qy * tx;
    return out;
  },

  transformMat3(out: Vec3, a: Vec3, m: Mat3): Vec3 {
    const x = a[0], y = a[1], z = a[2];
    out[0] = m[0] * x + m[3] * y + m[6] * z;
    out[1] = m[1] * x + m[4] * y + m[7] * z;
    out[2] = m[2] * x + m[5] * y + m[8] * z;
    return out;
  },

  transformMat4(out: Vec3, a: Vec3, m: Mat4): Vec3 {
    const x = a[0], y = a[1], z = a[2];
    const w = m[3] * x + m[7] * y + m[11] * z + m[15];
    out[0] = (m[0] * x + m[4] * y + m[8] * z + m[12]) / w;
    out[1] = (m[1] * x + m[5] * y + m[9] * z + m[13]) / w;
    out[2] = (m[2] * x + m[6] * y + m[10] * z + m[14]) / w;
    return out;
  },

  // Rotation
  rotateX(out: Vec3, a: Vec3, rad: number): Vec3 {
    const y = a[1], z = a[2];
    const c = Math.cos(rad), s = Math.sin(rad);
    out[0] = a[0];
    out[1] = y * c - z * s;
    out[2] = y * s + z * c;
    return out;
  },

  rotateY(out: Vec3, a: Vec3, rad: number): Vec3 {
    const x = a[0], z = a[2];
    const c = Math.cos(rad), s = Math.sin(rad);
    out[0] = x * c + z * s;
    out[1] = a[1];
    out[2] = -x * s + z * c;
    return out;
  },

  rotateZ(out: Vec3, a: Vec3, rad: number): Vec3 {
    const x = a[0], y = a[1];
    const c = Math.cos(rad), s = Math.sin(rad);
    out[0] = x * c - y * s;
    out[1] = x * s + y * c;
    out[2] = a[2];
    return out;
  },

  // Angle between vectors
  angle(a: Vec3, b: Vec3): number {
    const dot = Vec3.dot(a, b);
    const len = Math.sqrt(Vec3.lengthSq(a) * Vec3.lengthSq(b));
    return Math.acos(Math.max(-1, Math.min(1, dot / len)));
  },

  // Projection
  project(out: Vec3, a: Vec3, b: Vec3): Vec3 {
    const dot = Vec3.dot(a, b);
    const lenSq = Vec3.lengthSq(b);
    return lenSq > 0 ? Vec3.scale(out, b, dot / lenSq) : Vec3.set(out, 0, 0, 0);
  },

  reject(out: Vec3, a: Vec3, b: Vec3): Vec3 {
    Vec3.project(out, a, b);
    return Vec3.sub(out, a, out);
  },

  // Orthogonal basis from forward vector
  orthonormalBasis(forward: Vec3, outRight: Vec3, outUp: Vec3): void {
    // Right = normalize(forward × worldUp) or fallback
    const worldUp = Vec3.fromValues(0, 0, 1);
    Vec3.cross(outRight, forward, worldUp);
    if (Vec3.lengthSq(outRight) < 1e-6) {
      Vec3.set(outRight, 1, 0, 0);
    } else {
      Vec3.normalize(outRight, outRight);
    }
    // Up = right × forward
    Vec3.cross(outUp, outRight, forward);
    Vec3.normalize(outUp, outUp);
  },

  // Constants
  ZERO: brandVec3(new Float32Array([0, 0, 0])),
  ONE: brandVec3(new Float32Array([1, 1, 1])),
  X: brandVec3(new Float32Array([1, 0, 0])),
  Y: brandVec3(new Float32Array([0, 1, 0])),
  Z: brandVec3(new Float32Array([0, 0, 1])),
  NEG_X: brandVec3(new Float32Array([-1, 0, 0])),
  NEG_Y: brandVec3(new Float32Array([0, -1, 0])),
  NEG_Z: brandVec3(new Float32Array([0, 0, -1])),
  UP: brandVec3(new Float32Array([0, 0, 1])),
  DOWN: brandVec3(new Float32Array([0, 0, -1])),
  FORWARD: brandVec3(new Float32Array([1, 0, 0])),
  BACK: brandVec3(new Float32Array([-1, 0, 0])),
  LEFT: brandVec3(new Float32Array([0, -1, 0])),
  RIGHT: brandVec3(new Float32Array([0, 1, 0])),
};

// ============================================================================
// Quat Operations (4-element Float32Array, [x, y, z, w] order for gl-matrix compat)
// ============================================================================

export const Quat = {
  create(): Quat {
    const out = new Float32Array(4);
    out[3] = 1; // w = 1
    return brandQuat(out);
  },

  fromValues(x: number, y: number, z: number, w: number): Quat {
    const out = new Float32Array(4);
    out[0] = x; out[1] = y; out[2] = z; out[3] = w;
    return brandQuat(out);
  },

  clone(a: Quat): Quat {
    const out = new Float32Array(4);
    out[0] = a[0]; out[1] = a[1]; out[2] = a[2]; out[3] = a[3];
    return brandQuat(out);
  },

  copy(out: Quat, a: Quat): Quat {
    out[0] = a[0]; out[1] = a[1]; out[2] = a[2]; out[3] = a[3];
    return out;
  },

  set(out: Quat, x: number, y: number, z: number, w: number): Quat {
    out[0] = x; out[1] = y; out[2] = z; out[3] = w;
    return out;
  },

  identity(out: Quat): Quat {
    out[0] = 0; out[1] = 0; out[2] = 0; out[3] = 1;
    return out;
  },

  // From axis-angle
  fromAxisAngle(out: Quat, axis: Vec3, rad: number): Quat {
    const half = rad * 0.5;
    const s = Math.sin(half);
    out[0] = axis[0] * s;
    out[1] = axis[1] * s;
    out[2] = axis[2] * s;
    out[3] = Math.cos(half);
    return out;
  },

  // From Euler angles (ZYX order: yaw, pitch, roll)
  fromEuler(out: Quat, yaw: number, pitch: number, roll: number): Quat {
    const cy = Math.cos(yaw * 0.5);
    const sy = Math.sin(yaw * 0.5);
    const cp = Math.cos(pitch * 0.5);
    const sp = Math.sin(pitch * 0.5);
    const cr = Math.cos(roll * 0.5);
    const sr = Math.sin(roll * 0.5);

    out[0] = sr * cp * cy - cr * sp * sy; // x
    out[1] = cr * sp * cy + sr * cp * sy; // y
    out[2] = cr * cp * sy - sr * sp * cy; // z
    out[3] = cr * cp * cy + sr * sp * sy; // w
    return out;
  },

  // To Euler angles (ZYX order)
  toEuler(out: Vec3, q: Quat): Vec3 {
    const x = q[0], y = q[1], z = q[2], w = q[3];

    // Roll (x-axis rotation)
    const sinr_cosp = 2 * (w * x + y * z);
    const cosr_cosp = 1 - 2 * (x * x + y * y);
    out[0] = Math.atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    const sinp = 2 * (w * y - z * x);
    out[1] = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);

    // Yaw (z-axis rotation)
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    out[2] = Math.atan2(siny_cosp, cosy_cosp);

    return out;
  },

  // From rotation matrix (Mat3)
  fromMat3(out: Quat, m: Mat3): Quat {
    const trace = m[0] + m[4] + m[8];
    if (trace > 0) {
      const s = Math.sqrt(trace + 1) * 2;
      out[3] = 0.25 * s;
      out[0] = (m[7] - m[5]) / s;
      out[1] = (m[2] - m[6]) / s;
      out[2] = (m[3] - m[1]) / s;
    } else if (m[0] > m[4] && m[0] > m[8]) {
      const s = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2;
      out[3] = (m[7] - m[5]) / s;
      out[0] = 0.25 * s;
      out[1] = (m[1] + m[3]) / s;
      out[2] = (m[2] + m[6]) / s;
    } else if (m[4] > m[8]) {
      const s = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2;
      out[3] = (m[2] - m[6]) / s;
      out[0] = (m[1] + m[3]) / s;
      out[1] = 0.25 * s;
      out[2] = (m[5] + m[7]) / s;
    } else {
      const s = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2;
      out[3] = (m[3] - m[1]) / s;
      out[0] = (m[2] + m[6]) / s;
      out[1] = (m[5] + m[7]) / s;
      out[2] = 0.25 * s;
    }
    return out;
  },

  // Multiplication (q1 * q2)
  multiply(out: Quat, a: Quat, b: Quat): Quat {
    const ax = a[0], ay = a[1], az = a[2], aw = a[3];
    const bx = b[0], by = b[1], bz = b[2], bw = b[3];
    out[0] = ax * bw + aw * bx + ay * bz - az * by;
    out[1] = ay * bw + aw * by + az * bx - ax * bz;
    out[2] = az * bw + aw * bz + ax * by - ay * bx;
    out[3] = aw * bw - ax * bx - ay * by - az * bz;
    return out;
  },

  // Conjugate
  conjugate(out: Quat, a: Quat): Quat {
    out[0] = -a[0];
    out[1] = -a[1];
    out[2] = -a[2];
    out[3] = a[3];
    return out;
  },

  // Inverse (same as conjugate for unit quaternions)
  invert(out: Quat, a: Quat): Quat {
    return Quat.conjugate(out, a);
  },

  // Normalize
  normalize(out: Quat, a: Quat): Quat {
    const len = Math.hypot(a[0], a[1], a[2], a[3]);
    if (len > 0) {
      out[0] = a[0] / len;
      out[1] = a[1] / len;
      out[2] = a[2] / len;
      out[3] = a[3] / len;
    } else {
      out[0] = 0; out[1] = 0; out[2] = 0; out[3] = 1;
    }
    return out;
  },

  // Dot product
  dot(a: Quat, b: Quat): number {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  },

  // Length
  length(a: Quat): number {
    return Math.hypot(a[0], a[1], a[2], a[3]);
  },

  // Slerp
  slerp(out: Quat, a: Quat, b: Quat, t: number): Quat {
    let dot = Quat.dot(a, b);
    const bCopy = Quat.create();
    
    if (dot < 0) {
      dot = -dot;
      Quat.scale(bCopy, b, -1);
    } else {
      Quat.copy(bCopy, b);
    }

    if (dot > 0.9995) {
      // Linear interpolation for very close quaternions
      Quat.lerp(out, a, bCopy, t);
      return Quat.normalize(out, out);
    }

    const theta0 = Math.acos(Math.max(-1, Math.min(1, dot)));
    const theta = theta0 * t;
    const sinTheta = Math.sin(theta);
    const sinTheta0 = Math.sin(theta0);

    const s0 = Math.cos(theta) - dot * sinTheta / sinTheta0;
    const s1 = sinTheta / sinTheta0;

    out[0] = s0 * a[0] + s1 * bCopy[0];
    out[1] = s0 * a[1] + s1 * bCopy[1];
    out[2] = s0 * a[2] + s1 * bCopy[2];
    out[3] = s0 * a[3] + s1 * bCopy[3];
    return out;
  },

  // Lerp (unnormalized)
  lerp(out: Quat, a: Quat, b: Quat, t: number): Quat {
    out[0] = a[0] + (b[0] - a[0]) * t;
    out[1] = a[1] + (b[1] - a[1]) * t;
    out[2] = a[2] + (b[2] - a[2]) * t;
    out[3] = a[3] + (b[3] - a[3]) * t;
    return out;
  },

  // Rotate vector by quaternion
  rotateVec3(out: Vec3, q: Quat, v: Vec3): Vec3 {
    return Vec3.transformQuat(out, v, q);
  },

  // Integration: q' = q + 0.5 * q ⊗ [0, ω] * dt
  integrate(out: Quat, q: Quat, omega: Vec3, dt: number): Quat {
    const wx = omega[0] * 0.5 * dt;
    const wy = omega[1] * 0.5 * dt;
    const wz = omega[2] * 0.5 * dt;

    const qx = q[0], qy = q[1], qz = q[2], qw = q[3];

    out[0] = qx + qw * wx + qy * wz - qz * wy;
    out[1] = qy + qw * wy + qz * wx - qx * wz;
    out[2] = qz + qw * wz + qx * wy - qy * wx;
    out[3] = qw - qx * wx - qy * wy - qz * wz;

    return Quat.normalize(out, out);
  },

  // Scale (multiply all components)
  scale(out: Quat, a: Quat, s: number): Quat {
    out[0] = a[0] * s;
    out[1] = a[1] * s;
    out[2] = a[2] * s;
    out[3] = a[3] * s;
    return out;
  },

  // Add
  add(out: Quat, a: Quat, b: Quat): Quat {
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
    out[3] = a[3] + b[3];
    return out;
  },

  // Constants
  IDENTITY: brandQuat(new Float32Array([0, 0, 0, 1])),
  ZERO: brandQuat(new Float32Array([0, 0, 0, 0])),
};

// ============================================================================
// Mat3 Operations (9-element Float32Array, column-major)
// ============================================================================

export const Mat3 = {
  create(): Mat3 {
    return brandMat3(new Float32Array(9));
  },

  clone(a: Mat3): Mat3 {
    const out = new Float32Array(9);
    out.set(a);
    return brandMat3(out);
  },

  copy(out: Mat3, a: Mat3): Mat3 {
    out.set(a);
    return out;
  },

  identity(out: Mat3): Mat3 {
    out[0] = 1; out[1] = 0; out[2] = 0;
    out[3] = 0; out[4] = 1; out[5] = 0;
    out[6] = 0; out[7] = 0; out[8] = 1;
    return out;
  },

  fromQuat(out: Mat3, q: Quat): Mat3 {
    const x = q[0], y = q[1], z = q[2], w = q[3];
    const x2 = x + x, y2 = y + y, z2 = z + z;
    const xx = x * x2, xy = x * y2, xz = x * z2;
    const yy = y * y2, yz = y * z2, zz = z * z2;
    const wx = w * x2, wy = w * y2, wz = w * z2;

    out[0] = 1 - (yy + zz);
    out[1] = xy + wz;
    out[2] = xz - wy;

    out[3] = xy - wz;
    out[4] = 1 - (xx + zz);
    out[5] = yz + wx;

    out[6] = xz + wy;
    out[7] = yz - wx;
    out[8] = 1 - (xx + yy);
    return out;
  },

  fromQuatTransposed(out: Mat3, q: Quat): Mat3 {
    // Transpose of fromQuat - useful for world←body vs body←world
    const x = q[0], y = q[1], z = q[2], w = q[3];
    const x2 = x + x, y2 = y + y, z2 = z + z;
    const xx = x * x2, xy = x * y2, xz = x * z2;
    const yy = y * y2, yz = y * z2, zz = z * z2;
    const wx = w * x2, wy = w * y2, wz = w * z2;

    out[0] = 1 - (yy + zz);
    out[3] = xy + wz;
    out[6] = xz - wy;

    out[1] = xy - wz;
    out[4] = 1 - (xx + zz);
    out[7] = yz + wx;

    out[2] = xz + wy;
    out[5] = yz - wx;
    out[8] = 1 - (xx + yy);
    return out;
  },

  multiply(out: Mat3, a: Mat3, b: Mat3): Mat3 {
    const a00 = a[0], a01 = a[1], a02 = a[2];
    const a10 = a[3], a11 = a[4], a12 = a[5];
    const a20 = a[6], a21 = a[7], a22 = a[8];

    const b00 = b[0], b01 = b[1], b02 = b[2];
    const b10 = b[3], b11 = b[4], b12 = b[5];
    const b20 = b[6], b21 = b[7], b22 = b[8];

    out[0] = a00 * b00 + a01 * b10 + a02 * b20;
    out[1] = a00 * b01 + a01 * b11 + a02 * b21;
    out[2] = a00 * b02 + a01 * b12 + a02 * b22;

    out[3] = a10 * b00 + a11 * b10 + a12 * b20;
    out[4] = a10 * b01 + a11 * b11 + a12 * b21;
    out[5] = a10 * b02 + a11 * b12 + a12 * b22;

    out[6] = a20 * b00 + a21 * b10 + a22 * b20;
    out[7] = a20 * b01 + a21 * b11 + a22 * b21;
    out[8] = a20 * b02 + a21 * b12 + a22 * b22;
    return out;
  },

  transpose(out: Mat3, a: Mat3): Mat3 {
    out[0] = a[0]; out[1] = a[3]; out[2] = a[6];
    out[3] = a[1]; out[4] = a[4]; out[5] = a[7];
    out[6] = a[2]; out[7] = a[5]; out[8] = a[8];
    return out;
  },

  invert(out: Mat3, a: Mat3): Mat3 {
    const a00 = a[0], a01 = a[1], a02 = a[2];
    const a10 = a[3], a11 = a[4], a12 = a[5];
    const a20 = a[6], a21 = a[7], a22 = a[8];

    const b01 = a22 * a11 - a12 * a21;
    const b11 = -a22 * a01 + a02 * a21;
    const b21 = a12 * a01 - a02 * a11;

    const det = a00 * b01 + a10 * b11 + a20 * b21;

    if (Math.abs(det) < 1e-10) return Mat3.identity(out);

    const invDet = 1 / det;

    out[0] = b01 * invDet;
    out[1] = b11 * invDet;
    out[2] = b21 * invDet;
    out[3] = (-a22 * a10 + a12 * a20) * invDet;
    out[4] = (a22 * a00 - a02 * a20) * invDet;
    out[5] = (-a12 * a00 + a02 * a10) * invDet;
    out[6] = (a21 * a10 - a11 * a20) * invDet;
    out[7] = (-a21 * a00 + a01 * a20) * invDet;
    out[8] = (a11 * a00 - a01 * a10) * invDet;
    return out;
  },

  // Multiply vector (column vector convention: out = M * v)
  multiplyVec3(out: Vec3, m: Mat3, v: Vec3): Vec3 {
    const x = v[0], y = v[1], z = v[2];
    out[0] = m[0] * x + m[3] * y + m[6] * z;
    out[1] = m[1] * x + m[4] * y + m[7] * z;
    out[2] = m[2] * x + m[5] * y + m[8] * z;
    return out;
  },

  // Multiply vector transposed (row vector: out = v^T * M)
  multiplyVec3Transposed(out: Vec3, m: Mat3, v: Vec3): Vec3 {
    const x = v[0], y = v[1], z = v[2];
    out[0] = m[0] * x + m[1] * y + m[2] * z;
    out[1] = m[3] * x + m[4] * y + m[5] * z;
    out[2] = m[6] * x + m[7] * y + m[8] * z;
    return out;
  },

  // Scale
  scale(out: Mat3, a: Mat3, s: number): Mat3 {
    for (let i = 0; i < 9; i++) out[i] = a[i] * s;
    return out;
  },

  // Add
  add(out: Mat3, a: Mat3, b: Mat3): Mat3 {
    for (let i = 0; i < 9; i++) out[i] = a[i] + b[i];
    return out;
  },

  // Determinant
  determinant(a: Mat3): number {
    return a[0] * (a[4] * a[8] - a[5] * a[7])
         - a[1] * (a[3] * a[8] - a[5] * a[6])
         + a[2] * (a[3] * a[7] - a[4] * a[6]);
  },

  // Frobenius norm
  frob(a: Mat3): number {
    let sum = 0;
    for (let i = 0; i < 9; i++) sum += a[i] * a[i];
    return Math.sqrt(sum);
  },

  IDENTITY: brandMat3(new Float32Array([1,0,0, 0,1,0, 0,0,1])),
  ZERO: brandMat3(new Float32Array(9)),
};

// ============================================================================
// Mat4 Operations (16-element Float32Array, column-major)
// ============================================================================

export const Mat4 = {
  create(): Mat4 {
    return brandMat4(new Float32Array(16));
  },

  clone(a: Mat4): Mat4 {
    const out = new Float32Array(16);
    out.set(a);
    return brandMat4(out);
  },

  copy(out: Mat4, a: Mat4): Mat4 {
    out.set(a);
    return out;
  },

  identity(out: Mat4): Mat4 {
    out[0] = 1; out[1] = 0; out[2] = 0; out[3] = 0;
    out[4] = 0; out[5] = 1; out[6] = 0; out[7] = 0;
    out[8] = 0; out[9] = 0; out[10] = 1; out[11] = 0;
    out[12] = 0; out[13] = 0; out[14] = 0; out[15] = 1;
    return out;
  },

  fromQuat(out: Mat4, q: Quat): Mat4 {
    Mat3.fromQuat(out, q);
    out[3] = 0; out[7] = 0; out[11] = 0;
    out[12] = 0; out[13] = 0; out[14] = 0; out[15] = 1;
    return out;
  },

  fromRotationTranslation(out: Mat4, q: Quat, v: Vec3): Mat4 {
    Mat3.fromQuat(out, q);
    out[3] = 0; out[7] = 0; out[11] = 0;
    out[12] = v[0]; out[13] = v[1]; out[14] = v[2]; out[15] = 1;
    return out;
  },

  fromTranslation(out: Mat4, v: Vec3): Mat4 {
    Mat4.identity(out);
    out[12] = v[0]; out[13] = v[1]; out[14] = v[2];
    return out;
  },

  fromScaling(out: Mat4, v: Vec3): Mat4 {
    out[0] = v[0]; out[1] = 0; out[2] = 0; out[3] = 0;
    out[4] = 0; out[5] = v[1]; out[6] = 0; out[7] = 0;
    out[8] = 0; out[9] = 0; out[10] = v[2]; out[11] = 0;
    out[12] = 0; out[13] = 0; out[14] = 0; out[15] = 1;
    return out;
  },

  perspective(out: Mat4, fovy: number, aspect: number, near: number, far: number): Mat4 {
    const f = 1 / Math.tan(fovy / 2);
    out[0] = f / aspect; out[1] = 0; out[2] = 0; out[3] = 0;
    out[4] = 0; out[5] = f; out[6] = 0; out[7] = 0;
    out[8] = 0; out[9] = 0; out[10] = (far + near) / (near - far); out[11] = -1;
    out[12] = 0; out[13] = 0; out[14] = (2 * far * near) / (near - far); out[15] = 0;
    return out;
  },

  perspectiveInfinite(out: Mat4, fovy: number, aspect: number, near: number): Mat4 {
    const f = 1 / Math.tan(fovy / 2);
    out[0] = f / aspect; out[1] = 0; out[2] = 0; out[3] = 0;
    out[4] = 0; out[5] = f; out[6] = 0; out[7] = 0;
    out[8] = 0; out[9] = 0; out[10] = -1; out[11] = -1;
    out[12] = 0; out[13] = 0; out[14] = -2 * near; out[15] = 0;
    return out;
  },

  ortho(out: Mat4, left: number, right: number, bottom: number, top: number, near: number, far: number): Mat4 {
    const lr = 1 / (left - right);
    const bt = 1 / (bottom - top);
    const nf = 1 / (near - far);
    out[0] = -2 * lr; out[1] = 0; out[2] = 0; out[3] = 0;
    out[4] = 0; out[5] = -2 * bt; out[6] = 0; out[7] = 0;
    out[8] = 0; out[9] = 0; out[10] = 2 * nf; out[11] = 0;
    out[12] = (left + right) * lr; out[13] = (top + bottom) * bt; out[14] = (far + near) * nf; out[15] = 1;
    return out;
  },

  lookAt(out: Mat4, eye: Vec3, center: Vec3, up: Vec3): Mat4 {
    const f = Vec3.create(); Vec3.sub(f, center, eye); Vec3.normalize(f, f);
    const s = Vec3.create(); Vec3.cross(s, f, up); Vec3.normalize(s, s);
    const u = Vec3.create(); Vec3.cross(u, s, f);

    out[0] = s[0]; out[1] = u[0]; out[2] = -f[0]; out[3] = 0;
    out[4] = s[1]; out[5] = u[1]; out[6] = -f[1]; out[7] = 0;
    out[8] = s[2]; out[9] = u[2]; out[10] = -f[2]; out[11] = 0;
    out[12] = -Vec3.dot(s, eye);
    out[13] = -Vec3.dot(u, eye);
    out[14] = Vec3.dot(f, eye);
    out[15] = 1;
    return out;
  },

  multiply(out: Mat4, a: Mat4, b: Mat4): Mat4 {
    const a00 = a[0], a01 = a[1], a02 = a[2], a03 = a[3];
    const a10 = a[4], a11 = a[5], a12 = a[6], a13 = a[7];
    const a20 = a[8], a21 = a[9], a22 = a[10], a23 = a[11];
    const a30 = a[12], a31 = a[13], a32 = a[14], a33 = a[15];

    const b00 = b[0], b01 = b[1], b02 = b[2], b03 = b[3];
    const b10 = b[4], b11 = b[5], b12 = b[6], b13 = b[7];
    const b20 = b[8], b21 = b[9], b22 = b[10], b23 = b[11];
    const b30 = b[12], b31 = b[13], b32 = b[14], b33 = b[15];

    out[0] = a00*b00 + a01*b10 + a02*b20 + a03*b30;
    out[1] = a00*b01 + a01*b11 + a02*b21 + a03*b31;
    out[2] = a00*b02 + a01*b12 + a02*b22 + a03*b32;
    out[3] = a00*b03 + a01*b13 + a02*b23 + a03*b33;

    out[4] = a10*b00 + a11*b10 + a12*b20 + a13*b30;
    out[5] = a10*b01 + a11*b11 + a12*b21 + a13*b31;
    out[6] = a10*b02 + a11*b12 + a12*b22 + a13*b32;
    out[7] = a10*b03 + a11*b13 + a12*b23 + a13*b33;

    out[8] = a20*b00 + a21*b10 + a22*b20 + a23*b30;
    out[9] = a20*b01 + a21*b11 + a22*b21 + a23*b31;
    out[10] = a20*b02 + a21*b12 + a22*b22 + a23*b32;
    out[11] = a20*b03 + a21*b13 + a22*b23 + a23*b33;

    out[12] = a30*b00 + a31*b10 + a32*b20 + a33*b30;
    out[13] = a30*b01 + a31*b11 + a32*b21 + a33*b31;
    out[14] = a30*b02 + a31*b12 + a32*b22 + a33*b32;
    out[15] = a30*b03 + a31*b13 + a32*b23 + a33*b33;
    return out;
  },

  invert(out: Mat4, a: Mat4): Mat4 {
    const a00 = a[0], a01 = a[1], a02 = a[2], a03 = a[3];
    const a10 = a[4], a11 = a[5], a12 = a[6], a13 = a[7];
    const a20 = a[8], a21 = a[9], a22 = a[10], a23 = a[11];
    const a30 = a[12], a31 = a[13], a32 = a[14], a33 = a[15];

    const b00 = a00 * a11 - a01 * a10;
    const b01 = a00 * a12 - a02 * a10;
    const b02 = a00 * a13 - a03 * a10;
    const b03 = a01 * a12 - a02 * a11;
    const b04 = a01 * a13 - a03 * a11;
    const b05 = a02 * a13 - a03 * a12;
    const b06 = a20 * a31 - a21 * a30;
    const b07 = a20 * a32 - a22 * a30;
    const b08 = a20 * a33 - a23 * a30;
    const b09 = a21 * a32 - a22 * a31;
    const b10 = a21 * a33 - a23 * a31;
    const b11 = a22 * a33 - a23 * a32;

    const det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;

    if (Math.abs(det) < 1e-10) return Mat4.identity(out);

    const invDet = 1 / det;

    out[0] = (a11 * b11 - a12 * b10 + a13 * b09) * invDet;
    out[1] = (a02 * b10 - a01 * b11 - a03 * b09) * invDet;
    out[2] = (a31 * b05 - a32 * b04 + a33 * b03) * invDet;
    out[3] = (a22 * b04 - a21 * b05 - a23 * b03) * invDet;

    out[4] = (a12 * b08 - a10 * b11 - a13 * b07) * invDet;
    out[5] = (a00 * b11 - a02 * b08 + a03 * b07) * invDet;
    out[6] = (a32 * b02 - a30 * b05 - a33 * b01) * invDet;
    out[7] = (a20 * b05 - a22 * b02 + a23 * b01) * invDet;

    out[8] = (a10 * b10 - a11 * b08 + a13 * b06) * invDet;
    out[9] = (a01 * b08 - a00 * b10 - a03 * b06) * invDet;
    out[10] = (a30 * b04 - a31 * b02 + a33 * b00) * invDet;
    out[11] = (a21 * b02 - a20 * b04 - a23 * b00) * invDet;

    out[12] = (a11 * b07 - a10 * b09 - a12 * b06) * invDet;
    out[13] = (a00 * b09 - a01 * b07 + a02 * b06) * invDet;
    out[14] = (a31 * b01 - a30 * b03 - a32 * b00) * invDet;
    out[15] = (a20 * b03 - a21 * b01 + a22 * b00) * invDet;
    return out;
  },

  transpose(out: Mat4, a: Mat4): Mat4 {
    out[0] = a[0]; out[1] = a[4]; out[2] = a[8]; out[3] = a[12];
    out[4] = a[1]; out[5] = a[5]; out[6] = a[9]; out[7] = a[13];
    out[8] = a[2]; out[9] = a[6]; out[10] = a[10]; out[11] = a[14];
    out[12] = a[3]; out[13] = a[7]; out[14] = a[11]; out[15] = a[15];
    return out;
  },

  multiplyVec3(out: Vec3, m: Mat4, v: Vec3): Vec3 {
    const x = v[0], y = v[1], z = v[2];
    const w = m[3] * x + m[7] * y + m[11] * z + m[15];
    out[0] = (m[0] * x + m[4] * y + m[8] * z + m[12]) / w;
    out[1] = (m[1] * x + m[5] * y + m[9] * z + m[13]) / w;
    out[2] = (m[2] * x + m[6] * y + m[10] * z + m[14]) / w;
    return out;
  },

  multiplyVec4(out: Float32Array, m: Mat4, v: Float32Array): Float32Array {
    const x = v[0], y = v[1], z = v[2], w = v[3];
    out[0] = m[0]*x + m[4]*y + m[8]*z + m[12]*w;
    out[1] = m[1]*x + m[5]*y + m[9]*z + m[13]*w;
    out[2] = m[2]*x + m[6]*y + m[10]*z + m[14]*w;
    out[3] = m[3]*x + m[7]*y + m[11]*z + m[15]*w;
    return out;
  },

  IDENTITY: brandMat4(new Float32Array([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1])),
  ZERO: brandMat4(new Float32Array(16)),
};

// ============================================================================
// Random Number Generation (deterministic, seedable)
// ============================================================================

export class Random {
  private state: number;

  constructor(seed: number = Date.now()) {
    this.state = seed >>> 0;
  }

  // xorshift32
  next(): number {
    this.state ^= this.state << 13;
    this.state ^= this.state >>> 17;
    this.state ^= this.state << 5;
    return (this.state >>> 0) / 0x100000000;
  }

  nextInt(max: number): number {
    return Math.floor(this.next() * max);
  }

  nextRange(min: number, max: number): number {
    return min + this.next() * (max - min);
  }

  nextNormal(mean: number = 0, stdDev: number = 1): number {
    // Box-Muller transform
    const u1 = this.next();
    const u2 = this.next();
    const z = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    return mean + z * stdDev;
  }

  nextVec3(out: Vec3, scale: number = 1): Vec3 {
    out[0] = this.nextNormal(0, scale);
    out[1] = this.nextNormal(0, scale);
    out[2] = this.nextNormal(0, scale);
    return out;
  }

  nextQuat(out: Quat): Quat {
    // Uniform random quaternion (Hopf coordinates)
    const u1 = this.next();
    const u2 = this.next();
    const u3 = this.next();
    const r1 = Math.sqrt(1 - u1);
    const r2 = Math.sqrt(u1);
    const t1 = 2 * Math.PI * u2;
    const t2 = 2 * Math.PI * u3;
    out[0] = r1 * Math.sin(t1);
    out[1] = r1 * Math.cos(t1);
    out[2] = r2 * Math.sin(t2);
    out[3] = r2 * Math.cos(t2);
    return out;
  }

  setSeed(seed: number): void {
    this.state = seed >>> 0;
  }

  getSeed(): number {
    return this.state;
  }

  clone(): Random {
    const r = new Random(0);
    r.state = this.state;
    return r;
  }
}

// Global default instance
export const random = new Random();

// ============================================================================
// Constants
// ============================================================================

export const EPSILON = 1e-6;
export const PI = Math.PI;
export const TAU = Math.PI * 2;
export const DEG2RAD = Math.PI / 180;
export const RAD2DEG = 180 / Math.PI;
export const GRAVITY = 9.80665; // m/s²
export const AIR_DENSITY = 1.225; // kg/m³ at sea level