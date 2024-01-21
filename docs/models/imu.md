IMU Measurement Models {#imu_model}
============

The IMU error model used is
\f{align}{
    a_m =
    \omega \times \omega \times p_a +
    \alpha \times p_a +
    \mathcal{C}\left(q_i^b\right)
    \left[a + g\right]
\f}
where
- \f$a_m\f$            is the measured acceleration,
- \f$\omega\f$         is the true angular velocity,
- \f$p_a\f$            is the true position of the accelerometer in the body frame,
- \f$\alpha\f$         is the true angular acceleration,
- \f$\mathcal{C}\f$    is the quaternion to rotation matrix function,
- \f$q_i^b\f$          is the rotation from the body frame to the IMU frame,
- \f$a_b\f$            is the true body acceleration, and
- \f$g\f$              is the gravity vector.
