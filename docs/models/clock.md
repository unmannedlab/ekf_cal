Clock Error Models {#clock_model}
============

The time error model used is
\f{align}{
    t_m = \alpha t + \beta + n_t
\f}
where
- \f$t_m\f$    is the measured time,
- \f$\alpha\f$ is the time skew factor,
- \f$t\f$      is the true time,
- \f$\beta\f$  is the time offset, and
- \f$n_t\f$    is a Gaussian, white noise process
