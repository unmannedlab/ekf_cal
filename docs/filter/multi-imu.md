Multi-IMU Filtering {#multi-imu}
============

\f{align}{
\boldsymbol{h}(\boldsymbol{x}_{b}) = 
\begin{bmatrix}
    \mathcal{C}(\prescript{B}{I_i}{q})^T
    \left(
    \mathcal{C}(\prescript{G}{B}{q})^T \boldsymbol{a} +
    \boldsymbol{\alpha} \times \prescript{B}{}{\boldsymbol{p}}_{I_i} +
    \boldsymbol{\omega} \times \boldsymbol{\omega} \times \prescript{B}{}{\boldsymbol{p}}_{I_i}
    \right)
    \\
    \mathcal{C}(\prescript{B}{I_i}{q})^T
    \mathcal{C}(\prescript{G}{B}{q})^T
    \boldsymbol{\omega}
\end{bmatrix}
\f}
