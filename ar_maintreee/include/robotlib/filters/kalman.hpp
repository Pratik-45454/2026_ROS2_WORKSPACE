

#ifndef KALMAN_HPP__
#define KALMAN_HPP__

/*
    X = state vector (estimate)
    W = preticted state noise vector
    U = control/input vector vx,vy,vz

    P = process covariance matrix (error in estimate)
    Q = process noise covariance matrix

    K = kalman gain
    R = sensor noise covariance matrix (measurement error)

    Y = actual measurement of state
    Z = measurement noise (due to sensor delay or instrumental error)

    A = State transition matrix
    B = Control transiton matrix
    C = measurement transition matrix
    H = Observation/measurement matrix depends on sensor data structure
    I = identity matrix


    X = A * X + B * U + W
    P = A * P * A_T + Q

    Y = C * Y + Z
    K = P * H / (H * P * H_T + R)   [divide means inverse from right side]
    X = X + K * (Y - H * X)         [update with measurement]
    P = (I - K * H) * P
*/

/* For 3 variable with zero noise
Sx = Sx + Vx*t + 0.5*Ax*t^2
Sy = Sy + Vy*t + 0.5*Ay*t^2
Sa = Sa + Va*t + 0.5*Aa*t^2 ; (a) stands for angular

Similarly
Vx = Vx + Ax*t
....

[Sx]   [1 0 0 t 0 0] [Sx]   [0.5*t^2  0        0      ]
[Sy]   [0 1 0 0 t 0] [Sy]   [0        0.5*t^2  0      ] [Ax]
[Sa] = [0 0 1 0 0 t] [Sa] + [0        0        0.5*t^2] [Ay]
[Vx]   [0 0 0 1 0 0] [Vx]   [t        0        0      ] [Aa]
[Vy]   [0 0 0 0 1 0] [Vy]   [0        t        0      ]
[Va]   [0 0 0 0 0 1] [Va]   [0        0        t      ]
*/

/*
    deviation matrix = matrix - [1] * matrix
    covariance matrix = transpose of deviation matrix * deviation matrix
*/

#include <stdio.h>
#include "../maths/matrixf32.hpp"

template <uint8_t stateNum, uint8_t controlNum, uint8_t measurementNum>
struct Kalman_InitStruct
{
    float32_t A[stateNum * stateNum];               // State transition matrix
    float32_t B[stateNum * controlNum];             // Control input transition matrix
    float32_t w[stateNum * 1];                      // state noise vector
    float32_t Q[stateNum * stateNum];               // Process noise covariance matrix
    float32_t H[measurementNum * stateNum];         // Measurement matrix
    float32_t R[measurementNum * measurementNum];   // Measurement noise covariance matrix
    float32_t z[measurementNum];                    // Measurement noise vector
    float32_t P[stateNum * stateNum]; // Process covariance matrix
    float32_t x[stateNum * 1];                      // state vector
    float32_t u[controlNum * 1];                    // control input vector
    float32_t dT;                                   // time difference between two calculation
};

template <uint8_t stateNum, uint8_t controlNum, uint8_t measurementNum>
class Kalman
{
private:
    const Matrix32<stateNum, stateNum> A;             // State transition matrix
    const Matrix32<stateNum, controlNum> B;           // Control input transition matrix
    const Matrix32<stateNum, 1> w;                    // State noise vector
    const Matrix32<stateNum, stateNum> Q;             // Process noise covariance matrix
    const Matrix32<measurementNum, stateNum> H;       // Measurement matrix
    const Matrix32<measurementNum, measurementNum> R; // Measurement noise covariance matrix
    const Matrix32<measurementNum, 1> Z;              // Measurement noise vector
    Matrix32<stateNum, stateNum> P;                   // Process covariance matrix
    Matrix32<stateNum, stateNum> K;                   // Kalman gain matrix
    Matrix32<stateNum, stateNum> I;                   // Identity matrix

public:
    Matrix32<stateNum, 1> x;       // state vector
    Matrix32<controlNum, 1> u;     // control input vector
    Matrix32<measurementNum, 1> y; // measurement vector

    float32_t dT; // time difference between two measurement

    /* We don't need default contructor */
    Kalman() = delete;

    /* Initialize:
     *  transition matrices,
     *  covariance matrices,
     *  initial control input
     *  measurement matrix
     *  identity matrix
     *  initial state
     */
    Kalman(Kalman_InitStruct<stateNum, controlNum> initial)
        : A(initial.A), B(initial.B), Q(initial.Q), H(initial.H), R(initial.R), P(initial.P), x(initial.x), u(initial.u), dT(initial.dT)
    {
        I.setIdentity();
    }

    /* Compute and update */
    void update()
    {
        /* Predict state vector */
        x = A * x + B * u + w;

        /* Predict state covariance matrix */
        P = A * P * A.trans() + Q;

        /* Calculate kalman gain */
        Matrix32<stateNum, stateNum> K_denom = H * P * H.trans() + R;
        K = P * H * K_denom.inverse();

        y = C * y + z;

        /* Update estimate */
        x = x + K * (y - H * x);

        /* Update covariance matrix */
        P = (I - K * H) * P;
    }

    /* As usual, but need not to do explicitly */
    ~Kalman() = default;
};

#endif
