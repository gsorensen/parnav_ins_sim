
# Tuning parameters

This document outlines the various tuning parameters used when generating
measurements and tuning the ESKF-based and FGO-based estimators.

## Sensor parameters for measurement generation and estimator tuning

| Component                | Parameter                             | Value                                          | Unit        |
| ------------------------ | ------------------------------------- | ---------------------------------------------- | ----------- |
| GNSS position            | N                                     | 1.5                                            | m           |
|                          | E                                     | 1.5                                            | m           |
|                          | D                                     | 3.0                                            | m           |
| GNSS Heading (lever arm) | N, E, D                               | 0.05                                           | m           |
| PARS                     | Range                                 | 5.0                                            | m           |
|                          | Azimuth                               | 7.0                                            | deg         |
|                          | Elevation                             | 7.0                                            | deg         |
| IMU                      | VRW                                   | 0.07                                           | m/s/sqrt(h) |
|                          | ARW                                   | 0.15                                           | deg/sqrt(h) |
|                          | Accelerometer bias instability        | 0.05                                           | milli g     |
|                          | Gyro bias instability                 | 0.5                                            | deg/h       |
|                          | Accelerometer/gyro bias time constant | 3600                                           | s           |
|                          | True accelerometer bias               | \[ <br>-0.276839, -0.244186,<br>0.337360<br>\] | m/s^2       |
|                          | True gyro bias                        | \[ <br>-0.0028,<br> 0.0021,<br>-0.0032<br>\]   | deg/s       |

## FGO-estimator specific tuning parameters

| Component                           | Value | Unit |
| ----------------------------------- | ----- | ---- |
| Fixed-lag smoothing time horizon    | 10    | s    |
| Fixed-lag relinearisation threshold | 0.001 |      |
| Preintegration error cov            | 1e-10 | m    |

## Initial estimates

The initial state covariance estimate was obtained from the square of the magnitude values below. The initial state estimates for the position, velocity, and attitude used the same magnitude with a 3D-vector of `randn` in Matlab so that the estimate has the same error magnitude, but along different axes. The bias estimates were zero-initialised.

| Magnitude          | Value | Unit  |
| ------------------ | ----- | ----- |
| Position           | 5     | m     |
| Velocity           | 1     | m/s   |
| Attitude           | 5     | deg   |
| Accelerometer bias | 0.2   | m/s^2 |
| Gyroscope bias     | 0.1   | deg/s |
