# KalmanFilter
Kalman filter library to provide state updates with asynchronous measurement updates. When supply a measurement, the user also gives the nonlinear equation relating the state to the measurement and the covariance of that measurement. The framework will linearize the nonlinear functions and concenate the measurement matrices according to the available information. State updates can then be retreived at any frequency.

This was tested with a MEKF to estimate attitude using rotation matrices.
