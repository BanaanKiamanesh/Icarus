# PID Controller

A Proportional-Integral-Derivative (PID) controller is a widely used control strategy in engineering. It works by continuously measuring the error between a desired setpoint and the actual process variable, and then adjusting the control output to minimize this error.

The PID controller consists of three terms: the proportional term, the integral term, and the derivative term. The control output is the sum of these three terms:

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

where:

- $u(t)$ is the control output at time $t$
- $e(t)$ is the error at time $t$, which is the difference between the setpoint and the process variable
- $K_p$, $K_i$, and $K_d$ are the proportional, integral, and derivative gains, respectively

## Proportional Term

The proportional term is proportional to the current error. It provides an immediate response to any changes in the error and can help to reduce overshoot and settling time. However, it can also cause steady-state error if the gain is too low.

The proportional term is given by:

$$K_p e(t)$$

where:

- $K_p$ is the proportional gain

## Integral Term

The integral term is proportional to the accumulated error over time. It helps to eliminate steady-state error by continuously adjusting the control output. However, it can also cause overshoot and instability if the gain is too high.

The integral term is given by:

$$K_i \int_0^t e(\tau) d\tau$$

where:

- $K_i$ is the integral gain

## Derivative Term

The derivative term is proportional to the rate of change of the error. It helps to reduce overshoot and settling time by anticipating changes in the error. However, it can also amplify noise and cause instability if the gain is too high. Inorder to overcome this problem, a low-pass filter is often used to smooth out the derivative term.

The derivative term is given by:

$$K_d \frac{de(t)}{dt}$$

where:

- $K_d$ is the derivative gain

## Tuning

The gains of the PID controller must be tuned to achieve the desired control performance. This can be done using various methods, such as Ziegler-Nichols method, trial-and-error, or optimization algorithms. When the dynamics of the system is coupled, the tuning of the controller becomes more complex. In this case, using th optimization algorithms is the best option.

The proportional gain affects the response speed and stability of the system. A higher gain results in a faster response but can also cause instability.

The integral gain affects the steady-state error and response time of the system. A higher gain reduces steady-state error but can also cause overshoot and instability.

The derivative gain affects the damping of the system. A higher gain reduces overshoot and settling time but can also amplify noise and cause instability.
