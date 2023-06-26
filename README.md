# **Project: Icarus**

## Dynamical Modeling and Control of a QuadCopter

---

## Abstract

**This Project Consists of 3 phases:**

1. Phase 1: Mathematical Modelling of the QuadCopter and Desiging of a Linear PID controller to control all 6 DoF of the QuadCopter Considering IMU Noise.
2. Phase 2: Simulating a flipping maneuver using the extracted Mathematical Model.
3. Phase 3: Designing a Non-Linear Sliding-Mode controller to track a desired trajectory under the influence of external disturbances and IMU noises.

> ***Note:** The whole simulation process is done in MATLAB. And every algorithm is implemented from scratch.*

---

## 0. Introduction

The QuadCopter is a popular choice for aerial robotics research and applications due to its highly maneuverable capabilities, including vertical takeoff and landing, hovering, and multidirectional flight. In addition to its agility, the QuadCopter is relatively simple to construct and maintain, making it a cost-effective option for research and development in the field of aerial robotics. However, despite its advantages, the system is inherently unstable, necessitating the development of a robust mathematical model and controller. This project aims to derive a mathematical model of the system and design a control system using control theory to stabilize the QuadCopter and enable precise trajectory tracking.

The QuadCopter is a versatile aerial robotics platform that offers exceptional maneuverability in flight. It is capable of executing complex movements such as vertical takeoff and landing, hovering, and multi-directional flight. The construction and maintenance of the QuadCopter are relatively straightforward, making it an affordable option for research and development in the field of aerial robotics. However, the system's inherent instability requires the development of a robust mathematical model and controller to ensure stable flight and precise trajectory tracking.

---

## 1. Phase 1 - Dynamical Modelling and Implemetation of a Linear PID Controller

The QuadCopter is a rigid body with six degrees of freedom in space, defined by its center of mass position and Euler angles. It is propelled by four rotors, each modeled as a point mass with thrust force acting in the positive z-direction. The system is subject to gravitational and aerodynamic forces, as well as environmental disturbances. The QuadCopter is assumed to be symmetric about the x and y axes for the sake of simplicity, and aerodynamic forces are assumed to be linearly proportional to the velocity of the QuadCopter. Additionally, the aerodynamic forces are negligible when compared to the thrust forces of the rotors. The system is assumed to be rigid, with rotors rigidly attached to the body of the QuadCopter. The QuadCopter is also assumed to be a point mass, with negligible rotor mass compared to the QuadCopter's mass.

This part consists of three main sections: kinematics, dynamics and control. The kinematics section describes the relationship between the angular velocity of the QuadCopter and the roll, pitch, and yaw angle rates. The dynamics section describes the forces and moments acting on the QuadCopter. Finally, the control section describes the design of a linear PID controller to control all six degrees of freedom of the QuadCopter.

### 1.1 Kinematics

The relation between the angular velocity of the quadcopter and the roll, pitch, and yaw angle rates can be expressed using the following equation(Diebel, 2006):

$$
\begin{aligned}
\omega &=
\begin{bmatrix}
1 & 0 & -\sin\theta \\ 0 & \cos\phi & \cos\theta \sin\phi \\ 0 & -\sin\phi & \cos\theta \cos\phi
\end{bmatrix} ^ {-1}
\begin{bmatrix} \dot{\phi} \\  \dot{\theta} \\ \dot{\psi}
\end{bmatrix} \\ &=
\begin{bmatrix}
1 & \sin\phi \tan\theta & \cos\phi \tan\theta \\ 0 & \cos\phi & -\sin\phi \\ 0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta}
\end{bmatrix}
\begin{bmatrix}
\dot{\phi} \\ \dot{\theta} \\ \dot{\psi}
\end{bmatrix}
\end{aligned}
$$

where:

- $\phi$, $\theta$, and $\psi$ are the roll, pitch, and yaw angles, respectively
- $\omega$ is the angular velocity of the quadcopter

> Note that the representation suffers from a sigularity when $ \theta = \pm \frac{\pi}{2} $, which is known as gimbal lock. This can be avoided by using quaternions to represent the attitude of the quadcopter.

The relation between the translational velocity of the quadcopter and the roll, pitch, and yaw angles can be expressed using the following equation:

$$
\begin{bmatrix}
\dot{x} \\ \dot{y} \\ \dot{z}
\end{bmatrix}
= R(\phi, \theta, \psi)
\begin{bmatrix} u \\ v \\ w
\end{bmatrix} \\ \Rightarrow
\begin{bmatrix}
\dot{x} \\ \dot{y} \\ \dot{z}
\end{bmatrix} =
\begin{bmatrix}
\cos\theta \cos\psi & \sin\phi \sin\theta \cos\psi - \cos\phi \sin\psi & \cos\phi \sin\theta \cos\psi + \sin\phi \sin\psi \\ \cos\theta \sin\psi & \sin\phi \sin\theta \sin\psi + \cos\phi \cos\psi & \cos\phi \sin\theta \sin\psi - \sin\phi \cos\psi \\ -\sin\theta & \sin\phi \cos\theta & \cos\phi \cos\theta
\end{bmatrix} \begin{bmatrix}
u \\ v \\ w
\end{bmatrix}
$$

where:

- $\dot{x}$, $\dot{y}$, and $\dot{z}$ are the translational velocities of the quadcopter in the inertial frame
- $u$, $v$, and $w$ are the velocities of the quadcopter in the body frame
- $R(\phi, \theta, \psi)$ is the ZYX rotation matrix corresponding to the roll, pitch, and yaw angles

---

### 1.2 Dynamics

The motion of a quadcopter can be described using six degrees of freedom: three translational (surge, sway, heave) and three rotational (roll, pitch, yaw). The forces and moments acting on the quadcopter can be modeled using the following equations.

#### 1.2.1 Translational Dynamics

There are three main forces acting on the quadcopter:

1. Thrust force generated by the rotors
2. Aerodynamic drag force
3. Gravitational force

The gravitational force is always directed downwards, while the thrust and drag forces are directed along the body axes of the quadcopter. The thrust and drag forces are functions of the angular velocity of the rotors, which is controlled by the flight controller. Using the Newton's equations, the translational motion of the quadcopter can be described by the following equations:

$$\begin{bmatrix} \ddot{x} \\ \ddot{y} \\ \ddot{z} \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ -g \end{bmatrix} + \frac{1}{m} R(\phi,\theta,\psi) \begin{bmatrix} 0 \\ 0 \\ T_{\Sigma} \end{bmatrix} + \frac{1}{m} F_{D} $$

where:

- $m$ is the mass of the quadcopter
- $g$ is the gravitational acceleration
- $x$, $y$, and $z$ are the positions of the quadcopter in the inertial frame
- $\phi$, $\theta$, and $\psi$ are the roll, pitch, and yaw angles, respectively
- $R(\phi,\theta,\psi)$ is the rotation matrix corresponding to the roll, pitch, and yaw angles
- $T_{\Sigma}$ is the total thrust generated by the four rotors

The drag force is usually modelled as a function of the translational velocity of the quadcopter:

$$ F_{D} = \begin{bmatrix} -D_{x} \dot{x}  \\ - D_{y} \dot{y} \\ - D_{z} \dot{z} \end{bmatrix}$$

where $D_{x}$, $D_{y}$, and $D_{z}$ are the drag coefficients in the $x$, $y$, and $z$ directions, respectively.

#### 1.2.2 Rotational Dynamics

Euler's equation for rotatinal motion:

$$ I \dot{\omega} + \omega \times (I\omega) = \tau \\
\bm{\Rightarrow} \dot{\omega} = \begin{bmatrix} \dot\omega_{x}  \\ \dot\omega_{y} \\ \dot\omega_{z} \end{bmatrix} = I^{-1} (\tau - \omega \times (I\omega)) $$

where:

- $I$ is the moment of inertia matrix
- $\omega$ is the angular velocity of the quadcopter
- $\tau$ is the total moment acting on the quadcopter

assuming that the quadcopter is symmetric about the $x$ and $y$ axes, the moment of inertia matrix will be diagonal and can be expressed as:

$$ I = \begin{bmatrix} I_{xx} & 0 & 0 \\ 0 & I_{yy} & 0 \\ 0 & 0 & I_{zz} \end{bmatrix} $$

where $I_{xx}$, $I_{yy}$, and $I_{zz}$ are the moments of inertia about the $x$, $y$, and $z$ axes, respectively. The quadcopter can be modelled as two thin rods of length $l$ connected at the center.

> ***Note 1:*** For the sake of simplicity the angular motion of the quadcopter is expressed in the body frame, translational motion are expressed in the inertial frame.
> ***Note 2:*** All the system parameter are better be identified using system identification techniques for precision. And all the system parameters are assumed to be constant.

---

### 1.3.1 Thrusts and Torques

Each motor generates a thrust and a torque. The thrust is directed along the $z$ axis of the body frame, while the torque is directed along the $x$ and $y$ axes of the body frame. Thrust force of each motor is modelled as a function of the square of the angular velocity of the motor:

$$ T_{i} = k \omega_{i}^{2} $$

where $k$ is a constant that depends on the propeller and motor characteristics.

Torque on each $x$, $y$ and $z$ axes are modelled as:

$$ \tau = \begin{bmatrix} \tau_{x}  \\ \tau_{y} \\ \tau_{z} \end{bmatrix} = \begin{bmatrix} l k (\omega_{2}^{2} - \omega_{4}^{2})  \\ l k (\omega_{3}^{2} - \omega_{1}^{2}) \\ b (\omega_{1}^{2} - \omega_{2}^{2} + \omega_{3}^{2} - \omega_{4}^{2}) \end{bmatrix} $$

The total thrust and torque generated by the rotors can be expressed as a single matrix equation:

$$ \begin{bmatrix} T_{\Sigma} \\ \tau_{x} \\ \tau_{y} \\ \tau_{z} \end{bmatrix} = \begin{bmatrix} k & k & k & k \\ 0 & -l k & 0 & l k \\ -l k & 0 & l k & 0 \\ b & -b & b & -b \end{bmatrix} \begin{bmatrix} \omega_{1}^{2} \\ \omega_{2}^{2} \\ \omega_{3}^{2} \\ \omega_{4}^{2} \end{bmatrix} $$

where:

- $k$ is a constant that depends on the propeller and motor characteristics
- $l$ is the distance between the center of the quadcopter and the center of each rotor
- $b$ is a constant that depends on the propeller and motor characteristics
- $\omega_{1}$, $\omega_{2}$, $\omega_{3}$, and $\omega_{4}$ are the angular velocities of the four rotors

---

### 1.3.2 Motor Dynamics

Each motor has a PWM pulse input which corresponds to the angular velocity of the motor. The relationship between the PWM pulse and the angular velocity can be expressed as quadratic function. Since, Icarus is a simulation project this relationship is not considered.

The Motors are modelled as first order systems with two time constants for the rise and fall in there speeds, since the motors are faster to speed up and slower to slow down. The motor dynamics can be expressed as:

$$ \dot{\omega}_{i} = \begin{cases} P_{up} (\omega_{des} - \omega_{i}) & \text{for } \omega_{des} > \omega_{i} \\ P_{down}(\omega_{des} - \omega_{i}) & \text{otherwise.} \end{cases} $$

where:

- $P_{up}$ and $P_{down}$ are the motor time constants
- $\omega_{des}$ is the desired angular velocity of the motor

Motors are faster to speed up than to slow down. To model this, we can define a $P_{up} \geq P_{down}$. To bypass motor dynamics, we may set $\omega_{des_{1..4}} = \omega_{i_{1..4}} $. To ignore motor saturation, we may set $\omega_{max} = -\omega_{min} = \infty$.

---

### 1.4 Controller

#### 1.4.1 QuadCopter Control Structure

At it's most basic level, to be able to control and move the quadcopter, there must be a control system to control the thrust vector's amplitude and direction. To obtain this goal the problem is divided into two sub-problems:

1. **Attitude Control:** To control the orientation of the quadcopter in the 3D space.
2. **Altitude Control:** To control the altitude of the quadcopter in the 3D space.

The attitude control is basically controlling the moments acting on the quadcopter, in order to control the orientation of the thrust vector. In a similar manner the altitude control is controlling the thrust vector's amplitude, in order to control the altitude of the quadcopter.

The feedback needed for the attitude control system is the orientation of the quadcopter in the 3D space. This can be obtained by using an IMU sensor. The feedback needed for the altitude control system is the altitude of the quadcopter in the 3D space. However the height feedback can not be constructed simply and there are several ways that are usually used to obtain it.

1. Using Barometer
2. Odometry Sensors Containing a Laser Range Finder and a Camera
3. Using a Motion Capture System
4. Using a GPS
5. Using Visual Localization Systems Like SLAM
6. ...

Although, as it is a simulation project, the altitude feedback is obtained by using the simulation environment and the sensor noise is simulated as a Gaussian noise $ \mathcal{N}(0, \sigma^{2}) $ with $ \sigma = 1 $ .

As the system is underactuated, all the 6 degrees of freedom can not be controlled seperately. To be able to control the position of the quadcopter the altitude control system is cascaded with the attitude control system. The altitude control system is the outer loop and the attitude control system is the inner loop.

The desired attitude controller inputs are calculated as:

$$ \begin{bmatrix}
\phi_{des} \\ \theta_{des}
\end{bmatrix} = m T_{\Sigma} \begin{bmatrix}
\sin(\psi) & -\cos(\psi) \\ \cos(\psi) & \sin(\psi)
\end{bmatrix}
\begin{bmatrix}
U_{ctrl}(x) \\ U_{ctrl}(y)
\end{bmatrix}
$$

---

#### 1.4.2 Continuous PID Controller Theory

A Proportional-Integral-Derivative (PID) controller is a widely used control strategy in engineering. It works by continuously measuring the error between a desired setpoint and the actual process variable, and then adjusting the control output to minimize the tracking error.

The PID controller consists of three terms: the proportional term, the integral term, and the derivative term. The control output is the sum of these three terms:

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

where:

- $u(t)$ is the control output at time $t$
- $e(t)$ is the error at time $t$, which is the difference between the setpoint and the process variable
- $K_p$, $K_i$, and $K_d$ are the proportional, integral, and derivative gains, respectively

#### 1.4.3 Discrete PID Controller

The PID controller can be discretized using the zero-order hold method. The control output is calculated at a fixed sampling time $T_s$. The integral term is approximated using the trapezoidal rule, and the derivative term is approximated using the backward difference formula. The discrete PID controller can be expressed as:

$$u[k] = K_p e[k] + K_i T_s \frac{e[k] + e[k-1]}{2} + K_d \frac{e[k] - e[k-1]}{T_s}$$

where:

- $u[k]$ is the control output at time $k$
- $e[k]$ is the error at time $k$
- $T_s$ is the sampling time
- $K_p$, $K_i$, and $K_d$ are the proportional, integral, and derivative gains, respectively

##### PID Control from an Engineering Prespective

There are multiple approaches to design a PID controller for the system stability and control. When analysing the mathematical model of the systems, the most common way is use analytical methods like root-locus, bode-plot, etc, Specially when the system is linear. However, when the system is non-linear, the analytical methods are not applicable. In this case, the most common way is to use the trial and error method. In this method, the PID gains are tuned manually until the desired response is achieved. This method is not efficient and can be time consuming. To overcome this problem, there are multiple methods to tune the PID gains automatically. The most common methods are:

1. **Proptional Term:** The proportional term is proportional to the current error. It provides an immediate response to any changes in the error and can help to reduce overshoot and settling time. However, it can also cause steady-state error if the gain is too low.
2. **Integral Term:** The integral term is proportional to the accumulated error over time. It helps to eliminate steady-state error by continuously adjusting the control output. However, it can also cause overshoot and instability if the gain is too high.
3. **Derivative Term:** The derivative term is proportional to the rate of change of the error. It helps to reduce overshoot and settling time by anticipating changes in the error. However, it can also amplify noise and cause instability if the gain is too high. Inorder to overcome this problem, a low-pass filter is often used to smooth out the derivative term.

##### Notes to consider when tuning the PID gains

> **Note 1:** The proportional gain affects the response speed and stability of the system. A higher gain results in a faster response but can also cause instability.
> **Note 2:** The integral gain affects the steady-state error of the system. A higher gain results in a smaller steady-state error but can also cause instability.
> **Note 3:** The derivative gain affects the overshoot and settling time of the system. A higher gain results in a smaller overshoot and settling time but can also cause instability.

#### 1.4.2 AutoTune PID Controller

The quadcopter is a complex non-linear coupled system that cannot be analyzed using traditional methods like linearization over equilibrium point and linear controller design methods. As a result, the trial and error method is commonly employed to tune the initial PID gains manually until the desired response is achieved. In order to minimize control effort and tracking error, a Bio-Geography based optimization algorithm is then used to fine-tune the gains with the given initial guesses.

When tuning the PID gains for a quadcopter, there are several important factors to consider. Firstly, the dynamics of the quadcopter consist of two control loops: the attitude control loop, which controls the roll, pitch, and yaw angles, and the altitude control loop, which controls the altitude of the quadcopter. Secondly, there are six states that need to be controlled, including the roll, pitch, and yaw angles, as well as the roll, pitch, and yaw angular velocities. This means that there are six PID controllers that need to be tuned. Thirdly, the attitude dynamics are not independent of position dynamics, meaning that the attitude controllers can be affected by the position controllers and vice versa. Finally, it is important to note that the position controller calculates the desired roll and pitch setpoint angles.

### 1.5 Results

---

## 2. Phase 2 - A Simple Learning Strategy for QuadCopter MultiFlips

In this phase, a simple learning strategy is implemented to make the quadcopter perform a flip maneuver. First, the process of $ N $ flips is formulated on a low-order 2D model as a 5 step sequence with 5 adjustable parameters. An optimization algorithm is used to find the optimal parameters based on a cost function that minimizes the difference between the starting point and the end point of the flip maneuver.

### 2.1 Low-Order 2D Model

For the purpose of this phase, a low-order 2D model is used to formulate the propblem and out of the plane dynamics are ignored and assumed to be stable. The 2D model is:

$$
\begin{cases}
m \ddot{z} = T_{\Sigma} \cos\theta - mg \\
m \ddot{x} = T_{\Sigma} \sin\theta \\
I_{yy} \ddot{\theta} = l \tau_{y}
\end{cases}
\Rightarrow
\begin{cases}
\ddot{z} = \frac{T_{\Sigma}}{m} \cos\theta - g \\
\ddot{x} = \frac{T_{\Sigma}}{m} \sin\theta \\
\ddot{\theta} = \frac{l}{I_{yy}} \tau_{y}
\end{cases}
$$

where:

- $m$ is the mass of the quadcopter
- $I_{yy}$ is the moment of inertia of the quadcopter about the $y$ axis
- $l$ is the arm length of the quadcopter

### 2.2 Parameterization of Flip Maneuver

The QuadCopter needs to execute a flip that results in a rotation offset of a multiple of $2\pi$ while keeping all other states unaltered. All the out of plane dynamics are ignored and assumed to be stable.

The initial and final state conditions of the multi-flip maneuver are:

$$
\begin{cases}
x_0 = x_f = 0 \\
z_0 = z_f = 0 \\
\dot{x}_0 = \dot{x}_f = 0 \\
\dot{z}_0 = \dot{z}_f = 0 \\
\theta_f = \theta_0 + 2\pi N
\end{cases}
$$

where $N$ is the number of flips.

To seek an almost time optimal control, a control envelope is defined. So, the control inputs can be seeked on the odge of the control envelope. A reduced order control envelope is defined as $[\underline{\beta}, \overline{\beta}]$ to account for modeling uncertainties and to reserve
some control authority for the on-board feedback controllers. The desired $T_{\Sigma}$ must be consistent with a slightly reduced range of accelerations. A convenient way to parameterize it is:

$$
U_{min} \le m \underline{\beta} \le U \le m \overline{\beta} \le U_{max}
$$

where $U_{min}$ and $U_{max}$ correspond to the minimum and maximum thrusts that the quadcopter can produce, respectively. If no control margin is reserved ($U_{min} = 0$ and $U_{max} = \infty$), $\overline{\beta}$ corresponds to the vehicles maximum acceleration at full thrust.

The maximum and minumum collective thrust can be formulated as:

$$
\begin{cases}
T_{\Sigma,upper} = m\overline{\beta} - \frac{2 \lvert \ddot{\theta}*{max}\rvert I*{yy}}{l} \\
T_{\Sigma,lower} = m\underline{\beta} + \frac{2 \lvert \ddot{\theta}*{max}\rvert I*{yy}}{l} \\
\lvert \ddot{\theta}*{max}\rvert \le l m (\overline{\beta} - \underline{\beta})/4I*{yy}
\end{cases}
$$

Consider that there is a $\dot{\theta}_{max}$ which is the maximum angular velocity that the quadcopter can achieve. This is a design parameter.

The model inputs are the collective thrust $U_{des}$ and the desired motion rates. Having the control envelope defined, the flip maneuver can be parameterized as a 5 step sequence where each step excepts a control action in the form of $\{U_{des}, \ddot{\theta}_{max}, t\}$ where $t$ is the duration of the step.

The 5 step sequence is:

1. **Acceleration**:Accelerate up at near-maximum collective acceleration while rotating slightly away.
2. **Start Rotate**: Use maximum differential thrust to achieve $\dot{\theta}_{max}$.
3. **Coast**: Hold $\dot{\theta}_{max}$ (use a low collective thrust command to prevent accelerating into ground).
4. **Stop Rotate**:  Maximum differential thrust to reach $\dot{\theta}$ slightly less than 0.
5. **Recovery**: Accelerate up with near-maximum collective thrust with a slight rotational acceleration to stop vertical descent and any remaining horizontal movement.

Of all the steps, there are 5 unknown parameters:

1. $U_1$ - collective acceleration during step 1.
2. $T_1$ - duration of step 1.
3. $T_3$ - duration of step 3 (coasting at $\dot{\theta}_{max}$).
4. $U_5$ - collective acceleration during step 5.
5. $T_5$ - duration of step 5.

At each step all the other parameters can be considered as known. For conciseness, a normalized mass distribution is variable is defined as:

$$ \alpha = 2 I_{yy} / (m l^2) $$

Now, the other parameters can be defined as:

$$
\begin{aligned}
\ddot{\theta}*{1} &= -(\overline{\beta}-U_1)/\alpha L\\
\ddot{\theta}*{2} = -\ddot{\theta}*{4} &=  (\overline{\beta} - \underline{\beta}) / 2\alpha L\\
U_2 = U_4 &= (\overline{\beta} + \underline{\beta}) / 2 \\
T_2 &= (\dot{\theta}*{max} - \ddot{\theta}*{1}T_1) / \ddot{\theta}*{2} \\
\ddot{\theta}*{3} &= 0 \\
U_3 &= \underline{\beta} \\
T_4 &= -(\dot{\theta}*{max} - \ddot{\theta}*{5}T_5) / \ddot{\theta}*{4} \\
\ddot{\theta}_{5} &= (\overline{\beta} - U_5) / \alpha L
\end{aligned}
$$

Now, the whole problem is formulated as an optimization problem to finding values of the unknown parameters to satisfy the objective function below:

$$
\begin{aligned}
J &= \sum_{i=1}^{6} T_i  \\
where,  \ T &=
\begin{bmatrix}
x_0 \\ z_0 \\ \dot{x}_0 \\ \dot{z}_0 \\ \theta_0 \\ \dot{\theta}_0
\end{bmatrix} -
\begin{bmatrix}
x_f \\ z_f \\ \dot{x}_f \\ \dot{z}_f \\ \theta_f \\ \dot{\theta}_f
\end{bmatrix}
\end{aligned}
$$

### 2.3 3D Generalization

As it is just a simulation, the 3D genralization is just a simple change in the cost function and setting all the outer-plane desired accelerations to zero($\ddot{\phi}*{des} = 0$, $\ddot{\psi}*{des} = 0$). However in a real world scenario, the 3D generalization would require a more complex control system.

The cost function is changed to:

$$
\begin{aligned}
X &= (x, y, z, \dot{x}, \dot{y}, \dot{z}, \phi, \theta, \psi, \dot{\phi}, \dot{\theta}, \dot{\psi}) \\
X_0 &= (x_0, y_0, z_0, \dot{x}_0, \dot{y}_0, \dot{z}_0, \phi_0, \theta_0, \psi_0, \dot{\phi}_0, \dot{\theta}_0, \dot{\psi}_0) \\
X_f &= (x_f, y_f, z_f, \dot{x}_f, \dot{y}_f, \dot{z}_f, \phi_f, \theta_f, \psi_f, \dot{\phi}_f, \dot{\theta}_f, \dot{\psi}_f)
\end{aligned}
$$

$$
T = X_0 - X_f \Rightarrow J = \sum_{i=1}^{12} T_i
$$

### 2.4 Results

---

## 3. Phase 3 - Dynamic Modelling and Implementation of a non-Linear Sliding Mode Controller

### 3.1 Sliding Mode Theory

Sliding mode control is a nonlinear control method that alters the dynamics of a nonlinear system by application of a discontinuous control signal that forces the system to "slide" along a cross-section of the system's normal behavior. This cross-section is called the "switching surface" or "switching hyperplane" or "slidng surface". The switching surface is a function of the system state and is defined such that the system trajectories always converge to it. The control signal is then designed to drive the system trajectories along the switching surface to the origin. The control signal is discontinuous because it is composed of two or more continuous signals, each active over a different region of the state space. The switching surface is designed such that the system trajectories are always driven toward the origin, regardless of the direction of the discontinuity in the control signal. This is the fundamental property of sliding mode control that makes it robust to system uncertainties and disturbances. The discontinuous control signal is also called a "chattering" control signal because it causes the system trajectories to "chatter" as they slide along the switching surface. The chattering is undesirable because it can cause high-frequency oscillations in the system states and control signal. The chattering can be reduced with several methods, including the use of a saturation function in the control signal, the use of a boundary layer around the switching surface, and the use of a high-gain control signal.

#### 3.1.1 Sliding Mode Control Design

For a typical nonlinear system of the form:

$$
\ddot{x} = f(x, \dot{x}, t) + g(x, \dot{x}, t)u + \omega_x
$$

where:

- $x$ is the state vector
- $u$ is the control input
- $f(x, \dot{x}, t)$ is the nonlinear system dynamics
- $g(x, \dot{x}, t)$ is the control input gain matrix
- $\omega_x$ is the system uncertainties and disturbances($\lvert \omega_x \rvert \le \kappa$)

The sliding mode control design is as follows:

1. Define the system error as:

    $$
    e = x_d - x
    $$

2. Define the sliding surface as:

    $$
    \begin{aligned}
    s &= \dot{e} + \lambda e \\
    \Rightarrow \dot{s} &= \ddot{e} + \lambda \dot{e} \\
    \Rightarrow \dot{s} &= \ddot{x}_d - \ddot{x} + \lambda \dot{e} \\
    \Rightarrow \dot{s} &= \ddot{x}_d - f(x, \dot{x}, t) - g(x, \dot{x}, t)u - \omega_x + \lambda \dot{e}
    \end{aligned}
    $$

    where $\lambda$ is a positive constant.

3. Define a lyaupnov function as:

    $$
    V = \frac{1}{2}s^2
    $$

    Taking the time derivative of $V$ and extracting the conditions to make it negative definite:

    $$
    \begin{aligned}
    \dot{V} &= s\dot{s} \le -\eta \lvert s \rvert \\
    \Rightarrow \dot{V} &= \dot{s}\le -\eta \ sign(s)
    \end{aligned}
    $$

    where $\eta$ is a positive constant.

4. Define the control input as:

    $$
    u = \frac{1}{g(x, \dot{x}, t)}(-f(x, \dot{x}, t) - \omega_x + \lambda \dot{e} - \eta \ sign(s))
    $$

5. Assuming that the system disturbances and uncertainties are bounded by $\kappa$, the control input can be written as:

    $$
    d_L \le \lvert \omega_x \rvert \le d_U \Rightarrow
    \begin{cases}
    d_1 = \frac{d_U - d_L}{2} \\
    d_2 = \frac{d_U + d_L}{2} \\
    \end{cases}
    \Rightarrow \omega_c = d_2 - d_1 \ sign(s) \\
    u = \frac{1}{g(x, \dot{x}, t)}(-f(x, \dot{x}, t) - \lambda \dot{e} - \eta \ sign(s) - \omega_c) \\
    $$

### 3.2 QuadCopter Dynamic Model

The quadcopter dynamical model is discussed in the previous phases. But, it is simpler to use the equations as second order differential equations and not in the state space representation. The equations are as follows:

$$
\begin{cases}
\begin{aligned}
\ddot{x} &= (\cos(\phi)\sin(\theta)\cos(\psi) + \sin(\phi)\sin(\psi)) \frac{l}{m} U_1 + \omega_x\\
\ddot{y} &= (\cos(\phi)\sin(\theta)\sin(\psi) - \sin(\phi)\cos(\psi)) \frac{l}{m} U_1 + \omega_y\\
\ddot{z} &= -g + (\cos(\phi)\cos(\theta)) \frac{l}{m} U_1 + \omega_z \\
\ddot{\phi} &= \dot{\theta}\dot{\psi}(\frac{I_{yy} - I_{zz}}{I_{xx}}) - \frac{I_{r}}{I_{xx}}\dot{\theta}\Omega + \frac{l}{I_{xx}}U_2 + \omega_\phi\\
\ddot{\theta} &= \dot{\phi}\dot{\psi}(\frac{I_{zz} - I_{xx}}{I_{yy}}) - \frac{I_{r}}{I_{xx}}\dot{\phi}\Omega + \frac{l}{I_{y}}U_3 + \omega_\theta\\
\ddot{\psi} &= \dot{\theta}\dot{\phi}(\frac{I_{xx} - I_{yy}}{I_{zz}}) + \frac{l}{I_{zz}}U_4 + \omega_\psi\\
\end{aligned}
\end{cases}
$$

> **Note:** Torques and forces are considered the same as discussed in the first phase.

For the sake of simplicity, the following constants are declared:

$$
\begin{aligned}
a_1 &= \frac{I_{yy} - I_{zz}}{I_{xx}}\\
a_2 &= -\frac{I_r}{I_{xx}}\\
a_3 &= \frac{I_{zz} - I_{xx}}{I_{yy}}\\
a_4 &= -\frac{I_r}{I_{yy}}\\
a_5 &= \frac{I_{xx} - I_{yy}}{I_{zz}}\\
b_1 &= \frac{l}{I_{xx}}\\
b_2 &= \frac{l}{I_{yy}}\\
b_3 &= \frac{l}{I_{zz}}\\
u_x &= (\cos(\phi)\sin(\theta)\cos(\psi) + \sin(\phi)\sin(\psi)) \\
u_y &= (\cos(\phi)\sin(\theta)\sin(\psi) - \sin(\phi)\cos(\psi))
\end{aligned}
$$

Then the equations can be written as:

$$
\begin{cases}
\begin{aligned}
\ddot{x} &= u_x \frac{l}{m} U_1 + \omega_x\\
\ddot{y} &= u_y \frac{l}{m} U_1 + \omega_y\\
\ddot{z} &= -g + (\cos(\phi)\cos(\theta)) \frac{l}{m} U_1 + \omega_z \\
\ddot{\phi} &= \dot{\theta}\dot{\psi}a_1 + \dot{\theta}\Omega a_2 + b_1 U_2 + \omega_\phi\\
\ddot{\theta} &= \dot{\phi}\dot{\psi}a_3 + \dot{\phi}\Omega a_4 + b_2 U_3 + \omega_\theta\\
\ddot{\psi} &= \dot{\theta}\dot{\phi}a_5 + b_3 U_4 + \omega_\psi\\
\end{aligned}
\end{cases}
$$

### 3.3 Sliding Mode Control Design

As said before, in a quadcopter control system trajectories are expressed for the motion of $[X, Y, Z, \psi]$. So, the control system is designed for these variables.

#### 3.3.1 Translational Control

1. Z Control System:
    $$
    \begin{aligned}
    e_z &= z_d - z \\
    s_z &= \dot{e}_z + \lambda_z e_z \\
    \rightarrow \dot{s}_z &= \ddot{e}_z + \lambda_z \dot{e}_z \\
    &= \ddot{z}_d - \ddot{z} + \lambda_z \dot{e}_z \\
    \rightarrow \dot{s}_z &= \ddot{z}_d + g - (\cos(\phi)\cos(\theta)) \frac{l}{m} U_1 - \omega_z + \lambda_z \dot{e}_z \\
    &= -\eta_z \ sign(s_z) \\
    \Rightarrow U_z &= \frac{m}{\cos(\phi)\cos(\theta)}(\ddot{z}_d + g - \omega_z + \lambda_z \dot{e}_z + \eta_z \ sign(s_z)) \\
    \end{aligned}
    $$

    $U_z$ can be sigular when $\cos(\phi)\cos(\theta) = 0$. So, we assume: $\lvert \phi \rvert \ , \ \lvert \theta \rvert \le \frac{\pi}{2}$.

2. As we have 2 control loops the $U_x$ and $U_y$ are controlled through virtual inputs $u_x , u_y$. Following the same principles:

    $$
    \begin{aligned}
    e_x &= x_d - x \\
    s_x &= \dot{e}_x + \lambda_x e_x \\
    \rightarrow \dot{s}_x &= \ddot{e}_x + \lambda_x \dot{e}_x \\
    &= \ddot{x}_d - \ddot{x} + \lambda_x \dot{e}_x \\
    \rightarrow \dot{s}_x &= \ddot{x}_d - u_x \frac{l}{m} U_1 - \omega_x + \lambda_x \dot{e}_x \\
    &= -\eta_x \ sign(s_x) \\
    \Rightarrow U_x &= \frac{m}{U_1}(\ddot{x}_d - \omega_x + \lambda_x \dot{e}_x + \eta_x \ sign(s_x)) \\
    \end{aligned}
    $$

    In the same manner $U_y$ can be extracted as:

    $$
    U_y = \frac{m}{U_1}(\ddot{y}_d - \omega_y + \lambda_y \dot{e}_y + \eta_y \ sign(s_y))
    $$

3. Stability check:

$$
\begin{aligned}
V(s_x, s_y, s_z) = \frac{1}{2}\sum_{i=x,y,z}s_i^2 \rightarrow \dot{V} &= \sum_{i=x,y,z}s_i\dot{s}*i \\
&= -\sum*{i=x,y,z}-\eta_i \ \lvert s_i \rvert \lt 0\\
\end{aligned}
$$

This shows that the system approaches the sliding surface in finite time.

#### 3.3.2 Rotational Control

To extract the desired &\phi& and $\theta$ from the desired $x_d$ and $y_d$, $u_x$ and $u_y$ can be used by substituting the $\psi_d = 0$. So:

$$
\begin{cases}
\begin{aligned}
u_x &= (\cos(\phi_d)\sin(\theta_d)\cos(0) + \sin(\phi_d)\sin(0)) \\
u_y &= (\cos(\phi_d)\sin(\theta_d)\sin(0) - \sin(\phi_d)\cos(0))
\end{aligned}
\end{cases}
\Rightarrow
\begin{cases}
\begin{aligned}
\phi_d &= -\arcsin(u_y) \\
\theta_d &= \arcsin(\frac{u_x}{\cos(\phi_d)}) \\
\end{aligned}
\end{cases}
$$

Also for the simplification, $\dot{\phi}_d$ and $\dot{\phi}_d$ are also assumed to be $0$.

Controller design:

1. design for $\phi$:

    $$
    \begin{aligned}
    e_\phi &= \phi_d - \phi \\
    s_\phi &= \dot{e}*\phi + \lambda*\phi e_\phi \\
    \rightarrow \dot{s}*\phi &= \ddot{e}*\phi + \lambda_\phi \dot{e}*\phi \\
    &= \ddot{\phi}*d - \ddot{\phi} + \lambda*\phi \dot{e}*\phi \\
    \rightarrow \dot{s}*\phi &= \ddot{\phi}*d -\dot{\theta}\dot{\psi}a_1 - \dot{\theta}\Omega a_2 - b_1 U_2 - \omega\phi + \lambda*\phi \dot{e}*\phi \\
    &= -\eta_\phi \ sign(s_\phi) \\
    \Rightarrow U_\phi &= \frac{1}{b_1}(\ddot{\phi}*d -\dot{\theta}\dot{\psi}a_1 - \dot{\theta}\Omega a_2 - \omega*\phi + \lambda_\phi \dot{e}*\phi + \eta*\phi \ sign(s_\phi)) \\
    \end{aligned}
    $$

2. design for $\theta$. In the same manner:

    $$
    U_\theta = \frac{1}{b_2}(\ddot{\theta}*d + \dot{\phi}\dot{\psi}a_3 - \dot{\phi}\Omega a_4 - \omega*\theta + \lambda_\theta \dot{e}*\theta + \eta*\theta \ sign(s_\theta))
    $$

3. design for $\psi$. In the same manner:

    $$
    U_\psi = \frac{1}{b_3}(\ddot{\psi}*d + \dot{\phi}\dot{\theta}a_5 - \omega*\psi + \lambda_\psi \dot{e}*\psi + \eta*\psi \ sign(s_\psi))
    $$

4. Stability is guaranteed in the exact same manner as the translational control.

> **Note:** The disturbance effect is modelled as $\omega = 0.5\sin(t/10) + 0.1\sin(t/5) + 0.02\sin(t)$ for all the inputs.

### 3.4 Results

---

## 4. Simulation Enviroment

For all the phases of the project, the simulation environment is the same. All the simulation is done in MATLAB. For all the phases the ODE function of the system is created and solved using the 4th order Runge-Kutta method or Euler's method of ODE solving which will be discussed in the following sections.

### 4.1 4th Order Runge-Kutta Method

The 4th order Runge-Kutta algorithm, also known as RK4, is a numerical method used to solve ordinary differential equations (ODEs) of the form:

$$
\frac{dy}{dt} = f(t, y)
$$

where $y$ is the dependent variable and $t$ is the independent variable. The algorithm is based on the idea of approximating the solution at each time step by using a weighted average of four estimates.

The general form of the RK4 algorithm can be written as:

$$
y_{n+1} = y_n + \frac{1}{6}(k_1 + 2k_2 + 2k_3 + k_4)
$$

where $y_n$ is the value of $y$ at time $t_n$, and $y_{n+1}$ is the value at time $t_{n+1} = t_n + h$, where $h$ is the time step. The four estimates $k_1$, $k_2$, $k_3$, and $k_4$ are given by:

$$
\begin{aligned}
k_1 &= hf(t_n, y_n) \\
k_2 &= hf(t_n + \frac{h}{2}, y_n + \frac{k_1}{2}) \\
k_3 &= hf(t_n + \frac{h}{2}, y_n + \frac{k_2}{2}) \\
k_4 &= hf(t_n + h, y_n + k_3)
\end{aligned}
$$

In words, the algorithm works as follows:

1. Evaluate the function $f(t, y)$ at the current time and state $(t_n, y_n)$ to obtain $k_1$.
2. Use $k_1$ to estimate the value of $y$ at the midpoint of the time step, $(t_n + \frac{h}{2}, y_n + \frac{k_1}{2})$, and evaluate $f(t, y)$ at this point to obtain $k_2$.
3. Use $k_2$ to estimate the value of $y$ at the midpoint of the time step, $(t_n + \frac{h}{2}, y_n + \frac{k_2}{2})$, and evaluate $f(t, y)$ at this point to obtain $k_3$.
4. Use $k_3$ to estimate the value of $y$ at the end of the time step, $(t_n + h, y_n + k_3)$, and evaluate $f(t, y)$ at this point to obtain $k_4$.
5. Combine the four estimates using a weighted average to obtain the final estimate for $y_{n+1}$.

The RK4 algorithm is widely used because it is relatively simple to implement and provides accurate results for a wide range of ODEs. However, it can be computationally expensive for very large systems of equations or for highly stiff ODEs.

### 4.2 Euler's Method

Euler's method is a simple numerical method used to solve ordinary differential equations (ODEs) of the form:

$$
\frac{dy}{dt} = f(t, y)
$$

where $y$ is the dependent variable and $t$ is the independent variable. The method involves approximating the solution at each time step by using the derivative of the solution at that point.

The general form of the Euler's method can be written as:

$$
y_{n+1} = y_n + hf(t_n, y_n)
$$

where $y_n$ is the value of $y$ at time $t_n$, and $y_{n+1}$ is the value at time $t_{n+1} = t_n + h$, where $h$ is the time step. The derivative $f(t_n, y_n)$ is evaluated at the current time and state $(t_n, y_n)$.

In words, the algorithm works as follows:

1. Start with an initial value of $y$ at time $t_0$.
2. Evaluate the derivative $f(t_0, y_0)$ at the initial time and state.
3. Use this derivative to estimate the value of $y$ at the next time step, $t_1 = t_0 + h$, using the formula above.
4. Repeat steps 2-3 for each subsequent time step until the desired end time is reached.

Euler's method is easy to implement and computationally efficient, but it may not always provide accurate results for complex ODEs or for large time steps. It is often used as a starting point for more advanced numerical methods like the 4th order Runge-Kutta algorithm.

### 4.3 Comparison of the Two Methods

The 4th order Runge-Kutta algorithm is more accurate than Euler's method, especially for complex ODEs or for large time steps. However, it is also more computationally expensive and may not be necessary for simple ODEs or small time steps. The choice between these methods depends on the complexity of the ODE and the desired level of accuracy.

---

## 5. References

[1] [Robotics, Vision and Control: Fundamental Algorithms in MATLAB](http://www.petercorke.com/RVC/)
[2] [Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotor](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=6289431)
[3] [A platform for aerial robotics research and demonstration: The Flying Machine Arena](https://www.flyingmachinearena.ethz.ch/wp-content/publications/2014/lupashin2014platform.pdf)
[4] [Backflipping with Miniature Quadcopters by Gaussian Process Based Control and Planning](https://www.researchgate.net/publication/364025711_Backflipping_with_Miniature_Quadcopters_by_Gaussian_Process_Based_Control_and_Planning)
[5] [A Simple Learning Strategy for High-Speed Quadrocopter Multi-Flips](https://ieeexplore.ieee.org/document/5509452)
[6] [Adaptive Open-Loop Aerobatic Maneuvers for Quadrocopters](https://www.sciencedirect.com/science/article/pii/S147466701644005X)
[7] [Adaptive fast open-loop maneuvers for quadrocopters](https://www.researchgate.net/publication/257523032_Adaptive_fast_open-loop_maneuvers_for_quadrocopters)
[8] [MODELING, SIMULATION AND COMPLETE CONTROL OF A QUADCOPTER](https://harikrishnansuresh.github.io/assets/quadcopter_control_project_report.pdf)
[9] [Modeling and Nonlinear Control of a Quadcopter for Stabilization and Trajectory Tracking](https://www.hindawi.com/journals/je/2022/2449901/)
[10] [Trajectory planning of quadrotor using sliding mode control with extended state observer](https://journals.sagepub.com/doi/10.1177/0020294020927419)
[11] [Sliding Mode Control: An Approach to Control a Quadrotor](https://www.researchgate.net/publication/279450122_Sliding_Mode_Control_An_Approach_to_Control_a_Quadrotor)
[12] [Observer-Based Sliding Mode Control of a 6-DOF Quadrotor UAV](https://ieeexplore.ieee.org/document/8592769)
[13] [Runge–Kutta methods](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods)
[14] [Euler method](https://en.wikipedia.org/wiki/Euler_method)
[15] [BBO]