# 4th Order Runge Kutta Method

The 4th Order Runge Kutta method is a numerical method used for solving ordinary differential equations (ODEs). It is a popular method because of its accuracy and efficiency.

The general form of an ODE is:

$$\frac{dy}{dx} = f(x,y)$$

where $y$ is the dependent variable and $x$ is the independent variable. The function $f(x,y)$ represents the rate of change of $y$ with respect to $x$.

The 4th Order Runge Kutta method involves computing four intermediate values of $y$ at each step using the following equations:

$$k_1 = hf(x_n, y_n)$$

$$k_2 = hf(x_n + \frac{h}{2}, y_n + \frac{k_1}{2})$$

$$k_3 = hf(x_n + \frac{h}{2}, y_n + \frac{k_2}{2})$$

$$k_4 = hf(x_n + h, y_n + k_3)$$

where $h$ is the step size, $x_n$ and $y_n$ are the values of $x$ and $y$ at the current step, and $k_i$ are the intermediate values.

Then, the value of $y$ at the next step is computed using the formula:

$$y_{n+1} = y_n + \frac{1}{6}(k_1 + 2k_2 + 2k_3 + k_4)$$

This process is repeated until the desired accuracy is achieved or the desired number of steps is reached.

Overall, the 4th Order Runge Kutta method is a powerful tool for solving ODEs numerically and is widely used in various fields of science and engineering.
