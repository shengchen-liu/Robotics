# Design of Multi-Dimensional Kalman Filters

From this point forward we will transition to using linear algebra, as it allows us to easily work with multi-dimensional problems. To begin with, let’s write the state prediction in linear algebra form.

## State Transition

The formula below is the state transition function that advances the state from time _t_ to time *t + 1*. It is just the relationship between the robot’s position, x*x*, and velocity, \dot{x}*x*˙. Here, we will assume that the robot’s velocity is not changing.
$$
\large x' = x + \Delta t \dot{x}
$$

$$
\large \dot{x}' = \dot{x}
$$

We can express the same relationship in matrix form, as seen below. On the left, is the posterior state (denoted with the prime symbol, '′), and on the right are the state transition function and the prior state. This equation shows how the state changes over the time period, \Delta tΔ*t*. Note that we are only working with the means here; the covariance matrix will appear later.
$$
\large \begin{bmatrix} x \\ \dot{x} \end{bmatrix}' = \begin{bmatrix} 1 & \Delta{t} \\ 0 & 1 \end{bmatrix} \begin{bmatrix} x \\ \dot{x} \end{bmatrix}
$$
The State Transition Function is denoted F*F*, and the formula can be written as so,
$$
\large x' = F
$$
In reality, the equation should also account for process noise, as its own term in the equation. However, process noise is a Gaussian with a mean of 0, so the update equation for the mean need not include it.
$$
\large x' = Fx + noise
$$

$$
\large noise \sim N(0,Q)
$$

Now, what happens to the covariance? How does it change in this process?

**Sidenote:** While it is common to use $\Sigma$ to represent the covariance of a Gaussian distribution in mathematics, it is more common to use the letter P to represent the state covariance in localization.

If you multiply the state, *x*, by *F*, then the covariance will be affected by the square of *F*. In matrix form, this will look like so:
$$
\large P' = FPF^T
$$
However, your intuition may suggest that it should be affected by more than just the state transition function. For instance, additional uncertainty may arise from the prediction itself. If so, you’re correct!

To calculate the posterior covariance, the prior covariance is multiplied by the state transition function squared, and Q*Q* added as an increase of uncertainty due to process noise. Q*Q* can account for a robot slowing down unexpectedly, or being drawn off course by an external influence.
$$
\large P' = FPF^T + Q
$$
Now we’ve updated the mean and the covariance as part of the state prediction.



### Quiz 1

Now that you've seen how a simple state transition function is created, let's see if you can construct a more complicated one for the following problem:

You are tracking the position and velocity of a robot in two dimensions, x and y. The state is represented as so,
$$
\large x = \begin{bmatrix} p_x \\ p_y \\ v_x \\ v_y \end{bmatrix}
$$
Find the state update function, F, that will advance the state from time _t_ to time *t + 1* based on the state transition equation below.
$$
\large x' = Fx
$$
Try to work through this on paper before looking at the quiz options below.



### QUESTION 1 OF 3

nlp clients

