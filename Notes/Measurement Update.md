# Measurement Update

The new mean is a weighted sum of the prior belief and measurement means. With uncertainty, a larger number represents a more uncertain probability distribution. However, the new mean should be biased towards the measurement update, which has a smaller standard deviation than the prior. How do we accomplish this?

$$
\large \mu' = \frac{r^2 \mu + \sigma^2 v}{r^2 + \sigma^2}
$$
The answer is - the uncertainty of the prior is multiplied by the mean of the measurement, to give it more weight, and similarly the uncertainty of the measurement is multiplied with the mean of the prior. Applying this formula to our example generates a new mean of 27.5, which we can label on our graph below.



[![img](https://video.udacity-data.com/topher/2018/February/5a77fa82_c2l2-graph-1-v1/c2l2-graph-1-v1.png)](https://classroom.udacity.com/nanodegrees/nd209/parts/a431d446-05df-4641-9e3d-79e1d55a7a2f/modules/b66739be-878e-4cea-8569-881b7eb2d34c/lessons/f002d591-94af-4c70-aeac-ac2ed6f7b527/concepts/0976a728-e605-4149-9074-cba310f02b9f#)



#### Variance Calculation

Next, we need to determine the variance of the new state estimate.

The two Gaussians provide us with more information together than either Gaussian offered alone. As a result, our new state estimate is more confident than our prior belief and our measurement. This means that it has a higher peak and is narrower. You can see this in the graph below.



[![img](https://video.udacity-data.com/topher/2018/January/5a6a7575_c2l2-graph-2/c2l2-graph-2.png)](https://classroom.udacity.com/nanodegrees/nd209/parts/a431d446-05df-4641-9e3d-79e1d55a7a2f/modules/b66739be-878e-4cea-8569-881b7eb2d34c/lessons/f002d591-94af-4c70-aeac-ac2ed6f7b527/concepts/0976a728-e605-4149-9074-cba310f02b9f#)



The formula for the new variance is presented below.
$$
\large \sigma^{2'} = \frac{1}{\frac{1}{r^2} + \frac{1}{\sigma^2}}
$$
Entering the variances from our example into this formula produces a new variance of 2.25. The new state estimate, often called the posterior, is drawn below.



[![img](https://video.udacity-data.com/topher/2018/January/5a6cfd03_c2l2-graph-3/c2l2-graph-3.png)](https://classroom.udacity.com/nanodegrees/nd209/parts/a431d446-05df-4641-9e3d-79e1d55a7a2f/modules/b66739be-878e-4cea-8569-881b7eb2d34c/lessons/f002d591-94af-4c70-aeac-ac2ed6f7b527/concepts/0976a728-e605-4149-9074-cba310f02b9f#)





 $ \mu $ : Mean of the prior belief 
$ \sigma^{2}$: Variance of the prior belief 

$v$: Mean of the measurement 
$r^{2}$: Variance of the measurement 

$\tau$: Mean of the posterior 
$s^{2}$: Variance of the posterior



It’s time to implement these two formulas in C++. Place your code within a function called measurement_update, such that you can use it as a building block in your Kalman Filter implementation.

When you’re done, calculate the posterior mean and variance for a prior of $N(x: μ_1=10, σ^2=8)$and measurement $N(x: μ_2=13, σ^2=2)$. Is it what you expected?



