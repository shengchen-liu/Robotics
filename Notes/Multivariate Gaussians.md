# Multivariate Gaussians

Most robots that we would be interested in modeling are moving in more than one dimension. For instance, a robot on a plane would have an x & y position.

The simple approach to take, would be to have a 1-dimensional Gaussian represent each dimension - one for the x-axis and one for the y-axis.

## Formulas for the Multivariate Gaussian

### Mean

The mean is now a vector,

\large \mu = \left[ \begin{array}{c} \mu_{x} \\ \mu_{y} \end{array} \right]*μ*=[*μ**x**μ**y*]

### Covariance

And the multidimensional equivalent of variance is a covariance matrix,

\large \Sigma = \left[ \begin{array}{cc} \sigma_{x}^2 & \sigma_{x}\sigma_{y} \\ \sigma_{y}\sigma_{x} & \sigma_{y}^2 \end{array} \right]Σ=[*σ**x*2*σ**y**σ**x**σ**x**σ**y**σ**y*2]

Where \sigma_{x}^2*σ**x*2 and \sigma_{y}^2*σ**y*2 represent the variances, while \sigma_{y}\sigma_{x}*σ**y**σ**x* and \sigma_{x}\sigma_{y}*σ**x**σ**y* are correlation terms. These terms are non-zero if there is a correlation between the variance in one dimension and the variance in another. When that is the case, the Gaussian function looks 'skewed' when looked at from above.

If we were to evaluate this mathematically, the eigenvalues and eigenvectors of the covariance matrix describe the amount and direction of uncertainty.

### Multivariate Gaussian

Below is the formula for the multivariate Gaussian. Note that x*x* and \mu*μ* are vectors, and \SigmaΣ is a matrix.

\large p(x) = \frac{1}{(2\pi)^{\frac{D}{2}}|\Sigma|^\frac{1}{2}}e^{-\frac{1}{2}(x-\mu)^T\Sigma^{-1}(x-\mu)}*p*(*x*)=(2*π*)2*D*∣Σ∣211*e*−21(*x*−*μ*)*T*Σ−1(*x*−*μ*)

If D=1, the formula simplifies to the formula for the one-dimensional Gaussian that you have seen before.

