# CarND-Controls-MPC

I implemented Model Predictive Control (MPC) to drive the car around the track. The goal of MPC is to calculate the steering, throttle and steering values for a car to follow a given trajectory. The MPC fits a polynomial line to the planned trajectory and uses an optimzier to find the control inputs and minimize a user-defined cost function. 

![demo](demo.gif)

## The model

State: (x, y, psi, v), where psi is the yaw angle and v is the velocity. In the implementation, the state also includes cte (cross-track error) and epsi (error in the yaw angle).

The update equations are the following:

```python
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt 
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
```

, where delta is the steering angle and a is the throttle value.


## Timestep Length and Elapsed Duration (N & dt)

I set N (unmber of step to be predicted) to be 10 and dt (time elapsed between steps) to be 0.05. With the set of values, the car is able to complete a full lap without going off track.

I previously tried N = 10, and dt = 0.1. This also works ok. The predicted trajectory would be longer but the qualitative driving behavior don't differ much.

I also tried N = 20, and = 0.1. With this set however, at certain points in the loop, the predited trajectory can become erratic (for example, contains loop). The car goes off track. I suspect that is because the program cannot converge to a solution with a wider search space.

## Polynomial Fitting and MPC Preprocessing

I fit a third order polynomial to the planned trajectory.

## Model Predictive Control with Latency

I assume a latency of 100 milleseconds, and estimate the car's state after 100 milliseconds with the following set of equations:

  ```python
      double est_px = v * latency ;
      double est_py = 0.;
      double est_psi = 0. - v * delta / Lf * latency;
      double est_v = v + a * latency;
      double est_cte = cte + v * sin(epsi) * latency;
      double est_epsi = epsi - v * delta / Lf * latency;
  ```

. Essentially, it assumes the car moves straight in the x direction during the latency.


