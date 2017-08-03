# Model Predictive Controller (MPC)

### The Model
The presented system makes use of model-predictive-controller, which is an improvement over the PID controller. PID controller computes error w.r.t. present state, but the actuation is applied in the future (and likely different state). This can lead to instability at times.

MPC makes use of dynamic kinematics model to acount for the latency. Once we know the latency, we can compute the state of vehicle forward in time using a simple dynamic model and compute actuation based on that. 

# Timestep Length and Elapsed Duration (N & dt)

# Polynomial Fitting and MPC Preprocessing

# Model Predictive Control with Latency