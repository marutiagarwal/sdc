# Model Predictive Controller (MPC)

### The Model
The presented system makes use of model-predictive-controller, which is an improvement over the PID controller. PID controller computes error w.r.t. present state, but the actuation is applied in the future (and likely different state). This can lead to instability at times.

MPC makes use of dynamic kinematics model to acount for the latency. Once we know the latency, we can compute the state of vehicle forward in time using a simple dynamic model and compute actuation based on that. 

# Timestep Length and Elapsed Duration (N & dt)
We chose N=15 and dt=0.1 by trial and error. Choosing a smaller N makes the ride very unstable and the car keeps swinging left and right and finally gets off the track. Lasger values for N results into the ride taking more time.

# Polynomial Fitting and MPC Preprocessing
The polynomial fitting is performed in the polyfit function in main.cpp. The code for this is adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716

Prior to feeding state variable to MPC solver, we transform the x and y coordinates into the vehicle coordinate system. 

# Model Predictive Control with Latency
Prior to feeding state variable to MPC solver, we compensate for the 100ms latency using simple kinematics model. The state variables are predicted 100ms into future and then fed to MPC solver. We use the beloe given kinematics equations for that:

![alt tag](https://github.com/marutiagarwal/sdc/blob/term2/Model-Predictive-Controller/images/kinematics.png)
