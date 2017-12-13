# Write-Up

Author: Abhishek Mantha
Date: 11/13/17

This document is an official write-up for SDCND Project 5: Model Predictive Control. I will further elaborate on implementation details and approach.

---

## MPC Implementation

### A.
The model I used is an adapted version of a standard kinematics model. In a basic kinematics model, we ignore tire forces, gravity and car mass, enabling us to approximate actual vehicle dynamics at lowspeed. For this project, we implement a dynamic model with an aim to embody actual vehicle dynamics as close as possible. 

Our state vector is therefore comprised of 6 variables: __px, py, psi, v, cte, epsi__. px and py are the car's x and y location (in the environment's coordinate system), psi is the car's orientation or heading, v is velocity (which must be converted to m/s), cte is the cross track error, and epsi is the orientation error value. Using these 6 state variables, we can model a vehicle's behavior on the road. 

But what actually affects that behavior? Actuators! Actuators are parameters that change how the vehicle will respond over time. The two  actuators we use in this project are delta and alpha: delta changes steering wheel angle and alpha changes the car's throttle, or acceleration (roughly!). 

Combined with our state vector, we now have our model!

Here's the entire list of equations that we'll use on each time step of the simulation:

    * x' = x + v * cos(psi) * dt
    * y' = y + v * sin(psi) * dt
    * psi' = psi + v / Lf * delta * dt
    * v' = v + a * dt
    * cte' = cte + v * sin(epsi) * dt
    * epsi' = epsi + v / Lf * delta * dt

To clarify some variables: 
    dt = time step 
    Lf = turn radius coefficient (the larger the turn radius, the longer it takes to make the turn)

It's key to note that we use the previous state to calculate the next state.

### B.
The final values of N and dt are 25 and 0.05. The following are combinations of values that I tested:

    * (10, 0.1) -- the car drives at an average of 35-40 mph, it does have a tendency to touch the lines, but it is good about correcting itself

    * (25, 0.05) -- the car drives at an average of 25-30 mph, it does a very good job of staying in the center of the lane, but it definitely requires more compute time

    * (10, 0.05) -- the car would start out fast, then fly off the curb very quickly 

    * (25, 0.1) -- the car moves very slowly, it stops and starts, not ideal for the road, and then eventually stops

Lines 53-70 of MPC.cpp contain my cost function implementation for the IPOPT optimizer. I have a high coefficient for optimizing CTE as I want the model to stay towards the center of the lane as much as possible. In addition, I have lower coefficient values for epsi and v, with a relative epsi of 0 and relative v of 100. I found that if I did not include a high enough coefficient or any coefficient at all, the car would move backwards :(

I place a significantly high value on the car's steering wheel angle. I found that values greater than 4000 allowed the car to focus on its correction. With averaging the actuator inputs and latency calculation, this enabled the car to move relatively smoothly. 

Finally, I place importance on the pairwise transition of delta and alpha, more so on delta. Again, this enables the car to correct its behavior. 

These values were tuned by hand. I think that I have gained a fairly good understanding of each costs contributions, as I did with the PID project. 

One thing that I would like to correct, however, is the start and stop behavior of the car, as well as increase its overall speed. 

### C.
I fit a 3rd order polynomial to the waypoints, as per the recommendation from lecture. I found that it aligns with the curvature of the road well. 

I did preprocess the data prior to passing it along to the MPC model. First, I converted the velocity from miles/hour to meters/second by multiplying v by 0.44704. This was a significant improvement on the vehicle's behavior. Second, as was the recommendation of the form leaders, I converted the waypoints from the map's coordinate system to the car's coordinate system by shifting by px and py respectively for each coordinate and then applying a rotation along the new axis. This can be seen in lines 101-107 of main.cpp. Due to this transformation, px, py and psi will be 0 as all model updates are computed relative to the car's perspective. This also simplifies our initial cte and epsi calculations, as many terms will 0 out. This is seen in lines 116-117 of main.cpp. 

### D.
To handle 100 millisecond latency, I utilized an approach recommended by my first reviewer and others in the forum. The solution given by the MPC is meant for immediate effect. When the actuators receive them, it will already be 100ms too late, which is a huge problem. To handle this issue, I effectively predict the state variables after 100ms before sending them to the MPC. This way, the values returned by the MPC will therefore be useful for immediate effect. This can be seen in lines 119-127 of main.cpp.

To make sure that I am correcting the actuator values on each new input state, I store the upper and lower bounds of alpha and delta to be the previous alpha and delta values respectively. The previous values are actually not the first values computed in the solution vector returned from the IPOPT solver. I instead use the second values returned after 100ms and store them for the next run.

The MPC returns a vector of the current run's actuator inputs and the predicted MPC waypoints for display (green line). I found that averaging the first three values of the actuator inputs allowed for a very smooth ride. 

I make sure to normalize both the delta and alpha values returned from the MPC to be within [-1, 1]. 


# MPC Simulation

The car does not leave the drivable portion of the track. The car NEVER touches the curb at any point. 