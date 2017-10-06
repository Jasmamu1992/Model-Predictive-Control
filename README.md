# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Model
A simple kinematic model was used as discussed in the course

State of a vehicle consiste of 6 parameter, x coordinate, y coordinate, orientation, velocity, crosstrack error, orientation error.
```c++
state << px, py, psi, v, cte, epsi;
```
Actuators are steering angle[-25deg 25deg] and acceleration[-1 1]. There is not break since negative acceleration is considered as break.
```c++
actuators << delta, a;
```
The state of the car at time t is predicted from state and actuators at time t-1 as follows,
```c++
px1 = px0 + v0 * cos(psi0) * dt;
py1 = py0 + v0 * sin(psi0) * dt;
psi1 = psi0 + v0 * delta0 * dt / Lf;
v1 = v0 + a0 * dt;
cte1 = f(x) - y0 + v0 * sin(epsi0) * dt;
epsi1 = psi0 - psi_des + v0 * delta0 * dt / Lf;
```
where, Lf --------------- Length of vehicle from front axle to the CoG
       epsi0 ------------ orientation error at time t-1
       cte0 ------------- cross track error at time t-1
       
       
## Timestep Length and Elapsed Duration (N & dt)
I have chosen Timestep length N as 25 and Elapsed Duration dt as 0.1 sec
with the above values the total prediction time T will be 2.5 sec. At a speed of reference velocity 40 mph, the vehicle will cover almost 45 meters, which is a substantial amount. Anything more than that should be waste in computational time

I have tried with different value of Timestep lengths like 10 and 15, but the vehicle performance for much smoother and better when I chose higher number like 25. I then tuned dt accordingly

## Polynomial fitting and mpc processing
I changed the waypoints to the vehicle point of reference
```c++
for(unsigned int i = 0; i < ptsx.size(); i++){
              double x = ptsx[i] - px;
              double y = ptsy[i] - py;
              pts_x(i) = x * cos(-psi) - y * sin(-psi);
              pts_y(i) = y * cos(-psi) + x * sin(-psi);
          }
```
This conversion simplifies the equations to calculate elements of intial state of car like cte, epsi etc,.
For example with the above transformation px, py and psi will become zero, since the waypoints are transformed to the vehicle point of reference. The velocity will be the velocity of the car. The equations for cte and epsi will be simplified as follows
```
Eigen::VectorXd coeffs = polyfit(pts_x, pts_y, order);
double cte = polyeval(coeffs, 0);
double epsi = - atan(coeffs[1]);
```

## Model Predictive Control with Latency
