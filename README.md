# CarND Controls MPC Project
Self-Driving Car Engineer Nanodegree Program

The main objective of MPC is solve the optimization problem for trajectory. The Model Predictive Controller (MPC) was used to derive throttle, brake and steering angle actuators for a car to drive around a circular track.

* Model Predictive Control involves
	* simulating different actuator inputs
	* predicting the resulting trajectory
	* selecting the trajectory with minimum cost.

[//]: # (Image References)
[image1]: ./images/eqns.png
[image2]: ./images/global-map-axes.png
[image3]: ./images/result40.png
[image4]: ./images/cte-formula.png
[image5]: ./images/epsi-formula.png
[image6]: ./images/result.png


---

# Model Predictive Controller (MPC) Basic

* Input Parameter

	* Kinematic Model State
		* x: x position of vehicle
		* y: y position of vehicle
		* ψ (psi): vehicle's angle in radians from the x-direction (radians)
		* ν: velocity of vehicle
		* cte: cross track error
		* eψ : orientation error

	* Actuator
		* δ (delta): steering angle
		* a : acceleration (including throttle and break)

* Chose initial parameter
	* Lf (radius of circule) as 2.67
	* ref_v (reference velocity) as 40 mph
	* N (timestemp length) as 25
	* dt (elapsed duration between timestemps) as 0.05
	* latency as 100 ms

* Basic Algorithm

	* Calculate the predicted x, y position and velocity based on given kinematic model
		* Given x and y position of vehicle, velocity and given parameter, calculated following.

		* ![alt text][image1]

	* Transform the next state of vehicle coordinate system to the global map system.

		* ![alt text][image2]
		  ```
		  for(unsigned int i = 0; i < ptsx.size(); i++) {
             x_veh[i] = (ptsx[i] - px_pred) * cos(-psi_pred) - (ptsy[i] - py_pred) * sin(-psi_pred);
             y_veh[i] = (ptsx[i] - px_pred) * sin(-psi_pred) + (ptsy[i] - py_pred) * cos(-psi_pred);
          }
		  ```

	* Calculate the reference waypoint trajectory

		* ![alt text][image3]
		
		the waypoints were created by coordinate tramsform with smooth curved trajectory, which was used for motion of car. This was implemented with polynomial fitting or regression technique. The discrete point of x and y were fitted to generated a polynomial 3rd degree function and using the function drawed the yellow waypoint of smooth curve in the simulator.

	* Calculate the CTE (Cross Track Error) and EPSI (Orientation angle - psi error)

		* ![alt text][image4] 
			```
			fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			```

		* ![alt text][image5]
			```
			fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
			```

	* Calculate the cost
		* define actuator constraints
			* CTE expected value is zero
			* EPSI expected value is zero
			* Max speed limit is 100 mph
		* MPC solver - Using Ipopt and Cppad library, calculated the actuator vaule at the minimal value of the cost.
			* Calculate the CTE and EPSI for highest weight. This ensure the car to keep in the middle of lane.
				```
				for (unsigned int t = 0; t < N; t++) {
				  fg[0] += 1500 * CppAD::pow(vars[cte_start + t], 2);
				  fg[0] += 1500 * CppAD::pow(vars[epsi_start + t], 2);
				  fg[0] += 10 * CppAD::pow(vars[v_start + t] - ref_v, 2);
				}
				```
			* Calculate square sume of delta and a to minimize the use of actuators
				```
				for (unsigned int t = 0; t < N -1 ; t++) {
					fg[0] += 100 * CppAD::pow(vars[delta_start + t], 2);
					fg[0] += 10 * CppAD::pow(vars[a_start + t], 2);

					//This is to reduce speed at turns and increase on straight road
					fg[0] += 500 * CppAD::pow((vars[v_start + t] * vars[delta_start + t]), 2);
				}
				```
			* Calculate the minimum speeds at higher steering angle  and minimum steering at higher speed.
				```
				for (unsigned int t = 0; t < N - 2; t++) {
				  fg[0] += 50 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
				  fg[0] += 50 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
				}
				```
			* Calculate sum of difference between v and reference velocity to encourage the vehicle to stay at the reference velocty

# Project Output & tuning
* ![alt text][image6] 
* After trial the vehicle was able to run successfully with 100 mph, N = 7, and dt = 0.07. 
	```
	|      mph      |          N        |        dt         |        result           |
	|:-------------:|:-----------------:|:-----------------:|:-----------------------:|
	|      80       |         25        |       0.05        |       successful        |
	|      90       |         25        |       0.05        | failed to handle curve  |
	|      90       |         10        |       0.10        |       successful        |
	|     100       |         10        |       0.1         | failed to handle curve  |
	|     100       |          5        |       0.1         |       failed            |
	|     100       |          8        |       0.1         | 50% of track was compl  |
	|     100       |          8        |       0.13        |         failed          |
	|     100       |          8        |       0.08        |         failed          |
	|     100       |          7        |       0.07        |       successful        |
	```

# Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
