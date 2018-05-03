## Udacity - Self Driving Car Nanodegree (Term 2) - Kidnapped Vehicle Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Background
---
In this project, my goal is to build a particle filter in C++ that will estimate the correct position of a vehicle in a simulation.

Overview of Repository
---
This repository contains the following source files that I have forked from the [main repository](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project) and subsequently modified for the extended Kalman filter project:

1.  [particle_filter.cpp](https://github.com/MartinKan/CarND-Kidnapped-Vehicle-Project/blob/master/src/particle_filter.cpp)

The particle class implements the particle filter. It is initialized at first by placing 10 particles at a given location (i.e. simulates GPS positioning) with random noise added to simulate real world error (this is known as initializing the priors - the initial belief state).  Each time the filter is called, it goes through the prediction step first followed by the update (or correction) step, just like any other Bayes filter.  The prediction step predicts the new location of each particle using the motion model, which in our case, is the motion model of a bicycle (the motion of a car can be simulated by the motion of a bicycle).  The motion model equations depends on whether or not the yaw rate (rate of turning) is zero.  

If the yaw rate is zero, the following equations would apply:

![alt text](https://github.com/MartinKan/CarND-Kidnapped-Vehicle-Project/blob/master/Motion-model-1.jpg)

If the yaw rate is not zero, the following equations would apply:

![alt text](https://github.com/MartinKan/CarND-Kidnapped-Vehicle-Project/blob/master/Motion-model-2.jpg)

After the location of each particle has been updated by the prediction step, the update step is then called to update the weight of each particle.  Each particle carries a weight value, which determines how much weight is given to the particle when estimating the location of the vehicle.  The weight value of each particle is updated as follows:

	1.	For each particle, we will first map each observation to the particle's POV and transform its coordinates to map coordinates (recall that the sensors will make observations on the location of nearby landmarks using the vehicle coordinate space, so we need to first map them from the POV of each particle - since each particle represents a possible location of the vehicle - and then map the observations' coordinates from the persective of each particle to the map's coordinate space)
	2.	Then, we will associate each transformed observation to a landmark using the nearest neighbour approach (we associate the closest landmark to each of the transformed observations).  
	3.	Next, we will update the weight of each measurement using the multivariate-Gaussian probability density function, which is described below:

![alt text](https://github.com/MartinKan/CarND-Kidnapped-Vehicle-Project/blob/master/Multivariate-Gaussian.jpg)

	4.	Finally, the weight of the particle is calculated by multiplying the weight of all measurement values (that we derived in step 3) together.

Integral to the particle filter is the resampling step, which is called after the weight of all of the particles have been updated.  This resampling step will repopulate the particle list in accordance with the probability of each particle's weight - so more important particles will appear more relevant and less important particles will appear less relevant in the new particle list.

To run the code, first compile the code by running the following command:

	./clean.sh
	./build.sh

Then run the code by executing the following command:

	./run.sh
