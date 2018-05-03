/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Set the number of particles
	num_particles = 10;

	// Set standard deviations for x, y, and theta
	default_random_engine gen;
	double std_x, std_y, std_theta;
	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];

	// Create normal distributions for x, y and theta
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	for (int i = 0; i < num_particles; ++i) {

		Particle aParticle;
		
		// Initialize particle
		aParticle.id = i;
		aParticle.x = dist_x(gen);  //this will sample a number from the gaussian distribution
		aParticle.y = dist_y(gen); 
		aParticle.theta = dist_theta(gen);
		aParticle.weight = 1.0;
		 
		// Print samples to the terminal.
		// cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " " << sample_theta << endl;

		// Add particle to particle list
		particles.push_back(aParticle);
	}

	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Set standard deviations for x, y, and theta
	default_random_engine gen;
	double std_x, std_y, std_theta;
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];

	// Create normal distributions for x, y and theta
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);

	// Iterate through the particle list
	for (int i = 0; i < num_particles; ++i) {

		double x_f, x_0, y_f, y_0, theta_f, theta_0;
		Particle aParticle = particles[i]; 

		// Initialize variables of motion model
		x_0 = aParticle.x;
		y_0 = aParticle.y;
		theta_0 = aParticle.theta;

		// Calcaulate new values using motion model
		if (fabs(yaw_rate) > 0.00001) {
			// Motion model when yaw rate is not equal to zero
			x_f = x_0 + ((velocity / yaw_rate) * (sin(theta_0 + (yaw_rate * delta_t)) - sin(theta_0)));		
			y_f = y_0 + ((velocity / yaw_rate) * (cos(theta_0) - cos(theta_0 + (yaw_rate * delta_t))));
			theta_f = theta_0 + (yaw_rate * delta_t);
		}
		else {
			// Motion model when yaw rate is equal to zero
			x_f = x_0 + (velocity * delta_t * cos(theta_0));
			y_f = y_0 + (velocity * delta_t * sin(theta_0));
			theta_f = theta_0;	
		}

		// Add noise to x, y and theta
        x_f += dist_x(gen);
        y_f += dist_y(gen);
        theta_f += dist_theta(gen);

        // Update particle values
        aParticle.x = x_f;
        aParticle.y = y_f;
        aParticle.theta = theta_f;
		
		// Update particle list
		particles[i] = aParticle;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// This function is not used in our code
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// For each particle, we will first map each observation to the particle's POV and transform its coordinate 
	// to map coordinates.  Then, we will associate each transformed observation to a landmark using the nearest 
	// neighbour approach.  Finally, we will calculate the weight of measurement using the multivariate-Gaussian 
	// probability density function.

	// Tracks the cumulative weight values for all of the particles 
	double cumulative_weight = 0.0;

	for (int i = 0; i < num_particles; ++i) {

		Particle aParticle = particles[i];

		// define particle coordinates and theta
		double x_part = aParticle.x;
		double y_part = aParticle.y;
		double theta = aParticle.theta;

		// initialize sigmas
		double sig_x = std_landmark[0];
		double sig_y = std_landmark[1];

		// total weight value;
		double total_part_weight = 1.0;

		for (unsigned int j = 0; j < observations.size(); ++j) {

			// Step 1: Transform an observation to map coordinates in relation to each particle

			LandmarkObs aObservation = observations[j];

			// define observation coordinates
			double x_obs = aObservation.x;
			double y_obs = aObservation.y;
	
			// transform to map x coordinate
			double x_map = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);

			// transform to map y coordinate
			double y_map = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);

			// define new transformed observation using the map coordinates
			LandmarkObs transformedOb;
			transformedOb.x = x_map;
			transformedOb.y = y_map;

			// Step 2: Find the landmark that has the closest distance to the transformed observation (i.e. nearest neighbour)

			// variables to track landmark distance and position in vector
			double closest_landmark_distance = 0.0;
			int closest_landmark_pos = 0; 

			// Iterate through all of the landmarks to find the nearest neighbour to the transformed observation
			for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); ++k) {

				Map::single_landmark_s aLandmark = map_landmarks.landmark_list[k];

				double distance_from_particle = dist(x_part, y_part, aLandmark.x_f, aLandmark.y_f);

				// If location of landmark is beyond sensor range, then skip to next landmark 
				if (distance_from_particle > sensor_range)
					continue;

				double distance_from_transformedOb = dist(transformedOb.x, transformedOb.y, aLandmark.x_f, aLandmark.y_f);

				if (fabs(closest_landmark_distance) < 0.00001) {
					// Associate with first landmark
					closest_landmark_distance = distance_from_transformedOb;
					closest_landmark_pos = k;
				}
				else if (distance_from_transformedOb < closest_landmark_distance) {
					closest_landmark_distance = distance_from_transformedOb;
					closest_landmark_pos = k;
				}
			}

			// Initialize the nearest neighbour
			Map::single_landmark_s closest_Landmark = map_landmarks.landmark_list[closest_landmark_pos];
			transformedOb.id = closest_Landmark.id_i;

			// Step 3: Calculate the weight of each measurement using the multivariate-Gaussian probability density function

			// calculate normalization term
			double gauss_norm = (1/(2 * M_PI * sig_x * sig_y));

			// calculate exponent
			double exponent = (pow((transformedOb.x - closest_Landmark.x_f), 2))/(2 * pow(sig_x,2)) + (pow((transformedOb.y - closest_Landmark.y_f), 2))/(2 * pow(sig_y, 2));

			// calculate weight using normalization terms and exponent
			double weight = gauss_norm * exp(-exponent);

			// update total particle weight value
			total_part_weight *= weight;
		}

		// Final step is to assign the final weight value to the particle
		aParticle.weight = total_part_weight;
		particles[i] = aParticle;

		// Update cumulative weight, which will be used to normalize each of the particle's weight later
		cumulative_weight += total_part_weight;
	}

	// Normalize the weight value of each particle
	for (int i = 0; i < num_particles; ++i) {

		Particle aParticle = particles[i];
		aParticle.weight /= cumulative_weight;
		particles[i] = aParticle;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::vector<double> weights;
	default_random_engine gen;

	// Extract weight from each particle
	for (int i = 0; i < num_particles; ++i) {

		Particle aParticle = particles[i];
		double weight = aParticle.weight;
		weights.push_back(weight);
	}

	// resample the particles in a frequency that is proportional to their weight
	discrete_distribution<> distribution(weights.begin(), weights.end());

	vector<Particle> newParticles;

	for (int i = 0; i < num_particles; ++i) {

		int weighted_index = distribution(gen);
		Particle aParticle = particles[weighted_index];
		newParticles.push_back(aParticle);
	}

	particles = newParticles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
