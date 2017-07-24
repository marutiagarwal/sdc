/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random> // Need this for sampling from distributions
#include <algorithm>
#include <iostream>
#include <numeric> // std::accumulate
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double stddev[]) {
	// std : array of uncertainties for the given measurements

	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	num_particles = 11;
	weights.resize(num_particles);

	default_random_engine gen;
	double std_x, std_y, std_theta; // Standard deviations for x, y, and psi
	std_x = *stddev;
	std_y = *(stddev + 1);
	std_theta = *(stddev + 2);

	// This line creates a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	double sample_x, sample_y, sample_theta;
	for (int i = 0; i < num_particles; ++i) {
		// sample from normal distributions
		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_theta = dist_theta(gen);		

		Particle p;
		p.id = i;
		p.x = sample_x;
		p.y = sample_y;
		p.theta = sample_theta;
		p.weight = 1.0;

		// add to the set of particles
		particles.push_back(p);

		// particle-filter initialized
		is_initialized = true;
	}
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	if (fabs(yaw_rate) < 0.0001) {
		yaw_rate = 0.0001;
	}

	//std_pos: velocity and yaw_rate measurement uncertainties
	// velocity: current timestep velocity measurement
	// yaw_rate: current timestep yaw_rate measurement
	default_random_engine gen;

	// This line creates a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(0.0, std_pos[0]);
	normal_distribution<double> dist_y(0.0, std_pos[1]);
	normal_distribution<double> dist_theta(0.0, std_pos[2]);

	for (int i = 0; i < num_particles; ++i) {		
		// predict x, y, theta using velocity and yaw_rate
        particles[i].x += (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
        particles[i].y += (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
        particles[i].theta += yaw_rate * delta_t;
        //add measurement noise
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution

	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	double weight;
	for (int i=0; i < num_particles; i++) {
		Particle& particle = particles[i];

		// initialize particle weight
		weight = 1.0;

		// transform obserations from vehicle's coordinate system to map's coordinate system, w.r.t. above "particle"
		double transformed_x, transformed_y;
		for (int j=0; j < observations.size(); j++) {
			LandmarkObs obs = observations[j];

			transformed_x = obs.x * cos(particle.theta) - obs.y * sin(particle.theta) + particle.x;
			transformed_y = obs.x * sin(particle.theta) + obs.y * cos(particle.theta) + particle.y;

			// After the observations are transformed into the map coordinate's system, the next step is to
			// associate each trasnformed observation with a landmark identifier. Find which landmark each 
			// of the observations is associated to.
			double distance = 0;
			double min_distance = sensor_range;
			Map::single_landmark_s nearest_landmark;
			for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
				Map::single_landmark_s landmark = map_landmarks.landmark_list[k];
				distance = dist(transformed_x, transformed_y, landmark.x_f,  landmark.y_f);
				if (distance < min_distance){
					min_distance = distance;
					nearest_landmark = landmark;
				}				
			} // for map_landmarks_list


			// What would happen if all landmarks are outside of sensor range? In such case, 
			// we should set the weight of this particle as zero, right?


			// Finally, compute new weight of each particle using multi-variate Gaussian
			// particle weight is computed as the product of each measurement's multivariate Gaussian probability
			// Mean of Gaussian: Associated landmark position, corresponding to "this" observation
			// Std of Gaussian: uncertainty in x and y ranges of landmarks
			double gauss_x = compute_gaussian(transformed_x, nearest_landmark.x_f, std_landmark[0]);
			double gauss_y = compute_gaussian(transformed_y, nearest_landmark.y_f, std_landmark[1]);
			double gauss = gauss_x * gauss_y;
			weight *= gauss;
		} // for observations

		// update particle weight 
		particle.weight = weight;
		weights[i] = weight;
	} // for num_particles

	// Normalize weights by their sum
	/*
		NO need to normalize weight as:
		http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
		above function used for resampling already nomalizes weights !!
	*/
	// double weight_sum = std::accumulate(weights.begin(), weights.end(), 0);
	// for (int i=0; i < num_particles; i++) {
	// 	Particle& particle = particles[i];
	// 	particle.weight = particle.weight/weight_sum;
	// 	weights[i] = particle.weight;
	// }

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;

	// Take a discrete distribution equal to weights
    discrete_distribution<> weights_distribution(weights.begin(), weights.end());
    
    // initialise new particle array
    vector<Particle> newParticles;

    // resample particles
    for (int i = 0; i < num_particles; ++i){
        newParticles.push_back(particles[weights_distribution(gen)]);
    }

    // update older particles with new ones
    particles = newParticles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

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
