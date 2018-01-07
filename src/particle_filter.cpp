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
#include <cfloat>		// For using DBL_MAX

#include "particle_filter.h"

using namespace std;

//Initialize random engine
default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	

	// Initilize num_particles
	num_particles = 200;



	// Inherit the std values from std[]
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];



	// Generate normal distribution to x, y and theta
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);


	
	//Add random Gaussian noise and weight to each particle
	for (unsigned int i = 0; i < num_particles; i++){

		Particle pt;

		pt.id = i;
		pt.x = dist_x(gen);
		pt.y = dist_y(gen);
        pt.theta = dist_theta(gen);
        pt.weight = 1.0;

        particles.push_back(pt);
	}
	is_initialized = true;
	cout<<"Initialization is Done!"<<endl;
}


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	double delta_theta = delta_t * yaw_rate ;

	//Initialize random engine
	//default_random_engine gen;

	// Generate normal distribution to x, y and theta
	normal_distribution<double> nd_x(0, std_pos[0]);
	normal_distribution<double> nd_y(0, std_pos[1]);
	normal_distribution<double> nd_theta(0, std_pos[2]);

	
	//Add random Gaussian noise and weight to each particle
	for (unsigned int i = 0; i < num_particles; i++){
		double th0 = particles[i].theta;

		if (fabs(yaw_rate) < 0.00001) {  
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
	   	    particles[i].y += velocity * delta_t * sin(particles[i].theta);
    	} else{
			particles[i].x += velocity / yaw_rate * (sin(delta_theta + th0) - sin(th0)) + nd_x(gen);
			particles[i].y += velocity / yaw_rate * (-cos(delta_theta + th0) + cos(th0)) + nd_y(gen);
    	}
        particles[i].theta += delta_theta + nd_theta(gen);

	}
	cout<<"Prediction Done"<<endl;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	if (predicted.size() <= 0){
		cout<<"Predicted landmarks size Zero!!"<<endl;
	} 

	for(auto& obs : observations){
		double min_dist = 0;
		for (unsigned int i = 0; i < predicted.size(); i++){
			double dist = sqrt(pow(obs.x - predicted[i].x, 2) + pow(obs.y - predicted[i].y, 2));
			min_dist = min(min_dist, dist);
			obs.id = (min_dist == dist) ? predicted[i].id:obs.id;
		}
	}
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

	// calculate normalization term
	double sig_x = std_landmark[0];
	double sig_y = std_landmark[1];
	double gauss_norm = 1/(2 * M_PI * sig_x * sig_y);

	for (unsigned int i = 0; i < num_particles; i++){

		double total_weight = 1.0;

		for (auto obs : observations){
			double r = sqrt(pow(obs.x, 2) + pow(obs.y, 2));
			if (r <= sensor_range){

				// Transform to MAP coord.
				double sin_th = sin(particles[i].theta);
				double cos_th = cos(particles[i].theta);
				double obs_mx = particles[i].x + cos_th * obs.x - sin_th * obs.y;
				double obs_my = particles[i].y + sin_th * obs.x + cos_th * obs.y;

				// Find the closest neighbor in Map
				double min_dist = DBL_MAX;

				int index = -1;
				for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++){
					Map::single_landmark_s m = map_landmarks.landmark_list[j];
					double dist = pow(obs_mx - m.x_f, 2) + pow(obs_my - m.y_f, 2);
					if (dist < min_dist){
						min_dist = dist;
						obs.id = m.id_i;
						index = j;
					}
				}

				cout<<min_dist<<endl;

				if (index != -1){
					Map::single_landmark_s m = map_landmarks.landmark_list[index];

					// calculate exponent
					double exponent= pow(obs_mx - m.x_f, 2)/2.0 / pow(sig_x, 2) + pow(obs_my - m.y_f, 2)/2.0 / pow(sig_y, 2);

					// calculate weight using normalization terms and exponent
					//cout<<gauss_norm<<"\t"<<-exponent<<endl;
					total_weight *= gauss_norm * exp(-exponent);
					//cout<<"total_weight:"<<total_weight<<endl;
				}
			}
		}

		particles[i].weight = total_weight;
	}
	cout<<"update weights Done"<<endl;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// Find the highest weight throught the particles
  	double highest_weight = 0.0;
  	for (int i = 0; i < num_particles; ++i) {
		if (particles[i].weight > highest_weight) {
			highest_weight = particles[i].weight;
		}
	}

	// Resample
	vector<Particle> particles_resample;

	double beta = 0.0;

	// Get a random index from [0, n-1]
	uniform_int_distribution<int> uni_index(0, num_particles-1);

	int index =  uni_index(gen);


	// uniform random distribution [0.0, max_weight)
  	uniform_real_distribution<double> unirealdist(0.0, highest_weight);


  	for (int i = 0; i < num_particles; ++i) {
  		beta += unirealdist(gen) * 2.0;
  		while(beta > particles[index].weight){
  			beta -= particles[index].weight;
  			index = (index + 1) % num_particles;
  		}

  		particles_resample.push_back(particles[index]);
  	}
  	particles = particles_resample;

  	cout<<"Resample Done"<<endl;
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
