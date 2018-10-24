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




inline double distance(double x1, double y1, double x2, double y2) {
	auto x_dist = x1 - x2;
	auto y_dist = y1 - y2;
	return sqrt(x_dist*x_dist + y_dist*y_dist);
}

inline long double multivariateGaussProb(double sigmaX, double sigmaY, double x, double y, double mux, double muy) {
    double xTerm = pow(x - mux, 2) / pow(sigmaX, 2) / 2;
    double yTerm = pow(y - muy, 2) / pow(sigmaY, 2) / 2;    
    return exp(-(xTerm + yTerm)) / (2. * M_PI * sigmaX * sigmaY);
}

inline LandmarkObs transformObservation(Particle& particle, LandmarkObs& observation) {
    LandmarkObs transformedObs;    
    transformedObs.x = particle.x + (observation.x*cos(particle.theta) - observation.y*sin(particle.theta));
    transformedObs.y = particle.y + (observation.x*sin(particle.theta) + observation.y*cos(particle.theta));    
    return transformedObs;
}






void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	num_particles = 10;
	weights.resize(num_particles);
	default_random_engine gen;

	// Create normal (Gaussian) distributions for x, y, theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < num_particles; i++) {
		Particle new_particle;
		new_particle.id = i;
		new_particle.x = dist_x(gen);
		new_particle.y = dist_y(gen);
		new_particle.theta = dist_theta(gen);
		new_particle.weight = 1.0;
        
        // Add particle to list of particles
        particles.push_back(new_particle);
	}
	is_initialized = true;
}



void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    default_random_engine gen;
    
    for (Particle& particle: particles) {
        double predictionX;
        double predictionY;
        double predictionTh;
        
        if (yaw_rate == 0) {
            predictionTh = particle.theta;
            predictionX = particle.x + velocity * delta_t * cos(particle.theta);
            predictionY = particle.y + velocity * delta_t * sin(particle.theta);
        } else {
            predictionTh = particle.theta + yaw_rate * delta_t;
            predictionX = particle.x + velocity / yaw_rate * (sin(predictionTh) - sin(particle.theta));
            predictionY = particle.y + velocity / yaw_rate * (cos(particle.theta) - cos(predictionTh));
        }        
       
        normal_distribution<double> dist_x(predictionX, std_pos[0]);
		normal_distribution<double> dist_y(predictionY, std_pos[1]);
		normal_distribution<double> dist_theta(predictionTh, std_pos[2]);

		particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
    }
}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    // not used 
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks) {
    
    for (Particle& particle: particles) {
        // Set particle weight to 1.
	    particle.weight = 1;
	    
	    vector<int> associations;
	    vector<double> senseX;
	    vector<double> senseY;
	    
	    // Analyze observations and update weight 
	    for (LandmarkObs& obs:observations) {
	        
	        LandmarkObs transformedObs = transformObservation(particle, obs);
	        
	        // Find closest landmark location 
	        ////////////////////////////////////////////////////////////////////////////////////////////////
		    double minDist = sensor_range;
		    LandmarkObs closest_landmark;
		    bool found_closest_landmark = false;
		    
		    for (auto& landmark: map_landmarks.landmark_list) {
		        double currrentDist = distance(landmark.x_f, landmark.y_f, transformedObs.x, transformedObs.y);
		        if (currrentDist < minDist) {
		        	found_closest_landmark = true;
		            minDist = currrentDist;
		            closest_landmark.x = landmark.x_f;
		            closest_landmark.y = landmark.y_f;
		            closest_landmark.id = landmark.id_i; 
		        }
		    }
	        ////////////////////////////////////////////////////////////////////////////////////////////////  
	        if (found_closest_landmark) {
            
	            double obsMultiVariateGaussProb = multivariateGaussProb(
	            	std_landmark[0], std_landmark[1], 
	            	transformedObs.x, transformedObs.y, 
	            	closest_landmark.x, closest_landmark.y);
	            if (obsMultiVariateGaussProb > 0) {
	                // Update particle's weight
	                particle.weight *= obsMultiVariateGaussProb;
	                
	                // save associations and transformed observations
	                associations.push_back(closest_landmark.id);
	                senseX.push_back(transformedObs.x);
	                senseY.push_back(transformedObs.y);
	            }
	        }
	        ////////////////////////////////////////////////////////////////////////////////////////////////
	    }	    
	    // Set associations (for visualization purposes).
	    SetAssociations(particle, associations, senseX, senseY);    
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    default_random_engine gen;
    
    std::vector<double> weights;
    for (Particle& p: particles) {
        weights.push_back(p.weight);
    }
    
    discrete_distribution<int> distribution(weights.begin(), weights.end());    
    vector<Particle> resample_particles;
    
    for (int i = 0; i < num_particles; ++i) 
        resample_particles.push_back(particles[distribution(gen)]);    
    
    particles = resample_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, const std::vector<double>& sense_x, const std::vector<double>& sense_y){
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    //Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    //cout << associations.size() << sense_x.size() << sense_y.size() << endl;

	return particle;
}

string ParticleFilter::getAssociations(Particle best){
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best){
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best){
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
