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

int debug = 0;
void ParticleFilter::init(double x, double y, double theta, double std[]) {
	if (debug) cout << "init " << endl;
        default_random_engine gen;
	gen.seed(123);
        num_particles = 1000;
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

        normal_distribution<double> dist_x(x, std[0]);
        normal_distribution<double> dist_y(y, std[1]);
        normal_distribution<double> dist_psi(theta, std[2]);

        for (int i = 0; i < num_particles; ++i) {
		Particle p = Particle();
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_psi(gen);
		p.weight = 1.0;
		particles.push_back(p);
		weights.push_back(p.weight);
		//cout << "initValue:" << p.x << "," << p.y << "," << p.theta << endl;
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	if (debug) cout << "prediction " << endl;
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;

	vector<Particle> pList;
	for (int i = 0; i < num_particles; ++i) {
		Particle p = particles[i];

		if (yaw_rate == 0) {
			yaw_rate = 0.00001;
		}
		double dt = delta_t;
		double x = p.x + (velocity / yaw_rate) * (sin(p.theta + yaw_rate * dt) - sin(p.theta));
		double y = p.y + (velocity / yaw_rate) * (cos(p.theta) - cos(p.theta +  yaw_rate * dt));
		double theta = p.theta + yaw_rate * dt;

        	normal_distribution<double> dist_x(x , std_pos[0]);
        	normal_distribution<double> dist_y(y, std_pos[1]);
        	normal_distribution<double> dist_psi(theta, std_pos[2]);

		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_psi(gen);
		
		pList.push_back(p);
		//cout << "Pred:" << p.x << "," << p.y << "," << "," << p.theta << endl;
	}
	particles.clear();
	particles = pList;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	if (debug) cout << "data association " << endl;
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (int i = 0; i < observations.size(); ++i) {
		observations[i].id = -1;
		double minId = -1;
		double minDist = 10000000;
		for (int j = 0; j < predicted.size(); ++j) {
			double dx = observations[i].x - predicted[j].x;
			double dy = observations[i].y - predicted[j].y;
			double dist = dx * dx + dy * dy;
			
			if (dist < minDist) {
				minDist = dist;
				observations[i].id = j;
			}	
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	if (debug) cout << "updateWeights " << endl;
	double sensor_range2 = sensor_range * sensor_range;
	double sx = std_landmark[0];
	double sy = std_landmark[1];
	sx = sx == 0 ? 0.1 : sx;
	sy = sy == 0 ? 0.1 : sy;
	double sx2 = 2 * sx * sx;
	double sy2 = 2 * sy * sy;

	vector<Particle> pList;
	vector<double> wList;
	vector<LandmarkObs> pred;

	// convert land mark to LandmarkObs vector for pred
	for (int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
		LandmarkObs obj;
                obj.id = map_landmarks.landmark_list[j].id_i;
		obj.x = (double)map_landmarks.landmark_list[j].x_f;
		obj.y = (double)map_landmarks.landmark_list[j].y_f;
		pred.push_back(obj);
	}

	for (int i = 0; i < num_particles; i++) {
		Particle p = particles[i];
		std:vector<LandmarkObs> obsInMap;
		for (int j = 0; j < observations.size(); j++) {
			LandmarkObs o = observations[j];
			LandmarkObs newO;
			newO.x = p.x + o.x * cos(p.theta) - o.y * sin(p.theta);
			newO.y = p.y + o.x * sin(p.theta) + o.y * cos(p.theta);

			double dx = newO.x - p.x;
			double dy = newO.y - p.y;
			if (dx * dx + dy * dy <= sensor_range2) {
				obsInMap.push_back(newO);
			}
		}
		dataAssociation(pred, obsInMap); 

		p.weight = 1.0;
		for (int j = 0; j < obsInMap.size(); ++j) {
			int id = obsInMap[j].id;
			LandmarkObs preObj = pred[id];

			double dx = preObj.x - obsInMap[j].x;
			double dy = preObj.y - obsInMap[j].y;
			double dx2 = dx * dx;
			double dy2 = dy * dy;

			double v0 = 2 * M_PI * sx * sy;
			double v1 = -(dx2 / sx2 + dy2 / sy2);	
			double change = exp (v1) / v0;
			p.weight *= change;
		//	cout << dx << "," << dy << " -> " << change << " -> " << p.weight<< endl;
		}
		pList.push_back(p);
		wList.push_back(p.weight);
	}
	particles.clear();
	particles = pList;
	weights.clear();
	weights = wList;
}

void ParticleFilter::resample() {
    	default_random_engine gen;
    	discrete_distribution<> distribution(weights.begin(), weights.end());

	vector<Particle> pList; 
    	for (int i = 0; i < num_particles; ++i) {
       		int idx = distribution(gen);
        	pList.push_back(particles[idx]);
		//cout << idx << "," << particles[idx].weight << endl;
    	}

	particles.clear();
	particles = pList;
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
