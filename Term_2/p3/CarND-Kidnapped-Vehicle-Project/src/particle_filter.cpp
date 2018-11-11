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
  // Set the number of particles. Initialize all particles to first position (based on estimates of 
  //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  if (is_initialized){
    return;
  }

  default_random_engine gen;
  num_particles = 50;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  for (int i = 0; i < num_particles; i++) {
    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;  
    weights.push_back(particle.weight);
    particles.push_back(particle);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/
  std::cout << "PREDICTION" << std::endl;
  default_random_engine gen;
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);

  for (int i = 0; i < num_particles; i++) {
    if (fabs(yaw_rate) < .0001) {  
      double velocity_dt = velocity * delta_t;
      particles[i].x += velocity_dt * cos(particles[i].theta);
      particles[i].y += velocity_dt * sin(particles[i].theta);
    }
    else {
      double yaw_dt = yaw_rate * delta_t;
      particles[i].x += (velocity/yaw_rate) * ( sin(particles[i].theta + yaw_dt) - sin(particles[i].theta) );
      particles[i].y += (velocity/yaw_rate) * ( cos(particles[i].theta) - cos(particles[i].theta + yaw_dt) );
      particles[i].theta += yaw_dt;
    }
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen); 
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // Find the predicted measurement that is closest to each observed measurement and assign the 
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
  //   implement this method and use it as a helper during the updateWeights phase.
  for (int i = 0; i < observations.size(); i++) {
    double min_dist = 1.e10;
    for (int j = 0; j < predicted.size(); j++) {
      double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y); 
      if (distance < min_dist) {
        min_dist = distance;
        observations[i].id = j;
      }
    } 
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
    const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
  // Update the weights of each particle using a mult-variate Gaussian distribution. You can read
  //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  //   according to the MAP'S coordinate system. You will need to transform between the two systems.
  //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  //   The following is a good resource for the theory:
  //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //   and the following is a good resource for the actual equation to implement (look at equation 
  //   3.33
  //   http://planning.cs.uiuc.edu/node99.html

  for (int i = 0; i < num_particles; i++) {
    Particle particle = particles[i];


    // generate a list of predicted measurements, it contains all the map landmarks within the sensor range.
    std::vector<LandmarkObs> predicted;
    for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      int landmark_id = map_landmarks.landmark_list[j].id_i;
      double landmark_x = double(map_landmarks.landmark_list[j].x_f);
      double landmark_y = double(map_landmarks.landmark_list[j].y_f);
      double distance = dist(particle.x, particle.y, landmark_x, landmark_y);
      if (distance <= sensor_range) {
        LandmarkObs new_landmark;
        new_landmark.x = landmark_x;
        new_landmark.y = landmark_y;
        new_landmark.id = landmark_id; 
        predicted.push_back(new_landmark);
      } 
    }

    std::vector<LandmarkObs> obs_in_map_coordinates;
    for (int j = 0; j < observations.size(); j++) {
      LandmarkObs current_obs = observations[j];
      LandmarkObs converted_obs;
      converted_obs.x = particle.x + (cos(particle.theta)*current_obs.x) - (sin(particle.theta)*current_obs.y);
      converted_obs.y = particle.y + (sin(particle.theta)*current_obs.x) + (cos(particle.theta)*current_obs.y);
      obs_in_map_coordinates.push_back(converted_obs);
    }

    dataAssociation(predicted, obs_in_map_coordinates);

    double weight = 1.0;
    double std_x = std_landmark[0];
    double std_y = std_landmark[1];
    for (int j = 0; j < obs_in_map_coordinates.size(); j++) {
      int meas_idx = obs_in_map_coordinates[j].id;  
      double landmark_x = predicted[meas_idx].x;
      double landmark_y = predicted[meas_idx].y;
      double meas_x = obs_in_map_coordinates[j].x;
      double meas_y = obs_in_map_coordinates[j].y;
      double a = pow(meas_x - landmark_x, 2.0) / (2 * pow(std_x, 2.0));
      double b = pow(meas_y - landmark_y, 2.0) / (2 * pow(std_y, 2.0));
      double cur_weight = 1/(2*M_PI*std_x*std_y) * exp(-(a+b));
      weight *= cur_weight;
    }
    particle.weight = weight;
    weights[i] = weight;
    std::cout << "update weights" << std::endl;
  }
}


void ParticleFilter::resample() {
  // Resample particles with replacement with probability proportional to their weight. 
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  default_random_engine gen;
  discrete_distribution<int> dist(weights.begin(), weights.end());
  std::vector<Particle> new_particles;
  for (int i = 0; i < num_particles; i++) {
    int new_idx = dist(gen);
    new_particles.push_back(particles[new_idx]);
  }
  particles = new_particles;
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