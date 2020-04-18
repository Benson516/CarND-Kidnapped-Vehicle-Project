/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  num_particles = 1000;  // TODO: Set the number of particles

  // Random generator
  std::default_random_engine gen;

  // These create normal (Gaussian) distributions
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  // Initialize the particles
  for (int i=0; i < num_particles; ++i){
      Particle a_particle;
      // Sample from these normal distributions
      a_particle.x = dist_x(gen);
      a_particle.y = dist_y(gen);
      a_particle.theta = dist_theta(gen);
      a_particle.weight = 1.0; // Initialize the weight to 1.0
      particles.push_back(a_particle);
      //
      // Print your samples to the terminal.
      // std::cout << "Sample " << i + 1 << " " << a_particle.x << " " << a_particle.y << " "
      //         << a_particle.theta << std::endl;
  }

  // Initialize. the weight of particles
  // weights.resize(num_particles, 1.0); // Note: since the Particle contains weight, this is not necessary

  // Update state
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

   // Noise (Gaussian)
   //-----------------------------//
   // Random generator
   static std::default_random_engine gen;
   // These create normal (Gaussian) distributions (zero mean)
   std::vector< std::normal_distribution<double> > dist_list(3);
   for (size_t i=0; i < 3; ++i){
       dist_list.emplace_back(0.0, std_pos[i]);
   }
   //-----------------------------//

   // Prediction + noise
   double wdt = yaw_rate * delta_t;
   if ( fabs(yaw_rate) < 0.001){
       // yaw_rate --> 0.0
       double vdt = velocity * delta_t;
       for (size_t i=0; i < particles.size(); ++i){
           particles[i].x += vdt * cos(particles[i].theta); // + dist_list[0](gen);
           particles[i].y += vdt * sin(particles[i].theta); // + dist_list[1](gen);
           particles[i].theta += wdt; //  + dist_list[2](gen); // Since the yaw rate is not exactly zero.
       }
   }else{
       double v_w = velocity/yaw_rate;
       for (size_t i=0; i < particles.size(); ++i){
           double theta_1 = particles[i].theta + wdt;
           particles[i].x += v_w * ( sin(theta_1) - sin(particles[i].theta) ); // + dist_list[0](gen);
           particles[i].y += v_w * ( cos(particles[i].theta) - cos(theta_1) ); // + dist_list[1](gen);
           particles[i].theta = theta_1; // + dist_list[2](gen);
       }
   }

   // Add noise (separate the this part for examining the effect of noise)
   for (size_t i=0; i < particles.size(); ++i){
       particles[i].x += dist_list[0](gen);
       particles[i].y += dist_list[1](gen);
       particles[i].theta += dist_list[2](gen);
   }

}

void ParticleFilter::dataAssociation(const vector<LandmarkObs> &predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */
   // Note: this is actually performed in the vehicle coordinate.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a multi-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

   // For each particle
   for (size_t i=0; i < particles.size(); ++i){
       // transformation: from vehicle frame to map frame
       vector<LandmarkObs> observation_map_list;
       // For each landmark observed
       for (size_t j=0; j < observations.size(); ++j){
           LandmarkObs an_observation;
           double c_t = cos(particles[i].theta);
           double s_t = sin(particles[i].theta);
           an_observation.x = c_t * observations[j].x - s_t * observations[j].y + particles[i].x;
           an_observation.y = s_t * observations[j].x + c_t * observations[j].y + particles[i].y;
           observation_map_list.push_back( an_observation );
       }
       // Association
       // Update weights
   }

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
