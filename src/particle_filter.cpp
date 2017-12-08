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

void ParticleFilter::init(double x, double y, double theta, double std[])
{
  //Create Gaussian distributions for the GPS x, y, and theta using their standard deviations
  random_device seed;
  mt19937 gen(seed());
  normal_distribution<double> noise_x(x, std[0]);
  normal_distribution<double> noise_y(y, std[1]);
  normal_distribution<double> noise_theta(theta, std[2]);

  //Set the number of particles.
  num_particles = 10;

  //Initialize all particles based on GPS estimates with random Gaussian noise
  cout << "GPS x:" << x << "\ty:" << y << "\ttheta:" << theta << endl;
  for (int i = 0; i < num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = noise_x(gen);
    p.y = noise_y(gen);
    p.theta = noise_theta(gen);
    p.weight = 1;
    particles.push_back(p);
  }
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
  // Add measurements to each particle and add random Gaussian noise
  random_device seed;
  mt19937 gen(seed());
  double theta_d = delta_t * yaw_rate;
  double vt = (fabs(yaw_rate) > 0.001) ? velocity / theta_d : 0;
  normal_distribution<double> noise_x(0, std_pos[0]);
  normal_distribution<double> noise_y(0, std_pos[1]);
  normal_distribution<double> noise_theta(0, std_pos[2]);

  for (auto &particle : particles) {
    cout << "*x:" << particle.x << "\t" << particle.y << "\t" << particle.theta
         << endl;
    if (fabs(yaw_rate) > 0.001) {
      particle.x += vt * (sin(particle.theta + theta_d) - sin(particle.theta));
      particle.y += vt * (cos(particle.theta) - cos(particle.theta + theta_d));
    } else {
      particle.x += velocity * cos(particle.theta) * delta_t;
      particle.y += velocity * sin(particle.theta) * delta_t;
    }
    particle.x += noise_x(gen);
    particle.y += noise_y(gen);
    particle.theta += theta_d + noise_theta(gen);

    cout << " x:" << particle.x << "\t" << particle.y << "\t" << particle.theta
         << endl;
    cout << endl;
  }
}

void ParticleFilter::dataAssociation(double sensor_range,
                                     LandmarkObs &observation,
                                     const Map &map_landmarks)
{
  // Find the landmark that is closest to the observed measurement and assign the
  //   observed measurement to this particular landmark.
  double delta = 0;
  double delta_min = sensor_range;
  for (auto &landmark : map_landmarks.landmark_list) {
    delta = fabs(observation.x - landmark.x_f)
        + fabs(observation.y - landmark.y_f);
    if (delta < delta_min) {
      observation.id = landmark.id_i;
      delta_min = delta;
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
  // Update the weights of each particle using a mult-variate Gaussian distribution.
  const double PI = 4 * atan(1);
  double sig_x = std_landmark[0];
  double sig_x_sqr = pow(sig_x, 2);
  double sig_y = std_landmark[1];
  double sig_y_sqr = pow(sig_y, 2);
  double gauss_norm = (1 / (2 * PI * sig_x * sig_y));

  weights.clear();
  for (auto &particle : particles) {
    //Reset the weight
    particle.weight = 1;
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    for (auto &obsVehicle : observations) {
      LandmarkObs obsMap;

      //Translate to map coordinates
      obsMap.id = 0;
      obsMap.x = particle.x + obsVehicle.x * cos(particle.theta)
          - obsVehicle.y * sin(particle.theta);
      obsMap.y = particle.y + obsVehicle.x * sin(particle.theta)
          + obsVehicle.y * cos(particle.theta);

      //Find the closest landmark and update weight
      dataAssociation(sensor_range, obsMap, map_landmarks);
      if (obsMap.id != 0) {
        int landmarkIndex = obsMap.id - 1;
        double mu_x = map_landmarks.landmark_list[landmarkIndex].x_f;
        double mu_y = map_landmarks.landmark_list[landmarkIndex].y_f;

        //Calculate weight
        double exponent = pow((obsMap.x - mu_x), 2) / (2 * sig_x_sqr)
            + pow((obsMap.y - mu_y), 2) / (2 * sig_y_sqr);
        double weight = gauss_norm * exp(-exponent);
        cout << obsVehicle.id << " weight:" << weight << endl;
        particle.weight *= weight;

        particle.associations.push_back(obsMap.id);
        particle.sense_x.push_back(obsMap.x);
        particle.sense_y.push_back(obsMap.y);
      }
    }
    cout << particle.id << " Associations:" << getAssociations(particle)
         << " x:" << getSenseX(particle) << " y:" << getSenseY(particle)
         << endl;
    cout << "Final:" << particle.id << " Px:" << particle.x << "\tPy:"
         << particle.y << "\tW:" << particle.weight << endl;
    cout << endl;
    weights.push_back(particle.weight);
  }
}

void ParticleFilter::resample()
{
// Resample particles with replacement with probability proportional to their weight.
  vector<Particle> resampled;
  random_device seed;
  mt19937 gen(seed());
  discrete_distribution<> d(weights.begin(), weights.end());
  for (int i = 0; i < num_particles; i++) {
    Particle p = particles[d(gen)];
    cout << p.id << " Px:" << p.x << "\tPy:" << p.y << "\tW:" << p.weight
         << endl;
    resampled.push_back(p);
  }
  particles = resampled;
}

Particle ParticleFilter::SetAssociations(Particle& particle,
                                         const std::vector<int>& associations,
                                         const std::vector<double>& sense_x,
                                         const std::vector<double>& sense_y)
{
//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
// associations: The landmark id that goes along with each listed association
// sense_x: the associations x mapping already converted to world coordinates
// sense_y: the associations y mapping already converted to world coordinates

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best)
{
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best)
{
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
