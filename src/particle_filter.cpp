/**
 * particle_filter.cpp
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
using std::random_device;
using std::default_random_engine;
using std::normal_distribution;
using std::discrete_distribution;

/**
 * Set the number of particles. Initialize all particles to first position (based on estimates of x, y, theta and
 * their uncertainties from GPS) and all weights to 1.
 *
 * Add random Gaussian noise to each particle.
 *
 * NOTE: Consult particle_filter.h for more information about this method (and others in this file).
 */
void ParticleFilter::init(double x, double y, double theta, double std[])
{
    num_particles = 100;

    // Resize weights vector based on num_particles
    weights.resize(num_particles);

    // Resize vector of particles
    particles.resize(num_particles);

    // Engine for later generation of particles
    random_device rd;
    default_random_engine gen(rd());

    // Creates a normal (Gaussian) distribution for x, y and theta (yaw).
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    // Initializes particles from the normal distributions set above
    for(int i = 0; i < num_particles; ++i)
    {

        // Add generated particle data
        particles[i].id = i;
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
        particles[i].weight = 1.0;
    }

    is_initialized = true;
}

/**
 * Add measurements to each particle and add random Gaussian noise. When adding noise std::normal_distribution
 * and std::default_random_engine are useful.
 * http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
 * http://www.cplusplus.com/reference/random/default_random_engine/
 */
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    // Engine for later generation of particles
    default_random_engine particle_generator;

    // Make distributions for adding noise
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    // Different equations based on if yaw_rate is zero or not
    for(int i = 0; i < num_particles; ++i)
    {
        if(abs(yaw_rate) != 0)
        {
            // Add measurements to particles
            particles[i].x +=
                    (velocity / yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
            particles[i].y +=
                    (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)));
            particles[i].theta += yaw_rate * delta_t;
        }
        else
        {
            // Add measurements to particles
            particles[i].x += velocity * delta_t * cos(particles[i].theta);
            particles[i].y += velocity * delta_t * sin(particles[i].theta);
            // Theta will stay the same due to no yaw_rate
        }

        // Add noise to the particles
        particles[i].x += dist_x(particle_generator);
        particles[i].y += dist_y(particle_generator);
        particles[i].theta += dist_theta(particle_generator);
    }

}

/**
 * Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to
 * this particular landmark.
 *
 * NOTE: this method will NOT be called by the grading code. But you will probably find it useful to implement this
 * method and use it as a helper during the updateWeights phase.
 */
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs> & observations)
{
}

/**
 * Update the weights of each particle using a multi-variate Gaussian distribution.
 * You can read more about this distribution here:
 * https://en.wikipedia.org/wiki/Multivariate_normal_distribution
 *
 * NOTE: The observations are given in the VEHICLE'S coordinate system.
 * Your particles are located according to the MAP'S coordinate system.
 * You will need to transform between the two systems. Keep in mind that
 * this transformation requires both rotation AND translation (but no scaling).
 *
 * The following is a good resource for the theory:
 * https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
 *
 * and the following is a good resource for the actual equation to implement
 * (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
 */
void ParticleFilter::updateWeights(double sensor_range,
                                   double std_landmark[],
                                   const vector<LandmarkObs> & observations,
                                   const Map & map_landmarks)
{
    // First term of multi-variate normal Gaussian distribution calculated below
    // It stays the same so can be outside the loop
    const double a = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);

    // The denominators of the multi-variate normal Gaussian distribution also stay the same
    const double x_denominator = 2 * std_landmark[0] * std_landmark[0];
    const double y_denominator = 2 * std_landmark[1] * std_landmark[1];

    // Iterate through each particle
    for(int i = 0; i < num_particles; ++i)
    {
        // For calculating multi-variate Gaussian distribution of each observation, for each particle
        double multiVariateGaussianDistribution = 1.0;

        // For each observation
        for(int j = 0; j < observations.size(); ++j)
        {
            // Transform the observation point (from vehicle coordinates to map coordinates)
            double trans_obs_x, trans_obs_y;
            trans_obs_x = observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta) +
                          particles[i].x;
            trans_obs_y = observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta) +
                          particles[i].y;

            // Find nearest landmark
            vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
            vector<double> landmark_obs_dist(landmarks.size());

            for(int k = 0; k < landmarks.size(); ++k)
            {
                // Down-size possible amount of landmarks to look at by only looking at those in sensor range of the particle
                // If in range, put in the distance vector for calculating nearest neighbor
                double landmark_part_dist = sqrt(
                        pow(particles[i].x - landmarks[k].x_f, 2) + pow(particles[i].y - landmarks[k].y_f, 2));

                if(landmark_part_dist <= sensor_range)
                {
                    landmark_obs_dist[k] = sqrt(
                            pow(trans_obs_x - landmarks[k].x_f, 2) + pow(trans_obs_y - landmarks[k].y_f, 2));
                }
                else
                {
                    // Need to fill those outside of distance with huge number, or they'll be a zero (and think they are closest)
                    landmark_obs_dist[k] = 999999.0;
                }
            }

            // Associate the observation point with its nearest landmark neighbor
            int min_pos = distance(landmark_obs_dist.begin(),
                                   min_element(landmark_obs_dist.begin(), landmark_obs_dist.end()));
            float nn_x = landmarks[min_pos].x_f;
            float nn_y = landmarks[min_pos].y_f;

            // Calculate multi-variate Gaussian distribution
            double x_diff = trans_obs_x - nn_x;
            double y_diff = trans_obs_y - nn_y;
            double b = ((x_diff * x_diff) / x_denominator) + ((y_diff * y_diff) / y_denominator);

            multiVariateGaussianDistribution *= a * exp(-b);
        }

        // Update particle weights with combined multi-variate Gaussian distribution
        particles[i].weight = multiVariateGaussianDistribution;
        weights[i] = particles[i].weight;
    }
}

/**
 * Resample particles with replacement with probability proportional to their weight.
 *
 * NOTE: std::discrete_distribution helpful here.
 *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
 */
void ParticleFilter::resample()
{
    // Vector for new particles
    vector<Particle> new_particles(num_particles);

    // Use discrete distribution to return particles by weight
    random_device rd;
    default_random_engine gen(rd());

    for(int i = 0; i < num_particles; ++i)
    {
        discrete_distribution<int> index(weights.begin(), weights.end());
        new_particles[i] = particles[index(gen)];
    }

    // Replace old particles with the resampled particles
    particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle & particle,
                                     const vector<int> & associations,
                                     const vector<double> & sense_x,
                                     const vector<double> & sense_y)
{
    // particle: the particle to which assign each listed association, and association's (x,y) world coordinates mapping
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
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space

    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
    vector<double> v;

    if(coord == "X")
    {
        v = best.sense_x;
    }
    else
    {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}