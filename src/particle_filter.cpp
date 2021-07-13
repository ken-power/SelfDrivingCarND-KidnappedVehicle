/**
 * particle_filter.cpp
 */

#include "particle_filter.h"

#include <cmath>
#include <algorithm>
#include <iterator>
#include <random>
#include <string>
#include <vector>
#include <iostream>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::random_device;
using std::default_random_engine;
using std::normal_distribution;
using std::discrete_distribution;
using std::numeric_limits;
using std::cout;
using std::endl;


/**
 * Set the number of particles. Initialize all particles to first position (based on estimates of x, y, theta and
 * their uncertainties from GPS) and all weights to 1.
 *
 * Add random Gaussian noise to each particle.
 *
 * NOTE: Consult particle_filter.h for more information about this method (and others in this file).
 *
 * (x,y) is a GPS position
 * theta is an initial heading estimate
 * std[] is an array of uncertainties for these measurements
 */
void ParticleFilter::Initialize(double x, double y, double theta, double *std)
{
    num_particles = 101;

    // Resize weights vector based on num_particles
    weights.resize(num_particles);

    // Resize vector of particles
    particles.resize(num_particles);

    // Particle generator
    random_device rd;
    default_random_engine particle_generator(rd());

    // Creates a normal (Gaussian) distribution for x, y and theta (yaw).
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    // Initializes particles from the normal distributions set above
    for(int i = 0; i < num_particles; i++)
    {
        particles[i].id = i;
        particles[i].x = dist_x(particle_generator);
        particles[i].y = dist_y(particle_generator);
        particles[i].theta = dist_theta(particle_generator);
        particles[i].weight = 1.0;
    }

    is_initialized = true;
}

/**
 * Add measurements to each particle and add random Gaussian noise. When adding noise std::normal_distribution
 * and std::default_random_engine are useful.
 * http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
 * http://www.cplusplus.com/reference/random/default_random_engine/
 *
 * delta_t is the amount of time between time steps
 * std_pos[] is the measurement uncertainties associated with the velocity and yaw rate
 * velocity is the current time step's velocity measurement
 * yaw_Rate is the current time step's yaw rate measurement
 */
void ParticleFilter::Prediction(double delta_t, double *std_pos, double velocity, double yaw_rate)
{
    // Make distributions for adding noise
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    // Engine for generating particles
    default_random_engine particle_generator;

    for(int i = 0; i < num_particles; i++)
    {
        if(fabs(yaw_rate) != 0)
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
            // Theta will stay the same if there no yaw_rate, or if yaw_rate is too small
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
 * method and use it as a helper during the UpdateWeights phase.
 */
void ParticleFilter::DataAssociation(vector<LandmarkObservationMeasurement> predicted,
                                     vector<LandmarkObservationMeasurement> & observations)
{
    for(auto & observation : observations)
    {
        // grab current observation
        LandmarkObservationMeasurement observation_measurement = observation;

        // init minimum distance to maximum possible
        double min_dist = numeric_limits<double>::max();

        // init id of landmark from map placeholder to be associated with the observation
        int map_id = -1;

        for(auto prediction_measurement : predicted)
        {
            // grab current prediction
            // get distance between current/predicted landmarks
            double distance = dist(observation_measurement.x,
                                   observation_measurement.y,
                                   prediction_measurement.x,
                                   prediction_measurement.y);

            // find the predicted landmark nearest the current observed landmark
            if(distance < min_dist)
            {
                min_dist = distance;
                map_id = prediction_measurement.id;
            }
        }

        // set the observation's id to the nearest predicted landmark's id
        observation.id = map_id;
    }
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
void ParticleFilter::UpdateWeights(double sensor_range,
                                   const double *std_landmark,
                                   const vector<LandmarkObservationMeasurement> & observations,
                                   const Map & map_landmarks)
{
    // First term of multi-variate Gaussian distribution
    const double a = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);

    // The denominators of the multi-variate normal Gaussian distribution also stay the same
    const double x_denominator = 2 * std_landmark[0] * std_landmark[0];
    const double y_denominator = 2 * std_landmark[1] * std_landmark[1];

    // Iterate through each particle
    for(int i = 0; i < num_particles; i++)
    {
        double observation_weight = 1.0;

        // For each observation ...
        for(const auto & observation : observations)
        {
            // Transform the observation point (from vehicle coordinates to map coordinates)
            double transform_observation_pt_x =
                    observation.x * cos(particles[i].theta) - observation.y * sin(particles[i].theta) +
                    particles[i].x;
            double transform_observation_pt_y =
                    observation.x * sin(particles[i].theta) + observation.y * cos(particles[i].theta) +
                    particles[i].y;

            // Find nearest landmark
            vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
            vector<double> landmark_observation_dist(landmarks.size());

            // For each landmark ...
            for(int k = 0; k < landmarks.size(); k++)
            {
                // Reduce number of landmarks to those in sensor range of the particle
                double landmark_part_dist = sqrt(
                        pow(particles[i].x - landmarks[k].x_f, 2) + pow(particles[i].y - landmarks[k].y_f, 2));

                // If landmark is in sensor range put in the distance vector for calculating nearest neighbor
                if(landmark_part_dist <= sensor_range)
                {
                    landmark_observation_dist[k] = sqrt(
                            pow(transform_observation_pt_x - landmarks[k].x_f, 2) +
                            pow(transform_observation_pt_y - landmarks[k].y_f, 2));
                }
                else
                {
                    // Need to fill those outside of distance with huge number, or they'll be a zero (and think they are closest)
                    landmark_observation_dist[k] = 1000000.0;
                }
            }

            // Associate the observation point with its nearest landmark neighbor
            int min_pos = distance(landmark_observation_dist.begin(),
                                   min_element(landmark_observation_dist.begin(), landmark_observation_dist.end()));
            float nearest_neighbor_x = landmarks[min_pos].x_f;
            float nearest_neighbor_y = landmarks[min_pos].y_f;

            // Calculate multi-variate Gaussian distribution
            double x_diff = transform_observation_pt_x - nearest_neighbor_x;
            double y_diff = transform_observation_pt_y - nearest_neighbor_y;
            double b = ((x_diff * x_diff) / x_denominator) + ((y_diff * y_diff) / y_denominator);

            observation_weight *= a * exp(-b);
        }

        // Update particle weights with combined multi-variate Gaussian distribution
        particles[i].weight = observation_weight;
        weights[i] = particles[i].weight;
    }
}

/**
 * Resample particles with replacement with probability proportional to their weight. Resamples from the updated set
 * of particles to form the new set of particles.
 *
 * NOTE: std::discrete_distribution helpful here.
 * http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
 */
void ParticleFilter::Resample()
{
    vector<Particle> new_particles(num_particles);

    // Use discrete distribution to return particles by weight
    random_device rd;
    default_random_engine particle_generator(rd());

    for(int i = 0; i < num_particles; i++)
    {
        discrete_distribution<int> index(weights.begin(), weights.end());
        new_particles[i] = particles[index(particle_generator)];
    }

    // Replace old particles with the resampled particles
    particles = new_particles;
}

/**
 *
 * @param particle associate this particle with the given associations and (x,y) world coordinates
 * @param associations The landmark id that goes along with each listed association
 * @param sense_x the associations x mapping already converted to world coordinates
 * @param sense_y the associations y mapping already converted to world coordinates
 */
void ParticleFilter::SetAssociations(Particle & particle,
                                     const vector<int> & associations,
                                     const vector<double> & sense_x,
                                     const vector<double> & sense_y)
{
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::GetAssociations(const Particle & best)
{
    vector<int> v = best.associations;
    std::stringstream ss;

    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space

    return s;
}

string ParticleFilter::GetSenseCoord(const Particle & best, const string & coord)
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
