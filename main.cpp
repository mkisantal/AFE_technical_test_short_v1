#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <math.h>
#include <set>

#include "wrap_to_pi.hpp"
#include "file_io.hpp"
#include "types.hpp"
#include "display.hpp"

#include "spacecraft.hpp"
#include "asteroid.hpp"

//#define INITIALIZE_ONLY // Comment out to run the full set of data

// #include <Eigen/Dense>

int main(int argc, char* argv[]) {
  /* Problem parameters */
  // double thrust = 5;   // (m/s/s) Constant acceleration (when applied)
  // double ang_vel = 1;  // (rad/s) Constant turn rate (when applied)
  // double dt = 0.2;  // (s) Time each control action (thrust or turn) is applied
                    // for
  double bullet_speed = 30;  // (m/s) Speed of the bullet, relative to the ship
                             // at the time the bullet was shot
  // int num_pixels = 360;      // (px) Number of pixels in the camera
  size_t number_of_asteroids = 7;  // Total number of asteroids

  if (argc < 3) {
    std::cout << "Please provide files for sensor data and verification, e.g. "
              << std::endl
              << "     " << argv[0] << " test_cases/measure_1.csv "
              << "test_cases/check_1.csv" << std::endl;
    std::cout << "An additional 3rd input may also be provided for "
                 "groundtruth, which then produces debugging images"
              << std::endl;
    return 1;
  }

  // Open measurement and verification files
  std::string line;
  std::ifstream measure_file((std::string(argv[1])));
  if (!measure_file.is_open())
    throw std::runtime_error("Could not open " + std::string(argv[1]));
  std::getline(measure_file, line, '\n');  // Get rid of units at top

  std::ifstream verify_file((std::string(argv[2])));
  if (!verify_file.is_open())
    throw std::runtime_error("Could not open " + std::string(argv[2]));
  std::getline(verify_file, line, '\n');  // Get rid of units at top

  // Open the true positions if provided (this prompts plotting)
  bool plot_results = false;
  std::ifstream groundtruth_file;
  if (argc > 3) {
    plot_results = true;
    groundtruth_file = std::ifstream(std::string(argv[3]));
    if (!groundtruth_file.is_open())
      throw std::runtime_error("Could not open " + std::string(argv[3]));
    std::getline(groundtruth_file, line, '\n');  // Get rid of units at topa
  }

  std::vector<SensorData> sensor_data;  // Storage location for sensor data
  double measurement_time_s;            // Time the measurement was received
  int sun_angle_px;                     // Measurement location of the sun

  std::set<size_t> destroyed_ids;  // Ids of asteroids already destroyed
  size_t iterations = 0;


  // INITIALIZATION
  get_next_sensor_data(destroyed_ids, measure_file, measurement_time_s, sun_angle_px, sensor_data);
  // Create ship
  Spacecraft spacecraft(sun_angle_px);

  // Create asteroids
  std::vector<Asteroid> asteroids;
  for (const auto& detection : sensor_data){
    Asteroid asteroid(detection, sun_angle_px);
    asteroids.push_back(asteroid);
  }


  // RUN
  // Loop through all sensor data until scenario is over, or asteroids are
  // destroyed
  while (destroyed_ids.size() < number_of_asteroids &&
         get_next_sensor_data(destroyed_ids, measure_file, measurement_time_s,
                              sun_angle_px, sensor_data) ) { //&& iterations<40

    ++iterations;

    // Propagate
    
    std::vector<std::array<double, 4>> asteroid_posvel(sensor_data.size());

    Eigen::MatrixXd ss_state = spacecraft.Propagate(sun_angle_px, false);
    for (size_t i=0; i<asteroids.size(); i++){
      asteroids[i].Propagate(sensor_data[i], sun_angle_px, ss_state, true);
    }

    if (iterations%10 == 0){
      std::cout << iterations << std::endl;
    }


    /* UPDATE THE LOCATIONS OF ASTEROID_POSVEL */
    for (size_t i=0; i<asteroids.size(); i++){
      asteroids[i].WritePosvel(asteroid_posvel[i], ss_state);
    }

    Shot shot = Shot(sensor_data.back().id, bullet_speed, asteroid_posvel.back());

    // Check for success
    size_t hit_id;
    std::pair<double, double> shot_lim;
    bool hit = check_shot(shot, destroyed_ids, verify_file, hit_id, shot_lim);

    /* YOU MAY UPDATE YOUR ALGORITHM WITH THE HIT */
    if (hit){
      std::cout << "💥 Asteroid  " << hit_id << " destroyed!" << std::endl;
      for (size_t i=0; i<asteroids.size(); i++){
        if (hit_id == asteroids[i].id){
          asteroids.erase(asteroids.begin() + i);
          break;
        }
      }
    }

    // Plot results


    if (plot_results) {
      display_results(groundtruth_file, asteroid_posvel, shot, shot_lim, hit, iterations);
    }
  }
  std::cout << "You shot: " << destroyed_ids.size() << "/"
            << number_of_asteroids << " asteroids in " << iterations
            << " iterations!" << std::endl;
}
