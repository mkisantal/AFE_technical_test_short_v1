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

#define INITIALIZE_ONLY // Comment out to run the full set of data

// #include <Eigen/Dense>

int main(int argc, char* argv[]) {
  /* Problem parameters */
  double thrust = 5;   // (m/s/s) Constant acceleration (when applied)
  double ang_vel = 1;  // (rad/s) Constant turn rate (when applied)
  double dt = 0.2;  // (s) Time each control action (thrust or turn) is applied
                    // for
  double bullet_speed = 30;  // (m/s) Speed of the bullet, relative to the ship
                             // at the time the bullet was shot
  int num_pixels = 360;      // (px) Number of pixels in the camera
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


#ifdef INITIALIZE_ONLY
  /* Use this case to only run the first few steps for initialization */
  size_t initialization_iterations; // Change to steps required for your initialization procedure
  while (iterations < initialization_iterations) {
#else
  /* Use this case to run the full dataset */
  // Loop through all sensor data until scenario is over, or asteroids are
  // destroyed
  while (destroyed_ids.size() < number_of_asteroids &&
         get_next_sensor_data(destroyed_ids, measure_file, measurement_time_s,
                              sun_angle_px, sensor_data)) {
#endif

    ++iterations;

    // Fill in this vector with relative positions and velocities of the
    // asteroids, in the coordinate frame of the space ship.
    // Coordinate system:
    // x-axis: forward of ship
    // y-axis: left of ship
    // position and velocity stored as (x, y, vx, vy) relative to the ship
    std::vector<std::array<double, 4>> asteroid_posvel(sensor_data.size());

    /* UPDATE THE LOCATIONS OF ASTEROID_POSVEL */
    //    your_update_function(measurement_time_s, sensor_data, sun_angle_px, asteroid_posvel);

    Shot shot = Shot(sensor_data.back().id, bullet_speed, asteroid_posvel.back());


    // Check for success
    size_t hit_id;
    std::pair<double, double> shot_lim;
    bool hit = check_shot(shot, destroyed_ids, verify_file, hit_id, shot_lim);

    /* YOU MAY UPDATE YOUR ALGORITHM WITH THE HIT */
    // your_hit_function(shot, hit)

    // Print results
    if (hit) {
      std::cout << "HIT!! You destroyed asteroid: " << hit_id << std::endl;
    } else {
    std::cout << "You missed asteroid: " << shot.id << std::endl;
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
