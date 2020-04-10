#ifndef FILE_IO_HPP
#define FILE_IO_HPP

#include <vector>
#include <set>
#include <fstream>

#include "wrap_to_pi.hpp"
#include "types.hpp"

/** @brief Return sensor data from the next timestep
 *
 * @param destroyed_ids Set of asteroids already shot, we don't get sensor data
 * from them
 * @param file File to read sensor data from
 * @param measurement_time_s (s) Time of the measurement data
 * @param sun_angle_px (px) Pixel the sun was seen in
 * @param sensor_data (SensorData) Vector of data from asteroids (see types.hpp)
 *
 * @return Whether data was available in file
 */
static bool get_next_sensor_data(const std::set<size_t>& destroyed_ids,
                                 std::ifstream& file,
                                 double& measurement_time_s, int& sun_angle_px,
                                 std::vector<SensorData>& sensor_data) {
  sensor_data.clear();

  std::string line;
  if (std::getline(file, line, '\n')) {
    std::stringstream ss_line(line);
    std::string token;

    // Get time of measurement
    std::getline(ss_line, token, ',');
    measurement_time_s = std::stod(token);

    // Get sun angle
    std::getline(ss_line, token, ',');
    sun_angle_px = std::stoi(token);

    // Get measurements
    while (std::getline(ss_line, token, ',')) {
      SensorData data;
      data.id = std::stoi(token);
      if (!std::getline(ss_line, token, ','))
        throw std::runtime_error(
            "Sensor data file did not match the expected data dimension");
      data.range = std::stod(token);
      if (!std::getline(ss_line, token, ','))
        throw std::runtime_error(
            "Sensor data file did not match the expected data dimension");
      data.bearing = std::stod(token);

      if (destroyed_ids.find(data.id) == destroyed_ids.end()) {
        sensor_data.push_back(data);
      }
    }
    return true;  // Still had data
  } else {
    return false;  // File over
  }
}

/** @brief Check whether the shot hits the intended asteroid
 *
 * @param shot Shot taken, see types.hpp
 * @param destroyed_ids (return) Set of destroyed asteroids
 * @param file File to read correct shooting angles from
 * @param hit_id Id of the asteroid hit
 * @param shot_lim (return) pair of minimum and maximum correct shooting angles
 * for shot.id
 *
 * @return If the shot hit the intended asteroid
 */
bool check_shot(const Shot shot, std::set<size_t>& destroyed_ids,
                std::ifstream& file, size_t& hit_id,
                std::pair<double, double>& shot_lim) {
  std::string line;
  if (std::getline(file, line, '\n')) {
    std::stringstream ss_line(line);
    std::string token;

    // Get time of measurement
    std::getline(ss_line, token, ',');  // Remove time

    // Get valid regions
    while (std::getline(ss_line, token, ',')) {
      double min_shot;
      double max_shot;
      size_t id = std::stoi(token);

      if (!std::getline(ss_line, token, ','))
        throw std::runtime_error(
            "Verify data file did not match the expected dimension");
      min_shot = std::stod(token);
      if (!std::getline(ss_line, token, ','))
        throw std::runtime_error(
            "Verify data file did not match the expected dimension");
      max_shot = std::stod(token);

      double firing_angle = wrapToPi(shot.firing_angle);  // Keep it [-pi, pi]
      if (id == shot.id) {
        shot_lim = std::make_pair(min_shot, max_shot);
        for (int offset = -1; offset < 2; ++offset) {  // Check both directions
                                                       // to match
                                                       // min_shot/max_shot
          double s = firing_angle + 2 * M_PI * (double)offset;
          if (s >= min_shot && s <= max_shot) {
            hit_id = id;
            destroyed_ids.insert(id);
            return true;
          }
        }
        return false;
      }
    }
  } else {
    throw std::runtime_error(
        "Sensor file and verify file had different lengths!");
  }
  return false;
}

/** @brief Read in groundtruth information for debugging only
 *
 * @param file Groundtruth file
 * @param gt (return) See "types.hpp"
 */
void read_groundtruth(std::ifstream& file, Groundtruth& gt) {
  std::string line;
  if (std::getline(file, line, '\n')) {
    std::stringstream ss_line(line);
    std::string token;

    // Get time of measurement
    std::getline(ss_line, token, ',');
    gt.time_s = std::stod(token);

    // Get ship
    for (size_t ii = 0; ii < 5; ++ii) {
      if (!std::getline(ss_line, token, ','))
        throw std::runtime_error(
            "Groundtruth data file did not match the expected dimension");
      gt.ship[ii] = std::stod(token);
    }

    // Get asteroids
    while (std::getline(ss_line, token, ',')) {
      size_t id = std::stoi(token);
      if (id != gt.asteroids.size())
        throw std::runtime_error("Asteroids read out of order in GT");

      std::array<double, 4> asteroid;
      for (size_t ii = 0; ii < 4; ++ii) {
        if (!std::getline(ss_line, token, ','))
          throw std::runtime_error(
              "Groundtruth data file did not match the expected dimension");
        asteroid[ii] = std::stod(token);
      }
      gt.asteroids.push_back(asteroid);
    }
  }
}

#endif
