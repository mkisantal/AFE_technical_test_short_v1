#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <array>

// Storage struct for new sensor data
struct SensorData {
  size_t id;     // Id of the asteroid the data is for
  double range;  // (m) [0, inf) Range to the asteroid
  int bearing;  // (pixel) [0, 359], with 0 corresponding to forward, increasing
                // counter clockwise

  // method for calculating position in ship-fixed inertial frame (x pointing the sun)
  std::array<double, 2> getXY(int sun_position){

    double alpha = (double)(bearing - sun_position) * M_PI / 180.0;
    std::array<double, 2>  xy;
    xy[0] = range * cos(alpha);
    xy[1] = range * sin(alpha);
    return xy;
  }

};

// Storage struct for a shot
struct Shot {
  Shot(const size_t _id, const double bullet_speed, const std::array<double, 4>& relative_position_velocity)
      : id(_id) {
    double phi = atan2(relative_position_velocity[1], relative_position_velocity[0]);
    double vy_rot = -sin(phi) * relative_position_velocity[2] + cos(phi) * relative_position_velocity[3];
    firing_angle = asin(vy_rot / bullet_speed) + phi;
  };

  size_t id;            // Id shot at
  double firing_angle;  // Angle to shoot at, 0 corresponding to directly in
                        // front of the ship, increasing counter clockwise
};

// Storage struct for groundtruth information for plotting
struct Groundtruth {
  double time_s;   // (s) Time of the groundtruth sample
  double ship[5];  // State of this spaceship in some world fixed frame (x, y,
                   // vx, vy, theta), (m, m, m/s, m/s, rad)
  std::vector<std::array<double, 4>> asteroids;  // Vector of states of all
                                                 // asteroids, where location in
                                                 // the vector corresponds to id
                                                 // of the asteroid. State of
                                                 // each asteroid is (x, y, vx,
                                                 // vy), (m, m, m/s, m/s)
};
#endif
