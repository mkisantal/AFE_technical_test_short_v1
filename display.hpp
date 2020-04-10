#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include "bmp.hpp"
#include "file_io.hpp"
#include "types.hpp"


static void display_results(std::ifstream& groundtruth_file,
                     const std::vector<std::array<double, 4>>& asteroid_posvel,
                            Shot& shot, const std::pair<double, double>& shot_lim, const bool hit, const size_t iterations) {

  BMP image(800, 800);
  Groundtruth gt;
  read_groundtruth(groundtruth_file, gt);
  // Draw ship
  image.draw_ship(gt.ship[0], gt.ship[1], gt.ship[4]);
  double velocity_factor = 4;
  // Draw asteroids
  for (size_t ii = 0; ii < gt.asteroids.size(); ++ii) {
    image.draw_circle(gt.asteroids[ii][0], gt.asteroids[ii][1], 3, Color(ii));
    // Draw velocity vector
    image.draw_line(gt.asteroids[ii][0], gt.asteroids[ii][1],
                    gt.asteroids[ii][0] + velocity_factor * gt.asteroids[ii][2],
                    gt.asteroids[ii][1] + velocity_factor * gt.asteroids[ii][3], Color(ii), 1);
  }

  // Draw estimated asteroids
  for (size_t ii = 0; ii < asteroid_posvel.size(); ++ii) {
    std::array<double, 4> ast_posvel = asteroid_posvel[ii];
    double x = gt.ship[0] + cos(gt.ship[4]) * ast_posvel[0] -
               sin(gt.ship[4]) * ast_posvel[1];
    double y = gt.ship[1] + sin(gt.ship[4]) * ast_posvel[0] +
               cos(gt.ship[4]) * ast_posvel[1];
    image.draw_circle(x, y, 4, Color(ii), 2);

    // Draw velocity
    double vx = gt.ship[2] + cos(gt.ship[4]) * ast_posvel[2] -
               sin(gt.ship[4]) * ast_posvel[3];
    double vy = gt.ship[3] + sin(gt.ship[4]) * ast_posvel[2] +
               cos(gt.ship[4]) * ast_posvel[3];
    image.draw_line(x, y, x + velocity_factor * vx, y + velocity_factor * vy, Color(ii), 1, true);
  }

  // Draw correct shot
  image.draw_ray(gt.ship[0], gt.ship[1], shot_lim.first + gt.ship[4],
                 Color(shot.id));
  image.draw_ray(gt.ship[0], gt.ship[1], shot_lim.second + gt.ship[4],
                 Color(shot.id));
  // Draw chosen shot
  image.draw_ray(gt.ship[0], gt.ship[1], shot.firing_angle + gt.ship[4],
                 Color(shot.id), 1, true);
  /* PRINT */
  std::string image_name = "result_" + std::to_string(iterations) + "_";
  if (hit) {
    image_name += "hit";
  } else {
    image_name += "miss";
  }
  image_name += ".bmp";
  image.write(image_name.c_str());
}

#endif
