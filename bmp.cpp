#include "bmp.hpp"
#include "wrap_to_pi.hpp"

BMP::BMP(int32_t width, int32_t height, bool has_alpha) {
  if (width <= 0 || height <= 0) {
    throw std::runtime_error(
        "The image width and height must be positive numbers.");
  }

  bmp_info_header.width = width;
  bmp_info_header.height = height;
  if (has_alpha) {
    bmp_info_header.size = sizeof(BMPInfoHeader) + sizeof(BMPColorHeader);
    file_header.offset_data =
        sizeof(BMPFileHeader) + sizeof(BMPInfoHeader) + sizeof(BMPColorHeader);

    bmp_info_header.bit_count = 32;
    bmp_info_header.compression = 3;
    row_stride = width * 4;
    data.resize(row_stride * height);
    file_header.file_size = file_header.offset_data + data.size();
  } else {
    bmp_info_header.size = sizeof(BMPInfoHeader);
    file_header.offset_data = sizeof(BMPFileHeader) + sizeof(BMPInfoHeader);

    bmp_info_header.bit_count = 24;
    bmp_info_header.compression = 0;
    row_stride = width * 3;
    data.resize(row_stride * height);
    std::fill(data.begin(), data.end(), 255);

    uint32_t new_stride = make_stride_aligned(4);
    file_header.file_size = file_header.offset_data +
                            static_cast<uint32_t>(data.size()) +
                            bmp_info_header.height * (new_stride - row_stride);
  }
}

void BMP::write(const char* fname) {
  std::ofstream of(fname, std::ofstream::out);
  if (of) {
    if (bmp_info_header.bit_count == 32) {
      write_headers_and_data(of);
    } else if (bmp_info_header.bit_count == 24) {
      if (bmp_info_header.width % 4 == 0) {
        write_headers_and_data(of);
      } else {
        uint32_t new_stride = make_stride_aligned(4);
        std::vector<uint8_t> padding_row(new_stride - row_stride);

        write_headers(of);

        for (int y = 0; y < bmp_info_header.height; ++y) {
          of.write((const char*)(data.data() + row_stride * y), row_stride);
          of.write((const char*)padding_row.data(), padding_row.size());
        }
      }
    } else {
      throw std::runtime_error(
          "The program can treat only 24 or 32 bits per pixel BMP files");
    }
  } else {
    throw std::runtime_error("Unable to open the output image file.");
  }
}

void BMP::fill_region(uint32_t x0, uint32_t y0, uint32_t w, uint32_t h,
                      Color col) {
  if (x0 + w > (uint32_t)bmp_info_header.width ||
      y0 + h > (uint32_t)bmp_info_header.height) {
    throw std::runtime_error("The region does not fit in the image!");
  }

  uint32_t channels = bmp_info_header.bit_count / 8;
  for (uint32_t y = y0; y < y0 + h; ++y) {
    for (uint32_t x = x0; x < x0 + w; ++x) {
      data[channels * (y * bmp_info_header.width + x) + 0] = col.B;
      data[channels * (y * bmp_info_header.width + x) + 1] = col.G;
      data[channels * (y * bmp_info_header.width + x) + 2] = col.R;
      if (channels == 4) {
        data[channels * (y * bmp_info_header.width + x) + 3] = col.A;
      }
    }
  }
}

bool BMP::set_pixel(double x0, double y0, Color col) {
  if (x0 <= 0 || y0 <= 0) {
    return false;
  }
  return set_pixel((uint32_t)round(x0), (uint32_t)round(y0), col);
}

bool BMP::set_pixel(int x0, int y0, Color col) {
  if (x0 <= 0 || y0 <= 0) {
    return false;
  }
  return set_pixel((uint32_t)round(x0), (uint32_t)round(y0), col);
}

bool BMP::set_pixel(uint32_t x0, uint32_t y0, Color col) {
  if (x0 >= (uint32_t)bmp_info_header.width ||
      y0 >= (uint32_t)bmp_info_header.height) {
    // Pixel out of image, just ignore it
    return false;
  }

  uint32_t channels = bmp_info_header.bit_count / 8;
  data[channels * (y0 * bmp_info_header.width + x0) + 0] = col.B;
  data[channels * (y0 * bmp_info_header.width + x0) + 1] = col.G;
  data[channels * (y0 * bmp_info_header.width + x0) + 2] = col.R;
  if (channels == 4) {
    data[channels * (y0 * bmp_info_header.width + x0) + 3] = col.A;
  }
  return true;
}

void BMP::draw_circle(double x0, double y0, double r, Color col, double r_min) {
  double x_min = 0;
  double y_min = 0;
  if (x0 > r) {
    x_min = x0 - r;
  }
  if (y0 > r) {
    y_min = y0 - r;
  }

  double r_sq = r * r;
  double r_min_sq = r_min * r_min;

  for (double x = x_min; x <= (x0 + r); ++x) {
    for (double y = y_min; y <= (y0 + r); ++y) {
      double dist = (x - x0) * (x - x0) + (y - y0) * (y - y0);
      if (dist <= r_sq && dist >= r_min_sq) {
        set_pixel(x, y, col);
      }
    }
  }
}

void BMP::draw_ray(double x0, double y0, double angle, Color col,
                   uint8_t line_width, bool dashed, uint32_t cycle_length) {
  double max_length =
      (double)std::max(bmp_info_header.width, bmp_info_header.height);
  double xf = x0 + max_length * cos(angle);
  double yf = y0 + max_length * sin(angle);
  if (xf < 0) {
    xf = 0;
    double dx = (xf - x0);
    yf = y0 + dx * tan(angle);
  }
  if (yf < 0) {
    yf = 0;
    if (fabs(tan(angle)) > 1e-5) {
      double dy = (yf - y0);
      xf = x0 + dy / tan(angle);
    }
  }
  // Clip again just incase the above went slightly over
  xf = std::max(0., xf);
  yf = std::max(0., yf);
  draw_line(x0, y0, xf, yf, col, line_width, dashed, cycle_length);
}

void BMP::draw_line(double x1, double y1, double x2, double y2, Color col,
                    uint8_t line_width, bool dashed, uint32_t cycle_length) {
  int half_line_width = (int)std::floor(0.5 * (double)line_width);

  // Bresenham's line algorithm
  const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
  if (steep) {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }

  if (x1 > x2) {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }

  if (x1 < 0) {
    if (x2 < 0)
      return;
    double dx_orig = x2 - x1;
    double dx_new = x2;
    x1 = 0;
    y1 = y2 - (y2 - y1) * dx_new / dx_orig;
  }
  if (y1 < 0) {
    if (y2 < 0)
      return;
    double dy_orig = y2 - y1;
    double dy_new = y2;
    y1 = 0;
    x1 = x2 - (x2 - x1) * dy_new / dy_orig;
  }
  x1 = std::max(0., x1);  // Still zero incase of rounding
  y1 = std::max(0., y1);  // Still zero incase of rounding

  const float dx = x2 - x1;
  const float dy = fabs(y2 - y1);

  float error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 1 : -1;
  int y = y1;

  const int maxX = x2;

  const float cycle_dx_f = dx / sqrt(dx * dx + dy * dy) * (float)cycle_length;
  const float on_dx_f = (dashed) ? (cycle_dx_f / 2) : cycle_dx_f;
  const int cycle_dx = (int)cycle_dx_f;
  const int on_dx = (int)on_dx_f;

  for (int x = (int)x1; x <= maxX; x++) {
    if ((x % cycle_dx) < on_dx) {  // plot
      for (int offset = -half_line_width; offset <= half_line_width; ++offset) {
        if (steep) {
          if (!set_pixel(y + offset, x, col)) {
            break;
          }
        } else {
          if (!set_pixel(x, y + offset, col)) {
            break;
          }
        }
      }
    }

    error -= dy;
    if (error < 0) {
      y += ystep;
      error += dx;
    }
  }
}

void BMP::draw_ship(double x0, double y0, double heading, double width,
                    double length, Color col, uint8_t line_width) {
  double ch = cos(heading);
  double sh = sin(heading);

  double ul_x = (-.3 * length) * ch + (.5 * width) * (-sh) + x0;
  double ul_y = (-.3 * length) * sh + (.5 * width) * (ch) + y0;
  double ll_x = (-.3 * length) * ch + (-.5 * width) * (-sh) + x0;
  double ll_y = (-.3 * length) * sh + (-.5 * width) * (ch) + y0;
  double fr_x = (.7 * length) * ch + x0;
  double fr_y = (.7 * length) * sh + y0;

  draw_line(ul_x, ul_y, fr_x, fr_y, col, line_width);
  draw_line(ul_x, ul_y, ll_x, ll_y, col, line_width);
  draw_line(ll_x, ll_y, fr_x, fr_y, col, line_width);
}

void BMP::write_headers(std::ofstream& of) {
  of.write((const char*)&file_header, sizeof(file_header));
  of.write((const char*)&bmp_info_header, sizeof(bmp_info_header));
  if (bmp_info_header.bit_count == 32) {
    of.write((const char*)&bmp_color_header, sizeof(bmp_color_header));
  }
}

void BMP::write_headers_and_data(std::ofstream& of) {
  write_headers(of);
  of.write((const char*)data.data(), data.size());
}

// Add 1 to the row_stride until it is divisible with align_stride
uint32_t BMP::make_stride_aligned(uint32_t align_stride) {
  uint32_t new_stride = row_stride;
  while (new_stride % align_stride != 0) {
    new_stride++;
  }
  return new_stride;
}

// Check if the pixel data is stored as BGRA and if the color space type is
// sRGB
void BMP::check_color_header(BMPColorHeader& bmp_color_header) {
  BMPColorHeader expected_color_header;
  if (expected_color_header.red_mask != bmp_color_header.red_mask ||
      expected_color_header.blue_mask != bmp_color_header.blue_mask ||
      expected_color_header.green_mask != bmp_color_header.green_mask ||
      expected_color_header.alpha_mask != bmp_color_header.alpha_mask) {
    throw std::runtime_error(
        "Unexpected color mask format! The program expects the pixel data to "
        "be in the BGRA format");
  }
  if (expected_color_header.color_space_type !=
      bmp_color_header.color_space_type) {
    throw std::runtime_error(
        "Unexpected color space type! The program expects sRGB values");
  }
}
