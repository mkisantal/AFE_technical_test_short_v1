#ifndef BMP_HPP
#define BMP_HPP
#include <fstream>
#include <vector>
#include <stdexcept>
#include <iostream>

struct Color {
  /*
 - 0: Red
 - 1: Green
 - 2: Blue
 - 3: Fuchsia
 - 4: Aqua
 - 5: Yellow
 - 6: Maroon
 - 7: Dark green
 - 8: Navy
 - 9: Teal
 - 10: Purple
 - else: Black
  */
  Color(int id) {
    switch (id) {
      case 0:
        R = 255;
        return;
      case 1:
        G = 255;
        return;
      case 2:
        B = 255;
        return;
      case 3:
        R = 255;
        B = 255;
        return;
      case 4:
        G = 255;
        B = 255;
        return;
      case 5:
        R = 255;
        G = 255;
        return;
      case 6:
        R = 127;
        return;
      case 7:
        G = 127;
        return;
      case 8:
        B = 127;
        return;
      case 9:
        G = 127;
        B = 127;
        return;
      case 10:
        R = 127;
        B = 127;
        return;
    }
  }
  Color(uint8_t b, uint8_t g, uint8_t r) : B(b), G(g), R(r){};
  Color() : Color(-1) {
  }

  uint8_t B = 0;
  uint8_t G = 0;
  uint8_t R = 0;
  uint8_t A = 0;
};

#pragma pack(push, 1)
struct BMPFileHeader {
  uint16_t file_type{0x4D42};  // File type always BM which is 0x4D42 (stored as
                               // hex uint16_t in little endian)
  uint32_t file_size{0};       // Size of the file (in bytes)
  uint16_t reserved1{0};       // Reserved, always 0
  uint16_t reserved2{0};       // Reserved, always 0
  uint32_t offset_data{0};     // Start position of pixel data (bytes from the
                               // beginning of the file)
};

struct BMPInfoHeader {
  uint32_t size{0};    // Size of this header (in bytes)
  int32_t width{0};    // width of bitmap in pixels
  int32_t height{0};   // width of bitmap in pixels
                       //       (if positive, bottom-up, with origin in lower
                       //       left corner) (if negative, top-down, with origin
                       //       in upper left corner)
  uint16_t planes{1};  // No. of planes for the target device, this is always 1
  uint16_t bit_count{0};    // No. of bits per pixel
  uint32_t compression{0};  // 0 or 3 - uncompressed. THIS PROGRAM CONSIDERS
                            // ONLY UNCOMPRESSED BMP images
  uint32_t size_image{0};   // 0 - for uncompressed images
  int32_t x_pixels_per_meter{0};
  int32_t y_pixels_per_meter{0};
  uint32_t colors_used{0};  // No. color indexes in the color table. Use 0 for
                            // the max number of colors allowed by bit_count
  uint32_t colors_important{0};  // No. of colors used for displaying the
                                 // bitmap. If 0 all colors are required
};

struct BMPColorHeader {
  uint32_t red_mask{0x00ff0000};          // Bit mask for the red channel
  uint32_t green_mask{0x0000ff00};        // Bit mask for the green channel
  uint32_t blue_mask{0x000000ff};         // Bit mask for the blue channel
  uint32_t alpha_mask{0xff000000};        // Bit mask for the alpha channel
  uint32_t color_space_type{0x73524742};  // Default "sRGB" (0x73524742)
  uint32_t unused[16]{0};                 // Unused data for sRGB color space
};
#pragma pack(pop)

struct BMP {
  BMPFileHeader file_header;
  BMPInfoHeader bmp_info_header;
  BMPColorHeader bmp_color_header;
  std::vector<uint8_t> data;

  BMP() : BMP(1000, 1000) {
  }
  BMP(int32_t width, int32_t height, bool has_alpha = false);

  void write(const char* fname);

  void fill_region(uint32_t x0, uint32_t y0, uint32_t w, uint32_t h, Color col);

  bool set_pixel(double x0, double y0, Color col);
  bool set_pixel(int x0, int y0, Color col);
  bool set_pixel(uint32_t x0, uint32_t y0, Color col);

  /* DRAW FUNCTIONS */
  // All values in pixels, where (0,0) is the bottom left of the image. Angles
  // are referenced where 0 is pointing at the right side of the image, and pi/2
  // is pointing at the top of the image. See Color() for coloring of pixels.
  // All functions call set_pixel which protects against writing outside the
  // image
  void draw_circle(double x0, double y0, double r, Color col, double r_min = 0);
  void draw_ray(double x0, double y0, double angle, Color col,
                uint8_t line_width = 1, bool dashed = false,
                uint32_t cycle_length = 10);
  void draw_line(double x0, double y0, double xf, double yf, Color col,
                 uint8_t line_width = 1, bool dashed = false,
                 uint32_t cycle_length = 10);

  // Draws ship as a triangle
  void draw_ship(double x0, double y0, double heading, double width = 4,
                 double length = 8, Color col = Color(),
                 uint8_t line_width = 1);
  /* END DRAW FUNCTIONS */

 private:
  uint32_t row_stride{0};

  void write_headers(std::ofstream& of);

  void write_headers_and_data(std::ofstream& of);

  // Add 1 to the row_stride until it is divisible with align_stride
  uint32_t make_stride_aligned(uint32_t align_stride);

  // Check if the pixel data is stored as BGRA and if the color space type is
  // sRGB
  void check_color_header(BMPColorHeader& bmp_color_header);
};

#endif
