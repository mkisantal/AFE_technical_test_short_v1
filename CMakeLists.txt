cmake_minimum_required(VERSION 3.10.0)

project(technical-test)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-function"
)

# /usr/local/Cellar/eigen/3.3.7
# # Eigen 
find_package(Eigen3 REQUIRED)
add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE ${EIGEN3_INCLUDE_DIRS})
# target_include_directories(eigen INTERFACE /usr/local/Cellar/eigen/3.3.7)



add_executable(asteroids main.cpp bmp.cpp)
target_link_libraries(asteroids eigen)
