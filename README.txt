Find enclosed main.cpp, which gives a framework to load test cases and check whether your shot hit the intended asteroid. Please edit with code to initialize estimates of asteroids in the world. The full framework is present to run the full task, but feel free to ignore unless you want to present your full solution in C++/.

# Building

A CMakeLists.txt file is included with the code for your convenience, but you may use your preferred build system as long as instructions are provided.

# Libraries

Please make use utility libraries (such as Eigen) as you need/will make your task easier, but implement the core of your solution yourself and make sure to document your dependencies!

# Inputs

main.cpp has inputs:
(1) measure_X.csv
(2) check_X.csv
(3) (optional) gt_X.csv (triggers plotting)

We have included in ./test_cases 3 sets of data with {measure_X.csv, check_X.csv, gt_X.csv} to aid you with debugging, and 3 with only {measure_X.csv, check_X.csv} to check your solution. {measure_0.csv, check_0.csv, gt_0.csv} is an easy scenario where no control inputs are given, so the spaceship is moving with constant velocity and heading.

Have fun!
