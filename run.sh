cd build
make
cd ..
rm out/*
rm out.mp4

export RUN=0

./build/bin/asteroids test_cases/measure_$RUN.csv test_cases/check_$RUN.csv test_cases/gt_$RUN.csv
# ./build/bin/asteroids test_cases/measure_$RUN.csv test_cases/check_$RUN.csv

ffmpeg -r 20 -i out/result_%05d.bmp -c:v libx264 -vf fps=25 -pix_fmt yuv420p out.mp4
#