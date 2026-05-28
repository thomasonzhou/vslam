## Visual SLAM

This repo is based on the code introduced in the SLAM book: https://github.com/gaoxiang12/slambook2/tree/master



https://github.com/user-attachments/assets/741a252c-8132-44f8-828c-573cb4db1c2d



Gray points are initial guesses, while black points are optimized. An extension of this could be to visualize only the most recent optimization step.

## Quickstart
Ensure bazel is installed, then run:
```sh
bazel run //:vslam -- $PWD/sphere.g2o
```

## To-do
- profiling helper to call any function and evaluate time taken
- trajectory evaluation metrics
- camera calibration methods
