cmake_minimum_required(VERSION 3.16)
project(degenbench LANGUAGES CXX)

# Builds the labeler and the detector library together. The labeler can still
# be built on its own with `cmake -S labeler`.
enable_testing()
add_subdirectory(labeler)
add_subdirectory(detectors)
