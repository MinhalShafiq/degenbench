cmake_minimum_required(VERSION 3.16)
project(degenbench_labeler LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(small_gicp REQUIRED)

add_library(degenbench_labeler
  src/localizability.cpp
  src/scenes.cpp
)
target_include_directories(degenbench_labeler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(degenbench_labeler
  PUBLIC Eigen3::Eigen
  PRIVATE small_gicp::small_gicp OpenMP::OpenMP_CXX
)
target_compile_options(degenbench_labeler PRIVATE -Wall -Wextra -Wpedantic)

option(DEGENBENCH_BUILD_TESTS "Build unit tests" ON)
if(DEGENBENCH_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(test_analytic_scenes test/test_analytic_scenes.cpp)
  target_link_libraries(test_analytic_scenes PRIVATE degenbench_labeler GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(test_analytic_scenes DISCOVERY_TIMEOUT 60)
endif()
