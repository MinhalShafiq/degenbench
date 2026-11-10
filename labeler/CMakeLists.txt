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
  src/io.cpp
  src/localizability.cpp
  src/map.cpp
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

foreach(tool label_sequence register_pairs)
  add_executable(${tool} tools/${tool}.cpp)
  target_link_libraries(${tool} PRIVATE degenbench_labeler small_gicp::small_gicp OpenMP::OpenMP_CXX)
  target_compile_options(${tool} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()

option(DEGENBENCH_BUILD_TESTS "Build unit tests" ON)
if(DEGENBENCH_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)
  foreach(name test_analytic_scenes test_solver_agreement test_io_map)
    add_executable(${name} test/${name}.cpp)
    target_link_libraries(${name} PRIVATE degenbench_labeler GTest::gtest_main)
    gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
  endforeach()
  target_compile_definitions(test_io_map PRIVATE
    LABEL_SEQUENCE_TOOL="$<TARGET_FILE:label_sequence>")
  add_dependencies(test_io_map label_sequence)
endif()
